#!/usr/bin/env python3
"""Strict Minecraft-style commands for Matrix teleport-point gameplay.

The text parser runs in the supervised input provider.  Only the typed command
AST is sent to the physics runtime; command text is never passed to a shell,
Unreal ``ExecCmds``, ``eval``, or ``subprocess``.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import re
from typing import Any, Mapping, TypeAlias

from matrix_world_state import (
    MatrixWorldState,
    TELEPORT_POINT_TYPE,
    WorldPose,
    WorldStateError,
    validate_tag,
)


COMMAND_PROTOCOL = "matrix-game-command/v1"
MAX_COMMAND_CHARS = 512
MAX_COMMAND_PACKET_BYTES = 4096
_SESSION_RE = re.compile(r"[0-9a-f]{32}\Z")
_REQUEST_ID_RE = re.compile(r"cmd-[0-9a-f]{32}\Z")
_ERROR_CODE_RE = re.compile(r"[A-Z][A-Z0-9_]{1,63}\Z")
_NUMBER_RE = re.compile(
    r"[+-]?(?:(?:[0-9]+(?:\.[0-9]*)?)|(?:\.[0-9]+))(?:[eE][+-]?[0-9]+)?\Z"
)
_SUMMON_RE = re.compile(
    r"/?(?P<name>summon|summom)\s+"
    r"(?P<entity>[A-Za-z0-9_.:+-]+)\s+"
    r"(?P<x>\S+)\s+(?P<y>\S+)\s+(?P<z>\S+)\s+"
    r"\{Tags:\[(?P<tags>.*)\]\}\s*\Z"
)
_TP_RE = re.compile(r"/?tp\s+@s\s+(?P<target>.+?)\s*\Z")
_POLICY_RE = re.compile(
    r"/?policy\s+(?P<slot>locomotion|recovery)\s+"
    r"(?P<policy>[a-z0-9][a-z0-9._-]{0,63})\s*\Z",
    re.IGNORECASE,
)
_ITEM_RE = re.compile(
    r"/?item\s+spawn\s+(?P<item>[a-z0-9][a-z0-9_-]{0,47})\s*\Z",
    re.IGNORECASE,
)
_SELECTOR_RE = re.compile(r"@e\[(?P<body>[^\]]+)\]\Z")
_POLICY_ID_RE = re.compile(r"[a-z0-9][a-z0-9._-]{0,63}\Z")


class CommandParseError(ValueError):
    def __init__(self, code: str, message: str, *, column: int | None = None) -> None:
        super().__init__(message)
        self.code = code
        self.message = message
        self.column = column


class CommandProtocolError(ValueError):
    """Raised for malformed command-channel packets."""


class CommandExecutionError(RuntimeError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


@dataclass(frozen=True)
class Coordinate:
    value: float
    relative: bool = False

    def __post_init__(self) -> None:
        if isinstance(self.value, bool) or not isinstance(self.value, (int, float)):
            raise CommandParseError("E_COORD_INVALID", "coordinate must be numeric")
        value = float(self.value)
        if not math.isfinite(value):
            raise CommandParseError("E_COORD_NONFINITE", "coordinate must be finite")
        if type(self.relative) is not bool:
            raise CommandParseError("E_COORD_INVALID", "relative flag must be boolean")
        object.__setattr__(self, "value", value)

    def resolve(self, origin: float) -> float:
        result = self.value + float(origin) if self.relative else self.value
        if not math.isfinite(result):
            raise CommandExecutionError(
                "E_COORD_NONFINITE", "resolved coordinate is not finite"
            )
        return result

    def to_mapping(self) -> dict[str, object]:
        return {"relative": self.relative, "value": self.value}

    @classmethod
    def from_mapping(cls, value: object, *, index: int) -> "Coordinate":
        if not isinstance(value, dict) or set(value) != {"relative", "value"}:
            raise CommandProtocolError(f"coordinates[{index}] has an invalid schema")
        try:
            return cls(value=value.get("value"), relative=value.get("relative"))
        except CommandParseError as exc:
            raise CommandProtocolError(str(exc)) from exc


@dataclass(frozen=True)
class SummonTeleportPoint:
    coordinates: tuple[Coordinate, Coordinate, Coordinate]
    tags: tuple[str, ...]

    def __post_init__(self) -> None:
        if (
            not isinstance(self.coordinates, tuple)
            or len(self.coordinates) != 3
            or any(not isinstance(value, Coordinate) for value in self.coordinates)
        ):
            raise CommandParseError(
                "E_COORD_ARITY", "summon requires exactly three coordinates"
            )
        if not isinstance(self.tags, tuple) or not self.tags:
            raise CommandParseError(
                "E_TAG_REQUIRED", "teleport point requires at least one tag"
            )
        try:
            validated = tuple(validate_tag(tag) for tag in self.tags)
        except WorldStateError as exc:
            raise CommandParseError("E_TAG_INVALID", str(exc)) from exc
        if len(set(validated)) != len(validated):
            raise CommandParseError("E_TAG_DUPLICATE", "Tags must be unique")
        object.__setattr__(self, "tags", validated)


@dataclass(frozen=True)
class TeleportCoordinates:
    coordinates: tuple[Coordinate, Coordinate, Coordinate]

    def __post_init__(self) -> None:
        if (
            not isinstance(self.coordinates, tuple)
            or len(self.coordinates) != 3
            or any(not isinstance(value, Coordinate) for value in self.coordinates)
        ):
            raise CommandParseError(
                "E_COORD_ARITY", "tp requires exactly three coordinates"
            )


@dataclass(frozen=True)
class TeleportSelector:
    tag: str
    limit: int = 1
    sort: str = "nearest"

    def __post_init__(self) -> None:
        try:
            tag = validate_tag(self.tag)
        except WorldStateError as exc:
            raise CommandParseError("E_TAG_INVALID", str(exc)) from exc
        if isinstance(self.limit, bool) or not isinstance(self.limit, int) or self.limit != 1:
            raise CommandParseError(
                "E_SELECTOR_LIMIT", "teleport selector requires limit=1"
            )
        if self.sort != "nearest":
            raise CommandParseError(
                "E_SELECTOR_SORT", "teleport selector supports only sort=nearest"
            )
        object.__setattr__(self, "tag", tag)


@dataclass(frozen=True)
class PolicySlotAssignment:
    """Select one already-resident policy for a gameplay strategy slot."""

    slot: str
    policy_id: str

    def __post_init__(self) -> None:
        slot = str(self.slot).strip().lower()
        policy_id = str(self.policy_id).strip().lower()
        if slot not in {"locomotion", "recovery"}:
            raise CommandParseError(
                "E_POLICY_SLOT", "policy slot must be locomotion or recovery"
            )
        if _POLICY_ID_RE.fullmatch(policy_id) is None:
            raise CommandParseError("E_POLICY_ID", "policy id is invalid")
        object.__setattr__(self, "slot", slot)
        object.__setattr__(self, "policy_id", policy_id)


@dataclass(frozen=True)
class CreativeSpawnItem:
    """Take one standalone physical prop from the creative inventory pool."""

    item_id: str

    def __post_init__(self) -> None:
        item_id = str(self.item_id).strip().lower()
        if re.fullmatch(r"[a-z0-9][a-z0-9_-]{0,47}", item_id) is None:
            raise CommandParseError("E_INVENTORY_ITEM", "creative item id is invalid")
        object.__setattr__(self, "item_id", item_id)


McCommand: TypeAlias = (
    SummonTeleportPoint
    | TeleportCoordinates
    | TeleportSelector
    | PolicySlotAssignment
    | CreativeSpawnItem
)


@dataclass(frozen=True)
class ParsedCommand:
    command: McCommand
    warning: str | None = None


def _validate_text(text: object) -> str:
    if not isinstance(text, str):
        raise CommandParseError("E_COMMAND_TYPE", "command must be text")
    if not text or not text.strip():
        raise CommandParseError("E_COMMAND_EMPTY", "command is empty")
    if len(text) > MAX_COMMAND_CHARS:
        raise CommandParseError(
            "E_COMMAND_TOO_LONG",
            f"command exceeds {MAX_COMMAND_CHARS} characters",
        )
    if any(ord(character) < 0x20 or ord(character) == 0x7F for character in text):
        raise CommandParseError(
            "E_COMMAND_CONTROL", "command contains a control character"
        )
    return text.strip()


def parse_coordinate(token: str) -> Coordinate:
    if token.startswith("^"):
        raise CommandParseError(
            "E_LOCAL_COORD_UNSUPPORTED", "local ^ coordinates are not supported"
        )
    relative = token.startswith("~")
    number = token[1:] if relative else token
    if relative and number == "":
        return Coordinate(0.0, relative=True)
    if _NUMBER_RE.fullmatch(number) is None:
        raise CommandParseError(
            "E_COORD_INVALID", f"invalid coordinate {token!r}"
        )
    try:
        value = float(number)
    except ValueError as exc:  # pragma: no cover - guarded by the regex.
        raise CommandParseError(
            "E_COORD_INVALID", f"invalid coordinate {token!r}"
        ) from exc
    return Coordinate(value, relative=relative)


def _parse_tags(body: str) -> tuple[str, ...]:
    try:
        value = json.loads(
            f"[{body}]",
            parse_constant=lambda token: (_ for _ in ()).throw(
                ValueError(f"invalid constant {token}")
            ),
        )
    except (json.JSONDecodeError, ValueError) as exc:
        raise CommandParseError(
            "E_NBT_TAGS", 'Tags must use JSON-style strings, e.g. {Tags:["home"]}'
        ) from exc
    if not isinstance(value, list) or not value:
        raise CommandParseError("E_TAG_REQUIRED", "Tags must contain at least one tag")
    if any(not isinstance(tag, str) for tag in value):
        raise CommandParseError("E_TAG_INVALID", "every Tags entry must be a string")
    return tuple(value)


def _parse_selector(text: str) -> TeleportSelector:
    match = _SELECTOR_RE.fullmatch(text)
    if match is None:
        raise CommandParseError("E_SELECTOR_SYNTAX", "invalid entity selector")
    entries: dict[str, str] = {}
    for raw_entry in match.group("body").split(","):
        if "=" not in raw_entry:
            raise CommandParseError(
                "E_SELECTOR_SYNTAX", f"invalid selector entry {raw_entry!r}"
            )
        key, value = (part.strip() for part in raw_entry.split("=", 1))
        if not key or not value:
            raise CommandParseError(
                "E_SELECTOR_SYNTAX", f"invalid selector entry {raw_entry!r}"
            )
        if key in entries:
            raise CommandParseError(
                "E_SELECTOR_DUPLICATE", f"duplicate selector key {key!r}"
            )
        entries[key] = value
    allowed = {"type", "tag", "limit", "sort"}
    unknown = set(entries) - allowed
    if unknown:
        raise CommandParseError(
            "E_SELECTOR_KEY", f"unsupported selector key {sorted(unknown)[0]!r}"
        )
    if entries.get("type") != TELEPORT_POINT_TYPE:
        raise CommandParseError(
            "E_SELECTOR_TYPE",
            f"selector type must be {TELEPORT_POINT_TYPE}",
        )
    if "tag" not in entries:
        raise CommandParseError("E_SELECTOR_TAG", "selector requires tag=...")
    if entries.get("limit") != "1":
        raise CommandParseError(
            "E_SELECTOR_LIMIT", "teleport selector requires limit=1"
        )
    sort = entries.get("sort", "nearest")
    return TeleportSelector(tag=entries["tag"], limit=1, sort=sort)


def parse_mc_command(text: object) -> ParsedCommand:
    command_text = _validate_text(text)
    item = _ITEM_RE.fullmatch(command_text)
    if item is not None:
        return ParsedCommand(CreativeSpawnItem(item.group("item")))
    policy = _POLICY_RE.fullmatch(command_text)
    if policy is not None:
        return ParsedCommand(
            PolicySlotAssignment(
                slot=policy.group("slot"),
                policy_id=policy.group("policy"),
            )
        )
    summon = _SUMMON_RE.fullmatch(command_text)
    if summon is not None:
        if summon.group("entity") != TELEPORT_POINT_TYPE:
            raise CommandParseError(
                "E_ENTITY_TYPE",
                f"summon supports only {TELEPORT_POINT_TYPE}",
            )
        coordinates = tuple(
            parse_coordinate(summon.group(axis)) for axis in ("x", "y", "z")
        )
        tags = _parse_tags(summon.group("tags"))
        warning = (
            "已兼容执行；标准命令是 /summon"
            if summon.group("name") == "summom"
            else None
        )
        return ParsedCommand(
            SummonTeleportPoint(coordinates=coordinates, tags=tags),
            warning=warning,
        )

    teleport = _TP_RE.fullmatch(command_text)
    if teleport is not None:
        target = teleport.group("target")
        if target.startswith("@e"):
            return ParsedCommand(_parse_selector(target))
        tokens = target.split()
        if len(tokens) != 3:
            raise CommandParseError(
                "E_COORD_ARITY", "tp @s requires three coordinates or one selector"
            )
        return ParsedCommand(
            TeleportCoordinates(tuple(parse_coordinate(token) for token in tokens))
        )

    first = command_text.lstrip("/").split(maxsplit=1)[0]
    if first in {"sumon", "summonn", "summom"}:
        raise CommandParseError(
            "E_COMMAND_UNKNOWN", f"unknown command {first!r}; did you mean /summon?"
        )
    raise CommandParseError(
        "E_COMMAND_UNKNOWN", "supported commands are /summon, /tp, /policy, and /item spawn"
    )


def command_to_mapping(command: McCommand) -> dict[str, object]:
    if isinstance(command, CreativeSpawnItem):
        return {"name": "creative_spawn_item", "item_id": command.item_id}
    if isinstance(command, PolicySlotAssignment):
        return {
            "name": "policy_slot_assignment",
            "slot": command.slot,
            "policy_id": command.policy_id,
        }
    if isinstance(command, SummonTeleportPoint):
        return {
            "name": "summon_teleport_point",
            "coordinates": [coordinate.to_mapping() for coordinate in command.coordinates],
            "tags": list(command.tags),
        }
    if isinstance(command, TeleportCoordinates):
        return {
            "name": "teleport_coordinates",
            "coordinates": [coordinate.to_mapping() for coordinate in command.coordinates],
        }
    if isinstance(command, TeleportSelector):
        return {
            "name": "teleport_selector",
            "tag": command.tag,
            "limit": command.limit,
            "sort": command.sort,
            "type": TELEPORT_POINT_TYPE,
        }
    raise TypeError(f"unsupported command AST: {type(command).__name__}")


def command_from_mapping(value: object) -> McCommand:
    if not isinstance(value, dict) or not isinstance(value.get("name"), str):
        raise CommandProtocolError("command AST has an invalid schema")
    name = value["name"]
    if name == "creative_spawn_item":
        if set(value) != {"name", "item_id"}:
            raise CommandProtocolError("creative spawn item has an invalid schema")
        try:
            return CreativeSpawnItem(item_id=value.get("item_id"))
        except CommandParseError as exc:
            raise CommandProtocolError(str(exc)) from exc
    if name == "policy_slot_assignment":
        if set(value) != {"name", "slot", "policy_id"}:
            raise CommandProtocolError(
                "policy slot assignment has an invalid schema"
            )
        try:
            return PolicySlotAssignment(
                slot=value.get("slot"),
                policy_id=value.get("policy_id"),
            )
        except CommandParseError as exc:
            raise CommandProtocolError(str(exc)) from exc
    if name in {"summon_teleport_point", "teleport_coordinates"}:
        required = {"name", "coordinates"}
        if name == "summon_teleport_point":
            required.add("tags")
        if set(value) != required:
            raise CommandProtocolError(f"{name} command has an invalid schema")
        coordinates = value.get("coordinates")
        if not isinstance(coordinates, list) or len(coordinates) != 3:
            raise CommandProtocolError("command requires exactly three coordinates")
        parsed_coordinates = tuple(
            Coordinate.from_mapping(item, index=index)
            for index, item in enumerate(coordinates)
        )
        try:
            if name == "summon_teleport_point":
                tags = value.get("tags")
                if not isinstance(tags, list):
                    raise CommandProtocolError("summon tags must be a list")
                return SummonTeleportPoint(parsed_coordinates, tuple(tags))
            return TeleportCoordinates(parsed_coordinates)
        except CommandParseError as exc:
            raise CommandProtocolError(str(exc)) from exc
    if name == "teleport_selector":
        if set(value) != {"name", "type", "tag", "limit", "sort"}:
            raise CommandProtocolError("teleport selector has an invalid schema")
        if value.get("type") != TELEPORT_POINT_TYPE:
            raise CommandProtocolError("teleport selector has an invalid entity type")
        try:
            return TeleportSelector(
                tag=value.get("tag"),
                limit=value.get("limit"),
                sort=value.get("sort"),
            )
        except CommandParseError as exc:
            raise CommandProtocolError(str(exc)) from exc
    raise CommandProtocolError(f"unsupported typed command {name!r}")


@dataclass(frozen=True)
class GameCommandRequest:
    session: str
    sequence: int
    request_id: str
    command: McCommand

    def __post_init__(self) -> None:
        if not isinstance(self.session, str) or _SESSION_RE.fullmatch(self.session) is None:
            raise CommandProtocolError("request session is invalid")
        if (
            isinstance(self.sequence, bool)
            or not isinstance(self.sequence, int)
            or not 1 <= self.sequence < 2**63
        ):
            raise CommandProtocolError("request sequence is invalid")
        if not isinstance(self.request_id, str) or _REQUEST_ID_RE.fullmatch(
            self.request_id
        ) is None:
            raise CommandProtocolError("request_id is invalid")
        command_to_mapping(self.command)

    def to_mapping(self) -> dict[str, object]:
        return {
            "protocol": COMMAND_PROTOCOL,
            "kind": "request",
            "session": self.session,
            "sequence": self.sequence,
            "request_id": self.request_id,
            "command": command_to_mapping(self.command),
        }


@dataclass(frozen=True)
class GameCommandResponse:
    session: str
    sequence: int
    request_id: str
    ok: bool
    code: str
    message: str
    restart_required: bool = False
    data: Mapping[str, object] | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.session, str) or _SESSION_RE.fullmatch(self.session) is None:
            raise CommandProtocolError("response session is invalid")
        if (
            isinstance(self.sequence, bool)
            or not isinstance(self.sequence, int)
            or not 1 <= self.sequence < 2**63
        ):
            raise CommandProtocolError("response sequence is invalid")
        if not isinstance(self.request_id, str) or _REQUEST_ID_RE.fullmatch(
            self.request_id
        ) is None:
            raise CommandProtocolError("response request_id is invalid")
        if type(self.ok) is not bool or type(self.restart_required) is not bool:
            raise CommandProtocolError("response boolean fields are invalid")
        if self.restart_required and not self.ok:
            raise CommandProtocolError(
                "an unsuccessful response cannot request a runtime restart"
            )
        if not isinstance(self.code, str) or _ERROR_CODE_RE.fullmatch(self.code) is None:
            raise CommandProtocolError("response code is invalid")
        if (
            not isinstance(self.message, str)
            or not self.message
            or len(self.message) > 512
            or any(ord(character) < 0x20 for character in self.message)
        ):
            raise CommandProtocolError("response message is invalid")
        if self.data is not None and not isinstance(self.data, Mapping):
            raise CommandProtocolError("response data must be an object or null")
        try:
            json.dumps(self.data, allow_nan=False)
        except (TypeError, ValueError) as exc:
            raise CommandProtocolError("response data is not strict JSON") from exc

    def to_mapping(self) -> dict[str, object]:
        return {
            "protocol": COMMAND_PROTOCOL,
            "kind": "response",
            "session": self.session,
            "sequence": self.sequence,
            "request_id": self.request_id,
            "ok": self.ok,
            "code": self.code,
            "message": self.message,
            "restart_required": self.restart_required,
            "data": dict(self.data) if self.data is not None else None,
        }


def _strict_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise CommandProtocolError(f"duplicate JSON field {key!r}")
        result[key] = value
    return result


def _decode_json_packet(payload: object) -> dict[str, object]:
    if not isinstance(payload, bytes):
        raise CommandProtocolError("command packet must be bytes")
    if not payload or len(payload) > MAX_COMMAND_PACKET_BYTES:
        raise CommandProtocolError("command packet size is invalid")
    try:
        value = json.loads(
            payload.decode("utf-8"),
            object_pairs_hook=_strict_object,
            parse_constant=lambda token: (_ for _ in ()).throw(
                CommandProtocolError(f"invalid JSON constant {token}")
            ),
        )
    except UnicodeDecodeError as exc:
        raise CommandProtocolError("command packet is not UTF-8") from exc
    except json.JSONDecodeError as exc:
        raise CommandProtocolError(f"invalid command JSON: {exc}") from exc
    if not isinstance(value, dict):
        raise CommandProtocolError("command packet must be a JSON object")
    return value


def encode_command_request(request: GameCommandRequest) -> bytes:
    payload = json.dumps(
        request.to_mapping(), separators=(",", ":"), sort_keys=True, allow_nan=False
    ).encode("utf-8")
    if len(payload) > MAX_COMMAND_PACKET_BYTES:
        raise CommandProtocolError("encoded command request is oversized")
    return payload


def decode_command_request(payload: bytes) -> GameCommandRequest:
    value = _decode_json_packet(payload)
    if set(value) != {
        "protocol",
        "kind",
        "session",
        "sequence",
        "request_id",
        "command",
    }:
        raise CommandProtocolError("command request has an invalid schema")
    if value.get("protocol") != COMMAND_PROTOCOL or value.get("kind") != "request":
        raise CommandProtocolError("command request identity is invalid")
    return GameCommandRequest(
        session=value.get("session"),
        sequence=value.get("sequence"),
        request_id=value.get("request_id"),
        command=command_from_mapping(value.get("command")),
    )


def encode_command_response(response: GameCommandResponse) -> bytes:
    payload = json.dumps(
        response.to_mapping(), separators=(",", ":"), sort_keys=True, allow_nan=False
    ).encode("utf-8")
    if len(payload) > MAX_COMMAND_PACKET_BYTES:
        raise CommandProtocolError("encoded command response is oversized")
    return payload


def decode_command_response(payload: bytes) -> GameCommandResponse:
    value = _decode_json_packet(payload)
    if set(value) != {
        "protocol",
        "kind",
        "session",
        "sequence",
        "request_id",
        "ok",
        "code",
        "message",
        "restart_required",
        "data",
    }:
        raise CommandProtocolError("command response has an invalid schema")
    if value.get("protocol") != COMMAND_PROTOCOL or value.get("kind") != "response":
        raise CommandProtocolError("command response identity is invalid")
    return GameCommandResponse(
        session=value.get("session"),
        sequence=value.get("sequence"),
        request_id=value.get("request_id"),
        ok=value.get("ok"),
        code=value.get("code"),
        message=value.get("message"),
        restart_required=value.get("restart_required"),
        data=value.get("data"),
    )


@dataclass(frozen=True)
class CommandEffect:
    state: MatrixWorldState
    code: str
    message: str
    restart_required: bool
    data: Mapping[str, object]


def _resolve_pose(
    coordinates: tuple[Coordinate, Coordinate, Coordinate], origin: WorldPose
) -> WorldPose:
    try:
        return WorldPose(
            coordinates[0].resolve(origin.x),
            coordinates[1].resolve(origin.y),
            coordinates[2].resolve(origin.z),
            origin.yaw_rad,
        )
    except WorldStateError as exc:
        raise CommandExecutionError("E_OUT_OF_WORLD", str(exc)) from exc


def execute_command(
    command: McCommand,
    *,
    state: MatrixWorldState,
    current_pose: WorldPose,
    now_unix_ns: int | None = None,
) -> CommandEffect:
    if isinstance(command, SummonTeleportPoint):
        pose = _resolve_pose(command.coordinates, current_pose)
        try:
            next_state, point = state.add_teleport_point(
                pose, command.tags, now_unix_ns=now_unix_ns
            )
        except WorldStateError as exc:
            raise CommandExecutionError("E_POINT_CREATE", str(exc)) from exc
        return CommandEffect(
            state=next_state,
            code="OK_SUMMONED",
            message=f"Summoned {TELEPORT_POINT_TYPE} with tag {point.tags[0]}",
            restart_required=False,
            data={
                "entity_id": point.entity_id,
                "position": [pose.x, pose.y, pose.z],
                "tags": list(point.tags),
            },
        )
    if isinstance(command, TeleportCoordinates):
        pose = _resolve_pose(command.coordinates, current_pose)
        next_state = state.set_resume_pose(
            pose, source="teleport_command", now_unix_ns=now_unix_ns
        )
        return CommandEffect(
            state=next_state,
            code="OK_TELEPORT_RESTART",
            message="Teleport saved; reloading Matrix at the destination",
            restart_required=True,
            data={"position": [pose.x, pose.y, pose.z]},
        )
    if isinstance(command, TeleportSelector):
        try:
            matches = state.select_teleport_points(
                tag=command.tag,
                origin=current_pose,
                sort=command.sort,
                limit=command.limit,
            )
        except WorldStateError as exc:
            raise CommandExecutionError("E_SELECTOR_INVALID", str(exc)) from exc
        if not matches:
            raise CommandExecutionError(
                "E_SELECTOR_NO_TARGET",
                f"no {TELEPORT_POINT_TYPE} has tag {command.tag!r}",
            )
        point = matches[0]
        next_state = state.set_resume_pose(
            point.pose,
            source="teleport_command",
            now_unix_ns=now_unix_ns,
        )
        return CommandEffect(
            state=next_state,
            code="OK_TELEPORT_RESTART",
            message=f"Teleporting to {command.tag}; reloading Matrix",
            restart_required=True,
            data={
                "entity_id": point.entity_id,
                "position": [point.pose.x, point.pose.y, point.pose.z],
                "tags": list(point.tags),
            },
        )
    raise TypeError(f"unsupported command AST: {type(command).__name__}")


__all__ = [
    "COMMAND_PROTOCOL",
    "CommandEffect",
    "CommandExecutionError",
    "CommandParseError",
    "CommandProtocolError",
    "Coordinate",
    "CreativeSpawnItem",
    "GameCommandRequest",
    "GameCommandResponse",
    "ParsedCommand",
    "PolicySlotAssignment",
    "SummonTeleportPoint",
    "TeleportCoordinates",
    "TeleportSelector",
    "command_from_mapping",
    "command_to_mapping",
    "decode_command_request",
    "decode_command_response",
    "encode_command_request",
    "encode_command_response",
    "execute_command",
    "parse_mc_command",
]
