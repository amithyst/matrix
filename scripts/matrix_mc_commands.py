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

from matrix_movement_modes import validate_movement_mode
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
_POSE_YAW_RE = re.compile(r"/?(?:pose|rot)\s+@s\s+yaw\s+(?P<angle>\S+)\s*\Z")
_RECOVER_RE = re.compile(r"/?(?:recover|tpstand)(?:\s+@s)?\s*\Z")
_MODE_RE = re.compile(r"/?mode\s+(?P<mode>[A-Za-z0-9_+-]+)\s*\Z")
_SONIC_MODE_RE = re.compile(
    r"/?(?:sonic|native)\s+mode\s+(?P<mode>auto|[0-9]{1,2})\s*\Z"
)
_FUNCTION_RE = re.compile(r"/?function\s+(?P<body>.+?)\s*\Z")
_SELECTOR_RE = re.compile(r"@e\[(?P<body>[^\]]+)\]\Z")


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
class Angle:
    value_rad: float
    relative: bool = False

    def __post_init__(self) -> None:
        if isinstance(self.value_rad, bool) or not isinstance(
            self.value_rad, (int, float)
        ):
            raise CommandParseError("E_ANGLE_INVALID", "angle must be numeric")
        value = float(self.value_rad)
        if not math.isfinite(value):
            raise CommandParseError("E_ANGLE_NONFINITE", "angle must be finite")
        if type(self.relative) is not bool:
            raise CommandParseError("E_ANGLE_INVALID", "relative flag must be boolean")
        object.__setattr__(self, "value_rad", value)

    def resolve(self, origin_rad: float) -> float:
        result = self.value_rad + float(origin_rad) if self.relative else self.value_rad
        if not math.isfinite(result):
            raise CommandExecutionError(
                "E_ANGLE_NONFINITE", "resolved angle is not finite"
            )
        return math.atan2(math.sin(result), math.cos(result))

    def to_mapping(self) -> dict[str, object]:
        return {"relative": self.relative, "value_rad": self.value_rad}

    @classmethod
    def from_mapping(cls, value: object) -> "Angle":
        if not isinstance(value, dict) or set(value) != {"relative", "value_rad"}:
            raise CommandProtocolError("angle has an invalid schema")
        try:
            return cls(value_rad=value.get("value_rad"), relative=value.get("relative"))
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
class PoseYawSet:
    angle: Angle

    def __post_init__(self) -> None:
        if not isinstance(self.angle, Angle):
            raise CommandParseError("E_ANGLE_INVALID", "pose yaw requires an angle")


@dataclass(frozen=True)
class RecoverHere:
    """Reload at the current XY using the last known safe upright pose."""


@dataclass(frozen=True)
class MovementModeSet:
    movement_mode: str

    def __post_init__(self) -> None:
        try:
            mode = validate_movement_mode(self.movement_mode)
        except ValueError as exc:
            raise CommandParseError("E_MOVEMENT_MODE", str(exc)) from exc
        object.__setattr__(self, "movement_mode", mode)


@dataclass(frozen=True)
class NativeModeSet:
    native_mode: int | None

    def __post_init__(self) -> None:
        mode = self.native_mode
        if mode is not None and (
            type(mode) is not int or not 0 <= mode <= 19
        ):
            raise CommandParseError(
                "E_NATIVE_MODE", "native SONIC mode must be auto or an integer in [0, 19]"
            )


@dataclass(frozen=True)
class CommandFunctionRun:
    commands: tuple["AtomicMcCommand", ...]

    def __post_init__(self) -> None:
        if (
            not isinstance(self.commands, tuple)
            or not self.commands
            or len(self.commands) > 8
            or any(not _is_atomic_command(command) for command in self.commands)
        ):
            raise CommandParseError(
                "E_FUNCTION_INVALID",
                "function requires 1-8 non-nested Matrix commands",
            )


AtomicMcCommand: TypeAlias = (
    SummonTeleportPoint
    | TeleportCoordinates
    | TeleportSelector
    | PoseYawSet
    | RecoverHere
    | MovementModeSet
    | NativeModeSet
)
McCommand: TypeAlias = (
    AtomicMcCommand | CommandFunctionRun
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


def parse_angle(token: str) -> Angle:
    relative = token.startswith("~")
    body = token[1:] if relative else token
    if not body:
        raise CommandParseError(
            "E_ANGLE_INVALID", "yaw angle requires a value and unit, e.g. 90deg"
        )
    unit = None
    for suffix in ("deg", "rad"):
        if body.endswith(suffix):
            unit = suffix
            body = body[: -len(suffix)]
            break
    if unit is None:
        raise CommandParseError("E_ANGLE_UNIT", "yaw angle must end in deg or rad")
    if _NUMBER_RE.fullmatch(body) is None:
        raise CommandParseError("E_ANGLE_INVALID", f"invalid angle {token!r}")
    try:
        value = float(body)
    except ValueError as exc:  # pragma: no cover - guarded by the regex.
        raise CommandParseError("E_ANGLE_INVALID", f"invalid angle {token!r}") from exc
    if unit == "deg":
        value = math.radians(value)
    return Angle(value, relative=relative)


def _is_atomic_command(command: object) -> bool:
    return isinstance(
        command,
        (
            SummonTeleportPoint,
            TeleportCoordinates,
            TeleportSelector,
            PoseYawSet,
            RecoverHere,
            MovementModeSet,
            NativeModeSet,
        ),
    )


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


def _parse_function_body(body: str) -> CommandFunctionRun:
    named = body.strip()
    presets = {
        "recover_here": (RecoverHere(),),
        "tpstand": (RecoverHere(),),
        "sonic_auto": (NativeModeSet(None),),
    }
    if named in presets:
        return CommandFunctionRun(presets[named])
    parts = tuple(part.strip() for part in body.split(";") if part.strip())
    if not parts:
        raise CommandParseError("E_FUNCTION_EMPTY", "function command is empty")
    if len(parts) > 8:
        raise CommandParseError("E_FUNCTION_TOO_LONG", "function supports at most 8 steps")
    commands: list[AtomicMcCommand] = []
    for part in parts:
        parsed = _parse_mc_command(part, allow_function=False).command
        if not _is_atomic_command(parsed):
            raise CommandParseError("E_FUNCTION_NESTED", "nested functions are not supported")
        commands.append(parsed)
    return CommandFunctionRun(tuple(commands))


def _parse_mc_command(text: object, *, allow_function: bool) -> ParsedCommand:
    command_text = _validate_text(text)
    function = _FUNCTION_RE.fullmatch(command_text)
    if function is not None:
        if not allow_function:
            raise CommandParseError("E_FUNCTION_NESTED", "nested functions are not supported")
        return ParsedCommand(_parse_function_body(function.group("body")))
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

    pose_yaw = _POSE_YAW_RE.fullmatch(command_text)
    if pose_yaw is not None:
        return ParsedCommand(PoseYawSet(parse_angle(pose_yaw.group("angle"))))

    if _RECOVER_RE.fullmatch(command_text) is not None:
        return ParsedCommand(RecoverHere())

    mode = _MODE_RE.fullmatch(command_text)
    if mode is not None:
        return ParsedCommand(MovementModeSet(mode.group("mode")))

    sonic_mode = _SONIC_MODE_RE.fullmatch(command_text)
    if sonic_mode is not None:
        value = sonic_mode.group("mode")
        return ParsedCommand(
            NativeModeSet(None if value == "auto" else int(value, 10))
        )

    first = command_text.lstrip("/").split(maxsplit=1)[0]
    if first in {"sumon", "summonn", "summom"}:
        raise CommandParseError(
            "E_COMMAND_UNKNOWN", f"unknown command {first!r}; did you mean /summon?"
        )
    raise CommandParseError(
        "E_COMMAND_UNKNOWN",
        "supported commands are /summon, /tp, /pose, /recover, /mode, /sonic mode, and /function",
    )


def parse_mc_command(text: object) -> ParsedCommand:
    return _parse_mc_command(text, allow_function=True)


def command_to_mapping(command: McCommand) -> dict[str, object]:
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
    if isinstance(command, PoseYawSet):
        return {"name": "pose_yaw_set", "angle": command.angle.to_mapping()}
    if isinstance(command, RecoverHere):
        return {"name": "recover_here"}
    if isinstance(command, MovementModeSet):
        return {
            "name": "movement_mode_set",
            "movement_mode": command.movement_mode,
        }
    if isinstance(command, NativeModeSet):
        return {
            "name": "native_mode_set",
            "native_mode": command.native_mode,
        }
    if isinstance(command, CommandFunctionRun):
        return {
            "name": "function_run",
            "commands": [command_to_mapping(item) for item in command.commands],
        }
    raise TypeError(f"unsupported command AST: {type(command).__name__}")


def command_from_mapping(value: object) -> McCommand:
    if not isinstance(value, dict) or not isinstance(value.get("name"), str):
        raise CommandProtocolError("command AST has an invalid schema")
    name = value["name"]
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
    if name == "movement_mode_set":
        if set(value) != {"name", "movement_mode"}:
            raise CommandProtocolError("movement mode command has an invalid schema")
        try:
            return MovementModeSet(value.get("movement_mode"))
        except CommandParseError as exc:
            raise CommandProtocolError(str(exc)) from exc
    if name == "native_mode_set":
        if set(value) != {"name", "native_mode"}:
            raise CommandProtocolError("native mode command has an invalid schema")
        try:
            return NativeModeSet(value.get("native_mode"))
        except CommandParseError as exc:
            raise CommandProtocolError(str(exc)) from exc
    if name == "pose_yaw_set":
        if set(value) != {"name", "angle"}:
            raise CommandProtocolError("pose yaw command has an invalid schema")
        try:
            return PoseYawSet(Angle.from_mapping(value.get("angle")))
        except CommandParseError as exc:
            raise CommandProtocolError(str(exc)) from exc
    if name == "recover_here":
        if set(value) != {"name"}:
            raise CommandProtocolError("recover command has an invalid schema")
        return RecoverHere()
    if name == "function_run":
        if set(value) != {"name", "commands"}:
            raise CommandProtocolError("function command has an invalid schema")
        commands = value.get("commands")
        if not isinstance(commands, list) or not 1 <= len(commands) <= 8:
            raise CommandProtocolError("function command requires 1-8 steps")
        parsed_commands = tuple(command_from_mapping(item) for item in commands)
        if any(not _is_atomic_command(item) for item in parsed_commands):
            raise CommandProtocolError("function command cannot contain nested functions")
        try:
            return CommandFunctionRun(parsed_commands)
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
    if isinstance(command, PoseYawSet):
        yaw_rad = command.angle.resolve(current_pose.yaw_rad)
        pose = WorldPose(current_pose.x, current_pose.y, current_pose.z, yaw_rad)
        next_state = state.set_resume_pose(
            pose, source="pose_command", now_unix_ns=now_unix_ns
        )
        return CommandEffect(
            state=next_state,
            code="OK_POSE_RESTART",
            message="Pose saved; reloading Matrix with the requested yaw",
            restart_required=True,
            data={
                "position": [pose.x, pose.y, pose.z],
                "yaw_rad": pose.yaw_rad,
            },
        )
    if isinstance(command, RecoverHere):
        if state.last_safe is None:
            raise CommandExecutionError(
                "E_RECOVER_NO_SAFE_POSE",
                "No upright checkpoint is available for recover-here",
            )
        pose = WorldPose(
            current_pose.x,
            current_pose.y,
            state.last_safe.z,
            state.last_safe.yaw_rad,
        )
        next_state = state.set_resume_pose(
            pose, source="recover_here", now_unix_ns=now_unix_ns
        )
        return CommandEffect(
            state=next_state,
            code="OK_RECOVER_RESTART",
            message="Recover pose saved; reloading Matrix upright at current XY",
            restart_required=True,
            data={
                "position": [pose.x, pose.y, pose.z],
                "yaw_rad": pose.yaw_rad,
                "source": "last_safe",
            },
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
    if isinstance(command, CommandFunctionRun):
        next_state = state
        working_pose = current_pose
        restart_required = False
        results: list[dict[str, object]] = []
        for item in command.commands:
            if isinstance(item, (MovementModeSet, NativeModeSet)):
                raise CommandExecutionError(
                    "E_FUNCTION_RUNTIME_COMMAND",
                    "movement/native mode function steps require runtime support",
                )
            effect = execute_command(
                item,
                state=next_state,
                current_pose=working_pose,
                now_unix_ns=now_unix_ns,
            )
            next_state = effect.state
            if effect.state.last_exit is not None:
                working_pose = effect.state.last_exit
            restart_required = restart_required or effect.restart_required
            results.append(
                {
                    "code": effect.code,
                    "message": effect.message,
                    "restart_required": effect.restart_required,
                    "data": dict(effect.data),
                }
            )
        return CommandEffect(
            state=next_state,
            code="OK_FUNCTION_RESTART" if restart_required else "OK_FUNCTION",
            message=f"Function executed {len(results)} step(s)",
            restart_required=restart_required,
            data={"steps": results},
        )
    raise TypeError(f"unsupported command AST: {type(command).__name__}")


__all__ = [
    "COMMAND_PROTOCOL",
    "Angle",
    "CommandEffect",
    "CommandExecutionError",
    "CommandParseError",
    "CommandProtocolError",
    "Coordinate",
    "GameCommandRequest",
    "GameCommandResponse",
    "MovementModeSet",
    "NativeModeSet",
    "ParsedCommand",
    "PoseYawSet",
    "RecoverHere",
    "SummonTeleportPoint",
    "TeleportCoordinates",
    "TeleportSelector",
    "CommandFunctionRun",
    "command_from_mapping",
    "command_to_mapping",
    "decode_command_request",
    "decode_command_response",
    "encode_command_request",
    "encode_command_response",
    "execute_command",
    "parse_mc_command",
]
