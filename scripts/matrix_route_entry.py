#!/usr/bin/env python3
"""Strict one-shot parser for catalog-backed Matrix route entries."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import re
import sys
from typing import Sequence

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from matrix_world_state import (  # noqa: E402
    TeleportPoint,
    WorldPose,
    WorldStateError,
    validate_tag,
    validate_world_id,
)


ROUTE_SCHEMA = "matrix-celestial-launch-route/v1"
OUTPUT_SCHEMA = "matrix-route-entry/v1"
ROUTE_FIELDS = frozenset(
    {
        "schema",
        "destination_id",
        "teleport_tag",
        "target_scene_id",
        "target_world_id",
        "entry_pose",
        "entity_id",
    }
)
OUTPUT_FIELDS = frozenset(
    {
        "schema",
        "destination_id",
        "teleport_tag",
        "target_scene_id",
        "target_world_id",
        "entity_id",
        "entry_x",
        "entry_y",
        "entry_z",
        "entry_yaw_rad",
    }
)
_DESTINATION_ID_RE = re.compile(r"[a-z0-9][a-z0-9._-]{0,63}\Z")


class RouteEntryError(ValueError):
    """Raised when a one-shot route entry is malformed or mismatched."""


@dataclass(frozen=True)
class RouteEntry:
    destination_id: str
    teleport_tag: str
    target_scene_id: int
    target_world_id: str
    entry_pose: WorldPose
    entity_id: str

    def output_mapping(self) -> dict[str, object]:
        return {
            "schema": OUTPUT_SCHEMA,
            "destination_id": self.destination_id,
            "teleport_tag": self.teleport_tag,
            "target_scene_id": self.target_scene_id,
            "target_world_id": self.target_world_id,
            "entity_id": self.entity_id,
            "entry_x": self.entry_pose.x,
            "entry_y": self.entry_pose.y,
            "entry_z": self.entry_pose.z,
            "entry_yaw_rad": self.entry_pose.yaw_rad,
        }


def _strict_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise RouteEntryError(f"duplicate JSON field {key!r}")
        result[key] = value
    return result


def _reject_json_constant(token: str) -> object:
    raise RouteEntryError(f"invalid JSON constant {token}")


def _decode_route_json(text: str) -> object:
    try:
        text.encode("ascii")
    except UnicodeEncodeError as exc:
        raise RouteEntryError("route JSON must be ASCII") from exc
    try:
        return json.loads(
            text,
            object_pairs_hook=_strict_object,
            parse_constant=_reject_json_constant,
        )
    except json.JSONDecodeError as exc:
        raise RouteEntryError(f"invalid route JSON: {exc}") from exc


def _validate_destination_id(value: object) -> str:
    if not isinstance(value, str) or _DESTINATION_ID_RE.fullmatch(value) is None:
        raise RouteEntryError(
            "destination_id must be 1-64 lowercase safe ASCII characters"
        )
    return value


def _validate_scene_id(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 99:
        raise RouteEntryError("target_scene_id must be an integer in [0, 99]")
    return value


def _world_state_error(label: str, exc: WorldStateError) -> RouteEntryError:
    return RouteEntryError(f"{label}: {exc}")


def parse_route_entry(
    value: object,
    *,
    expected_world_id: str,
    expected_scene_id: int,
) -> RouteEntry:
    try:
        expected_world_id = validate_world_id(expected_world_id)
    except WorldStateError as exc:
        raise _world_state_error("expected_world_id", exc) from exc
    expected_scene_id = _validate_scene_id(expected_scene_id)

    if not isinstance(value, dict) or frozenset(value) != ROUTE_FIELDS:
        raise RouteEntryError("route entry has an invalid schema")
    if value.get("schema") != ROUTE_SCHEMA:
        raise RouteEntryError("route entry schema is unsupported")

    destination_id = _validate_destination_id(value.get("destination_id"))
    try:
        teleport_tag = validate_tag(value.get("teleport_tag"))
        target_world_id = validate_world_id(value.get("target_world_id"))
        entry_pose = WorldPose.from_mapping(
            value.get("entry_pose"),
            label="entry_pose",
        )
    except WorldStateError as exc:
        raise _world_state_error("route entry", exc) from exc
    target_scene_id = _validate_scene_id(value.get("target_scene_id"))
    if target_world_id != expected_world_id:
        raise RouteEntryError(
            "route target world mismatch: "
            f"{target_world_id!r} != {expected_world_id!r}"
        )
    if target_scene_id != expected_scene_id:
        raise RouteEntryError(
            "route target scene mismatch: "
            f"{target_scene_id!r} != {expected_scene_id!r}"
        )

    entity_id = value.get("entity_id")
    try:
        # TeleportPoint is the public world-state validator that binds entity,
        # tag, pose, and timestamp contracts without any runtime dependencies.
        point = TeleportPoint(
            entity_id=entity_id,
            pose=entry_pose,
            tags=(teleport_tag,),
            created_at_unix_ns=0,
        )
    except WorldStateError as exc:
        raise _world_state_error("route entry", exc) from exc

    return RouteEntry(
        destination_id=destination_id,
        teleport_tag=teleport_tag,
        target_scene_id=target_scene_id,
        target_world_id=target_world_id,
        entry_pose=entry_pose,
        entity_id=point.entity_id,
    )


def parse_route_entry_text(
    text: str,
    *,
    expected_world_id: str,
    expected_scene_id: int,
) -> RouteEntry:
    return parse_route_entry(
        _decode_route_json(text),
        expected_world_id=expected_world_id,
        expected_scene_id=expected_scene_id,
    )


def parse_route_entry_output(
    value: object,
    *,
    expected_world_id: str,
    expected_scene_id: int,
) -> RouteEntry:
    """Revalidate the canonical one-shot payload at the run_sim boundary."""

    if not isinstance(value, dict) or frozenset(value) != OUTPUT_FIELDS:
        raise RouteEntryError("canonical route entry has an invalid schema")
    if value.get("schema") != OUTPUT_SCHEMA:
        raise RouteEntryError("canonical route entry schema is unsupported")
    return parse_route_entry(
        {
            "schema": ROUTE_SCHEMA,
            "destination_id": value.get("destination_id"),
            "teleport_tag": value.get("teleport_tag"),
            "target_scene_id": value.get("target_scene_id"),
            "target_world_id": value.get("target_world_id"),
            "entity_id": value.get("entity_id"),
            "entry_pose": {
                "position": [
                    value.get("entry_x"),
                    value.get("entry_y"),
                    value.get("entry_z"),
                ],
                "yaw_rad": value.get("entry_yaw_rad"),
            },
        },
        expected_world_id=expected_world_id,
        expected_scene_id=expected_scene_id,
    )


def parse_route_entry_output_text(
    text: str,
    *,
    expected_world_id: str,
    expected_scene_id: int,
) -> RouteEntry:
    return parse_route_entry_output(
        _decode_route_json(text),
        expected_world_id=expected_world_id,
        expected_scene_id=expected_scene_id,
    )


def encode_route_entry_line(entry: RouteEntry) -> str:
    return json.dumps(
        entry.output_mapping(),
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    )


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Validate one matrix-celestial-launch-route/v1 JSON payload and "
            "emit one matrix-route-entry/v1 JSON line."
        )
    )
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--json", dest="route_json", help="route JSON payload")
    source.add_argument("--file", type=Path, help="route JSON file; use '-' for stdin")
    parser.add_argument("--expected-world-id", required=True)
    parser.add_argument("--expected-scene-id", required=True, type=int)
    return parser.parse_args(argv)


def _read_route_text(args: argparse.Namespace) -> str:
    if args.route_json is not None:
        return args.route_json
    if args.file is None or str(args.file) == "-":
        return sys.stdin.read()
    try:
        return args.file.read_text(encoding="ascii")
    except (OSError, UnicodeError) as exc:
        raise RouteEntryError(f"cannot read route JSON: {exc}") from exc


def main(argv: Sequence[str] | None = None) -> int:
    try:
        args = _parse_args(argv)
        entry = parse_route_entry_text(
            _read_route_text(args),
            expected_world_id=args.expected_world_id,
            expected_scene_id=args.expected_scene_id,
        )
    except RouteEntryError as exc:
        print(f"matrix-route-entry ERROR: {exc}", file=sys.stderr)
        return 2
    print(encode_route_entry_line(entry))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
