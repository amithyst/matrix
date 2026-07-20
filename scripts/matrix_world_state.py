#!/usr/bin/env python3
"""Durable, scene-bound Matrix game state and logical teleport points.

The store intentionally persists semantic poses rather than a complete MuJoCo
``mjData`` image.  A later runtime generation can combine one of these poses
with its canonical upright joint configuration, zero velocities, and a fresh
SONIC policy process.  Dynamic simulator state is never deserialized here.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
import hashlib
import json
import math
import os
from pathlib import Path
import re
import stat
import sys
import time
from typing import Iterable
import uuid

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from prepare_sonic_physics_model import (
    SCENE_TRANSFORM_NONE,
    SonicPhysicsModelError,
    TOWN10_OPEN_BOUNDARY_TRANSFORM,
    physics_revision_payload,
)


WORLD_STATE_SCHEMA = "matrix-world-state/v1"
WORLD_FRAME = "matrix_mj_world"
WORLD_UNITS = "m"
TELEPORT_POINT_TYPE = "matrix:teleport_point"
MAX_WORLD_ID_CHARS = 160
MAX_TAG_CHARS = 64
MAX_TAGS_PER_POINT = 16
MAX_TELEPORT_POINTS = 1024
MAX_HORIZONTAL_METRES = 100_000.0
MIN_VERTICAL_METRES = -1_000.0
MAX_VERTICAL_METRES = 10_000.0

_WORLD_ID_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.:/-]{0,159}\Z")
_TAG_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.:+-]{0,63}\Z")
_ENTITY_ID_RE = re.compile(r"tp-[0-9a-f]{32}\Z")


class WorldStateError(ValueError):
    """Raised when a state document or requested mutation is invalid."""


class _WorldStatePathMissing(WorldStateError):
    """Raised internally when a securely traversed state path is absent."""


def _finite_number(value: object, *, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise WorldStateError(f"{label} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise WorldStateError(f"{label} must be a finite number")
    return result


def validate_world_id(value: object) -> str:
    if not isinstance(value, str) or _WORLD_ID_RE.fullmatch(value) is None:
        raise WorldStateError(
            "world_id must be 1-160 safe ASCII characters"
        )
    return value


def validate_world_revision(value: object) -> str:
    if (
        not isinstance(value, str)
        or not value
        or len(value) > 256
        or any(ord(character) < 0x21 or ord(character) > 0x7E for character in value)
    ):
        raise WorldStateError("world_revision must be bounded printable ASCII")
    return value


def validate_tag(value: object) -> str:
    if not isinstance(value, str) or _TAG_RE.fullmatch(value) is None:
        raise WorldStateError(
            "teleport tag must be 1-64 safe ASCII characters"
        )
    return value


@dataclass(frozen=True)
class WorldPose:
    """A Matrix root pose in the right-handed X-forward/Y-left/Z-up frame."""

    x: float
    y: float
    z: float
    yaw_rad: float

    def __post_init__(self) -> None:
        values = {
            "x": _finite_number(self.x, label="pose.x"),
            "y": _finite_number(self.y, label="pose.y"),
            "z": _finite_number(self.z, label="pose.z"),
            "yaw_rad": _finite_number(self.yaw_rad, label="pose.yaw_rad"),
        }
        if abs(values["x"]) > MAX_HORIZONTAL_METRES:
            raise WorldStateError("pose.x is outside the supported world bound")
        if abs(values["y"]) > MAX_HORIZONTAL_METRES:
            raise WorldStateError("pose.y is outside the supported world bound")
        if not MIN_VERTICAL_METRES <= values["z"] <= MAX_VERTICAL_METRES:
            raise WorldStateError("pose.z is outside the supported world bound")
        normalized_yaw = values["yaw_rad"]
        if not -math.pi <= normalized_yaw <= math.pi:
            normalized_yaw = math.atan2(
                math.sin(normalized_yaw), math.cos(normalized_yaw)
            )
        for name in ("x", "y", "z"):
            object.__setattr__(self, name, values[name])
        object.__setattr__(self, "yaw_rad", normalized_yaw)

    def to_mapping(self) -> dict[str, object]:
        return {
            "position": [self.x, self.y, self.z],
            "yaw_rad": self.yaw_rad,
        }

    @classmethod
    def from_mapping(cls, value: object, *, label: str = "pose") -> "WorldPose":
        if not isinstance(value, dict) or set(value) != {"position", "yaw_rad"}:
            raise WorldStateError(f"{label} has an invalid schema")
        position = value.get("position")
        if not isinstance(position, list) or len(position) != 3:
            raise WorldStateError(f"{label}.position must contain three numbers")
        return cls(
            _finite_number(position[0], label=f"{label}.position[0]"),
            _finite_number(position[1], label=f"{label}.position[1]"),
            _finite_number(position[2], label=f"{label}.position[2]"),
            _finite_number(value.get("yaw_rad"), label=f"{label}.yaw_rad"),
        )

    def distance_xy(self, other: "WorldPose") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)


@dataclass(frozen=True)
class TeleportPoint:
    entity_id: str
    pose: WorldPose
    tags: tuple[str, ...]
    created_at_unix_ns: int

    def __post_init__(self) -> None:
        if not isinstance(self.entity_id, str) or _ENTITY_ID_RE.fullmatch(
            self.entity_id
        ) is None:
            raise WorldStateError("teleport point has an invalid entity_id")
        if not isinstance(self.pose, WorldPose):
            raise WorldStateError("teleport point pose is invalid")
        if not isinstance(self.tags, tuple) or not 1 <= len(self.tags) <= MAX_TAGS_PER_POINT:
            raise WorldStateError(
                f"teleport point must have 1-{MAX_TAGS_PER_POINT} tags"
            )
        validated = tuple(validate_tag(tag) for tag in self.tags)
        if len(set(validated)) != len(validated):
            raise WorldStateError("teleport point tags must be unique")
        if (
            isinstance(self.created_at_unix_ns, bool)
            or not isinstance(self.created_at_unix_ns, int)
            or self.created_at_unix_ns < 0
        ):
            raise WorldStateError("teleport point timestamp is invalid")
        object.__setattr__(self, "tags", validated)

    def to_mapping(self) -> dict[str, object]:
        return {
            "entity_id": self.entity_id,
            "type": TELEPORT_POINT_TYPE,
            "pose": self.pose.to_mapping(),
            "tags": list(self.tags),
            "created_at_unix_ns": self.created_at_unix_ns,
        }

    @classmethod
    def from_mapping(cls, value: object, *, index: int) -> "TeleportPoint":
        required = {
            "entity_id",
            "type",
            "pose",
            "tags",
            "created_at_unix_ns",
        }
        if not isinstance(value, dict) or set(value) != required:
            raise WorldStateError(f"teleport_points[{index}] has an invalid schema")
        if value.get("type") != TELEPORT_POINT_TYPE:
            raise WorldStateError(f"teleport_points[{index}] has an invalid type")
        tags = value.get("tags")
        if not isinstance(tags, list):
            raise WorldStateError(f"teleport_points[{index}].tags must be a list")
        return cls(
            entity_id=value.get("entity_id"),
            pose=WorldPose.from_mapping(
                value.get("pose"), label=f"teleport_points[{index}].pose"
            ),
            tags=tuple(tags),
            created_at_unix_ns=value.get("created_at_unix_ns"),
        )


@dataclass(frozen=True)
class MatrixWorldState:
    world_id: str
    world_revision: str
    last_observed: WorldPose | None = None
    last_safe: WorldPose | None = None
    last_exit: WorldPose | None = None
    home: WorldPose | None = None
    resume_source: str | None = None
    teleport_points: tuple[TeleportPoint, ...] = ()
    updated_at_unix_ns: int = 0

    def __post_init__(self) -> None:
        object.__setattr__(self, "world_id", validate_world_id(self.world_id))
        object.__setattr__(
            self, "world_revision", validate_world_revision(self.world_revision)
        )
        for name in ("last_observed", "last_safe", "last_exit", "home"):
            value = getattr(self, name)
            if value is not None and not isinstance(value, WorldPose):
                raise WorldStateError(f"{name} must be a WorldPose or null")
        if self.resume_source is not None and self.resume_source not in {
            "upright_checkpoint",
            "fallen_xy_last_safe_upright",
            "teleport_command",
            "home",
        }:
            raise WorldStateError("resume_source is invalid")
        if not isinstance(self.teleport_points, tuple) or len(
            self.teleport_points
        ) > MAX_TELEPORT_POINTS:
            raise WorldStateError(
                f"teleport_points exceeds the {MAX_TELEPORT_POINTS} point limit"
            )
        if any(not isinstance(point, TeleportPoint) for point in self.teleport_points):
            raise WorldStateError("teleport_points contains an invalid point")
        entity_ids = [point.entity_id for point in self.teleport_points]
        if len(entity_ids) != len(set(entity_ids)):
            raise WorldStateError("teleport point entity IDs must be unique")
        if (
            isinstance(self.updated_at_unix_ns, bool)
            or not isinstance(self.updated_at_unix_ns, int)
            or self.updated_at_unix_ns < 0
        ):
            raise WorldStateError("updated_at_unix_ns is invalid")

    @classmethod
    def empty(cls, *, world_id: str, world_revision: str) -> "MatrixWorldState":
        return cls(
            world_id=world_id,
            world_revision=world_revision,
            updated_at_unix_ns=time.time_ns(),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "schema": WORLD_STATE_SCHEMA,
            "world": {
                "id": self.world_id,
                "revision": self.world_revision,
                "frame": WORLD_FRAME,
                "units": WORLD_UNITS,
            },
            "last_observed": (
                self.last_observed.to_mapping()
                if self.last_observed is not None
                else None
            ),
            "last_safe": (
                self.last_safe.to_mapping() if self.last_safe is not None else None
            ),
            "last_exit": (
                self.last_exit.to_mapping() if self.last_exit is not None else None
            ),
            "home": self.home.to_mapping() if self.home is not None else None,
            "resume_source": self.resume_source,
            "teleport_points": [
                point.to_mapping() for point in self.teleport_points
            ],
            "updated_at_unix_ns": self.updated_at_unix_ns,
        }

    @classmethod
    def from_mapping(cls, value: object) -> "MatrixWorldState":
        required = {
            "schema",
            "world",
            "last_observed",
            "last_safe",
            "last_exit",
            "home",
            "resume_source",
            "teleport_points",
            "updated_at_unix_ns",
        }
        if not isinstance(value, dict) or set(value) != required:
            raise WorldStateError("world state has an invalid schema")
        if value.get("schema") != WORLD_STATE_SCHEMA:
            raise WorldStateError("world state schema version is unsupported")
        world = value.get("world")
        if not isinstance(world, dict) or set(world) != {
            "id",
            "revision",
            "frame",
            "units",
        }:
            raise WorldStateError("world identity has an invalid schema")
        if world.get("frame") != WORLD_FRAME or world.get("units") != WORLD_UNITS:
            raise WorldStateError("world coordinate contract is incompatible")

        def optional_pose(name: str) -> WorldPose | None:
            item = value.get(name)
            return None if item is None else WorldPose.from_mapping(item, label=name)

        points = value.get("teleport_points")
        if not isinstance(points, list):
            raise WorldStateError("teleport_points must be a list")
        return cls(
            world_id=world.get("id"),
            world_revision=world.get("revision"),
            last_observed=optional_pose("last_observed"),
            last_safe=optional_pose("last_safe"),
            last_exit=optional_pose("last_exit"),
            home=optional_pose("home"),
            resume_source=value.get("resume_source"),
            teleport_points=tuple(
                TeleportPoint.from_mapping(point, index=index)
                for index, point in enumerate(points)
            ),
            updated_at_unix_ns=value.get("updated_at_unix_ns"),
        )

    def checkpoint(
        self,
        pose: WorldPose,
        *,
        upright: bool,
        now_unix_ns: int | None = None,
    ) -> "MatrixWorldState":
        if not isinstance(pose, WorldPose) or type(upright) is not bool:
            raise WorldStateError("checkpoint requires a pose and boolean upright flag")
        timestamp = time.time_ns() if now_unix_ns is None else now_unix_ns
        if upright:
            return replace(
                self,
                last_observed=pose,
                last_safe=pose,
                last_exit=pose,
                resume_source="upright_checkpoint",
                updated_at_unix_ns=timestamp,
            )
        if self.last_safe is None:
            return replace(
                self,
                last_observed=pose,
                updated_at_unix_ns=timestamp,
            )
        upright_exit = WorldPose(
            pose.x,
            pose.y,
            self.last_safe.z,
            self.last_safe.yaw_rad,
        )
        return replace(
            self,
            last_observed=pose,
            last_exit=upright_exit,
            resume_source="fallen_xy_last_safe_upright",
            updated_at_unix_ns=timestamp,
        )

    def set_resume_pose(
        self,
        pose: WorldPose,
        *,
        source: str = "teleport_command",
        now_unix_ns: int | None = None,
    ) -> "MatrixWorldState":
        if source not in {"teleport_command", "home"}:
            raise WorldStateError("unsupported explicit resume source")
        timestamp = time.time_ns() if now_unix_ns is None else now_unix_ns
        return replace(
            self,
            last_observed=pose,
            last_safe=pose,
            last_exit=pose,
            resume_source=source,
            updated_at_unix_ns=timestamp,
        )

    def add_teleport_point(
        self,
        pose: WorldPose,
        tags: Iterable[str],
        *,
        entity_id: str | None = None,
        now_unix_ns: int | None = None,
    ) -> tuple["MatrixWorldState", TeleportPoint]:
        if len(self.teleport_points) >= MAX_TELEPORT_POINTS:
            raise WorldStateError("teleport point limit reached")
        normalized_tags = tuple(dict.fromkeys(validate_tag(tag) for tag in tags))
        if not normalized_tags:
            raise WorldStateError("teleport point requires at least one tag")
        timestamp = time.time_ns() if now_unix_ns is None else now_unix_ns
        point = TeleportPoint(
            entity_id=entity_id or f"tp-{uuid.uuid4().hex}",
            pose=pose,
            tags=normalized_tags,
            created_at_unix_ns=timestamp,
        )
        home = pose if "home" in normalized_tags else self.home
        return (
            replace(
                self,
                home=home,
                teleport_points=(*self.teleport_points, point),
                updated_at_unix_ns=timestamp,
            ),
            point,
        )

    def select_teleport_points(
        self,
        *,
        tag: str,
        origin: WorldPose,
        sort: str = "nearest",
        limit: int = 1,
    ) -> tuple[TeleportPoint, ...]:
        validated_tag = validate_tag(tag)
        if sort != "nearest":
            raise WorldStateError("only sort=nearest is supported")
        if isinstance(limit, bool) or not isinstance(limit, int) or limit != 1:
            raise WorldStateError("teleport selectors require limit=1")
        matches = [
            point for point in self.teleport_points if validated_tag in point.tags
        ]
        matches.sort(
            key=lambda point: (
                point.pose.distance_xy(origin),
                point.created_at_unix_ns,
                point.entity_id,
            )
        )
        return tuple(matches[:limit])

    def startup_pose(self, default: WorldPose) -> tuple[WorldPose, str]:
        if self.last_exit is not None:
            return self.last_exit, "last_exit"
        if self.home is not None:
            return self.home, "home"
        return default, "default"


def _strict_json_object(
    pairs: list[tuple[str, object]],
) -> dict[str, object]:
    value: dict[str, object] = {}
    for key, item in pairs:
        if key in value:
            raise WorldStateError(f"duplicate world-state JSON field {key!r}")
        value[key] = item
    return value


def _decode_state_bytes(payload: bytes) -> MatrixWorldState:
    if len(payload) > 4 * 1024 * 1024:
        raise WorldStateError("world state file exceeds 4 MiB")
    try:
        value = json.loads(
            payload.decode("utf-8"),
            object_pairs_hook=_strict_json_object,
            parse_constant=lambda token: (_ for _ in ()).throw(
                WorldStateError(
                    f"invalid world-state JSON constant {token!r}"
                )
            ),
        )
    except WorldStateError:
        raise
    except (
        UnicodeDecodeError,
        json.JSONDecodeError,
        RecursionError,
        ValueError,
    ) as exc:
        raise WorldStateError(f"invalid world state JSON: {exc}") from exc
    return MatrixWorldState.from_mapping(value)


def _state_path_components(path: Path) -> tuple[str, ...]:
    if not path.is_absolute() or path.anchor != os.sep:
        raise WorldStateError("world state path must be absolute")
    components = tuple(path.parts[1:])
    if not components:
        raise WorldStateError("world state path must name a file below /")
    if any(component in {"", ".", ".."} for component in components):
        raise WorldStateError("world state path contains an unsafe component")
    return components


def _directory_open_flags() -> int:
    nofollow = getattr(os, "O_NOFOLLOW", None)
    directory = getattr(os, "O_DIRECTORY", None)
    if nofollow is None or directory is None:
        raise WorldStateError(
            "secure world-state paths require O_DIRECTORY and O_NOFOLLOW support"
        )
    flags = os.O_RDONLY | directory | nofollow
    flags |= getattr(os, "O_CLOEXEC", 0)
    return flags


def _open_parent_directory(path: Path, *, create: bool) -> tuple[int, str]:
    components = _state_path_components(path)
    flags = _directory_open_flags()
    try:
        current_fd = os.open(os.sep, flags)
    except OSError as exc:
        raise WorldStateError(f"cannot open filesystem root: {exc}") from exc
    try:
        traversed: list[str] = []
        for component in components[:-1]:
            traversed.append(component)
            try:
                next_fd = os.open(component, flags, dir_fd=current_fd)
            except FileNotFoundError as exc:
                if not create:
                    raise _WorldStatePathMissing(
                        f"world state parent does not exist: {path}"
                    ) from exc
                try:
                    os.mkdir(component, mode=0o700, dir_fd=current_fd)
                except FileExistsError:
                    pass
                except OSError as mkdir_exc:
                    raise WorldStateError(
                        "cannot create world-state directory "
                        f"/{'/'.join(traversed)}: {mkdir_exc}"
                    ) from mkdir_exc
                else:
                    try:
                        os.fsync(current_fd)
                    except OSError as fsync_exc:
                        raise WorldStateError(
                            "cannot sync new world-state directory entry "
                            f"/{'/'.join(traversed)}: {fsync_exc}"
                        ) from fsync_exc
                try:
                    next_fd = os.open(component, flags, dir_fd=current_fd)
                except OSError as open_exc:
                    raise WorldStateError(
                        "cannot open newly created world-state directory "
                        f"/{'/'.join(traversed)}: {open_exc}"
                    ) from open_exc
            except OSError as exc:
                raise WorldStateError(
                    "refusing unsafe world-state directory "
                    f"/{'/'.join(traversed)}: {exc}"
                ) from exc
            previous_fd = current_fd
            current_fd = next_fd
            os.close(previous_fd)
        return current_fd, components[-1]
    except BaseException:
        os.close(current_fd)
        raise


def _read_regular_file(path: Path) -> bytes:
    nofollow = getattr(os, "O_NOFOLLOW", None)
    nonblock = getattr(os, "O_NONBLOCK", None)
    if nofollow is None or nonblock is None:
        raise WorldStateError(
            "secure world-state reads require O_NOFOLLOW and O_NONBLOCK support"
        )
    flags = os.O_RDONLY | nofollow | nonblock
    flags |= getattr(os, "O_CLOEXEC", 0)
    parent_fd, filename = _open_parent_directory(path, create=False)
    try:
        try:
            descriptor = os.open(filename, flags, dir_fd=parent_fd)
        except FileNotFoundError as exc:
            raise _WorldStatePathMissing(
                f"world state does not exist: {path}"
            ) from exc
        except OSError as exc:
            raise WorldStateError(f"cannot read world state {path}: {exc}") from exc
        try:
            metadata = os.fstat(descriptor)
            if not stat.S_ISREG(metadata.st_mode):
                raise WorldStateError(
                    f"world state is not a regular file: {path}"
                )
            remaining = 4 * 1024 * 1024 + 1
            chunks: list[bytes] = []
            while remaining:
                chunk = os.read(descriptor, min(1024 * 1024, remaining))
                if not chunk:
                    break
                chunks.append(chunk)
                remaining -= len(chunk)
            return b"".join(chunks)
        except OSError as exc:
            raise WorldStateError(f"cannot read world state {path}: {exc}") from exc
        finally:
            os.close(descriptor)
    finally:
        os.close(parent_fd)


def _atomic_write(path: Path, payload: bytes) -> None:
    nofollow = getattr(os, "O_NOFOLLOW", None)
    if nofollow is None:
        raise WorldStateError("secure world-state writes require O_NOFOLLOW support")
    parent_fd, filename = _open_parent_directory(path, create=True)
    temporary_name: str | None = None
    descriptor: int | None = None
    try:
        try:
            existing = os.stat(filename, dir_fd=parent_fd, follow_symlinks=False)
        except FileNotFoundError:
            existing = None
        if existing is not None:
            if stat.S_ISLNK(existing.st_mode):
                raise WorldStateError(f"refusing symlink world state: {path}")
            if not stat.S_ISREG(existing.st_mode):
                raise WorldStateError(
                    f"refusing non-regular world state: {path}"
                )

        temporary_flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | nofollow
        temporary_flags |= getattr(os, "O_CLOEXEC", 0)
        for _attempt in range(16):
            candidate_name = f".{filename}.{uuid.uuid4().hex}.tmp"
            try:
                descriptor = os.open(
                    candidate_name,
                    temporary_flags,
                    0o600,
                    dir_fd=parent_fd,
                )
            except FileExistsError:
                continue
            temporary_name = candidate_name
            break
        else:
            raise WorldStateError(
                f"cannot allocate temporary world state beside {path}"
            )

        if descriptor is None or temporary_name is None:
            raise WorldStateError(f"cannot allocate temporary world state beside {path}")
        os.fchmod(descriptor, 0o600)
        payload_view = memoryview(payload)
        offset = 0
        while offset < len(payload_view):
            written = os.write(descriptor, payload_view[offset:])
            if written <= 0:
                raise WorldStateError(f"short write while persisting {path}")
            offset += written
        os.fsync(descriptor)
        os.close(descriptor)
        descriptor = None
        os.replace(
            temporary_name,
            filename,
            src_dir_fd=parent_fd,
            dst_dir_fd=parent_fd,
        )
        temporary_name = None
        os.fsync(parent_fd)
    except WorldStateError:
        raise
    except OSError as exc:
        raise WorldStateError(f"cannot persist world state {path}: {exc}") from exc
    finally:
        if descriptor is not None:
            try:
                os.close(descriptor)
            except OSError:
                pass
        if temporary_name is not None:
            try:
                os.unlink(temporary_name, dir_fd=parent_fd)
            except OSError:
                pass
            else:
                try:
                    os.fsync(parent_fd)
                except OSError:
                    pass
        os.close(parent_fd)


class WorldStateStore:
    """Load, mutate, and durably persist one exact world revision."""

    def __init__(self, path: Path, *, world_id: str, world_revision: str) -> None:
        _state_path_components(path)
        self.path = path
        self.backup_path = path.with_name(f"{path.name}.bak")
        self.world_id = validate_world_id(world_id)
        self.world_revision = validate_world_revision(world_revision)
        self.load_status = "missing"
        self.load_error: str | None = None
        self.state = MatrixWorldState.empty(
            world_id=self.world_id, world_revision=self.world_revision
        )

    def load(self) -> MatrixWorldState:
        errors: list[str] = []
        found = False
        for label, candidate in (("primary", self.path), ("backup", self.backup_path)):
            try:
                payload = _read_regular_file(candidate)
            except _WorldStatePathMissing:
                continue
            except WorldStateError as exc:
                found = True
                errors.append(f"{label}: {exc}")
                continue
            found = True
            try:
                state = _decode_state_bytes(payload)
                if state.world_id != self.world_id:
                    raise WorldStateError(
                        f"world id {state.world_id!r} does not match {self.world_id!r}"
                    )
                if state.world_revision != self.world_revision:
                    raise WorldStateError(
                        "world revision does not match the active physics model"
                    )
            except WorldStateError as exc:
                errors.append(f"{label}: {exc}")
                continue
            self.state = state
            self.load_status = "loaded" if label == "primary" else "backup"
            self.load_error = "; ".join(errors) or None
            return self.state
        self.state = MatrixWorldState.empty(
            world_id=self.world_id, world_revision=self.world_revision
        )
        self.load_status = "invalid" if found else "missing"
        self.load_error = "; ".join(errors) or None
        return self.state

    def save(self, state: MatrixWorldState | None = None) -> None:
        candidate = self.state if state is None else state
        if (
            candidate.world_id != self.world_id
            or candidate.world_revision != self.world_revision
        ):
            raise WorldStateError("refusing to save state for another world revision")
        payload = (
            json.dumps(
                candidate.to_mapping(),
                indent=2,
                sort_keys=True,
                allow_nan=False,
            )
            + "\n"
        ).encode("utf-8")
        try:
            previous = _read_regular_file(self.path)
            previous_state = _decode_state_bytes(previous)
            if (
                previous_state.world_id != self.world_id
                or previous_state.world_revision != self.world_revision
            ):
                previous = None
        except WorldStateError:
            previous = None
        if previous is not None:
            _atomic_write(self.backup_path, previous)
        _atomic_write(self.path, payload)
        self.state = candidate

    def replace(self, state: MatrixWorldState) -> MatrixWorldState:
        self.save(state)
        return self.state


def default_world_state_path(*, profile: str, world_id: str) -> Path:
    profile_value = validate_tag(profile)
    world_value = validate_world_id(world_id)
    safe_world = re.sub(r"[^A-Za-z0-9_.+-]", "_", world_value)
    world_digest = hashlib.sha256(world_value.encode("ascii")).hexdigest()[:32]
    root = Path(
        os.environ.get(
            "XDG_STATE_HOME",
            os.fspath(Path.home() / ".local" / "state"),
        )
    )
    lexical_path = root / "matrix" / profile_value / (
        f"{safe_world}-{world_digest}.json"
    )
    return Path(os.path.abspath(os.fspath(lexical_path)))


def world_revision_for_files(
    *,
    world_id: str,
    native_scene: Path,
    canonical_model: Path,
    canonical_meshes: Path,
    scene_transform: str | None = None,
) -> str:
    """Bind a save slot to physics inputs without including its spawn override."""

    identity = validate_world_id(world_id)
    for path in (native_scene, canonical_model):
        if not path.is_file() or path.is_symlink():
            raise WorldStateError(f"world revision input is not a regular file: {path}")
    if not canonical_meshes.is_dir() or canonical_meshes.is_symlink():
        raise WorldStateError(
            f"world revision input is not a regular directory: {canonical_meshes}"
        )
    try:
        source_contract = physics_revision_payload(
            canonical_model,
            canonical_meshes,
            native_scene,
            scene_transform=scene_transform,
        )
    except (OSError, SonicPhysicsModelError) as exc:
        raise WorldStateError(f"cannot build physics source contract: {exc}") from exc
    payload = json.dumps(
        source_contract,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    digest = hashlib.sha256()
    digest.update(b"matrix-world-revision/v2\0")
    digest.update(identity.encode("ascii"))
    digest.update(b"\0")
    digest.update(payload)
    return digest.hexdigest()


def _parse_cli_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    revision = subparsers.add_parser("revision")
    revision.add_argument("--world-id", required=True)
    revision.add_argument("--native-scene", type=Path, required=True)
    revision.add_argument("--canonical-model", type=Path, required=True)
    revision.add_argument("--canonical-meshes", type=Path, required=True)
    revision.add_argument(
        "--scene-transform",
        choices=(SCENE_TRANSFORM_NONE, TOWN10_OPEN_BOUNDARY_TRANSFORM),
        default=SCENE_TRANSFORM_NONE,
    )

    default_path = subparsers.add_parser("default-path")
    default_path.add_argument("--profile", required=True)
    default_path.add_argument("--world-id", required=True)

    resolve = subparsers.add_parser("resolve-start")
    resolve.add_argument("--file", type=Path, required=True)
    resolve.add_argument("--world-id", required=True)
    resolve.add_argument("--world-revision", required=True)
    return parser.parse_args()


def main() -> int:
    args = _parse_cli_args()
    try:
        if args.command == "revision":
            print(
                world_revision_for_files(
                    world_id=args.world_id,
                    native_scene=args.native_scene,
                    canonical_model=args.canonical_model,
                    canonical_meshes=args.canonical_meshes,
                    scene_transform=args.scene_transform,
                )
            )
            return 0
        if args.command == "default-path":
            print(default_world_state_path(profile=args.profile, world_id=args.world_id))
            return 0
        if args.command == "resolve-start":
            store = WorldStateStore(
                args.file,
                world_id=args.world_id,
                world_revision=args.world_revision,
            )
            state = store.load()
            pose = state.last_exit or state.home
            source = "last_exit" if state.last_exit is not None else "home"
            if pose is None:
                print("none")
            else:
                print("pose")
                print(format(pose.x, ".17g"))
                print(format(pose.y, ".17g"))
                print(format(pose.z, ".17g"))
                print(format(pose.yaw_rad, ".17g"))
                print(source)
            print(store.load_status)
            return 0
    except WorldStateError as exc:
        raise SystemExit(f"[ERROR] {exc}") from exc
    raise SystemExit("[ERROR] unsupported world-state command")


__all__ = [
    "MatrixWorldState",
    "TeleportPoint",
    "TELEPORT_POINT_TYPE",
    "WorldPose",
    "WorldStateError",
    "WorldStateStore",
    "default_world_state_path",
    "validate_tag",
    "validate_world_id",
    "world_revision_for_files",
]


if __name__ == "__main__":
    raise SystemExit(main())
