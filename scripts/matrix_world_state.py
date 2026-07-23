#!/usr/bin/env python3
"""Durable, scene-bound Matrix game state and logical teleport points.

The store intentionally persists semantic poses rather than a complete MuJoCo
``mjData`` image.  A later runtime generation can combine one of these poses
with its canonical upright joint configuration, zero velocities, and a fresh
SONIC policy process.  Dynamic simulator state is never deserialized here.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from dataclasses import dataclass, replace
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import stat
import sys
import time
from typing import Callable, Iterable
import uuid

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from prepare_sonic_physics_model import (
    MOON_DYNAMIC_GROUND_MOCAP_TRANSFORM,
    SCENE_TRANSFORM_NONE,
    SonicPhysicsModelError,
    TOWN10_OPEN_BOUNDARY_TRANSFORM,
    physics_revision_payload,
)


WORLD_STATE_SCHEMA_V1 = "matrix-world-state/v1"
WORLD_STATE_SCHEMA = "matrix-world-state/v2"
WORLD_FRAME = "matrix_mj_world"
WORLD_UNITS = "m"
TELEPORT_POINT_TYPE = "matrix:teleport_point"
MAX_WORLD_ID_CHARS = 160
MAX_TAG_CHARS = 64
MAX_TAGS_PER_POINT = 16
MAX_TELEPORT_POINTS = 1024
MAX_RESUME_CHECKPOINTS = 16
MAX_INVALID_CHECKPOINTS = 64
RESUME_CHECKPOINT_DEDUP_METRES = 1.0
RESUME_CHECKPOINT_DEDUP_YAW_RAD = math.radians(30.0)
MAX_HORIZONTAL_METRES = 100_000.0
MIN_VERTICAL_METRES = -1_000.0
MAX_VERTICAL_METRES = 10_000.0
MAX_FALLEN_RESUME_DRIFT_METRES = 25.0
REJECT_COMMIT_GATE_DEFAULT_TIMEOUT_SECONDS = 15.0
REJECT_COMMIT_GATE_MIN_TIMEOUT_SECONDS = 0.1
REJECT_COMMIT_GATE_MAX_TIMEOUT_SECONDS = 60.0
REJECT_COMMIT_GATE_POLL_SECONDS = 0.01
REJECT_COMMIT_READY_PAYLOAD = b"matrix-world-state-reject-ready/v1\n"
REJECT_COMMIT_AUTHORIZE_PAYLOAD = b"matrix-world-state-reject-authorize/v1\n"
REJECT_COMMIT_CANCEL_PAYLOAD = b"matrix-world-state-reject-cancel/v1\n"
MAX_COMMIT_MARKER_PATH_BYTES = 4096

_WORLD_ID_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.:/-]{0,159}\Z")
_TAG_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.:+-]{0,63}\Z")
_ENTITY_ID_RE = re.compile(r"tp-[0-9a-f]{32}\Z")
_CHECKPOINT_ID_RE = re.compile(r"cp-[0-9a-f]{32}\Z")
_AUDIT_VALUE_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.:+-]{0,127}\Z")
_RESUME_SOURCE_VALUES = frozenset(
    {
        "upright_checkpoint",
        "fallen_xy_last_safe_upright",
        "fallen_outlier_last_safe",
        "teleport_command",
        "home",
    }
)
_SPECULATIVE_RESUME_SOURCES = frozenset(
    {
        "fallen_xy_last_safe_upright",
        "fallen_outlier_last_safe",
    }
)


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


def validate_checkpoint_id(value: object) -> str:
    if not isinstance(value, str) or _CHECKPOINT_ID_RE.fullmatch(value) is None:
        raise WorldStateError("checkpoint_id must use cp- followed by 32 lowercase hex digits")
    return value


def _validate_audit_value(value: object, *, label: str) -> str:
    if not isinstance(value, str) or _AUDIT_VALUE_RE.fullmatch(value) is None:
        raise WorldStateError(f"{label} must be 1-128 safe ASCII characters")
    return value


def _validate_timestamp(value: object, *, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise WorldStateError(f"{label} is invalid")
    return value


def _validate_generation(value: object, *, label: str = "generation") -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise WorldStateError(f"{label} must be a non-negative integer")
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


def _pose_distance_metres(left: WorldPose, right: WorldPose) -> float:
    return math.sqrt(
        ((left.x - right.x) ** 2)
        + ((left.y - right.y) ** 2)
        + ((left.z - right.z) ** 2)
    )


def _yaw_distance_rad(left: float, right: float) -> float:
    return abs(math.atan2(math.sin(left - right), math.cos(left - right)))


@dataclass(frozen=True)
class ResumeCheckpoint:
    checkpoint_id: str
    pose: WorldPose
    source: str
    created_at_unix_ns: int
    anchor_pose: WorldPose | None = None

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "checkpoint_id", validate_checkpoint_id(self.checkpoint_id)
        )
        if not isinstance(self.pose, WorldPose):
            raise WorldStateError("resume checkpoint pose is invalid")
        anchor_pose = self.pose if self.anchor_pose is None else self.anchor_pose
        if not isinstance(anchor_pose, WorldPose):
            raise WorldStateError("resume checkpoint anchor pose is invalid")
        object.__setattr__(self, "anchor_pose", anchor_pose)
        object.__setattr__(
            self,
            "source",
            _validate_audit_value(self.source, label="resume checkpoint source"),
        )
        object.__setattr__(
            self,
            "created_at_unix_ns",
            _validate_timestamp(
                self.created_at_unix_ns,
                label="resume checkpoint timestamp",
            ),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "checkpoint_id": self.checkpoint_id,
            "pose": self.pose.to_mapping(),
            "source": self.source,
            "created_at_unix_ns": self.created_at_unix_ns,
            "anchor_pose": self.anchor_pose.to_mapping(),
        }

    @classmethod
    def from_mapping(cls, value: object, *, index: int) -> "ResumeCheckpoint":
        legacy_required = {"checkpoint_id", "pose", "source", "created_at_unix_ns"}
        current_required = legacy_required | {"anchor_pose"}
        if not isinstance(value, dict) or frozenset(value) not in {
            frozenset(legacy_required),
            frozenset(current_required),
        }:
            raise WorldStateError(
                f"resume_checkpoints[{index}] has an invalid schema"
            )
        pose = WorldPose.from_mapping(
            value.get("pose"), label=f"resume_checkpoints[{index}].pose"
        )
        anchor_value = value.get("anchor_pose", value.get("pose"))
        return cls(
            checkpoint_id=value.get("checkpoint_id"),
            pose=pose,
            source=value.get("source"),
            created_at_unix_ns=value.get("created_at_unix_ns"),
            anchor_pose=WorldPose.from_mapping(
                anchor_value,
                label=f"resume_checkpoints[{index}].anchor_pose",
            ),
        )


@dataclass(frozen=True)
class InvalidCheckpoint:
    checkpoint: ResumeCheckpoint
    invalidated_at_unix_ns: int
    reason: str
    run_id: str

    def __post_init__(self) -> None:
        if not isinstance(self.checkpoint, ResumeCheckpoint):
            raise WorldStateError("invalid checkpoint tombstone payload is invalid")
        object.__setattr__(
            self,
            "invalidated_at_unix_ns",
            _validate_timestamp(
                self.invalidated_at_unix_ns,
                label="invalid checkpoint timestamp",
            ),
        )
        object.__setattr__(
            self,
            "reason",
            _validate_audit_value(self.reason, label="invalid checkpoint reason"),
        )
        object.__setattr__(
            self,
            "run_id",
            _validate_audit_value(self.run_id, label="invalid checkpoint run_id"),
        )

    @property
    def checkpoint_id(self) -> str:
        return self.checkpoint.checkpoint_id

    def to_mapping(self) -> dict[str, object]:
        return {
            "checkpoint": self.checkpoint.to_mapping(),
            "invalidated_at_unix_ns": self.invalidated_at_unix_ns,
            "reason": self.reason,
            "run_id": self.run_id,
        }

    @classmethod
    def from_mapping(cls, value: object, *, index: int) -> "InvalidCheckpoint":
        required = {"checkpoint", "invalidated_at_unix_ns", "reason", "run_id"}
        if not isinstance(value, dict) or set(value) != required:
            raise WorldStateError(
                f"invalid_checkpoints[{index}] has an invalid schema"
            )
        return cls(
            checkpoint=ResumeCheckpoint.from_mapping(
                value.get("checkpoint"), index=index
            ),
            invalidated_at_unix_ns=value.get("invalidated_at_unix_ns"),
            reason=value.get("reason"),
            run_id=value.get("run_id"),
        )


@dataclass(frozen=True)
class ResolvedStart:
    pose: WorldPose | None
    source: str
    checkpoint_id: str | None
    generation: int

    def __post_init__(self) -> None:
        if self.pose is not None and not isinstance(self.pose, WorldPose):
            raise WorldStateError("resolved start pose is invalid")
        object.__setattr__(
            self, "source", _validate_audit_value(self.source, label="start source")
        )
        if self.checkpoint_id is not None:
            object.__setattr__(
                self,
                "checkpoint_id",
                validate_checkpoint_id(self.checkpoint_id),
            )
        object.__setattr__(self, "generation", _validate_generation(self.generation))


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
    generation: int = 0
    resume_checkpoints: tuple[ResumeCheckpoint, ...] = ()
    invalid_checkpoints: tuple[InvalidCheckpoint, ...] = ()
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
        if (
            self.resume_source is not None
            and self.resume_source not in _RESUME_SOURCE_VALUES
        ):
            raise WorldStateError("resume_source is invalid")
        object.__setattr__(self, "generation", _validate_generation(self.generation))
        if not isinstance(self.resume_checkpoints, tuple) or len(
            self.resume_checkpoints
        ) > MAX_RESUME_CHECKPOINTS:
            raise WorldStateError(
                "resume_checkpoints exceeds the "
                f"{MAX_RESUME_CHECKPOINTS} checkpoint limit"
            )
        if any(
            not isinstance(checkpoint, ResumeCheckpoint)
            for checkpoint in self.resume_checkpoints
        ):
            raise WorldStateError("resume_checkpoints contains an invalid checkpoint")
        if not isinstance(self.invalid_checkpoints, tuple) or len(
            self.invalid_checkpoints
        ) > MAX_INVALID_CHECKPOINTS:
            raise WorldStateError(
                "invalid_checkpoints exceeds the "
                f"{MAX_INVALID_CHECKPOINTS} tombstone limit"
            )
        if any(
            not isinstance(checkpoint, InvalidCheckpoint)
            for checkpoint in self.invalid_checkpoints
        ):
            raise WorldStateError("invalid_checkpoints contains an invalid tombstone")
        active_ids = [
            checkpoint.checkpoint_id for checkpoint in self.resume_checkpoints
        ]
        invalid_ids = [
            checkpoint.checkpoint_id for checkpoint in self.invalid_checkpoints
        ]
        if len(active_ids) != len(set(active_ids)):
            raise WorldStateError("active resume checkpoint IDs must be unique")
        if len(invalid_ids) != len(set(invalid_ids)):
            raise WorldStateError("invalid checkpoint IDs must be unique")
        if set(active_ids).intersection(invalid_ids):
            raise WorldStateError(
                "active and invalid checkpoint IDs must be disjoint"
            )
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
        object.__setattr__(
            self,
            "updated_at_unix_ns",
            _validate_timestamp(
                self.updated_at_unix_ns, label="updated_at_unix_ns"
            ),
        )

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
            "generation": self.generation,
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
            "resume_checkpoints": [
                checkpoint.to_mapping() for checkpoint in self.resume_checkpoints
            ],
            "invalid_checkpoints": [
                checkpoint.to_mapping() for checkpoint in self.invalid_checkpoints
            ],
            "teleport_points": [
                point.to_mapping() for point in self.teleport_points
            ],
            "updated_at_unix_ns": self.updated_at_unix_ns,
        }

    @classmethod
    def from_mapping(cls, value: object) -> "MatrixWorldState":
        common = {
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
        if not isinstance(value, dict):
            raise WorldStateError("world state has an invalid schema")
        schema = value.get("schema")
        if schema == WORLD_STATE_SCHEMA_V1:
            required = common
        elif schema == WORLD_STATE_SCHEMA:
            required = common | {
                "generation",
                "resume_checkpoints",
                "invalid_checkpoints",
            }
        else:
            raise WorldStateError("world state schema version is unsupported")
        if set(value) != required:
            raise WorldStateError("world state has an invalid schema")
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
        generation = 0
        resume_checkpoints: tuple[ResumeCheckpoint, ...] = ()
        invalid_checkpoints: tuple[InvalidCheckpoint, ...] = ()
        if schema == WORLD_STATE_SCHEMA:
            generation = _validate_generation(value.get("generation"))
            raw_checkpoints = value.get("resume_checkpoints")
            if not isinstance(raw_checkpoints, list):
                raise WorldStateError("resume_checkpoints must be a list")
            resume_checkpoints = tuple(
                ResumeCheckpoint.from_mapping(checkpoint, index=index)
                for index, checkpoint in enumerate(raw_checkpoints)
            )
            raw_invalid = value.get("invalid_checkpoints")
            if not isinstance(raw_invalid, list):
                raise WorldStateError("invalid_checkpoints must be a list")
            invalid_checkpoints = tuple(
                InvalidCheckpoint.from_mapping(checkpoint, index=index)
                for index, checkpoint in enumerate(raw_invalid)
            )

        state = cls(
            world_id=world.get("id"),
            world_revision=world.get("revision"),
            last_observed=optional_pose("last_observed"),
            last_safe=optional_pose("last_safe"),
            last_exit=optional_pose("last_exit"),
            home=optional_pose("home"),
            resume_source=value.get("resume_source"),
            generation=generation,
            resume_checkpoints=resume_checkpoints,
            invalid_checkpoints=invalid_checkpoints,
            teleport_points=tuple(
                TeleportPoint.from_mapping(point, index=index)
                for index, point in enumerate(points)
            ),
            updated_at_unix_ns=value.get("updated_at_unix_ns"),
        )
        if schema == WORLD_STATE_SCHEMA_V1:
            legacy_pose, legacy_source = state.resume_pose()
            if legacy_pose is not None:
                checkpoint = ResumeCheckpoint(
                    checkpoint_id=_legacy_checkpoint_id(
                        world_id=state.world_id,
                        world_revision=state.world_revision,
                        pose=legacy_pose,
                        source=legacy_source,
                        updated_at_unix_ns=state.updated_at_unix_ns,
                    ),
                    pose=legacy_pose,
                    source=legacy_source,
                    created_at_unix_ns=state.updated_at_unix_ns,
                )
                state = replace(state, resume_checkpoints=(checkpoint,))
        return state._normalize_speculative_checkpoint_ring()

    def _normalize_speculative_checkpoint_ring(self) -> "MatrixWorldState":
        """Migrate legacy fall cascades into one quarantinable candidate.

        Older v2 writers appended a new synthesized upright pose while SONIC's
        session-sticky fall flag remained set.  A long recovery could therefore
        evict all sixteen trusted checkpoints.  Retain only the newest fallen
        candidate, restore a deterministic trusted anchor from ``last_safe``
        when needed, and tombstone the superseded speculative entries.
        """

        trusted = [
            checkpoint
            for checkpoint in self.resume_checkpoints
            if checkpoint.source not in _SPECULATIVE_RESUME_SOURCES
        ]
        speculative = [
            checkpoint
            for checkpoint in self.resume_checkpoints
            if checkpoint.source in _SPECULATIVE_RESUME_SOURCES
        ]
        newest_speculative = speculative[-1] if speculative else None
        if not trusted and self.last_safe is not None:
            used_ids = {
                checkpoint.checkpoint_id for checkpoint in self.resume_checkpoints
            } | {
                checkpoint.checkpoint_id for checkpoint in self.invalid_checkpoints
            }
            candidate = _legacy_checkpoint_id(
                world_id=self.world_id,
                world_revision=self.world_revision,
                pose=self.last_safe,
                source="upright_checkpoint",
                updated_at_unix_ns=self.updated_at_unix_ns,
            )
            if candidate in used_ids:
                for nonce in range(1, 17):
                    digest = hashlib.sha256(
                        f"{candidate}:{nonce}".encode("ascii")
                    ).hexdigest()
                    candidate = f"cp-{digest[:32]}"
                    if candidate not in used_ids:
                        break
                else:
                    raise WorldStateError(
                        "cannot allocate migrated trusted checkpoint ID"
                    )
            trusted.append(
                ResumeCheckpoint(
                    checkpoint_id=candidate,
                    pose=self.last_safe,
                    source="upright_checkpoint",
                    created_at_unix_ns=self.updated_at_unix_ns,
                )
            )
        if newest_speculative is not None and not trusted:
            # A fallen-derived pose without a trusted upright anchor cannot be
            # reconstructed safely.  Preserve it only as audit evidence.
            newest_speculative = None

        capacity = MAX_RESUME_CHECKPOINTS - (
            1 if newest_speculative is not None else 0
        )
        normalized = tuple(trusted[-capacity:])
        if newest_speculative is not None:
            normalized = (*normalized, newest_speculative)

        superseded = [
            checkpoint
            for checkpoint in speculative
            if newest_speculative is None
            or checkpoint.checkpoint_id != newest_speculative.checkpoint_id
        ]
        invalid = self.invalid_checkpoints
        if superseded:
            invalid = (
                *invalid,
                *(
                    InvalidCheckpoint(
                        checkpoint=checkpoint,
                        invalidated_at_unix_ns=self.updated_at_unix_ns,
                        reason="speculative_checkpoint_superseded",
                        run_id="world-state-load-migration",
                    )
                    for checkpoint in superseded
                ),
            )[-MAX_INVALID_CHECKPOINTS:]

        if (
            normalized == self.resume_checkpoints
            and invalid == self.invalid_checkpoints
        ):
            return self
        active = normalized[-1] if normalized else None
        trusted_active = next(
            (
                checkpoint
                for checkpoint in reversed(normalized)
                if checkpoint.source not in _SPECULATIVE_RESUME_SOURCES
            ),
            None,
        )
        last_safe = (
            self.last_safe
            if self.last_safe is not None
            else trusted_active.pose
            if trusted_active is not None
            else None
        )
        return replace(
            self,
            last_safe=last_safe,
            last_exit=active.pose if active is not None else last_safe,
            resume_source=(
                active.source
                if active is not None and active.source in _RESUME_SOURCE_VALUES
                else "upright_checkpoint"
                if active is not None
                else None
            ),
            resume_checkpoints=normalized,
            invalid_checkpoints=invalid,
        )

    def _next_generation(self) -> int:
        return self.generation + 1

    def _new_checkpoint(
        self,
        *,
        pose: WorldPose,
        source: str,
        timestamp: int,
        force_new: bool,
    ) -> tuple[ResumeCheckpoint, ...]:
        speculative = tuple(
            checkpoint
            for checkpoint in self.resume_checkpoints
            if checkpoint.source in _SPECULATIVE_RESUME_SOURCES
        )
        trusted = tuple(
            checkpoint
            for checkpoint in self.resume_checkpoints
            if checkpoint.source not in _SPECULATIVE_RESUME_SOURCES
        )
        if source in _SPECULATIVE_RESUME_SOURCES:
            if speculative:
                candidate = replace(
                    speculative[-1],
                    pose=pose,
                    source=source,
                    created_at_unix_ns=timestamp,
                )
            else:
                used_ids = {
                    checkpoint.checkpoint_id
                    for checkpoint in self.resume_checkpoints
                } | {
                    checkpoint.checkpoint_id
                    for checkpoint in self.invalid_checkpoints
                }
                checkpoint_id = ""
                for _attempt in range(16):
                    proposed = f"cp-{uuid.uuid4().hex}"
                    if proposed not in used_ids:
                        checkpoint_id = proposed
                        break
                if not checkpoint_id:
                    raise WorldStateError(
                        "cannot allocate a unique speculative checkpoint ID"
                    )
                candidate = ResumeCheckpoint(
                    checkpoint_id=checkpoint_id,
                    pose=pose,
                    source=source,
                    created_at_unix_ns=timestamp,
                )
            return (*trusted[-(MAX_RESUME_CHECKPOINTS - 1):], candidate)

        # A newly verified upright/teleport point supersedes the prior fallen
        # candidate.  The full trusted quota becomes available again.
        checkpoints = trusted
        if checkpoints and not force_new:
            latest = checkpoints[-1]
            if (
                _pose_distance_metres(latest.anchor_pose, pose)
                < RESUME_CHECKPOINT_DEDUP_METRES
                and _yaw_distance_rad(latest.anchor_pose.yaw_rad, pose.yaw_rad)
                < RESUME_CHECKPOINT_DEDUP_YAW_RAD
            ):
                # Keep the segment's original anchor for cumulative thresholding,
                # while refreshing the resumable pose so a normal restart does
                # not retreat by almost the entire deduplication window.
                latest = replace(
                    latest,
                    pose=pose,
                    source=source,
                    created_at_unix_ns=timestamp,
                )
                return (*checkpoints[:-1], latest)
        used_ids = {
            checkpoint.checkpoint_id for checkpoint in self.resume_checkpoints
        } | {checkpoint.checkpoint_id for checkpoint in self.invalid_checkpoints}
        checkpoint_id = ""
        for _attempt in range(16):
            candidate = f"cp-{uuid.uuid4().hex}"
            if candidate not in used_ids:
                checkpoint_id = candidate
                break
        if not checkpoint_id:
            raise WorldStateError("cannot allocate a unique resume checkpoint ID")
        checkpoint = ResumeCheckpoint(
            checkpoint_id=checkpoint_id,
            pose=pose,
            source=source,
            created_at_unix_ns=timestamp,
        )
        return (*checkpoints, checkpoint)[-MAX_RESUME_CHECKPOINTS:]

    def checkpoint(
        self,
        pose: WorldPose,
        *,
        upright: bool,
        clearance_safe: bool = True,
        now_unix_ns: int | None = None,
    ) -> "MatrixWorldState":
        if (
            not isinstance(pose, WorldPose)
            or type(upright) is not bool
            or type(clearance_safe) is not bool
        ):
            raise WorldStateError(
                "checkpoint requires a pose, upright flag, and clearance flag"
            )
        timestamp = time.time_ns() if now_unix_ns is None else now_unix_ns
        timestamp = _validate_timestamp(timestamp, label="checkpoint timestamp")
        if not clearance_safe:
            # Preserve every previously validated resume checkpoint.  A pose
            # intersecting scene geometry is useful as last-observed evidence,
            # but its XY must never be combined with an older upright Z/yaw.
            return replace(
                self,
                last_observed=pose,
                generation=self._next_generation(),
                updated_at_unix_ns=timestamp,
            )
        if upright:
            checkpoints = self._new_checkpoint(
                pose=pose,
                source="upright_checkpoint",
                timestamp=timestamp,
                force_new=False,
            )
            return replace(
                self,
                last_observed=pose,
                last_safe=pose,
                last_exit=pose,
                resume_source="upright_checkpoint",
                generation=self._next_generation(),
                resume_checkpoints=checkpoints,
                updated_at_unix_ns=timestamp,
            )
        if self.last_safe is None:
            return replace(
                self,
                last_observed=pose,
                generation=self._next_generation(),
                updated_at_unix_ns=timestamp,
            )
        if pose.distance_xy(self.last_safe) > MAX_FALLEN_RESUME_DRIFT_METRES:
            checkpoints = self._new_checkpoint(
                pose=self.last_safe,
                source="fallen_outlier_last_safe",
                timestamp=timestamp,
                force_new=False,
            )
            return replace(
                self,
                last_observed=pose,
                last_exit=self.last_safe,
                resume_source="fallen_outlier_last_safe",
                generation=self._next_generation(),
                resume_checkpoints=checkpoints,
                updated_at_unix_ns=timestamp,
            )
        upright_exit = WorldPose(
            pose.x,
            pose.y,
            self.last_safe.z,
            self.last_safe.yaw_rad,
        )
        checkpoints = self._new_checkpoint(
            pose=upright_exit,
            source="fallen_xy_last_safe_upright",
            timestamp=timestamp,
            force_new=False,
        )
        return replace(
            self,
            last_observed=pose,
            last_exit=upright_exit,
            resume_source="fallen_xy_last_safe_upright",
            generation=self._next_generation(),
            resume_checkpoints=checkpoints,
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
        timestamp = _validate_timestamp(timestamp, label="resume pose timestamp")
        checkpoints = self._new_checkpoint(
            pose=pose,
            source=source,
            timestamp=timestamp,
            force_new=True,
        )
        return replace(
            self,
            last_observed=pose,
            last_safe=pose,
            last_exit=pose,
            resume_source=source,
            generation=self._next_generation(),
            resume_checkpoints=checkpoints,
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
        timestamp = _validate_timestamp(timestamp, label="teleport point timestamp")
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
                generation=self._next_generation(),
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

    def resolve_start(self, default: WorldPose | None = None) -> ResolvedStart:
        if default is not None and not isinstance(default, WorldPose):
            raise WorldStateError("default start pose must be a WorldPose or null")
        if self.resume_checkpoints:
            checkpoint = self.resume_checkpoints[-1]
            return ResolvedStart(
                pose=checkpoint.pose,
                source="last_exit",
                checkpoint_id=checkpoint.checkpoint_id,
                generation=self.generation,
            )
        if self.last_exit is not None:
            if (
                self.last_safe is not None
                and self.last_exit.distance_xy(self.last_safe)
                > MAX_FALLEN_RESUME_DRIFT_METRES
            ):
                return ResolvedStart(
                    pose=self.last_safe,
                    source="last_safe_outlier_fallback",
                    checkpoint_id=None,
                    generation=self.generation,
                )
            return ResolvedStart(
                pose=self.last_exit,
                source="last_exit",
                checkpoint_id=None,
                generation=self.generation,
            )
        if self.home is not None:
            return ResolvedStart(
                pose=self.home,
                source="home",
                checkpoint_id=None,
                generation=self.generation,
            )
        if default is not None:
            return ResolvedStart(
                pose=default,
                source="default",
                checkpoint_id=None,
                generation=self.generation,
            )
        return ResolvedStart(
            pose=None,
            source="none",
            checkpoint_id=None,
            generation=self.generation,
        )

    def resume_pose(self) -> tuple[WorldPose | None, str]:
        resolved = self.resolve_start()
        return resolved.pose, resolved.source

    def startup_pose(self, default: WorldPose) -> tuple[WorldPose, str]:
        resolved = self.resolve_start(default)
        assert resolved.pose is not None
        return resolved.pose, resolved.source

    def reject_active_checkpoint(
        self,
        *,
        expected_id: str,
        expected_generation: int,
        reason: str,
        run_id: str,
        now_unix_ns: int | None = None,
    ) -> "RejectActiveCheckpointResult":
        checkpoint_id = validate_checkpoint_id(expected_id)
        generation = _validate_generation(
            expected_generation, label="expected_generation"
        )
        validated_reason = _validate_audit_value(
            reason, label="invalid checkpoint reason"
        )
        validated_run_id = _validate_audit_value(
            run_id, label="invalid checkpoint run_id"
        )
        for tombstone in self.invalid_checkpoints:
            if tombstone.checkpoint_id != checkpoint_id:
                continue
            if tombstone.reason != validated_reason or tombstone.run_id != validated_run_id:
                raise WorldStateError(
                    "checkpoint was already rejected by a different audit event"
                )
            replacement_checkpoint = (
                self.resume_checkpoints[-1] if self.resume_checkpoints else None
            )
            return RejectActiveCheckpointResult(
                state=self,
                tombstone=tombstone,
                rejected_checkpoint=tombstone.checkpoint,
                replacement_checkpoint=replacement_checkpoint,
                idempotent=True,
            )
        if generation != self.generation:
            raise WorldStateError(
                "resume checkpoint generation changed before rejection"
            )
        if not self.resume_checkpoints:
            raise WorldStateError("there is no active resume checkpoint to reject")
        rejected = self.resume_checkpoints[-1]
        if rejected.checkpoint_id != checkpoint_id:
            raise WorldStateError("only the exact active resume checkpoint may be rejected")
        timestamp = time.time_ns() if now_unix_ns is None else now_unix_ns
        timestamp = _validate_timestamp(
            timestamp, label="checkpoint rejection timestamp"
        )
        tombstone = InvalidCheckpoint(
            checkpoint=rejected,
            invalidated_at_unix_ns=timestamp,
            reason=validated_reason,
            run_id=validated_run_id,
        )
        checkpoints = self.resume_checkpoints[:-1]
        invalid = (*self.invalid_checkpoints, tombstone)[-MAX_INVALID_CHECKPOINTS:]
        replacement_checkpoint = checkpoints[-1] if checkpoints else None
        rejected_speculative = rejected.source in _SPECULATIVE_RESUME_SOURCES
        replacement_pose = (
            replacement_checkpoint.pose
            if replacement_checkpoint is not None
            else self.last_safe
            if rejected_speculative
            else None
        )
        checkpoint_source = (
            replacement_checkpoint.source
            if replacement_checkpoint is not None
            else None
        )
        replacement_source = (
            checkpoint_source
            if checkpoint_source in _RESUME_SOURCE_VALUES
            else ("upright_checkpoint" if checkpoint_source is not None else None)
        )
        state = replace(
            self,
            last_observed=replacement_pose,
            last_safe=(self.last_safe if rejected_speculative else replacement_pose),
            last_exit=replacement_pose,
            resume_source=replacement_source,
            generation=self._next_generation(),
            resume_checkpoints=checkpoints,
            invalid_checkpoints=invalid,
            updated_at_unix_ns=timestamp,
        )
        return RejectActiveCheckpointResult(
            state=state,
            tombstone=tombstone,
            rejected_checkpoint=rejected,
            replacement_checkpoint=replacement_checkpoint,
            idempotent=False,
        )


@dataclass(frozen=True)
class RejectActiveCheckpointResult:
    state: MatrixWorldState
    tombstone: InvalidCheckpoint
    rejected_checkpoint: ResumeCheckpoint
    replacement_checkpoint: ResumeCheckpoint | None
    idempotent: bool

    def __post_init__(self) -> None:
        if not isinstance(self.state, MatrixWorldState):
            raise WorldStateError("checkpoint rejection state is invalid")
        if not isinstance(self.tombstone, InvalidCheckpoint):
            raise WorldStateError("checkpoint rejection tombstone is invalid")
        if not isinstance(self.rejected_checkpoint, ResumeCheckpoint):
            raise WorldStateError("rejected checkpoint payload is invalid")
        if self.replacement_checkpoint is not None and not isinstance(
            self.replacement_checkpoint, ResumeCheckpoint
        ):
            raise WorldStateError("replacement checkpoint payload is invalid")
        if type(self.idempotent) is not bool:
            raise WorldStateError("checkpoint rejection idempotency flag is invalid")


def _legacy_checkpoint_id(
    *,
    world_id: str,
    world_revision: str,
    pose: WorldPose,
    source: str,
    updated_at_unix_ns: int,
) -> str:
    payload = json.dumps(
        {
            "world_id": world_id,
            "world_revision": world_revision,
            "pose": pose.to_mapping(),
            "source": source,
            "updated_at_unix_ns": updated_at_unix_ns,
        },
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    digest = hashlib.sha256(
        b"matrix-world-state-v1-migration\0" + payload
    ).hexdigest()
    return f"cp-{digest[:32]}"


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


def _validate_commit_marker_path(
    path: Path,
    *,
    label: str,
    forbidden_paths: frozenset[Path],
) -> Path:
    marker_path = Path(path)
    _state_path_components(marker_path)
    encoded = os.fsencode(marker_path)
    if len(encoded) > MAX_COMMIT_MARKER_PATH_BYTES:
        raise WorldStateError(f"{label} path is too long")
    if any(byte < 0x20 or byte == 0x7F for byte in encoded):
        raise WorldStateError(f"{label} path contains control characters")
    if marker_path in forbidden_paths:
        raise WorldStateError(f"{label} path collides with durable world state")
    return marker_path


def _open_private_marker_parent(path: Path) -> tuple[int, str]:
    parent_fd, filename = _open_parent_directory(path, create=False)
    try:
        metadata = os.fstat(parent_fd)
        if not stat.S_ISDIR(metadata.st_mode):
            raise WorldStateError(
                f"commit marker parent is not a directory: {path.parent}"
            )
        if metadata.st_uid != os.getuid():
            raise WorldStateError(
                f"commit marker parent has an unexpected owner: {path.parent}"
            )
        if stat.S_IMODE(metadata.st_mode) & 0o077:
            raise WorldStateError(
                f"commit marker parent must be private: {path.parent}"
            )
        return parent_fd, filename
    except BaseException:
        os.close(parent_fd)
        raise


def _read_private_commit_marker(
    path: Path,
    *,
    expected_payload: bytes,
    label: str,
) -> bool:
    """Return whether an exact, private, regular marker is present."""

    nofollow = getattr(os, "O_NOFOLLOW", None)
    nonblock = getattr(os, "O_NONBLOCK", None)
    if nofollow is None or nonblock is None:
        raise WorldStateError(
            "secure commit markers require O_NOFOLLOW and O_NONBLOCK support"
        )
    parent_fd, filename = _open_private_marker_parent(path)
    descriptor: int | None = None
    try:
        flags = os.O_RDONLY | nofollow | nonblock
        flags |= getattr(os, "O_CLOEXEC", 0)
        try:
            descriptor = os.open(filename, flags, dir_fd=parent_fd)
        except FileNotFoundError:
            return False
        except OSError as exc:
            raise WorldStateError(f"cannot open {label} marker {path}: {exc}") from exc
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode):
            raise WorldStateError(f"{label} marker is not a regular file: {path}")
        if metadata.st_uid != os.getuid():
            raise WorldStateError(f"{label} marker has an unexpected owner: {path}")
        if stat.S_IMODE(metadata.st_mode) & 0o077:
            raise WorldStateError(f"{label} marker must be private: {path}")
        if metadata.st_nlink != 1:
            raise WorldStateError(f"{label} marker must not be hard-linked: {path}")
        chunks: list[bytes] = []
        remaining = len(expected_payload) + 1
        while remaining:
            chunk = os.read(descriptor, remaining)
            if not chunk:
                break
            chunks.append(chunk)
            remaining -= len(chunk)
        payload = b"".join(chunks)
        if payload != expected_payload:
            raise WorldStateError(f"{label} marker has an invalid protocol payload")
        return True
    except WorldStateError:
        raise
    except OSError as exc:
        raise WorldStateError(f"cannot read {label} marker {path}: {exc}") from exc
    finally:
        if descriptor is not None:
            os.close(descriptor)
        os.close(parent_fd)


def _create_private_commit_marker(
    path: Path,
    *,
    payload: bytes,
    label: str,
) -> None:
    nofollow = getattr(os, "O_NOFOLLOW", None)
    if nofollow is None:
        raise WorldStateError("secure commit markers require O_NOFOLLOW support")
    parent_fd, filename = _open_private_marker_parent(path)
    descriptor: int | None = None
    created = False
    published = False
    try:
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | nofollow
        flags |= getattr(os, "O_CLOEXEC", 0)
        try:
            descriptor = os.open(filename, flags, 0o600, dir_fd=parent_fd)
            created = True
        except FileExistsError as exc:
            raise WorldStateError(f"refusing stale {label} marker: {path}") from exc
        except OSError as exc:
            raise WorldStateError(f"cannot create {label} marker {path}: {exc}") from exc
        os.fchmod(descriptor, 0o600)
        view = memoryview(payload)
        offset = 0
        while offset < len(view):
            written = os.write(descriptor, view[offset:])
            if written <= 0:
                raise WorldStateError(f"short write while creating {label} marker")
            offset += written
        os.fsync(descriptor)
        os.close(descriptor)
        descriptor = None
        os.fsync(parent_fd)
        published = True
    except WorldStateError:
        raise
    except OSError as exc:
        raise WorldStateError(f"cannot publish {label} marker {path}: {exc}") from exc
    finally:
        if descriptor is not None:
            try:
                os.close(descriptor)
            except OSError:
                pass
        if created and not published:
            try:
                os.unlink(filename, dir_fd=parent_fd)
            except OSError:
                pass
        os.close(parent_fd)


class _RejectCheckpointCommitGate:
    """Hold a prepared rejection until a private authorize marker is published."""

    def __init__(
        self,
        *,
        state_path: Path,
        backup_path: Path,
        ready_path: Path,
        authorize_path: Path,
        cancel_path: Path,
        timeout_seconds: float,
    ) -> None:
        if isinstance(timeout_seconds, bool) or not isinstance(
            timeout_seconds, (int, float)
        ):
            raise WorldStateError("commit gate timeout must be a finite number")
        timeout = float(timeout_seconds)
        if not math.isfinite(timeout) or not (
            REJECT_COMMIT_GATE_MIN_TIMEOUT_SECONDS
            <= timeout
            <= REJECT_COMMIT_GATE_MAX_TIMEOUT_SECONDS
        ):
            raise WorldStateError(
                "commit gate timeout must be between "
                f"{REJECT_COMMIT_GATE_MIN_TIMEOUT_SECONDS:g} and "
                f"{REJECT_COMMIT_GATE_MAX_TIMEOUT_SECONDS:g} seconds"
            )
        forbidden = frozenset({state_path, backup_path})
        self.ready_path = _validate_commit_marker_path(
            ready_path,
            label="commit-ready",
            forbidden_paths=forbidden,
        )
        self.authorize_path = _validate_commit_marker_path(
            authorize_path,
            label="commit-authorize",
            forbidden_paths=forbidden,
        )
        self.cancel_path = _validate_commit_marker_path(
            cancel_path,
            label="commit-cancel",
            forbidden_paths=forbidden,
        )
        if len({self.ready_path, self.authorize_path, self.cancel_path}) != 3:
            raise WorldStateError("commit marker paths must be distinct")
        self.timeout_seconds = timeout
        self._signal_number: int | None = None
        self._commit_started = False

    def _handle_signal(self, signal_number: int, _frame: object) -> None:
        if self._commit_started or self._signal_number is not None:
            return
        self._signal_number = signal_number

    @contextmanager
    def signal_handlers(self):
        previous: dict[int, object] = {}
        try:
            for signal_number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
                previous[signal_number] = signal.getsignal(signal_number)
                signal.signal(signal_number, self._handle_signal)
            yield
        finally:
            for signal_number, handler in previous.items():
                signal.signal(signal_number, handler)

    def await_authorization(self, _result: "RejectActiveCheckpointResult") -> None:
        if self._signal_number is not None:
            raise WorldStateError(
                f"checkpoint rejection canceled by signal {self._signal_number}"
            )
        if _read_private_commit_marker(
            self.ready_path,
            expected_payload=REJECT_COMMIT_READY_PAYLOAD,
            label="commit-ready",
        ):
            raise WorldStateError(f"refusing stale commit-ready marker: {self.ready_path}")
        if _read_private_commit_marker(
            self.authorize_path,
            expected_payload=REJECT_COMMIT_AUTHORIZE_PAYLOAD,
            label="commit-authorize",
        ):
            raise WorldStateError(
                f"refusing preexisting commit-authorize marker: {self.authorize_path}"
            )
        if _read_private_commit_marker(
            self.cancel_path,
            expected_payload=REJECT_COMMIT_CANCEL_PAYLOAD,
            label="commit-cancel",
        ):
            raise WorldStateError("checkpoint rejection canceled before readiness")

        _create_private_commit_marker(
            self.ready_path,
            payload=REJECT_COMMIT_READY_PAYLOAD,
            label="commit-ready",
        )
        deadline_monotonic = time.monotonic() + self.timeout_seconds
        while True:
            if _read_private_commit_marker(
                self.cancel_path,
                expected_payload=REJECT_COMMIT_CANCEL_PAYLOAD,
                label="commit-cancel",
            ):
                raise WorldStateError("checkpoint rejection canceled by marker")
            if _read_private_commit_marker(
                self.authorize_path,
                expected_payload=REJECT_COMMIT_AUTHORIZE_PAYLOAD,
                label="commit-authorize",
            ):
                # Close the outer launcher's cancel-vs-authorize race with one
                # final cancel observation.  The signal-mask operation below is
                # the irreversible helper-side commit point: a signal delivered
                # before it must set _signal_number and cancel, while a signal
                # generated after it remains pending until _commit_started is
                # true and therefore cannot interrupt the two-replica write.
                if _read_private_commit_marker(
                    self.cancel_path,
                    expected_payload=REJECT_COMMIT_CANCEL_PAYLOAD,
                    label="commit-cancel",
                ):
                    raise WorldStateError("checkpoint rejection canceled by marker")
                if self._signal_number is not None:
                    raise WorldStateError(
                        "checkpoint rejection canceled by signal "
                        f"{self._signal_number}"
                    )
                commit_signals = {
                    signal.SIGINT,
                    signal.SIGTERM,
                    signal.SIGHUP,
                }
                try:
                    previous_mask = signal.pthread_sigmask(
                        signal.SIG_BLOCK,
                        commit_signals,
                    )
                except (AttributeError, OSError, ValueError) as exc:
                    raise WorldStateError(
                        "cannot establish checkpoint rejection signal boundary"
                    ) from exc
                commit_started = False
                try:
                    if commit_signals.intersection(previous_mask):
                        raise WorldStateError(
                            "checkpoint rejection commit signals were already blocked"
                        )
                    # A signal generated before pthread_sigmask's atomic boundary
                    # is delivered through _handle_signal before Python resumes.
                    # Recheck while new deliveries are blocked, then publish the
                    # commit state without another interruptible bytecode window.
                    if self._signal_number is not None:
                        raise WorldStateError(
                            "checkpoint rejection canceled by signal "
                            f"{self._signal_number}"
                        )
                    self._commit_started = True
                    commit_started = True
                finally:
                    try:
                        signal.pthread_sigmask(signal.SIG_SETMASK, previous_mask)
                    except (AttributeError, OSError, ValueError) as exc:
                        if not commit_started:
                            raise WorldStateError(
                                "cannot restore checkpoint rejection signal mask"
                            ) from exc
                        # The transaction has crossed its irreversible boundary.
                        # Keeping these signals blocked for this short-lived
                        # helper is safer than aborting between replica writes.
                return
            if self._signal_number is not None:
                raise WorldStateError(
                    f"checkpoint rejection canceled by signal {self._signal_number}"
                )
            if time.monotonic() >= deadline_monotonic:
                raise WorldStateError("checkpoint rejection commit gate timed out")
            time.sleep(
                min(
                    REJECT_COMMIT_GATE_POLL_SECONDS,
                    max(0.0, deadline_monotonic - time.monotonic()),
                )
            )


@contextmanager
def _state_file_lock(path: Path):
    nofollow = getattr(os, "O_NOFOLLOW", None)
    if nofollow is None:
        raise WorldStateError("secure world-state locks require O_NOFOLLOW support")
    directory_flags = _directory_open_flags()
    try:
        parent_fd = os.open("/tmp", directory_flags)
    except OSError as exc:
        raise WorldStateError(f"cannot open world-state lock directory: {exc}") from exc
    descriptor: int | None = None
    try:
        flags = os.O_RDWR | os.O_CREAT | nofollow
        flags |= getattr(os, "O_CLOEXEC", 0)
        path_digest = hashlib.sha256(os.fsencode(path)).hexdigest()
        lock_name = f".matrix-world-state-{os.getuid()}-{path_digest}.lock"
        try:
            descriptor = os.open(lock_name, flags, 0o600, dir_fd=parent_fd)
        except OSError as exc:
            raise WorldStateError(f"cannot open world-state lock for {path}: {exc}") from exc
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode):
            raise WorldStateError(f"world-state lock is not a regular file: {path}")
        if metadata.st_uid != os.getuid():
            raise WorldStateError(f"world-state lock has an unexpected owner: {path}")
        os.fchmod(descriptor, 0o600)
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX)
        except OSError as exc:
            raise WorldStateError(f"cannot lock world state {path}: {exc}") from exc
        yield
    finally:
        if descriptor is not None:
            try:
                fcntl.flock(descriptor, fcntl.LOCK_UN)
            except OSError:
                pass
            os.close(descriptor)
        os.close(parent_fd)


def _serialize_state(state: MatrixWorldState) -> bytes:
    return (
        json.dumps(
            state.to_mapping(),
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")


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
        valid: list[tuple[str, MatrixWorldState]] = []
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
            valid.append((label, state))
        if valid:
            label, state = max(
                valid,
                key=lambda item: (
                    item[1].generation,
                    1 if item[0] == "primary" else 0,
                ),
            )
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
        with _state_file_lock(self.path):
            self._save_unlocked(candidate)

    def _save_unlocked(self, candidate: MatrixWorldState) -> None:
        predecessors: list[tuple[str, MatrixWorldState, bytes]] = []
        for label, path in (("primary", self.path), ("backup", self.backup_path)):
            try:
                payload = _read_regular_file(path)
                state = _decode_state_bytes(payload)
            except WorldStateError:
                continue
            if (
                state.world_id == self.world_id
                and state.world_revision == self.world_revision
            ):
                predecessors.append((label, state, payload))

        predecessor: tuple[str, MatrixWorldState, bytes] | None = None
        if predecessors:
            predecessor = max(
                predecessors,
                key=lambda item: (
                    item[1].generation,
                    1 if item[0] == "primary" else 0,
                ),
            )
        if predecessor is not None and (
            candidate.generation < predecessor[1].generation
            or (
                candidate.generation == predecessor[1].generation
                and candidate != predecessor[1]
            )
        ):
            raise WorldStateError(
                "refusing stale world-state generation overwrite"
            )
        payload = _serialize_state(candidate)
        # Rotate only the selected predecessor.  When backup is newer than
        # primary it is already the durable predecessor and must survive a
        # failed primary write; copying the stale primary over it would revive
        # state that a newer generation had quarantined.
        if predecessor is not None and predecessor[0] == "primary":
            _atomic_write(self.backup_path, predecessor[2])
        _atomic_write(self.path, payload)
        self.state = candidate

    def reject_active_checkpoint(
        self,
        *,
        expected_id: str,
        expected_generation: int,
        reason: str,
        run_id: str,
        now_unix_ns: int | None = None,
        precommit: Callable[[RejectActiveCheckpointResult], None] | None = None,
    ) -> RejectActiveCheckpointResult:
        with _state_file_lock(self.path):
            state = self.load()
            result = state.reject_active_checkpoint(
                expected_id=expected_id,
                expected_generation=expected_generation,
                reason=reason,
                run_id=run_id,
                now_unix_ns=now_unix_ns,
            )
            if precommit is not None:
                if not callable(precommit):
                    raise WorldStateError("checkpoint rejection precommit gate is invalid")
                precommit(result)
            payload = _serialize_state(result.state)
            # A rejection is a quarantine transaction rather than an ordinary
            # previous-version rotation.  Replicate the tombstone to both files
            # so corruption of the primary can never reactivate the rejected
            # candidate through a stale backup.
            _atomic_write(self.backup_path, payload)
            _atomic_write(self.path, payload)
            self.state = result.state
            self.load_status = "loaded"
            self.load_error = None
            return result

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
    creative_inventory_catalog: Path | None = None,
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
            creative_inventory_catalog=creative_inventory_catalog,
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
    digest.update(b"matrix-world-revision/v3\0")
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
    revision.add_argument("--creative-inventory-catalog", type=Path)
    revision.add_argument(
        "--scene-transform",
        choices=(
            SCENE_TRANSFORM_NONE,
            TOWN10_OPEN_BOUNDARY_TRANSFORM,
            MOON_DYNAMIC_GROUND_MOCAP_TRANSFORM,
        ),
        default=SCENE_TRANSFORM_NONE,
    )

    default_path = subparsers.add_parser("default-path")
    default_path.add_argument("--profile", required=True)
    default_path.add_argument("--world-id", required=True)

    resolve = subparsers.add_parser("resolve-start")
    resolve.add_argument("--file", type=Path, required=True)
    resolve.add_argument("--world-id", required=True)
    resolve.add_argument("--world-revision", required=True)
    resolve.add_argument("--include-checkpoint-meta", action="store_true")

    reject = subparsers.add_parser("reject-checkpoint")
    reject.add_argument("--file", type=Path, required=True)
    reject.add_argument("--world-id", required=True)
    reject.add_argument("--world-revision", required=True)
    reject.add_argument("--checkpoint-id", required=True)
    reject.add_argument("--expected-generation", type=int, required=True)
    reject.add_argument("--reason", required=True)
    reject.add_argument("--run-id", required=True)
    reject.add_argument("--commit-ready-file", type=Path)
    reject.add_argument("--commit-authorize-file", type=Path)
    reject.add_argument("--commit-cancel-file", type=Path)
    reject.add_argument("--commit-timeout-seconds", type=float)
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
                    creative_inventory_catalog=args.creative_inventory_catalog,
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
            resolved = state.resolve_start()
            if resolved.pose is None:
                print("none")
            else:
                print("pose")
                print(format(resolved.pose.x, ".17g"))
                print(format(resolved.pose.y, ".17g"))
                print(format(resolved.pose.z, ".17g"))
                print(format(resolved.pose.yaw_rad, ".17g"))
                print(resolved.source)
            print(store.load_status)
            if args.include_checkpoint_meta:
                print(resolved.checkpoint_id or "none")
                print(resolved.generation)
            return 0
        if args.command == "reject-checkpoint":
            store = WorldStateStore(
                args.file,
                world_id=args.world_id,
                world_revision=args.world_revision,
            )
            marker_paths = (
                args.commit_ready_file,
                args.commit_authorize_file,
                args.commit_cancel_file,
            )
            marker_count = sum(path is not None for path in marker_paths)
            if marker_count not in {0, 3}:
                raise WorldStateError(
                    "commit-ready, commit-authorize, and commit-cancel files "
                    "must be supplied together"
                )
            if marker_count == 0 and args.commit_timeout_seconds is not None:
                raise WorldStateError(
                    "commit gate timeout requires all three commit marker files"
                )
            if marker_count == 3:
                assert all(path is not None for path in marker_paths)
                gate = _RejectCheckpointCommitGate(
                    state_path=store.path,
                    backup_path=store.backup_path,
                    ready_path=args.commit_ready_file,
                    authorize_path=args.commit_authorize_file,
                    cancel_path=args.commit_cancel_file,
                    timeout_seconds=(
                        REJECT_COMMIT_GATE_DEFAULT_TIMEOUT_SECONDS
                        if args.commit_timeout_seconds is None
                        else args.commit_timeout_seconds
                    ),
                )
                with gate.signal_handlers():
                    result = store.reject_active_checkpoint(
                        expected_id=args.checkpoint_id,
                        expected_generation=args.expected_generation,
                        reason=args.reason,
                        run_id=args.run_id,
                        precommit=gate.await_authorization,
                    )
            else:
                result = store.reject_active_checkpoint(
                    expected_id=args.checkpoint_id,
                    expected_generation=args.expected_generation,
                    reason=args.reason,
                    run_id=args.run_id,
                )
            replacement = result.replacement_checkpoint
            print(
                json.dumps(
                    {
                        "schema": "matrix-world-state-rejection/v1",
                        "rejected_checkpoint_id": (
                            result.rejected_checkpoint.checkpoint_id
                        ),
                        "replacement_checkpoint_id": (
                            replacement.checkpoint_id
                            if replacement is not None
                            else None
                        ),
                        "generation": result.state.generation,
                        "idempotent": result.idempotent,
                    },
                    sort_keys=True,
                    separators=(",", ":"),
                )
            )
            return 0
    except WorldStateError as exc:
        raise SystemExit(f"[ERROR] {exc}") from exc
    raise SystemExit("[ERROR] unsupported world-state command")


__all__ = [
    "MatrixWorldState",
    "InvalidCheckpoint",
    "MAX_INVALID_CHECKPOINTS",
    "MAX_RESUME_CHECKPOINTS",
    "RejectActiveCheckpointResult",
    "ResolvedStart",
    "ResumeCheckpoint",
    "TeleportPoint",
    "TELEPORT_POINT_TYPE",
    "WorldPose",
    "WorldStateError",
    "WorldStateStore",
    "default_world_state_path",
    "validate_tag",
    "validate_checkpoint_id",
    "validate_world_id",
    "world_revision_for_files",
]


if __name__ == "__main__":
    raise SystemExit(main())
