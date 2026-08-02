#!/usr/bin/env python3
"""MoonWorld rolling-ground loader and MuJoCo mocap updater.

The bundled MoonWorld height map stores absolute world-Z samples on a 0.1 m
grid.  Matrix renders a large Moon surface in UE, while MuJoCo keeps a 16x16
rolling collision window around the robot.  This module keeps those 256 MuJoCo
tile bodies aligned to the locked height map.
"""

from __future__ import annotations

import argparse
import hashlib
import math
import mmap
import os
from pathlib import Path
import re
import stat
from typing import Any, Iterable

import numpy as np


MAP_SIDE_SAMPLES = 6000
MAP_SAMPLE_COUNT = MAP_SIDE_SAMPLES * MAP_SIDE_SAMPLES
MAP_DTYPE = np.dtype("<f4")
MAP_SIZE_BYTES = MAP_SAMPLE_COUNT * MAP_DTYPE.itemsize
MAP_RESOLUTION_M = 0.1
MAP_HALF_CELL_M = 0.05
MAP_HALF_EXTENT_M = 300.0

TILE_SIDE_COUNT = 16
TILE_COUNT = TILE_SIDE_COUNT * TILE_SIDE_COUNT
TILE_INDEX_ORIGIN = -(TILE_SIDE_COUNT // 2)
TILE_BODY_PATTERN = re.compile(r"gb_([0-9]+)_([0-9]+)\Z")

CONTINUOUS_SUPPORT_ASSET_NAME = "matrix_moon_continuous_support_hfield"
CONTINUOUS_SUPPORT_GEOM_NAME = "matrix_moon_continuous_support"
SPAWN_PAD_GEOM_NAME = "matrix_moon_spawn_pad"
CONTINUOUS_SUPPORT_GRID_SIDE_SAMPLES = 33
CONTINUOUS_SUPPORT_SAMPLE_COUNT = (
    CONTINUOUS_SUPPORT_GRID_SIDE_SAMPLES * CONTINUOUS_SUPPORT_GRID_SIDE_SAMPLES
)
CONTINUOUS_SUPPORT_HALF_EXTENT_M = 1.6
CONTINUOUS_SUPPORT_HEIGHT_RANGE_M = 64.0
CONTINUOUS_SUPPORT_BASE_DEPTH_M = 1.0

COLLISION_MODE_ROLLING_TILES = "rolling-mocap-tiles-v1"
COLLISION_MODE_ROLLING_HFIELD = "rolling-heightfield-v2"
DEFAULT_COLLISION_MODE = COLLISION_MODE_ROLLING_HFIELD
LOCKED_MOONWORLD_SHA256 = (
    "62e624b5feca0111033c60d0e820f3a320257acd72b565234ac79c704dbca1df"
)
TELEMETRY_SCHEMA = "matrix-moon-dynamic-ground/v2"
DEFAULT_SPAWN_X_M = -94.7
DEFAULT_SPAWN_Y_M = -65.6
DEFAULT_SPAWN_PAD_TOP_Z_M = -6.101562023162842
DEFAULT_ROOT_CLEARANCE_M = 0.85
DEFAULT_SPAWN_Z_M = DEFAULT_SPAWN_PAD_TOP_Z_M + DEFAULT_ROOT_CLEARANCE_M
DEFAULT_SPAWN_YAW_RAD = 0.0
DEFAULT_MIN_RESUME_CLEARANCE_M = 0.45
DEFAULT_MAX_RESUME_CLEARANCE_M = 1.30
_EXPECTED_TILE_KEYS = tuple(
    (i, j)
    for i in range(TILE_SIDE_COUNT)
    for j in range(TILE_SIDE_COUNT)
)
_EXPECTED_TILE_KEY_SET = frozenset(_EXPECTED_TILE_KEYS)
_SHA256_PATTERN = re.compile(r"[0-9a-f]{64}\Z")


class MoonDynamicGroundError(RuntimeError):
    """The MoonWorld height map or mocap model contract is invalid."""


def _finite_float(value: object, *, label: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise MoonDynamicGroundError(f"{label} must be numeric") from exc
    if not math.isfinite(number):
        raise MoonDynamicGroundError(f"{label} must be finite")
    return number


def round_away_from_zero(value: object) -> int:
    number = _finite_float(value, label="round input")
    if number >= 0.0:
        return math.trunc(number + 0.5)
    return math.trunc(number - 0.5)


def native_quantize(value: object) -> float:
    number = _finite_float(value, label="quantize input")
    scaled = (number - MAP_HALF_CELL_M) / MAP_RESOLUTION_M
    return round_away_from_zero(scaled) * MAP_RESOLUTION_M + MAP_HALF_CELL_M


def normalize_height_filter(value: object | None = None) -> str:
    raw = (
        os.environ.get("MATRIX_MOON_DYNAMIC_GROUND_HEIGHT_FILTER")
        if value is None
        else value
    )
    text = str(raw or "raw").strip().lower().replace("_", "-")
    if text in {"", "raw"}:
        return "raw"
    if text in {"flat", "flat-anchor", "anchor", "stable", "default"}:
        return "flat-anchor"
    if text in {"flat-local", "local"}:
        return "flat-local"
    raise MoonDynamicGroundError(
        "MATRIX_MOON_DYNAMIC_GROUND_HEIGHT_FILTER must be raw, "
        "flat-local, or flat-anchor"
    )


def normalize_collision_mode(value: object | None = None) -> str:
    raw = (
        os.environ.get("MATRIX_MOON_DYNAMIC_GROUND_COLLISION_MODE")
        if value is None
        else value
    )
    text = str(raw or DEFAULT_COLLISION_MODE).strip().lower().replace("_", "-")
    if text in {
        "",
        "stable",
        "default",
        "tiles",
        "tile",
        "mocap-tiles",
        "rolling-tiles",
        "rolling-mocap-tiles",
        COLLISION_MODE_ROLLING_TILES,
        "leo",
        "official",
    }:
        return COLLISION_MODE_ROLLING_TILES
    if text in {
        "hfield",
        "heightfield",
        "continuous",
        "continuous-hfield",
        "rolling-hfield",
        "rolling-heightfield",
        COLLISION_MODE_ROLLING_HFIELD,
    }:
        return COLLISION_MODE_ROLLING_HFIELD
    raise MoonDynamicGroundError(
        "MATRIX_MOON_DYNAMIC_GROUND_COLLISION_MODE must be "
        f"{COLLISION_MODE_ROLLING_HFIELD} or {COLLISION_MODE_ROLLING_TILES}"
    )


def _sample_raw_height_from_array(
    heights: np.ndarray,
    x_m: object,
    y_m: object,
) -> float:
    x = _finite_float(x_m, label="world x")
    y = _finite_float(y_m, label="world y")
    fractional_x = min(
        max((x + MAP_HALF_EXTENT_M) / MAP_RESOLUTION_M, 0.0),
        MAP_SIDE_SAMPLES - 1.0,
    )
    fractional_y = min(
        max((y + MAP_HALF_EXTENT_M) / MAP_RESOLUTION_M, 0.0),
        MAP_SIDE_SAMPLES - 1.0,
    )
    x0 = int(math.floor(fractional_x))
    y0 = int(math.floor(fractional_y))
    x1 = min(x0 + 1, MAP_SIDE_SAMPLES - 1)
    y1 = min(y0 + 1, MAP_SIDE_SAMPLES - 1)
    weight_x = fractional_x - x0
    weight_y = fractional_y - y0
    height_00 = float(heights[y0, x0])
    height_10 = float(heights[y0, x1])
    height_01 = float(heights[y1, x0])
    height_11 = float(heights[y1, x1])
    if weight_x >= weight_y:
        height = (
            (1.0 - weight_x) * height_00
            + (weight_x - weight_y) * height_10
            + weight_y * height_11
        )
    else:
        height = (
            (1.0 - weight_y) * height_00
            + (weight_y - weight_x) * height_01
            + weight_x * height_11
        )
    if not math.isfinite(height):
        raise MoonDynamicGroundError("MoonWorld sampled a non-finite height")
    return height


def sample_raw_height_from_map(
    path: str | os.PathLike[str],
    x_m: object,
    y_m: object,
    *,
    expected_sha256: str | None = None,
) -> float:
    map_path = Path(path).expanduser().resolve()
    if expected_sha256 is not None and (
        not isinstance(expected_sha256, str)
        or _SHA256_PATTERN.fullmatch(expected_sha256) is None
    ):
        raise MoonDynamicGroundError(
            "expected_sha256 must be 64 lowercase hexadecimal characters"
        )
    stream = None
    mapped = None
    try:
        stream = map_path.open("rb")
        metadata = os.fstat(stream.fileno())
        if not stat.S_ISREG(metadata.st_mode):
            raise MoonDynamicGroundError(
                f"MoonWorld height map is not a regular file: {map_path}"
            )
        if metadata.st_size != MAP_SIZE_BYTES:
            raise MoonDynamicGroundError(
                "MoonWorld height map size mismatch: "
                f"expected={MAP_SIZE_BYTES} actual={metadata.st_size}"
            )
        mapped = mmap.mmap(stream.fileno(), MAP_SIZE_BYTES, access=mmap.ACCESS_READ)
        if expected_sha256 is not None:
            actual_sha256 = hashlib.sha256(mapped).hexdigest()
            if actual_sha256 != expected_sha256:
                raise MoonDynamicGroundError(
                    "MoonWorld height map SHA256 mismatch: "
                    f"expected={expected_sha256} actual={actual_sha256}"
                )
        heights = np.ndarray(
            shape=(MAP_SIDE_SAMPLES, MAP_SIDE_SAMPLES),
            dtype=MAP_DTYPE,
            buffer=mapped,
            order="C",
        )
        return _sample_raw_height_from_array(heights, x_m, y_m)
    except MoonDynamicGroundError:
        raise
    except (OSError, ValueError) as exc:
        raise MoonDynamicGroundError(
            f"cannot open MoonWorld height map {map_path}: {exc}"
        ) from exc
    finally:
        if mapped is not None:
            mapped.close()
        if stream is not None:
            stream.close()


def resolve_spawn_pose_for_moon_dynamic_ground(
    *,
    map_path: str | os.PathLike[str],
    expected_sha256: str | None = None,
    pose_xyz: tuple[object, object, object] | None = None,
    yaw_rad: object | None = None,
    source: str = "map_default",
    fallback_xyz: tuple[object, object, object] = (
        DEFAULT_SPAWN_X_M,
        DEFAULT_SPAWN_Y_M,
        DEFAULT_SPAWN_Z_M,
    ),
    fallback_yaw_rad: object = DEFAULT_SPAWN_YAW_RAD,
    root_clearance_m: object = DEFAULT_ROOT_CLEARANCE_M,
    min_resume_clearance_m: object = DEFAULT_MIN_RESUME_CLEARANCE_M,
    max_resume_clearance_m: object = DEFAULT_MAX_RESUME_CLEARANCE_M,
) -> dict[str, object]:
    """Return a MoonWorld spawn pose that is safe for raw dynamic ground.

    Older flat-anchor or cross-world resumes may have a root z that is far away
    from the locked MoonWorld height map.  Those poses can cause a large drop or
    immediate collision impulse when the startup pad hands off to raw ground.
    Valid MoonWorld resumes keep their x/y/yaw but rebase z to the current raw
    terrain height plus the configured upright root clearance.
    """

    root_clearance = _finite_float(root_clearance_m, label="root clearance")
    min_clearance = _finite_float(
        min_resume_clearance_m,
        label="minimum resume clearance",
    )
    max_clearance = _finite_float(
        max_resume_clearance_m,
        label="maximum resume clearance",
    )
    if root_clearance <= 0.0:
        raise MoonDynamicGroundError("root clearance must be positive")
    if min_clearance <= 0.0 or max_clearance <= 0.0 or min_clearance > max_clearance:
        raise MoonDynamicGroundError("resume clearance bounds are invalid")

    fallback_x = _finite_float(fallback_xyz[0], label="fallback x")
    fallback_y = _finite_float(fallback_xyz[1], label="fallback y")
    fallback_z = _finite_float(fallback_xyz[2], label="fallback z")
    fallback_yaw = _finite_float(fallback_yaw_rad, label="fallback yaw")

    if pose_xyz is None:
        return {
            "x": fallback_x,
            "y": fallback_y,
            "z": fallback_z,
            "yaw_rad": fallback_yaw,
            "source": "moon_map_default",
            "input_source": source,
            "raw_ground_height_m": None,
            "input_clearance_m": None,
        }

    x = _finite_float(pose_xyz[0], label="pose x")
    y = _finite_float(pose_xyz[1], label="pose y")
    z = _finite_float(pose_xyz[2], label="pose z")
    yaw = (
        _finite_float(yaw_rad, label="pose yaw")
        if yaw_rad is not None
        else fallback_yaw
    )
    raw_ground_height = sample_raw_height_from_map(
        map_path,
        x,
        y,
        expected_sha256=expected_sha256,
    )
    input_clearance = z - raw_ground_height
    if min_clearance <= input_clearance <= max_clearance:
        return {
            "x": x,
            "y": y,
            "z": raw_ground_height + root_clearance,
            "yaw_rad": yaw,
            "source": f"moon_terrain_rebased_{source}",
            "input_source": source,
            "raw_ground_height_m": raw_ground_height,
            "input_clearance_m": input_clearance,
        }
    return {
        "x": fallback_x,
        "y": fallback_y,
        "z": fallback_z,
        "yaw_rad": fallback_yaw,
        "source": f"moon_rejected_{source}_clearance",
        "input_source": source,
        "raw_ground_height_m": raw_ground_height,
        "input_clearance_m": input_clearance,
    }


def _round_array_away_from_zero(values: np.ndarray) -> np.ndarray:
    shifts = np.where(values >= 0.0, 0.5, -0.5)
    return np.trunc(values + shifts).astype(np.int64)


def _model_name(model: Any, kind: str, item_id: int) -> str | None:
    try:
        name = getattr(model, kind)(item_id).name
    except (AttributeError, IndexError, KeyError, TypeError, ValueError) as exc:
        raise MoonDynamicGroundError(
            f"cannot inspect MuJoCo {kind} id {item_id}"
        ) from exc
    if isinstance(name, bytes):
        return name.decode("utf-8")
    if name is None or isinstance(name, str):
        return name
    raise MoonDynamicGroundError(
        f"MuJoCo {kind} id {item_id} has a non-string name"
    )


def _unique_named_model_id(model: Any, kind: str, name: str, *, count: int) -> int:
    matches: list[int] = []
    for item_id in range(count):
        if _model_name(model, kind, item_id) == name:
            matches.append(item_id)
    if not matches:
        raise MoonDynamicGroundError(f"MuJoCo model is missing {kind} {name!r}")
    if len(matches) != 1:
        raise MoonDynamicGroundError(f"MuJoCo {kind} {name!r} must be unique")
    return matches[0]


def resolve_tile_mocap_ids(model: Any) -> np.ndarray:
    try:
        nbody = int(model.nbody)
        nmocap = int(model.nmocap)
        body_mocapid = model.body_mocapid
    except (AttributeError, TypeError, ValueError) as exc:
        raise MoonDynamicGroundError(
            "MuJoCo model is missing nbody/nmocap/body_mocapid metadata"
        ) from exc
    if nbody <= 0 or nmocap < TILE_COUNT:
        raise MoonDynamicGroundError(
            "MuJoCo model cannot provide 256 MoonWorld mocap bodies: "
            f"nbody={nbody} nmocap={nmocap}"
        )

    body_id_by_key: dict[tuple[int, int], int] = {}
    malformed: list[str] = []
    for body_id in range(nbody):
        name = _model_name(model, "body", body_id)
        if name is None or not name.startswith("gb_"):
            continue
        match = TILE_BODY_PATTERN.fullmatch(name)
        if match is None:
            malformed.append(name)
            continue
        key = (int(match.group(1)), int(match.group(2)))
        if key in body_id_by_key:
            raise MoonDynamicGroundError(
                f"duplicate MoonWorld tile body coordinates: {name}"
            )
        body_id_by_key[key] = body_id

    actual_keys = frozenset(body_id_by_key)
    missing = sorted(_EXPECTED_TILE_KEY_SET - actual_keys)
    unexpected = sorted(actual_keys - _EXPECTED_TILE_KEY_SET)
    if malformed or missing or unexpected:
        raise MoonDynamicGroundError(
            "MoonWorld tile body set drifted: "
            f"missing={missing[:8]} unexpected={unexpected[:8]} "
            f"malformed={sorted(malformed)[:8]}"
        )

    try:
        mocap_ids = np.asarray(
            [int(body_mocapid[body_id_by_key[key]]) for key in _EXPECTED_TILE_KEYS],
            dtype=np.int64,
        )
    except (IndexError, TypeError, ValueError) as exc:
        raise MoonDynamicGroundError(
            "MuJoCo body_mocapid metadata is unavailable or truncated"
        ) from exc
    invalid = mocap_ids[(mocap_ids < 0) | (mocap_ids >= nmocap)]
    if invalid.size:
        raise MoonDynamicGroundError(
            "MoonWorld tile body is not a valid mocap body: "
            f"mocap_ids={invalid[:8].tolist()} nmocap={nmocap}"
        )
    if np.unique(mocap_ids).size != TILE_COUNT:
        raise MoonDynamicGroundError(
            "MoonWorld tile bodies do not map one-to-one to mocap ids"
        )
    mocap_ids.setflags(write=False)
    return mocap_ids


class MoonDynamicGround:
    """Read-only height map plus rolling 16x16 MuJoCo mocap tiles."""

    def __init__(
        self,
        path: str | os.PathLike[str],
        model: Any,
        *,
        expected_sha256: str | None = None,
    ) -> None:
        self.path = Path(path).expanduser().resolve()
        if expected_sha256 is not None and (
            not isinstance(expected_sha256, str)
            or _SHA256_PATTERN.fullmatch(expected_sha256) is None
        ):
            raise MoonDynamicGroundError(
                "expected_sha256 must be 64 lowercase hexadecimal characters"
            )
        self.expected_sha256 = expected_sha256
        self.height_filter = normalize_height_filter()
        self.collision_mode = normalize_collision_mode()
        self.actual_sha256: str | None = None
        self.file_size_bytes: int | None = None
        self.file_inode: int | None = None
        self.file_mtime_ns: int | None = None
        self.minimum_height_m: float | None = None
        self.maximum_height_m: float | None = None
        self._stream: Any | None = None
        self._mapped: mmap.mmap | None = None
        self._heights: np.ndarray | None = None
        self._closed = False
        self._update_count = 0
        self._tile_update_count = 0
        self._cache_hit_count = 0
        self._cache_invalidation_count = 0
        self._last_update: dict[str, object] | None = None
        self._cached_data: Any | None = None
        self._cached_quantized_base_xy: tuple[float, float] | None = None
        self._filtered_height_m: float | None = None
        self._filtered_center_xy_m: tuple[float, float] | None = None
        self._collision_handoff_active = False
        self._model = model

        tile_i = np.repeat(np.arange(TILE_SIDE_COUNT, dtype=np.float64), TILE_SIDE_COUNT)
        tile_j = np.tile(np.arange(TILE_SIDE_COUNT, dtype=np.float64), TILE_SIDE_COUNT)
        self._tile_x_offsets = (
            (tile_i + TILE_INDEX_ORIGIN) * MAP_RESOLUTION_M + MAP_HALF_CELL_M
        )
        self._tile_y_offsets = (
            (tile_j + TILE_INDEX_ORIGIN) * MAP_RESOLUTION_M + MAP_HALF_CELL_M
        )
        self._positions = np.empty((TILE_COUNT, 3), dtype=np.float64)
        self._identity_quaternions = np.zeros((TILE_COUNT, 4), dtype=np.float64)
        self._identity_quaternions[:, 0] = 1.0
        self._support_axis_offsets = np.linspace(
            -CONTINUOUS_SUPPORT_HALF_EXTENT_M,
            CONTINUOUS_SUPPORT_HALF_EXTENT_M,
            CONTINUOUS_SUPPORT_GRID_SIDE_SAMPLES,
            dtype=np.float64,
        )
        self._support_values = np.empty(CONTINUOUS_SUPPORT_SAMPLE_COUNT, dtype=np.float32)
        self._support_position = np.empty(3, dtype=np.float64)

        try:
            self._open_and_validate_map()
            self.mocap_ids = resolve_tile_mocap_ids(model)
            self.support_geom_id = _unique_named_model_id(
                model, "geom", CONTINUOUS_SUPPORT_GEOM_NAME, count=int(model.ngeom)
            )
            self.spawn_pad_geom_id = _unique_named_model_id(
                model, "geom", SPAWN_PAD_GEOM_NAME, count=int(model.ngeom)
            )
            self.support_hfield_id = _unique_named_model_id(
                model,
                "hfield",
                CONTINUOUS_SUPPORT_ASSET_NAME,
                count=int(model.nhfield),
            )
            self.support_data_adr = int(model.hfield_adr[self.support_hfield_id])
            self.tile_geom_ids = tuple(
                _unique_named_model_id(model, "geom", f"soil_{i}_{j}", count=int(model.ngeom))
                for i, j in _EXPECTED_TILE_KEYS
            )
            # Spawn starts on the finite pad.  Runtime enables either the
            # rolling boxes or hfield in one explicit handoff after the first
            # mocap update.
            model.geom_contype[self.support_geom_id] = 0
            model.geom_conaffinity[self.support_geom_id] = 0
            model.geom_contype[list(self.tile_geom_ids)] = 0
            model.geom_conaffinity[list(self.tile_geom_ids)] = 0
            model.geom_contype[self.spawn_pad_geom_id] = 1
            model.geom_conaffinity[self.spawn_pad_geom_id] = 1
        except Exception:
            self.close()
            raise

    @property
    def collision_handoff_active(self) -> bool:
        return self._collision_handoff_active

    def _open_and_validate_map(self) -> None:
        stream = None
        mapped = None
        try:
            stream = self.path.open("rb")
            metadata = os.fstat(stream.fileno())
            if not stat.S_ISREG(metadata.st_mode):
                raise MoonDynamicGroundError(
                    f"MoonWorld height map is not a regular file: {self.path}"
                )
            if metadata.st_size != MAP_SIZE_BYTES:
                raise MoonDynamicGroundError(
                    "MoonWorld height map size mismatch: "
                    f"expected={MAP_SIZE_BYTES} actual={metadata.st_size}"
                )
            mapped = mmap.mmap(stream.fileno(), MAP_SIZE_BYTES, access=mmap.ACCESS_READ)
            actual_sha256 = hashlib.sha256(mapped).hexdigest()
            if self.expected_sha256 is not None and actual_sha256 != self.expected_sha256:
                raise MoonDynamicGroundError(
                    "MoonWorld height map SHA256 mismatch: "
                    f"expected={self.expected_sha256} actual={actual_sha256}"
                )
            heights = np.ndarray(
                shape=(MAP_SIDE_SAMPLES, MAP_SIDE_SAMPLES),
                dtype=MAP_DTYPE,
                buffer=mapped,
                order="C",
            )
            # Full finite scan is intentionally strict; this file is the source
            # of truth for both collision and fall-height checks.
            finite = np.isfinite(heights)
            if not bool(np.all(finite)):
                row, column = np.argwhere(~finite)[0].tolist()
                raise MoonDynamicGroundError(
                    "MoonWorld height map contains a non-finite sample: "
                    f"row={row} column={column}"
                )
            self.actual_sha256 = actual_sha256
            self.file_size_bytes = int(metadata.st_size)
            self.file_inode = int(metadata.st_ino)
            self.file_mtime_ns = int(metadata.st_mtime_ns)
            self.minimum_height_m = float(np.min(heights))
            self.maximum_height_m = float(np.max(heights))
            self._stream = stream
            self._mapped = mapped
            self._heights = heights
        except MoonDynamicGroundError:
            if mapped is not None:
                mapped.close()
            if stream is not None:
                stream.close()
            raise
        except (OSError, ValueError) as exc:
            if mapped is not None:
                mapped.close()
            if stream is not None:
                stream.close()
            raise MoonDynamicGroundError(
                f"cannot open MoonWorld height map {self.path}: {exc}"
            ) from exc

    def _require_open_heights(self) -> np.ndarray:
        if self._closed or self._heights is None:
            raise MoonDynamicGroundError("MoonWorld dynamic ground is closed")
        return self._heights

    def _sample_raw_height(self, x_m: object, y_m: object) -> float:
        heights = self._require_open_heights()
        return _sample_raw_height_from_array(heights, x_m, y_m)

    def sample_height(self, x_m: object, y_m: object) -> float:
        if (
            self.height_filter in {"flat-local", "flat-anchor"}
            and self._filtered_height_m is not None
        ):
            return float(self._filtered_height_m)
        return self._sample_raw_height(x_m, y_m)

    def update_mocap(
        self,
        data: Any,
        *,
        base_xy: Iterable[object] | None = None,
    ) -> dict[str, object]:
        heights = self._require_open_heights()
        if base_xy is None:
            try:
                base_values = data.qpos[:2]
            except (AttributeError, IndexError, TypeError) as exc:
                raise MoonDynamicGroundError(
                    "MuJoCo data does not expose root qpos x/y"
                ) from exc
        else:
            try:
                base_values = tuple(base_xy)
            except TypeError as exc:
                raise MoonDynamicGroundError(
                    "base_xy must be an iterable of two coordinates"
                ) from exc
        if len(base_values) != 2:
            raise MoonDynamicGroundError("base_xy must contain exactly two values")

        base_x = _finite_float(base_values[0], label="base x")
        base_y = _finite_float(base_values[1], label="base y")
        quantized_x = native_quantize(base_x)
        quantized_y = native_quantize(base_y)
        quantized_base_xy = (quantized_x, quantized_y)
        raw_local_ground_height_m = self._sample_raw_height(base_x, base_y)
        local_ground_height_m = raw_local_ground_height_m
        if self.height_filter == "flat-local":
            self._filtered_height_m = float(raw_local_ground_height_m)
            self._filtered_center_xy_m = (base_x, base_y)
            local_ground_height_m = float(self._filtered_height_m)
        elif self.height_filter == "flat-anchor":
            if self._filtered_height_m is None:
                self._filtered_height_m = float(raw_local_ground_height_m)
                self._filtered_center_xy_m = (base_x, base_y)
            local_ground_height_m = float(self._filtered_height_m)

        if (
            data is self._cached_data
            and quantized_base_xy == self._cached_quantized_base_xy
        ):
            self._update_count += 1
            self._cache_hit_count += 1
            assert self._last_update is not None
            self._last_update = {
                **self._last_update,
                "base_xy_m": [base_x, base_y],
                "local_ground_height_m": local_ground_height_m,
                "raw_local_ground_height_m": raw_local_ground_height_m,
                "cache_hit": True,
                "tiles_updated": False,
            }
            return dict(self._last_update)

        tile_x = quantized_x + self._tile_x_offsets
        tile_y = quantized_y + self._tile_y_offsets
        pixel_x = _round_array_away_from_zero(
            (tile_x + MAP_HALF_EXTENT_M) / MAP_RESOLUTION_M
        )
        pixel_y = _round_array_away_from_zero(
            (tile_y + MAP_HALF_EXTENT_M) / MAP_RESOLUTION_M
        )
        np.clip(pixel_x, 0, MAP_SIDE_SAMPLES - 1, out=pixel_x)
        np.clip(pixel_y, 0, MAP_SIDE_SAMPLES - 1, out=pixel_y)
        tile_z = heights[pixel_y, pixel_x]
        if not bool(np.all(np.isfinite(tile_z))):
            raise MoonDynamicGroundError(
                "MoonWorld rolling tile update sampled a non-finite height"
            )
        raw_tile_z = np.asarray(tile_z, dtype=np.float64)
        if self.height_filter in {"flat-local", "flat-anchor"}:
            tile_z = np.full_like(tile_z, local_ground_height_m)

        support_center_x = quantized_x + MAP_HALF_CELL_M
        support_center_y = quantized_y + MAP_HALF_CELL_M
        support_x = support_center_x + self._support_axis_offsets
        support_y = support_center_y + self._support_axis_offsets
        support_pixel_x = _round_array_away_from_zero(
            (support_x + MAP_HALF_EXTENT_M) / MAP_RESOLUTION_M
        )
        support_pixel_y = _round_array_away_from_zero(
            (support_y + MAP_HALF_EXTENT_M) / MAP_RESOLUTION_M
        )
        np.clip(support_pixel_x, 0, MAP_SIDE_SAMPLES - 1, out=support_pixel_x)
        np.clip(support_pixel_y, 0, MAP_SIDE_SAMPLES - 1, out=support_pixel_y)
        support_z = heights[np.ix_(support_pixel_y, support_pixel_x)]
        if self.height_filter in {"flat-local", "flat-anchor"}:
            support_z = np.full_like(support_z, local_ground_height_m)
        assert self.minimum_height_m is not None
        normalized_support = (
            np.asarray(support_z, dtype=np.float64) - self.minimum_height_m
        ) / CONTINUOUS_SUPPORT_HEIGHT_RANGE_M

        try:
            mocap_pos = data.mocap_pos
            mocap_quat = data.mocap_quat
            hfield_data = self._model.hfield_data
            geom_pos = self._model.geom_pos
            self._positions[:, 0] = tile_x
            self._positions[:, 1] = tile_y
            self._positions[:, 2] = tile_z
            self._support_values[:] = normalized_support.reshape(-1)
            self._support_position[:] = (
                support_center_x,
                support_center_y,
                self.minimum_height_m,
            )
            mocap_pos[self.mocap_ids, :] = self._positions
            mocap_quat[self.mocap_ids, :] = self._identity_quaternions
            hfield_data[
                self.support_data_adr : (
                    self.support_data_adr + CONTINUOUS_SUPPORT_SAMPLE_COUNT
                )
            ] = self._support_values
            geom_pos[self.support_geom_id, :] = self._support_position
        except (AttributeError, IndexError, TypeError, ValueError) as exc:
            raise MoonDynamicGroundError(
                f"cannot write MoonWorld mocap poses: {exc}"
            ) from exc

        self._cached_data = data
        self._cached_quantized_base_xy = quantized_base_xy
        self._update_count += 1
        self._tile_update_count += 1
        self._last_update = {
            "base_xy_m": [base_x, base_y],
            "quantized_base_xy_m": [quantized_x, quantized_y],
            "local_ground_height_m": local_ground_height_m,
            "raw_local_ground_height_m": raw_local_ground_height_m,
            "height_filter": self.height_filter,
            "filtered_center_xy_m": (
                list(self._filtered_center_xy_m)
                if self._filtered_center_xy_m is not None
                else None
            ),
            "pixel_x_range": [int(np.min(pixel_x)), int(np.max(pixel_x))],
            "pixel_y_range": [int(np.min(pixel_y)), int(np.max(pixel_y))],
            "height_range_m": [float(np.min(tile_z)), float(np.max(tile_z))],
            "raw_height_range_m": [
                float(np.min(raw_tile_z)),
                float(np.max(raw_tile_z)),
            ],
            "support_pixel_x_range": [
                int(np.min(support_pixel_x)),
                int(np.max(support_pixel_x)),
            ],
            "support_pixel_y_range": [
                int(np.min(support_pixel_y)),
                int(np.max(support_pixel_y)),
            ],
            "support_height_range_m": [
                float(np.min(support_z)),
                float(np.max(support_z)),
            ],
            "cache_hit": False,
            "tiles_updated": True,
        }
        return dict(self._last_update)

    def activate_collision_handoff(self, data: Any, *, forward: Any) -> dict[str, object]:
        if self._collision_handoff_active:
            return {
                "active": True,
                "ground_mode": self.collision_mode,
                "already_active": True,
            }
        if self._update_count <= 0:
            raise MoonDynamicGroundError(
                "MoonWorld collision handoff requires a populated rolling grid"
            )
        if not callable(forward):
            raise MoonDynamicGroundError(
                "MoonWorld collision handoff requires a callable forward"
            )
        active_support_mask = (
            (1, 1)
            if self.collision_mode == COLLISION_MODE_ROLLING_HFIELD
            else (0, 0)
        )
        active_tile_mask = (
            (0, 0)
            if self.collision_mode == COLLISION_MODE_ROLLING_HFIELD
            else (1, 1)
        )
        try:
            self._model.geom_contype[self.support_geom_id] = active_support_mask[0]
            self._model.geom_conaffinity[self.support_geom_id] = active_support_mask[1]
            self._model.geom_contype[list(self.tile_geom_ids)] = active_tile_mask[0]
            self._model.geom_conaffinity[list(self.tile_geom_ids)] = active_tile_mask[1]
            self._model.geom_contype[self.spawn_pad_geom_id] = 0
            self._model.geom_conaffinity[self.spawn_pad_geom_id] = 0
            forward(self._model, data)
        except (AttributeError, IndexError, TypeError, ValueError) as exc:
            raise MoonDynamicGroundError(
                f"MoonWorld collision handoff failed: {exc}"
            ) from exc
        self._collision_handoff_active = True
        return {
            "active": True,
            "ground_mode": self.collision_mode,
            "support_geom_id": self.support_geom_id,
            "support_collision_mask": list(active_support_mask),
            "tile_geom_count": len(self.tile_geom_ids),
            "tile_collision_mask": list(active_tile_mask),
            "spawn_pad_geom_id": self.spawn_pad_geom_id,
            "spawn_pad_collision_mask": [0, 0],
        }

    def telemetry(self) -> dict[str, object]:
        return {
            "schema": TELEMETRY_SCHEMA,
            "closed": self._closed,
            "height_filter": self.height_filter,
            "collision_mode": self.collision_mode,
            "filtered_height_m": self._filtered_height_m,
            "filtered_center_xy_m": (
                list(self._filtered_center_xy_m)
                if self._filtered_center_xy_m is not None
                else None
            ),
            "map": {
                "path": str(self.path),
                "size_bytes": self.file_size_bytes,
                "sha256": self.actual_sha256,
                "expected_sha256": self.expected_sha256,
                "shape": [MAP_SIDE_SAMPLES, MAP_SIDE_SAMPLES],
                "dtype": "little-endian-float32",
                "resolution_m": MAP_RESOLUTION_M,
                "half_extent_m": MAP_HALF_EXTENT_M,
                "minimum_height_m": self.minimum_height_m,
                "maximum_height_m": self.maximum_height_m,
                "inode": self.file_inode,
                "mtime_ns": self.file_mtime_ns,
                "storage": "read-only-mmap",
            },
            "tiles": {
                "count": TILE_COUNT,
                "side_count": TILE_SIDE_COUNT,
                "unique_mocap_ids": int(np.unique(self.mocap_ids).size),
            },
            "continuous_support": {
                "geom_name": CONTINUOUS_SUPPORT_GEOM_NAME,
                "asset_name": CONTINUOUS_SUPPORT_ASSET_NAME,
                "geom_id": getattr(self, "support_geom_id", None),
                "hfield_id": getattr(self, "support_hfield_id", None),
                "grid_shape": [
                    CONTINUOUS_SUPPORT_GRID_SIDE_SAMPLES,
                    CONTINUOUS_SUPPORT_GRID_SIDE_SAMPLES,
                ],
                "half_extent_m": CONTINUOUS_SUPPORT_HALF_EXTENT_M,
            },
            "collision_handoff": {
                "active": self._collision_handoff_active,
                "contract": (
                    "spawn-pad-to-rolling-heightfield-v2"
                    if self.collision_mode == COLLISION_MODE_ROLLING_HFIELD
                    else "spawn-pad-to-rolling-mocap-tiles-v1"
                ),
                "spawn_pad_geom_name": SPAWN_PAD_GEOM_NAME,
                "spawn_pad_geom_id": getattr(self, "spawn_pad_geom_id", None),
            },
            "update_count": self._update_count,
            "tile_update_count": self._tile_update_count,
            "cache_hit_count": self._cache_hit_count,
            "cache_invalidation_count": self._cache_invalidation_count,
            "last_update": (
                dict(self._last_update)
                if self._last_update is not None
                else None
            ),
        }

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        heights = self._heights
        self._heights = None
        if heights is not None:
            del heights
        if self._mapped is not None:
            self._mapped.close()
            self._mapped = None
        if self._stream is not None:
            self._stream.close()
            self._stream = None

    def __enter__(self) -> "MoonDynamicGround":
        self._require_open_heights()
        return self

    def __exit__(self, _exc_type, _exc, _traceback) -> None:
        self.close()


def _parse_cli_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    sample = subparsers.add_parser("sample-height")
    sample.add_argument("--map", type=Path, required=True)
    sample.add_argument("--map-sha256")
    sample.add_argument("--x", type=float, required=True)
    sample.add_argument("--y", type=float, required=True)

    spawn = subparsers.add_parser("resolve-spawn-pose")
    spawn.add_argument("--map", type=Path, required=True)
    spawn.add_argument("--map-sha256")
    spawn.add_argument("--x", type=float)
    spawn.add_argument("--y", type=float)
    spawn.add_argument("--z", type=float)
    spawn.add_argument("--yaw", type=float)
    spawn.add_argument("--source", default="map_default")
    spawn.add_argument("--fallback-x", type=float, default=DEFAULT_SPAWN_X_M)
    spawn.add_argument("--fallback-y", type=float, default=DEFAULT_SPAWN_Y_M)
    spawn.add_argument("--fallback-z", type=float, default=DEFAULT_SPAWN_Z_M)
    spawn.add_argument("--fallback-yaw", type=float, default=DEFAULT_SPAWN_YAW_RAD)
    spawn.add_argument(
        "--root-clearance",
        type=float,
        default=DEFAULT_ROOT_CLEARANCE_M,
    )
    spawn.add_argument(
        "--min-resume-clearance",
        type=float,
        default=DEFAULT_MIN_RESUME_CLEARANCE_M,
    )
    spawn.add_argument(
        "--max-resume-clearance",
        type=float,
        default=DEFAULT_MAX_RESUME_CLEARANCE_M,
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_cli_args()
    try:
        if args.command == "sample-height":
            print(
                format(
                    sample_raw_height_from_map(
                        args.map,
                        args.x,
                        args.y,
                        expected_sha256=args.map_sha256,
                    ),
                    ".17g",
                )
            )
            return 0
        if args.command == "resolve-spawn-pose":
            supplied = [args.x is not None, args.y is not None, args.z is not None]
            if any(supplied) and not all(supplied):
                raise MoonDynamicGroundError("--x, --y, and --z must be set together")
            resolved = resolve_spawn_pose_for_moon_dynamic_ground(
                map_path=args.map,
                expected_sha256=args.map_sha256,
                pose_xyz=(
                    (args.x, args.y, args.z)
                    if all(supplied)
                    else None
                ),
                yaw_rad=args.yaw,
                source=args.source,
                fallback_xyz=(args.fallback_x, args.fallback_y, args.fallback_z),
                fallback_yaw_rad=args.fallback_yaw,
                root_clearance_m=args.root_clearance,
                min_resume_clearance_m=args.min_resume_clearance,
                max_resume_clearance_m=args.max_resume_clearance,
            )
            print("pose")
            print(format(float(resolved["x"]), ".17g"))
            print(format(float(resolved["y"]), ".17g"))
            print(format(float(resolved["z"]), ".17g"))
            print(format(float(resolved["yaw_rad"]), ".17g"))
            print(str(resolved["source"]))
            raw_ground_height = resolved["raw_ground_height_m"]
            input_clearance = resolved["input_clearance_m"]
            print(
                "raw_ground_height="
                + (
                    "none"
                    if raw_ground_height is None
                    else format(float(raw_ground_height), ".17g")
                )
                + " input_clearance="
                + (
                    "none"
                    if input_clearance is None
                    else format(float(input_clearance), ".17g")
                )
            )
            return 0
    except MoonDynamicGroundError as exc:
        raise SystemExit(f"[ERROR] {exc}") from exc
    raise SystemExit("[ERROR] unsupported MoonWorld command")


__all__ = (
    "CONTINUOUS_SUPPORT_HALF_EXTENT_M",
    "DEFAULT_COLLISION_MODE",
    "LOCKED_MOONWORLD_SHA256",
    "MAP_HALF_EXTENT_M",
    "MoonDynamicGround",
    "MoonDynamicGroundError",
    "resolve_spawn_pose_for_moon_dynamic_ground",
    "sample_raw_height_from_map",
)


if __name__ == "__main__":
    raise SystemExit(main())
