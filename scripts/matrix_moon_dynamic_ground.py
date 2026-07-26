#!/usr/bin/env python3
"""Strict MoonWorld rolling-ground loader and MuJoCo mocap updater.

The coordinate math in this module mirrors the bundled native
``DynamicHeightField`` implementation.  The height samples are absolute world
Z values: no centre-height subtraction, vertical scale, or Z offset is applied.
"""

from __future__ import annotations

import hashlib
import json
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
TILE_BODY_PATTERN = re.compile(r"gb_([0-9]+)_([0-9]+)")
CONTINUOUS_SUPPORT_ASSET_NAME = "matrix_moon_continuous_support_hfield"
CONTINUOUS_SUPPORT_GEOM_NAME = "matrix_moon_continuous_support"
CONTINUOUS_SUPPORT_GRID_SIDE_SAMPLES = (TILE_SIDE_COUNT * 2) + 1
CONTINUOUS_SUPPORT_SAMPLE_COUNT = (
    CONTINUOUS_SUPPORT_GRID_SIDE_SAMPLES * CONTINUOUS_SUPPORT_GRID_SIDE_SAMPLES
)
CONTINUOUS_SUPPORT_HALF_EXTENT_M = 1.6
CONTINUOUS_SUPPORT_HEIGHT_RANGE_M = 64.0
CONTINUOUS_SUPPORT_BASE_DEPTH_M = 1.0
_MUJOCO_HFIELD_GEOM_TYPE = 1
TELEMETRY_SCHEMA = "matrix-moon-dynamic-ground/v2"
LOCKED_MOONWORLD_SHA256 = (
    "62e624b5feca0111033c60d0e820f3a320257acd72b565234ac79c704dbca1df"
)

_EXPECTED_TILE_KEYS = tuple(
    (i, j)
    for i in range(TILE_SIDE_COUNT)
    for j in range(TILE_SIDE_COUNT)
)
_EXPECTED_TILE_KEY_SET = frozenset(_EXPECTED_TILE_KEYS)
_SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")
_FINITE_SCAN_CHUNK_SAMPLES = 1024 * 1024
_CACHE_SENTINEL_TILE_INDICES = (0, TILE_COUNT - 1)


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
    """Return the C/C++ ``round`` result used by the native implementation."""

    number = _finite_float(value, label="round input")
    if number >= 0.0:
        return math.trunc(number + 0.5)
    return math.trunc(number - 0.5)


def native_quantize(value: object) -> float:
    """Quantize one coordinate to the native 10 cm rolling-grid lattice."""

    number = _finite_float(value, label="quantize input")
    scaled = (number - MAP_HALF_CELL_M) / MAP_RESOLUTION_M
    return (
        round_away_from_zero(scaled) * MAP_RESOLUTION_M
        + MAP_HALF_CELL_M
    )


def world_to_pixel(x_m: object, y_m: object) -> tuple[int, int]:
    """Convert world X/Y to the clamped native height-map pixel coordinates."""

    x = _finite_float(x_m, label="world x")
    y = _finite_float(y_m, label="world y")
    pixel_x = round_away_from_zero(
        (x + MAP_HALF_EXTENT_M) / MAP_RESOLUTION_M
    )
    pixel_y = round_away_from_zero(
        (y + MAP_HALF_EXTENT_M) / MAP_RESOLUTION_M
    )
    limit = MAP_SIDE_SAMPLES - 1
    return (
        min(max(pixel_x, 0), limit),
        min(max(pixel_y, 0), limit),
    )


def _model_body_name(model: Any, body_id: int) -> str | None:
    try:
        name = model.body(body_id).name
    except (AttributeError, IndexError, KeyError, TypeError, ValueError) as exc:
        raise MoonDynamicGroundError(
            f"cannot inspect MuJoCo body id {body_id}"
        ) from exc
    if name is None:
        return None
    if not isinstance(name, str):
        raise MoonDynamicGroundError(
            f"MuJoCo body id {body_id} has a non-string name"
        )
    return name


def resolve_tile_mocap_ids(model: Any) -> np.ndarray:
    """Resolve the exact ``gb_0_0`` .. ``gb_15_15`` mocap-body contract.

    The returned array is ordered with ``i`` as the major index and ``j`` as
    the minor index, matching the native body-name mapping.
    """

    try:
        nbody = int(model.nbody)
        nmocap = int(model.nmocap)
    except (AttributeError, TypeError, ValueError) as exc:
        raise MoonDynamicGroundError(
            "MuJoCo model is missing nbody/nmocap metadata"
        ) from exc
    if nbody <= 0 or nmocap < TILE_COUNT:
        raise MoonDynamicGroundError(
            "MuJoCo model cannot provide 256 MoonWorld mocap bodies: "
            f"nbody={nbody} nmocap={nmocap}"
        )

    body_id_by_key: dict[tuple[int, int], int] = {}
    malformed_names: list[str] = []
    for body_id in range(nbody):
        name = _model_body_name(model, body_id)
        if name is None or not name.startswith("gb_"):
            continue
        match = TILE_BODY_PATTERN.fullmatch(name)
        if match is None:
            malformed_names.append(name)
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
    if malformed_names or missing or unexpected:
        raise MoonDynamicGroundError(
            "MoonWorld tile body set drifted: "
            f"missing={missing[:8]} unexpected={unexpected[:8]} "
            f"malformed={sorted(malformed_names)[:8]}"
        )

    try:
        body_mocapid = model.body_mocapid
        mocap_ids = np.asarray(
            [int(body_mocapid[body_id_by_key[key]]) for key in _EXPECTED_TILE_KEYS],
            dtype=np.int64,
        )
    except (AttributeError, IndexError, TypeError, ValueError) as exc:
        raise MoonDynamicGroundError(
            "MuJoCo body_mocapid metadata is unavailable or truncated"
        ) from exc

    invalid = mocap_ids[(mocap_ids < 0) | (mocap_ids >= nmocap)]
    if invalid.size:
        raise MoonDynamicGroundError(
            "MoonWorld tile body is not a valid mocap body: "
            f"mocap_ids={invalid[:8].tolist()} nmocap={nmocap}"
        )
    unique_ids = np.unique(mocap_ids)
    if unique_ids.size != TILE_COUNT:
        raise MoonDynamicGroundError(
            "MoonWorld tile bodies do not map one-to-one to mocap ids: "
            f"unique={int(unique_ids.size)} expected={TILE_COUNT}"
        )
    mocap_ids.setflags(write=False)
    return mocap_ids


def _named_model_id(model: Any, kind: str, name: str) -> int:
    try:
        item = getattr(model, kind)(name)
        item_id = int(item.id)
    except (AttributeError, IndexError, KeyError, TypeError, ValueError) as exc:
        raise MoonDynamicGroundError(
            f"MuJoCo model is missing {kind} {name!r}"
        ) from exc
    if item_id < 0:
        raise MoonDynamicGroundError(f"MuJoCo {kind} {name!r} has an invalid id")
    return item_id


def _unique_named_model_id(
    model: Any,
    kind: str,
    name: str,
    *,
    count: int,
) -> int:
    matches: list[int] = []
    try:
        accessor = getattr(model, kind)
        for item_id in range(count):
            item_name = accessor(item_id).name
            if isinstance(item_name, bytes):
                item_name = item_name.decode("utf-8")
            if item_name == name:
                matches.append(item_id)
    except (AttributeError, IndexError, KeyError, TypeError, UnicodeDecodeError) as exc:
        raise MoonDynamicGroundError(
            f"MuJoCo {kind} name table is unavailable or truncated"
        ) from exc
    if not matches:
        raise MoonDynamicGroundError(f"MuJoCo model is missing {kind} {name!r}")
    if len(matches) != 1:
        raise MoonDynamicGroundError(
            f"MuJoCo {kind} {name!r} must be unique"
        )
    return matches[0]


def resolve_continuous_support(model: Any) -> dict[str, int]:
    """Resolve the non-colliding rolling hfield used for terrain observations."""

    try:
        ngeom = int(model.ngeom)
        nhfield = int(model.nhfield)
    except (AttributeError, TypeError, ValueError) as exc:
        raise MoonDynamicGroundError(
            "MuJoCo continuous-support model counts are unavailable"
        ) from exc
    if ngeom < 0 or nhfield < 0:
        raise MoonDynamicGroundError(
            "MuJoCo continuous-support model counts are invalid"
        )
    geom_id = _unique_named_model_id(
        model,
        "geom",
        CONTINUOUS_SUPPORT_GEOM_NAME,
        count=ngeom,
    )
    hfield_id = _unique_named_model_id(
        model,
        "hfield",
        CONTINUOUS_SUPPORT_ASSET_NAME,
        count=nhfield,
    )
    try:
        geom_data_id = int(model.geom_dataid[geom_id])
        geom_body_id = int(model.geom_bodyid[geom_id])
        geom_type = int(model.geom_type[geom_id])
        geom_contype = int(model.geom_contype[geom_id])
        geom_conaffinity = int(model.geom_conaffinity[geom_id])
        nrow = int(model.hfield_nrow[hfield_id])
        ncol = int(model.hfield_ncol[hfield_id])
        data_adr = int(model.hfield_adr[hfield_id])
        hfield_size = tuple(float(value) for value in model.hfield_size[hfield_id])
        hfield_data_size = int(np.size(model.hfield_data))
    except (AttributeError, IndexError, TypeError, ValueError) as exc:
        raise MoonDynamicGroundError(
            "MuJoCo continuous-support metadata is unavailable or truncated"
        ) from exc
    if not 0 <= geom_id < ngeom or not 0 <= hfield_id < nhfield:
        raise MoonDynamicGroundError("continuous-support id is outside the model table")
    if geom_body_id != 0:
        raise MoonDynamicGroundError(
            "continuous support must be attached to the world body"
        )
    if geom_type != _MUJOCO_HFIELD_GEOM_TYPE:
        raise MoonDynamicGroundError(
            "continuous support geom must be a MuJoCo hfield"
        )
    if geom_data_id != hfield_id:
        raise MoonDynamicGroundError(
            "continuous support geom must bind the named hfield asset"
        )
    if geom_contype != 0 or geom_conaffinity != 0:
        raise MoonDynamicGroundError(
            "continuous support geom must use contype=0 and conaffinity=0"
        )
    if (
        nrow != CONTINUOUS_SUPPORT_GRID_SIDE_SAMPLES
        or ncol != CONTINUOUS_SUPPORT_GRID_SIDE_SAMPLES
        or data_adr < 0
        or data_adr + CONTINUOUS_SUPPORT_SAMPLE_COUNT > hfield_data_size
    ):
        raise MoonDynamicGroundError(
            "continuous-support hfield shape/address contract drifted"
        )
    expected_size = (
        CONTINUOUS_SUPPORT_HALF_EXTENT_M,
        CONTINUOUS_SUPPORT_HALF_EXTENT_M,
        CONTINUOUS_SUPPORT_HEIGHT_RANGE_M,
        CONTINUOUS_SUPPORT_BASE_DEPTH_M,
    )
    if len(hfield_size) != 4 or any(
        not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-12)
        for actual, expected in zip(hfield_size, expected_size, strict=True)
    ):
        raise MoonDynamicGroundError(
            "continuous-support hfield size contract drifted"
        )

    for i, j in _EXPECTED_TILE_KEYS:
        geom_name = f"soil_{i}_{j}"
        tile_geom_id = _named_model_id(model, "geom", geom_name)
        try:
            contype = int(model.geom_contype[tile_geom_id])
            conaffinity = int(model.geom_conaffinity[tile_geom_id])
        except (AttributeError, IndexError, TypeError, ValueError) as exc:
            raise MoonDynamicGroundError(
                f"MuJoCo tile geom collision metadata is unavailable: {geom_name}"
            ) from exc
        if contype != 0 or conaffinity != 0:
            raise MoonDynamicGroundError(
                f"MoonWorld source tile geom remains collidable: {geom_name}"
            )
    return {
        "geom_id": geom_id,
        "hfield_id": hfield_id,
        "data_adr": data_adr,
    }


def _round_array_away_from_zero(values: np.ndarray) -> np.ndarray:
    shifts = np.where(values >= 0.0, 0.5, -0.5)
    return np.trunc(values + shifts).astype(np.int64)


class MoonDynamicGround:
    """Read-only mmap plus rolling visual tiles and one collision hfield."""

    def __init__(
        self,
        path: str | os.PathLike[str],
        model: Any,
        *,
        expected_sha256: str | None = None,
    ) -> None:
        self.path = Path(path).expanduser().resolve()
        if expected_sha256 is not None:
            if not isinstance(expected_sha256, str) or (
                _SHA256_PATTERN.fullmatch(expected_sha256) is None
            ):
                raise MoonDynamicGroundError(
                    "expected_sha256 must be 64 lowercase hexadecimal characters"
                )
        self.expected_sha256 = expected_sha256
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
        self._cached_pixel_x_range: tuple[int, int] | None = None
        self._cached_pixel_y_range: tuple[int, int] | None = None
        self._cached_height_range_m: tuple[float, float] | None = None
        self._cached_support_pixel_x_range: tuple[int, int] | None = None
        self._cached_support_pixel_y_range: tuple[int, int] | None = None
        self._cached_support_height_range_m: tuple[float, float] | None = None

        tile_i = np.repeat(
            np.arange(TILE_SIDE_COUNT, dtype=np.float64),
            TILE_SIDE_COUNT,
        )
        tile_j = np.tile(
            np.arange(TILE_SIDE_COUNT, dtype=np.float64),
            TILE_SIDE_COUNT,
        )
        self._tile_x_offsets = (
            (tile_i + TILE_INDEX_ORIGIN) * MAP_RESOLUTION_M
            + MAP_HALF_CELL_M
        )
        self._tile_y_offsets = (
            (tile_j + TILE_INDEX_ORIGIN) * MAP_RESOLUTION_M
            + MAP_HALF_CELL_M
        )
        self._positions = np.empty((TILE_COUNT, 3), dtype=np.float64)
        self._identity_quaternions = np.zeros(
            (TILE_COUNT, 4), dtype=np.float64
        )
        self._identity_quaternions[:, 0] = 1.0
        self._support_axis_offsets = np.linspace(
            -CONTINUOUS_SUPPORT_HALF_EXTENT_M,
            CONTINUOUS_SUPPORT_HALF_EXTENT_M,
            CONTINUOUS_SUPPORT_GRID_SIDE_SAMPLES,
            dtype=np.float64,
        )
        self._support_values = np.empty(
            CONTINUOUS_SUPPORT_SAMPLE_COUNT,
            dtype=np.float32,
        )
        self._support_position = np.empty(3, dtype=np.float64)

        try:
            self._open_and_validate_map()
            assert self.minimum_height_m is not None
            assert self.maximum_height_m is not None
            if (
                self.maximum_height_m - self.minimum_height_m
                > CONTINUOUS_SUPPORT_HEIGHT_RANGE_M
            ):
                raise MoonDynamicGroundError(
                    "MoonWorld map height range exceeds continuous-support capacity: "
                    f"minimum={self.minimum_height_m} maximum={self.maximum_height_m} "
                    f"capacity={CONTINUOUS_SUPPORT_HEIGHT_RANGE_M}"
                )
            self.mocap_ids = resolve_tile_mocap_ids(model)
            support = resolve_continuous_support(model)
            self._model = model
            self.support_geom_id = support["geom_id"]
            self.support_hfield_id = support["hfield_id"]
            self.support_data_adr = support["data_adr"]
            self._cache_sentinel_mocap_ids = tuple(
                int(self.mocap_ids[tile_index])
                for tile_index in _CACHE_SENTINEL_TILE_INDICES
            )
        except Exception:
            self.close()
            raise

    def _open_and_validate_map(self) -> None:
        stream = None
        mapped = None
        heights = None
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
                    f"expected={MAP_SIZE_BYTES} actual={metadata.st_size} "
                    f"path={self.path}"
                )
            mapped = mmap.mmap(
                stream.fileno(),
                MAP_SIZE_BYTES,
                access=mmap.ACCESS_READ,
            )
            actual_sha256 = hashlib.sha256(mapped).hexdigest()
            if (
                self.expected_sha256 is not None
                and actual_sha256 != self.expected_sha256
            ):
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
            flat = heights.reshape(-1)
            minimum = math.inf
            maximum = -math.inf
            for start in range(0, MAP_SAMPLE_COUNT, _FINITE_SCAN_CHUNK_SAMPLES):
                stop = min(start + _FINITE_SCAN_CHUNK_SAMPLES, MAP_SAMPLE_COUNT)
                chunk = flat[start:stop]
                finite = np.isfinite(chunk)
                if not bool(np.all(finite)):
                    relative = int(np.flatnonzero(~finite)[0])
                    absolute = start + relative
                    row, column = divmod(absolute, MAP_SIDE_SAMPLES)
                    raise MoonDynamicGroundError(
                        "MoonWorld height map contains a non-finite sample: "
                        f"row={row} column={column}"
                    )
                minimum = min(minimum, float(np.min(chunk)))
                maximum = max(maximum, float(np.max(chunk)))

            final_metadata = os.fstat(stream.fileno())
            if (
                final_metadata.st_dev != metadata.st_dev
                or final_metadata.st_ino != metadata.st_ino
                or final_metadata.st_size != metadata.st_size
                or final_metadata.st_mtime_ns != metadata.st_mtime_ns
            ):
                raise MoonDynamicGroundError(
                    "MoonWorld height map changed while it was being validated"
                )

            self.actual_sha256 = actual_sha256
            self.file_size_bytes = int(metadata.st_size)
            self.file_inode = int(metadata.st_ino)
            self.file_mtime_ns = int(metadata.st_mtime_ns)
            self.minimum_height_m = minimum
            self.maximum_height_m = maximum
            self._stream = stream
            self._mapped = mapped
            self._heights = heights
        except MoonDynamicGroundError:
            if heights is not None:
                del heights
            if mapped is not None:
                mapped.close()
            if stream is not None:
                stream.close()
            raise
        except (OSError, ValueError) as exc:
            if heights is not None:
                del heights
            if mapped is not None:
                mapped.close()
            if stream is not None:
                stream.close()
            raise MoonDynamicGroundError(
                f"cannot open MoonWorld height map {self.path}: {exc}"
            ) from exc

    @property
    def closed(self) -> bool:
        return self._closed

    def _require_open_heights(self) -> np.ndarray:
        if self._closed or self._heights is None:
            raise MoonDynamicGroundError("MoonWorld dynamic ground is closed")
        return self._heights

    def sample_height(self, x_m: object, y_m: object) -> float:
        """Sample the continuous support using MuJoCo's hfield triangulation."""

        heights = self._require_open_heights()
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
        # MuJoCo splits every hfield cell along the (0, 0) -> (1, 1)
        # diagonal. Match that piecewise-linear surface exactly so fall and
        # clearance metrics agree with collision rays.
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
            raise MoonDynamicGroundError(
                "MoonWorld sampled a non-finite continuous support height at "
                f"({fractional_x}, {fractional_y})"
            )
        return height

    def _cache_sentinels_match(self, data: Any) -> bool:
        """Check cached poses and support so external resets are visible."""

        try:
            mocap_pos = data.mocap_pos
            mocap_quat = data.mocap_quat
            for tile_index, mocap_id in zip(
                _CACHE_SENTINEL_TILE_INDICES,
                self._cache_sentinel_mocap_ids,
                strict=True,
            ):
                for axis in range(3):
                    if (
                        float(mocap_pos[mocap_id, axis])
                        != float(self._positions[tile_index, axis])
                    ):
                        return False
                for axis in range(4):
                    if (
                        float(mocap_quat[mocap_id, axis])
                        != float(self._identity_quaternions[tile_index, axis])
                    ):
                        return False
            support_position = self._model.geom_pos[self.support_geom_id]
            for axis in range(3):
                if float(support_position[axis]) != float(
                    self._support_position[axis]
                ):
                    return False
            support_data = self._model.hfield_data[
                self.support_data_adr : (
                    self.support_data_adr + CONTINUOUS_SUPPORT_SAMPLE_COUNT
                )
            ]
            if not bool(np.array_equal(support_data, self._support_values)):
                return False
        except (
            AttributeError,
            IndexError,
            KeyError,
            TypeError,
            ValueError,
        ):
            return False
        return True

    def update_mocap(
        self,
        data: Any,
        *,
        base_xy: Iterable[object] | None = None,
    ) -> dict[str, object]:
        """Refresh tiles after a grid crossing and report the current ground."""

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
        local_ground_height_m = self.sample_height(base_x, base_y)
        cache_invalidated = False

        if (
            data is self._cached_data
            and quantized_base_xy == self._cached_quantized_base_xy
        ):
            if (
                self._cached_pixel_x_range is None
                or self._cached_pixel_y_range is None
                or self._cached_height_range_m is None
                or self._cached_support_pixel_x_range is None
                or self._cached_support_pixel_y_range is None
                or self._cached_support_height_range_m is None
            ):
                raise MoonDynamicGroundError(
                    "MoonWorld dynamic-ground cache metadata is incomplete"
                )
            if self._cache_sentinels_match(data):
                self._update_count += 1
                self._cache_hit_count += 1
                self._last_update = {
                    "base_xy_m": [base_x, base_y],
                    "quantized_base_xy_m": [quantized_x, quantized_y],
                    "local_ground_height_m": local_ground_height_m,
                    "pixel_x_range": list(self._cached_pixel_x_range),
                    "pixel_y_range": list(self._cached_pixel_y_range),
                    "height_range_m": list(self._cached_height_range_m),
                    "support_pixel_x_range": list(
                        self._cached_support_pixel_x_range
                    ),
                    "support_pixel_y_range": list(
                        self._cached_support_pixel_y_range
                    ),
                    "support_height_range_m": list(
                        self._cached_support_height_range_m
                    ),
                    "cache_hit": True,
                    "cache_invalidated": False,
                    "tiles_updated": False,
                    "continuous_support_updated": False,
                }
                return dict(self._last_update)
            cache_invalidated = True

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
        if not bool(np.all(np.isfinite(support_z))):
            raise MoonDynamicGroundError(
                "MoonWorld continuous support sampled a non-finite height"
            )
        assert self.minimum_height_m is not None
        normalized_support = (
            np.asarray(support_z, dtype=np.float64) - self.minimum_height_m
        ) / CONTINUOUS_SUPPORT_HEIGHT_RANGE_M
        if bool(np.any(normalized_support < 0.0)) or bool(
            np.any(normalized_support > 1.0)
        ):
            raise MoonDynamicGroundError(
                "MoonWorld continuous support exceeds its normalized height range"
            )

        try:
            mocap_pos = data.mocap_pos
            mocap_quat = data.mocap_quat
            hfield_data = self._model.hfield_data
            geom_pos = self._model.geom_pos
        except AttributeError as exc:
            raise MoonDynamicGroundError(
                "MuJoCo data is missing mocap_pos/mocap_quat"
            ) from exc
        mocap_pos_shape = tuple(np.shape(mocap_pos))
        mocap_quat_shape = tuple(np.shape(mocap_quat))
        if len(mocap_pos_shape) != 2 or mocap_pos_shape[1] != 3:
            raise MoonDynamicGroundError(
                f"MuJoCo mocap_pos has invalid shape: {mocap_pos_shape}"
            )
        if len(mocap_quat_shape) != 2 or mocap_quat_shape[1] != 4:
            raise MoonDynamicGroundError(
                f"MuJoCo mocap_quat has invalid shape: {mocap_quat_shape}"
            )
        if (
            mocap_pos_shape[0] <= int(np.max(self.mocap_ids))
            or mocap_quat_shape[0] <= int(np.max(self.mocap_ids))
        ):
            raise MoonDynamicGroundError(
                "MuJoCo mocap arrays are shorter than the resolved tile ids"
            )
        if np.size(hfield_data) < (
            self.support_data_adr + CONTINUOUS_SUPPORT_SAMPLE_COUNT
        ):
            raise MoonDynamicGroundError(
                "MuJoCo hfield data is shorter than the continuous support"
            )
        if tuple(np.shape(geom_pos)) != (int(self._model.ngeom), 3):
            raise MoonDynamicGroundError("MuJoCo geom_pos has an invalid shape")

        self._positions[:, 0] = tile_x
        self._positions[:, 1] = tile_y
        self._positions[:, 2] = tile_z
        self._support_values[:] = normalized_support.reshape(-1)
        self._support_position[:] = (
            support_center_x,
            support_center_y,
            self.minimum_height_m,
        )
        try:
            mocap_pos[self.mocap_ids, :] = self._positions
            mocap_quat[self.mocap_ids, :] = self._identity_quaternions
            hfield_data[
                self.support_data_adr : (
                    self.support_data_adr + CONTINUOUS_SUPPORT_SAMPLE_COUNT
                )
            ] = self._support_values
            geom_pos[self.support_geom_id, :] = self._support_position
        except (IndexError, TypeError, ValueError) as exc:
            raise MoonDynamicGroundError(
                f"cannot write MoonWorld mocap poses: {exc}"
            ) from exc

        pixel_x_range = (int(np.min(pixel_x)), int(np.max(pixel_x)))
        pixel_y_range = (int(np.min(pixel_y)), int(np.max(pixel_y)))
        height_range_m = (float(np.min(tile_z)), float(np.max(tile_z)))
        support_pixel_x_range = (
            int(np.min(support_pixel_x)),
            int(np.max(support_pixel_x)),
        )
        support_pixel_y_range = (
            int(np.min(support_pixel_y)),
            int(np.max(support_pixel_y)),
        )
        support_height_range_m = (
            float(np.min(support_z)),
            float(np.max(support_z)),
        )
        self._cached_data = data
        self._cached_quantized_base_xy = quantized_base_xy
        self._cached_pixel_x_range = pixel_x_range
        self._cached_pixel_y_range = pixel_y_range
        self._cached_height_range_m = height_range_m
        self._cached_support_pixel_x_range = support_pixel_x_range
        self._cached_support_pixel_y_range = support_pixel_y_range
        self._cached_support_height_range_m = support_height_range_m
        self._update_count += 1
        self._tile_update_count += 1
        if cache_invalidated:
            self._cache_invalidation_count += 1
        self._last_update = {
            "base_xy_m": [base_x, base_y],
            "quantized_base_xy_m": [quantized_x, quantized_y],
            "local_ground_height_m": local_ground_height_m,
            "pixel_x_range": list(pixel_x_range),
            "pixel_y_range": list(pixel_y_range),
            "height_range_m": list(height_range_m),
            "support_pixel_x_range": list(support_pixel_x_range),
            "support_pixel_y_range": list(support_pixel_y_range),
            "support_height_range_m": list(support_height_range_m),
            "cache_hit": False,
            "cache_invalidated": cache_invalidated,
            "tiles_updated": True,
            "continuous_support_updated": True,
        }
        return dict(self._last_update)

    def telemetry(self) -> dict[str, object]:
        """Return a JSON-serializable attestation and latest-update summary."""

        payload: dict[str, object] = {
            "schema": TELEMETRY_SCHEMA,
            "closed": self._closed,
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
                "unique_mocap_ids": (
                    int(np.unique(self.mocap_ids).size)
                    if hasattr(self, "mocap_ids")
                    else 0
                ),
            },
            "continuous_support": {
                "mode": "rolling-heightfield-v1",
                "geom_name": CONTINUOUS_SUPPORT_GEOM_NAME,
                "asset_name": CONTINUOUS_SUPPORT_ASSET_NAME,
                "geom_id": getattr(self, "support_geom_id", None),
                "hfield_id": getattr(self, "support_hfield_id", None),
                "grid_shape": [
                    CONTINUOUS_SUPPORT_GRID_SIDE_SAMPLES,
                    CONTINUOUS_SUPPORT_GRID_SIDE_SAMPLES,
                ],
                "half_extent_m": CONTINUOUS_SUPPORT_HALF_EXTENT_M,
                "height_range_m": CONTINUOUS_SUPPORT_HEIGHT_RANGE_M,
                "base_depth_m": CONTINUOUS_SUPPORT_BASE_DEPTH_M,
                "source_tile_collision_enabled": False,
            },
            # ``update_count`` retains its v1 meaning: successful calls.
            # ``tile_update_count`` counts the cache misses that wrote 256 poses.
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
        # Exercise the serialization contract here so NumPy scalars cannot
        # silently leak into status publication.
        json.dumps(payload, sort_keys=True)
        return payload

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._cached_data = None
        self._cached_quantized_base_xy = None
        self._cached_pixel_x_range = None
        self._cached_pixel_y_range = None
        self._cached_height_range_m = None
        self._cached_support_pixel_x_range = None
        self._cached_support_pixel_y_range = None
        self._cached_support_height_range_m = None
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


__all__ = (
    "CONTINUOUS_SUPPORT_ASSET_NAME",
    "CONTINUOUS_SUPPORT_BASE_DEPTH_M",
    "CONTINUOUS_SUPPORT_GEOM_NAME",
    "CONTINUOUS_SUPPORT_GRID_SIDE_SAMPLES",
    "CONTINUOUS_SUPPORT_HALF_EXTENT_M",
    "CONTINUOUS_SUPPORT_HEIGHT_RANGE_M",
    "CONTINUOUS_SUPPORT_SAMPLE_COUNT",
    "MAP_DTYPE",
    "MAP_HALF_CELL_M",
    "MAP_HALF_EXTENT_M",
    "MAP_RESOLUTION_M",
    "MAP_SAMPLE_COUNT",
    "MAP_SIDE_SAMPLES",
    "MAP_SIZE_BYTES",
    "LOCKED_MOONWORLD_SHA256",
    "MoonDynamicGround",
    "MoonDynamicGroundError",
    "TELEMETRY_SCHEMA",
    "TILE_BODY_PATTERN",
    "TILE_COUNT",
    "TILE_SIDE_COUNT",
    "native_quantize",
    "resolve_continuous_support",
    "resolve_tile_mocap_ids",
    "round_away_from_zero",
    "world_to_pixel",
)
