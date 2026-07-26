from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest import mock

import numpy as np


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "matrix_moon_dynamic_ground.py"
)
SPEC = importlib.util.spec_from_file_location("matrix_moon_dynamic_ground", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class FakeBody:
    def __init__(self, body_id: int, name: str | None) -> None:
        self.id = body_id
        self.name = name


class FakeModel:
    def __init__(
        self,
        *,
        missing: tuple[int, int] | None = None,
        extra_name: str | None = None,
        unmapped: tuple[int, int] | None = None,
        duplicate_mocap: tuple[tuple[int, int], tuple[int, int]] | None = None,
        continuous_support: bool = True,
        tile_collision_enabled: bool = False,
    ) -> None:
        names: list[str | None] = ["world", "pelvis"]
        keys: list[tuple[int, int] | None] = [None, None]
        for i in range(MODULE.TILE_SIDE_COUNT):
            for j in range(MODULE.TILE_SIDE_COUNT):
                if (i, j) == missing:
                    continue
                names.append(f"gb_{i}_{j}")
                keys.append((i, j))
        if extra_name is not None:
            names.append(extra_name)
            keys.append(None)

        self.nbody = len(names)
        self.nmocap = MODULE.TILE_COUNT
        self._names = names
        self.body_mocapid = np.full(self.nbody, -1, dtype=np.int64)
        body_id_by_key: dict[tuple[int, int], int] = {}
        for body_id, key in enumerate(keys):
            if key is None:
                continue
            body_id_by_key[key] = body_id
            self.body_mocapid[body_id] = key[0] * MODULE.TILE_SIDE_COUNT + key[1]
        if unmapped is not None and unmapped in body_id_by_key:
            self.body_mocapid[body_id_by_key[unmapped]] = -1
        if duplicate_mocap is not None:
            first, second = duplicate_mocap
            self.body_mocapid[body_id_by_key[second]] = self.body_mocapid[
                body_id_by_key[first]
            ]
        self._geom_names: list[str] = []
        geom_body_ids: list[int] = []
        geom_data_ids: list[int] = []
        geom_types: list[int] = []
        geom_contype: list[int] = []
        geom_conaffinity: list[int] = []
        if continuous_support:
            self._geom_names.append(MODULE.CONTINUOUS_SUPPORT_GEOM_NAME)
            geom_body_ids.append(0)
            geom_data_ids.append(0)
            geom_types.append(1)
            geom_contype.append(0)
            geom_conaffinity.append(0)
        for i, j in MODULE._EXPECTED_TILE_KEYS:
            self._geom_names.append(f"soil_{i}_{j}")
            geom_body_ids.append(body_id_by_key.get((i, j), 0))
            geom_data_ids.append(-1)
            geom_types.append(6)
            geom_contype.append(1 if tile_collision_enabled else 0)
            geom_conaffinity.append(1 if tile_collision_enabled else 0)
        self.ngeom = len(self._geom_names)
        self.geom_bodyid = np.asarray(geom_body_ids, dtype=np.int64)
        self.geom_dataid = np.asarray(geom_data_ids, dtype=np.int64)
        self.geom_type = np.asarray(geom_types, dtype=np.int64)
        self.geom_contype = np.asarray(geom_contype, dtype=np.int64)
        self.geom_conaffinity = np.asarray(geom_conaffinity, dtype=np.int64)
        self.geom_pos = np.zeros((self.ngeom, 3), dtype=np.float64)
        self.nhfield = 1 if continuous_support else 0
        self._hfield_names = (
            [MODULE.CONTINUOUS_SUPPORT_ASSET_NAME]
            if continuous_support
            else []
        )
        self.hfield_nrow = np.asarray(
            [MODULE.CONTINUOUS_SUPPORT_GRID_SIDE_SAMPLES]
            if continuous_support
            else [],
            dtype=np.int64,
        )
        self.hfield_ncol = self.hfield_nrow.copy()
        self.hfield_adr = np.asarray(
            [0] if continuous_support else [],
            dtype=np.int64,
        )
        self.hfield_size = np.asarray(
            [
                [
                    MODULE.CONTINUOUS_SUPPORT_HALF_EXTENT_M,
                    MODULE.CONTINUOUS_SUPPORT_HALF_EXTENT_M,
                    MODULE.CONTINUOUS_SUPPORT_HEIGHT_RANGE_M,
                    MODULE.CONTINUOUS_SUPPORT_BASE_DEPTH_M,
                ]
            ]
            if continuous_support
            else [],
            dtype=np.float64,
        ).reshape((-1, 4))
        self.hfield_data = np.zeros(
            MODULE.CONTINUOUS_SUPPORT_SAMPLE_COUNT if continuous_support else 0,
            dtype=np.float32,
        )

    def body(self, key: int | str) -> FakeBody:
        if isinstance(key, str):
            try:
                body_id = self._names.index(key)
            except ValueError as exc:
                raise KeyError(key) from exc
            return FakeBody(body_id, self._names[body_id])
        if key < 0 or key >= self.nbody:
            raise KeyError(key)
        return FakeBody(key, self._names[key])

    def geom(self, key: int | str) -> FakeBody:
        if isinstance(key, str):
            try:
                geom_id = self._geom_names.index(key)
            except ValueError as exc:
                raise KeyError(key) from exc
        else:
            geom_id = key
        if geom_id < 0 or geom_id >= self.ngeom:
            raise KeyError(key)
        return FakeBody(geom_id, self._geom_names[geom_id])

    def hfield(self, key: int | str) -> FakeBody:
        if isinstance(key, str):
            try:
                hfield_id = self._hfield_names.index(key)
            except ValueError as exc:
                raise KeyError(key) from exc
        else:
            hfield_id = key
        if hfield_id < 0 or hfield_id >= self.nhfield:
            raise KeyError(key)
        return FakeBody(hfield_id, self._hfield_names[hfield_id])


class FakeData:
    def __init__(self) -> None:
        self.qpos = np.asarray([0.0, 0.0, 0.8], dtype=np.float64)
        self.mocap_pos = np.full(
            (MODULE.TILE_COUNT, 3),
            np.nan,
            dtype=np.float64,
        )
        self.mocap_quat = np.full(
            (MODULE.TILE_COUNT, 4),
            np.nan,
            dtype=np.float64,
        )


class CountingMocapArray:
    def __init__(self, shape: tuple[int, int]) -> None:
        self.values = np.full(shape, np.nan, dtype=np.float64)
        self.read_count = 0
        self.write_count = 0

    @property
    def shape(self) -> tuple[int, ...]:
        return self.values.shape

    def __setitem__(self, key: object, value: object) -> None:
        self.write_count += 1
        self.values[key] = value

    def __getitem__(self, key: object) -> object:
        self.read_count += 1
        return self.values[key]


def write_sparse_map(
    path: Path,
    samples: dict[tuple[int, int], float] | None = None,
) -> None:
    with path.open("wb") as stream:
        stream.truncate(MODULE.MAP_SIZE_BYTES)
    if not samples:
        return
    with path.open("r+b") as stream:
        for (row, column), value in samples.items():
            offset = (
                (row * MODULE.MAP_SIDE_SAMPLES + column)
                * MODULE.MAP_DTYPE.itemsize
            )
            stream.seek(offset)
            stream.write(struct.pack("<f", value))


class MoonDynamicGroundTest(unittest.TestCase):
    def test_native_rounding_quantization_and_pixels(self) -> None:
        self.assertEqual(MODULE.CONTINUOUS_SUPPORT_GRID_SIDE_SAMPLES, 33)
        self.assertEqual(MODULE.CONTINUOUS_SUPPORT_HALF_EXTENT_M, 1.6)
        self.assertAlmostEqual(
            (2.0 * MODULE.CONTINUOUS_SUPPORT_HALF_EXTENT_M)
            / (MODULE.CONTINUOUS_SUPPORT_GRID_SIDE_SAMPLES - 1),
            MODULE.MAP_RESOLUTION_M,
        )
        self.assertEqual(MODULE.round_away_from_zero(0.49), 0)
        self.assertEqual(MODULE.round_away_from_zero(0.5), 1)
        self.assertEqual(MODULE.round_away_from_zero(1.5), 2)
        self.assertEqual(MODULE.round_away_from_zero(-0.49), 0)
        self.assertEqual(MODULE.round_away_from_zero(-0.5), -1)
        self.assertEqual(MODULE.round_away_from_zero(-1.5), -2)

        self.assertAlmostEqual(MODULE.native_quantize(0.0), -0.05)
        self.assertAlmostEqual(MODULE.native_quantize(0.049), 0.05)
        self.assertAlmostEqual(MODULE.native_quantize(0.1), 0.15)
        self.assertAlmostEqual(MODULE.native_quantize(-0.1), -0.15)
        self.assertEqual(MODULE.world_to_pixel(0.0, 0.0), (3000, 3000))
        self.assertEqual(
            MODULE.world_to_pixel(-1000.0, 1000.0),
            (0, MODULE.MAP_SIDE_SAMPLES - 1),
        )

    def test_resolves_exact_one_to_one_tile_mocap_mapping(self) -> None:
        model = FakeModel()
        mocap_ids = MODULE.resolve_tile_mocap_ids(model)
        self.assertEqual(mocap_ids.shape, (MODULE.TILE_COUNT,))
        self.assertEqual(mocap_ids.tolist(), list(range(MODULE.TILE_COUNT)))
        self.assertFalse(mocap_ids.flags.writeable)
        self.assertEqual(
            MODULE.resolve_continuous_support(model),
            {"geom_id": 0, "hfield_id": 0, "data_adr": 0},
        )

        drifted_models = (
            FakeModel(missing=(15, 15)),
            FakeModel(extra_name="gb_16_0"),
            FakeModel(extra_name="gb_bad"),
            FakeModel(unmapped=(3, 4)),
            FakeModel(duplicate_mocap=((0, 0), (0, 1))),
        )
        for drifted in drifted_models:
            with self.subTest(model=drifted):
                with self.assertRaises(MODULE.MoonDynamicGroundError):
                    MODULE.resolve_tile_mocap_ids(drifted)

        with self.assertRaisesRegex(
            MODULE.MoonDynamicGroundError,
            "missing geom",
        ):
            MODULE.resolve_continuous_support(
                FakeModel(continuous_support=False)
            )
        with self.assertRaisesRegex(
            MODULE.MoonDynamicGroundError,
            "remains collidable",
        ):
            MODULE.resolve_continuous_support(
                FakeModel(tile_collision_enabled=True)
            )

    def test_rejects_drifted_continuous_support_binding_and_collision(self) -> None:
        cases = (
            (
                "world body",
                {"geom_bodyid": 1},
                "world body",
            ),
            (
                "geom type",
                {"geom_type": 0},
                "MuJoCo hfield",
            ),
            (
                "hfield binding",
                {"geom_dataid": 1},
                "bind the named hfield asset",
            ),
            (
                "collision mask",
                {"geom_contype": 1, "geom_conaffinity": 1},
                "contype=0 and conaffinity=0",
            ),
        )
        for label, replacements, message in cases:
            with self.subTest(label=label):
                model = FakeModel()
                if label == "hfield binding":
                    model.nhfield = 2
                    model._hfield_names.append("other_hfield")
                for attribute, value in replacements.items():
                    table = getattr(model, attribute).copy()
                    table[0] = value
                    setattr(model, attribute, table)
                with self.assertRaisesRegex(
                    MODULE.MoonDynamicGroundError,
                    message,
                ):
                    MODULE.resolve_continuous_support(model)

    def test_rejects_duplicate_continuous_support_names(self) -> None:
        duplicate_geom = FakeModel()
        duplicate_geom._geom_names[-1] = MODULE.CONTINUOUS_SUPPORT_GEOM_NAME
        with self.assertRaisesRegex(
            MODULE.MoonDynamicGroundError,
            "geom .* must be unique",
        ):
            MODULE.resolve_continuous_support(duplicate_geom)

        duplicate_hfield = FakeModel()
        duplicate_hfield.nhfield = 2
        duplicate_hfield._hfield_names.append(
            MODULE.CONTINUOUS_SUPPORT_ASSET_NAME
        )
        with self.assertRaisesRegex(
            MODULE.MoonDynamicGroundError,
            "hfield .* must be unique",
        ):
            MODULE.resolve_continuous_support(duplicate_hfield)

    def test_rejects_size_hash_and_non_finite_samples(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_dir:
            root = Path(temporary_dir)

            wrong_size = root / "wrong-size.bin"
            wrong_size.write_bytes(b"\0" * 16)
            with self.assertRaisesRegex(
                MODULE.MoonDynamicGroundError, "size mismatch"
            ):
                MODULE.MoonDynamicGround(wrong_size, FakeModel())

            valid = root / "valid.bin"
            write_sparse_map(valid)
            with self.assertRaisesRegex(
                MODULE.MoonDynamicGroundError, "SHA256 mismatch"
            ):
                MODULE.MoonDynamicGround(
                    valid,
                    FakeModel(),
                    expected_sha256="f" * 64,
                )

            non_finite = root / "non-finite.bin"
            write_sparse_map(non_finite, {(3000, 3000): float("nan")})
            with self.assertRaisesRegex(
                MODULE.MoonDynamicGroundError, "non-finite sample"
            ):
                MODULE.MoonDynamicGround(non_finite, FakeModel())

    def test_mmap_update_writes_native_tile_poses_and_telemetry(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_dir:
            path = Path(temporary_dir) / "moonworld.bin"
            write_sparse_map(
                path,
                {
                    (2984, 2984): -2.0,
                    (2992, 2992): -2.0,
                    (3000, 3000): 1.25,
                    (3007, 3007): 3.5,
                    (3016, 3016): 4.0,
                },
            )
            model = FakeModel()
            data = FakeData()
            with MODULE.MoonDynamicGround(path, model) as ground:
                update = ground.update_mocap(data)

                tile_0_0 = 0
                tile_8_8 = 8 * MODULE.TILE_SIDE_COUNT + 8
                tile_15_15 = 15 * MODULE.TILE_SIDE_COUNT + 15
                np.testing.assert_allclose(
                    data.mocap_pos[tile_0_0],
                    [-0.8, -0.8, -2.0],
                    rtol=0.0,
                    atol=1e-12,
                )
                np.testing.assert_allclose(
                    data.mocap_pos[tile_8_8],
                    [0.0, 0.0, 1.25],
                    rtol=0.0,
                    atol=1e-12,
                )
                np.testing.assert_allclose(
                    data.mocap_pos[tile_15_15],
                    [0.7, 0.7, 3.5],
                    rtol=0.0,
                    atol=1e-12,
                )
                expected_quaternions = np.zeros((MODULE.TILE_COUNT, 4))
                expected_quaternions[:, 0] = 1.0
                np.testing.assert_array_equal(
                    data.mocap_quat, expected_quaternions
                )
                np.testing.assert_allclose(
                    model.geom_pos[0],
                    [0.0, 0.0, -2.0],
                    rtol=0.0,
                    atol=1e-12,
                )
                support = model.hfield_data.reshape(
                    MODULE.CONTINUOUS_SUPPORT_GRID_SIDE_SAMPLES,
                    MODULE.CONTINUOUS_SUPPORT_GRID_SIDE_SAMPLES,
                )
                self.assertAlmostEqual(float(support[0, 0]), 0.0)
                self.assertAlmostEqual(
                    float(support[16, 16]),
                    (1.25 - (-2.0))
                    / MODULE.CONTINUOUS_SUPPORT_HEIGHT_RANGE_M,
                )
                self.assertAlmostEqual(
                    float(support[-1, -1]),
                    (4.0 - (-2.0))
                    / MODULE.CONTINUOUS_SUPPORT_HEIGHT_RANGE_M,
                )
                self.assertEqual(update["pixel_x_range"], [2992, 3007])
                self.assertEqual(update["pixel_y_range"], [2992, 3007])
                self.assertEqual(update["height_range_m"], [-2.0, 3.5])
                self.assertEqual(update["local_ground_height_m"], 1.25)
                self.assertFalse(update["cache_hit"])
                self.assertFalse(update["cache_invalidated"])
                self.assertTrue(update["tiles_updated"])
                self.assertTrue(update["continuous_support_updated"])
                self.assertEqual(update["support_pixel_x_range"], [2984, 3016])
                self.assertEqual(update["support_pixel_y_range"], [2984, 3016])
                self.assertEqual(update["support_height_range_m"], [-2.0, 4.0])
                self.assertEqual(ground.sample_height(0.0, 0.0), 1.25)

                telemetry = ground.telemetry()
                self.assertEqual(telemetry["schema"], MODULE.TELEMETRY_SCHEMA)
                self.assertEqual(telemetry["update_count"], 1)
                self.assertEqual(telemetry["tile_update_count"], 1)
                self.assertEqual(telemetry["cache_hit_count"], 0)
                self.assertEqual(telemetry["cache_invalidation_count"], 0)
                self.assertEqual(telemetry["tiles"]["count"], MODULE.TILE_COUNT)
                self.assertEqual(
                    telemetry["continuous_support"]["mode"],
                    "rolling-heightfield-v1",
                )
                self.assertFalse(
                    telemetry["continuous_support"][
                        "source_tile_collision_enabled"
                    ]
                )
                self.assertEqual(
                    telemetry["map"]["size_bytes"], MODULE.MAP_SIZE_BYTES
                )
                self.assertEqual(
                    telemetry["map"]["storage"], "read-only-mmap"
                )
                self.assertEqual(
                    len(telemetry["map"]["sha256"]),
                    64,
                )
                json.dumps(telemetry, sort_keys=True)

            self.assertTrue(ground.closed)
            with self.assertRaises(MODULE.MoonDynamicGroundError):
                ground.update_mocap(data)

    def test_quantized_base_cache_skips_tile_calculation_and_writes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_dir:
            path = Path(temporary_dir) / "moonworld.bin"
            write_sparse_map(
                path,
                {
                    (2999, 2999): 2.75,
                    (3000, 3000): 1.25,
                },
            )
            data = FakeData()
            data.mocap_pos = CountingMocapArray(
                (MODULE.TILE_COUNT, 3)
            )
            data.mocap_quat = CountingMocapArray(
                (MODULE.TILE_COUNT, 4)
            )

            with MODULE.MoonDynamicGround(path, FakeModel()) as ground:
                with mock.patch.object(
                    MODULE,
                    "_round_array_away_from_zero",
                    wraps=MODULE._round_array_away_from_zero,
                ) as tile_round:
                    first = ground.update_mocap(data, base_xy=(0.0, 0.0))
                    self.assertFalse(first["cache_hit"])
                    self.assertEqual(tile_round.call_count, 4)
                    self.assertEqual(data.mocap_pos.write_count, 1)
                    self.assertEqual(data.mocap_quat.write_count, 1)

                    for _ in range(64):
                        cached = ground.update_mocap(
                            data,
                            base_xy=(-0.06, -0.06),
                        )
                    self.assertEqual(
                        cached["quantized_base_xy_m"],
                        first["quantized_base_xy_m"],
                    )
                    self.assertEqual(cached["base_xy_m"], [-0.06, -0.06])
                    self.assertAlmostEqual(
                        cached["local_ground_height_m"],
                        2.15,
                    )
                    self.assertTrue(cached["cache_hit"])
                    self.assertFalse(cached["cache_invalidated"])
                    self.assertFalse(cached["tiles_updated"])
                    self.assertFalse(cached["continuous_support_updated"])
                    self.assertEqual(tile_round.call_count, 4)
                    self.assertEqual(data.mocap_pos.write_count, 1)
                    self.assertEqual(data.mocap_quat.write_count, 1)
                    self.assertEqual(
                        data.mocap_pos.read_count
                        + data.mocap_quat.read_count,
                        64
                        * len(MODULE._CACHE_SENTINEL_TILE_INDICES)
                        * 7,
                    )

                    telemetry = ground.telemetry()
                    self.assertEqual(telemetry["update_count"], 65)
                    self.assertEqual(telemetry["tile_update_count"], 1)
                    self.assertEqual(telemetry["cache_hit_count"], 64)
                    self.assertEqual(
                        telemetry["cache_invalidation_count"],
                        0,
                    )
                    self.assertAlmostEqual(
                        telemetry["last_update"]["local_ground_height_m"],
                        2.15,
                    )

                    for invalid_base in (
                        (float("nan"), -0.06),
                        (-0.06, float("inf")),
                    ):
                        with self.subTest(base_xy=invalid_base):
                            with self.assertRaisesRegex(
                                MODULE.MoonDynamicGroundError,
                                "must be finite",
                            ):
                                ground.update_mocap(
                                    data,
                                    base_xy=invalid_base,
                                )
                    self.assertEqual(tile_round.call_count, 4)
                    self.assertEqual(data.mocap_pos.write_count, 1)
                    self.assertEqual(data.mocap_quat.write_count, 1)

                    moved = ground.update_mocap(
                        data,
                        base_xy=(0.06, -0.06),
                    )
                    self.assertFalse(moved["cache_hit"])
                    self.assertFalse(moved["cache_invalidated"])
                    self.assertTrue(moved["tiles_updated"])
                    self.assertNotEqual(
                        moved["quantized_base_xy_m"],
                        first["quantized_base_xy_m"],
                    )
                    self.assertEqual(tile_round.call_count, 8)
                    self.assertEqual(data.mocap_pos.write_count, 2)
                    self.assertEqual(data.mocap_quat.write_count, 2)

                    telemetry = ground.telemetry()
                    self.assertEqual(telemetry["update_count"], 66)
                    self.assertEqual(telemetry["tile_update_count"], 2)
                    self.assertEqual(telemetry["cache_hit_count"], 64)
                    self.assertEqual(
                        telemetry["cache_invalidation_count"],
                        0,
                    )

    def test_same_cell_cache_rewrites_after_external_mocap_reset(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_dir:
            path = Path(temporary_dir) / "moonworld.bin"
            write_sparse_map(
                path,
                {
                    (2992, 2992): -2.0,
                    (3000, 3000): 1.25,
                    (3007, 3007): 3.5,
                },
            )
            data = FakeData()
            with MODULE.MoonDynamicGround(path, FakeModel()) as ground:
                first = ground.update_mocap(data, base_xy=(0.0, 0.0))
                cached = ground.update_mocap(data, base_xy=(0.0, 0.0))
                self.assertFalse(first["cache_hit"])
                self.assertTrue(cached["cache_hit"])

                data.mocap_pos.fill(0.0)
                data.mocap_quat.fill(0.0)
                restored = ground.update_mocap(data, base_xy=(0.0, 0.0))

                self.assertFalse(restored["cache_hit"])
                self.assertTrue(restored["cache_invalidated"])
                self.assertTrue(restored["tiles_updated"])
                np.testing.assert_allclose(
                    data.mocap_pos[0],
                    [-0.8, -0.8, -2.0],
                    rtol=0.0,
                    atol=1e-12,
                )
                np.testing.assert_allclose(
                    data.mocap_pos[-1],
                    [0.7, 0.7, 3.5],
                    rtol=0.0,
                    atol=1e-12,
                )
                expected_quaternions = np.zeros((MODULE.TILE_COUNT, 4))
                expected_quaternions[:, 0] = 1.0
                np.testing.assert_array_equal(
                    data.mocap_quat,
                    expected_quaternions,
                )

                telemetry = ground.telemetry()
                self.assertEqual(telemetry["update_count"], 3)
                self.assertEqual(telemetry["tile_update_count"], 2)
                self.assertEqual(telemetry["cache_hit_count"], 1)
                self.assertEqual(telemetry["cache_invalidation_count"], 1)

    def test_same_cell_cache_rewrites_after_external_support_reset(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_dir:
            path = Path(temporary_dir) / "moonworld.bin"
            write_sparse_map(
                path,
                {
                    (2992, 2992): -2.0,
                    (2992, 2993): -1.0,
                    (3000, 3000): 1.25,
                },
            )
            model = FakeModel()
            data = FakeData()
            with MODULE.MoonDynamicGround(path, model) as ground:
                first = ground.update_mocap(data, base_xy=(0.0, 0.0))
                expected_support = model.hfield_data.copy()
                expected_position = model.geom_pos[ground.support_geom_id].copy()
                self.assertFalse(first["cache_hit"])

                # Index 1 is deliberately outside the old three-point sentinel.
                model.hfield_data[1] = 0.75
                model.geom_pos[ground.support_geom_id, 0] += 0.25
                restored = ground.update_mocap(data, base_xy=(0.0, 0.0))

                self.assertFalse(restored["cache_hit"])
                self.assertTrue(restored["cache_invalidated"])
                self.assertTrue(restored["continuous_support_updated"])
                np.testing.assert_array_equal(
                    model.hfield_data,
                    expected_support,
                )
                np.testing.assert_array_equal(
                    model.geom_pos[ground.support_geom_id],
                    expected_position,
                )

                telemetry = ground.telemetry()
                self.assertEqual(telemetry["update_count"], 2)
                self.assertEqual(telemetry["tile_update_count"], 2)
                self.assertEqual(telemetry["cache_hit_count"], 0)
                self.assertEqual(telemetry["cache_invalidation_count"], 1)


if __name__ == "__main__":
    unittest.main()
