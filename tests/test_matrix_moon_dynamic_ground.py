from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "matrix_moon_dynamic_ground.py"
SPEC = importlib.util.spec_from_file_location("matrix_moon_dynamic_ground", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class MatrixMoonDynamicGroundTest(unittest.TestCase):
    def _write_fake_height_map(
        self,
        directory: Path,
        values: np.ndarray,
    ) -> Path:
        path = directory / "moonworld.bin"
        path.write_bytes(np.asarray(values, dtype=MODULE.MAP_DTYPE).tobytes(order="C"))
        return path

    def test_height_filter_default_preserves_raw_moon_terrain(self) -> None:
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assertEqual(MODULE.normalize_height_filter(), "raw")

    def test_collision_default_uses_rolling_tiles(self) -> None:
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assertEqual(
                MODULE.normalize_collision_mode(),
                MODULE.COLLISION_MODE_ROLLING_TILES,
            )

    def test_run_sim_moon_default_does_not_flatten_height_map(self) -> None:
        run_sim = (REPO_ROOT / "scripts" / "run_sim.sh").read_text(encoding="utf-8")

        self.assertIn(
            'MATRIX_MOON_DYNAMIC_GROUND_HEIGHT_FILTER:-raw',
            run_sim,
        )
        self.assertNotIn(
            'MATRIX_MOON_DYNAMIC_GROUND_HEIGHT_FILTER:-flat-anchor',
            run_sim,
        )
        self.assertIn(
            'MATRIX_MOON_DYNAMIC_GROUND_COLLISION_MODE:-rolling-mocap-tiles-v1',
            run_sim,
        )
        self.assertIn(
            'MATRIX_MOON_DYNAMIC_GROUND_ROOT_CLEARANCE:-0.78696775',
            run_sim,
        )

    def test_run_sim_moon_default_uses_verified_playable_route(self) -> None:
        run_sim = (REPO_ROOT / "scripts" / "run_sim.sh").read_text(encoding="utf-8")

        self.assertIn("resolve_default_moon_spawn_args", run_sim)
        self.assertIn("MoonWorld verified playable spawn route selected", run_sim)
        self.assertIn("24.43", run_sim)
        self.assertIn("110.77", run_sim)
        self.assertIn("-5.3145942731628422", run_sim)
        self.assertIn("3.141592653589793", run_sim)
        self.assertIn("MATRIX_MOON_SPAWN_X/Y/Z/YAW are all-or-none", run_sim)

    def test_moon_default_spawn_samples_raw_ground_height(self) -> None:
        values = np.full((4, 4), -2.0, dtype=MODULE.MAP_DTYPE)
        with tempfile.TemporaryDirectory() as temporary:
            path = self._write_fake_height_map(Path(temporary), values)
            with (
                mock.patch.object(MODULE, "MAP_SIDE_SAMPLES", 4),
                mock.patch.object(MODULE, "MAP_SAMPLE_COUNT", 16),
                mock.patch.object(MODULE, "MAP_SIZE_BYTES", values.nbytes),
                mock.patch.object(MODULE, "MAP_HALF_EXTENT_M", 0.2),
            ):
                resolved = MODULE.resolve_spawn_pose_for_moon_dynamic_ground(
                    map_path=path,
                    fallback_xyz=(0.0, 0.0, None),
                    root_clearance_m=0.75,
                )

        self.assertEqual(resolved["source"], "moon_map_default")
        self.assertAlmostEqual(resolved["raw_ground_height_m"], -2.0)
        self.assertAlmostEqual(resolved["fallback_ground_height_m"], -2.0)
        self.assertAlmostEqual(resolved["z"], -1.25)

    def test_moon_spawn_rebases_valid_resume_z_to_raw_ground(self) -> None:
        values = np.full((4, 4), 2.0, dtype=MODULE.MAP_DTYPE)
        with tempfile.TemporaryDirectory() as temporary:
            path = self._write_fake_height_map(Path(temporary), values)
            with (
                mock.patch.object(MODULE, "MAP_SIDE_SAMPLES", 4),
                mock.patch.object(MODULE, "MAP_SAMPLE_COUNT", 16),
                mock.patch.object(MODULE, "MAP_SIZE_BYTES", values.nbytes),
                mock.patch.object(MODULE, "MAP_HALF_EXTENT_M", 0.2),
            ):
                resolved = MODULE.resolve_spawn_pose_for_moon_dynamic_ground(
                    map_path=path,
                    pose_xyz=(0.0, 0.0, 2.9),
                    yaw_rad=0.25,
                    source="last_exit",
                )

        self.assertEqual(resolved["source"], "moon_terrain_rebased_last_exit")
        self.assertEqual(resolved["x"], 0.0)
        self.assertEqual(resolved["y"], 0.0)
        self.assertAlmostEqual(resolved["z"], 2.0 + MODULE.DEFAULT_ROOT_CLEARANCE_M)
        self.assertEqual(resolved["yaw_rad"], 0.25)
        self.assertAlmostEqual(resolved["input_clearance_m"], 0.9)

    def test_moon_spawn_rejects_polluted_resume_z_to_safe_default(self) -> None:
        values = np.full((4, 4), -1.0, dtype=MODULE.MAP_DTYPE)
        with tempfile.TemporaryDirectory() as temporary:
            path = self._write_fake_height_map(Path(temporary), values)
            with (
                mock.patch.object(MODULE, "MAP_SIDE_SAMPLES", 4),
                mock.patch.object(MODULE, "MAP_SAMPLE_COUNT", 16),
                mock.patch.object(MODULE, "MAP_SIZE_BYTES", values.nbytes),
                mock.patch.object(MODULE, "MAP_HALF_EXTENT_M", 0.2),
            ):
                resolved = MODULE.resolve_spawn_pose_for_moon_dynamic_ground(
                    map_path=path,
                    pose_xyz=(5.0, 4.0, 0.8),
                    yaw_rad=0.25,
                    source="last_exit",
                )

        self.assertEqual(resolved["source"], "moon_rejected_last_exit_clearance")
        self.assertEqual(resolved["x"], MODULE.DEFAULT_SPAWN_X_M)
        self.assertEqual(resolved["y"], MODULE.DEFAULT_SPAWN_Y_M)
        self.assertAlmostEqual(resolved["z"], -1.0 + MODULE.DEFAULT_ROOT_CLEARANCE_M)
        self.assertEqual(resolved["yaw_rad"], MODULE.DEFAULT_SPAWN_YAW_RAD)
        self.assertAlmostEqual(resolved["input_clearance_m"], 1.8)
