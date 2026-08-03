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

    def test_height_filter_accepts_playable_local_limited_relief(self) -> None:
        self.assertEqual(
            MODULE.normalize_height_filter("playable-local"),
            "playable-local",
        )
        self.assertEqual(MODULE.normalize_height_filter("limited"), "playable-local")

    def test_playable_local_filter_preserves_small_relief_and_clamps_steps(self) -> None:
        filtered = MODULE.apply_playable_local_height_filter(
            np.asarray([-0.35, -0.05, 0.0, 0.08, 0.4], dtype=np.float32),
            0.0,
            max_delta_m=0.18,
        )

        np.testing.assert_allclose(filtered, [-0.18, -0.05, 0.0, 0.08, 0.18])
        self.assertGreater(float(np.max(filtered) - np.min(filtered)), 0.0)

    def test_playable_local_height_delta_validates_positive_finite_values(self) -> None:
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assertAlmostEqual(
                MODULE.normalize_playable_local_height_delta(),
                0.04,
            )
        self.assertAlmostEqual(
            MODULE.normalize_playable_local_height_delta("0.12"),
            0.12,
        )
        with self.assertRaisesRegex(MODULE.MoonDynamicGroundError, "must be positive"):
            MODULE.normalize_playable_local_height_delta("0")

    def test_collision_default_uses_mainline_rolling_tiles(self) -> None:
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
            "cleanup_runtime_generated_integrity_files",
            run_sim,
        )
        self.assertIn(
            "Binaries/Linux/MUJOCO_LOG.TXT",
            run_sim,
        )
        self.assertIn(
            'MATRIX_MOON_DYNAMIC_GROUND_ROOT_CLEARANCE:-0.85',
            run_sim,
        )

    def test_run_sim_moon_default_uses_verified_mainline_plain(self) -> None:
        run_sim = (REPO_ROOT / "scripts" / "run_sim.sh").read_text(encoding="utf-8")

        self.assertIn("resolve_default_moon_spawn_args", run_sim)
        self.assertIn("MoonWorld verified mainline plain spawn selected", run_sim)
        self.assertIn("-94.7", run_sim)
        self.assertIn("-65.6", run_sim)
        self.assertIn("-5.251562023162842", run_sim)
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
