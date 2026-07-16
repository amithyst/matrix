from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "run_matrix_sonic.py"
SPEC = importlib.util.spec_from_file_location("run_matrix_sonic", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class MatrixSonicRuntimeTest(unittest.TestCase):
    def test_root_up_z_is_one_for_upright_quaternion(self) -> None:
        self.assertAlmostEqual(MODULE._root_up_z([0, 0, 0, 1, 0, 0, 0]), 1.0)

    def test_root_up_z_is_negative_for_upside_down_quaternion(self) -> None:
        self.assertAlmostEqual(MODULE._root_up_z([0, 0, 0, 0, 1, 0, 0]), -1.0)

    def test_applies_optional_world_spawn_pose(self) -> None:
        qpos = [0.0, 0.0, 0.793, 1.0, 0.0, 0.0, 0.0]
        MODULE._apply_spawn_pose(
            qpos, x=124.0, y=-105.05, z=None, yaw=math.pi / 2.0
        )
        self.assertEqual(qpos[:3], [124.0, -105.05, 0.793])
        self.assertAlmostEqual(qpos[3], math.sqrt(0.5))
        self.assertAlmostEqual(qpos[6], math.sqrt(0.5))

    def test_preserves_root_quaternion_without_spawn_yaw(self) -> None:
        qpos = [0.0, 0.0, 0.793, 0.5, 0.5, 0.5, 0.5]
        MODULE._apply_spawn_pose(qpos, x=1.0, y=None, z=None, yaw=None)
        self.assertEqual(qpos[3:7], [0.5, 0.5, 0.5, 0.5])


if __name__ == "__main__":
    unittest.main()
