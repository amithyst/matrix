from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import unittest

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "matrix_render_protocol.py"
SPEC = importlib.util.spec_from_file_location("matrix_render_protocol", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class MatrixRenderProtocolTest(unittest.TestCase):
    def test_g1_packet_matches_observed_matrix_wire_size(self) -> None:
        payload = MODULE.pack_mujoco_state(
            12.5,
            np.arange(36, dtype=np.float64),
            np.arange(35, dtype=np.float64) * -0.5,
            np.arange(29, dtype=np.float64) * 0.25,
        )
        self.assertEqual(len(payload), 820)
        self.assertEqual(MODULE.packet_size(nq=36, nv=35, nu=29), 820)

        state = MODULE.unpack_mujoco_state(payload)
        self.assertEqual(state.sim_time, 12.5)
        np.testing.assert_array_equal(state.qpos, np.arange(36, dtype=np.float64))
        np.testing.assert_array_equal(
            state.qvel, np.arange(35, dtype=np.float64) * -0.5
        )
        np.testing.assert_array_equal(
            state.ctrl, np.arange(29, dtype=np.float64) * 0.25
        )

    def test_rejects_nonfinite_state(self) -> None:
        with self.assertRaises(ValueError):
            MODULE.pack_mujoco_state(0.0, [0.0, float("nan")], [], [])

    def test_rejects_trailing_bytes(self) -> None:
        payload = MODULE.pack_mujoco_state(0.0, [1.0], [2.0], [3.0])
        with self.assertRaisesRegex(ValueError, "trailing bytes"):
            MODULE.unpack_mujoco_state(payload + b"extra")


if __name__ == "__main__":
    unittest.main()
