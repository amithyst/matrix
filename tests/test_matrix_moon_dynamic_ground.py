from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import unittest
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "matrix_moon_dynamic_ground.py"
SPEC = importlib.util.spec_from_file_location("matrix_moon_dynamic_ground", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class MatrixMoonDynamicGroundTest(unittest.TestCase):
    def test_height_filter_default_preserves_raw_moon_terrain(self) -> None:
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assertEqual(MODULE.normalize_height_filter(), "raw")

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

