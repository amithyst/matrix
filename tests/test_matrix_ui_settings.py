from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts" / "matrix_ui_settings.py"
SPEC = importlib.util.spec_from_file_location("matrix_ui_settings_tested", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class MatrixUiSettingsTest(unittest.TestCase):
    def test_scale_steps_are_discrete_and_bounded(self) -> None:
        self.assertEqual(MODULE.step_font_scale(1.0, 1), 1.1)
        self.assertEqual(MODULE.step_font_scale(1.0, -1), 0.9)
        self.assertEqual(MODULE.step_font_scale(MODULE.MIN_FONT_SCALE, -1), 0.8)
        self.assertEqual(MODULE.step_font_scale(MODULE.MAX_FONT_SCALE, 1), 1.5)
        with self.assertRaisesRegex(ValueError, "must use one of"):
            MODULE.canonical_font_scale(1.05)

    def test_missing_and_invalid_files_fail_safe_to_default(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "ui.json"
            missing = MODULE.load_settings(path)
            self.assertEqual(missing.status, "missing")
            self.assertEqual(missing.settings.font_scale, 1.0)
            path.write_text('{"version":1,"font_scale":9}', encoding="utf-8")
            invalid = MODULE.load_settings(path)
            self.assertEqual(invalid.status, "invalid")
            self.assertEqual(invalid.settings.font_scale, 1.0)
            self.assertIsNotNone(invalid.error)

    def test_atomic_save_round_trips_with_private_permissions(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "config/matrix/ui-settings.json"
            settings = MODULE.UiSettings(font_scale=1.3)
            MODULE.atomic_save_settings(path, settings)
            self.assertEqual(
                json.loads(path.read_text(encoding="utf-8")),
                {"font_scale": 1.3, "version": 1},
            )
            self.assertEqual(path.stat().st_mode & 0o777, 0o600)
            loaded = MODULE.load_settings(path)
            self.assertEqual(loaded.status, "loaded")
            self.assertEqual(loaded.settings, settings)


if __name__ == "__main__":
    unittest.main()
