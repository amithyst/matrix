from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import stat
import tempfile
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts/matrix_mouse_settings.py"
SPEC = importlib.util.spec_from_file_location("matrix_mouse_settings_tested", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
os.sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class MouseSettingsFileTest(unittest.TestCase):
    def test_missing_and_corrupt_files_fall_back_to_local_one_x(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "mouse.json"
            missing = MODULE.load_settings(path)
            self.assertEqual(missing.status, "missing")
            self.assertEqual(missing.settings.profile, "local")
            self.assertEqual(missing.settings.effective_scale, 1.0)

            path.write_text("{", encoding="utf-8")
            corrupt = MODULE.load_settings(path)
            self.assertEqual(corrupt.status, "invalid")
            self.assertEqual(corrupt.settings.profile, "local")
            self.assertEqual(corrupt.settings.effective_scale, 1.0)
            self.assertIsNotNone(corrupt.error)

            path.write_text(
                json.dumps(
                    {"version": 1, "profile": "remote", "speed_scale": 2.0}
                ),
                encoding="utf-8",
            )
            out_of_range = MODULE.load_settings(path)
            self.assertEqual(out_of_range.status, "invalid")
            self.assertEqual(out_of_range.settings.effective_scale, 1.0)

    def test_remote_file_is_atomic_private_and_strictly_versioned(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "nested/mouse.json"
            settings = MODULE.MouseSettings(profile="remote", speed_scale=0.4)
            MODULE.atomic_save_settings(path, settings)

            self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o600)
            loaded = MODULE.load_settings(path)
            self.assertEqual(loaded.status, "loaded")
            self.assertEqual(loaded.settings, settings)
            self.assertEqual(loaded.settings.effective_scale, 0.4)
            self.assertEqual(
                set(json.loads(path.read_text(encoding="utf-8"))),
                {"version", "profile", "speed_scale"},
            )

    def test_local_keeps_remote_preset_but_effective_scale_is_one(self) -> None:
        settings = MODULE.MouseSettings(profile="local", speed_scale=0.3)
        self.assertEqual(settings.speed_scale, 0.3)
        self.assertEqual(settings.effective_scale, 1.0)


if __name__ == "__main__":
    unittest.main()
