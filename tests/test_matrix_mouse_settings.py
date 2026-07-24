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
    def test_remote_presets_are_exactly_the_nineteen_discrete_values(self) -> None:
        expected = tuple(value / 100 for value in range(1, 11)) + tuple(
            value / 10 for value in range(2, 11)
        )
        self.assertEqual(len(expected), 19)
        self.assertEqual(MODULE.REMOTE_SPEED_SCALE_STEPS, expected)
        self.assertEqual(expected[9:11], (0.10, 0.20))

    def test_canonical_scale_accepts_presets_and_rejects_untrusted_values(self) -> None:
        for value in (0.01, 0.10, 0.20, 0.40, 1.00):
            with self.subTest(accepted=value):
                self.assertEqual(MODULE.canonical_remote_speed_scale(value), value)

        for value in (0.00, 0.11, 0.15, 1.01, True, float("nan")):
            with self.subTest(rejected=value), self.assertRaises(ValueError):
                MODULE.canonical_remote_speed_scale(value)

    def test_discrete_stepper_traverses_without_drift_and_clamps_endpoints(self) -> None:
        expected = MODULE.REMOTE_SPEED_SCALE_STEPS
        forward = [expected[0]]
        while forward[-1] != expected[-1]:
            forward.append(MODULE.step_remote_speed_scale(forward[-1], 1))
        self.assertEqual(tuple(forward), expected)
        self.assertEqual(MODULE.step_remote_speed_scale(expected[-1], 1), 1.0)

        backward = [expected[-1]]
        while backward[-1] != expected[0]:
            backward.append(MODULE.step_remote_speed_scale(backward[-1], -1))
        self.assertEqual(tuple(backward), tuple(reversed(expected)))
        self.assertEqual(MODULE.step_remote_speed_scale(expected[0], -1), 0.01)
        self.assertEqual(MODULE.step_remote_speed_scale(0.10, 1), 0.20)
        self.assertEqual(MODULE.step_remote_speed_scale(0.20, -1), 0.10)
        for direction in (0, True, 1.0):
            with self.subTest(direction=direction), self.assertRaises(ValueError):
                MODULE.step_remote_speed_scale(0.10, direction)

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

            path.write_text(
                json.dumps(
                    {"version": 1, "profile": "remote", "speed_scale": 0.15}
                ),
                encoding="utf-8",
            )
            off_table = MODULE.load_settings(path)
            self.assertEqual(off_table.status, "invalid")
            self.assertEqual(off_table.settings.profile, "local")
            self.assertEqual(off_table.settings.effective_scale, 1.0)

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

    def test_low_remote_preset_round_trips_atomically(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "nested/mouse.json"
            settings = MODULE.MouseSettings(profile="remote", speed_scale=0.01)
            MODULE.atomic_save_settings(path, settings)

            loaded = MODULE.load_settings(path)
            self.assertEqual(loaded.status, "loaded")
            self.assertEqual(loaded.settings, settings)
            self.assertEqual(loaded.settings.effective_scale, 0.01)

    def test_default_path_is_host_scoped_and_legacy_file_is_preserved(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            target = MODULE.default_settings_file("trna", config_home=root)
            self.assertEqual(
                target,
                root / "matrix/hosts/trna/mouse-control.json",
            )
            legacy = MODULE.legacy_settings_file(config_home=root)
            legacy.parent.mkdir(parents=True)
            legacy.write_text(
                '{"version":1,"profile":"remote","speed_scale":0.4}',
                encoding="utf-8",
            )

            loaded = MODULE.load_settings_with_legacy_fallback(target)

            self.assertEqual(loaded.status, "loaded_legacy")
            self.assertEqual(loaded.settings.profile, "remote")
            self.assertEqual(loaded.settings.effective_scale, 0.4)


if __name__ == "__main__":
    unittest.main()
