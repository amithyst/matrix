from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock


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
            self.assertEqual(missing.settings.font_size, 13)
            path.write_text('{"version":1,"font_scale":9}', encoding="utf-8")
            invalid = MODULE.load_settings(path)
            self.assertEqual(invalid.status, "invalid")
            self.assertEqual(invalid.settings.font_scale, 1.0)
            self.assertEqual(invalid.settings.font_size, 13)
            self.assertIsNotNone(invalid.error)

    def test_duplicate_keys_are_rejected_instead_of_last_value_winning(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "ui.json"
            path.write_text(
                '{"version":2,"font_scale":1.0,"font_size":13,"font_size":22}',
                encoding="utf-8",
            )

            loaded = MODULE.load_settings(path)

            self.assertEqual(loaded.status, "invalid")
            self.assertEqual(loaded.settings, MODULE.UiSettings())
            self.assertIn("duplicate", loaded.error or "")

    def test_atomic_save_round_trips_with_private_permissions(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "config/matrix/hosts/trna/ui-settings.json"
            settings = MODULE.UiSettings(font_scale=1.3, font_size=19)
            MODULE.atomic_save_settings(path, settings)
            self.assertEqual(
                json.loads(path.read_text(encoding="utf-8")),
                {
                    "font_scale": 1.3,
                    "font_size": 19,
                    "version": 2,
                },
            )
            self.assertEqual(path.stat().st_mode & 0o777, 0o600)
            loaded = MODULE.load_settings(path)
            self.assertEqual(loaded.status, "loaded")
            self.assertEqual(loaded.settings, settings)

    def test_default_path_is_host_scoped_and_legacy_file_can_seed_v2(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self.assertEqual(
                MODULE.default_settings_file("trna", config_home=root),
                root / "matrix/hosts/trna/ui-settings.json",
            )
            legacy = MODULE.legacy_settings_file(config_home=root)
            legacy.parent.mkdir(parents=True)
            legacy.write_text('{"version":1,"font_scale":1.3}', encoding="utf-8")

            loaded = MODULE.load_settings_with_legacy_fallback(
                MODULE.default_settings_file("trna", config_home=root)
            )

            self.assertEqual(loaded.status, "loaded_legacy")
            self.assertEqual(loaded.settings.font_scale, 1.3)
            self.assertEqual(loaded.settings.font_size, MODULE.font_size_for_scale(1.3))

    def test_launcher_matrix_profile_selects_the_host_file(self) -> None:
        with tempfile.TemporaryDirectory() as temporary, mock.patch.dict(
            os.environ,
            {"MATRIX_PROFILE": "trna", "PROFILE": "legacy-profile"},
            clear=True,
        ):
            root = Path(temporary)

            self.assertEqual(
                MODULE.default_settings_file(config_home=root),
                root / "matrix/hosts/trna/ui-settings.json",
            )

    def test_explicit_and_host_environment_profiles_override_launcher_profile(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary, mock.patch.dict(
            os.environ,
            {
                "MATRIX_HOST_PROFILE": "host-override",
                "MATRIX_PROFILE": "trna",
                "PROFILE": "legacy-profile",
            },
            clear=True,
        ):
            root = Path(temporary)

            self.assertEqual(
                MODULE.default_settings_file(config_home=root),
                root / "matrix/hosts/host-override/ui-settings.json",
            )
            self.assertEqual(
                MODULE.default_settings_file("explicit-host", config_home=root),
                root / "matrix/hosts/explicit-host/ui-settings.json",
            )

    def test_invalid_selected_profile_fails_closed_without_fallback(self) -> None:
        cases = (
            ({"MATRIX_HOST_PROFILE": "../escape", "MATRIX_PROFILE": "trna"}, None),
            ({"MATRIX_PROFILE": ""}, None),
            ({"MATRIX_HOST_PROFILE": "trna"}, "bad/profile"),
        )
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for environment, explicit in cases:
                with self.subTest(environment=environment, explicit=explicit), mock.patch.dict(
                    os.environ,
                    environment,
                    clear=True,
                ), self.assertRaises(ValueError):
                    MODULE.default_settings_file(explicit, config_home=root)

    def test_font_size_nine_survives_host_scoped_restarts(self) -> None:
        with tempfile.TemporaryDirectory() as temporary, mock.patch.dict(
            os.environ,
            {"MATRIX_PROFILE": "trna"},
            clear=True,
        ):
            root = Path(temporary)
            path = MODULE.default_settings_file(config_home=root)
            MODULE.atomic_save_settings(path, MODULE.UiSettings(font_size=9))

            first_launch = MODULE.load_settings_with_legacy_fallback(path)
            restarted = MODULE.load_settings_with_legacy_fallback(path)

            self.assertEqual(first_launch.status, "loaded")
            self.assertEqual(first_launch.settings.font_size, 9)
            self.assertEqual(restarted, first_launch)

    def test_valid_host_is_authoritative_across_environment_and_reload(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            host = MODULE.default_settings_file("trna", config_home=root)
            legacy = MODULE.legacy_settings_file(config_home=root)
            legacy.parent.mkdir(parents=True)
            legacy.write_text('{"version":1,"font_scale":1.5}', encoding="utf-8")
            expected = MODULE.UiSettings(font_scale=0.9, font_size=7)
            MODULE.atomic_save_settings(host, expected)

            with mock.patch.dict(
                os.environ,
                {
                    "MATRIX_HOST_PROFILE": "other-host",
                    "PROFILE": "legacy-profile",
                },
                clear=False,
            ):
                first_launch = MODULE.load_settings_with_legacy_fallback(host)
                restarted = MODULE.load_settings_with_legacy_fallback(host)

            self.assertEqual(first_launch.status, "loaded")
            self.assertEqual(first_launch.settings, expected)
            self.assertEqual(restarted, first_launch)

    def test_font_size_steps_are_discrete_and_bounded(self) -> None:
        self.assertEqual(MODULE.step_font_size(13, 1), 14)
        self.assertEqual(MODULE.step_font_size(MODULE.MIN_FONT_SIZE, -1), 1)
        self.assertEqual(MODULE.step_font_size(MODULE.MAX_FONT_SIZE, 1), 22)


if __name__ == "__main__":
    unittest.main()
