#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import stat
import sys
import tempfile
import unittest
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts/matrix_motion_settings.py"
SPEC = importlib.util.spec_from_file_location("matrix_motion_settings_tested", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def path(gear: str, field: str) -> str:
    return f"control.motion.gears.{gear}.{field}"


class MotionSettingsValueTest(unittest.TestCase):
    def test_defaults_cover_three_base_and_double_tap_tiers(self) -> None:
        settings = MODULE.MotionSettings()
        self.assertEqual(settings.revision, 0)
        self.assertEqual(settings.slow_speed_mps, 0.10)
        self.assertEqual(settings.slow_double_tap_speed_mps, 0.20)
        self.assertEqual(settings.walk_speed_mps, 0.80)
        self.assertEqual(settings.walk_double_tap_speed_mps, 1.00)
        self.assertEqual(settings.run_speed_mps, 2.50)
        self.assertEqual(settings.run_double_tap_speed_mps, 2.75)
        self.assertEqual(settings.max_turn_rate_rad_s, 2.50)
        self.assertEqual(settings.keyboard_look_rate_deg_s, 120.0)
        self.assertEqual(len(MODULE.MOTION_SETTING_PATHS), 8)

    def test_strict_mapping_round_trip(self) -> None:
        settings = MODULE.MotionSettings(
            revision=7,
            slow_speed_mps=0.15,
            slow_double_tap_speed_mps=0.25,
            walk_speed_mps=1.10,
            walk_double_tap_speed_mps=1.30,
            run_speed_mps=3.00,
            run_double_tap_speed_mps=3.50,
        )
        mapping = settings.to_mapping()
        self.assertEqual(
            set(mapping),
            {
                "version",
                "revision",
                "gears",
                "max_turn_rate_rad_s",
                "camera",
            },
        )
        self.assertEqual(
            mapping["camera"],
            {"keyboard_look_rate_deg_s": 120.0},
        )
        self.assertEqual(set(mapping["gears"]), {"slow", "walk", "run"})
        for gear in MODULE.GEARS:
            self.assertEqual(
                set(mapping["gears"][gear]),
                {"speed_mps", "double_tap_speed_mps"},
            )
        self.assertEqual(MODULE.MotionSettings.from_mapping(mapping), settings)

    def test_schema_version_revision_and_nested_fields_are_exact(self) -> None:
        valid = MODULE.MotionSettings().to_mapping()
        mutations = []

        extra_top = json.loads(json.dumps(valid))
        extra_top["extra"] = 1
        mutations.append(extra_top)
        missing_top = json.loads(json.dumps(valid))
        del missing_top["revision"]
        mutations.append(missing_top)
        wrong_version = json.loads(json.dumps(valid))
        wrong_version["version"] = 2
        mutations.append(wrong_version)
        bool_version = json.loads(json.dumps(valid))
        bool_version["version"] = True
        mutations.append(bool_version)
        bad_revision = json.loads(json.dumps(valid))
        bad_revision["revision"] = -1
        mutations.append(bad_revision)
        bool_revision = json.loads(json.dumps(valid))
        bool_revision["revision"] = True
        mutations.append(bool_revision)
        missing_gear = json.loads(json.dumps(valid))
        del missing_gear["gears"]["walk"]
        mutations.append(missing_gear)
        extra_gear = json.loads(json.dumps(valid))
        extra_gear["gears"]["crawl"] = dict(extra_gear["gears"]["slow"])
        mutations.append(extra_gear)
        missing_field = json.loads(json.dumps(valid))
        del missing_field["gears"]["run"]["double_tap_speed_mps"]
        mutations.append(missing_field)
        extra_camera_field = json.loads(json.dumps(valid))
        extra_camera_field["camera"]["extra"] = 1
        mutations.append(extra_camera_field)
        null_camera = json.loads(json.dumps(valid))
        null_camera["camera"] = None
        mutations.append(null_camera)

        for value in mutations:
            with self.subTest(value=value), self.assertRaises(MODULE.MotionSettingsError):
                MODULE.MotionSettings.from_mapping(value)

    def test_legacy_v1_mapping_without_camera_uses_safe_default(self) -> None:
        legacy = MODULE.MotionSettings().to_mapping()
        del legacy["camera"]
        loaded = MODULE.MotionSettings.from_mapping(legacy)
        self.assertEqual(
            loaded.keyboard_look_rate_deg_s,
            MODULE.DEFAULT_KEYBOARD_LOOK_RATE_DEG_S,
        )

    def test_each_value_must_be_finite_numeric_and_inside_native_tier(self) -> None:
        cases = (
            {"slow_speed_mps": 0.09},
            {"slow_double_tap_speed_mps": 0.81},
            {"walk_speed_mps": 0.79},
            {"walk_double_tap_speed_mps": 2.51},
            {"run_speed_mps": 2.49},
            {"run_double_tap_speed_mps": 7.51},
            {"slow_speed_mps": True},
            {"walk_speed_mps": float("nan")},
            {"run_speed_mps": float("inf")},
            {"max_turn_rate_rad_s": 2.75},
            {"max_turn_rate_rad_s": True},
            {"keyboard_look_rate_deg_s": 29.0},
            {"keyboard_look_rate_deg_s": 361.0},
            {"keyboard_look_rate_deg_s": True},
        )
        for values in cases:
            with self.subTest(values=values), self.assertRaises(
                MODULE.MotionSettingsError
            ):
                MODULE.MotionSettings(**values)

    def test_double_tap_speed_must_be_strictly_above_its_base(self) -> None:
        for values in (
            {"slow_speed_mps": 0.30, "slow_double_tap_speed_mps": 0.20},
            {"slow_speed_mps": 0.20, "slow_double_tap_speed_mps": 0.20},
            {"walk_speed_mps": 1.20, "walk_double_tap_speed_mps": 1.00},
            {"walk_speed_mps": 1.00, "walk_double_tap_speed_mps": 1.00},
            {"run_speed_mps": 3.00, "run_double_tap_speed_mps": 2.75},
            {"run_speed_mps": 2.75, "run_double_tap_speed_mps": 2.75},
        ):
            with self.subTest(values=values), self.assertRaisesRegex(
                MODULE.MotionSettingsError, "greater than"
            ):
                MODULE.MotionSettings(**values)

    def test_value_lookup_and_replacement_accept_only_whitelisted_paths(self) -> None:
        settings = MODULE.MotionSettings()
        slow = path("slow", "speed_mps")
        self.assertEqual(settings.value_for_path(slow), 0.10)
        self.assertEqual(
            settings.value_for_path(MODULE.MAX_TURN_RATE_PATH),
            MODULE.DEFAULT_MAX_TURN_RATE_RAD_S,
        )
        replacement = settings.with_value(slow, 0.15, revision=3)
        self.assertEqual(replacement.slow_speed_mps, 0.15)
        self.assertEqual(replacement.revision, 3)
        self.assertEqual(settings.slow_speed_mps, 0.10)
        turn_replacement = settings.with_value(
            MODULE.MAX_TURN_RATE_PATH,
            2.25,
            revision=4,
        )
        self.assertEqual(turn_replacement.max_turn_rate_rad_s, 2.25)
        self.assertEqual(turn_replacement.revision, 4)
        self.assertEqual(
            settings.value_for_path(MODULE.KEYBOARD_LOOK_RATE_PATH),
            MODULE.DEFAULT_KEYBOARD_LOOK_RATE_DEG_S,
        )
        look_replacement = settings.with_value(
            MODULE.KEYBOARD_LOOK_RATE_PATH,
            180.0,
            revision=5,
        )
        self.assertEqual(look_replacement.keyboard_look_rate_deg_s, 180.0)
        self.assertEqual(look_replacement.revision, 5)
        for invalid in (
            "control.motion.gears.slow.unknown",
            "control.motion.gears.crawl.speed_mps",
            "../slow.speed_mps",
            1,
        ):
            with self.subTest(invalid=invalid), self.assertRaisesRegex(
                MODULE.MotionSettingsError, "unsupported"
            ):
                settings.value_for_path(invalid)


class MotionSettingsPathTest(unittest.TestCase):
    def test_default_path_is_host_scoped(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self.assertEqual(
                MODULE.default_settings_file("trna", config_home=root),
                root / "matrix/hosts/trna/motion-control.json",
            )
            with mock.patch.dict(
                os.environ,
                {"MATRIX_HOST_PROFILE": "heyuan", "XDG_CONFIG_HOME": str(root)},
                clear=False,
            ):
                self.assertEqual(
                    MODULE.default_settings_file(),
                    root / "matrix/hosts/heyuan/motion-control.json",
                )

    def test_profile_rejects_unsafe_or_unbounded_names(self) -> None:
        for profile in (
            "",
            ".hidden",
            "../trna",
            "trna/other",
            "trna other",
            "a" * 65,
            True,
            None,
        ):
            with self.subTest(profile=profile), self.assertRaises(
                MODULE.MotionSettingsError
            ):
                MODULE.canonical_host_profile(profile)
        for profile in ("trna", "TRNA-01", "lab_a", "lab.a"):
            with self.subTest(profile=profile):
                self.assertEqual(MODULE.canonical_host_profile(profile), profile)

    def test_omitted_profile_requires_environment_value(self) -> None:
        with mock.patch.dict(os.environ, {}, clear=True), self.assertRaisesRegex(
            MODULE.MotionSettingsError, "required"
        ):
            MODULE.default_settings_file()


class MotionSettingsPersistenceTest(unittest.TestCase):
    def test_missing_and_invalid_files_fail_safe_to_defaults(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            file_path = root / "motion.json"
            missing = MODULE.load_settings(file_path)
            self.assertEqual(missing.status, "missing")
            self.assertEqual(missing.settings, MODULE.MotionSettings())

            invalid_payloads = (
                "{",
                '{"version":1,"version":1,"revision":0,"gears":{}}',
                json.dumps(
                    {
                        "version": 1,
                        "revision": 0,
                        "gears": {
                            "slow": {
                                "speed_mps": float("nan"),
                                "double_tap_speed_mps": 0.2,
                            },
                            "walk": {
                                "speed_mps": 0.8,
                                "double_tap_speed_mps": 1.0,
                            },
                            "run": {
                                "speed_mps": 2.5,
                                "double_tap_speed_mps": 2.75,
                            },
                        },
                    }
                ),
                json.dumps(
                    {
                        **MODULE.MotionSettings().to_mapping(),
                        "revision": "0",
                    }
                ),
            )
            for payload in invalid_payloads:
                with self.subTest(payload=payload):
                    file_path.write_text(payload, encoding="utf-8")
                    loaded = MODULE.load_settings(file_path)
                    self.assertEqual(loaded.status, "invalid")
                    self.assertIsNotNone(loaded.error)
                    self.assertEqual(loaded.settings, MODULE.MotionSettings())

    def test_atomic_save_is_private_strict_and_fsyncs_file_and_directory(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            file_path = Path(temporary) / "nested/motion.json"
            settings = MODULE.MotionSettings(revision=9, run_double_tap_speed_mps=3.5)
            with mock.patch.object(
                MODULE.os, "fsync", wraps=MODULE.os.fsync
            ) as fsync:
                MODULE.atomic_save_settings(file_path, settings)
            self.assertGreaterEqual(fsync.call_count, 2)
            self.assertEqual(stat.S_IMODE(file_path.stat().st_mode), 0o600)
            self.assertEqual(
                json.loads(file_path.read_text(encoding="utf-8")),
                settings.to_mapping(),
            )
            self.assertEqual(MODULE.load_settings(file_path).settings, settings)
            self.assertFalse(tuple(file_path.parent.glob(f".{file_path.name}.*")))

    def test_atomic_save_replaces_a_preexisting_broad_mode(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            file_path = Path(temporary) / "motion.json"
            file_path.write_text("old", encoding="utf-8")
            file_path.chmod(0o644)
            MODULE.atomic_save_settings(file_path, MODULE.MotionSettings())
            self.assertEqual(stat.S_IMODE(file_path.stat().st_mode), 0o600)

    def test_atomic_save_requires_an_absolute_path(self) -> None:
        with self.assertRaisesRegex(ValueError, "absolute"):
            MODULE.atomic_save_settings(Path("motion.json"), MODULE.MotionSettings())


class MotionSettingsStepTest(unittest.TestCase):
    def test_step_helper_uses_drift_free_tier_steps(self) -> None:
        settings = MODULE.MotionSettings()
        cases = (
            (path("slow", "speed_mps"), 1, 0.15),
            (path("slow", "double_tap_speed_mps"), -1, 0.15),
            (path("walk", "speed_mps"), 1, 0.90),
            (path("walk", "double_tap_speed_mps"), -1, 0.90),
            # The base speed cannot step onto the current boost preset.
            (path("run", "speed_mps"), 1, 2.50),
            (path("run", "double_tap_speed_mps"), 1, 3.00),
            (MODULE.MAX_TURN_RATE_PATH, -1, 2.25),
            (MODULE.KEYBOARD_LOOK_RATE_PATH, 1, 150.0),
        )
        for setting_path, direction, expected in cases:
            with self.subTest(path=setting_path, direction=direction):
                self.assertEqual(
                    MODULE.step_motion_speed(settings, setting_path, direction),
                    expected,
                )

    def test_step_helper_clamps_native_and_pair_boundaries(self) -> None:
        maximum = MODULE.MotionSettings(
            slow_speed_mps=0.75,
            slow_double_tap_speed_mps=0.80,
            walk_speed_mps=2.40,
            walk_double_tap_speed_mps=2.50,
            run_speed_mps=7.25,
            run_double_tap_speed_mps=7.50,
        )
        minimum = MODULE.MotionSettings(
            slow_speed_mps=0.10,
            slow_double_tap_speed_mps=0.15,
            walk_speed_mps=0.80,
            walk_double_tap_speed_mps=0.90,
            run_speed_mps=2.50,
            run_double_tap_speed_mps=2.75,
        )
        for gear in MODULE.GEARS:
            self.assertEqual(
                MODULE.step_motion_speed(
                    maximum, path(gear, "double_tap_speed_mps"), 1
                ),
                maximum.value_for_path(path(gear, "double_tap_speed_mps")),
            )
            self.assertEqual(
                MODULE.step_motion_speed(minimum, path(gear, "speed_mps"), -1),
                minimum.value_for_path(path(gear, "speed_mps")),
            )

        pair_limited = MODULE.MotionSettings(
            slow_speed_mps=0.15,
            slow_double_tap_speed_mps=0.20,
            walk_speed_mps=1.00,
            walk_double_tap_speed_mps=1.10,
            run_speed_mps=3.00,
            run_double_tap_speed_mps=3.25,
        )
        self.assertEqual(
            MODULE.step_motion_speed(
                pair_limited, path("slow", "speed_mps"), 1
            ),
            0.15,
        )
        self.assertEqual(
            MODULE.step_motion_speed(
                pair_limited, path("slow", "speed_mps"), 1
            ),
            pair_limited.slow_speed_mps,
        )
        self.assertEqual(
            MODULE.step_motion_speed(
                pair_limited, path("run", "double_tap_speed_mps"), -1
            ),
            3.25,
        )
        self.assertEqual(
            MODULE.step_motion_speed(pair_limited, MODULE.MAX_TURN_RATE_PATH, 1),
            2.50,
        )
        self.assertEqual(
            MODULE.step_motion_speed(
                MODULE.MotionSettings(keyboard_look_rate_deg_s=360.0),
                MODULE.KEYBOARD_LOOK_RATE_PATH,
                1,
            ),
            360.0,
        )

        sub_step_gap = MODULE.MotionSettings(
            slow_speed_mps=0.10,
            slow_double_tap_speed_mps=0.1000000000001,
            walk_speed_mps=0.80,
            walk_double_tap_speed_mps=0.8000000000001,
            run_speed_mps=7.4999999999999,
            run_double_tap_speed_mps=7.50,
        )
        self.assertEqual(
            MODULE.step_motion_speed(
                sub_step_gap, path("slow", "speed_mps"), 1
            ),
            sub_step_gap.slow_speed_mps,
        )
        self.assertEqual(
            MODULE.step_motion_speed(
                sub_step_gap, path("run", "double_tap_speed_mps"), -1
            ),
            sub_step_gap.run_double_tap_speed_mps,
        )

    def test_step_direction_is_strict(self) -> None:
        for direction in (0, 2, -2, True, 1.0):
            with self.subTest(direction=direction), self.assertRaises(
                MODULE.MotionSettingsError
            ):
                MODULE.step_motion_speed(
                    MODULE.MotionSettings(),
                    path("slow", "speed_mps"),
                    direction,
                )


class MotionSettingsStoreTest(unittest.TestCase):
    def test_modify_is_persisted_revisioned_and_cas_guarded(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            file_path = Path(temporary) / "motion.json"
            store = MODULE.MotionSettingsStore(
                file_path, initial=MODULE.MotionSettings()
            )
            modified = store.modify(
                path("walk", "speed_mps"),
                0.90,
                expected_revision=0,
            )
            self.assertTrue(modified.changed)
            self.assertEqual(modified.previous_value, 0.80)
            self.assertEqual(modified.value, 0.90)
            self.assertEqual(store.settings.revision, 1)
            self.assertEqual(store.settings.walk_speed_mps, 0.90)
            self.assertEqual(MODULE.load_settings(file_path).settings, store.settings)
            with self.assertRaisesRegex(
                MODULE.MotionSettingsError, "expected revision"
            ) as conflict:
                store.modify(
                    path("walk", "speed_mps"),
                    1.00,
                    expected_revision=0,
                )
            self.assertEqual(conflict.exception.code, "E_DATA_REVISION_CONFLICT")
            self.assertEqual(store.settings.revision, 1)
            self.assertEqual(store.settings.walk_speed_mps, 0.90)
            self.assertEqual(MODULE.load_settings(file_path).settings, store.settings)

    def test_same_value_is_idempotent_without_save_or_revision_increment(self) -> None:
        saves = []

        def saver(file_path, settings):
            saves.append((file_path, settings))

        with tempfile.TemporaryDirectory() as temporary:
            store = MODULE.MotionSettingsStore(
                Path(temporary) / "motion.json",
                initial=MODULE.MotionSettings(revision=4),
                saver=saver,
            )
            result = store.modify(
                path("run", "speed_mps"),
                2.50,
                expected_revision=4,
            )
            self.assertFalse(result.changed)
            self.assertEqual(result.settings.revision, 4)
            self.assertEqual(saves, [])

    def test_range_constraint_path_and_persistence_errors_preserve_old_value(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            file_path = Path(temporary) / "motion.json"
            original = MODULE.MotionSettings(revision=2)
            MODULE.atomic_save_settings(file_path, original)

            def failing_saver(_file_path, _settings):
                raise OSError("injected disk failure")

            store = MODULE.MotionSettingsStore(
                file_path,
                initial=original,
                saver=failing_saver,
            )
            with self.assertRaises(MODULE.MotionSettingsError) as out_of_range:
                store.modify(path("slow", "speed_mps"), 0.81, expected_revision=2)
            self.assertEqual(out_of_range.exception.code, "E_DATA_RANGE")
            with self.assertRaises(MODULE.MotionSettingsError) as constraint:
                store.modify(path("slow", "speed_mps"), 0.30, expected_revision=2)
            self.assertEqual(constraint.exception.code, "E_DATA_CONSTRAINT")
            with self.assertRaises(MODULE.MotionSettingsError) as unknown:
                store.modify(
                    "control.motion.gears.crawl.speed_mps",
                    0.20,
                    expected_revision=2,
                )
            self.assertEqual(unknown.exception.code, "E_DATA_PATH_UNKNOWN")
            with self.assertRaises(MODULE.MotionSettingsPersistenceError) as persisted:
                store.modify(path("walk", "speed_mps"), 0.90, expected_revision=2)
            self.assertEqual(persisted.exception.code, "E_DATA_PERSIST")
            self.assertEqual(store.settings, original)
            self.assertEqual(MODULE.load_settings(file_path).settings, original)

    def test_step_uses_the_same_cas_and_persistence_path(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            store = MODULE.MotionSettingsStore(
                Path(temporary) / "motion.json",
                initial=MODULE.MotionSettings(),
            )
            result = store.step(
                path("slow", "speed_mps"),
                1,
                expected_revision=0,
            )
            self.assertTrue(result.changed)
            self.assertEqual(result.value, 0.15)
            self.assertEqual(result.settings.revision, 1)
            self.assertEqual(store.mapping()["settings"]["revision"], 1)

    def test_turn_rate_modify_is_persisted_revisioned_and_capped(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            file_path = Path(temporary) / "motion.json"
            store = MODULE.MotionSettingsStore(
                file_path,
                initial=MODULE.MotionSettings(max_turn_rate_rad_s=2.25),
            )
            modified = store.step(
                MODULE.MAX_TURN_RATE_PATH,
                1,
                expected_revision=0,
            )
            self.assertTrue(modified.changed)
            self.assertEqual(modified.value, 2.50)
            self.assertEqual(store.settings.revision, 1)
            self.assertEqual(
                MODULE.load_settings(file_path).settings.max_turn_rate_rad_s,
                2.50,
            )
            self.assertFalse(
                store.step(
                    MODULE.MAX_TURN_RATE_PATH,
                    1,
                    expected_revision=1,
                ).changed
            )
            with self.assertRaises(MODULE.MotionSettingsError):
                store.modify(
                    MODULE.MAX_TURN_RATE_PATH,
                    2.75,
                    expected_revision=1,
                )

    def test_store_loads_missing_or_invalid_state_safely(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            file_path = Path(temporary) / "motion.json"
            missing = MODULE.MotionSettingsStore(file_path)
            self.assertEqual(missing.settings, MODULE.MotionSettings())
            self.assertEqual(missing.load_status, "missing")
            file_path.write_text("not-json", encoding="utf-8")
            invalid = MODULE.MotionSettingsStore(file_path)
            self.assertEqual(invalid.settings, MODULE.MotionSettings())
            self.assertEqual(invalid.load_status, "invalid")
            self.assertIsNotNone(invalid.load_error)

    def test_store_uses_profile_fallback_only_without_a_valid_host_file(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            file_path = Path(temporary) / "motion.json"
            profile = MODULE.MotionSettings(
                walk_speed_mps=0.9,
                walk_double_tap_speed_mps=1.1,
            )
            missing = MODULE.MotionSettingsStore(file_path, fallback=profile)
            self.assertEqual(missing.settings, profile)
            self.assertEqual(missing.load_status, "missing")

            file_path.write_text("invalid", encoding="utf-8")
            invalid = MODULE.MotionSettingsStore(file_path, fallback=profile)
            self.assertEqual(invalid.settings, profile)
            self.assertEqual(invalid.load_status, "invalid")
            self.assertIsNotNone(invalid.load_error)

            persisted = MODULE.MotionSettings(
                revision=4,
                walk_speed_mps=1.2,
                walk_double_tap_speed_mps=1.4,
            )
            MODULE.atomic_save_settings(file_path, persisted)
            loaded = MODULE.MotionSettingsStore(file_path, fallback=profile)
            self.assertEqual(loaded.settings, persisted)
            self.assertEqual(loaded.load_status, "loaded")

    def test_store_rejects_ambiguous_or_invalid_initialization(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            file_path = Path(temporary) / "motion.json"
            with self.assertRaisesRegex(ValueError, "mutually exclusive"):
                MODULE.MotionSettingsStore(
                    file_path,
                    initial=MODULE.MotionSettings(),
                    fallback=MODULE.MotionSettings(),
                )
            for keyword in ("initial", "fallback"):
                with self.subTest(keyword=keyword), self.assertRaises(TypeError):
                    MODULE.MotionSettingsStore(file_path, **{keyword: object()})


if __name__ == "__main__":
    unittest.main()
