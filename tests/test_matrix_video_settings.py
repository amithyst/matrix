#!/usr/bin/env python3

from __future__ import annotations

from contextlib import redirect_stderr, redirect_stdout
import importlib.util
import io
import json
import os
from pathlib import Path
import stat
import sys
import tempfile
import unittest
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts/matrix_video_settings.py"
SPEC = importlib.util.spec_from_file_location("matrix_video_settings_tested", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class VideoSettingsValueTest(unittest.TestCase):
    def test_defaults_have_exact_versioned_schema_and_runtime_values(self) -> None:
        settings = MODULE.VideoSettings()
        self.assertEqual(
            settings.to_mapping(),
            {
                "version": 1,
                "revision": 0,
                "resolution": "1920x1080",
                "window_mode": "borderless",
                "fps_limit": 60,
                "quality": "high",
                "camera_smoothing": "medium",
            },
        )
        self.assertEqual(settings.runtime_mapping()["resolution_width"], 1920)
        self.assertEqual(settings.runtime_mapping()["resolution_height"], 1080)
        explicit = MODULE.VideoSettings(resolution="2560x1440")
        self.assertEqual(explicit.runtime_mapping()["resolution_width"], 2560)
        self.assertEqual(explicit.runtime_mapping()["resolution_height"], 1440)

    def test_mapping_requires_exact_schema_version_and_strict_revision(self) -> None:
        valid = MODULE.VideoSettings(revision=3).to_mapping()
        self.assertEqual(MODULE.VideoSettings.from_mapping(valid).revision, 3)
        mutations = []
        for key in valid:
            changed = dict(valid)
            del changed[key]
            mutations.append(changed)
        mutations.extend(
            (
                {**valid, "extra": "value"},
                {**valid, "version": 2},
                {**valid, "version": True},
                {**valid, "revision": -1},
                {**valid, "revision": True},
                {**valid, "revision": 1.0},
            )
        )
        for value in mutations:
            with self.subTest(value=value), self.assertRaises(
                MODULE.VideoSettingsError
            ):
                MODULE.VideoSettings.from_mapping(value)

    def test_every_runtime_value_is_a_fixed_preset(self) -> None:
        cases = (
            {"resolution": "1920x1080;quit"},
            {"resolution": "1x1"},
            {"window_mode": "fullscreen\nset foo"},
            {"fps_limit": 59},
            {"fps_limit": True},
            {"fps_limit": "60"},
            {"quality": "ultra;open /tmp"},
            {"camera_smoothing": "0;quit"},
        )
        for values in cases:
            with self.subTest(values=values), self.assertRaises(
                MODULE.VideoSettingsError
            ) as caught:
                MODULE.VideoSettings(**values)
            self.assertEqual(caught.exception.code, "E_VIDEO_PRESET")

    def test_patch_is_strict_and_does_not_mutate_source(self) -> None:
        settings = MODULE.VideoSettings()
        replacement = settings.with_patch(
            {
                "resolution": "1920x1080",
                "window_mode": "fullscreen",
                "fps_limit": 120,
                "quality": "epic",
                "camera_smoothing": "high",
            },
            revision=7,
        )
        self.assertEqual(replacement.revision, 7)
        self.assertEqual(replacement.resolution, "1920x1080")
        self.assertEqual(settings, MODULE.VideoSettings())
        for patch in (
            {"unknown": "value"},
            {"camera_smoothing": "on;quit"},
        ):
            with self.subTest(patch=patch), self.assertRaises(
                MODULE.VideoSettingsError
            ):
                settings.with_patch(patch)
        with self.assertRaises(MODULE.VideoSettingsError):
            settings.with_patch([("quality", "low")])


class VideoSettingsPathTest(unittest.TestCase):
    def test_default_path_is_host_scoped_and_honors_xdg(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self.assertEqual(
                MODULE.default_settings_file("trna", config_home=root),
                root / "matrix/hosts/trna/video-settings.json",
            )
            with mock.patch.dict(
                os.environ,
                {"MATRIX_HOST_PROFILE": "heyuan", "XDG_CONFIG_HOME": str(root)},
                clear=False,
            ):
                self.assertEqual(
                    MODULE.default_settings_file(),
                    root / "matrix/hosts/heyuan/video-settings.json",
                )

    def test_profile_and_absolute_path_validation_reject_escapes(self) -> None:
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
                MODULE.VideoSettingsError
            ):
                MODULE.canonical_host_profile(profile)
        for profile in ("trna", "TRNA-01", "lab_a", "lab.a"):
            self.assertEqual(MODULE.canonical_host_profile(profile), profile)
        with self.assertRaisesRegex(ValueError, "absolute"):
            MODULE.load_settings(Path("video-settings.json"))

    def test_omitted_profile_requires_environment_value(self) -> None:
        with mock.patch.dict(os.environ, {}, clear=True), self.assertRaisesRegex(
            MODULE.VideoSettingsError, "required"
        ):
            MODULE.default_settings_file()


class VideoSettingsPersistenceTest(unittest.TestCase):
    def test_missing_invalid_duplicate_and_oversized_files_fail_safe(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "video.json"
            missing = MODULE.load_settings(path)
            self.assertEqual(missing.status, "missing")
            self.assertEqual(missing.settings, MODULE.VideoSettings())

            invalid_payloads = (
                b"{",
                (
                    b'{"version":1,"version":1,"revision":0,'
                    b'"resolution":"native","window_mode":"borderless",'
                    b'"fps_limit":60,"quality":"high","camera_smoothing":"medium"}'
                ),
                json.dumps(
                    {
                        **MODULE.VideoSettings().to_mapping(),
                        "fps_limit": float("nan"),
                    }
                ).encode(),
                b"x" * (MODULE.MAX_SETTINGS_BYTES + 1),
                b"\xff",
            )
            for payload in invalid_payloads:
                with self.subTest(size=len(payload)):
                    path.write_bytes(payload)
                    loaded = MODULE.load_settings(path)
                    self.assertEqual(loaded.status, "invalid")
                    self.assertEqual(loaded.settings, MODULE.VideoSettings())
                    self.assertIsNotNone(loaded.error)

    def test_atomic_save_round_trips_is_private_and_fsyncs(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "nested/video.json"
            settings = MODULE.VideoSettings(
                revision=9,
                resolution="2560x1440",
                window_mode="fullscreen",
                fps_limit=120,
                quality="epic",
                camera_smoothing="high",
            )
            with mock.patch.object(
                MODULE.os, "fsync", wraps=MODULE.os.fsync
            ) as fsync:
                MODULE.atomic_save_settings(path, settings)
            self.assertGreaterEqual(fsync.call_count, 2)
            self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o600)
            self.assertEqual(MODULE.load_settings(path).settings, settings)
            self.assertEqual(
                json.loads(path.read_text(encoding="utf-8")),
                settings.to_mapping(),
            )
            self.assertFalse(tuple(path.parent.glob(f".{path.name}.*")))

    def test_replacing_a_broad_regular_file_still_produces_mode_0600(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "video.json"
            path.write_text("old", encoding="utf-8")
            path.chmod(0o644)
            MODULE.atomic_save_settings(path, MODULE.VideoSettings())
            self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o600)

    def test_atomic_save_expected_revision_is_a_file_level_cas(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "video.json"
            first = MODULE.VideoSettings(revision=1, quality="medium")
            MODULE.atomic_save_settings(path, first, expected_revision=0)
            conflicting = MODULE.VideoSettings(revision=2, quality="low")
            with self.assertRaises(MODULE.VideoSettingsError) as caught:
                MODULE.atomic_save_settings(
                    path, conflicting, expected_revision=0
                )
            self.assertEqual(caught.exception.code, "E_VIDEO_REVISION_CONFLICT")
            self.assertEqual(MODULE.load_settings(path).settings, first)
            MODULE.atomic_save_settings(
                path, conflicting, expected_revision=1
            )
            self.assertEqual(MODULE.load_settings(path).settings, conflicting)

    def test_leaf_symlink_is_never_read_or_followed_for_write(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            target = root / "outside.json"
            original = b'{"outside":"unchanged"}\n'
            target.write_bytes(original)
            link = root / "video.json"
            link.symlink_to(target)

            loaded = MODULE.load_settings(link)
            self.assertEqual(loaded.status, "invalid")
            self.assertIn("cannot read", loaded.error)
            with self.assertRaises(MODULE.VideoSettingsPersistenceError):
                MODULE.atomic_save_settings(link, MODULE.VideoSettings())
            self.assertTrue(link.is_symlink())
            self.assertEqual(target.read_bytes(), original)

    def test_parent_symlink_is_never_followed_for_read_or_write(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            outside = root / "outside"
            outside.mkdir()
            linked = root / "linked"
            linked.symlink_to(outside, target_is_directory=True)
            path = linked / "video.json"

            loaded = MODULE.load_settings(path)
            self.assertEqual(loaded.status, "invalid")
            with self.assertRaises(MODULE.VideoSettingsPersistenceError):
                MODULE.atomic_save_settings(path, MODULE.VideoSettings())
            self.assertFalse((outside / "video.json").exists())


class VideoSettingsStoreTest(unittest.TestCase):
    def test_multi_field_patch_is_one_persisted_cas_revision(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "video.json"
            store = MODULE.VideoSettingsStore(
                path, initial=MODULE.VideoSettings()
            )
            result = store.patch(
                {
                    "resolution": "2560x1440",
                    "fps_limit": 120,
                    "camera_smoothing": "high",
                },
                expected_revision=0,
            )
            self.assertTrue(result.changed)
            self.assertEqual(
                result.changed_fields,
                ("camera_smoothing", "fps_limit", "resolution"),
            )
            self.assertEqual(result.previous_settings.revision, 0)
            self.assertEqual(result.settings.revision, 1)
            self.assertEqual(MODULE.load_settings(path).settings, store.settings)
            with self.assertRaises(MODULE.VideoSettingsError) as caught:
                store.modify("quality", "epic", expected_revision=0)
            self.assertEqual(caught.exception.code, "E_VIDEO_REVISION_CONFLICT")
            self.assertEqual(store.settings.revision, 1)

    def test_same_or_empty_patch_is_idempotent_without_save(self) -> None:
        saves = []

        def saver(path, settings):
            saves.append((path, settings))

        with tempfile.TemporaryDirectory() as temporary:
            store = MODULE.VideoSettingsStore(
                Path(temporary) / "video.json",
                initial=MODULE.VideoSettings(revision=4),
                saver=saver,
            )
            for patch in ({"quality": "high"}, {}):
                result = store.patch(patch, expected_revision=4)
                self.assertFalse(result.changed)
                self.assertEqual(result.settings.revision, 4)
            self.assertEqual(saves, [])

    def test_step_setting_and_store_step_use_adjacent_fixed_presets(self) -> None:
        settings = MODULE.VideoSettings()
        self.assertEqual(MODULE.step_setting(settings, "fps_limit", 1), 90)
        self.assertEqual(MODULE.step_setting(settings, "quality", -1), "medium")
        self.assertEqual(
            MODULE.step_setting(
                MODULE.VideoSettings(camera_smoothing="off"),
                "camera_smoothing",
                -1,
            ),
            "off",
        )
        for direction in (0, 2, True, 1.0):
            with self.subTest(direction=direction), self.assertRaises(
                MODULE.VideoSettingsError
            ):
                MODULE.step_setting(settings, "quality", direction)
        with tempfile.TemporaryDirectory() as temporary:
            store = MODULE.VideoSettingsStore(
                Path(temporary) / "video.json",
                initial=settings,
            )
            changed = store.step("quality", 1, expected_revision=0)
            self.assertTrue(changed.changed)
            self.assertEqual(changed.settings.quality, "epic")
            self.assertEqual(changed.settings.revision, 1)

    def test_validation_and_persistence_failures_preserve_old_snapshot(self) -> None:
        def failing_saver(_path, _settings):
            raise OSError("injected disk failure")

        with tempfile.TemporaryDirectory() as temporary:
            original = MODULE.VideoSettings(revision=2)
            store = MODULE.VideoSettingsStore(
                Path(temporary) / "video.json",
                initial=original,
                saver=failing_saver,
            )
            with self.assertRaises(MODULE.VideoSettingsError):
                store.modify("fps_limit", 59, expected_revision=2)
            with self.assertRaises(MODULE.VideoSettingsError):
                store.modify("console", "quit", expected_revision=2)
            with self.assertRaises(MODULE.VideoSettingsPersistenceError):
                store.modify("quality", "epic", expected_revision=2)
            self.assertEqual(store.settings, original)

    def test_two_default_stores_cannot_overwrite_a_stale_revision(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "video.json"
            first = MODULE.VideoSettingsStore(path)
            stale = MODULE.VideoSettingsStore(path)
            first.modify("quality", "epic", expected_revision=0)
            with self.assertRaises(MODULE.VideoSettingsError) as caught:
                stale.modify("fps_limit", 90, expected_revision=0)
            self.assertEqual(caught.exception.code, "E_VIDEO_REVISION_CONFLICT")
            self.assertEqual(MODULE.load_settings(path).settings, first.settings)
            self.assertEqual(stale.settings.revision, 0)

    def test_missing_and_invalid_files_load_defaults_or_explicit_fallback(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "video.json"
            missing = MODULE.VideoSettingsStore(path)
            self.assertEqual(missing.settings, MODULE.VideoSettings())
            self.assertEqual(missing.load_status, "missing")
            fallback = MODULE.VideoSettings(fps_limit=90, quality="medium")
            fallback_store = MODULE.VideoSettingsStore(path, fallback=fallback)
            self.assertEqual(fallback_store.settings, fallback)
            path.write_text("invalid", encoding="utf-8")
            invalid = MODULE.VideoSettingsStore(path, fallback=fallback)
            self.assertEqual(invalid.settings, fallback)
            self.assertEqual(invalid.load_status, "invalid")


class VideoSettingsCliTest(unittest.TestCase):
    def test_show_and_patch_offer_machine_readable_provider_api(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "video.json"
            stdout = io.StringIO()
            with redirect_stdout(stdout):
                self.assertEqual(
                    MODULE.main(["--settings-file", str(path), "show"]), 0
                )
            shown = json.loads(stdout.getvalue())
            self.assertEqual(shown["load_status"], "missing")
            self.assertEqual(shown["runtime"]["fps_limit"], 60)

            stdout = io.StringIO()
            with redirect_stdout(stdout):
                self.assertEqual(
                    MODULE.main(
                        [
                            "--settings-file",
                            str(path),
                            "patch",
                            "--expected-revision",
                            "0",
                            "--resolution",
                            "1920x1080",
                            "--fps-limit",
                            "120",
                            "--camera-smoothing",
                            "high",
                        ]
                    ),
                    0,
                )
            patched = json.loads(stdout.getvalue())
            self.assertTrue(patched["changed"])
            self.assertEqual(patched["settings"]["revision"], 1)
            self.assertEqual(patched["runtime"]["resolution_width"], 1920)

            stderr = io.StringIO()
            with redirect_stderr(stderr):
                self.assertEqual(
                    MODULE.main(
                        [
                            "--settings-file",
                            str(path),
                            "patch",
                            "--expected-revision",
                            "0",
                            "--quality",
                            "epic",
                        ]
                    ),
                    2,
                )
            error = json.loads(stderr.getvalue())
            self.assertEqual(error["error"], "E_VIDEO_REVISION_CONFLICT")

    def test_empty_patch_is_rejected_by_cli(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "video.json"
            stderr = io.StringIO()
            with redirect_stderr(stderr):
                self.assertEqual(
                    MODULE.main(
                        [
                            "--settings-file",
                            str(path),
                            "patch",
                            "--expected-revision",
                            "0",
                        ]
                    ),
                    2,
                )
            self.assertEqual(
                json.loads(stderr.getvalue())["error"], "E_VIDEO_PATCH"
            )

    def test_launch_json_file_form_is_compact_and_launcher_safe(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "video.json"
            MODULE.atomic_save_settings(
                path,
                MODULE.VideoSettings(
                    revision=2,
                    resolution="2560x1440",
                    fps_limit=90,
                    camera_smoothing="high",
                ),
            )
            stdout = io.StringIO()
            with redirect_stdout(stdout):
                self.assertEqual(
                    MODULE.main(["launch-json", "--file", str(path)]), 0
                )
            encoded = stdout.getvalue()
            self.assertNotIn(": ", encoded)
            self.assertNotIn("\n ", encoded)
            launch = json.loads(encoded)
            self.assertEqual(launch["revision"], 2)
            self.assertEqual(launch["resolution_width"], 2560)
            self.assertEqual(launch["fps_limit"], 90)


if __name__ == "__main__":
    unittest.main()
