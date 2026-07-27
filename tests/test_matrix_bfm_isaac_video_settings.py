from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts" / "matrix_bfm_isaac_video_settings.py"
SETTINGS = REPO_ROOT / "config/runtime/matrix-bfm-isaac-video-settings.json"
SPEC = importlib.util.spec_from_file_location(
    "matrix_bfm_isaac_video_settings", SCRIPT
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class MatrixBfmIsaacVideoSettingsTest(unittest.TestCase):
    def test_tracked_baseline_is_the_full_resolution_visual_default(self) -> None:
        self.assertEqual(
            MODULE.runtime_mapping(MODULE.resolve_settings(SETTINGS, {})),
            {
                "schema": MODULE.SCHEMA,
                "resolution": "1280x720",
                "resolution_width": 1280,
                "resolution_height": 720,
                "window_mode": "borderless",
                "fps_limit": 30,
                "quality": "low",
                "camera_smoothing": "medium",
                "screen_percentage": 100,
            },
        )

    def test_explicit_bfm_ab_overrides_are_bounded_and_complete(self) -> None:
        value = MODULE.runtime_mapping(
            MODULE.resolve_settings(
                SETTINGS,
                {
                    "MATRIX_BFM_ISAAC_VIDEO_RESOLUTION": "1920x1080",
                    "MATRIX_BFM_ISAAC_VIDEO_WINDOW_MODE": "windowed",
                    "MATRIX_BFM_ISAAC_UE_MAX_FPS": "60",
                    "MATRIX_BFM_ISAAC_VIDEO_QUALITY": "high",
                    "MATRIX_BFM_ISAAC_VIDEO_CAMERA_SMOOTHING": "off",
                    "MATRIX_BFM_ISAAC_SCREEN_PERCENTAGE": "75",
                },
            )
        )

        self.assertEqual(value["resolution_width"], 1920)
        self.assertEqual(value["resolution_height"], 1080)
        self.assertEqual(value["window_mode"], "windowed")
        self.assertEqual(value["fps_limit"], 60)
        self.assertEqual(value["quality"], "high")
        self.assertEqual(value["camera_smoothing"], "off")
        self.assertEqual(value["screen_percentage"], 75)

    def test_invalid_override_and_duplicate_or_symlink_config_fail_closed(self) -> None:
        for environment in (
            {"MATRIX_BFM_ISAAC_VIDEO_RESOLUTION": "800x600"},
            {"MATRIX_BFM_ISAAC_UE_MAX_FPS": "50"},
            {"MATRIX_BFM_ISAAC_SCREEN_PERCENTAGE": "24"},
            {"MATRIX_BFM_ISAAC_SCREEN_PERCENTAGE": "75;r.Exit"},
        ):
            with self.subTest(environment=environment):
                with self.assertRaises(MODULE.VideoSettingsError):
                    MODULE.resolve_settings(SETTINGS, environment)

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            duplicate = root / "duplicate.json"
            duplicate.write_text(
                SETTINGS.read_text(encoding="utf-8").replace(
                    '"screen_percentage": 100',
                    '"screen_percentage": 100, "screen_percentage": 75',
                ),
                encoding="utf-8",
            )
            with self.assertRaises(MODULE.VideoSettingsError):
                MODULE.load_settings(duplicate)

            alias = root / "alias.json"
            alias.symlink_to(SETTINGS)
            with self.assertRaises(MODULE.VideoSettingsError):
                MODULE.load_settings(alias)

    def test_cli_lines_are_safe_for_the_shell_launcher(self) -> None:
        result = subprocess.run(
            [
                sys.executable,
                os.fspath(SCRIPT),
                "--file",
                os.fspath(SETTINGS),
                "--format",
                "lines",
            ],
            env={
                **os.environ,
                "MATRIX_BFM_ISAAC_SCREEN_PERCENTAGE": "125",
            },
            text=True,
            capture_output=True,
            check=False,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(
            result.stdout.splitlines(),
            ["1280", "720", "borderless", "30", "low", "medium", "125"],
        )


if __name__ == "__main__":
    unittest.main()
