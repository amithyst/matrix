from __future__ import annotations

from contextlib import redirect_stderr
import importlib.util
import io
import math
from pathlib import Path
import sys
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "matrix_bfm_isaac_keyboard.py"
SPEC = importlib.util.spec_from_file_location(
    "matrix_bfm_isaac_keyboard", SCRIPT_PATH
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class MatrixBfmIsaacKeyboardTest(unittest.TestCase):
    def test_parser_exposes_frozen_xtest_camera_contract(self) -> None:
        defaults = MODULE._parser().parse_args(
            [
                "--socket",
                "/tmp/keyboard.sock",
                "--allowed-process-root",
                "/opt/matrix",
            ]
        )
        self.assertEqual(defaults.camera_look_backend, "xtest")
        self.assertEqual(defaults.camera_look_pixels_per_second, 600.0)
        self.assertFalse(defaults.ignore_escape)

        disabled = MODULE._parser().parse_args(
            [
                "--socket",
                "/tmp/keyboard.sock",
                "--allowed-process-root",
                "/opt/matrix",
                "--camera-look-backend",
                "off",
                "--camera-look-pixels-per-second",
                "720.5",
            ]
        )
        self.assertEqual(disabled.camera_look_backend, "off")
        self.assertEqual(disabled.camera_look_pixels_per_second, 720.5)

        no_escape = MODULE._parser().parse_args(
            [
                "--socket",
                "/tmp/keyboard.sock",
                "--allowed-process-root",
                "/opt/matrix",
                "--ignore-escape",
            ]
        )
        self.assertTrue(no_escape.ignore_escape)

        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            MODULE._parser().parse_args(
                [
                    "--socket",
                    "/tmp/keyboard.sock",
                    "--allowed-process-root",
                    "/opt/matrix",
                    "--camera-look-backend",
                    "xdotool",
                ]
            )

    def test_xmodmap_includes_arrows_without_losing_policy_controls(self) -> None:
        mapping = MODULE.parse_xmodmap(
            (
                "keycode  25 = w W w W",
                "keycode  50 = Shift_L NoSymbol Shift_L",
                "keycode   9 = Escape NoSymbol Escape",
                "keycode 113 = Left NoSymbol Left",
                "keycode 114 = Right NoSymbol Right",
                "keycode 111 = Up NoSymbol Up",
                "keycode 116 = Down NoSymbol Down",
                "keycode  67 = F1 F1 F1 F1",
            )
        )

        self.assertEqual(
            mapping,
            {
                25: "W",
                50: "LEFT_SHIFT",
                9: "ESCAPE",
                113: "LEFT",
                114: "RIGHT",
                111: "UP",
                116: "DOWN",
            },
        )
        self.assertEqual(
            MODULE.ARROW_KEYS,
            frozenset(("LEFT", "RIGHT", "UP", "DOWN")),
        )

    def test_xmodmap_can_filter_desktop_escape_exit(self) -> None:
        mapping = MODULE.parse_xmodmap(
            (
                "keycode   9 = Escape NoSymbol Escape",
                "keycode  25 = w W w W",
                "keycode  65 = space NoSymbol space",
            ),
            include_escape=False,
        )

        self.assertEqual(mapping, {25: "W", 65: "SPACE"})

    def test_raw_event_parser_pairs_press_and_release_with_detail(self) -> None:
        events = tuple(
            MODULE.parse_xinput_events(
                (
                    "EVENT type 13 (RawKeyPress)",
                    "    device: 3 (10)",
                    "    detail: 113",
                    "EVENT type 14 (RawKeyRelease)",
                    "    detail: 113",
                )
            )
        )

        self.assertEqual(events, ((113, True), (113, False)))

    def test_arrow_integrator_maps_yaw_pitch_and_resets(self) -> None:
        integrator = MODULE.ArrowLookIntegrator()

        self.assertEqual(
            integrator.update(
                {"RIGHT", "UP"}, dt_s=0.02, pixels_per_second=100.0
            ),
            (2, -2),
        )
        self.assertEqual(
            integrator.update(
                {"LEFT", "DOWN"}, dt_s=0.02, pixels_per_second=100.0
            ),
            (-2, 2),
        )
        self.assertEqual(
            integrator.update(set(), dt_s=0.02, pixels_per_second=100.0),
            (0, 0),
        )

    def test_arrow_integrator_accumulates_fractional_pixels_and_caps_stalls(
        self,
    ) -> None:
        integrator = MODULE.ArrowLookIntegrator()
        self.assertEqual(
            integrator.update(
                {"RIGHT"}, dt_s=0.005, pixels_per_second=100.0
            ),
            (0, 0),
        )
        self.assertEqual(
            integrator.update(
                {"RIGHT"}, dt_s=0.005, pixels_per_second=100.0
            ),
            (1, 0),
        )

        integrator.reset()
        self.assertEqual(
            integrator.update(
                {"RIGHT", "DOWN"}, dt_s=10.0, pixels_per_second=100.0
            ),
            (5, 5),
        )

    def test_arrow_integrator_rejects_non_physical_timing_and_speed(self) -> None:
        integrator = MODULE.ArrowLookIntegrator()

        for dt_s in (-0.01, math.inf, math.nan):
            with self.subTest(dt_s=dt_s), self.assertRaisesRegex(
                ValueError, "dt must be finite and non-negative"
            ):
                integrator.update(
                    {"RIGHT"}, dt_s=dt_s, pixels_per_second=100.0
                )
        for speed in (0.0, -1.0, math.inf, math.nan):
            with self.subTest(speed=speed), self.assertRaisesRegex(
                ValueError, "speed must be positive and finite"
            ):
                integrator.update(
                    {"RIGHT"}, dt_s=0.02, pixels_per_second=speed
                )


if __name__ == "__main__":
    unittest.main()
