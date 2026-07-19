from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import signal
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = REPO_ROOT / "scripts"
if os.fspath(SCRIPTS) not in sys.path:
    sys.path.insert(0, os.fspath(SCRIPTS))
SCRIPT = SCRIPTS / "matrix_calibration_overlay.py"
SPEC = importlib.util.spec_from_file_location("matrix_calibration_overlay", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class OverlayLayoutTest(unittest.TestCase):
    @staticmethod
    def intersects(left, right) -> bool:
        return bool(
            left[0] < right[0] + right[2]
            and right[0] < left[0] + left[2]
            and left[1] < right[1] + right[3]
            and right[1] < left[1] + left[3]
        )

    def test_crosshair_intersects_exact_client_centre(self) -> None:
        geometry = MODULE.WindowGeometry(
            window=41,
            x=100,
            y=80,
            width=801,
            height=601,
        )
        layout = MODULE.overlay_layout(geometry)
        centre_x, centre_y = geometry.centre

        horizontal = layout["horizontal"]
        vertical = layout["vertical"]
        self.assertLessEqual(horizontal[0], centre_x)
        self.assertGreater(horizontal[0] + horizontal[2], centre_x)
        self.assertLessEqual(horizontal[1], centre_y)
        self.assertGreater(horizontal[1] + horizontal[3], centre_y)
        self.assertLessEqual(vertical[0], centre_x)
        self.assertGreater(vertical[0] + vertical[2], centre_x)
        self.assertLessEqual(vertical[1], centre_y)
        self.assertGreater(vertical[1] + vertical[3], centre_y)

    def test_large_panel_and_controls_stay_inside_normal_client(self) -> None:
        geometry = MODULE.WindowGeometry(
            window=1,
            x=10,
            y=20,
            width=1280,
            height=800,
        )
        layout = MODULE.overlay_layout(geometry)
        panel = layout["panel"]
        self.assertGreaterEqual(panel[2], 800)
        self.assertGreaterEqual(panel[3], 560)
        self.assertEqual(
            (panel[0] + panel[2] // 2, panel[1] + panel[3] // 2),
            geometry.centre,
        )
        for action in MODULE._PANEL_ACTIONS:
            x, y, width, height = layout[action]
            self.assertGreaterEqual(width, 100)
            self.assertGreaterEqual(height, 60)
            self.assertTrue(MODULE.point_in_rectangle((x, y), panel))
            self.assertTrue(
                MODULE.point_in_rectangle((x + width - 1, y + height - 1), panel)
            )
        for name in (
            "profile_local",
            "profile_remote",
            "speed_down",
            "speed_value",
            "speed_up",
            "apply_return",
        ):
            self.assertFalse(
                self.intersects(layout[name], layout["crosshair_safe"]),
                msg=f"{name} intersects the centre calibration clearance",
            )

    def test_large_desktop_panel_reaches_requested_scale(self) -> None:
        for width, height in ((2560, 1600), (1920, 1200)):
            layout = MODULE.overlay_layout(
                MODULE.WindowGeometry(1, 0, 0, width, height)
            )
            panel = layout["panel"]
            self.assertGreaterEqual(panel[2], 1100)
            self.assertLessEqual(panel[2], 1200)
            self.assertGreaterEqual(panel[3], 760)
            self.assertLessEqual(panel[3], 800)

    def test_compact_layout_is_bounded_and_too_small_client_hides_safely(self) -> None:
        geometry = MODULE.WindowGeometry(1, 20, 30, 640, 420)
        self.assertTrue(MODULE.overlay_supported(geometry))
        layout = MODULE.overlay_layout(geometry)
        panel = layout["panel"]
        for name in MODULE._PANEL_ACTIONS + ("speed_value", "crosshair_safe"):
            rectangle = layout[name]
            self.assertTrue(MODULE.point_in_rectangle(rectangle[:2], panel))
            self.assertTrue(
                MODULE.point_in_rectangle(
                    (
                        rectangle[0] + rectangle[2] - 1,
                        rectangle[1] + rectangle[3] - 1,
                    ),
                    panel,
                )
            )
        for name in (
            "profile_local",
            "profile_remote",
            "speed_down",
            "speed_value",
            "speed_up",
            "apply_return",
        ):
            self.assertFalse(
                self.intersects(layout[name], layout["crosshair_safe"])
            )
        tiny = MODULE.WindowGeometry(1, 0, 0, 479, 359)
        self.assertFalse(MODULE.overlay_supported(tiny))
        with self.assertRaisesRegex(ValueError, "too small"):
            MODULE.overlay_layout(tiny)

    def test_root_coordinate_hit_test_handles_offset_remote_desktop_client(self) -> None:
        geometry = MODULE.WindowGeometry(1, -640, 120, 1600, 900)
        layout = MODULE.overlay_layout(geometry)
        for action in MODULE._PANEL_ACTIONS:
            x, y, width, height = layout[action]
            self.assertEqual(
                MODULE.panel_action_at(layout, x + width // 2, y + height // 2),
                action,
            )
        self.assertIsNone(MODULE.panel_action_at(layout, geometry.x + 3, geometry.y + 3))


class CursorShapeTest(unittest.TestCase):
    @staticmethod
    def pixels(rectangles):
        return {
            (x + dx, y + dy)
            for x, y, width, height in rectangles
            for dx in range(width)
            for dy in range(height)
        }

    def test_xrectangle_uses_the_x11_short_ushort_layout(self) -> None:
        self.assertEqual(MODULE.ctypes.sizeof(MODULE.XRectangle), 8)
        self.assertEqual(MODULE.XRectangle.x.offset, 0)
        self.assertEqual(MODULE.XRectangle.y.offset, 2)
        self.assertEqual(MODULE.XRectangle.width.offset, 4)
        self.assertEqual(MODULE.XRectangle.height.offset, 6)

    def test_arrow_is_shaped_with_hotspot_at_window_origin(self) -> None:
        shadow = self.pixels(MODULE._CURSOR_SHADOW_RECTANGLES)
        foreground = self.pixels(MODULE._CURSOR_FOREGROUND_RECTANGLES)

        self.assertIn((0, 0), shadow)
        self.assertTrue(foreground)
        self.assertLess(foreground, shadow)
        self.assertLess(len(shadow), MODULE._CURSOR_WIDTH * MODULE._CURSOR_HEIGHT / 2)
        self.assertTrue(
            all(
                0 <= x < MODULE._CURSOR_WIDTH and 0 <= y < MODULE._CURSOR_HEIGHT
                for x, y in shadow
            )
        )


class OverlayStateTest(unittest.TestCase):
    def test_state_is_visible_only_for_exact_versioned_true(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "state.json"
            self.assertFalse(MODULE.read_active_state(path))
            path.write_text("{", encoding="utf-8")
            self.assertFalse(MODULE.read_active_state(path))
            path.write_text(json.dumps({"version": 2, "active": True}))
            self.assertFalse(MODULE.read_active_state(path))
            path.write_text(json.dumps({"version": 1, "active": 1}))
            self.assertFalse(MODULE.read_active_state(path))
            path.write_text(json.dumps({"version": 1, "active": True}))
            self.assertTrue(MODULE.read_active_state(path))

    def test_settings_panel_distinguishes_current_next_and_pending(self) -> None:
        lines = MODULE.settings_hint_lines(
            {
                "version": 1,
                "active": True,
                "mouse_settings": {
                    "current": {"profile": "local", "effective_scale": 1.0},
                    "next_launch": {
                        "profile": "remote",
                        "effective_scale": 0.5,
                    },
                    "pending_restart": True,
                    "persistence_error": None,
                },
                "mirror_sensitivity": {
                    "units": "degrees_per_xi2_raw_unit",
                    "base_deg_per_raw_unit": 0.12,
                    "effective_deg_per_raw_unit": 0.12,
                    # Conflicting legacy aliases prove the raw-unit fields win.
                    "base_deg_per_px": 9.0,
                    "effective_deg_per_px": 9.0,
                },
                "restart": {"available": True, "requested": False},
            }
        )
        self.assertIn(b"CURRENT APPLIED (SDL): Local 1.00x", lines[0])
        self.assertIn(b"NEXT LAUNCH: Remote 0.50x", lines[0])
        self.assertIn(b"PENDING RESTART", lines[0])
        self.assertIn(b"base 0.120 -> effective 0.120", lines[1])
        self.assertIn(b"XI2 raw mirror", lines[1])
        self.assertIn(b"deg/raw", lines[1])
        self.assertIn(b"Enter: RETURN TO GAME & APPLY", lines[2])
        self.assertIn(b"F9: fallback", lines[2])
        hint = b" | ".join(lines)
        self.assertIn(b"0.01-0.10", hint)
        self.assertIn(b"0.20-1.00", hint)

    def test_panel_model_surfaces_restart_progress_and_errors(self) -> None:
        restarting = MODULE.settings_panel_model(
            {
                "mouse_settings": {"pending_restart": True},
                "restart": {"requested": True},
            }
        )
        self.assertEqual(restarting.status, "restarting")
        self.assertEqual(restarting.apply_label, "RELOADING MATRIX...")
        failed = MODULE.settings_panel_model(
            {
                "mouse_settings": {
                    "pending_restart": True,
                    "persistence_error": "read-only config",
                },
                "apply_return": {"status": "error"},
            }
        )
        self.assertEqual(failed.status, "error")
        self.assertEqual(failed.error, "read-only config")

        unavailable = MODULE.settings_panel_model(
            {
                "mouse_settings": {"pending_restart": True},
                "restart": {"available": False, "requested": False},
            }
        )
        self.assertEqual(unavailable.apply_label, "APPLY UNAVAILABLE")
        self.assertIn("unavailable", unavailable.error)
        self.assertFalse(unavailable.action_enabled("apply_return"))

    def test_remote_speed_boundary_buttons_are_independently_disabled(self) -> None:
        def model(scale: float):
            return MODULE.settings_panel_model(
                {
                    "mouse_settings": {
                        "next_launch": {
                            "profile": "remote",
                            "effective_scale": scale,
                        }
                    },
                    "restart": {"available": True, "requested": False},
                }
            )

        minimum = model(0.01)
        self.assertFalse(minimum.action_enabled("speed_down"))
        self.assertTrue(minimum.action_enabled("speed_up"))
        self.assertEqual(minimum.next_scale, 0.01)
        maximum = model(1.0)
        self.assertTrue(maximum.action_enabled("speed_down"))
        self.assertFalse(maximum.action_enabled("speed_up"))

    def test_off_table_untrusted_scale_fails_safe_to_local_one_x(self) -> None:
        model = MODULE.settings_panel_model(
            {
                "mouse_settings": {
                    "current": {
                        "profile": "remote",
                        "effective_scale": 0.15,
                    },
                    "next_launch": {
                        "profile": "remote",
                        "effective_scale": 0.11,
                    },
                },
                "restart": {"available": True, "requested": False},
            }
        )
        self.assertEqual(model.current_scale, 1.0)
        self.assertEqual(model.next_scale, 1.0)

    def test_low_remote_scale_is_rendered_with_discrete_step_hint(self) -> None:
        geometry = MODULE.WindowGeometry(1, 0, 0, 1280, 800)
        layout = MODULE.overlay_layout(geometry)
        model = MODULE.settings_panel_model(
            {
                "mouse_settings": {
                    "current": {
                        "profile": "remote",
                        "effective_scale": 0.01,
                    },
                    "next_launch": {
                        "profile": "remote",
                        "effective_scale": 0.01,
                    },
                },
                "restart": {"available": True, "requested": False},
            }
        )
        overlay = object.__new__(MODULE.X11CalibrationOverlay)
        overlay._x11 = mock.Mock()
        overlay._display = 1
        overlay._windows = {"panel": 2}
        overlay._panel_gc = 3
        overlay._colours = {
            name: index
            for index, name in enumerate(
                (
                    "white",
                    "muted",
                    "selected",
                    "button",
                    "disabled",
                    "pending",
                    "error",
                    "apply",
                ),
                10,
            )
        }
        overlay._draw_text = mock.Mock()
        overlay._draw_button = mock.Mock()

        overlay._draw_panel(layout, model)

        labels = [call.args[0] for call in overlay._draw_text.call_args_list]
        self.assertIn("0.01x", labels)
        combined = " | ".join(labels)
        self.assertIn("0.01-0.10", combined)
        self.assertIn("0.20-1.00", combined)

        overlay._draw_text.reset_mock()
        compact_layout = MODULE.overlay_layout(
            MODULE.WindowGeometry(1, 0, 0, 480, 360)
        )
        overlay._draw_panel(compact_layout, model)
        compact_labels = [
            call.args[0] for call in overlay._draw_text.call_args_list
        ]
        self.assertNotIn("0.01-0.10", " | ".join(compact_labels))
        self.assertNotIn("0.20-1.00", " | ".join(compact_labels))

    def test_font_fallbacks_match_heyuan_xlsfonts_probe(self) -> None:
        self.assertEqual(MODULE._LARGE_FONT_CANDIDATES[0], b"12x24")
        self.assertEqual(MODULE._BODY_FONT_CANDIDATES[:2], (b"10x20", b"9x15"))


class PointerActionPublisherTest(unittest.TestCase):
    def test_packets_are_bounded_ordered_and_session_bound(self) -> None:
        receiver, sender = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        publisher = MODULE.PointerActionPublisher(
            file_descriptor=sender.detach(),
            session="known-session",
        )
        try:
            publisher.publish("profile_remote")
            publisher.publish("speed_down")
            first = json.loads(receiver.recv(1024).decode("ascii"))
            second = json.loads(receiver.recv(1024).decode("ascii"))
            self.assertEqual(first["session"], "known-session")
            self.assertEqual((first["sequence"], second["sequence"]), (1, 2))
            self.assertEqual(second["action"], "speed_down")
            with self.assertRaisesRegex(ValueError, "unsupported"):
                publisher.publish("restart_directly")
        finally:
            publisher.close()
            receiver.close()


class OverlayRenderCacheTest(unittest.TestCase):
    @staticmethod
    def state(*, scale: float = 0.5) -> dict[str, object]:
        return {
            "version": 1,
            "active": True,
            "mouse_settings": {
                "current": {"profile": "remote", "effective_scale": 0.5},
                "next_launch": {
                    "profile": "remote",
                    "effective_scale": scale,
                },
                "pending_restart": scale != 0.5,
            },
            "restart": {"available": True, "requested": False},
        }

    def make_overlay(self):
        overlay = object.__new__(MODULE.X11CalibrationOverlay)
        overlay._x11 = mock.Mock()
        overlay._display = 1
        overlay._windows = {
            name: index
            for index, name in enumerate(MODULE.X11CalibrationOverlay._WINDOW_ORDER, 10)
        }
        overlay._visible = False
        overlay._cursor_visible = False
        overlay._last_layout = None
        overlay._last_geometry = None
        overlay._last_panel_model = None
        overlay._last_pointer = None
        overlay._last_raise_s = None
        overlay._pressed_action = None
        overlay._pressed_window = None
        overlay._draw_panel = mock.Mock()
        return overlay

    def test_steady_30hz_frames_only_move_cursor_and_do_not_redraw(self) -> None:
        overlay = self.make_overlay()
        geometry = MODULE.WindowGeometry(41, 100, 80, 1280, 800)
        state = self.state()
        overlay.show(geometry, (300, 300), state, now_s=10.0)
        self.assertEqual(overlay._draw_panel.call_count, 1)
        initial_static_moves = overlay._x11.XMoveResizeWindow.call_count
        self.assertEqual(initial_static_moves, 8)

        # Provider heartbeat-only JSON changes are intentionally absent from
        # the validated render key.  Only the proxy cursor moves at 30 Hz.
        heartbeat = {**state, "updated_monotonic_s": 99.0}
        overlay.show(geometry, (301, 302), heartbeat, now_s=10.03)
        self.assertEqual(overlay._draw_panel.call_count, 1)
        self.assertEqual(
            overlay._x11.XMoveResizeWindow.call_count,
            initial_static_moves + 2,
        )
        raises = overlay._x11.XRaiseWindow.call_count
        overlay.show(geometry, (301, 302), heartbeat, now_s=10.06)
        self.assertEqual(overlay._draw_panel.call_count, 1)
        self.assertEqual(overlay._x11.XMoveResizeWindow.call_count, initial_static_moves + 2)
        self.assertEqual(overlay._x11.XRaiseWindow.call_count, raises)

        # A low-frequency stack repair raises all eight windows without
        # clearing/redrawing the 1180x736 panel.
        overlay.show(geometry, (301, 302), heartbeat, now_s=11.1)
        self.assertEqual(overlay._x11.XRaiseWindow.call_count, raises + 8)
        self.assertEqual(overlay._draw_panel.call_count, 1)

        changed = self.state(scale=0.4)
        overlay.show(geometry, (301, 302), changed, now_s=11.2)
        self.assertEqual(overlay._draw_panel.call_count, 2)
        self.assertEqual(overlay._x11.XMoveResizeWindow.call_count, initial_static_moves + 2)


class TargetCacheTest(unittest.TestCase):
    def test_cached_live_pid_avoids_rewalking_the_x11_tree(self) -> None:
        overlay = object.__new__(MODULE.X11CalibrationOverlay)
        overlay.expected_ue_pid = 1234
        overlay._target_window = 99
        overlay._window_pid = lambda window: 1234 if window == 99 else None
        expected = MODULE.WindowGeometry(99, 1, 2, 800, 600)
        overlay._geometry = lambda window: expected if window == 99 else None
        overlay._children = lambda _window: self.fail("cached target walked tree")

        self.assertEqual(overlay.find_target(), expected)

    def test_invalid_cached_window_is_replaced_by_largest_viewable_match(self) -> None:
        overlay = object.__new__(MODULE.X11CalibrationOverlay)
        overlay.expected_ue_pid = 1234
        overlay._root = 1
        overlay._target_window = 90
        tree = {1: [10, 20], 10: [], 20: []}
        overlay._children = lambda window: tree[window]
        overlay._window_pid = lambda window: {
            90: None,
            10: 1234,
            20: 1234,
        }.get(window)
        geometries = {
            10: MODULE.WindowGeometry(10, 0, 0, 320, 240),
            20: MODULE.WindowGeometry(20, 0, 0, 1280, 720),
        }
        overlay._geometry = geometries.get

        self.assertEqual(overlay.find_target(), geometries[20])
        self.assertEqual(overlay._target_window, 20)


@unittest.skipUnless(sys.platform.startswith("linux"), "Linux prctl is required")
class ParentDeathTest(unittest.TestCase):
    def test_overlay_child_dies_when_its_provider_is_sigkilled(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            ready = Path(temporary) / "ready"
            child_source = (
                "import os, pathlib, sys, time; "
                f"sys.path.insert(0, {os.fspath(REPO_ROOT / 'scripts')!r}); "
                "import matrix_calibration_overlay as overlay; "
                "overlay.arm_parent_death_signal(os.getppid()); "
                f"pathlib.Path({os.fspath(ready)!r}).write_text('ready'); "
                "time.sleep(30)"
            )
            helper_source = (
                "import pathlib, subprocess, sys, time; "
                f"child=subprocess.Popen([sys.executable, '-c', {child_source!r}]); "
                f"ready=pathlib.Path({os.fspath(ready)!r}); "
                "deadline=time.monotonic()+5; "
                "\nwhile not ready.exists() and time.monotonic()<deadline: time.sleep(.01)\n"
                "print(child.pid, flush=True); time.sleep(30)"
            )
            helper = subprocess.Popen(
                [sys.executable, "-c", helper_source],
                stdout=subprocess.PIPE,
                text=True,
            )
            assert helper.stdout is not None
            line = helper.stdout.readline().strip()
            self.assertTrue(line.isdigit(), msg=f"helper output: {line!r}")
            child_pid = int(line)
            try:
                os.kill(helper.pid, signal.SIGKILL)
                helper.wait(timeout=3.0)
                deadline = time.monotonic() + 3.0
                while time.monotonic() < deadline:
                    stat = Path(f"/proc/{child_pid}/stat")
                    if not stat.exists():
                        break
                    try:
                        fields = stat.read_text(encoding="utf-8").split()
                    except OSError:
                        # The process can disappear between exists() and read().
                        break
                    if len(fields) > 2 and fields[2] == "Z":
                        break
                    time.sleep(0.02)
                else:
                    self.fail("overlay child survived provider SIGKILL")
            finally:
                try:
                    os.kill(child_pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                if helper.poll() is None:
                    helper.kill()
                    helper.wait(timeout=2.0)
                helper.stdout.close()


@unittest.skipUnless(
    all(
        shutil.which(command)
        for command in ("Xvfb", "xdotool", "xev", "stdbuf", "xprop", "xwininfo")
    ),
    "Xvfb and X11 smoke-test tools are required",
)
class X11IntegrationTest(unittest.TestCase):
    @staticmethod
    def run_x11(environment: dict[str, str], *command: str) -> str:
        return subprocess.run(
            command,
            env=environment,
            text=True,
            capture_output=True,
            timeout=5.0,
            check=True,
        ).stdout

    @staticmethod
    def wait_until(predicate, *, timeout: float = 5.0):
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            try:
                last = predicate()
                if last:
                    return last
            except (OSError, subprocess.SubprocessError, ValueError):
                pass
            time.sleep(0.03)
        raise AssertionError(f"X11 smoke condition timed out; last={last!r}")

    @classmethod
    def geometry(cls, environment: dict[str, str], window: str) -> dict[str, int]:
        output = cls.run_x11(environment, "xdotool", "getwindowgeometry", "--shell", window)
        values: dict[str, int] = {}
        for line in output.splitlines():
            key, separator, value = line.partition("=")
            if separator and key in {"X", "Y", "WIDTH", "HEIGHT"}:
                values[key] = int(value)
        if set(values) != {"X", "Y", "WIDTH", "HEIGHT"}:
            raise ValueError(f"incomplete geometry: {output!r}")
        return values

    @classmethod
    def client_geometry(
        cls, environment: dict[str, str], window: str
    ) -> dict[str, int]:
        output = cls.run_x11(environment, "xwininfo", "-id", window)
        labels = {
            "Absolute upper-left X": "X",
            "Absolute upper-left Y": "Y",
            "Width": "WIDTH",
            "Height": "HEIGHT",
            "Border width": "BORDER",
        }
        values: dict[str, int] = {}
        for line in output.splitlines():
            label, separator, value = line.strip().partition(":")
            if separator and label in labels:
                values[labels[label]] = int(value.strip())
        if set(values) != {"X", "Y", "WIDTH", "HEIGHT", "BORDER"}:
            raise ValueError(f"incomplete client geometry: {output!r}")
        # xwininfo's absolute origin is the outside of this window's border,
        # whereas XTranslateCoordinates(window, root, 0, 0) returns the
        # drawable client origin followed by the overlay implementation.
        values["X"] += values["BORDER"]
        values["Y"] += values["BORDER"]
        del values["BORDER"]
        return values

    @classmethod
    def pointer_location(
        cls, environment: dict[str, str]
    ) -> dict[str, int]:
        output = cls.run_x11(environment, "xdotool", "getmouselocation", "--shell")
        values: dict[str, int] = {}
        for line in output.splitlines():
            key, separator, value = line.partition("=")
            if separator and key in {"X", "Y", "WINDOW"}:
                values[key] = int(value)
        if set(values) != {"X", "Y", "WINDOW"}:
            raise ValueError(f"incomplete pointer location: {output!r}")
        return values

    def test_click_through_overlay_follows_client_and_preserves_focus(self) -> None:
        xvfb = subprocess.Popen(
            ["Xvfb", "-displayfd", "1", "-screen", "0", "1280x800x24", "-nolisten", "tcp"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        assert xvfb.stdout is not None
        display_number = xvfb.stdout.readline().strip()
        self.assertTrue(display_number.isdigit(), msg=f"Xvfb display={display_number!r}")
        environment = {**os.environ, "DISPLAY": f":{display_number}"}
        target: subprocess.Popen[bytes] | None = None
        overlay: subprocess.Popen[bytes] | None = None
        action_receiver: socket.socket | None = None
        action_sender: socket.socket | None = None
        target_events = bytearray()
        try:
            target = subprocess.Popen(
                [
                    "stdbuf",
                    "-oL",
                    "xev",
                    "-name",
                    "MatrixSmoke",
                    "-geometry",
                    "800x600+100+80",
                    "-event",
                    "button",
                ],
                env=environment,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
            )
            assert target.stdout is not None
            os.set_blocking(target.stdout.fileno(), False)

            def read_target_events() -> bytes:
                while True:
                    try:
                        chunk = os.read(target.stdout.fileno(), 65536)
                    except BlockingIOError:
                        break
                    if not chunk:
                        break
                    target_events.extend(chunk)
                return bytes(target_events)
            target_window = self.wait_until(
                lambda: self.run_x11(
                    environment, "xdotool", "search", "--name", "^MatrixSmoke$"
                ).splitlines()[0]
            )
            self.run_x11(
                environment,
                "xprop",
                "-id",
                target_window,
                "-f",
                "_NET_WM_PID",
                "32c",
                "-set",
                "_NET_WM_PID",
                str(target.pid),
            )
            self.run_x11(environment, "xdotool", "windowfocus", target_window)
            focus_before = self.run_x11(environment, "xdotool", "getwindowfocus").strip()

            with tempfile.TemporaryDirectory() as temporary:
                state = Path(temporary) / "state.json"
                status = Path(temporary) / "status.json"
                MODULE.atomic_json(
                    state,
                    {
                        "version": 1,
                        "active": True,
                        "mouse_settings": {
                            "current": {"profile": "local", "effective_scale": 1.0},
                            "next_launch": {"profile": "local", "effective_scale": 1.0},
                            "pending_restart": False,
                        },
                        "restart": {"available": True, "requested": False},
                    },
                )
                action_receiver, action_sender = socket.socketpair(
                    socket.AF_UNIX, socket.SOCK_SEQPACKET
                )
                action_receiver.setblocking(False)
                overlay = subprocess.Popen(
                    [
                        sys.executable,
                        os.fspath(SCRIPT),
                        "--state-file",
                        os.fspath(state),
                        "--status-file",
                        os.fspath(status),
                        "--expected-ue-pid",
                        str(target.pid),
                        "--expected-parent-pid",
                        str(os.getpid()),
                        "--action-fd",
                        str(action_sender.fileno()),
                        "--action-session",
                        "xvfb-test-session",
                        "--display",
                        environment["DISPLAY"],
                        "--poll-hz",
                        "60",
                    ],
                    env=environment,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                    pass_fds=(action_sender.fileno(),),
                )
                action_sender.close()
                action_sender = None
                ready_status = self.wait_until(
                    lambda: status.is_file()
                    and (
                        value := json.loads(status.read_text(encoding="utf-8"))
                    ).get("ready")
                    is True
                    and value
                )
                self.assertIn(
                    ready_status["fonts"]["large"],
                    [name.decode("ascii") for name in MODULE._LARGE_FONT_CANDIDATES],
                )
                horizontal = self.wait_until(
                    lambda: self.run_x11(
                        environment,
                        "xdotool",
                        "search",
                        "--name",
                        "^Matrix Calibration horizontal$",
                    ).splitlines()[0]
                )
                overlay_windows = {"horizontal": int(horizontal)}
                for role in (
                    "shield",
                    "panel",
                    "horizontal-shadow",
                    "vertical-shadow",
                    "vertical",
                    "cursor-shadow",
                    "cursor",
                ):
                    window = self.wait_until(
                        lambda role=role: self.run_x11(
                            environment,
                            "xdotool",
                            "search",
                            "--name",
                            f"^Matrix Calibration {role}$",
                        ).splitlines()[0]
                    )
                    overlay_windows[role] = int(window)
                # xdotool reports the outer origin for bordered test windows;
                # the overlay intentionally follows X11 client coordinates.
                target_geometry = self.client_geometry(environment, target_window)
                observed_centres: list[dict[str, int]] = []

                def centred_geometry():
                    value = self.geometry(environment, horizontal)
                    observed_centres.append(value)
                    expected_x = target_geometry["X"] + target_geometry["WIDTH"] // 2 - 32
                    expected_y = target_geometry["Y"] + target_geometry["HEIGHT"] // 2 - 1
                    return value if (value["X"], value["Y"]) == (expected_x, expected_y) else None

                try:
                    self.wait_until(centred_geometry)
                except AssertionError as exc:
                    stderr = ""
                    if overlay.poll() is not None and overlay.stderr is not None:
                        stderr = overlay.stderr.read().decode("utf-8", errors="replace")
                    self.fail(
                        f"{exc}; target={target_geometry}; "
                        f"last_cross={observed_centres[-1] if observed_centres else None}; "
                        f"overlay_code={overlay.poll()}; stderr={stderr!r}"
                    )
                focus_after = self.run_x11(environment, "xdotool", "getwindowfocus").strip()
                self.assertEqual(focus_after, focus_before)

                layout = MODULE.overlay_layout(
                    MODULE.WindowGeometry(
                        int(target_window),
                        target_geometry["X"],
                        target_geometry["Y"],
                        target_geometry["WIDTH"],
                        target_geometry["HEIGHT"],
                    )
                )
                remote_button = layout["profile_remote"]
                remote_point = (
                    remote_button[0] + remote_button[2] // 2,
                    remote_button[1] + remote_button[3] // 2,
                )
                self.run_x11(
                    environment,
                    "xdotool",
                    "mousemove",
                    "--sync",
                    str(remote_point[0]),
                    str(remote_point[1]),
                    "click",
                    "1",
                )

                def pointer_action():
                    try:
                        return json.loads(action_receiver.recv(1024).decode("ascii"))
                    except BlockingIOError:
                        return None

                action = self.wait_until(pointer_action)
                self.assertEqual(action["session"], "xvfb-test-session")
                self.assertEqual(action["action"], "profile_remote")
                focus_after_click = self.run_x11(
                    environment, "xdotool", "getwindowfocus"
                ).strip()
                self.assertEqual(focus_after_click, focus_before)
                time.sleep(0.08)
                self.assertNotIn(b"ButtonPress event", read_target_events())

                def assert_no_pointer_action() -> None:
                    assert action_receiver is not None
                    with self.assertRaises(BlockingIOError):
                        action_receiver.recv(1024)

                # A non-button part of the visible panel consumes the click but
                # emits no provider action and never reaches the UE target.
                panel = layout["panel"]
                panel_blank = (panel[0] + 8, panel[1] + 8)
                self.run_x11(
                    environment,
                    "xdotool",
                    "mousemove",
                    "--sync",
                    str(panel_blank[0]),
                    str(panel_blank[1]),
                    "click",
                    "1",
                )
                time.sleep(0.08)
                self.assertEqual(
                    self.pointer_location(environment)["WINDOW"],
                    overlay_windows["panel"],
                )
                assert_no_pointer_action()
                self.assertNotIn(b"ButtonPress event", read_target_events())

                # The transparent modal shield outside the panel also consumes
                # ButtonPress/Release instead of leaking them into the target.
                shield_blank = (
                    target_geometry["X"] + 3,
                    target_geometry["Y"] + 3,
                )
                self.run_x11(
                    environment,
                    "xdotool",
                    "mousemove",
                    "--sync",
                    str(shield_blank[0]),
                    str(shield_blank[1]),
                    "click",
                    "1",
                )
                time.sleep(0.08)
                self.assertEqual(
                    self.pointer_location(environment)["WINDOW"],
                    overlay_windows["shield"],
                )
                assert_no_pointer_action()
                self.assertNotIn(b"ButtonPress event", read_target_events())
                self.assertEqual(
                    self.run_x11(
                        environment, "xdotool", "getwindowfocus"
                    ).strip(),
                    focus_before,
                )

                # Every crosshair layer has an empty XFixes InputShape.  Moving
                # onto each mapped rectangle still resolves pointer input to
                # the interactive panel underneath, never to a visual window.
                click_through_roles = (
                    "horizontal-shadow",
                    "vertical-shadow",
                    "horizontal",
                    "vertical",
                )
                for role in click_through_roles:
                    rectangle = self.geometry(
                        environment, str(overlay_windows[role])
                    )
                    point = (
                        rectangle["X"] + rectangle["WIDTH"] // 2,
                        rectangle["Y"] + rectangle["HEIGHT"] // 2,
                    )
                    self.run_x11(
                        environment,
                        "xdotool",
                        "mousemove",
                        str(point[0]),
                        str(point[1]),
                    )
                    self.wait_until(
                        lambda point=point: (
                            pointer
                            if (
                                (pointer := self.pointer_location(environment))["X"],
                                pointer["Y"],
                            )
                            == point
                            else None
                        )
                    )
                    self.assertNotIn(
                        self.pointer_location(environment)["WINDOW"],
                        {overlay_windows[name] for name in click_through_roles},
                    )

                centre_x = target_geometry["X"] + target_geometry["WIDTH"] // 2
                centre_y = target_geometry["Y"] + target_geometry["HEIGHT"] // 2
                expected_pointer = (centre_x + 113, centre_y + 71)
                self.run_x11(
                    environment,
                    "xdotool",
                    "mousemove",
                    str(expected_pointer[0]),
                    str(expected_pointer[1]),
                )

                def cursor_follows_hotspot():
                    pointer = self.pointer_location(environment)
                    cursor = self.geometry(environment, str(overlay_windows["cursor"]))
                    shadow = self.geometry(
                        environment, str(overlay_windows["cursor-shadow"])
                    )
                    expected = (pointer["X"], pointer["Y"])
                    if (
                        expected == expected_pointer
                        and (cursor["X"], cursor["Y"]) == expected
                        and (shadow["X"], shadow["Y"]) == expected
                    ):
                        return pointer
                    return None

                pointer = self.wait_until(cursor_follows_hotspot)
                self.assertNotIn(
                    pointer["WINDOW"],
                    {
                        overlay_windows["horizontal"],
                        overlay_windows["horizontal-shadow"],
                        overlay_windows["vertical"],
                        overlay_windows["vertical-shadow"],
                        overlay_windows["cursor"],
                        overlay_windows["cursor-shadow"],
                    },
                )
                focus_after_pointer = self.run_x11(
                    environment, "xdotool", "getwindowfocus"
                ).strip()
                self.assertEqual(focus_after_pointer, focus_before)

                self.run_x11(environment, "xdotool", "windowmove", target_window, "20", "30")
                self.run_x11(environment, "xdotool", "windowsize", target_window, "700", "500")

                def moved_and_followed():
                    moved_target = self.client_geometry(environment, target_window)
                    moved_cross = self.geometry(environment, horizontal)
                    return (
                        moved_cross
                        if (
                            moved_cross["X"]
                            == moved_target["X"] + moved_target["WIDTH"] // 2 - 32
                            and moved_cross["Y"]
                            == moved_target["Y"] + moved_target["HEIGHT"] // 2 - 1
                        )
                        else None
                    )

                self.wait_until(moved_and_followed)
                MODULE.atomic_json(state, {"version": 1, "active": False})

                def all_are_unmapped():
                    self.assertEqual(len(overlay_windows), 8)
                    return all(
                        "Map State: IsUnMapped"
                        in self.run_x11(
                            environment, "xwininfo", "-id", str(window)
                        )
                        for window in overlay_windows.values()
                    )

                self.wait_until(all_are_unmapped)
                hidden_target = self.client_geometry(environment, target_window)
                self.run_x11(
                    environment,
                    "xdotool",
                    "mousemove",
                    "--sync",
                    str(hidden_target["X"] + hidden_target["WIDTH"] // 2),
                    str(hidden_target["Y"] + hidden_target["HEIGHT"] // 2),
                    "click",
                    "1",
                )
                self.wait_until(
                    lambda: b"ButtonPress event" in read_target_events()
                )
        finally:
            for connection in (action_receiver, action_sender):
                if connection is not None:
                    connection.close()
            for process in (overlay, target):
                if process is not None and process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=2.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=2.0)
            if overlay is not None and overlay.stderr is not None:
                overlay.stderr.close()
            if target is not None and target.stdout is not None:
                target.stdout.close()
            if xvfb.poll() is None:
                xvfb.terminate()
                try:
                    xvfb.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    xvfb.kill()
                    xvfb.wait(timeout=2.0)
            xvfb.stdout.close()
            assert xvfb.stderr is not None
            xvfb.stderr.close()


if __name__ == "__main__":
    unittest.main()
