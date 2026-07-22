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
        self.assertGreaterEqual(layout["command_result"][3], 14)
        self.assertGreaterEqual(layout["command_input"][3], 22)
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
            "font_down",
            "font_value",
            "font_up",
            "command_input",
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
        for name in MODULE._PANEL_ACTIONS + (
            "speed_value",
            "font_value",
            "crosshair_safe",
        ):
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
            "font_down",
            "font_value",
            "font_up",
            "command_input",
            "apply_return",
        ):
            self.assertFalse(
                self.intersects(layout[name], layout["crosshair_safe"])
            )
        tiny = MODULE.WindowGeometry(1, 0, 0, 479, 359)
        self.assertFalse(MODULE.overlay_supported(tiny))
        with self.assertRaisesRegex(ValueError, "too small"):
            MODULE.overlay_layout(tiny)

    def test_command_input_is_a_separate_hit_target_below_crosshair(self) -> None:
        geometry = MODULE.WindowGeometry(1, 0, 0, 480, 360)
        layout = MODULE.overlay_layout(geometry)
        command_input = layout["command_input"]
        point = (
            command_input[0] + command_input[2] // 2,
            command_input[1] + command_input[3] // 2,
        )
        self.assertEqual(MODULE.panel_action_at(layout, *point), "command_input")
        self.assertFalse(self.intersects(command_input, layout["crosshair_safe"]))
        self.assertGreaterEqual(layout["command_result"][3], 14)
        self.assertFalse(self.intersects(layout["title"], layout["profile_local"]))
        self.assertFalse(self.intersects(layout["title"], layout["profile_remote"]))

    def test_strategy_targets_are_page_scoped_and_outside_crosshair(self) -> None:
        geometry = MODULE.WindowGeometry(1, 0, 0, 1280, 800)
        layout = MODULE.overlay_layout(geometry)
        for index in range(3):
            name = f"recovery_policy_{index}"
            x, y, width, height = layout[name]
            self.assertEqual(
                MODULE.panel_action_at(
                    layout,
                    x + width // 2,
                    y + height // 2,
                    page="loadout",
                ),
                name,
            )
            self.assertFalse(self.intersects(layout[name], layout["crosshair_safe"]))

    def test_inventory_targets_are_page_scoped_and_bounded(self) -> None:
        geometry = MODULE.WindowGeometry(1, 0, 0, 1280, 800)
        layout = MODULE.overlay_layout(geometry)
        panel = layout["panel"]
        for index in range(4):
            name = f"creative_item_{index}"
            x, y, width, height = layout[name]
            point = (x + width // 2, y + height // 2)
            self.assertEqual(
                MODULE.panel_action_at(layout, *point, page="inventory"),
                name,
            )
            self.assertNotEqual(
                MODULE.panel_action_at(layout, *point, page="settings"),
                name,
            )
            self.assertTrue(MODULE.point_in_rectangle((x, y), panel))
            self.assertTrue(
                MODULE.point_in_rectangle((x + width - 1, y + height - 1), panel)
            )

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
        self.assertEqual(
            MODULE.ctypes.sizeof(MODULE.XKeyEvent),
            MODULE.ctypes.sizeof(MODULE.XButtonEvent),
        )
        self.assertEqual(MODULE.XKeyEvent.keycode.offset, MODULE.XButtonEvent.button.offset)

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
        self.assertIn(b"RETURN TO GAME & APPLY", lines[2])
        self.assertIn(b"Enter runs", lines[2])
        self.assertIn(b"ESC leaves editor first", lines[2])
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

    def test_command_state_is_strict_and_alias_warning_is_ascii_readable(self) -> None:
        status = MODULE.command_console_status(
            {
                "command_console": {
                    "available": True,
                    "editing": 1,
                    "in_flight": False,
                    "status": "success",
                    "sequence": 4,
                    "result_revision": 7,
                    "ok": True,
                    "warning": "已兼容执行；标准命令是 /summon",
                }
            }
        )
        self.assertTrue(status.available)
        self.assertFalse(status.provider_editing)
        self.assertEqual(status.result_revision, 7)
        self.assertEqual(status.warning, "Accepted /summom alias; standard command is /summon")
        malformed = MODULE.command_console_status(
            {
                "command_console": {
                    "status": {},
                    "sequence": True,
                    "result_revision": True,
                }
            }
        )
        self.assertEqual(malformed.status, "unavailable")
        self.assertIsNone(malformed.sequence)
        self.assertEqual(malformed.result_revision, 0)

    def test_strategy_loadout_model_exposes_two_slots_and_pending_selection(self) -> None:
        model = MODULE.strategy_loadout_model(
            {
                "strategy_loadout": {
                    "version": 1,
                    "available": True,
                    "status": "switching",
                    "active_slot": "locomotion",
                    "pending": {"policy_id": "host"},
                    "slots": [
                        {
                            "slot": "locomotion",
                            "selected_policy_id": "sonic",
                        },
                        {
                            "slot": "recovery",
                            "selected_policy_id": "kungfu",
                            "candidates": [
                                {
                                    "policy_id": "kungfu",
                                    "resident": True,
                                    "available": True,
                                },
                                {
                                    "policy_id": "host",
                                    "resident": True,
                                    "available": True,
                                },
                            ],
                        },
                    ],
                }
            }
        )
        self.assertEqual(model.locomotion_policy_id, "sonic")
        self.assertEqual(model.recovery_policy_id, "kungfu")
        self.assertEqual(model.pending_policy_id, "host")
        self.assertEqual(
            [candidate.policy_id for candidate in model.recovery_candidates],
            ["kungfu", "host"],
        )
        self.assertFalse(model.policy_enabled("host"))

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
                    "outline",
                ),
                10,
            )
        }
        overlay._command_editor = MODULE.CommandLineEditor()
        overlay._last_command_status = MODULE.command_console_status({})
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
        self.assertIn(b"size=13", MODULE.xft_font_candidates(1.0, large=False)[0])
        self.assertIn(b"size=27", MODULE.xft_font_candidates(1.5, large=True)[0])

    def test_persisted_font_scale_controls_are_bounded(self) -> None:
        def model(scale: object):
            return MODULE.settings_panel_model(
                {
                    "ui_settings": {"font_scale": scale},
                    "restart": {"available": True, "requested": False},
                }
            )

        minimum = model(0.8)
        self.assertEqual(minimum.font_scale, 0.8)
        self.assertFalse(minimum.action_enabled("font_down"))
        self.assertTrue(minimum.action_enabled("font_up"))
        maximum = model(1.5)
        self.assertTrue(maximum.action_enabled("font_down"))
        self.assertFalse(maximum.action_enabled("font_up"))
        self.assertEqual(model(1.05).font_scale, 1.0)


class PointerActionPublisherTest(unittest.TestCase):
    def test_strategy_selection_is_a_strict_typed_intent(self) -> None:
        receiver, sender = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        publisher = MODULE.PointerActionPublisher(
            file_descriptor=sender.detach(),
            session="known-session",
        )
        try:
            publisher.publish_strategy_select("recovery", "KungFu")
            packet = json.loads(receiver.recv(1024).decode("ascii"))
            self.assertEqual(packet["kind"], "strategy_select")
            self.assertEqual(packet["slot"], "recovery")
            self.assertEqual(packet["policy_id"], "kungfu")
            with self.assertRaisesRegex(ValueError, "invalid"):
                publisher.publish_strategy_select("recovery", "bad policy")
        finally:
            publisher.close()
            receiver.close()

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
            self.assertEqual(first["kind"], "action")
            self.assertEqual(second["action"], "speed_down")
            with self.assertRaisesRegex(ValueError, "unsupported"):
                publisher.publish("restart_directly")
        finally:
            publisher.close()
            receiver.close()

    def test_command_intents_have_disjoint_strict_shapes(self) -> None:
        receiver, sender = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        publisher = MODULE.PointerActionPublisher(
            file_descriptor=sender.detach(),
            session="known-session",
        )
        try:
            publisher.publish_command_edit(True)
            publisher.publish_command_submit("/tp @s ~ ~ ~")
            publisher.publish_command_edit(False)
            packets = [
                json.loads(receiver.recv(MODULE._MAX_INTENT_PACKET_BYTES).decode("ascii"))
                for _ in range(3)
            ]
            self.assertEqual(
                set(packets[0]),
                {"version", "session", "sequence", "kind", "active"},
            )
            self.assertEqual(packets[0]["kind"], "command_edit")
            self.assertIs(packets[0]["active"], True)
            self.assertEqual(
                set(packets[1]),
                {"version", "session", "sequence", "kind", "command"},
            )
            self.assertEqual(packets[1]["kind"], "command_submit")
            self.assertEqual(packets[1]["command"], "/tp @s ~ ~ ~")
            self.assertEqual([packet["sequence"] for packet in packets], [1, 2, 3])
            with self.assertRaisesRegex(ValueError, "printable ASCII"):
                publisher.publish_command_submit("/tp @s 1 2 3\n")
            with self.assertRaisesRegex(ValueError, "printable ASCII"):
                publisher.publish_command_submit("x" * (MODULE.MAX_COMMAND_CHARS + 1))
            with self.assertRaisesRegex(ValueError, "boolean"):
                publisher.publish_command_edit(1)
        finally:
            publisher.close()
            receiver.close()

    def test_creative_spawn_is_a_strict_typed_intent(self) -> None:
        receiver, sender = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        publisher = MODULE.PointerActionPublisher(
            file_descriptor=sender.detach(),
            session="known-session",
        )
        try:
            publisher.publish_creative_spawn("training_blaster")
            packet = json.loads(receiver.recv(1024).decode("ascii"))
            self.assertEqual(
                packet,
                {
                    "version": 1,
                    "session": "known-session",
                    "sequence": 1,
                    "kind": "creative_spawn",
                    "item_id": "training_blaster",
                },
            )
            with self.assertRaisesRegex(ValueError, "invalid"):
                publisher.publish_creative_spawn("../../bad")
        finally:
            publisher.close()
            receiver.close()


class CommandLineEditorTest(unittest.TestCase):
    @staticmethod
    def status(**overrides):
        values = {
            "available": True,
            "provider_editing": False,
            "in_flight": False,
            "status": "idle",
            "request_id": None,
            "sequence": None,
            "result_revision": 0,
            "ok": None,
            "code": None,
            "message": None,
            "warning": None,
            "restart_required": False,
            "outcome_unknown": False,
        }
        values.update(overrides)
        return MODULE.CommandConsoleStatus(**values)

    @staticmethod
    def key(editor, keysym=0, printable="", status=None):
        return editor.handle_key(
            keysym=keysym,
            printable=printable,
            status=status or CommandLineEditorTest.status(),
        )

    def test_bounded_ascii_cursor_and_delete_editing(self) -> None:
        editor = MODULE.CommandLineEditor()
        self.assertTrue(editor.begin())
        self.key(editor, printable="abcd")
        self.key(editor, MODULE._XK_LEFT)
        self.key(editor, MODULE._XK_LEFT)
        self.key(editor, MODULE._XK_BACK_SPACE)
        self.assertEqual((editor.text, editor.cursor), ("acd", 1))
        self.key(editor, MODULE._XK_DELETE)
        self.assertEqual((editor.text, editor.cursor), ("ad", 1))
        self.key(editor, MODULE._XK_HOME)
        self.key(editor, printable="/")
        self.key(editor, MODULE._XK_END)
        self.key(editor, printable=" " + "x" * 600)
        self.assertEqual(len(editor.text), MODULE.MAX_COMMAND_CHARS)
        before = editor.text
        self.key(editor, printable="é")
        self.assertEqual(editor.text, before)

    def test_history_pending_single_submit_and_escape_gate(self) -> None:
        editor = MODULE.CommandLineEditor()
        idle = self.status()
        editor.begin()
        self.key(editor, printable="/tp @s 1 2 3")
        submitted = self.key(editor, MODULE._XK_RETURN, status=idle)
        self.assertEqual(submitted.command, "/tp @s 1 2 3")
        self.assertTrue(editor.pending)
        self.assertEqual(editor.history, ["/tp @s 1 2 3"])
        self.assertIsNone(self.key(editor, MODULE._XK_RETURN).action)
        self.assertIsNone(self.key(editor, MODULE._XK_ESCAPE).action)
        self.assertTrue(editor.editing)

        success = self.status(
            status="success",
            request_id="cmd-" + "1" * 32,
            sequence=1,
            result_revision=1,
            ok=True,
            code="OK_TELEPORT",
            message="done",
        )
        self.assertTrue(editor.reconcile(success))
        self.assertFalse(editor.pending)
        ended = self.key(editor, MODULE._XK_ESCAPE, status=success)
        self.assertEqual(ended.action, "end")
        self.assertFalse(editor.editing)

        editor.begin()
        self.key(editor, printable="draft")
        self.key(editor, MODULE._XK_UP)
        self.assertEqual(editor.text, "/tp @s 1 2 3")
        self.key(editor, MODULE._XK_DOWN)
        self.assertEqual(editor.text, "draft")

    def test_repeated_identical_error_uses_result_revision_to_clear_pending(self) -> None:
        prior = self.status(
            status="error",
            result_revision=4,
            ok=False,
            code="E_COMMAND_UNKNOWN",
            message="supported commands are /summon and /tp",
        )
        editor = MODULE.CommandLineEditor()
        editor.begin()
        self.key(editor, printable="bad")
        self.assertEqual(
            self.key(editor, MODULE._XK_RETURN, status=prior).action,
            "submit",
        )
        repeated = self.status(
            status="error",
            result_revision=5,
            ok=False,
            code=prior.code,
            message=prior.message,
        )
        self.assertTrue(editor.reconcile(repeated))
        self.assertFalse(editor.pending)


class KeyboardGrabLifecycleTest(unittest.TestCase):
    @staticmethod
    def overlay(*, visible=True, grab_result=MODULE._GRAB_SUCCESS):
        overlay = object.__new__(MODULE.X11CalibrationOverlay)
        overlay._x11 = mock.Mock()
        overlay._x11.XGrabKeyboard.return_value = grab_result
        overlay._display = 1
        overlay._windows = {"panel": 2}
        overlay._visible = visible
        overlay._cursor_visible = False
        overlay._keyboard_grabbed = False
        overlay._deferred_ungrab_keycode = None
        overlay._command_editor = MODULE.CommandLineEditor()
        overlay._last_command_status = CommandLineEditorTest.status()
        overlay._last_layout = None
        overlay._last_geometry = None
        overlay._last_panel_model = None
        overlay._last_command_revision = -1
        overlay._last_pointer = None
        overlay._last_raise_s = None
        overlay._pressed_action = None
        overlay._pressed_window = None
        return overlay

    def test_grab_exists_only_during_visible_edit_and_hide_releases(self) -> None:
        publisher = mock.Mock()
        overlay = self.overlay()
        self.assertTrue(overlay._begin_command_editing(publisher))
        self.assertTrue(overlay._keyboard_grabbed)
        publisher.publish_command_edit.assert_called_once_with(True)
        overlay._deferred_ungrab_keycode = 42
        overlay.hide(publisher)
        self.assertFalse(overlay._keyboard_grabbed)
        self.assertIsNone(overlay._deferred_ungrab_keycode)
        self.assertFalse(overlay._command_editor.editing)
        publisher.publish_command_edit.assert_has_calls([mock.call(True), mock.call(False)])
        overlay._x11.XUngrabKeyboard.assert_called_once_with(1, MODULE._CURRENT_TIME)

        hidden = self.overlay(visible=False)
        with self.assertRaisesRegex(RuntimeError, "hidden"):
            hidden._grab_keyboard()
        hidden._x11.XGrabKeyboard.assert_not_called()

    def test_failed_begin_and_close_both_fail_safe_to_ungrabbed(self) -> None:
        publisher = mock.Mock()
        failed = self.overlay(grab_result=1)
        with self.assertRaisesRegex(RuntimeError, "X11 status 1"):
            failed._begin_command_editing(publisher)
        self.assertFalse(failed._keyboard_grabbed)
        self.assertFalse(failed._command_editor.editing)
        publisher.publish_command_edit.assert_not_called()

        closing = self.overlay()
        closing._keyboard_grabbed = True
        closing._deferred_ungrab_keycode = 42
        closing._command_editor.begin()
        closing._panel_gc = None
        closing._body_font = None
        closing._large_font = None
        closing.close()
        self.assertFalse(closing._keyboard_grabbed)
        self.assertIsNone(closing._deferred_ungrab_keycode)
        calls = closing._x11.method_calls
        self.assertLess(
            calls.index(mock.call.XUngrabKeyboard(1, MODULE._CURRENT_TIME)),
            calls.index(mock.call.XCloseDisplay(1)),
        )

    def test_escape_keeps_grab_until_physical_release(self) -> None:
        overlay = self.overlay()
        overlay._deferred_ungrab_keycode = None
        overlay._command_editor.begin()
        overlay._keyboard_grabbed = True
        overlay._lookup_key = lambda _event: (MODULE._XK_ESCAPE, "")
        publisher = mock.Mock()
        event = MODULE.XKeyEvent()
        event.keycode = 42

        self.assertEqual(overlay._handle_key_press(event, publisher), 1)
        publisher.publish_command_edit.assert_called_once_with(False)
        self.assertTrue(overlay._keyboard_grabbed)
        self.assertEqual(overlay._deferred_ungrab_keycode, 42)

        overlay._release_key_is_still_down = mock.Mock(return_value=True)
        overlay._handle_key_release(event)
        self.assertTrue(overlay._keyboard_grabbed)
        overlay._release_key_is_still_down.return_value = False
        overlay._handle_key_release(event)
        self.assertFalse(overlay._keyboard_grabbed)
        self.assertIsNone(overlay._deferred_ungrab_keycode)

    def test_deferred_escape_release_blocks_click_reentry(self) -> None:
        overlay = self.overlay()
        publisher = mock.Mock()
        self.assertTrue(overlay._begin_command_editing(publisher))
        overlay._lookup_key = lambda _event: (MODULE._XK_ESCAPE, "")
        event = MODULE.XKeyEvent()
        event.keycode = 42
        self.assertEqual(overlay._handle_key_press(event, publisher), 1)
        self.assertFalse(overlay._command_editor.editing)
        self.assertTrue(overlay._keyboard_grabbed)
        self.assertEqual(overlay._deferred_ungrab_keycode, 42)

        # Pointer input remains live during a keyboard grab.  A click on the
        # command field must not create editing=true on top of the old grab.
        self.assertFalse(overlay._begin_command_editing(publisher))
        self.assertFalse(overlay._command_editor.editing)
        self.assertEqual(
            publisher.publish_command_edit.call_args_list,
            [mock.call(True), mock.call(False)],
        )

        overlay._release_key_is_still_down = mock.Mock(return_value=False)
        overlay._handle_key_release(event)
        self.assertFalse(overlay._keyboard_grabbed)
        self.assertIsNone(overlay._deferred_ungrab_keycode)
        self.assertTrue(overlay._begin_command_editing(publisher))
        self.assertTrue(overlay._command_editor.editing)
        self.assertTrue(overlay._keyboard_grabbed)
        self.assertEqual(
            publisher.publish_command_edit.call_args_list,
            [mock.call(True), mock.call(False), mock.call(True)],
        )

    def test_pending_or_restart_provider_state_blocks_click_reentry(self) -> None:
        publisher = mock.Mock()
        blocked_states = (
            CommandLineEditorTest.status(in_flight=True, status="pending"),
            CommandLineEditorTest.status(status="pending"),
            CommandLineEditorTest.status(
                status="restarting", restart_required=True
            ),
            CommandLineEditorTest.status(
                status="error",
                code="E_COMMAND_OUTCOME_UNKNOWN",
                outcome_unknown=True,
            ),
        )
        for status in blocked_states:
            with self.subTest(status=status.status, in_flight=status.in_flight):
                overlay = self.overlay()
                overlay._last_command_status = status
                self.assertFalse(overlay._begin_command_editing(publisher))
                self.assertFalse(overlay._command_editor.editing)
                self.assertFalse(overlay._keyboard_grabbed)
        publisher.publish_command_edit.assert_not_called()
        publisher.reset_mock()

        ready = self.overlay()
        ready._last_command_status = CommandLineEditorTest.status(status="success")
        self.assertTrue(ready._begin_command_editing(publisher))
        publisher.publish_command_edit.assert_called_once_with(True)


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
            "command_console": {
                "available": True,
                "editing": False,
                "in_flight": False,
                "status": "idle",
            },
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
        overlay._command_editor = MODULE.CommandLineEditor()
        overlay._last_command_status = MODULE.command_console_status({})
        overlay._last_command_revision = -1
        overlay._last_pointer = None
        overlay._last_raise_s = None
        overlay._pressed_action = None
        overlay._pressed_window = None
        overlay._keyboard_grabbed = False
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
                    "-event",
                    "keyboard",
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
                        "command_console": {
                            "available": True,
                            "editing": False,
                            "in_flight": False,
                            "status": "idle",
                            "request_id": None,
                            "sequence": 0,
                            "result_revision": 0,
                            "ok": None,
                            "code": None,
                            "message": None,
                            "warning": None,
                            "restart_required": False,
                            "data": None,
                        },
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
                font_backend = ready_status["fonts"]["backend"]
                expected_fonts = (
                    MODULE._XFT_LARGE_FONT_CANDIDATES
                    if font_backend == "xft-utf8"
                    else MODULE._LARGE_FONT_CANDIDATES
                )
                self.assertIn(
                    ready_status["fonts"]["large"],
                    [name.decode("ascii") for name in expected_fonts],
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
                settings_tab = layout["tab_settings"]
                self.run_x11(
                    environment,
                    "xdotool",
                    "mousemove",
                    "--sync",
                    str(settings_tab[0] + settings_tab[2] // 2),
                    str(settings_tab[1] + settings_tab[3] // 2),
                    "click",
                    "1",
                )
                time.sleep(0.08)
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
                        return json.loads(
                            action_receiver.recv(
                                MODULE._MAX_INTENT_PACKET_BYTES
                            ).decode("ascii")
                        )
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
                        action_receiver.recv(MODULE._MAX_INTENT_PACKET_BYTES)

                # Clicking the command line is the only keyboard-grab entry.
                # Focus remains on UE, but all typed keys are delivered to the
                # overlay until its first Escape ends editing.
                read_target_events()
                target_events.clear()
                console_tab = layout["tab_console"]
                self.run_x11(
                    environment,
                    "xdotool",
                    "mousemove",
                    "--sync",
                    str(console_tab[0] + console_tab[2] // 2),
                    str(console_tab[1] + console_tab[3] // 2),
                    "click",
                    "1",
                )
                time.sleep(0.08)
                command_input = layout["command_input"]
                command_point = (
                    command_input[0] + command_input[2] // 2,
                    command_input[1] + command_input[3] // 2,
                )
                self.run_x11(
                    environment,
                    "xdotool",
                    "mousemove",
                    "--sync",
                    str(command_point[0]),
                    str(command_point[1]),
                    "click",
                    "1",
                )
                begin_edit = self.wait_until(pointer_action)
                self.assertEqual(begin_edit["kind"], "command_edit")
                self.assertIs(begin_edit["active"], True)
                self.assertEqual(
                    self.run_x11(environment, "xdotool", "getwindowfocus").strip(),
                    focus_before,
                )

                command_text = "/tp @s 1 2 3"
                self.run_x11(
                    environment,
                    "xdotool",
                    "type",
                    "--delay",
                    "1",
                    command_text,
                )
                self.run_x11(environment, "xdotool", "key", "Return")
                submitted = self.wait_until(pointer_action)
                self.assertEqual(submitted["kind"], "command_submit")
                self.assertEqual(submitted["command"], command_text)
                time.sleep(0.08)
                self.assertNotIn(b"KeyPress event", read_target_events())

                # The local pending latch suppresses key-repeat/second submit
                # and Escape until a distinct terminal provider result appears.
                self.run_x11(environment, "xdotool", "key", "Return")
                self.run_x11(environment, "xdotool", "key", "Escape")
                time.sleep(0.08)
                assert_no_pointer_action()
                terminal = json.loads(state.read_text(encoding="utf-8"))
                terminal["command_console"] = {
                    "available": True,
                    "editing": True,
                    "in_flight": False,
                    "status": "success",
                    "request_id": "cmd-" + "1" * 32,
                    "sequence": 1,
                    "result_revision": 1,
                    "ok": True,
                    "code": "OK_SUMMONED",
                    "message": "Command completed",
                    "warning": "已兼容执行；标准命令是 /summon",
                    "restart_required": False,
                    "data": None,
                }
                MODULE.atomic_json(state, terminal)
                time.sleep(0.12)
                self.run_x11(environment, "xdotool", "keydown", "Escape")
                end_edit = self.wait_until(pointer_action)
                self.assertEqual(end_edit["kind"], "command_edit")
                self.assertIs(end_edit["active"], False)

                # Pointer input remains live during the deferred Escape grab.
                # Clicking the command field before keyup must not re-enter;
                # otherwise the old release would ungrab a new editor session.
                self.run_x11(
                    environment,
                    "xdotool",
                    "click",
                    "1",
                )
                time.sleep(0.08)
                assert_no_pointer_action()
                self.run_x11(environment, "xdotool", "keyup", "Escape")
                time.sleep(0.08)

                # Active keyboard grabs do not change X focus.  Once editing
                # ends, the next key reaches the still-focused target again.
                read_target_events()
                target_events.clear()
                self.run_x11(environment, "xdotool", "key", "m")
                self.wait_until(lambda: b"KeyPress event" in read_target_events())

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
