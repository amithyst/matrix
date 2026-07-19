from __future__ import annotations

import importlib
import importlib.util
import json
import math
import os
from pathlib import Path
import socket
import tempfile
import unittest
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = REPO_ROOT / "scripts"
if os.fspath(SCRIPTS) not in os.sys.path:
    os.sys.path.insert(0, os.fspath(SCRIPTS))
CORE = importlib.import_module("matrix_game_control")
RESTART = importlib.import_module("matrix_restart_request")
SCRIPT_PATH = SCRIPTS / "matrix_game_control_input.py"
SPEC = importlib.util.spec_from_file_location("matrix_game_control_input", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
os.sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class CalibrationOverlaySupervisorTest(unittest.TestCase):
    def test_isolated_overlay_explicitly_disables_bytecode_writes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            script = root / "matrix_calibration_overlay.py"
            script.write_text("", encoding="utf-8")
            supervisor = MODULE.CalibrationOverlaySupervisor(
                state_file=root / "state.json",
                display_name=":123",
                expected_ue_pid=41,
                script=script,
                python="/locked/venv/bin/python",
                startup_timeout_s=0.1,
            )

            class ReadyProcess:
                def poll(self):
                    supervisor.ready_file.write_text(
                        json.dumps({"ready": True}), encoding="utf-8"
                    )
                    return None

            with mock.patch.object(
                MODULE.subprocess, "Popen", return_value=ReadyProcess()
            ) as popen:
                supervisor.start()

            command = popen.call_args.args[0]
            self.assertEqual(
                command[:4],
                ["/locked/venv/bin/python", "-B", "-I", "-u"],
            )
            self.assertIn("--action-fd", command)
            self.assertIn("--action-session", command)
            self.assertEqual(len(popen.call_args.kwargs["pass_fds"]), 1)
            assert supervisor._action_socket is not None
            supervisor._action_socket.close()
            supervisor._action_socket = None
            supervisor.process = None

    def test_private_action_socket_drains_ordered_validated_intents(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            script = root / "matrix_calibration_overlay.py"
            script.write_text("", encoding="utf-8")
            supervisor = MODULE.CalibrationOverlaySupervisor(
                state_file=root / "state.json",
                display_name=None,
                expected_ue_pid=41,
                script=script,
            )
            receiver, sender = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            receiver.setblocking(False)
            supervisor._action_socket = receiver
            packets = (
                {
                    "version": 1,
                    "session": supervisor._action_session,
                    "sequence": 1,
                    "action": "profile_remote",
                },
                {
                    "version": 1,
                    "session": supervisor._action_session,
                    "sequence": 2,
                    "action": "speed_down",
                },
            )
            try:
                for packet in packets:
                    sender.send(json.dumps(packet).encode("ascii"))
                self.assertEqual(
                    supervisor.drain_actions(),
                    ("profile_remote", "speed_down"),
                )
                self.assertEqual(supervisor.drain_actions(), ())
            finally:
                sender.close()
                receiver.close()
                supervisor._action_socket = None

    def test_pointer_packet_drains_into_atomic_mouse_settings_save(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            script = root / "matrix_calibration_overlay.py"
            script.write_text("", encoding="utf-8")
            supervisor = MODULE.CalibrationOverlaySupervisor(
                state_file=root / "state.json",
                display_name=None,
                expected_ue_pid=41,
                script=script,
            )
            receiver, sender = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            receiver.setblocking(False)
            supervisor._action_socket = receiver
            settings_file = root / "config/mouse.json"
            controller = MODULE.MouseSettingsController(
                path=settings_file,
                desired=MODULE.MouseSettings(),
                load_status="missing",
                load_error=None,
            )
            packet = {
                "version": 1,
                "session": supervisor._action_session,
                "sequence": 1,
                "action": "profile_remote",
            }
            try:
                sender.send(json.dumps(packet).encode("ascii"))
                actions = supervisor.drain_actions()
                self.assertEqual(actions, ("profile_remote",))
                for action in actions:
                    self.assertTrue(
                        controller.apply_panel_action(action, active=True)
                    )
                self.assertEqual(
                    json.loads(settings_file.read_text(encoding="utf-8")),
                    {"profile": "remote", "speed_scale": 0.5, "version": 1},
                )
                self.assertEqual(settings_file.stat().st_mode & 0o777, 0o600)
            finally:
                sender.close()
                receiver.close()
                supervisor._action_socket = None

    def test_private_action_socket_rejects_wrong_session_and_direct_restart(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            script = root / "matrix_calibration_overlay.py"
            script.write_text("", encoding="utf-8")
            for packet in (
                {"version": 1, "session": "wrong", "sequence": 1, "action": "speed_up"},
                {
                    "version": 1,
                    "session": "placeholder",
                    "sequence": 1,
                    "action": "restart_directly",
                },
            ):
                supervisor = MODULE.CalibrationOverlaySupervisor(
                    state_file=root / "state.json",
                    display_name=None,
                    expected_ue_pid=41,
                    script=script,
                )
                if packet["session"] == "placeholder":
                    packet["session"] = supervisor._action_session
                receiver, sender = socket.socketpair(
                    socket.AF_UNIX, socket.SOCK_SEQPACKET
                )
                receiver.setblocking(False)
                supervisor._action_socket = receiver
                try:
                    sender.send(json.dumps(packet).encode("ascii"))
                    with self.assertRaisesRegex(RuntimeError, "identity"):
                        supervisor.drain_actions()
                finally:
                    sender.close()
                    receiver.close()
                    supervisor._action_socket = None


class SourceArbitrationTest(unittest.TestCase):
    def setUp(self) -> None:
        self.keyboard = MODULE.KeyboardMouseSample(
            w=True, q=True, v=True, ctrl=True, shift=True, focused=True
        )
        self.gamepad = MODULE.GamepadSample(
            forward=0.75, right=-0.25, look_yaw=0.5, connected=True
        )

    def test_auto_carries_both_and_core_owns_digital_priority(self) -> None:
        keys, stick, look = MODULE.select_physical_inputs(
            self.keyboard, self.gamepad, source="auto"
        )
        self.assertTrue(keys.w)
        self.assertTrue(keys.q)
        self.assertTrue(keys.ctrl)
        self.assertTrue(keys.shift)
        self.assertEqual((stick.right, stick.forward), (-0.25, 0.75))
        self.assertEqual(look, 0.5)

    def test_gamepad_requires_actual_camera_readback(self) -> None:
        self.assertEqual(
            MODULE.effective_input_source("auto", "fixed"), "keyboard"
        )
        self.assertEqual(
            MODULE.effective_input_source("auto", "x11-mirror"), "keyboard"
        )
        self.assertEqual(
            MODULE.effective_input_source("gamepad", "carla"), "gamepad"
        )
        with self.assertRaisesRegex(ValueError, "observed CARLA"):
            MODULE.effective_input_source("gamepad", "fixed")

    def test_gamepad_hotplug_edges_are_safety_interlocks(self) -> None:
        self.assertTrue(
            MODULE.gamepad_input_available(
                "auto", connected=False, previous_connected=None
            )
        )
        self.assertFalse(
            MODULE.gamepad_input_available(
                "auto", connected=True, previous_connected=False
            )
        )
        self.assertFalse(
            MODULE.gamepad_input_available(
                "auto", connected=False, previous_connected=True
            )
        )
        self.assertFalse(
            MODULE.gamepad_input_available(
                "gamepad", connected=False, previous_connected=False
            )
        )
        self.assertTrue(
            MODULE.gamepad_input_available(
                "gamepad", connected=True, previous_connected=True
            )
        )

    def test_explicit_sources_never_mix_locomotion_axes(self) -> None:
        keys, stick, look = MODULE.select_physical_inputs(
            self.keyboard, self.gamepad, source="keyboard"
        )
        self.assertTrue(keys.w)
        self.assertTrue(keys.q)
        self.assertTrue(keys.v)
        self.assertTrue(keys.ctrl)
        self.assertTrue(keys.shift)
        self.assertEqual((stick.right, stick.forward, look), (0.0, 0.0, 0.0))

        keys, stick, look = MODULE.select_physical_inputs(
            self.keyboard, self.gamepad, source="gamepad"
        )
        self.assertFalse(keys.w)
        self.assertTrue(keys.q)
        self.assertTrue(keys.v)
        self.assertFalse(keys.ctrl)
        self.assertFalse(keys.shift)
        self.assertEqual((stick.right, stick.forward), (-0.25, 0.75))
        self.assertEqual(look, 0.5)


class SnapshotTest(unittest.TestCase):
    def test_client_uses_the_core_protocol_encoder_without_schema_drift(self) -> None:
        snapshot = MODULE.build_snapshot(
            sequence=7,
            timestamp_monotonic_s=12.5,
            keyboard=MODULE.KeyboardMouseSample(w=True, focused=True),
            gamepad=MODULE.GamepadSample(),
            input_source="auto",
            camera_yaw_rad=math.pi / 2,
            camera_available=True,
        )
        payload = CORE.encode_input_packet(snapshot)
        self.assertEqual(CORE.decode_input_packet(payload), snapshot)
        self.assertEqual(snapshot.protocol, CORE.PROTOCOL_NAME)
        self.assertFalse(snapshot.keys.ctrl)
        self.assertFalse(snapshot.keys.shift)

    def test_missing_actual_camera_yaw_disables_operator(self) -> None:
        snapshot = MODULE.build_snapshot(
            sequence=1,
            timestamp_monotonic_s=1.0,
            keyboard=MODULE.KeyboardMouseSample(w=True, focused=True),
            gamepad=MODULE.GamepadSample(),
            input_source="auto",
            camera_yaw_rad=0.25,
            camera_available=False,
        )
        self.assertFalse(snapshot.focused)

    def test_native_camera_drag_interlocks_robot_movement(self) -> None:
        snapshot = MODULE.build_snapshot(
            sequence=2,
            timestamp_monotonic_s=1.0,
            keyboard=MODULE.KeyboardMouseSample(
                w=True, focused=True, camera_dragging=True
            ),
            gamepad=MODULE.GamepadSample(),
            input_source="keyboard",
            camera_yaw_rad=0.5,
            camera_available=True,
        )
        self.assertFalse(snapshot.focused)
        self.assertTrue(snapshot.keys.w)


class CalibrationModeTest(unittest.TestCase):
    @staticmethod
    def snapshot(
        sequence: int,
        timestamp: float,
        keyboard: MODULE.KeyboardMouseSample,
        gamepad: MODULE.GamepadSample | None = None,
    ):
        return MODULE.build_snapshot(
            sequence=sequence,
            timestamp_monotonic_s=timestamp,
            keyboard=keyboard,
            gamepad=gamepad or MODULE.GamepadSample(),
            input_source="keyboard",
            camera_yaw_rad=0.0,
            camera_available=True,
        )

    def test_active_mode_is_unfocused_and_fully_neutral(self) -> None:
        keyboard = MODULE.KeyboardMouseSample(
            w=True,
            a=True,
            s=True,
            d=True,
            q=True,
            e=True,
            v=True,
            ctrl=True,
            shift=True,
            escape=True,
            mouse_dx=12.0,
            mouse_dy=-4.0,
            camera_dragging=True,
            focused=True,
            focus_title="Matrix",
            focus_pid=1234,
        )
        gamepad = MODULE.GamepadSample(
            forward=0.8,
            right=-0.4,
            look_yaw=0.6,
            look_pitch=-0.2,
            connected=True,
        )

        neutral_keyboard, neutral_pad = MODULE.apply_calibration_interlock(
            keyboard, gamepad, active=True
        )
        snapshot = self.snapshot(1, 10.0, neutral_keyboard, neutral_pad)

        self.assertFalse(snapshot.focused)
        key_levels = snapshot.keys.to_mapping()
        self.assertTrue(key_levels.pop("v"))
        self.assertFalse(any(key_levels.values()))
        self.assertEqual(
            (snapshot.move_stick.right, snapshot.move_stick.forward), (0.0, 0.0)
        )
        self.assertEqual((neutral_keyboard.mouse_dx, neutral_keyboard.mouse_dy), (0.0, 0.0))
        self.assertFalse(neutral_keyboard.camera_dragging)
        self.assertFalse(neutral_keyboard.escape)
        self.assertEqual(neutral_keyboard.focus_title, "Matrix")
        self.assertEqual(neutral_keyboard.focus_pid, 1234)
        self.assertFalse(neutral_pad.connected)

    def test_held_v_across_calibration_does_not_create_a_mode_edge(self) -> None:
        core = CORE.GameControlCore()

        # V is already held on entry.  Every calibration packet is unfocused,
        # but carries the real V level so the core keeps its edge memory true.
        for sequence, timestamp in ((1, 10.0), (2, 10.01), (3, 10.02)):
            keyboard, pad = MODULE.apply_calibration_interlock(
                MODULE.KeyboardMouseSample(v=True, focused=True),
                MODULE.GamepadSample(),
                active=True,
            )
            core.accept_snapshot(
                self.snapshot(sequence, timestamp, keyboard, pad),
                received_at_s=timestamp,
            )
            self.assertFalse(core.free_camera)

        # Exiting while V remains held must not look like a new focused edge.
        core.accept_snapshot(
            self.snapshot(
                4,
                10.03,
                MODULE.KeyboardMouseSample(v=True, focused=True),
            ),
            received_at_s=10.03,
        )
        self.assertFalse(core.free_camera)

        # A real release followed by a new press still toggles exactly once.
        core.accept_snapshot(
            self.snapshot(5, 10.04, MODULE.KeyboardMouseSample(focused=True)),
            received_at_s=10.04,
        )
        core.accept_snapshot(
            self.snapshot(
                6,
                10.05,
                MODULE.KeyboardMouseSample(v=True, focused=True),
            ),
            received_at_s=10.05,
        )
        self.assertTrue(core.free_camera)
        core.accept_snapshot(
            self.snapshot(
                7,
                10.06,
                MODULE.KeyboardMouseSample(v=True, focused=True),
            ),
            received_at_s=10.06,
        )
        self.assertTrue(core.free_camera)

    def test_second_escape_exits_after_ue_releases_focus_and_w_must_rearm(self) -> None:
        controller = MODULE.CalibrationModeController()
        core = CORE.GameControlCore()

        entered = controller.update(escape_pressed=True, ue_focused=True)
        self.assertTrue(entered)
        self.assertTrue(controller.active)
        keyboard, pad = MODULE.apply_calibration_interlock(
            MODULE.KeyboardMouseSample(w=True, escape=True, focused=True),
            MODULE.GamepadSample(),
            active=controller.active,
        )
        core.accept_snapshot(self.snapshot(1, 10.0, keyboard, pad), received_at_s=10.0)
        self.assertEqual(core.command(now_s=10.0, dt_s=0.01).reason, "focus_lost")

        # Releasing Escape does not toggle.  The cooked UE is allowed to drop
        # capture/focus while the click-through overlay remains active.
        self.assertFalse(controller.update(escape_pressed=False, ue_focused=False))
        self.assertTrue(controller.active)
        self.assertTrue(controller.update(escape_pressed=True, ue_focused=False))
        self.assertFalse(controller.active)

        # A held pre-calibration W cannot become motion on focus recovery.
        core.accept_snapshot(
            self.snapshot(
                2,
                10.01,
                MODULE.KeyboardMouseSample(w=True, focused=False),
            ),
            received_at_s=10.01,
        )
        core.accept_snapshot(
            self.snapshot(
                3,
                10.02,
                MODULE.KeyboardMouseSample(w=True, focused=True),
            ),
            received_at_s=10.02,
        )
        self.assertEqual(
            core.command(now_s=10.02, dt_s=0.01).reason, "awaiting_neutral"
        )
        core.accept_snapshot(
            self.snapshot(4, 10.03, MODULE.KeyboardMouseSample(focused=True)),
            received_at_s=10.03,
        )
        self.assertEqual(core.command(now_s=10.03, dt_s=0.01).mode, "idle")
        core.accept_snapshot(
            self.snapshot(
                5,
                10.04,
                MODULE.KeyboardMouseSample(w=True, focused=True),
            ),
            received_at_s=10.04,
        )
        self.assertEqual(core.command(now_s=10.04, dt_s=0.1).mode, "move")

    def test_escape_from_another_application_cannot_enter_calibration(self) -> None:
        controller = MODULE.CalibrationModeController()
        self.assertFalse(controller.update(escape_pressed=True, ue_focused=False))
        self.assertFalse(controller.active)

    def test_ui_and_escape_exit_frames_drop_release_delta_and_require_rearm(self) -> None:
        for exit_kind in ("ui", "escape"):
            with self.subTest(exit_kind=exit_kind):
                calibration = MODULE.CalibrationModeController()
                calibration.active = True
                if exit_kind == "ui":
                    self.assertTrue(calibration.exit())
                else:
                    self.assertTrue(
                        calibration.update(escape_pressed=True, ue_focused=False)
                    )
                self.assertFalse(calibration.active)
                self.assertTrue(
                    MODULE.calibration_interlock_required(
                        panel_was_active=True,
                        panel_active=calibration.active,
                    )
                )
                release_sample = MODULE.KeyboardMouseSample(
                    w=True,
                    mouse_dx=73.0,
                    mouse_dy=-11.0,
                    camera_dragging=False,
                    focused=True,
                )
                keyboard, pad = MODULE.apply_calibration_interlock(
                    release_sample,
                    MODULE.GamepadSample(),
                    active=True,
                )
                self.assertFalse(keyboard.focused)
                self.assertEqual((keyboard.mouse_dx, keyboard.mouse_dy), (0.0, 0.0))
                tracker = MODULE.CameraYawTracker(
                    0.0,
                    mouse_radians_per_pixel=0.1,
                    gamepad_radians_per_second=0.0,
                )
                self.assertEqual(
                    tracker.update(
                        dt=0.02,
                        mouse_dx=keyboard.mouse_dx,
                        gamepad_look_yaw=0.0,
                    ),
                    0.0,
                )

                core = CORE.GameControlCore()
                core.accept_snapshot(
                    self.snapshot(
                        1,
                        10.0,
                        MODULE.KeyboardMouseSample(focused=True),
                    ),
                    received_at_s=10.0,
                )
                core.accept_snapshot(
                    self.snapshot(
                        2,
                        10.01,
                        MODULE.KeyboardMouseSample(w=True, focused=True),
                    ),
                    received_at_s=10.01,
                )
                self.assertEqual(core.command(now_s=10.01, dt_s=0.1).mode, "move")
                core.accept_snapshot(
                    self.snapshot(3, 10.02, keyboard, pad),
                    received_at_s=10.02,
                )
                self.assertEqual(
                    core.command(now_s=10.02, dt_s=0.01).reason,
                    "focus_lost",
                )
                # The next frame resumes physical sampling, but held W remains
                # stopped until a focused neutral frame re-arms the core.
                self.assertFalse(
                    MODULE.calibration_interlock_required(
                        panel_was_active=False,
                        panel_active=False,
                    )
                )
                core.accept_snapshot(
                    self.snapshot(4, 10.03, release_sample),
                    received_at_s=10.03,
                )
                self.assertEqual(
                    core.command(now_s=10.03, dt_s=0.01).reason,
                    "awaiting_neutral",
                )


class MouseSettingsAndRestartTest(unittest.TestCase):
    def test_applied_remote_scale_is_discrete_but_local_remains_one_x(self) -> None:
        remote = MODULE.AppliedMouseSettings(
            profile="remote", effective_scale=0.01
        )
        self.assertEqual(remote.effective_scale, 0.01)
        local = MODULE.AppliedMouseSettings(profile="local", effective_scale=1.0)
        self.assertEqual(local.effective_scale, 1.0)

        for value in (0.0, 0.11, 0.15, 1.01, True, float("nan")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                MODULE.AppliedMouseSettings(
                    profile="remote", effective_scale=value
                )

        with self.assertRaises(ValueError):
            MODULE.AppliedMouseSettings(profile="local", effective_scale=0.40)

    def test_startup_requires_escape_and_f9_release_before_arming(self) -> None:
        arming = MODULE.StartupShortcutArming()
        self.assertFalse(arming.update(escape_pressed=True, restart_pressed=True))
        self.assertFalse(arming.update(escape_pressed=False, restart_pressed=True))
        self.assertTrue(arming.update(escape_pressed=False, restart_pressed=False))
        self.assertTrue(arming.update(escape_pressed=True, restart_pressed=False))

    def test_settings_edges_persist_only_next_launch_state(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "config/mouse.json"
            controller = MODULE.MouseSettingsController(
                path=path,
                desired=MODULE.MouseSettings(),
                load_status="missing",
                load_error=None,
            )
            applied = MODULE.AppliedMouseSettings(
                profile="local", effective_scale=1.0
            )
            self.assertFalse(
                controller.update(
                    active=False,
                    mode_pressed=True,
                    slower_pressed=False,
                    faster_pressed=False,
                )
            )
            controller.update(
                active=True,
                mode_pressed=False,
                slower_pressed=False,
                faster_pressed=False,
            )
            self.assertTrue(
                controller.update(
                    active=True,
                    mode_pressed=True,
                    slower_pressed=False,
                    faster_pressed=False,
                )
            )
            self.assertTrue(controller.pending_restart(applied))
            self.assertEqual(controller.desired.profile, "remote")
            self.assertEqual(
                json.loads(path.read_text(encoding="utf-8")),
                {"profile": "remote", "speed_scale": 0.5, "version": 1},
            )

            controller.update(
                active=True,
                mode_pressed=False,
                slower_pressed=False,
                faster_pressed=False,
            )
            self.assertTrue(
                controller.update(
                    active=True,
                    mode_pressed=False,
                    slower_pressed=True,
                    faster_pressed=False,
                )
            )
            self.assertEqual(controller.desired.effective_scale, 0.4)
            # The current generation remains exactly the launch snapshot.
            self.assertEqual(applied.effective_scale, 1.0)

    def test_panel_selects_profiles_and_speed_without_key_emulation(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "config/mouse.json"
            controller = MODULE.MouseSettingsController(
                path=path,
                desired=MODULE.MouseSettings(),
                load_status="missing",
                load_error=None,
            )
            self.assertFalse(
                controller.apply_panel_action("profile_remote", active=False)
            )
            self.assertTrue(
                controller.apply_panel_action("profile_remote", active=True)
            )
            self.assertEqual(controller.desired.profile, "remote")
            self.assertTrue(controller.apply_panel_action("speed_up", active=True))
            self.assertEqual(controller.desired.speed_scale, 0.6)
            self.assertTrue(
                controller.apply_panel_action("profile_local", active=True)
            )
            self.assertEqual(controller.desired.profile, "local")
            self.assertFalse(controller.apply_panel_action("speed_down", active=True))

    def test_keyboard_and_panel_steps_traverse_the_same_discrete_table(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            initial = MODULE.MouseSettings(profile="remote", speed_scale=0.01)
            keyboard = MODULE.MouseSettingsController(
                path=root / "keyboard.json",
                desired=initial,
                load_status="loaded",
                load_error=None,
            )
            panel = MODULE.MouseSettingsController(
                path=root / "panel.json",
                desired=initial,
                load_status="loaded",
                load_error=None,
            )

            def keyboard_step(*, slower: bool = False, faster: bool = False) -> bool:
                keyboard.update(
                    active=True,
                    mode_pressed=False,
                    slower_pressed=False,
                    faster_pressed=False,
                )
                return keyboard.update(
                    active=True,
                    mode_pressed=False,
                    slower_pressed=slower,
                    faster_pressed=faster,
                )

            expected = tuple(value / 100 for value in range(1, 11)) + tuple(
                value / 10 for value in range(2, 11)
            )
            self.assertEqual(keyboard.desired.speed_scale, expected[0])
            self.assertEqual(panel.desired.speed_scale, expected[0])
            for scale in expected[1:]:
                self.assertTrue(keyboard_step(faster=True))
                self.assertTrue(panel.apply_panel_action("speed_up", active=True))
                self.assertEqual(keyboard.desired.speed_scale, scale)
                self.assertEqual(panel.desired.speed_scale, scale)
            self.assertFalse(keyboard_step(faster=True))
            self.assertFalse(panel.apply_panel_action("speed_up", active=True))

            for scale in reversed(expected[:-1]):
                self.assertTrue(keyboard_step(slower=True))
                self.assertTrue(panel.apply_panel_action("speed_down", active=True))
                self.assertEqual(keyboard.desired.speed_scale, scale)
                self.assertEqual(panel.desired.speed_scale, scale)
            self.assertFalse(keyboard_step(slower=True))
            self.assertFalse(panel.apply_panel_action("speed_down", active=True))

    @staticmethod
    def requester(*, available: bool = True, succeeds: bool = True):
        class Requester:
            def __init__(self) -> None:
                self.available = available
                self.requested = False
                self.error = None
                self.calls = 0

            def request(self) -> bool:
                self.calls += 1
                if not self.available or not succeeds:
                    self.error = "injected restart failure"
                    return False
                self.requested = True
                self.available = False
                return True

        return Requester()

    def test_apply_return_waits_for_neutral_then_returns_without_reload(self) -> None:
        calibration = MODULE.CalibrationModeController()
        calibration.active = True
        controller = MODULE.ApplyReturnController()
        requester = self.requester()
        self.assertEqual(
            controller.update(
                enter_pressed=False,
                clicked=True,
                ue_focused=True,
                panel_was_active=True,
                calibration=calibration,
                neutral_frame_ready=False,
                pending_restart=False,
                persistence_error=None,
                requester=requester,
            ),
            (False, False),
        )
        self.assertTrue(controller.pending_intent)
        self.assertEqual(controller.status, "waiting_neutral")
        self.assertEqual(
            controller.update(
                enter_pressed=False,
                clicked=False,
                ue_focused=True,
                panel_was_active=True,
                calibration=calibration,
                neutral_frame_ready=True,
                pending_restart=False,
                persistence_error=None,
                requester=requester,
            ),
            (True, False),
        )
        self.assertFalse(calibration.active)
        self.assertEqual(requester.calls, 0)

    def test_apply_return_reuses_restart_requester_once_after_neutral(self) -> None:
        calibration = MODULE.CalibrationModeController()
        calibration.active = True
        controller = MODULE.ApplyReturnController()
        requester = self.requester()
        controller.update(
            enter_pressed=False,
            clicked=False,
            ue_focused=True,
            panel_was_active=True,
            calibration=calibration,
            neutral_frame_ready=True,
            pending_restart=True,
            persistence_error=None,
            requester=requester,
        )
        result = controller.update(
            enter_pressed=True,
            clicked=False,
            ue_focused=True,
            panel_was_active=True,
            calibration=calibration,
            neutral_frame_ready=True,
            pending_restart=True,
            persistence_error=None,
            requester=requester,
        )
        self.assertEqual(result, (False, True))
        self.assertTrue(calibration.active)
        self.assertEqual(controller.status, "restarting")
        self.assertEqual(requester.calls, 1)
        controller.update(
            enter_pressed=False,
            clicked=True,
            ue_focused=True,
            panel_was_active=True,
            calibration=calibration,
            neutral_frame_ready=True,
            pending_restart=True,
            persistence_error=None,
            requester=requester,
        )
        self.assertEqual(requester.calls, 1)

    def test_enter_requires_focused_release_after_each_panel_entry(self) -> None:
        calibration = MODULE.CalibrationModeController()
        calibration.active = True
        controller = MODULE.ApplyReturnController()
        requester = self.requester()

        def update(*, pressed: bool, focused: bool, was_active: bool):
            return controller.update(
                enter_pressed=pressed,
                clicked=False,
                ue_focused=focused,
                panel_was_active=was_active,
                calibration=calibration,
                neutral_frame_ready=True,
                pending_restart=True,
                persistence_error=None,
                requester=requester,
            )

        # ESC+Enter in the entry frame, and the following held frame, are not
        # a fresh panel key press.
        self.assertEqual(
            update(pressed=True, focused=True, was_active=False),
            (False, False),
        )
        self.assertEqual(
            update(pressed=True, focused=True, was_active=True),
            (False, False),
        )
        self.assertEqual(requester.calls, 0)
        update(pressed=False, focused=True, was_active=True)
        self.assertEqual(
            update(pressed=True, focused=True, was_active=True),
            (False, True),
        )
        self.assertEqual(requester.calls, 1)

    def test_terminal_and_cross_focus_held_enter_cannot_apply(self) -> None:
        calibration = MODULE.CalibrationModeController()
        calibration.active = True
        controller = MODULE.ApplyReturnController()
        requester = self.requester()

        def update(*, pressed: bool, focused: bool):
            return controller.update(
                enter_pressed=pressed,
                clicked=False,
                ue_focused=focused,
                panel_was_active=True,
                calibration=calibration,
                neutral_frame_ready=True,
                pending_restart=True,
                persistence_error=None,
                requester=requester,
            )

        update(pressed=False, focused=True)
        update(pressed=True, focused=False)  # Enter typed in a terminal.
        update(pressed=True, focused=True)  # Still held after Alt-Tab back.
        self.assertEqual(requester.calls, 0)
        update(pressed=False, focused=True)
        self.assertEqual(update(pressed=True, focused=True), (False, True))
        self.assertEqual(requester.calls, 1)

    def test_apply_failure_stays_in_safe_panel_and_is_visible(self) -> None:
        calibration = MODULE.CalibrationModeController()
        calibration.active = True
        controller = MODULE.ApplyReturnController()
        requester = self.requester()
        controller.update(
            enter_pressed=False,
            clicked=False,
            ue_focused=True,
            panel_was_active=True,
            calibration=calibration,
            neutral_frame_ready=True,
            pending_restart=True,
            persistence_error="read-only filesystem",
            requester=requester,
        )
        result = controller.update(
            enter_pressed=True,
            clicked=False,
            ue_focused=True,
            panel_was_active=True,
            calibration=calibration,
            neutral_frame_ready=True,
            pending_restart=True,
            persistence_error="read-only filesystem",
            requester=requester,
        )
        self.assertEqual(result, (False, False))
        self.assertTrue(calibration.active)
        self.assertEqual(controller.status, "error")
        self.assertIn("read-only filesystem", controller.error)
        self.assertEqual(requester.calls, 0)

        controller = MODULE.ApplyReturnController()
        requester = self.requester(succeeds=False)
        result = controller.update(
            enter_pressed=False,
            clicked=True,
            ue_focused=True,
            panel_was_active=True,
            calibration=calibration,
            neutral_frame_ready=True,
            pending_restart=True,
            persistence_error=None,
            requester=requester,
        )
        self.assertEqual(result, (False, False))
        self.assertTrue(calibration.active)
        self.assertEqual(controller.status, "error")
        self.assertEqual(controller.error, "injected restart failure")
        self.assertEqual(requester.calls, 1)

    def test_f9_requires_active_pending_saved_and_prior_neutral_send(self) -> None:
        class Requester:
            available = True

            def __init__(self) -> None:
                self.calls = 0

            def request(self) -> bool:
                self.calls += 1
                return True

        requester = Requester()
        key = MODULE.ApplyRestartKey()
        self.assertFalse(
            key.update(
                pressed=True,
                calibration_active=True,
                neutral_frame_ready=False,
                pending_restart=True,
                persistence_ok=True,
                requester=requester,
            )
        )
        # Holding F9 after the neutral frame cannot turn the rejected edge into
        # a restart; the operator must release and make a fresh press.
        self.assertFalse(
            key.update(
                pressed=True,
                calibration_active=True,
                neutral_frame_ready=True,
                pending_restart=True,
                persistence_ok=True,
                requester=requester,
            )
        )
        key.update(
            pressed=False,
            calibration_active=True,
            neutral_frame_ready=True,
            pending_restart=True,
            persistence_ok=True,
            requester=requester,
        )
        self.assertTrue(
            key.update(
                pressed=True,
                calibration_active=True,
                neutral_frame_ready=True,
                pending_restart=True,
                persistence_ok=True,
                requester=requester,
            )
        )
        self.assertEqual(requester.calls, 1)

    def test_requester_writes_once_without_signalling_or_exiting(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            root.chmod(0o700)
            capability = root / "capability"
            request_file = root / "request.json"
            RESTART.atomic_write_capability(capability)
            requester = MODULE.RuntimeRestartRequester(
                request_file=request_file,
                capability_file=capability,
                launcher_pid=os.getpid(),
            )
            self.assertTrue(requester.available)
            self.assertTrue(requester.request())
            self.assertTrue(request_file.is_file())
            self.assertFalse(requester.available)
            self.assertFalse(requester.request())

    def test_panel_apply_neutral_gate_writes_real_private_restart_request(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            root.chmod(0o700)
            capability = root / "capability"
            request_file = root / "request.json"
            RESTART.atomic_write_capability(capability)
            requester = MODULE.RuntimeRestartRequester(
                request_file=request_file,
                capability_file=capability,
                launcher_pid=os.getpid(),
            )
            calibration = MODULE.CalibrationModeController()
            calibration.active = True
            controller = MODULE.ApplyReturnController()
            self.assertEqual(
                controller.update(
                    enter_pressed=False,
                    clicked=True,
                    ue_focused=True,
                    panel_was_active=True,
                    calibration=calibration,
                    neutral_frame_ready=False,
                    pending_restart=True,
                    persistence_error=None,
                    requester=requester,
                ),
                (False, False),
            )
            self.assertFalse(request_file.exists())
            self.assertEqual(
                controller.update(
                    enter_pressed=False,
                    clicked=False,
                    ue_focused=True,
                    panel_was_active=True,
                    calibration=calibration,
                    neutral_frame_ready=True,
                    pending_restart=True,
                    persistence_error=None,
                    requester=requester,
                ),
                (False, True),
            )
            request = json.loads(request_file.read_text(encoding="utf-8"))
            self.assertEqual(request["action"], "restart-whole-runtime")
            self.assertEqual(request["launcher_pid"], os.getpid())
            self.assertEqual(request["provider_pid"], os.getpid())
            self.assertTrue(calibration.active)


class CameraYawTrackerTest(unittest.TestCase):
    def test_applied_sdl_scale_also_scales_x11_mirror_gain(self) -> None:
        base_deg_per_px = 0.12
        applied_scale = 0.01
        tracker = MODULE.CameraYawTracker(
            0.0,
            mouse_radians_per_pixel=math.radians(
                base_deg_per_px * applied_scale
            ),
            gamepad_radians_per_second=0.0,
        )
        yaw = tracker.update(dt=0.02, mouse_dx=100.0, gamepad_look_yaw=0.0)
        self.assertAlmostEqual(math.degrees(yaw), 0.12)

    def test_mouse_has_per_frame_priority_over_right_stick(self) -> None:
        tracker = MODULE.CameraYawTracker(
            0.0,
            mouse_radians_per_pixel=0.1,
            gamepad_radians_per_second=2.0,
        )
        yaw = tracker.update(dt=0.5, mouse_dx=2.0, gamepad_look_yaw=1.0)
        self.assertAlmostEqual(yaw, 0.2)
        yaw = tracker.update(dt=0.5, mouse_dx=0.0, gamepad_look_yaw=1.0)
        self.assertAlmostEqual(yaw, 1.2)

    def test_observed_yaw_is_absolute_and_wrapped(self) -> None:
        tracker = MODULE.CameraYawTracker(
            0.0,
            mouse_radians_per_pixel=0.1,
            gamepad_radians_per_second=2.0,
        )
        yaw = tracker.update(
            dt=1.0,
            mouse_dx=100.0,
            gamepad_look_yaw=1.0,
            observed_yaw_rad=3.0 * math.pi,
        )
        self.assertAlmostEqual(abs(yaw), math.pi)

    def test_provider_sign_and_offset_convert_to_sonic_frame(self) -> None:
        self.assertAlmostEqual(
            MODULE.transform_camera_yaw(
                math.radians(30.0),
                sign=-1,
                offset_rad=math.radians(90.0),
            ),
            math.radians(60.0),
        )
        with self.assertRaisesRegex(ValueError, "sign"):
            MODULE.transform_camera_yaw(0.0, sign=0, offset_rad=0.0)


class CarlaSpectatorCameraTest(unittest.TestCase):
    class Rotation:
        def __init__(self, *, yaw: float = 0.0, pitch: float = 0.0) -> None:
            self.yaw = yaw
            self.pitch = pitch

    class Transform:
        def __init__(self, *, yaw: float = 0.0, pitch: float = 0.0) -> None:
            self.rotation = CarlaSpectatorCameraTest.Rotation(
                yaw=yaw, pitch=pitch
            )

    class Spectator:
        def __init__(self, transform) -> None:
            self.transform = transform
            self.set_calls = 0
            self.fail_writes = False
            self.ignore_writes = False

        def get_transform(self):
            return CarlaSpectatorCameraTest.Transform(
                yaw=self.transform.rotation.yaw,
                pitch=self.transform.rotation.pitch,
            )

        def set_transform(self, transform) -> None:
            if self.fail_writes:
                raise RuntimeError("write failed")
            self.set_calls += 1
            if self.ignore_writes:
                return
            self.transform = CarlaSpectatorCameraTest.Transform(
                yaw=transform.rotation.yaw,
                pitch=transform.rotation.pitch,
            )

    class World:
        def __init__(self, spectator) -> None:
            self.spectator = spectator

        def get_spectator(self):
            return self.spectator

    def reader(self, spectator, **overrides):
        parameters = {
            "look_yaw_rate_rad_s": math.radians(90.0),
            "look_pitch_rate_rad_s": math.radians(60.0),
            "look_deadzone": 0.0,
            "minimum_pitch_rad": math.radians(-45.0),
            "maximum_pitch_rad": math.radians(30.0),
        }
        parameters.update(overrides)
        reader = MODULE.CarlaSpectatorYawReader("127.0.0.1", 2000, **parameters)
        reader._world = self.World(spectator)
        return reader

    def test_right_stick_writes_spectator_then_returns_absolute_readback(self) -> None:
        spectator = self.Spectator(self.Transform(yaw=10.0, pitch=0.0))
        reader = self.reader(spectator)

        yaw = reader.drive(
            now=1.0, dt=0.5, look_yaw=0.5, look_pitch=0.5
        )

        self.assertEqual(spectator.set_calls, 1)
        self.assertAlmostEqual(spectator.transform.rotation.yaw, 32.5)
        self.assertAlmostEqual(spectator.transform.rotation.pitch, 15.0)
        self.assertAlmostEqual(yaw, math.radians(32.5))

    def test_pitch_is_clamped_and_zero_stick_is_read_only(self) -> None:
        spectator = self.Spectator(self.Transform(yaw=-30.0, pitch=25.0))
        reader = self.reader(spectator)

        driven = reader.drive(
            now=1.0, dt=1.0, look_yaw=0.0, look_pitch=1.0
        )
        polled = reader.drive(
            now=1.1, dt=1.0, look_yaw=0.0, look_pitch=0.0
        )

        self.assertEqual(spectator.set_calls, 1)
        self.assertAlmostEqual(spectator.transform.rotation.pitch, 30.0)
        self.assertAlmostEqual(driven, math.radians(-30.0))
        self.assertAlmostEqual(polled, math.radians(-30.0))

    def test_failed_camera_write_drops_readback_and_disconnects(self) -> None:
        spectator = self.Spectator(self.Transform(yaw=0.0, pitch=0.0))
        spectator.fail_writes = True
        reader = self.reader(spectator)

        yaw = reader.drive(
            now=2.0, dt=0.1, look_yaw=1.0, look_pitch=0.0
        )

        self.assertIsNone(yaw)
        self.assertIsNone(reader._world)
        self.assertEqual(reader._next_connect, 3.0)

    def test_ignored_camera_write_drops_readback_and_disconnects(self) -> None:
        spectator = self.Spectator(self.Transform(yaw=0.0, pitch=0.0))
        spectator.ignore_writes = True
        reader = self.reader(spectator)

        yaw = reader.drive(
            now=2.0, dt=0.1, look_yaw=1.0, look_pitch=0.0
        )

        self.assertIsNone(yaw)
        self.assertEqual(spectator.set_calls, 1)
        self.assertIsNone(reader._world)

    def test_invalid_camera_tuning_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "deadzone"):
            MODULE.CarlaSpectatorYawReader(
                "127.0.0.1", 2000, look_deadzone=1.0
            )
        with self.assertRaisesRegex(ValueError, "pitch limits"):
            MODULE.CarlaSpectatorYawReader(
                "127.0.0.1",
                2000,
                minimum_pitch_rad=1.0,
                maximum_pitch_rad=0.0,
            )


class X11KeyboardMouseSafetyTest(unittest.TestCase):
    def test_teleport_is_rejected_and_rebaselined_without_clamping(self) -> None:
        samples = iter(
            (
                (0, 1 << 8),
                (200, 1 << 8),
                (401, 1 << 8),
                (406, 1 << 8),
            )
        )

        class FakeX11:
            @staticmethod
            def XQueryKeymap(_display, _buffer) -> int:
                return 1

            @staticmethod
            def XQueryPointer(*args) -> int:
                x, button_mask = next(samples)
                args[4]._obj.value = x
                args[5]._obj.value = 0
                args[8]._obj.value = button_mask
                return 1

        backend = object.__new__(MODULE.X11KeyboardMouse)
        backend._x11 = FakeX11()
        backend._display = 1
        backend._root = 2
        backend._keycodes = {
            name: index
            for index, name in enumerate(MODULE.X11KeyboardMouse._KEYSYMS, start=8)
        }
        backend._pressed = lambda _keymap, _code: False
        backend._focus_pattern = None
        backend._look_mask = 1 << 8
        backend._previous_pointer = None
        backend._previous_look_pressed = False
        backend._maximum_mouse_delta = 200.0
        backend._teleport_rejections = 0
        backend._last_teleport_delta = None
        backend._expected_ue_pid = 1234
        backend._focus_identity = lambda: (True, "Matrix", frozenset({1234}))

        baseline = backend.poll()
        exact_boundary = backend.poll()
        teleport = backend.poll()
        after_rebaseline = backend.poll()

        self.assertEqual(baseline.mouse_dx, 0.0)
        self.assertEqual(exact_boundary.mouse_dx, 200.0)
        self.assertEqual(teleport.mouse_dx, 0.0)
        self.assertEqual(after_rebaseline.mouse_dx, 5.0)
        self.assertEqual(
            backend.pointer_telemetry,
            {
                "teleport_rejections": 1,
                "last_teleport_delta": [201, 0],
                "maximum_mouse_delta_px": 200.0,
            },
        )

    def test_release_sample_keeps_final_held_drag_delta(self) -> None:
        samples = iter(((0, 1 << 8), (10, 1 << 8), (30, 0)))

        class FakeX11:
            @staticmethod
            def XQueryKeymap(_display, _buffer) -> int:
                return 1

            @staticmethod
            def XQueryPointer(*args) -> int:
                x, button_mask = next(samples)
                args[4]._obj.value = x
                args[5]._obj.value = 0
                args[8]._obj.value = button_mask
                return 1

        backend = object.__new__(MODULE.X11KeyboardMouse)
        backend._x11 = FakeX11()
        backend._display = 1
        backend._root = 2
        backend._keycodes = {
            name: index
            for index, name in enumerate(MODULE.X11KeyboardMouse._KEYSYMS, start=8)
        }
        backend._pressed = lambda _keymap, _code: False
        backend._focus_pattern = None
        backend._look_mask = 1 << 8
        backend._previous_pointer = None
        backend._previous_look_pressed = False
        backend._maximum_mouse_delta = 200.0
        backend._expected_ue_pid = 1234
        backend._focus_identity = lambda: (True, "Matrix", frozenset({1234}))

        pressed = backend.poll()
        held = backend.poll()
        released = backend.poll()

        self.assertEqual(pressed.mouse_dx, 0.0)
        self.assertEqual(held.mouse_dx, 10.0)
        self.assertEqual(released.mouse_dx, 20.0)
        self.assertTrue(pressed.camera_dragging)
        self.assertTrue(held.camera_dragging)
        self.assertFalse(released.camera_dragging)

    def test_left_or_right_modifier_keys_are_collapsed(self) -> None:
        class FakeX11:
            @staticmethod
            def XQueryKeymap(_display, _buffer) -> int:
                return 1

            @staticmethod
            def XQueryPointer(*_args) -> int:
                return 1

        backend = object.__new__(MODULE.X11KeyboardMouse)
        backend._x11 = FakeX11()
        backend._display = 1
        backend._root = 2
        backend._keycodes = {
            name: index
            for index, name in enumerate(MODULE.X11KeyboardMouse._KEYSYMS, start=8)
        }
        pressed_codes = {
            backend._keycodes["ctrl_left"],
            backend._keycodes["shift_right"],
        }
        backend._pressed = lambda _keymap, code: code in pressed_codes
        backend._focus_pattern = None
        backend._look_mask = 1 << 8
        backend._previous_pointer = None
        backend._previous_look_pressed = False
        backend._maximum_mouse_delta = 200.0
        backend._expected_ue_pid = 1234
        backend._focus_identity = lambda: (True, "Matrix", frozenset({1234}))

        sample = backend.poll()

        self.assertTrue(sample.ctrl)
        self.assertTrue(sample.shift)
        self.assertTrue(sample.focused)

    def test_pointer_query_failure_disarms_even_when_matrix_has_focus(self) -> None:
        class FakeX11:
            @staticmethod
            def XQueryKeymap(_display, _buffer) -> int:
                return 1

            @staticmethod
            def XQueryPointer(*_args) -> int:
                return 0

        backend = object.__new__(MODULE.X11KeyboardMouse)
        backend._x11 = FakeX11()
        backend._display = 1
        backend._root = 2
        backend._keycodes = {
            name: 8 for name in ("w", "a", "s", "d", "q", "e", "v")
        }
        backend._focus_pattern = None
        backend._look_mask = 1 << 8
        backend._previous_pointer = None
        backend._previous_look_pressed = False
        backend._maximum_mouse_delta = 200.0
        backend._expected_ue_pid = 1234
        backend._focus_identity = lambda: (True, "Matrix", frozenset({1234}))

        sample = backend.poll()

        self.assertEqual(sample.focus_title, "Matrix")
        self.assertFalse(sample.focused)
        self.assertFalse(sample.camera_dragging)

    def test_matching_title_with_wrong_pid_is_not_matrix_focus(self) -> None:
        class FakeX11:
            @staticmethod
            def XQueryKeymap(_display, _buffer) -> int:
                return 1

            @staticmethod
            def XQueryPointer(*_args) -> int:
                return 1

        backend = object.__new__(MODULE.X11KeyboardMouse)
        backend._x11 = FakeX11()
        backend._display = 1
        backend._root = 2
        backend._keycodes = {
            name: 8 for name in ("w", "a", "s", "d", "q", "e", "v")
        }
        backend._focus_pattern = None
        backend._look_mask = 1 << 8
        backend._previous_pointer = None
        backend._previous_look_pressed = False
        backend._maximum_mouse_delta = 200.0
        backend._expected_ue_pid = 1234
        backend._focus_identity = lambda: (
            True,
            "matrix-game-control terminal",
            frozenset({5678}),
        )

        sample = backend.poll()

        self.assertEqual(sample.focus_pid, 5678)
        self.assertFalse(sample.focused)

        backend._focus_identity = lambda: (
            True,
            "Matrix",
            frozenset({1234, 5678}),
        )
        sample = backend.poll()
        self.assertEqual(sample.focus_pid, 1234)
        self.assertTrue(sample.focused)

        # --allow-any-focus disables the title regex, not the UE PID binding.
        backend._focus_identity = lambda: (True, None, frozenset({1234}))
        sample = backend.poll()
        self.assertIsNone(sample.focus_title)
        self.assertTrue(sample.focused)


class FrameWaitTest(unittest.TestCase):
    def test_shutdown_during_sleep_prevents_an_extra_sample(self) -> None:
        running = [True]

        def interrupting_sleep(_seconds: float) -> None:
            running[0] = False

        should_sample, now = MODULE._wait_until_frame(
            1.0,
            2.0,
            keep_running=lambda: running[0],
            sleeper=interrupting_sleep,
            clock=lambda: 1.5,
        )

        self.assertFalse(should_sample)
        self.assertEqual(now, 1.5)


class SequenceTest(unittest.TestCase):
    def test_restart_uses_later_host_monotonic_nanoseconds(self) -> None:
        first_client = MODULE.initial_sequence(lambda: 1_000_000_000)
        second_client = MODULE.initial_sequence(lambda: 1_001_000_000)
        self.assertGreater(second_client, first_client)

        core = CORE.GameControlCore()
        first = CORE.InputSnapshot(
            sequence=first_client,
            timestamp_monotonic_s=10.0,
            focused=True,
            camera_yaw_rad=0.0,
            keys=CORE.KeySnapshot(False, False, False, False, False, False, False),
            move_stick=CORE.MoveStickSnapshot(0.0, 0.0),
        )
        second = CORE.InputSnapshot(
            sequence=second_client,
            timestamp_monotonic_s=10.1,
            focused=True,
            camera_yaw_rad=0.0,
            keys=first.keys,
            move_stick=first.move_stick,
        )
        core.accept_snapshot(first, received_at_s=10.0)
        core.accept_snapshot(second, received_at_s=10.1)
        self.assertEqual(core.command(now_s=10.1, dt_s=0.01).sequence, second_client)

    def test_sequence_must_fit_strict_core_protocol(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "protocol range"):
            MODULE.initial_sequence(lambda: 2**63)


class LinuxJoystickTest(unittest.TestCase):
    def test_axis_events_are_normalized_without_external_packages(self) -> None:
        events = [
            MODULE._JS_EVENT.pack(1, 16384, MODULE._JS_EVENT_AXIS, 0),
            MODULE._JS_EVENT.pack(
                2, -32767, MODULE._JS_EVENT_AXIS | MODULE._JS_EVENT_INIT, 1
            ),
            MODULE._JS_EVENT.pack(3, -16384, MODULE._JS_EVENT_AXIS, 3),
        ]
        closed: list[int] = []

        def reader(_fd: int, _size: int) -> bytes:
            if events:
                return events.pop(0)
            raise BlockingIOError()

        joystick = MODULE.LinuxJoystick(
            "/dev/input/js-test",
            left_x_axis=0,
            left_y_axis=1,
            right_x_axis=3,
            right_y_axis=4,
            opener=lambda *_args: 41,
            reader=reader,
            closer=closed.append,
        )
        sample = joystick.poll(10.0)
        self.assertTrue(sample.connected)
        self.assertAlmostEqual(sample.right, 16384 / 32767.0)
        self.assertEqual(sample.forward, 1.0)
        self.assertAlmostEqual(sample.look_yaw, -16384 / 32767.0)
        joystick.close()
        self.assertEqual(closed, [41])


@unittest.skipUnless(
    hasattr(socket, "SOCK_SEQPACKET") and hasattr(socket, "SO_PEERCRED"),
    "Linux Unix seqpacket support is required",
)
class UnixSeqpacketPublisherTest(unittest.TestCase):
    @staticmethod
    def snapshot(sequence: int = 1):
        return CORE.InputSnapshot(
            sequence=sequence,
            timestamp_monotonic_s=10.0,
            focused=True,
            camera_yaw_rad=0.0,
            keys=CORE.KeySnapshot(True, False, False, False, False, False, False),
            move_stick=CORE.MoveStickSnapshot(0.0, 0.0),
        )

    def test_publisher_connects_to_authenticated_core_server(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "input.sock"
            with CORE.UnixSeqpacketInputServer(path) as server:
                publisher = MODULE.UnixSeqpacketPublisher(path)
                try:
                    self.assertTrue(publisher.send(self.snapshot(), now=10.0))
                    with server.accept(timeout_s=1.0) as connection:
                        self.assertEqual(
                            connection.receive(timeout_s=1.0), self.snapshot()
                        )
                finally:
                    publisher.close()

    def test_missing_server_is_nonblocking_and_reconnects(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "input.sock"
            publisher = MODULE.UnixSeqpacketPublisher(path, reconnect_seconds=0.2)
            try:
                self.assertFalse(publisher.send(self.snapshot(), now=10.0))
                with CORE.UnixSeqpacketInputServer(path) as server:
                    self.assertFalse(publisher.send(self.snapshot(), now=10.1))
                    self.assertTrue(publisher.send(self.snapshot(2), now=10.21))
                    with server.accept(timeout_s=1.0) as connection:
                        self.assertEqual(connection.receive(timeout_s=1.0).sequence, 2)
            finally:
                publisher.close()

    def test_connect_has_a_bounded_io_timeout(self) -> None:
        calls: list[tuple[str, object]] = []

        class NeverConnects:
            def settimeout(self, value: float) -> None:
                calls.append(("timeout", value))

            def connect(self, path: str) -> None:
                calls.append(("connect", path))
                raise socket.timeout("bounded")

            def close(self) -> None:
                calls.append(("close", None))

        publisher = MODULE.UnixSeqpacketPublisher(
            "/tmp/never-connects.sock",
            io_timeout_seconds=0.007,
            socket_factory=lambda *_args: NeverConnects(),
        )
        self.assertFalse(publisher.send(self.snapshot(), now=10.0))
        self.assertEqual(calls[0], ("timeout", 0.007))
        self.assertEqual(calls[-1], ("close", None))

    def test_partial_seqpacket_write_drops_the_connection(self) -> None:
        calls: list[str] = []

        class PartialWriter:
            def settimeout(self, _value: float) -> None:
                pass

            def connect(self, _path: str) -> None:
                pass

            def send(self, payload: bytes) -> int:
                calls.append("send")
                return len(payload) - 1

            def close(self) -> None:
                calls.append("close")

        publisher = MODULE.UnixSeqpacketPublisher(
            "/tmp/partial-writer.sock",
            socket_factory=lambda *_args: PartialWriter(),
        )
        self.assertFalse(publisher.send(self.snapshot(), now=10.0))
        self.assertFalse(publisher.connected)
        self.assertEqual(calls, ["send", "close"])


if __name__ == "__main__":
    unittest.main()
