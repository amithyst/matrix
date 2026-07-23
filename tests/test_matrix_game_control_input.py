from __future__ import annotations

import ctypes
import ctypes.util
import importlib
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import socket
import tempfile
import unittest
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = REPO_ROOT / "scripts"
if os.fspath(SCRIPTS) not in os.sys.path:
    os.sys.path.insert(0, os.fspath(SCRIPTS))
CORE = importlib.import_module("matrix_game_control")
EXTERNAL = importlib.import_module("matrix_external_control")
MC_COMMANDS = importlib.import_module("matrix_mc_commands")
MOTION_SETTINGS = importlib.import_module("matrix_motion_settings")
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
                    "kind": "action",
                    "action": "profile_remote",
                },
                {
                    "version": 1,
                    "session": supervisor._action_session,
                    "sequence": 2,
                    "kind": "action",
                    "action": "speed_down",
                },
            )
            try:
                for packet in packets:
                    sender.send(json.dumps(packet).encode("ascii"))
                self.assertEqual(
                    supervisor.drain_intents(),
                    (
                        MODULE.OverlayIntent(kind="action", action="profile_remote"),
                        MODULE.OverlayIntent(kind="action", action="speed_down"),
                    ),
                )
                self.assertEqual(supervisor.drain_intents(), ())
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
                "kind": "action",
                "action": "profile_remote",
            }
            try:
                sender.send(json.dumps(packet).encode("ascii"))
                intents = supervisor.drain_intents()
                self.assertEqual(
                    intents,
                    (MODULE.OverlayIntent(kind="action", action="profile_remote"),),
                )
                for intent in intents:
                    self.assertTrue(
                        controller.apply_panel_action(intent.action, active=True)
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
                {
                    "version": 1,
                    "session": "wrong",
                    "sequence": 1,
                    "kind": "action",
                    "action": "speed_up",
                },
                {
                    "version": 1,
                    "session": "placeholder",
                    "sequence": 1,
                    "kind": "action",
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
                    with self.assertRaisesRegex(RuntimeError, "identity|action intent"):
                        supervisor.drain_intents()
                finally:
                    sender.close()
                    receiver.close()
                    supervisor._action_socket = None

    def test_private_intent_socket_accepts_strict_command_edit_and_submit(self) -> None:
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
                    "kind": "command_edit",
                    "active": True,
                },
                {
                    "version": 1,
                    "session": supervisor._action_session,
                    "sequence": 2,
                    "kind": "command_submit",
                    "command": "/tp @s ~1 ~ ~",
                },
                {
                    "version": 1,
                    "session": supervisor._action_session,
                    "sequence": 3,
                    "kind": "strategy_select",
                    "slot": "recovery",
                    "policy_id": "kungfu",
                },
            )
            try:
                for packet in packets:
                    sender.send(json.dumps(packet).encode("utf-8"))
                self.assertEqual(
                    supervisor.drain_intents(),
                    (
                        MODULE.OverlayIntent(kind="command_edit", active=True),
                        MODULE.OverlayIntent(
                            kind="command_submit", command="/tp @s ~1 ~ ~"
                        ),
                        MODULE.OverlayIntent(
                            kind="strategy_select",
                            slot="recovery",
                            policy_id="kungfu",
                        ),
                    ),
                )
            finally:
                sender.close()
                receiver.close()
                supervisor._action_socket = None

    def test_private_intent_socket_rejects_schema_smuggling_and_oversize(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            script = root / "matrix_calibration_overlay.py"
            script.write_text("", encoding="utf-8")
            invalid_packets = (
                {
                    "version": 1,
                    "session": "placeholder",
                    "sequence": 1,
                    "kind": "command_edit",
                    "active": 1,
                },
                {
                    "version": 1,
                    "session": "placeholder",
                    "sequence": 1,
                    "kind": "command_submit",
                    "command": "/tp @s 1 2 3",
                    "action": "apply_return",
                },
                {
                    "version": 1,
                    "session": "placeholder",
                    "sequence": 1,
                    "kind": "command_submit",
                    "command": "x" * (MC_COMMANDS.MAX_COMMAND_CHARS + 1),
                },
            )
            for packet in invalid_packets:
                supervisor = MODULE.CalibrationOverlaySupervisor(
                    state_file=root / "state.json",
                    display_name=None,
                    expected_ue_pid=41,
                    script=script,
                )
                packet["session"] = supervisor._action_session
                receiver, sender = socket.socketpair(
                    socket.AF_UNIX, socket.SOCK_SEQPACKET
                )
                receiver.setblocking(False)
                supervisor._action_socket = receiver
                try:
                    sender.send(json.dumps(packet).encode("utf-8"))
                    with self.assertRaisesRegex(RuntimeError, "intent"):
                        supervisor.drain_intents()
                finally:
                    sender.close()
                    receiver.close()
                    supervisor._action_socket = None

            supervisor = MODULE.CalibrationOverlaySupervisor(
                state_file=root / "state.json",
                display_name=None,
                expected_ue_pid=41,
                script=script,
            )
            receiver, sender = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            receiver.setblocking(False)
            supervisor._action_socket = receiver
            try:
                sender.send(b"x" * (supervisor._MAX_INTENT_PACKET_BYTES + 1))
                with self.assertRaisesRegex(RuntimeError, "oversized"):
                    supervisor.drain_intents()
            finally:
                sender.close()
                receiver.close()
                supervisor._action_socket = None


@unittest.skipUnless(
    hasattr(socket, "SOCK_SEQPACKET"), "Unix SOCK_SEQPACKET is required"
)
class GameCommandClientTest(unittest.TestCase):
    @staticmethod
    def motion_settings_telemetry(*, revision: int = 0) -> dict[str, object]:
        return {
            "settings_file": "/home/user/.config/matrix/hosts/trna/motion-control.json",
            "load_status": "loaded",
            "load_error": None,
            "settings": MOTION_SETTINGS.MotionSettings(
                revision=revision
            ).to_mapping(),
        }

    def test_motion_settings_telemetry_prefers_latest_runtime_ack(self) -> None:
        initial = self.motion_settings_telemetry(revision=0)
        provider, runtime = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        client = MODULE.GameCommandClient(
            provider.detach(),
            initial_strategy_loadout=self.strategy_loadout(),
            initial_motion_settings=initial,
        )
        self.addCleanup(client.close)
        self.addCleanup(runtime.close)

        self.assertEqual(
            MODULE.live_motion_settings_telemetry(initial, client),
            initial,
        )
        updated = self.motion_settings_telemetry(revision=1)
        self.assertTrue(
            client.submit(
                "/data modify entity @s "
                "control.motion.gears.slow.speed_mps set value 0.15",
                calibration_active=True,
                neutral_frame_ready=True,
                restart_requested=False,
            )
        )
        motion_request = MC_COMMANDS.decode_command_request(runtime.recv(4096))
        runtime.send(
            MC_COMMANDS.encode_command_response(
                MC_COMMANDS.GameCommandResponse(
                    session=motion_request.session,
                    sequence=motion_request.sequence,
                    request_id=motion_request.request_id,
                    ok=True,
                    code="OK_DATA_MODIFIED",
                    message="updated",
                    data={"motion_settings": updated},
                )
            )
        )
        self.assertTrue(client.poll())
        self.assertEqual(
            MODULE.live_motion_settings_telemetry(initial, client),
            updated,
        )
        # A later unrelated ACK replaces command_client.data, while the
        # dedicated runtime-owned motion snapshot must stay at revision 1.
        self.assertTrue(
            client.select_policy(
                "recovery",
                "host",
                calibration_active=True,
                neutral_frame_ready=True,
                restart_requested=False,
            )
        )
        policy_request = MC_COMMANDS.decode_command_request(runtime.recv(4096))
        runtime.send(
            MC_COMMANDS.encode_command_response(
                MC_COMMANDS.GameCommandResponse(
                    session=policy_request.session,
                    sequence=policy_request.sequence,
                    request_id=policy_request.request_id,
                    ok=True,
                    code="OK_POLICY_SLOT_ASSIGNED",
                    message="assigned",
                    data={"strategy_loadout": self.strategy_loadout(recovery="host")},
                )
            )
        )
        self.assertTrue(client.poll())
        self.assertEqual(
            MODULE.live_motion_settings_telemetry(initial, client),
            updated,
        )
        malformed = dict(updated)
        malformed["unknown"] = True
        with self.assertRaisesRegex(ValueError, "schema"):
            MODULE.validate_motion_settings_telemetry(malformed)

    @staticmethod
    def strategy_loadout(recovery="kungfu", status="ready"):
        return {
            "version": 1,
            "available": True,
            "status": status,
            "active_slot": "locomotion",
            "pending": None,
            "slots": [
                {
                    "slot": "locomotion",
                    "selected_policy_id": "sonic",
                    "locked": True,
                    "candidates": [
                        {
                            "policy_id": "sonic",
                            "resident": True,
                            "available": True,
                        }
                    ],
                },
                {
                    "slot": "recovery",
                    "selected_policy_id": recovery,
                    "locked": False,
                    "candidates": [
                        {
                            "policy_id": policy_id,
                            "resident": True,
                            "available": True,
                        }
                        for policy_id in ("kungfu", "host", "amp")
                    ],
                },
            ],
            "resident_models": [],
        }

    @staticmethod
    def creative_inventory(remaining=8, spawn_count=0):
        return {
            "version": 1,
            "available": True,
            "spawn_count": spawn_count,
            "items": [
                {
                    "item_id": "training_blaster",
                    "label": "Training Blaster",
                    "pool_size": 8,
                    "remaining": remaining,
                }
            ],
        }

    def make_client(self):
        provider, runtime = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        client = MODULE.GameCommandClient(provider.detach())
        runtime.setblocking(False)
        self.addCleanup(client.close)
        self.addCleanup(runtime.close)
        return client, runtime

    @staticmethod
    def enable_editor(client) -> None:
        assert client.set_editing(
            True, panel_active=True, restart_requested=False
        )

    def test_unavailable_channel_cannot_enter_editor_or_capture_escape(self) -> None:
        client = MODULE.GameCommandClient(None)
        self.addCleanup(client.close)
        self.assertFalse(
            client.set_editing(True, panel_active=True, restart_requested=False)
        )
        self.assertFalse(client.editing)
        self.assertTrue(
            client.panel_escape_pressed(True, editor_owned_this_frame=True)
        )
        self.assertFalse(client.panel_escape_pressed(False))

    def test_command_channel_rejects_a_unix_stream_socket(self) -> None:
        provider, runtime = socket.socketpair(socket.AF_UNIX, socket.SOCK_STREAM)
        descriptor = provider.detach()
        self.addCleanup(runtime.close)
        with self.assertRaisesRegex(ValueError, "SOCK_SEQPACKET"):
            MODULE.GameCommandClient(descriptor)

    def test_raw_text_is_parsed_to_typed_ast_and_response_is_surfaced(self) -> None:
        client, runtime = self.make_client()
        self.enable_editor(client)

        self.assertTrue(
            client.submit(
                "/tp @s ~1 2 ~-3",
                calibration_active=True,
                neutral_frame_ready=True,
                restart_requested=False,
            )
        )
        payload = runtime.recv(MC_COMMANDS.MAX_COMMAND_PACKET_BYTES + 1)
        self.assertNotIn(b"/tp", payload)
        request = MC_COMMANDS.decode_command_request(payload)
        self.assertIsInstance(request.command, MC_COMMANDS.TeleportCoordinates)
        self.assertEqual(request.sequence, 1)
        self.assertTrue(client.in_flight)

        response = MC_COMMANDS.GameCommandResponse(
            session=request.session,
            sequence=request.sequence,
            request_id=request.request_id,
            ok=True,
            code="OK_TELEPORT_RESTART",
            message="Teleport saved",
            restart_required=True,
            data={"position": [1.0, 2.0, 3.0]},
        )
        runtime.send(MC_COMMANDS.encode_command_response(response))
        self.assertTrue(client.poll())
        self.assertFalse(client.in_flight)
        self.assertEqual(
            client.mapping(),
            {
                "available": True,
                "editing": True,
                "in_flight": False,
                "status": "restarting",
                "request_id": request.request_id,
                "sequence": 1,
                "result_revision": 2,
                "ok": True,
                "code": "OK_TELEPORT_RESTART",
                "message": "Teleport saved",
                "warning": None,
                "restart_required": True,
                "outcome_unknown": False,
                "data": {"position": [1.0, 2.0, 3.0]},
            },
        )

    def test_strategy_slot_select_skips_text_editor_and_tracks_runtime_ack(self) -> None:
        provider, runtime = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        client = MODULE.GameCommandClient(
            provider.detach(),
            initial_strategy_loadout=self.strategy_loadout(),
        )
        runtime.settimeout(1.0)
        self.addCleanup(client.close)
        self.addCleanup(runtime.close)

        self.assertTrue(
            client.select_policy(
                "recovery",
                "host",
                calibration_active=True,
                neutral_frame_ready=True,
                restart_requested=False,
            )
        )
        request = MC_COMMANDS.decode_command_request(runtime.recv(4096))
        self.assertEqual(
            request.command,
            MC_COMMANDS.PolicySlotAssignment("recovery", "host"),
        )
        self.assertFalse(client.editing)
        changed = self.strategy_loadout(recovery="host")
        runtime.send(
            MC_COMMANDS.encode_command_response(
                MC_COMMANDS.GameCommandResponse(
                    session=request.session,
                    sequence=request.sequence,
                    request_id=request.request_id,
                    ok=True,
                    code="OK_POLICY_SLOT_ASSIGNED",
                    message="assigned",
                    data={"strategy_loadout": changed},
                )
            )
        )

        self.assertTrue(client.poll())
        self.assertEqual(
            client.strategy_loadout_mapping()["slots"][1]["selected_policy_id"],
            "host",
        )

    def test_creative_spawn_skips_editor_and_tracks_remaining_inventory(self) -> None:
        provider, runtime = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        client = MODULE.GameCommandClient(
            provider.detach(),
            initial_creative_inventory=self.creative_inventory(),
        )
        runtime.settimeout(1.0)
        self.addCleanup(client.close)
        self.addCleanup(runtime.close)

        self.assertTrue(
            client.spawn_creative_item(
                "training_blaster",
                calibration_active=True,
                neutral_frame_ready=True,
                restart_requested=False,
            )
        )
        request = MC_COMMANDS.decode_command_request(runtime.recv(4096))
        self.assertEqual(
            request.command,
            MC_COMMANDS.CreativeSpawnItem("training_blaster"),
        )
        self.assertFalse(client.editing)
        changed = self.creative_inventory(remaining=7, spawn_count=1)
        runtime.send(
            MC_COMMANDS.encode_command_response(
                MC_COMMANDS.GameCommandResponse(
                    session=request.session,
                    sequence=request.sequence,
                    request_id=request.request_id,
                    ok=True,
                    code="OK_INVENTORY_SPAWNED",
                    message="placed",
                    data={
                        "creative_inventory": changed,
                        "spawned_item": {
                            "item_id": "training_blaster",
                            "instance_name": "creative_item__training_blaster__0",
                            "position": [0.9, 0.0, 1.0],
                            "quaternion": [1.0, 0.0, 0.0, 0.0],
                        },
                    },
                )
            )
        )

        self.assertTrue(client.poll())
        inventory = client.creative_inventory_mapping()
        self.assertEqual(inventory["spawn_count"], 1)
        self.assertEqual(inventory["items"][0]["remaining"], 7)

    def test_only_one_request_is_in_flight_and_restart_response_is_terminal(self) -> None:
        client, runtime = self.make_client()
        self.enable_editor(client)
        arguments = {
            "calibration_active": True,
            "neutral_frame_ready": True,
            "restart_requested": False,
        }
        self.assertTrue(client.submit("/tp @s 1 2 3", **arguments))
        request = MC_COMMANDS.decode_command_request(runtime.recv(4096))

        self.assertFalse(client.submit("/tp @s 4 5 6", **arguments))
        for _ in range(5):
            self.assertFalse(client.poll())
        with self.assertRaises(BlockingIOError):
            runtime.recv(4096)

        runtime.send(
            MC_COMMANDS.encode_command_response(
                MC_COMMANDS.GameCommandResponse(
                    session=request.session,
                    sequence=request.sequence,
                    request_id=request.request_id,
                    ok=True,
                    code="OK_TELEPORT_RESTART",
                    message="saved",
                    restart_required=True,
                )
            )
        )
        self.assertTrue(client.poll())
        self.assertTrue(client.restart_required)
        self.assertFalse(client.submit("/tp @s 4 5 6", **arguments))
        with self.assertRaises(BlockingIOError):
            runtime.recv(4096)

    def test_external_data_modify_stays_provider_side_without_panel(self) -> None:
        client, runtime = self.make_client()
        modified = []
        token = EXTERNAL.ExternalInputToken("a" * 32, 1, 2)

        self.assertTrue(
            client.submit_external(
                "/data modify entity @s control.input.keyboard.w set value true",
                calibration_active=False,
                neutral_frame_ready=False,
                restart_requested=False,
                input_modifier=lambda command: (
                    modified.append(command) or token,
                    {"ok": True},
                ),
            )
        )
        self.assertEqual(
            modified,
            [MC_COMMANDS.DataModifyInput("control.input.keyboard.w", True)],
        )
        self.assertTrue(client.in_flight)
        self.assertEqual(client.status, "pending")
        self.assertIsNone(client.code)
        self.assertEqual(client.data, {"ok": True})
        stale = EXTERNAL.ExternalInputToken("a" * 32, 1, 1)
        self.assertFalse(
            client.resolve_external_input_publish(
                sampled_token=stale,
                current_token=token,
                authority_active=True,
                published=True,
                locomotion_admitted=True,
                interlock_reason=None,
            )
        )
        self.assertTrue(client.in_flight)
        self.assertTrue(
            client.resolve_external_input_publish(
                sampled_token=token,
                current_token=token,
                authority_active=True,
                published=True,
                locomotion_admitted=True,
                interlock_reason=None,
                data={"published": True},
            )
        )
        self.assertFalse(client.in_flight)
        self.assertEqual(client.code, "OK_DATA_INPUT_MODIFIED")
        self.assertEqual(client.data, {"published": True})
        with self.assertRaises(BlockingIOError):
            runtime.recv(4096)

    def test_external_input_publish_failures_are_typed_and_terminal(self) -> None:
        token = EXTERNAL.ExternalInputToken("a" * 32, 1, 2)
        cases = {
            "input_source_rejects_keyboard": (True, "E_INPUT_INTERLOCK"),
            "physical_focus_lost": (True, "E_INPUT_INTERLOCK"),
            "camera_unavailable": (True, "E_INPUT_INTERLOCK"),
            "calibration_interlock": (True, "E_INPUT_INTERLOCK"),
            "gamepad_connected_edge": (True, "E_INPUT_INTERLOCK"),
            None: (False, "E_INPUT_PUBLISH_FAILED"),
        }
        for reason, (published, expected_code) in cases.items():
            with self.subTest(reason=reason):
                client, _runtime = self.make_client()
                self.assertTrue(
                    client.submit_external(
                        "/data modify entity @s "
                        "control.input.keyboard.w set value true",
                        calibration_active=False,
                        neutral_frame_ready=False,
                        restart_requested=False,
                        input_modifier=lambda _command: (token, None),
                    )
                )
                self.assertTrue(
                    client.resolve_external_input_publish(
                        sampled_token=token,
                        current_token=token,
                        authority_active=True,
                        published=published,
                        locomotion_admitted=True,
                        interlock_reason=reason,
                    )
                )
                self.assertFalse(client.in_flight)
                self.assertFalse(client.ok)
                self.assertEqual(client.code, expected_code)
                if reason is not None:
                    self.assertIn(reason, client.message)

    def test_external_input_publish_supersede_revoke_and_shutdown(self) -> None:
        token = EXTERNAL.ExternalInputToken("a" * 32, 1, 2)
        successor = EXTERNAL.ExternalInputToken("a" * 32, 1, 3)

        client, _runtime = self.make_client()
        self.assertTrue(
            client.submit_external(
                "/data modify entity @s control.input.keyboard.w set value true",
                calibration_active=False,
                neutral_frame_ready=False,
                restart_requested=False,
                input_modifier=lambda _command: (token, None),
            )
        )
        self.assertTrue(
            client.resolve_external_input_publish(
                sampled_token=token,
                current_token=successor,
                authority_active=True,
                published=True,
                locomotion_admitted=True,
                interlock_reason=None,
            )
        )
        self.assertEqual(client.code, "E_INPUT_SUPERSEDED")

        client, _runtime = self.make_client()
        self.assertTrue(
            client.submit_external(
                "/data modify entity @s control.input.keyboard.w set value true",
                calibration_active=False,
                neutral_frame_ready=False,
                restart_requested=False,
                input_modifier=lambda _command: (token, None),
            )
        )
        self.assertTrue(
            client.resolve_external_input_publish(
                sampled_token=None,
                current_token=None,
                authority_active=False,
                published=False,
                locomotion_admitted=False,
                interlock_reason=None,
            )
        )
        self.assertEqual(client.code, "E_AUTHORITY_REVOKED")

        client, _runtime = self.make_client()
        self.assertTrue(
            client.submit_external(
                "/data modify entity @s control.input.keyboard.w set value true",
                calibration_active=False,
                neutral_frame_ready=False,
                restart_requested=False,
                input_modifier=lambda _command: (token, None),
            )
        )
        client.close()
        self.assertFalse(client.in_flight)
        self.assertTrue(client.outcome_unknown)
        self.assertEqual(client.code, "E_COMMAND_OUTCOME_UNKNOWN")

    def test_external_world_command_keeps_pause_gate_but_skips_editor_gate(self) -> None:
        client, runtime = self.make_client()
        arguments = {
            "neutral_frame_ready": True,
            "restart_requested": False,
            "input_modifier": lambda _command: None,
        }
        self.assertFalse(
            client.submit_external(
                "/tp @s ~ ~ ~",
                calibration_active=False,
                **arguments,
            )
        )
        self.assertEqual(client.code, "E_NOT_PAUSED")
        self.assertTrue(
            client.submit_external(
                "/tp @s ~ ~ ~",
                calibration_active=True,
                **arguments,
            )
        )
        request = MC_COMMANDS.decode_command_request(runtime.recv(4096))
        self.assertIsInstance(request.command, MC_COMMANDS.TeleportCoordinates)
        self.assertFalse(client.editing)

    def test_submit_requires_panel_neutral_editor_and_no_restart(self) -> None:
        client, runtime = self.make_client()
        self.enable_editor(client)
        cases = (
            (
                {
                    "calibration_active": False,
                    "neutral_frame_ready": True,
                    "restart_requested": False,
                },
                "E_NOT_PAUSED",
            ),
            (
                {
                    "calibration_active": True,
                    "neutral_frame_ready": False,
                    "restart_requested": False,
                },
                "E_NEUTRAL_REQUIRED",
            ),
            (
                {
                    "calibration_active": True,
                    "neutral_frame_ready": True,
                    "restart_requested": True,
                },
                "E_RESTART_PENDING",
            ),
        )
        for arguments, code in cases:
            with self.subTest(code=code):
                self.assertFalse(client.submit("/tp @s 1 2 3", **arguments))
                self.assertEqual(client.code, code)
                with self.assertRaises(BlockingIOError):
                    runtime.recv(4096)

        self.assertTrue(
            client.set_editing(False, panel_active=True, restart_requested=False)
        )
        self.assertFalse(
            client.submit(
                "/tp @s 1 2 3",
                calibration_active=True,
                neutral_frame_ready=True,
                restart_requested=False,
            )
        )
        self.assertEqual(client.code, "E_COMMAND_EDIT_REQUIRED")

    def test_whitelisted_data_modify_button_skips_text_editor_only(self) -> None:
        client, runtime = self.make_client()
        self.assertFalse(client.editing)

        self.assertTrue(
            client.submit(
                "/data modify entity @s "
                "control.motion.gears.slow.speed_mps set value 0.15",
                calibration_active=True,
                neutral_frame_ready=True,
                restart_requested=False,
            )
        )
        request = MC_COMMANDS.decode_command_request(runtime.recv(4096))
        self.assertEqual(
            request.command,
            MC_COMMANDS.DataModifyNumber(
                "control.motion.gears.slow.speed_mps", 0.15
            ),
        )
        self.assertFalse(client.editing)

        blocked, blocked_runtime = self.make_client()
        self.assertFalse(
            blocked.submit(
                "/data modify entity @s "
                "control.motion.gears.slow.speed_mps set value 0.15",
                calibration_active=False,
                neutral_frame_ready=True,
                restart_requested=False,
            )
        )
        self.assertEqual(blocked.code, "E_NOT_PAUSED")
        with self.assertRaises(BlockingIOError):
            blocked_runtime.recv(4096)

    def test_data_modify_input_never_crosses_the_private_runtime_channel(self) -> None:
        client, runtime = self.make_client()
        self.enable_editor(client)
        self.assertFalse(
            client.submit(
                "/data modify entity @s control.input.keyboard.w set value true",
                calibration_active=True,
                neutral_frame_ready=True,
                restart_requested=False,
            )
        )
        self.assertEqual(client.code, "E_EXTERNAL_API_REQUIRED")
        with self.assertRaises(BlockingIOError):
            runtime.recv(4096)

    def test_parser_error_and_summom_warning_stay_provider_side(self) -> None:
        client, runtime = self.make_client()
        self.enable_editor(client)
        arguments = {
            "calibration_active": True,
            "neutral_frame_ready": True,
            "restart_requested": False,
        }
        self.assertFalse(client.submit("/tp @s 1 2", **arguments))
        self.assertEqual(client.code, "E_COORD_ARITY")
        with self.assertRaises(BlockingIOError):
            runtime.recv(4096)

        self.assertTrue(
            client.submit(
                '/summom matrix:teleport_point ~ ~ ~ {Tags:["XX"]}',
                **arguments,
            )
        )
        request = MC_COMMANDS.decode_command_request(runtime.recv(4096))
        self.assertIsInstance(request.command, MC_COMMANDS.SummonTeleportPoint)
        self.assertIn("/summon", client.warning or "")
        runtime.send(
            MC_COMMANDS.encode_command_response(
                MC_COMMANDS.GameCommandResponse(
                    session=request.session,
                    sequence=request.sequence,
                    request_id=request.request_id,
                    ok=True,
                    code="OK_SUMMONED",
                    message="Summoned teleport point",
                )
            )
        )
        client.poll()
        self.assertIn("/summon", client.warning or "")

    def test_repeated_identical_parse_error_has_a_new_result_revision(self) -> None:
        client, runtime = self.make_client()
        self.enable_editor(client)
        arguments = {
            "calibration_active": True,
            "neutral_frame_ready": True,
            "restart_requested": False,
        }
        revisions = []
        for _ in range(2):
            self.assertFalse(client.submit("/tp @s 1 2", **arguments))
            mapping = client.mapping()
            self.assertEqual(mapping["code"], "E_COORD_ARITY")
            self.assertEqual(
                mapping["message"],
                "tp @s requires three coordinates or one selector",
            )
            revisions.append(mapping["result_revision"])
            with self.assertRaises(BlockingIOError):
                runtime.recv(4096)
        self.assertEqual(revisions, [1, 2])

    def test_wrong_response_identity_preserves_unknown_outcome_without_retry(self) -> None:
        client, runtime = self.make_client()
        self.enable_editor(client)
        self.assertTrue(
            client.submit(
                "/tp @s 1 2 3",
                calibration_active=True,
                neutral_frame_ready=True,
                restart_requested=False,
            )
        )
        request = MC_COMMANDS.decode_command_request(runtime.recv(4096))
        runtime.send(
            MC_COMMANDS.encode_command_response(
                MC_COMMANDS.GameCommandResponse(
                    session="f" * 32,
                    sequence=request.sequence,
                    request_id=request.request_id,
                    ok=False,
                    code="E_TEST_RESPONSE",
                    message="wrong session",
                )
            )
        )

        self.assertTrue(client.poll())
        self.assertFalse(client.available)
        self.assertFalse(client.in_flight)
        self.assertIsNone(client.ok)
        self.assertEqual(client.code, "E_COMMAND_OUTCOME_UNKNOWN")
        self.assertEqual(client.last_request_id, request.request_id)
        self.assertTrue(client.mapping()["outcome_unknown"])
        self.assertIn("do not retry blindly", client.message or "")
        self.assertIn("identity", client.message or "")
        self.assertFalse(
            client.submit(
                "/tp @s 4 5 6",
                calibration_active=True,
                neutral_frame_ready=True,
                restart_requested=False,
            )
        )
        self.assertEqual(client.code, "E_COMMAND_OUTCOME_UNKNOWN")
        self.assertEqual(client.last_request_id, request.request_id)
        self.assertTrue(
            client.set_editing(False, panel_active=True, restart_requested=False)
        )
        self.assertFalse(client.editing)
        self.assertFalse(client.panel_escape_pressed(True))
        self.assertFalse(client.panel_closed())

    def test_eof_after_send_is_unknown_but_unsolicited_response_is_protocol_error(self) -> None:
        client, runtime = self.make_client()
        self.enable_editor(client)
        self.assertTrue(
            client.submit(
                '/summon matrix:teleport_point ~ ~ ~ {Tags:["maybe"]}',
                calibration_active=True,
                neutral_frame_ready=True,
                restart_requested=False,
            )
        )
        request = MC_COMMANDS.decode_command_request(runtime.recv(4096))
        runtime.close()
        self.assertTrue(client.poll())
        self.assertEqual(client.code, "E_COMMAND_OUTCOME_UNKNOWN")
        self.assertEqual(client.last_request_id, request.request_id)

        unsolicited, peer = self.make_client()
        peer.send(
            MC_COMMANDS.encode_command_response(
                MC_COMMANDS.GameCommandResponse(
                    session="a" * 32,
                    sequence=1,
                    request_id="cmd-" + "b" * 32,
                    ok=False,
                    code="E_TEST_RESPONSE",
                    message="unsolicited",
                )
            )
        )
        self.assertTrue(unsolicited.poll())
        self.assertEqual(unsolicited.code, "E_COMMAND_PROTOCOL")
        self.assertIsNone(unsolicited.last_request_id)

    def test_shutdown_marks_an_unacknowledged_request_outcome_unknown(self) -> None:
        client, runtime = self.make_client()
        self.enable_editor(client)
        self.assertTrue(
            client.submit(
                '/summon matrix:teleport_point ~ ~ ~ {Tags:["maybe"]}',
                calibration_active=True,
                neutral_frame_ready=True,
                restart_requested=False,
            )
        )
        request = MC_COMMANDS.decode_command_request(runtime.recv(4096))

        # Production cleanup can be entered by SIGTERM before the provider's
        # next frame observes EOF from the runtime.  close() is therefore the
        # final authority for resolving an in-flight request safely.
        client.close()

        self.assertFalse(client.available)
        self.assertFalse(client.in_flight)
        self.assertTrue(client.outcome_unknown)
        self.assertIsNone(client.ok)
        self.assertEqual(client.code, "E_COMMAND_OUTCOME_UNKNOWN")
        self.assertEqual(client.last_request_id, request.request_id)
        self.assertIn("do not retry blindly", client.message or "")

    def test_shutdown_drains_a_buffered_response_before_closing(self) -> None:
        client, runtime = self.make_client()
        self.enable_editor(client)
        self.assertTrue(
            client.submit(
                "/tp @s 1 2 3",
                calibration_active=True,
                neutral_frame_ready=True,
                restart_requested=False,
            )
        )
        request = MC_COMMANDS.decode_command_request(runtime.recv(4096))
        runtime.send(
            MC_COMMANDS.encode_command_response(
                MC_COMMANDS.GameCommandResponse(
                    session=request.session,
                    sequence=request.sequence,
                    request_id=request.request_id,
                    ok=True,
                    code="OK_TELEPORT_RESTART",
                    message="saved",
                    restart_required=True,
                )
            )
        )

        client.close()

        self.assertFalse(client.available)
        self.assertFalse(client.in_flight)
        self.assertFalse(client.outcome_unknown)
        self.assertIs(client.ok, True)
        self.assertEqual(client.status, "restarting")
        self.assertEqual(client.code, "OK_TELEPORT_RESTART")
        self.assertEqual(client.last_request_id, request.request_id)

    def test_first_escape_exits_editor_only_after_release_and_pending_blocks_exit(self) -> None:
        client, runtime = self.make_client()
        # Even a begin/end pair drained inside one provider frame owns Escape.
        self.assertFalse(
            client.panel_escape_pressed(True, editor_owned_this_frame=True)
        )
        self.assertFalse(client.panel_escape_pressed(False))
        self.assertTrue(client.panel_escape_pressed(True))
        self.assertFalse(client.panel_escape_pressed(False))
        self.enable_editor(client)

        self.assertFalse(client.panel_escape_pressed(True))
        self.assertTrue(
            client.set_editing(False, panel_active=True, restart_requested=False)
        )
        self.assertFalse(client.panel_escape_pressed(True))
        self.assertFalse(client.panel_escape_pressed(False))
        self.assertTrue(client.panel_escape_pressed(True))
        self.assertFalse(client.panel_escape_pressed(False))

        self.enable_editor(client)
        self.assertTrue(
            client.submit(
                "/tp @s 1 2 3",
                calibration_active=True,
                neutral_frame_ready=True,
                restart_requested=False,
            )
        )
        request = MC_COMMANDS.decode_command_request(runtime.recv(4096))
        self.assertTrue(
            client.set_editing(False, panel_active=True, restart_requested=False)
        )
        self.assertFalse(client.editing)
        self.assertFalse(client.panel_escape_pressed(True))
        runtime.send(
            MC_COMMANDS.encode_command_response(
                MC_COMMANDS.GameCommandResponse(
                    session=request.session,
                    sequence=request.sequence,
                    request_id=request.request_id,
                    ok=True,
                    code="OK_TELEPORT_RESTART",
                    message="saved",
                    restart_required=True,
                )
            )
        )
        client.poll()
        # The restart response is terminal for this provider generation.  No
        # Escape or edit intent may reopen/close controls while the runtime is
        # transitioning to its new cold-start generation.
        self.assertFalse(client.panel_escape_pressed(True))
        self.assertFalse(client.panel_escape_pressed(False))
        self.assertFalse(client.panel_escape_pressed(True))
        self.assertFalse(
            client.set_editing(True, panel_active=True, restart_requested=False)
        )
        self.assertFalse(client.editing)
        self.assertFalse(client.panel_escape_pressed(True))
        self.assertFalse(client.panel_escape_pressed(False))
        self.assertFalse(client.panel_escape_pressed(True))

    def test_editor_consumes_global_settings_enter_and_f9_levels_until_release(self) -> None:
        client, _runtime = self.make_client()
        self.assertTrue(
            client.set_editing(True, panel_active=True, restart_requested=False)
        )
        with tempfile.TemporaryDirectory() as temporary:
            settings = MODULE.MouseSettingsController(
                path=Path(temporary) / "mouse.json",
                desired=MODULE.MouseSettings(),
                load_status="missing",
                load_error=None,
            )
            # Poll the physical levels while inactive so a held key cannot turn
            # into a fresh M/-/+ edge when editing ends.
            self.assertFalse(
                settings.update(
                    active=not client.editing,
                    mode_pressed=True,
                    slower_pressed=True,
                    faster_pressed=True,
                )
            )
            client.set_editing(False, panel_active=True, restart_requested=False)
            self.assertFalse(
                settings.update(
                    active=True,
                    mode_pressed=True,
                    slower_pressed=True,
                    faster_pressed=True,
                )
            )
            settings.update(
                active=True,
                mode_pressed=False,
                slower_pressed=False,
                faster_pressed=False,
            )
            self.assertTrue(
                settings.update(
                    active=True,
                    mode_pressed=True,
                    slower_pressed=False,
                    faster_pressed=False,
                )
            )

        class Requester:
            available = True

            def __init__(self) -> None:
                self.calls = 0

            def request(self) -> bool:
                self.calls += 1
                return True

        requester = Requester()
        key = MODULE.ApplyRestartKey()
        key.update(
            pressed=True,
            calibration_active=False,
            neutral_frame_ready=True,
            pending_restart=True,
            persistence_ok=True,
            requester=requester,
        )
        key.update(
            pressed=True,
            calibration_active=True,
            neutral_frame_ready=True,
            pending_restart=True,
            persistence_ok=True,
            requester=requester,
        )
        self.assertEqual(requester.calls, 0)

        calibration = MODULE.CalibrationModeController()
        calibration.active = True
        apply_return = MODULE.ApplyReturnController()
        restart = mock.Mock(available=False, requested=False)
        apply_return.update(
            enter_pressed=True,
            clicked=False,
            ue_focused=False,
            panel_was_active=True,
            calibration=calibration,
            neutral_frame_ready=True,
            pending_restart=False,
            persistence_error=None,
            requester=restart,
        )
        self.assertEqual(
            apply_return.update(
                enter_pressed=True,
                clicked=False,
                ue_focused=True,
                panel_was_active=True,
                calibration=calibration,
                neutral_frame_ready=True,
                pending_restart=False,
                persistence_error=None,
                requester=restart,
            ),
            (False, False),
        )
        self.assertTrue(calibration.active)


class SourceArbitrationTest(unittest.TestCase):
    def setUp(self) -> None:
        self.keyboard = MODULE.KeyboardMouseSample(
            w=True,
            q=True,
            v=True,
            ctrl=True,
            alt=True,
            shift=True,
            focused=True,
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
        self.assertTrue(keys.alt)
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
            MODULE.effective_input_source("auto", "x11-core-gated"), "keyboard"
        )
        self.assertEqual(
            MODULE.effective_input_source("auto", "x11-absolute"), "keyboard"
        )
        self.assertEqual(
            MODULE.effective_input_source("gamepad", "carla"), "gamepad"
        )
        self.assertEqual(
            MODULE.effective_input_source("auto", "ue-final-pov"), "auto"
        )
        self.assertEqual(
            MODULE.effective_input_source("gamepad", "ue-final-pov"),
            "gamepad",
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
        self.assertTrue(keys.alt)
        self.assertTrue(keys.shift)
        self.assertEqual((stick.right, stick.forward, look), (0.0, 0.0, 0.0))

        keys, stick, look = MODULE.select_physical_inputs(
            self.keyboard, self.gamepad, source="gamepad"
        )
        self.assertFalse(keys.w)
        self.assertTrue(keys.q)
        self.assertTrue(keys.v)
        self.assertFalse(keys.ctrl)
        self.assertFalse(keys.alt)
        self.assertFalse(keys.shift)
        self.assertEqual((stick.right, stick.forward), (-0.25, 0.75))
        self.assertEqual(look, 0.5)


class ExternalControlArbitrationTest(unittest.TestCase):
    def test_virtual_full_state_maps_to_provider_samples_and_source(self) -> None:
        state = MODULE.ExternalInputState.neutral()
        mapping = state.to_mapping()
        mapping["keyboard"]["w"] = True
        mapping["keyboard"]["alt"] = True
        mapping["mouse"]["buttons"]["left"] = True
        mapping["mouse"]["dx"] = 4.5
        mapping["gamepad"]["connected"] = True
        mapping["gamepad"]["axes"]["right"] = -0.25
        state = MODULE.ExternalInputState.from_mapping(mapping)
        focus = MODULE.KeyboardMouseSample(
            focused=True,
            focus_title="Matrix",
            focus_pid=42,
        )

        keyboard, gamepad = MODULE.external_input_samples(
            state,
            focus=focus,
            look_button="left",
        )
        self.assertTrue(keyboard.w)
        self.assertTrue(keyboard.alt)
        self.assertTrue(keyboard.camera_dragging)
        self.assertEqual(keyboard.mouse_dx, 4.5)
        self.assertEqual((keyboard.focus_title, keyboard.focus_pid), ("Matrix", 42))
        self.assertTrue(gamepad.connected)
        self.assertEqual(gamepad.right, -0.25)
        self.assertEqual(MODULE.external_active_input_device(state), "mixed")
        self.assertEqual(
            MODULE.external_frame_input_source(state, configured_source="auto"),
            "auto",
        )

    def test_virtual_gamepad_is_selected_only_when_keyboard_mouse_are_neutral(self) -> None:
        mapping = MODULE.ExternalInputState.neutral().to_mapping()
        mapping["gamepad"]["connected"] = True
        mapping["gamepad"]["axes"]["forward"] = 0.75
        state = MODULE.ExternalInputState.from_mapping(mapping)
        self.assertEqual(
            MODULE.external_frame_input_source(state, configured_source="keyboard"),
            "keyboard",
        )
        self.assertEqual(
            MODULE.external_frame_input_source(state, configured_source="auto"),
            "gamepad",
        )

    def test_trna_auto_final_pov_preserves_external_gamepad_movement(self) -> None:
        configured_source = MODULE.effective_input_source(
            "auto", "ue-final-pov"
        )
        mapping = MODULE.ExternalInputState.neutral().to_mapping()
        mapping["gamepad"]["connected"] = True
        mapping["gamepad"]["axes"]["forward"] = 0.5
        state = MODULE.ExternalInputState.from_mapping(mapping)
        keyboard, gamepad = MODULE.external_input_samples(
            state,
            focus=MODULE.KeyboardMouseSample(
                focused=True,
                focus_title="Matrix",
                focus_pid=42,
            ),
            look_button="left",
        )
        frame_source = MODULE.external_frame_input_source(
            state,
            configured_source=configured_source,
        )

        snapshot = MODULE.build_snapshot(
            sequence=1,
            timestamp_monotonic_s=10.0,
            keyboard=keyboard,
            gamepad=gamepad,
            input_source=frame_source,
            camera_yaw_rad=0.25,
            camera_available=True,
        )

        self.assertEqual(configured_source, "auto")
        self.assertEqual(frame_source, "gamepad")
        self.assertTrue(snapshot.focused)
        self.assertEqual(snapshot.move_stick.forward, 0.5)

    def test_any_local_safety_intent_identifies_an_external_override(self) -> None:
        pad = MODULE.GamepadSample()
        cases = (
            (MODULE.KeyboardMouseSample(focused=False), pad, "focus_lost"),
            (
                MODULE.KeyboardMouseSample(focused=True, escape=True),
                pad,
                "physical_escape",
            ),
            (
                MODULE.KeyboardMouseSample(focused=True, w=True),
                pad,
                "physical_keyboard",
            ),
            (
                MODULE.KeyboardMouseSample(focused=True, mouse_dx=1.0),
                pad,
                "physical_mouse",
            ),
            (
                MODULE.KeyboardMouseSample(focused=True),
                MODULE.GamepadSample(connected=True, forward=0.2),
                "physical_gamepad",
            ),
            (
                MODULE.KeyboardMouseSample(focused=True),
                MODULE.GamepadSample(connected=True, buttons_pressed=True),
                "physical_gamepad",
            ),
        )
        for keyboard, gamepad, expected in cases:
            with self.subTest(expected=expected):
                self.assertEqual(
                    MODULE.physical_external_override_reason(keyboard, gamepad),
                    expected,
                )
        self.assertIsNone(
            MODULE.physical_external_override_reason(
                MODULE.KeyboardMouseSample(focused=True),
                pad,
            )
        )
        self.assertIsNone(
            MODULE.physical_external_override_reason(
                MODULE.KeyboardMouseSample(focused=True),
                MODULE.GamepadSample(
                    connected=True,
                    forward=1.0 / 32767.0,
                    look_yaw=0.10,
                ),
            )
        )


class ExternalProviderGateTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        root = Path(self.temporary.name)
        self.clock_value = 10.0

        def clock() -> float:
            return self.clock_value

        self.broker = EXTERNAL.ExternalControlBroker(
            root / "control.sock",
            root / "control.cap",
            clock=clock,
        )
        self.broker.open()
        self.client = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.client.settimeout(1.0)
        self.client.connect(os.fspath(self.broker.path))
        self.broker.poll()
        self.request_sequence = 0
        self.provider_sequence = 100
        acquired = self.request("lease.acquire", {})
        self.lease_id = acquired["data"]["lease_id"]
        self.gate = MODULE.ExternalLocomotionProviderGate(self.broker)

    def tearDown(self) -> None:
        self.client.close()
        self.broker.close()
        self.temporary.cleanup()

    def request(
        self,
        operation: str,
        payload: dict[str, object],
        *,
        expect_ok: bool = True,
    ) -> dict[str, object]:
        self.request_sequence += 1
        packet = {
            "protocol": EXTERNAL.PROTOCOL,
            "kind": "request",
            "sequence": self.request_sequence,
            "capability": self.broker.capability,
            "operation": operation,
            "payload": payload,
        }
        self.client.send(json.dumps(packet, separators=(",", ":")).encode())
        self.broker.poll()
        response = json.loads(self.client.recv(EXTERNAL.MAX_PACKET_BYTES))
        if expect_ok:
            self.assertTrue(response["ok"], response)
        return response

    def replace_neutral(
        self,
        *,
        connected: bool = False,
    ) -> EXTERNAL.ExternalInputToken:
        mapping = EXTERNAL.ExternalInputState.neutral().to_mapping()
        mapping["gamepad"]["connected"] = connected
        response = self.request(
            "input.replace",
            {"lease_id": self.lease_id, "state": mapping},
        )
        return EXTERNAL.ExternalInputToken.from_mapping(
            response["data"]["input_token"]
        )

    def observe(
        self,
        *,
        published: bool = True,
        interlock_reason: str | None = None,
    ) -> bool:
        state, token = self.broker.sample_with_token()
        effective, frame = self.gate.prepare(state, token)
        self.assertIsNotNone(frame)
        assert frame is not None
        self.provider_sequence += 1
        updated = self.gate.observe_published(
            frame,
            sequence=self.provider_sequence,
            published=published,
            interlock_reason=interlock_reason,
        )
        self.last_effective_state = effective
        return updated

    def qualify(self) -> EXTERNAL.ExternalInputToken:
        self.assertTrue(self.observe())
        self.assertTrue(self.observe())
        token = self.broker.input_token
        self.assertIsNotNone(token)
        self.assertTrue(self.broker.provider_gate.ready)
        assert token is not None
        return token

    def snapshot_for_external_state(
        self,
        state: EXTERNAL.ExternalInputState,
        *,
        sequence: int,
    ) -> CORE.InputSnapshot:
        keyboard, gamepad = MODULE.external_input_samples(
            state,
            focus=MODULE.KeyboardMouseSample(
                focused=True,
                focus_title="Matrix",
                focus_pid=42,
            ),
            look_button="left",
        )
        return MODULE.build_snapshot(
            sequence=sequence,
            timestamp_monotonic_s=self.clock_value,
            keyboard=keyboard,
            gamepad=gamepad,
            input_source=MODULE.external_frame_input_source(
                state,
                configured_source="auto",
            ),
            camera_yaw_rad=0.0,
            camera_available=True,
        )

    def exercise_source_interlocked_publish(
        self,
        mapping: dict[str, object],
        *,
        configured_source: str,
        command: str,
    ) -> dict[str, object]:
        state = EXTERNAL.ExternalInputState.from_mapping(mapping)
        current_token = self.broker.input_token
        self.assertIsNotNone(current_token)
        effective, frame = self.gate.prepare(state, current_token)
        self.assertIsNotNone(frame)
        assert frame is not None
        gated, early_reason = MODULE.apply_external_source_gate(
            effective,
            frame,
            configured_source=configured_source,
        )
        self.assertIsNotNone(early_reason)
        self.assertEqual(gated, EXTERNAL.ExternalInputState.neutral())

        keyboard, gamepad = MODULE.external_input_samples(
            gated,
            focus=MODULE.KeyboardMouseSample(
                focused=True,
                focus_title="Matrix",
                focus_pid=42,
            ),
            look_button="left",
        )
        initial_yaw = 0.625
        tracker = MODULE.CameraYawTracker(
            initial_yaw,
            mouse_radians_per_pixel=0.1,
            gamepad_radians_per_second=2.0,
        )
        heading = tracker.update(
            dt=0.5,
            mouse_dx=keyboard.mouse_dx,
            gamepad_look_yaw=gamepad.look_yaw,
        )
        self.assertEqual(heading, initial_yaw)
        snapshot = MODULE.build_snapshot(
            sequence=self.provider_sequence + 1,
            timestamp_monotonic_s=self.clock_value,
            keyboard=keyboard,
            gamepad=gamepad,
            input_source=MODULE.external_frame_input_source(
                gated,
                configured_source=configured_source,
            ),
            camera_yaw_rad=heading,
            camera_available=True,
        )
        final_reason = MODULE.external_provider_publish_interlock_reason(
            frame,
            configured_source=configured_source,
            physical_focused=True,
            camera_dragging=False,
            camera_available=True,
            input_available=True,
            gamepad_connected_edge=False,
            calibration_interlock_active=False,
        )
        self.assertEqual(final_reason, early_reason)
        publish_snapshot = MODULE.apply_external_publish_interlock(
            snapshot,
            frame,
            final_reason,
        )

        socket_path = Path(self.temporary.name) / "interlocked-publish.sock"
        with CORE.UnixSeqpacketInputServer(socket_path) as server:
            publisher = MODULE.UnixSeqpacketPublisher(socket_path)
            try:
                self.assertTrue(
                    publisher.send(publish_snapshot, now=self.clock_value)
                )
                with server.accept(timeout_s=1.0) as connection:
                    received = connection.receive(timeout_s=1.0)
            finally:
                publisher.close()
        self.assertFalse(received.focused)
        self.assertFalse(any(received.keys.to_mapping().values()))
        self.assertEqual(
            (received.move_stick.right, received.move_stick.forward),
            (0.0, 0.0),
        )
        self.assertEqual(received.camera_yaw_rad, initial_yaw)

        command_client = MODULE.GameCommandClient(None)
        self.addCleanup(command_client.close)
        self.assertTrue(
            command_client.submit_external(
                command,
                calibration_active=False,
                neutral_frame_ready=False,
                restart_requested=False,
                input_modifier=lambda _command: (frame.token, None),
            )
        )
        self.assertTrue(
            command_client.resolve_external_input_publish(
                sampled_token=frame.token,
                current_token=frame.token,
                authority_active=True,
                published=True,
                locomotion_admitted=frame.locomotion_admitted,
                interlock_reason=final_reason,
            )
        )
        self.assertFalse(command_client.ok)
        self.assertEqual(command_client.code, "E_INPUT_INTERLOCK")
        return {
            "state": state,
            "effective": effective,
            "frame": frame,
            "keyboard": keyboard,
            "gamepad": gamepad,
            "heading": heading,
            "received": received,
            "reason": final_reason,
        }

    def test_connected_edge_then_two_successful_neutral_frames_are_required(self) -> None:
        token = self.replace_neutral(connected=True)
        self.assertTrue(
            self.observe(interlock_reason="gamepad_connected_edge")
        )
        self.assertEqual(self.broker.provider_gate.input_token, token)
        self.assertEqual(self.broker.provider_gate.neutral_sent_count, 0)
        self.assertFalse(self.broker.provider_gate.ready)
        self.assertTrue(self.observe())
        self.assertEqual(self.broker.provider_gate.neutral_sent_count, 1)
        self.assertFalse(self.broker.provider_gate.ready)
        self.assertTrue(self.observe())
        self.assertTrue(self.broker.provider_gate.ready)
        self.assertEqual(self.broker.provider_gate.neutral_sent_count, 2)
        self.assertIsNone(self.broker.provider_gate.last_interlock_reason)

    def test_final_snapshot_interlocks_are_typed_and_sticky(self) -> None:
        for reason in (
            "camera_unavailable",
            "calibration_interlock",
        ):
            with self.subTest(reason=reason):
                self.replace_neutral()
                self.assertTrue(self.observe(interlock_reason=reason))
                self.assertEqual(self.broker.provider_gate.phase, "interlocked")
                self.assertEqual(
                    self.broker.provider_gate.last_interlock_reason,
                    reason,
                )
                self.assertTrue(self.observe())
                self.assertEqual(self.broker.provider_gate.phase, "interlocked")

    def test_physical_focus_loss_latches_authority_but_calibration_does_not(self) -> None:
        self.replace_neutral()
        self.assertTrue(self.observe(interlock_reason="calibration_interlock"))
        self.assertIsNone(
            self.broker.telemetry()["fatal_authority_reason"]
        )
        renewed = self.request(
            "lease.renew",
            {"lease_id": self.lease_id},
        )
        self.assertTrue(renewed["ok"])
        queued = self.request(
            "command.submit",
            {"lease_id": self.lease_id, "command": "/tp @s ~ ~ ~"},
        )
        self.assertTrue(queued["ok"])
        noninput = self.broker.drain_commands(limit=1)[0]
        self.broker.complete_command(
            noninput,
            {
                "ok": True,
                "outcome_unknown": False,
                "code": "OK_TELEPORT_RESTART",
                "message": "saved",
            },
        )
        completed = self.request(
            "command.result",
            {"command_id": queued["data"]["command_id"]},
        )["data"]
        self.assertTrue(completed["terminal"])
        self.assertEqual(completed["state"], "completed")

        # A fresh authority demonstrates the fatal focus path independently
        # from the intentionally sticky calibration gate above.
        self.request("lease.release", {"lease_id": self.lease_id})
        acquired = self.request("lease.acquire", {})
        self.lease_id = acquired["data"]["lease_id"]
        self.replace_neutral()
        self.assertTrue(self.observe(interlock_reason="physical_focus_lost"))
        self.assertEqual(
            self.broker.telemetry()["fatal_authority_reason"],
            "physical_focus_lost",
        )
        self.assertEqual(
            self.broker.sample(),
            EXTERNAL.ExternalInputState.neutral(),
        )

    def test_failed_send_resets_count_without_counting_as_a_frame(self) -> None:
        self.replace_neutral()
        self.assertTrue(self.observe())
        self.assertEqual(self.broker.provider_gate.neutral_sent_count, 1)
        self.assertTrue(self.observe(published=False))
        self.assertEqual(self.broker.provider_gate.phase, "awaiting_neutral")
        self.assertEqual(self.broker.provider_gate.neutral_sent_count, 0)
        self.assertEqual(
            self.broker.provider_gate.last_interlock_reason,
            "publisher_send_failed",
        )
        self.assertTrue(self.observe())
        self.assertFalse(self.broker.provider_gate.ready)
        self.assertTrue(self.observe())
        self.assertTrue(self.broker.provider_gate.ready)
        self.assertIsNone(self.broker.provider_gate.last_interlock_reason)

    def test_duplicate_or_regressed_sequence_cannot_count_as_two_frames(self) -> None:
        self.replace_neutral()
        state, token = self.broker.sample_with_token()
        _effective, frame = self.gate.prepare(state, token)
        self.assertIsNotNone(frame)
        assert frame is not None
        self.provider_sequence += 1
        sequence = self.provider_sequence
        self.assertTrue(
            self.gate.observe_published(
                frame,
                sequence=sequence,
                published=True,
                interlock_reason=None,
            )
        )
        self.assertEqual(self.broker.provider_gate.neutral_sent_count, 1)
        self.assertFalse(
            self.gate.observe_published(
                frame,
                sequence=sequence,
                published=True,
                interlock_reason=None,
            )
        )
        self.assertFalse(
            self.gate.observe_published(
                frame,
                sequence=sequence - 1,
                published=True,
                interlock_reason=None,
            )
        )
        self.assertEqual(self.broker.provider_gate.neutral_sent_count, 1)
        self.provider_sequence += 1
        self.assertTrue(
            self.gate.observe_published(
                frame,
                sequence=self.provider_sequence,
                published=True,
                interlock_reason=None,
            )
        )
        self.assertTrue(self.broker.provider_gate.ready)

    def test_stale_sample_cannot_ack_revision_bumped_by_data_modify(self) -> None:
        self.replace_neutral()
        _state, sampled_token = self.broker.sample_with_token()
        self.assertIsNotNone(sampled_token)
        _effective, stale_frame = self.gate.prepare(_state, sampled_token)
        self.assertIsNotNone(stale_frame)
        current = self.broker.apply_data_modify(
            "control.input.keyboard.ctrl",
            True,
        )
        assert stale_frame is not None
        self.provider_sequence += 1
        self.assertFalse(
            self.gate.observe_published(
                stale_frame,
                sequence=self.provider_sequence,
                published=True,
                interlock_reason=None,
            )
        )
        self.assertEqual(self.broker.provider_gate.input_token, current)
        self.assertEqual(self.broker.provider_gate.neutral_sent_count, 0)
        self.assertFalse(self.broker.provider_gate.ready)

    def test_stale_failed_send_invalidates_ready_successor_revision(self) -> None:
        self.qualify()
        state, sampled_token = self.broker.sample_with_token()
        _effective, stale_frame = self.gate.prepare(state, sampled_token)
        self.assertIsNotNone(stale_frame)
        successor = self.broker.apply_data_modify(
            "control.input.keyboard.w",
            True,
        )
        self.assertTrue(self.broker.provider_gate.ready)
        assert stale_frame is not None
        self.provider_sequence += 1
        self.assertTrue(
            self.gate.observe_published(
                stale_frame,
                sequence=self.provider_sequence,
                published=False,
                interlock_reason=None,
            )
        )
        gate = self.broker.provider_gate
        self.assertEqual(gate.input_token, successor)
        self.assertEqual(gate.phase, "awaiting_neutral")
        self.assertFalse(gate.ready)
        self.assertEqual(gate.neutral_sent_count, 0)
        self.assertIsNone(gate.qualified_from_revision)
        self.assertEqual(gate.last_interlock_reason, "publisher_send_failed")
        current_state, current_token = self.broker.sample_with_token()
        effective, _frame = self.gate.prepare(current_state, current_token)
        self.assertFalse(current_state.locomotion_neutral)
        self.assertTrue(effective.locomotion_neutral)

    def test_stale_failure_then_exact_clamped_frame_cannot_complete_input(self) -> None:
        self.qualify()
        stale_state, stale_token = self.broker.sample_with_token()
        _effective, stale_frame = self.gate.prepare(stale_state, stale_token)
        self.assertIsNotNone(stale_frame)
        successor = self.broker.apply_data_modify(
            "control.input.keyboard.w",
            True,
        )
        command_client = MODULE.GameCommandClient(None)
        self.addCleanup(command_client.close)
        self.assertTrue(
            command_client.submit_external(
                "/data modify entity @s "
                "control.input.keyboard.w set value true",
                calibration_active=False,
                neutral_frame_ready=False,
                restart_requested=False,
                input_modifier=lambda _command: (successor, None),
            )
        )
        assert stale_frame is not None
        self.provider_sequence += 1
        self.assertTrue(
            self.gate.observe_published(
                stale_frame,
                sequence=self.provider_sequence,
                published=False,
                interlock_reason=None,
            )
        )
        current_state, current_token = self.broker.sample_with_token()
        _clamped, exact_frame = self.gate.prepare(current_state, current_token)
        self.assertIsNotNone(exact_frame)
        assert exact_frame is not None
        self.assertFalse(exact_frame.locomotion_admitted)
        self.assertTrue(
            command_client.resolve_external_input_publish(
                sampled_token=exact_frame.token,
                current_token=current_token,
                authority_active=True,
                published=True,
                locomotion_admitted=exact_frame.locomotion_admitted,
                interlock_reason=None,
            )
        )
        self.assertEqual(command_client.code, "E_INPUT_INTERLOCK")
        self.assertFalse(command_client.ok)

    def test_data_modify_receipt_stays_nonterminal_until_exact_publish(self) -> None:
        self.qualify()
        queued = self.request(
            "command.submit",
            {
                "lease_id": self.lease_id,
                "command": (
                    "/data modify entity @s "
                    "control.input.keyboard.w set value true"
                ),
            },
        )
        external_command = self.broker.drain_commands(limit=1)[0]
        stale_state, stale_token = self.broker.sample_with_token()
        _effective, stale_frame = self.gate.prepare(stale_state, stale_token)
        self.assertIsNotNone(stale_frame)
        command_client = MODULE.GameCommandClient(None)
        self.addCleanup(command_client.close)

        def modify(command: MC_COMMANDS.DataModifyInput):
            token = self.broker.apply_data_modify(command.path, command.value)
            return token, None

        self.assertTrue(
            command_client.submit_external(
                external_command.command,
                calibration_active=False,
                neutral_frame_ready=False,
                restart_requested=False,
                input_modifier=modify,
            )
        )
        pending_token = self.broker.input_token
        self.assertIsNotNone(pending_token)
        before = self.request(
            "command.result",
            {"command_id": queued["data"]["command_id"]},
        )["data"]
        self.assertEqual(before["state"], "admitted")
        self.assertFalse(before["terminal"])

        assert stale_frame is not None
        self.assertFalse(
            command_client.resolve_external_input_publish(
                sampled_token=stale_frame.token,
                current_token=pending_token,
                authority_active=True,
                published=True,
                locomotion_admitted=stale_frame.locomotion_admitted,
                interlock_reason=None,
            )
        )
        still_pending = self.request(
            "command.result",
            {"command_id": queued["data"]["command_id"]},
        )["data"]
        self.assertFalse(still_pending["terminal"])

        exact_state, exact_token = self.broker.sample_with_token()
        _effective, exact_frame = self.gate.prepare(exact_state, exact_token)
        self.assertIsNotNone(exact_frame)
        assert exact_frame is not None
        self.assertTrue(exact_frame.locomotion_admitted)
        self.assertTrue(
            command_client.resolve_external_input_publish(
                sampled_token=exact_frame.token,
                current_token=exact_token,
                authority_active=True,
                published=True,
                locomotion_admitted=exact_frame.locomotion_admitted,
                interlock_reason=None,
            )
        )
        self.broker.complete_command(
            external_command,
            command_client.mapping(),
        )
        terminal = self.request(
            "command.result",
            {"command_id": queued["data"]["command_id"]},
        )["data"]
        self.assertTrue(terminal["terminal"])
        self.assertEqual(terminal["state"], "completed")
        self.assertEqual(
            terminal["result"]["code"],
            "OK_DATA_INPUT_MODIFIED",
        )

    def test_stale_success_does_not_clear_ready_successor_revision(self) -> None:
        self.qualify()
        state, sampled_token = self.broker.sample_with_token()
        _effective, stale_frame = self.gate.prepare(state, sampled_token)
        self.assertIsNotNone(stale_frame)
        successor = self.broker.apply_data_modify(
            "control.input.keyboard.w",
            True,
        )
        before = self.broker.provider_gate
        assert stale_frame is not None
        self.provider_sequence += 1
        self.assertFalse(
            self.gate.observe_published(
                stale_frame,
                sequence=self.provider_sequence,
                published=True,
                interlock_reason=None,
            )
        )
        self.assertEqual(self.broker.provider_gate, before)
        self.assertEqual(self.broker.provider_gate.input_token, successor)
        self.assertTrue(self.broker.provider_gate.ready)

    def test_expired_publish_boundary_replaces_stale_motion_and_revokes_pending(self) -> None:
        self.qualify()
        self.broker.apply_data_modify(
            "control.input.keyboard.w",
            True,
            now=self.clock_value,
        )
        stale_state, stale_token = self.broker.sample_with_token(
            now=self.clock_value
        )
        effective, stale_frame = self.gate.prepare(stale_state, stale_token)
        self.assertIsNotNone(stale_frame)
        assert stale_frame is not None
        self.assertTrue(effective.keyboard["w"])

        command_client = MODULE.GameCommandClient(None)
        self.addCleanup(command_client.close)
        self.assertTrue(
            command_client.submit_external(
                "/data modify entity @s "
                "control.input.keyboard.d set value true",
                calibration_active=False,
                neutral_frame_ready=False,
                restart_requested=False,
                input_modifier=lambda command: (
                    self.broker.apply_data_modify(
                        command.path,
                        command.value,
                        now=self.clock_value,
                    ),
                    None,
                ),
            )
        )
        stale_snapshot = self.snapshot_for_external_state(
            effective,
            sequence=self.provider_sequence + 1,
        )
        self.assertTrue(stale_snapshot.keys.w)

        self.clock_value += 0.151
        boundary = MODULE.external_publish_boundary(
            self.broker,
            stale_frame,
            stale_snapshot,
            now=self.clock_value,
        )
        self.assertIsNone(boundary.current_token)
        self.assertFalse(boundary.exact_revision)
        self.assertFalse(boundary.snapshot.focused)
        self.assertFalse(any(boundary.snapshot.keys.to_mapping().values()))
        self.assertEqual(
            (
                boundary.snapshot.move_stick.right,
                boundary.snapshot.move_stick.forward,
            ),
            (0.0, 0.0),
        )

        # Model a successful send of the replacement safety-neutral packet.
        # It is not exact proof for the pending revision.
        self.provider_sequence += 1
        self.assertFalse(
            self.gate.observe_published(
                stale_frame,
                sequence=self.provider_sequence,
                published=True,
                interlock_reason=None,
            )
        )
        self.assertTrue(
            command_client.resolve_external_input_publish(
                sampled_token=stale_frame.token,
                current_token=boundary.current_token,
                authority_active=False,
                published=False,
                locomotion_admitted=stale_frame.locomotion_admitted,
                interlock_reason=None,
            )
        )
        self.assertFalse(command_client.ok)
        self.assertEqual(command_client.code, "E_AUTHORITY_REVOKED")

    def test_same_frame_revision_bump_sends_neutral_then_exact_publish_is_ok(self) -> None:
        self.qualify()
        self.broker.apply_data_modify(
            "control.input.keyboard.w",
            True,
            now=self.clock_value,
        )
        stale_state, stale_token = self.broker.sample_with_token(
            now=self.clock_value
        )
        stale_effective, stale_frame = self.gate.prepare(
            stale_state,
            stale_token,
        )
        self.assertIsNotNone(stale_frame)
        assert stale_frame is not None

        command_client = MODULE.GameCommandClient(None)
        self.addCleanup(command_client.close)
        self.assertTrue(
            command_client.submit_external(
                "/data modify entity @s "
                "control.input.keyboard.d set value true",
                calibration_active=False,
                neutral_frame_ready=False,
                restart_requested=False,
                input_modifier=lambda command: (
                    self.broker.apply_data_modify(
                        command.path,
                        command.value,
                        now=self.clock_value,
                    ),
                    None,
                ),
            )
        )
        pending_token = self.broker.input_token
        self.assertIsNotNone(pending_token)
        self.assertNotEqual(stale_frame.token, pending_token)

        stale_boundary = MODULE.external_publish_boundary(
            self.broker,
            stale_frame,
            self.snapshot_for_external_state(
                stale_effective,
                sequence=self.provider_sequence + 1,
            ),
            now=self.clock_value,
        )
        self.assertFalse(stale_boundary.exact_revision)
        self.assertFalse(stale_boundary.snapshot.focused)
        self.provider_sequence += 1
        self.assertFalse(
            self.gate.observe_published(
                stale_frame,
                sequence=self.provider_sequence,
                # The safety-neutral socket write succeeded, so the stale R1
                # callback must not erase R2's inherited ready proof.
                published=True,
                interlock_reason=None,
            )
        )
        self.assertTrue(self.broker.provider_gate.ready)
        self.assertFalse(
            command_client.resolve_external_input_publish(
                sampled_token=stale_frame.token,
                current_token=stale_boundary.current_token,
                authority_active=True,
                published=False,
                locomotion_admitted=stale_frame.locomotion_admitted,
                interlock_reason=None,
            )
        )

        exact_state, exact_token = self.broker.sample_with_token(
            now=self.clock_value
        )
        exact_effective, exact_frame = self.gate.prepare(exact_state, exact_token)
        self.assertIsNotNone(exact_frame)
        assert exact_frame is not None
        self.clock_value += 0.05
        exact_boundary = MODULE.external_publish_boundary(
            self.broker,
            exact_frame,
            self.snapshot_for_external_state(
                exact_effective,
                sequence=self.provider_sequence + 1,
            ),
            now=self.clock_value,
        )
        self.assertTrue(exact_boundary.exact_revision)
        self.assertTrue(exact_boundary.snapshot.focused)
        self.provider_sequence += 1
        self.assertTrue(
            self.gate.observe_published(
                exact_frame,
                sequence=self.provider_sequence,
                published=True,
                interlock_reason=None,
            )
        )

        # Freeze the send-boundary decision.  Even if scheduling advances the
        # clock past deadman immediately after send, receipt outcome cannot
        # flip to revoked by a second post-send authority read.
        self.clock_value += 0.20
        self.assertTrue(
            command_client.resolve_external_input_publish(
                sampled_token=exact_frame.token,
                current_token=exact_boundary.current_token,
                authority_active=exact_boundary.current_token is not None,
                published=True,
                locomotion_admitted=exact_frame.locomotion_admitted,
                interlock_reason=None,
            )
        )
        self.assertTrue(command_client.ok)
        self.assertEqual(command_client.code, "OK_DATA_INPUT_MODIFIED")
        self.assertIsNone(
            self.broker.publish_boundary_token(now=self.clock_value)
        )
        self.assertTrue(command_client.ok)

    def test_stale_sticky_interlock_invalidates_ready_successor_revision(self) -> None:
        self.qualify()
        state, sampled_token = self.broker.sample_with_token()
        _effective, stale_frame = self.gate.prepare(state, sampled_token)
        self.assertIsNotNone(stale_frame)
        successor = self.broker.apply_data_modify(
            "control.input.keyboard.w",
            True,
        )
        assert stale_frame is not None
        self.provider_sequence += 1
        self.assertTrue(
            self.gate.observe_published(
                stale_frame,
                sequence=self.provider_sequence,
                published=True,
                interlock_reason="camera_unavailable",
            )
        )
        gate = self.broker.provider_gate
        self.assertEqual(gate.input_token, successor)
        self.assertEqual(gate.phase, "interlocked")
        self.assertFalse(gate.ready)
        self.assertEqual(gate.last_interlock_reason, "camera_unavailable")

    def test_stale_connect_edge_rearms_ready_successor_revision(self) -> None:
        self.qualify()
        state, sampled_token = self.broker.sample_with_token()
        _effective, stale_frame = self.gate.prepare(state, sampled_token)
        self.assertIsNotNone(stale_frame)
        successor = self.broker.apply_data_modify(
            "control.input.keyboard.w",
            True,
        )
        assert stale_frame is not None
        self.provider_sequence += 1
        self.assertTrue(
            self.gate.observe_published(
                stale_frame,
                sequence=self.provider_sequence,
                published=True,
                interlock_reason="gamepad_connected_edge",
            )
        )
        gate = self.broker.provider_gate
        self.assertEqual(gate.input_token, successor)
        self.assertEqual(gate.phase, "awaiting_neutral")
        self.assertFalse(gate.ready)
        self.assertEqual(gate.last_interlock_reason, "gamepad_connected_edge")

    def test_stale_failure_is_isolated_by_authority_and_revision(self) -> None:
        current = self.replace_neutral()
        before = self.broker.provider_gate
        cases = {
            "different_lease": EXTERNAL.ExternalInputToken(
                lease_id=(
                    "0" * 32
                    if current.lease_id != "0" * 32
                    else "1" * 32
                ),
                authority_epoch=current.authority_epoch,
                input_revision=current.input_revision - 1,
            ),
            "different_epoch": EXTERNAL.ExternalInputToken(
                lease_id=current.lease_id,
                authority_epoch=current.authority_epoch + 1,
                input_revision=current.input_revision - 1,
            ),
            "non_successor_revision": EXTERNAL.ExternalInputToken(
                lease_id=current.lease_id,
                authority_epoch=current.authority_epoch,
                input_revision=current.input_revision + 1,
            ),
        }
        for label, token in cases.items():
            with self.subTest(label=label):
                frame = MODULE.ExternalProviderGateFrame(
                    token=token,
                    requested_neutral=True,
                    requested_device=None,
                    locomotion_admitted=True,
                )
                self.provider_sequence += 1
                self.assertFalse(
                    self.gate.observe_published(
                        frame,
                        sequence=self.provider_sequence,
                        published=False,
                        interlock_reason="camera_unavailable",
                    )
                )
                self.assertEqual(self.broker.provider_gate, before)

    def test_ready_motion_is_clamped_after_midhold_camera_interlock(self) -> None:
        proof = self.qualify()
        moving = EXTERNAL.ExternalInputState.neutral().to_mapping()
        moving["keyboard"]["w"] = True
        response = self.request(
            "input.replace",
            {
                "lease_id": self.lease_id,
                "state": moving,
                "qualified_token": proof.to_mapping(),
            },
        )
        active = EXTERNAL.ExternalInputToken.from_mapping(
            response["data"]["input_token"]
        )
        self.assertEqual(self.broker.provider_gate.input_token, active)
        self.assertTrue(self.observe(interlock_reason="camera_unavailable"))
        self.assertEqual(self.broker.provider_gate.phase, "interlocked")
        state, token = self.broker.sample_with_token()
        effective, frame = self.gate.prepare(state, token)
        self.assertIsNotNone(frame)
        self.assertFalse(state.locomotion_neutral)
        self.assertTrue(effective.locomotion_neutral)

    def test_configured_source_rejects_the_other_virtual_device(self) -> None:
        self.replace_neutral(connected=True)
        state, token = self.broker.sample_with_token()
        _effective, gamepad_frame = self.gate.prepare(state, token)
        self.assertIsNotNone(gamepad_frame)
        assert gamepad_frame is not None
        self.assertEqual(gamepad_frame.requested_device, "gamepad")
        self.assertEqual(
            MODULE.external_provider_source_interlock_reason(
                gamepad_frame,
                configured_source="keyboard",
            ),
            "input_source_rejects_gamepad",
        )
        self.assertIsNone(
            MODULE.external_provider_source_interlock_reason(
                gamepad_frame,
                configured_source="auto",
            )
        )
        self.assertTrue(
            self.observe(interlock_reason="input_source_rejects_gamepad")
        )
        self.assertEqual(self.broker.provider_gate.phase, "interlocked")

        self.replace_neutral()
        proof = self.qualify()
        moving = EXTERNAL.ExternalInputState.neutral().to_mapping()
        moving["keyboard"]["w"] = True
        self.request(
            "input.replace",
            {
                "lease_id": self.lease_id,
                "state": moving,
                "qualified_token": proof.to_mapping(),
            },
        )
        state, token = self.broker.sample_with_token()
        _effective, keyboard_frame = self.gate.prepare(state, token)
        self.assertIsNotNone(keyboard_frame)
        assert keyboard_frame is not None
        self.assertEqual(keyboard_frame.requested_device, "keyboard")
        self.assertEqual(
            MODULE.external_provider_source_interlock_reason(
                keyboard_frame,
                configured_source="gamepad",
            ),
            "input_source_rejects_keyboard",
        )
        self.assertIsNone(
            MODULE.external_provider_source_interlock_reason(
                keyboard_frame,
                configured_source="auto",
            )
        )
        self.assertTrue(
            self.observe(interlock_reason="input_source_rejects_keyboard")
        )
        self.assertEqual(self.broker.provider_gate.phase, "interlocked")

    def test_source_gate_covers_nonlocomotion_keyboard_and_mouse_input(self) -> None:
        cases = {
            "q": ("keyboard", "q", True),
            "mouse_button": ("mouse", "left", True),
            "mouse_delta": ("mouse", "dx", 12.0),
        }
        for label, (family, name, value) in cases.items():
            with self.subTest(label=label):
                mapping = EXTERNAL.ExternalInputState.neutral().to_mapping()
                if family == "keyboard":
                    mapping["keyboard"][name] = value
                elif name == "dx":
                    mapping["mouse"]["dx"] = value
                else:
                    mapping["mouse"]["buttons"][name] = value
                response = self.request(
                    "input.replace",
                    {"lease_id": self.lease_id, "state": mapping},
                )
                token = EXTERNAL.ExternalInputToken.from_mapping(
                    response["data"]["input_token"]
                )
                state, sampled_token = self.broker.sample_with_token()
                self.assertEqual(sampled_token, token)
                _effective, frame = self.gate.prepare(state, sampled_token)
                self.assertIsNotNone(frame)
                assert frame is not None
                self.assertEqual(frame.requested_device, "keyboard")
                self.assertEqual(
                    MODULE.external_provider_source_interlock_reason(
                        frame,
                        configured_source="gamepad",
                    ),
                    "input_source_rejects_keyboard",
                )

    def test_device_claim_is_shared_for_warmup_keyboard_and_mixed_input(self) -> None:
        current_token = self.broker.input_token
        self.assertIsNotNone(current_token)

        connected_neutral = EXTERNAL.ExternalInputState.neutral().to_mapping()
        connected_neutral["gamepad"]["connected"] = True
        neutral_state = EXTERNAL.ExternalInputState.from_mapping(connected_neutral)
        self.assertEqual(
            MODULE.external_active_input_device(neutral_state),
            "gamepad",
        )
        self.assertEqual(
            MODULE.external_frame_input_source(
                neutral_state,
                configured_source="auto",
            ),
            "gamepad",
        )
        _effective, neutral_frame = self.gate.prepare(
            neutral_state,
            current_token,
        )
        self.assertIsNotNone(neutral_frame)
        assert neutral_frame is not None
        self.assertEqual(neutral_frame.requested_device, "gamepad")

        keyboard_mapping = connected_neutral
        keyboard_mapping["keyboard"]["q"] = True
        keyboard_state = EXTERNAL.ExternalInputState.from_mapping(keyboard_mapping)
        self.assertEqual(
            MODULE.external_active_input_device(keyboard_state),
            "keyboard",
        )
        _effective, keyboard_frame = self.gate.prepare(
            keyboard_state,
            current_token,
        )
        self.assertIsNotNone(keyboard_frame)
        assert keyboard_frame is not None
        self.assertEqual(keyboard_frame.requested_device, "keyboard")

        mixed_mapping = keyboard_state.to_mapping()
        mixed_mapping["gamepad"]["axes"]["look_yaw"] = 0.5
        mixed_state = EXTERNAL.ExternalInputState.from_mapping(mixed_mapping)
        self.assertEqual(MODULE.external_active_input_device(mixed_state), "mixed")
        _effective, mixed_frame = self.gate.prepare(mixed_state, current_token)
        self.assertIsNotNone(mixed_frame)
        assert mixed_frame is not None
        self.assertEqual(mixed_frame.requested_device, "mixed")
        self.assertFalse(mixed_frame.locomotion_admitted)
        for configured_source in ("auto", "keyboard", "gamepad"):
            with self.subTest(configured_source=configured_source):
                self.assertEqual(
                    MODULE.external_provider_source_interlock_reason(
                        mixed_frame,
                        configured_source=configured_source,
                    ),
                    "input_source_mixed",
                )

    def test_mixed_q_v_and_gamepad_look_are_neutral_before_publish(self) -> None:
        mapping = EXTERNAL.ExternalInputState.neutral().to_mapping()
        mapping["keyboard"]["q"] = True
        mapping["keyboard"]["v"] = True
        mapping["gamepad"]["connected"] = True
        mapping["gamepad"]["axes"]["look_yaw"] = 0.75
        result = self.exercise_source_interlocked_publish(
            mapping,
            configured_source="auto",
            command=(
                "/data modify entity @s "
                "control.input.keyboard.q set value true"
            ),
        )
        frame = result["frame"]
        assert isinstance(frame, MODULE.ExternalProviderGateFrame)
        self.assertEqual(frame.requested_device, "mixed")
        self.assertEqual(result["reason"], "input_source_mixed")
        effective = result["effective"]
        assert isinstance(effective, EXTERNAL.ExternalInputState)
        self.assertTrue(effective.keyboard["q"])
        self.assertTrue(effective.keyboard["v"])
        self.assertEqual(effective.gamepad_axes["look_yaw"], 0.75)
        keyboard = result["keyboard"]
        gamepad = result["gamepad"]
        assert isinstance(keyboard, MODULE.KeyboardMouseSample)
        assert isinstance(gamepad, MODULE.GamepadSample)
        self.assertFalse(keyboard.q)
        self.assertFalse(keyboard.v)
        self.assertEqual(gamepad.look_yaw, 0.0)

    def test_configured_mismatch_mouse_delta_and_button_are_neutral(self) -> None:
        mapping = EXTERNAL.ExternalInputState.neutral().to_mapping()
        mapping["mouse"]["buttons"]["left"] = True
        mapping["mouse"]["dx"] = 18.0
        result = self.exercise_source_interlocked_publish(
            mapping,
            configured_source="gamepad",
            command=(
                "/data modify entity @s "
                "control.input.mouse.left set value true"
            ),
        )
        frame = result["frame"]
        assert isinstance(frame, MODULE.ExternalProviderGateFrame)
        self.assertEqual(frame.requested_device, "keyboard")
        self.assertEqual(result["reason"], "input_source_rejects_keyboard")
        effective = result["effective"]
        assert isinstance(effective, EXTERNAL.ExternalInputState)
        self.assertTrue(effective.mouse_buttons["left"])
        self.assertEqual(effective.mouse_dx, 18.0)
        keyboard = result["keyboard"]
        assert isinstance(keyboard, MODULE.KeyboardMouseSample)
        self.assertFalse(keyboard.camera_dragging)
        self.assertEqual(keyboard.mouse_dx, 0.0)

    def test_focus_loss_outranks_source_and_calibration_and_latches_deadman(self) -> None:
        connected_neutral = EXTERNAL.ExternalInputState.neutral().to_mapping()
        connected_neutral["gamepad"]["connected"] = True
        response = self.request(
            "input.replace",
            {"lease_id": self.lease_id, "state": connected_neutral},
        )
        token = EXTERNAL.ExternalInputToken.from_mapping(
            response["data"]["input_token"]
        )
        state, sampled_token = self.broker.sample_with_token()
        self.assertEqual(sampled_token, token)
        _effective, frame = self.gate.prepare(state, sampled_token)
        self.assertIsNotNone(frame)
        assert frame is not None
        self.assertEqual(frame.requested_device, "gamepad")

        for calibration_active in (False, True):
            with self.subTest(calibration_active=calibration_active):
                self.assertEqual(
                    MODULE.external_provider_publish_interlock_reason(
                        frame,
                        configured_source="keyboard",
                        physical_focused=False,
                        camera_dragging=False,
                        camera_available=True,
                        input_available=True,
                        gamepad_connected_edge=False,
                        calibration_interlock_active=calibration_active,
                    ),
                    "physical_focus_lost",
                )

        self.provider_sequence += 1
        self.assertTrue(
            self.gate.observe_published(
                frame,
                sequence=self.provider_sequence,
                published=True,
                interlock_reason="physical_focus_lost",
            )
        )
        blocked = self.request(
            "command.submit",
            {"lease_id": self.lease_id, "command": "/tp @s ~ ~ ~"},
            expect_ok=False,
        )
        self.assertFalse(blocked["ok"])
        self.assertEqual(blocked["code"], "E_AUTHORITY_REVOKED")

        self.clock_value += 0.10
        self.assertTrue(
            self.request(
                "lease.renew",
                {"lease_id": self.lease_id},
            )["ok"]
        )
        self.clock_value += 0.051
        self.assertIsNone(
            self.broker.publish_boundary_token(now=self.clock_value)
        )
        self.assertEqual(self.broker.deadman_stops, 1)

    def test_interlock_reason_uses_final_publish_preconditions(self) -> None:
        base = {
            "physical_focused": True,
            "camera_dragging": False,
            "camera_available": True,
            "input_available": True,
            "gamepad_connected_edge": False,
            "calibration_interlock_active": False,
        }
        cases = {
            "physical_focused": "physical_focus_lost",
            "camera_available": "camera_unavailable",
            "input_available": "input_unavailable",
            "gamepad_connected_edge": "gamepad_connected_edge",
            "calibration_interlock_active": "calibration_interlock",
        }
        for field, reason in cases.items():
            with self.subTest(field=field):
                values = dict(base)
                values[field] = not values[field]
                self.assertEqual(
                    MODULE.external_provider_interlock_reason(**values),
                    reason,
                )
        focus_and_calibration = dict(base)
        focus_and_calibration["physical_focused"] = False
        focus_and_calibration["calibration_interlock_active"] = True
        self.assertEqual(
            MODULE.external_provider_interlock_reason(**focus_and_calibration),
            "physical_focus_lost",
        )


class KeyboardDoubleTapDetectorTest(unittest.TestCase):
    @staticmethod
    def sample(**keys):
        return MODULE.KeyboardMouseSample(focused=True, **keys)

    def test_same_key_press_release_press_activates_until_key_up(self) -> None:
        detector = MODULE.KeyboardDoubleTapDetector(0.30)
        self.assertFalse(
            detector.update(self.sample(w=True), now_s=1.00, enabled=True)
        )
        self.assertFalse(detector.update(self.sample(), now_s=1.08, enabled=True))
        self.assertTrue(
            detector.update(self.sample(w=True), now_s=1.20, enabled=True)
        )
        self.assertTrue(
            detector.update(
                self.sample(w=True, d=True), now_s=1.25, enabled=True
            )
        )
        self.assertFalse(
            detector.update(self.sample(d=True), now_s=1.30, enabled=True)
        )

    def test_hold_timeout_other_key_and_opposites_do_not_activate(self) -> None:
        detector = MODULE.KeyboardDoubleTapDetector(0.30)
        self.assertFalse(
            detector.update(self.sample(w=True), now_s=1.00, enabled=True)
        )
        self.assertFalse(
            detector.update(self.sample(w=True), now_s=1.40, enabled=True)
        )
        self.assertFalse(detector.update(self.sample(), now_s=1.41, enabled=True))
        self.assertFalse(
            detector.update(self.sample(w=True), now_s=1.42, enabled=True)
        )

        detector = MODULE.KeyboardDoubleTapDetector(0.30)
        detector.update(self.sample(w=True), now_s=2.00, enabled=True)
        detector.update(self.sample(), now_s=2.05, enabled=True)
        self.assertFalse(
            detector.update(self.sample(d=True), now_s=2.10, enabled=True)
        )
        self.assertFalse(
            detector.update(
                self.sample(w=True, s=True), now_s=2.15, enabled=True
            )
        )

    def test_interlock_and_source_change_clear_candidates(self) -> None:
        detector = MODULE.KeyboardDoubleTapDetector(0.30)
        detector.update(
            self.sample(w=True), now_s=1.00, enabled=True, source_id="physical"
        )
        detector.update(
            self.sample(), now_s=1.05, enabled=False, source_id="physical"
        )
        self.assertFalse(
            detector.update(
                self.sample(w=True),
                now_s=1.10,
                enabled=True,
                source_id="physical",
            )
        )
        self.assertFalse(
            detector.update(
                self.sample(w=True),
                now_s=1.15,
                enabled=True,
                source_id="external",
            )
        )

    def test_speed_tier_change_clears_candidates_and_active_boost(self) -> None:
        transitions = (
            ({"alt": True}, {"shift": True}),
            ({}, {"shift": True}),
            ({"shift": True}, {}),
            ({}, {"ctrl": True}),
            ({"ctrl": True}, {}),
        )
        for first_tier, second_tier in transitions:
            with self.subTest(first_tier=first_tier, second_tier=second_tier):
                detector = MODULE.KeyboardDoubleTapDetector(0.30)
                self.assertFalse(
                    detector.update(
                        self.sample(w=True, **first_tier),
                        now_s=1.00,
                        enabled=True,
                    )
                )
                self.assertFalse(
                    detector.update(
                        self.sample(**first_tier),
                        now_s=1.05,
                        enabled=True,
                    )
                )
                self.assertFalse(
                    detector.update(
                        self.sample(w=True, **second_tier),
                        now_s=1.10,
                        enabled=True,
                    )
                )
                self.assertEqual(detector.last_reset_reason, "tier_changed")

        detector = MODULE.KeyboardDoubleTapDetector(0.30)
        detector.update(self.sample(w=True), now_s=2.00, enabled=True)
        detector.update(self.sample(), now_s=2.05, enabled=True)
        self.assertTrue(
            detector.update(self.sample(w=True), now_s=2.10, enabled=True)
        )
        self.assertFalse(
            detector.update(
                self.sample(w=True, shift=True),
                now_s=2.15,
                enabled=True,
            )
        )
        self.assertEqual(detector.last_reset_reason, "tier_changed")

    def test_ctrl_and_alt_share_the_same_slow_tier_identity(self) -> None:
        detector = MODULE.KeyboardDoubleTapDetector(0.30)
        self.assertFalse(
            detector.update(
                self.sample(w=True, ctrl=True), now_s=1.00, enabled=True
            )
        )
        self.assertFalse(
            detector.update(self.sample(ctrl=True), now_s=1.05, enabled=True)
        )
        self.assertTrue(
            detector.update(
                self.sample(w=True, alt=True), now_s=1.10, enabled=True
            )
        )


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
        self.assertFalse(snapshot.keys.alt)
        self.assertFalse(snapshot.keys.shift)
        self.assertFalse(snapshot.keyboard_boost)

        boosted = MODULE.build_snapshot(
            sequence=8,
            timestamp_monotonic_s=12.6,
            keyboard=MODULE.KeyboardMouseSample(w=True, alt=True, focused=True),
            gamepad=MODULE.GamepadSample(),
            input_source="keyboard",
            camera_yaw_rad=0.0,
            camera_available=True,
            keyboard_boost=True,
        )
        self.assertTrue(boosted.keys.alt)
        self.assertTrue(boosted.keyboard_boost)

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


class UeFinalPovYawReaderTest(unittest.TestCase):
    class State:
        def __init__(self, yaw_deg: float) -> None:
            self.yaw_deg = yaw_deg
            self.pitch_deg = -12.0
            self.roll_deg = 0.5
            self.sequence = 7
            self.monotonic_ns = 999_000_000
            self.cache_timestamp_s = 42.0

    class Reader:
        def __init__(self, samples) -> None:
            self.samples = iter(samples)
            self.angles_changed = False
            self.max_angle_delta_deg = 0.0
            self.last_error = None
            self.read_count = 0

        def read(self):
            self.read_count += 1
            state, self.angles_changed, self.last_error = next(self.samples)
            self.max_angle_delta_deg = 12.5 if self.angles_changed else 0.0
            return state

    def adapter(self, samples):
        reader = self.Reader(samples)
        adapter = MODULE.UeFinalPovYawReader(
            Path("/run/user/1000/camera-state.bin"),
            expected_ue_pid=4242,
            reader=reader,
        )
        return adapter, reader

    def test_missing_and_stale_state_fail_closed_then_recovery_is_observable(self) -> None:
        adapter, reader = self.adapter(
            (
                (None, False, "missing_file"),
                (None, False, "stale"),
                (self.State(30.0), False, None),
            )
        )

        missing = adapter.read(10.0)
        stale = adapter.read(10.02)
        recovered = adapter.read(10.04)

        self.assertIsNone(missing.yaw_rad)
        self.assertEqual(missing.error, "missing_file")
        self.assertIsNone(stale.yaw_rad)
        self.assertEqual(stale.error, "stale")
        self.assertAlmostEqual(recovered.yaw_rad, math.radians(30.0))
        self.assertFalse(recovered.angles_changed)
        self.assertEqual(reader.read_count, 3)

    def test_robot_follow_angle_change_does_not_impersonate_a_mouse_drag(self) -> None:
        adapter, _reader = self.adapter(
            (
                (self.State(0.0), False, None),
                # A centered camera can rotate as a consequence of robot
                # motion, without an operator mouse-button boundary.
                (self.State(0.0), True, None),
                (self.State(0.0), False, None),
                (self.State(0.0), False, None),
                (self.State(0.0), False, None),
            )
        )
        core = CORE.GameControlCore(
            CORE.ControlConfig(
                max_acceleration_mps2=100.0,
                max_deceleration_mps2=100.0,
                max_turn_rate_rad_s=100.0,
                max_step_s=1.0,
            )
        )

        def deliver(sequence: int, timestamp: float, *, w: bool):
            observation = adapter.read(timestamp)
            keyboard = MODULE.KeyboardMouseSample(
                w=w,
                focused=True,
            )
            snapshot = MODULE.build_snapshot(
                sequence=sequence,
                timestamp_monotonic_s=timestamp,
                keyboard=keyboard,
                gamepad=MODULE.GamepadSample(),
                input_source="keyboard",
                camera_yaw_rad=observation.yaw_rad or 0.0,
                camera_available=observation.yaw_rad is not None,
            )
            core.accept_snapshot(snapshot, received_at_s=timestamp)
            return core.command(now_s=timestamp, dt_s=1.0)

        self.assertFalse(deliver(1, 1.00, w=False).safe_stop)
        moving = deliver(2, 1.01, w=True)
        self.assertFalse(moving.safe_stop)
        self.assertEqual(moving.mode, "move")
        observation = adapter.read(1.02)
        self.assertFalse(observation.angles_changed)
        self.assertEqual(observation.max_angle_delta_deg, 0.0)
        self.assertFalse(deliver(4, 1.03, w=False).safe_stop)
        self.assertFalse(deliver(5, 1.04, w=True).safe_stop)

    def test_final_pov_yaw_uses_provider_sign_and_offset_transform(self) -> None:
        adapter, _reader = self.adapter(((self.State(30.0), False, None),))
        observation = adapter.read(1.0)
        assert observation.yaw_rad is not None
        sonic_yaw = MODULE.transform_camera_yaw(
            observation.yaw_rad,
            sign=-1,
            offset_rad=math.radians(90.0),
        )
        self.assertAlmostEqual(sonic_yaw, math.radians(60.0))

        telemetry = MODULE.ue_final_pov_telemetry(observation)
        self.assertTrue(telemetry["available"])
        self.assertEqual(telemetry["sequence"], 7)
        self.assertEqual(telemetry["sample_age_ms"], 1.0)
        self.assertAlmostEqual(telemetry["provider_yaw_deg"], 30.0)
        self.assertEqual(telemetry["pitch_deg"], -12.0)
        self.assertEqual(telemetry["cache_timestamp_s"], 42.0)

    def test_final_pov_uses_xi2_only_for_drag_boundaries(self) -> None:
        self.assertTrue(MODULE.captures_xi2_drag_boundaries("ue-final-pov"))
        self.assertTrue(MODULE.captures_xi2_drag_boundaries("x11-mirror"))
        self.assertTrue(MODULE.captures_xi2_drag_boundaries("x11-core-gated"))
        self.assertFalse(MODULE.captures_xi2_drag_boundaries("x11-absolute"))
        self.assertFalse(MODULE.captures_xi2_drag_boundaries("fixed"))
        self.assertFalse(MODULE.captures_xi2_drag_boundaries("carla"))


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

    def test_ui_font_scale_click_persists_and_restores(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "config/matrix/ui-settings.json"
            controller = MODULE.UiSettingsController(
                path=path,
                desired=MODULE.UiSettings(),
                load_status="missing",
                load_error=None,
            )
            self.assertFalse(
                controller.apply_panel_action("font_up", active=False)
            )
            self.assertTrue(controller.apply_panel_action("font_up", active=True))
            self.assertEqual(controller.desired.font_scale, 1.1)
            self.assertEqual(
                json.loads(path.read_text(encoding="utf-8")),
                {"font_scale": 1.1, "version": 1},
            )
            self.assertEqual(path.stat().st_mode & 0o777, 0o600)
            restored = MODULE.load_ui_settings(path)
            self.assertEqual(restored.status, "loaded")
            self.assertEqual(restored.settings.font_scale, 1.1)

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

    def test_source_specific_sensitivity_and_yaw_telemetry(self) -> None:
        core = MODULE.mirror_sensitivity_mapping(
            "x11-core-gated",
            base_deg_per_unit=0.12,
            effective_deg_per_unit=0.0024,
        )
        absolute = MODULE.mirror_sensitivity_mapping(
            "x11-absolute",
            base_deg_per_unit=0.12,
            effective_deg_per_unit=0.0024,
        )
        self.assertEqual(core["units"], "degrees_per_xi2_raw_unit")
        self.assertEqual(absolute["units"], "degrees_per_x11_root_pixel")
        self.assertEqual(core["effective_deg_per_unit"], 0.0024)
        yaw = MODULE.camera_yaw_telemetry(
            "x11-core-gated",
            provider_yaw_rad=math.pi / 2.0,
            sonic_yaw_rad=-math.pi / 2.0,
        )
        self.assertAlmostEqual(yaw["provider_yaw_deg"], 90.0)
        self.assertAlmostEqual(yaw["sonic_yaw_deg"], -90.0)
        claim = MODULE.camera_source_claim("x11-core-gated")
        self.assertEqual(
            claim["button_gate_truth_scope"],
            "xquerypointer_core_button_level_sampled_not_event_ordered",
        )
        self.assertTrue(claim["experimental"])
        self.assertFalse(claim["legacy"])
        self.assertFalse(claim["visible_follow_camera_verified"])
        final_pov = MODULE.camera_source_claim("ue-final-pov")
        self.assertEqual(
            final_pov["camera_yaw_truth_scope"],
            "player_camera_manager_final_pov",
        )
        self.assertTrue(final_pov["experimental"])
        self.assertFalse(final_pov["visible_follow_camera_verified"])
        self.assertEqual(
            final_pov["button_gate_truth_scope"],
            "xquerypointer_core_level_or_xi2_raw_button_edges",
        )


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


class XInput2RawMotionTest(unittest.TestCase):
    @staticmethod
    def raw(evtype, *, source=6, device=2, detail=0, dx=0.0, dy=0.0):
        return MODULE.XInput2RawEvent(
            evtype=evtype,
            deviceid=device,
            sourceid=source,
            detail=detail,
            dx=dx,
            dy=dy,
        )

    def test_sparse_valuator_mask_decodes_packed_xy(self) -> None:
        # Axes 0, 1, and 3 are present; packed values do not include axis 2.
        self.assertEqual(
            MODULE.decode_xinput2_xy(bytes((0b00001011,)), (4.5, -2.0, 99.0)),
            (4.5, -2.0),
        )
        self.assertEqual(
            MODULE.decode_xinput2_xy(bytes((0b00000010,)), (3.0,)),
            (0.0, 3.0),
        )
        with self.assertRaisesRegex(RuntimeError, "count differs"):
            MODULE.decode_xinput2_xy(bytes((0b00000011,)), (1.0,))
        with self.assertRaisesRegex(RuntimeError, "non-finite"):
            MODULE.decode_xinput2_xy(bytes((0b00000001,)), (math.nan,))

    def test_raw_button_edges_attribute_only_held_motion(self) -> None:
        accumulator = MODULE.XInput2DragAccumulator(look_button_detail=1)
        self.assertEqual(
            accumulator.update((), current_look_pressed=False),
            (0.0, 0.0, False),
        )
        dx, dy, drag_observed = accumulator.update(
            (
                self.raw(MODULE._XI_RAW_BUTTON_PRESS, detail=2),
                self.raw(MODULE._XI_RAW_MOTION, dx=9.0, dy=8.0),
                self.raw(MODULE._XI_RAW_BUTTON_PRESS, detail=1),
                self.raw(MODULE._XI_RAW_MOTION, dx=3.0, dy=-4.0),
                self.raw(MODULE._XI_RAW_BUTTON_RELEASE, detail=1),
                self.raw(MODULE._XI_RAW_MOTION, dx=100.0, dy=100.0),
            ),
            current_look_pressed=False,
        )
        self.assertEqual((dx, dy), (3.0, -4.0))
        self.assertTrue(drag_observed)
        self.assertEqual(accumulator.button_state_resyncs, 0)

        fresh_first_press = MODULE.XInput2DragAccumulator(1)
        self.assertEqual(
            fresh_first_press.update(
                (
                    self.raw(MODULE._XI_RAW_BUTTON_PRESS, detail=1),
                    self.raw(MODULE._XI_RAW_MOTION, dx=6.0),
                    self.raw(MODULE._XI_RAW_BUTTON_RELEASE, detail=1),
                ),
                current_look_pressed=False,
            ),
            (6.0, 0.0, True),
        )

    def test_cross_source_drag_fails_closed(self) -> None:
        accumulator = MODULE.XInput2DragAccumulator(look_button_detail=1)
        accumulator.update((), current_look_pressed=False)
        accumulator.update(
            (self.raw(MODULE._XI_RAW_BUTTON_PRESS, source=6, detail=1),),
            current_look_pressed=True,
        )
        with self.assertRaisesRegex(RuntimeError, "crossed input sources"):
            accumulator.update(
                (self.raw(MODULE._XI_RAW_MOTION, source=4, dx=20.0),),
                current_look_pressed=True,
            )

    def test_disarm_requires_release_then_fresh_same_source_press(self) -> None:
        accumulator = MODULE.XInput2DragAccumulator(look_button_detail=1)
        accumulator.update((), current_look_pressed=False)
        self.assertEqual(
            accumulator.update(
                (
                    self.raw(MODULE._XI_RAW_BUTTON_PRESS, detail=1),
                    self.raw(MODULE._XI_RAW_MOTION, dx=2.0),
                ),
                current_look_pressed=True,
            ),
            (2.0, 0.0, True),
        )
        accumulator.disarm()
        self.assertEqual(
            accumulator.update(
                (self.raw(MODULE._XI_RAW_MOTION, dx=30.0),),
                current_look_pressed=True,
            ),
            (0.0, 0.0, False),
        )
        self.assertEqual(
            accumulator.update(
                (self.raw(MODULE._XI_RAW_BUTTON_RELEASE, detail=1),),
                current_look_pressed=False,
            ),
            (0.0, 0.0, False),
        )
        self.assertEqual(
            accumulator.update(
                (
                    self.raw(MODULE._XI_RAW_BUTTON_PRESS, detail=1),
                    self.raw(MODULE._XI_RAW_MOTION, dx=5.0),
                ),
                current_look_pressed=True,
            ),
            (5.0, 0.0, True),
        )

    def test_missed_release_discards_ambiguously_attributed_batch(self) -> None:
        accumulator = MODULE.XInput2DragAccumulator(look_button_detail=1)
        accumulator.update((), current_look_pressed=False)
        accumulator.update(
            (self.raw(MODULE._XI_RAW_BUTTON_PRESS, detail=1),),
            current_look_pressed=True,
        )

        self.assertEqual(
            accumulator.update(
                (self.raw(MODULE._XI_RAW_MOTION, dx=30.0),),
                current_look_pressed=False,
            ),
            (0.0, 0.0, True),
        )
        self.assertEqual(accumulator.button_state_resyncs, 1)

    def test_core_gate_accepts_only_stable_held_intervals(self) -> None:
        accumulator = MODULE.XInput2CoreGatedAccumulator(look_button_detail=1)
        self.assertEqual(
            accumulator.update((), current_look_pressed=False),
            (0.0, 0.0, False),
        )
        self.assertEqual(
            accumulator.update(
                (self.raw(MODULE._XI_RAW_MOTION, dx=9.0),),
                current_look_pressed=True,
            ),
            (0.0, 0.0, True),
        )
        self.assertEqual(accumulator.last_drop_reason, "core_press_boundary")
        self.assertEqual(
            accumulator.update(
                (self.raw(MODULE._XI_RAW_MOTION, dx=4.0, dy=-2.0),),
                current_look_pressed=True,
            ),
            (4.0, -2.0, True),
        )
        self.assertIsNone(accumulator.last_drop_reason)
        self.assertEqual(
            accumulator.update(
                (self.raw(MODULE._XI_RAW_MOTION, dx=6.0),),
                current_look_pressed=False,
            ),
            (0.0, 0.0, True),
        )
        self.assertEqual(accumulator.last_drop_reason, "core_release_boundary")
        self.assertEqual(accumulator.ambiguous_raw_motion_events, 2)
        self.assertEqual(accumulator.ambiguous_raw_dx_total, 15.0)

    def test_core_gate_quick_drag_is_interlocked_but_never_integrated(self) -> None:
        accumulator = MODULE.XInput2CoreGatedAccumulator(look_button_detail=1)
        accumulator.update((), current_look_pressed=False)
        result = accumulator.update(
            (
                self.raw(MODULE._XI_RAW_BUTTON_PRESS, detail=1),
                self.raw(MODULE._XI_RAW_MOTION, dx=30.0),
                self.raw(MODULE._XI_RAW_BUTTON_RELEASE, detail=1),
            ),
            current_look_pressed=False,
        )
        self.assertEqual(result, (0.0, 0.0, True))
        self.assertEqual(
            accumulator.last_drop_reason, "quick_press_drag_release"
        )
        self.assertEqual(accumulator.ambiguous_raw_motion_events, 1)

    def test_core_gate_disarm_requires_release_then_fresh_level_edge(self) -> None:
        accumulator = MODULE.XInput2CoreGatedAccumulator(look_button_detail=1)
        self.assertEqual(
            accumulator.update(
                (self.raw(MODULE._XI_RAW_MOTION, dx=20.0),),
                current_look_pressed=True,
            ),
            (0.0, 0.0, True),
        )
        self.assertEqual(accumulator.last_drop_reason, "awaiting_core_release")
        self.assertEqual(
            accumulator.update((), current_look_pressed=False),
            (0.0, 0.0, False),
        )
        self.assertEqual(
            accumulator.update((), current_look_pressed=True),
            (0.0, 0.0, True),
        )
        self.assertEqual(
            accumulator.update(
                (self.raw(MODULE._XI_RAW_MOTION, dx=5.0),),
                current_look_pressed=True,
            ),
            (5.0, 0.0, True),
        )

    def test_core_gate_binds_one_slave_and_rearms_on_source_change(self) -> None:
        accumulator = MODULE.XInput2CoreGatedAccumulator(look_button_detail=1)
        accumulator.update((), current_look_pressed=False)
        accumulator.update(
            (self.raw(MODULE._XI_RAW_MOTION, source=6, dx=1.0),),
            current_look_pressed=True,
        )
        self.assertEqual(accumulator.bound_sourceid, 6)
        self.assertEqual(
            accumulator.update(
                (self.raw(MODULE._XI_RAW_MOTION, source=6, dx=2.0),),
                current_look_pressed=True,
            ),
            (2.0, 0.0, True),
        )
        self.assertEqual(
            accumulator.update(
                (self.raw(MODULE._XI_RAW_MOTION, source=7, dx=30.0),),
                current_look_pressed=True,
            ),
            (0.0, 0.0, True),
        )
        self.assertEqual(accumulator.last_drop_reason, "slave_source_changed")
        self.assertIsNone(accumulator.bound_sourceid)
        self.assertEqual(accumulator.source_bindings, 1)
        self.assertEqual(accumulator.source_rejections, 1)

    def test_core_gate_rejects_multiple_slaves_in_one_batch(self) -> None:
        accumulator = MODULE.XInput2CoreGatedAccumulator(look_button_detail=1)
        accumulator.update((), current_look_pressed=False)
        self.assertEqual(
            accumulator.update(
                (
                    self.raw(MODULE._XI_RAW_MOTION, source=6, dx=2.0),
                    self.raw(MODULE._XI_RAW_MOTION, source=7, dx=3.0),
                ),
                current_look_pressed=True,
            ),
            (0.0, 0.0, True),
        )
        self.assertEqual(accumulator.last_drop_reason, "multiple_slave_sources")
        self.assertEqual(accumulator.source_rejections, 1)

    def test_core_gate_delayed_raw_edges_drop_batch_but_keep_hold_binding(self) -> None:
        accumulator = MODULE.XInput2CoreGatedAccumulator(look_button_detail=1)
        accumulator.update((), current_look_pressed=False)
        accumulator.update(
            (self.raw(MODULE._XI_RAW_MOTION, source=6, dx=1.0),),
            current_look_pressed=True,
        )
        self.assertEqual(
            accumulator.update(
                (
                    self.raw(MODULE._XI_RAW_BUTTON_PRESS, detail=1),
                    self.raw(MODULE._XI_RAW_MOTION, dx=8.0),
                ),
                current_look_pressed=True,
            ),
            (0.0, 0.0, True),
        )
        self.assertEqual(
            accumulator.last_drop_reason,
            "raw_button_edge_inside_stable_core_hold",
        )
        self.assertEqual(accumulator.bound_sourceid, 6)
        self.assertEqual(
            accumulator.update(
                (self.raw(MODULE._XI_RAW_MOTION, source=6, dx=3.0),),
                current_look_pressed=True,
            ),
            (3.0, 0.0, True),
        )
        self.assertEqual(
            accumulator.update(
                (
                    self.raw(MODULE._XI_RAW_BUTTON_RELEASE, detail=1),
                    self.raw(MODULE._XI_RAW_MOTION, dx=9.0),
                ),
                current_look_pressed=True,
            ),
            (0.0, 0.0, True),
        )
        self.assertEqual(accumulator.bound_sourceid, 6)
        self.assertEqual(
            accumulator.update(
                (self.raw(MODULE._XI_RAW_MOTION, source=6, dx=4.0),),
                current_look_pressed=True,
            ),
            (4.0, 0.0, True),
        )

    @staticmethod
    def reader_for_events(events, *, enumerated_masters=(2,)):
        pending_values = []
        for _event in events:
            pending_values.extend((1,))
        pending_values.append(0)
        pending = iter(pending_values)
        event_iterator = iter(events)

        class Pending:
            @staticmethod
            def XPending(_display) -> int:
                return next(pending)

            @staticmethod
            def XFlush(_display) -> int:
                return 1

        reader = object.__new__(MODULE.XInput2RawMotion)
        reader._x11 = Pending()
        reader._display = 1
        reader._read_event = lambda: next(event_iterator)
        reader._accumulator = MODULE.XInput2DragAccumulator(1)
        reader.events_consumed = 0
        reader.raw_motion_events = 0
        reader.hierarchy_events = 0
        reader.foreign_master_events = 0
        reader.master_device_changes = 0
        reader._master_deviceid = 2
        reader._single_master_pointer_deviceid = lambda: next(
            reader._enumerated_masters
        )
        reader._enumerated_masters = iter(enumerated_masters)
        return reader

    @staticmethod
    def batched_reader(*, button_gate="xi2-events"):
        class Pending:
            def __init__(self) -> None:
                self.events = []

            def XPending(self, _display) -> int:
                return int(bool(self.events))

            @staticmethod
            def XFlush(_display) -> int:
                return 1

        pending = Pending()
        reader = object.__new__(MODULE.XInput2RawMotion)
        reader._x11 = pending
        reader._display = 1
        reader._read_event = lambda: pending.events.pop(0)
        reader._button_gate = button_gate
        accumulator_type = (
            MODULE.XInput2CoreGatedAccumulator
            if button_gate == "x11-core-level"
            else MODULE.XInput2DragAccumulator
        )
        reader._accumulator = accumulator_type(1)
        reader.events_consumed = 0
        reader.raw_motion_events = 0
        reader.hierarchy_events = 0
        reader.foreign_master_events = 0
        reader.master_device_changes = 0
        reader._master_deviceid = 2
        reader._negotiated_version = (2, 0)
        reader._single_master_pointer_deviceid = lambda: 2

        def load(*events) -> None:
            if pending.events:
                raise AssertionError("previous XI2 batch was not drained")
            pending.events.extend(events)

        reader.load = load
        return reader

    def test_xi2_event_gate_poll_behavior_is_preserved_with_telemetry(self) -> None:
        reader = self.batched_reader()
        self.assertEqual(
            reader.poll(current_look_pressed=False, focused=True),
            (0.0, 0.0, False),
        )
        reader.load(
            self.raw(MODULE._XI_RAW_BUTTON_PRESS, detail=1),
            self.raw(MODULE._XI_RAW_MOTION, dx=3.0),
        )
        self.assertEqual(
            reader.poll(current_look_pressed=True, focused=True),
            (3.0, 0.0, True),
        )
        reader.load(
            self.raw(MODULE._XI_RAW_MOTION, dx=2.0),
            self.raw(MODULE._XI_RAW_BUTTON_RELEASE, detail=1),
        )
        self.assertEqual(
            reader.poll(current_look_pressed=False, focused=True),
            (2.0, 0.0, True),
        )
        telemetry = reader.telemetry
        self.assertEqual(telemetry["button_gate"], "xi2-events")
        self.assertEqual(telemetry["accepted_dx_total"], 5.0)
        self.assertEqual(telemetry["dropped_motion_events"], 0)
        self.assertEqual(telemetry["button_state_resyncs"], 0)

    def test_core_gated_poll_counts_accepted_and_boundary_drops(self) -> None:
        reader = self.batched_reader(button_gate="x11-core-level")
        reader.poll(current_look_pressed=False, focused=True)
        reader.load(self.raw(MODULE._XI_RAW_MOTION, source=6, dx=10.0))
        self.assertEqual(
            reader.poll(current_look_pressed=True, focused=True),
            (0.0, 0.0, True),
        )
        reader.load(self.raw(MODULE._XI_RAW_MOTION, source=6, dx=4.0))
        self.assertEqual(
            reader.poll(current_look_pressed=True, focused=True),
            (4.0, 0.0, True),
        )
        reader.load(self.raw(MODULE._XI_RAW_MOTION, source=6, dx=6.0))
        self.assertEqual(
            reader.poll(current_look_pressed=False, focused=True),
            (0.0, 0.0, True),
        )
        telemetry = reader.telemetry
        self.assertEqual(telemetry["button_gate"], "x11-core-level")
        self.assertEqual(telemetry["accepted_dx_total"], 4.0)
        self.assertEqual(telemetry["dropped_motion_events"], 2)
        self.assertEqual(telemetry["dropped_dx_total"], 16.0)
        self.assertEqual(telemetry["source_bindings"], 1)
        self.assertEqual(
            telemetry["drop_reason_counts"],
            {"core_press_boundary": 1, "core_release_boundary": 1},
        )

    def test_poll_resync_and_focus_drop_include_motion_totals(self) -> None:
        reader = self.batched_reader()
        reader.poll(current_look_pressed=False, focused=True)
        reader.load(self.raw(MODULE._XI_RAW_BUTTON_PRESS, detail=1))
        reader.poll(current_look_pressed=True, focused=True)
        reader.load(self.raw(MODULE._XI_RAW_MOTION, dx=30.0, dy=-2.0))
        self.assertEqual(
            reader.poll(current_look_pressed=False, focused=True),
            (0.0, 0.0, True),
        )
        reader.load(self.raw(MODULE._XI_RAW_MOTION, dx=7.0, dy=1.0))
        reader.poll(current_look_pressed=True, focused=False)
        telemetry = reader.telemetry
        self.assertEqual(telemetry["dropped_motion_events"], 2)
        self.assertEqual(telemetry["dropped_dx_total"], 37.0)
        self.assertEqual(telemetry["dropped_dy_total"], -1.0)
        self.assertEqual(
            telemetry["drop_reason_counts"],
            {"xi2_button_state_resync": 1, "focus_or_pointer_invalid": 1},
        )

    def test_core_gated_drag_requires_neutral_before_camera_relative_w(self) -> None:
        accumulator = MODULE.XInput2CoreGatedAccumulator(look_button_detail=1)
        tracker = MODULE.CameraYawTracker(
            0.0,
            mouse_radians_per_pixel=math.pi / 2.0,
            gamepad_radians_per_second=0.0,
        )
        core = CORE.GameControlCore(
            CORE.ControlConfig(
                max_acceleration_mps2=100.0,
                max_deceleration_mps2=100.0,
                max_turn_rate_rad_s=100.0,
                max_step_s=1.0,
            )
        )

        def deliver(sequence, timestamp, *, w=False, dragging=False):
            snapshot = MODULE.build_snapshot(
                sequence=sequence,
                timestamp_monotonic_s=timestamp,
                keyboard=MODULE.KeyboardMouseSample(
                    w=w,
                    camera_dragging=dragging,
                    focused=True,
                ),
                gamepad=MODULE.GamepadSample(),
                input_source="keyboard",
                camera_yaw_rad=tracker.yaw,
                camera_available=True,
            )
            core.accept_snapshot(snapshot, received_at_s=timestamp)
            return core.command(now_s=timestamp, dt_s=1.0)

        accumulator.update((), current_look_pressed=False)
        self.assertFalse(deliver(1, 1.00).safe_stop)
        _, _, press_drag = accumulator.update(
            (), current_look_pressed=True
        )
        self.assertTrue(deliver(2, 1.01, dragging=press_drag).safe_stop)
        dx, _, held_drag = accumulator.update(
            (self.raw(MODULE._XI_RAW_MOTION, source=6, dx=1.0),),
            current_look_pressed=True,
        )
        tracker.update(dt=0.02, mouse_dx=dx, gamepad_look_yaw=0.0)
        self.assertTrue(deliver(3, 1.02, w=True, dragging=held_drag).safe_stop)
        _, _, release_drag = accumulator.update(
            (), current_look_pressed=False
        )
        self.assertTrue(deliver(4, 1.03, w=True, dragging=release_drag).safe_stop)
        awaiting = deliver(5, 1.04, w=True)
        self.assertTrue(awaiting.safe_stop)
        self.assertEqual(awaiting.reason, "awaiting_neutral")
        self.assertFalse(deliver(6, 1.05).safe_stop)
        resumed = deliver(7, 1.06, w=True)
        self.assertFalse(resumed.safe_stop)
        self.assertAlmostEqual(resumed.movement[0], 0.0, places=7)
        self.assertAlmostEqual(resumed.movement[1], 1.0, places=7)

    def test_hierarchy_event_discards_complete_batch_and_rebinds(self) -> None:
        reader = self.reader_for_events(
            (
                self.raw(MODULE._XI_RAW_BUTTON_PRESS, detail=1),
                self.raw(MODULE._XI_RAW_MOTION, dx=40.0),
                MODULE.XInput2RawEvent(MODULE._XI_HIERARCHY_CHANGED),
            ),
            enumerated_masters=(2,),
        )

        self.assertEqual(
            reader.poll(current_look_pressed=True, focused=True),
            (0.0, 0.0, True),
        )
        self.assertEqual(reader.hierarchy_events, 1)

    def test_hierarchy_master_change_discards_batch_without_reselecting_old_id(
        self,
    ) -> None:
        reader = self.reader_for_events(
            (MODULE.XInput2RawEvent(MODULE._XI_HIERARCHY_CHANGED),),
            enumerated_masters=(8,),
        )

        self.assertEqual(
            reader.poll(current_look_pressed=False, focused=True),
            (0.0, 0.0, True),
        )
        self.assertEqual(reader._master_deviceid, 8)
        self.assertEqual(reader.master_device_changes, 1)

    def test_raw_subscription_uses_all_master_devices_once(self) -> None:
        reader = object.__new__(MODULE.XInput2RawMotion)
        reader._raw_mask_buffer = object()
        selections = []
        reader._select_mask = lambda *, deviceid, buffer: selections.append(
            (deviceid, buffer)
        )

        reader._subscribe_raw_masters()

        self.assertEqual(
            selections,
            [(MODULE._XI_ALL_MASTER_DEVICES, reader._raw_mask_buffer)],
        )

    def test_unexpected_foreign_master_drops_batch_and_fails_closed(self) -> None:
        reader = self.reader_for_events(
            (
                self.raw(MODULE._XI_RAW_MOTION, device=9, dx=1.0),
                self.raw(MODULE._XI_RAW_MOTION, device=9, dx=2.0),
            ),
        )

        self.assertEqual(
            reader.poll(current_look_pressed=False, focused=True),
            (0.0, 0.0, True),
        )
        self.assertEqual(reader.foreign_master_events, 2)

    @staticmethod
    def _device_query(*entries):
        class FakeXi:
            def __init__(self) -> None:
                self.free_calls = 0
                if entries:
                    self.array = (MODULE._XIDeviceInfo * len(entries))(
                        *(
                            MODULE._XIDeviceInfo(
                                deviceid=deviceid,
                                name=name.encode(),
                                use=use,
                                attachment=attachment,
                                enabled=enabled,
                                num_classes=0,
                                classes=None,
                            )
                            for deviceid, name, use, attachment, enabled in entries
                        )
                    )
                    self.pointer = ctypes.cast(
                        self.array, ctypes.POINTER(MODULE._XIDeviceInfo)
                    )
                else:
                    self.pointer = ctypes.POINTER(MODULE._XIDeviceInfo)()

            def XIQueryDevice(self, _display, selector, count_pointer):
                if selector != MODULE._XI_ALL_MASTER_DEVICES:
                    raise AssertionError(f"unexpected selector {selector}")
                count_pointer._obj.value = len(entries)
                return self.pointer

            def XIFreeDeviceInfo(self, _devices) -> None:
                self.free_calls += 1

        return FakeXi()

    def test_device_query_accepts_only_one_enabled_master_pointer(self) -> None:
        xi = self._device_query(
            (2, "Virtual core pointer", MODULE._XI_MASTER_POINTER, 3, 1),
            (3, "Virtual core keyboard", 2, 2, 1),
        )
        reader = object.__new__(MODULE.XInput2RawMotion)
        reader._display = 1
        reader._xi = xi

        self.assertEqual(reader._single_master_pointer_deviceid(), 2)
        self.assertEqual(xi.free_calls, 1)

    def test_device_query_rejects_zero_or_two_master_pointers(self) -> None:
        for entries in (
            (),
            (
                (2, "master-a", MODULE._XI_MASTER_POINTER, 3, 1),
                (8, "master-b", MODULE._XI_MASTER_POINTER, 9, 1),
            ),
        ):
            with self.subTest(master_count=len(entries)):
                xi = self._device_query(*entries)
                reader = object.__new__(MODULE.XInput2RawMotion)
                reader._display = 1
                reader._xi = xi
                with self.assertRaisesRegex(RuntimeError, "exactly one"):
                    reader._single_master_pointer_deviceid()
                self.assertEqual(xi.free_calls, int(bool(entries)))

    def test_invalid_device_count_still_frees_nonnull_query_result(self) -> None:
        xi = self._device_query(
            (2, "master", MODULE._XI_MASTER_POINTER, 3, 1),
        )
        original_query = xi.XIQueryDevice

        def invalid_count_query(display, selector, count_pointer):
            devices = original_query(display, selector, count_pointer)
            count_pointer._obj.value = 257
            return devices

        xi.XIQueryDevice = invalid_count_query
        reader = object.__new__(MODULE.XInput2RawMotion)
        reader._display = 1
        reader._xi = xi

        with self.assertRaisesRegex(RuntimeError, "invalid device count"):
            reader._single_master_pointer_deviceid()
        self.assertEqual(xi.free_calls, 1)

    def test_hierarchy_with_ambiguous_master_topology_fails_closed(self) -> None:
        reader = self.reader_for_events(
            (MODULE.XInput2RawEvent(MODULE._XI_HIERARCHY_CHANGED),)
        )
        reader._single_master_pointer_deviceid = mock.Mock(
            side_effect=RuntimeError("requires exactly one master pointer")
        )

        with self.assertRaisesRegex(RuntimeError, "exactly one"):
            reader.poll(current_look_pressed=False, focused=True)

    def test_event_backlog_fails_closed_at_a_finite_bound(self) -> None:
        class PendingForever:
            @staticmethod
            def XPending(_display) -> int:
                return 1

        reader = object.__new__(MODULE.XInput2RawMotion)
        reader._x11 = PendingForever()
        reader._display = 1
        # Even unrelated GenericEvents must count toward the work bound.
        reader._read_event = lambda: None
        reader._accumulator = MODULE.XInput2DragAccumulator(1)
        reader.events_consumed = 0
        reader.raw_motion_events = 0
        reader.hierarchy_events = 0
        reader.foreign_master_events = 0
        reader.master_device_changes = 0
        reader._master_deviceid = 2

        with self.assertRaisesRegex(RuntimeError, "backlog"):
            reader.poll(current_look_pressed=True, focused=True)
        self.assertEqual(
            reader.events_consumed, MODULE._MAX_XI2_EVENTS_PER_POLL
        )

    def test_initialization_failure_closes_display_and_close_is_idempotent(self) -> None:
        class FakeX11:
            def __init__(self) -> None:
                self.close_calls = 0

            @staticmethod
            def XOpenDisplay(_name):
                return 11

            @staticmethod
            def XQueryExtension(*_args):
                return 1

            @staticmethod
            def XDefaultRootWindow(_display):
                return 22

            def XCloseDisplay(self, _display):
                self.close_calls += 1
                return 1

        class FakeXi:
            @staticmethod
            def XIQueryVersion(*_args):
                return 0

            @staticmethod
            def XIQueryDevice(_display, selector, count_pointer):
                if selector != MODULE._XI_ALL_MASTER_DEVICES:
                    raise AssertionError(f"unexpected selector {selector}")
                count_pointer._obj.value = 0
                return ctypes.POINTER(MODULE._XIDeviceInfo)()

            @staticmethod
            def XIFreeDeviceInfo(_devices):
                raise AssertionError("null device array must not be freed")

        x11 = FakeX11()
        with mock.patch.object(
            MODULE.XInput2RawMotion,
            "_configure_signatures",
            lambda _self: None,
        ):
            with self.assertRaisesRegex(RuntimeError, "exactly one"):
                MODULE.XInput2RawMotion(
                    display_name=":999",
                    look_button="left",
                    x11_library=x11,
                    xi_library=FakeXi(),
                )
        self.assertEqual(x11.close_calls, 1)

        reader = object.__new__(MODULE.XInput2RawMotion)
        reader._x11 = x11
        reader._display = 33
        reader.close()
        reader.close()
        self.assertEqual(x11.close_calls, 2)


class X11KeyboardMouseSafetyTest(unittest.TestCase):
    class AsyncFocusX11:
        """Deliver one queued X error only when the backend calls XSync."""

        def __init__(self, *, error_code=MODULE._X11_BAD_WINDOW, resource=77):
            self.error_code = error_code
            self.resource = resource
            self.handler = 0
            self.pending = False
            self.sync_calls = 0
            self.delivered = 0

        def XSync(self, display, _discard) -> int:
            self.sync_calls += 1
            if self.pending:
                self.pending = False
                event = MODULE._XErrorEvent(
                    type=0,
                    display=display,
                    resourceid=self.resource,
                    serial=1,
                    error_code=self.error_code,
                    request_code=20,
                    minor_code=0,
                )
                if not self.handler:
                    raise AssertionError("queued X error had no installed handler")
                MODULE._X11_ERROR_HANDLER(self.handler)(
                    display, ctypes.byref(event)
                )
                self.delivered += 1
            return 1

        def XSetErrorHandler(self, handler):
            previous = self.handler
            self.handler = MODULE.X11KeyboardMouse._pointer_value(handler)
            return previous or None

        @staticmethod
        def XGetInputFocus(_display, focus, _revert) -> int:
            focus._obj.value = 77
            return 1

        def XFetchName(self, _display, _window, _name) -> int:
            self.pending = True
            return 0

        @staticmethod
        def XGetWindowProperty(*args) -> int:
            args[8]._obj.value = 0
            args[9]._obj.value = 0
            return 0

        @staticmethod
        def XQueryTree(*_args) -> int:
            return 0

        @staticmethod
        def XFree(_value) -> int:
            return 1

    @staticmethod
    def _focus_backend(x11):
        backend = object.__new__(MODULE.X11KeyboardMouse)
        backend._x11 = x11
        backend._display = 11
        backend._root = 2
        backend._pid_atom = 9
        backend._active_focus_error_scope = None
        backend._previous_x_error_handler = None
        backend._x_error_handler_callback = MODULE._X11_ERROR_HANDLER(
            backend._handle_x_error
        )
        return backend

    def test_async_badwindow_in_tracked_focus_chain_fails_closed(self) -> None:
        x11 = self.AsyncFocusX11()
        backend = self._focus_backend(x11)

        focus = backend._focus_identity()

        self.assertEqual(focus, (False, None, frozenset()))
        self.assertEqual(x11.delivered, 1)
        self.assertEqual(x11.sync_calls, 2)
        self.assertEqual(x11.handler, 0)
        self.assertEqual(backend._focus_badwindow_recoveries, 1)
        self.assertEqual(backend._last_focus_badwindow_resource, 77)

    def test_non_badwindow_focus_error_is_not_swallowed(self) -> None:
        x11 = self.AsyncFocusX11(error_code=2)
        backend = self._focus_backend(x11)

        with self.assertRaisesRegex(RuntimeError, "unexpected X11 error"):
            backend._focus_identity()

        self.assertEqual(x11.delivered, 1)
        self.assertEqual(x11.handler, 0)

    def test_focus_scope_restores_previous_handler_on_body_exception(self) -> None:
        x11 = self.AsyncFocusX11()
        x11.handler = 1234
        backend = self._focus_backend(x11)

        with self.assertRaisesRegex(ValueError, "body failed"):
            with backend._focus_window_error_scope():
                raise ValueError("body failed")

        self.assertEqual(x11.handler, 1234)
        self.assertIsNone(backend._active_focus_error_scope)
        self.assertIsNone(backend._previous_x_error_handler)

    def test_untracked_badwindow_is_not_misclassified_as_focus_race(self) -> None:
        x11 = self.AsyncFocusX11(resource=88)
        backend = self._focus_backend(x11)

        with self.assertRaisesRegex(RuntimeError, "resource=88"):
            backend._focus_identity()

        self.assertEqual(x11.delivered, 1)
        self.assertEqual(x11.handler, 0)

    @staticmethod
    def _raw_backend(
        *,
        focus_results,
        raw_deltas,
        pointer_values=((10, 1 << 8), (20, 1 << 8)),
        pressed_names=(),
    ):
        pointer_samples = iter(pointer_values)

        class FakeX11:
            @staticmethod
            def XQueryKeymap(_display, _buffer) -> int:
                return 1

            @staticmethod
            def XQueryPointer(*args) -> int:
                x, button_mask = next(pointer_samples)
                args[4]._obj.value = x
                args[5]._obj.value = 0
                args[8]._obj.value = button_mask
                return 1

        class FakeRaw:
            telemetry = {"motion_source": "xi2-raw"}

            def __init__(self) -> None:
                self.deltas = iter(raw_deltas)
                self.button_states = []

            def poll(self, *, current_look_pressed, focused):
                self.button_states.append((current_look_pressed, focused))
                return next(self.deltas)

        backend = object.__new__(MODULE.X11KeyboardMouse)
        backend._x11 = FakeX11()
        backend._display = 1
        backend._root = 2
        backend._keycodes = {
            name: index
            for index, name in enumerate(MODULE.X11KeyboardMouse._KEYSYMS, start=8)
        }
        pressed_codes = {
            backend._keycodes[name] for name in pressed_names
        }
        backend._pressed = lambda _keymap, code: code in pressed_codes
        backend._focus_pattern = None
        backend._look_mask = 1 << 8
        backend._previous_pointer = None
        backend._previous_look_pressed = False
        backend._maximum_mouse_delta = 200.0
        backend._teleport_rejections = 0
        backend._last_teleport_delta = None
        backend._expected_ue_pid = 1234
        backend._focus_identity = lambda: next(focus_results)
        backend._raw_motion = FakeRaw()
        return backend

    def test_raw_focus_loss_discards_delta_without_replay(self) -> None:
        backend = self._raw_backend(
            focus_results=iter(
                (
                    (False, "Other", frozenset()),
                    (True, "Matrix", frozenset({1234})),
                )
            ),
            raw_deltas=((7.0, -3.0, True), (0.0, 0.0, False)),
        )

        unfocused = backend.poll()
        refocused = backend.poll()

        self.assertEqual((unfocused.mouse_dx, unfocused.mouse_dy), (0.0, 0.0))
        self.assertEqual((refocused.mouse_dx, refocused.mouse_dy), (0.0, 0.0))
        self.assertFalse(unfocused.focused)
        self.assertTrue(refocused.focused)
        self.assertEqual(
            backend._raw_motion.button_states,
            [(True, False), (True, True)],
        )
        self.assertEqual(
            backend.pointer_telemetry["motion_source"], "xi2-raw"
        )

    def test_completed_raw_drag_interlocks_same_frame_w(self) -> None:
        backend = self._raw_backend(
            focus_results=iter(((True, "Matrix", frozenset({1234})),)),
            raw_deltas=((12.0, 0.0, True),),
            pointer_values=((10, 0),),
            pressed_names=("w",),
        )

        sample = backend.poll()
        self.assertTrue(sample.w)
        self.assertTrue(sample.camera_dragging)
        self.assertEqual(sample.mouse_dx, 12.0)
        snapshot = MODULE.build_snapshot(
            sequence=2,
            timestamp_monotonic_s=1.01,
            keyboard=sample,
            gamepad=MODULE.GamepadSample(),
            input_source="keyboard",
            camera_yaw_rad=0.1,
            camera_available=True,
        )
        self.assertFalse(snapshot.focused)

        core = CORE.GameControlCore(
            CORE.ControlConfig(
                max_acceleration_mps2=100.0,
                max_deceleration_mps2=100.0,
                max_step_s=1.0,
            )
        )
        core.accept_snapshot(
            CORE.InputSnapshot(
                sequence=1,
                timestamp_monotonic_s=1.0,
                focused=True,
                camera_yaw_rad=0.0,
                keys=CORE.KeySnapshot(
                    False, False, False, False, False, False, False
                ),
                move_stick=CORE.MoveStickSnapshot(0.0, 0.0),
            ),
            received_at_s=1.0,
        )
        core.accept_snapshot(snapshot, received_at_s=1.01)
        command = core.command(now_s=1.01, dt_s=0.01)
        self.assertTrue(command.safe_stop)
        self.assertEqual(command.reason, "focus_lost")
        self.assertEqual(command.speed_mps, 0.0)
        self.assertEqual(command.locomotion_mode, CORE.SONIC_IDLE_MODE)

        def focused_snapshot(
            sequence: int,
            timestamp: float,
            *,
            w: bool = False,
            ctrl: bool = False,
            shift: bool = False,
        ) -> CORE.InputSnapshot:
            return MODULE.build_snapshot(
                sequence=sequence,
                timestamp_monotonic_s=timestamp,
                keyboard=MODULE.KeyboardMouseSample(
                    w=w,
                    ctrl=ctrl,
                    shift=shift,
                    focused=True,
                ),
                gamepad=MODULE.GamepadSample(),
                input_source="keyboard",
                camera_yaw_rad=0.1,
                camera_available=True,
            )

        held_w = focused_snapshot(3, 1.02, w=True)
        core.accept_snapshot(held_w, received_at_s=1.02)
        still_stopped = core.command(now_s=1.02, dt_s=1.0)
        self.assertTrue(still_stopped.safe_stop)
        self.assertEqual(still_stopped.reason, "awaiting_neutral")
        self.assertEqual(still_stopped.locomotion_mode, CORE.SONIC_IDLE_MODE)

        neutral = focused_snapshot(4, 1.03)
        core.accept_snapshot(neutral, received_at_s=1.03)
        neutral_command = core.command(now_s=1.03, dt_s=1.0)
        self.assertFalse(neutral_command.safe_stop)
        self.assertEqual(neutral_command.locomotion_mode, CORE.SONIC_IDLE_MODE)
        for sequence, timestamp, modifiers, expected_mode, expected_speed in (
            (5, 1.04, {"ctrl": True}, CORE.SONIC_SLOW_WALK_MODE, 0.10),
            (6, 1.05, {}, CORE.SONIC_WALK_MODE, 0.80),
            (7, 1.06, {"shift": True}, CORE.SONIC_RUN_MODE, 2.50),
        ):
            core.accept_snapshot(
                focused_snapshot(sequence, timestamp, w=True, **modifiers),
                received_at_s=timestamp,
            )
            resumed = core.command(now_s=timestamp, dt_s=1.0)
            self.assertFalse(resumed.safe_stop)
            self.assertEqual(resumed.locomotion_mode, expected_mode)
            self.assertAlmostEqual(resumed.speed_mps, expected_speed)

    def test_completed_raw_click_without_motion_still_interlocks(self) -> None:
        backend = self._raw_backend(
            focus_results=iter(((True, "Matrix", frozenset({1234})),)),
            raw_deltas=((0.0, 0.0, True),),
            pointer_values=((10, 0),),
        )

        sample = backend.poll()

        self.assertTrue(sample.camera_dragging)
        self.assertEqual((sample.mouse_dx, sample.mouse_dy), (0.0, 0.0))

    def test_teleport_is_rejected_and_rebaselined_without_clamping(self) -> None:
        samples = iter(
            (
                (0, 0),
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
        backend._raw_motion = None
        backend._absolute_motion = MODULE.X11AbsoluteDragAccumulator(200.0)
        backend._expected_ue_pid = 1234
        backend._focus_identity = lambda: (True, "Matrix", frozenset({1234}))

        baseline = backend.poll()
        pressed = backend.poll()
        exact_boundary = backend.poll()
        teleport = backend.poll()
        after_rebaseline = backend.poll()

        self.assertEqual(baseline.mouse_dx, 0.0)
        self.assertEqual(pressed.mouse_dx, 0.0)
        self.assertEqual(exact_boundary.mouse_dx, 200.0)
        self.assertEqual(teleport.mouse_dx, 0.0)
        self.assertEqual(after_rebaseline.mouse_dx, 5.0)
        telemetry = backend.pointer_telemetry
        self.assertEqual(telemetry["motion_source"], "x11-absolute-root-delta")
        self.assertEqual(telemetry["teleport_rejections"], 1)
        self.assertEqual(telemetry["last_teleport_delta"], [201, 0])
        self.assertEqual(telemetry["accepted_dx_total"], 205.0)
        self.assertEqual(telemetry["drop_reason_counts"], {"teleport_rejected": 1})
        self.assertEqual(telemetry["dropped_motion_events"], 1)
        self.assertEqual(telemetry["dropped_dx_total"], 201.0)
        self.assertEqual(telemetry["focus_badwindow_recoveries"], 0)
        self.assertIsNone(telemetry["last_focus_badwindow_resource"])

    def test_release_sample_keeps_final_held_drag_delta(self) -> None:
        samples = iter(((0, 0), (0, 1 << 8), (10, 1 << 8), (30, 0)))

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
        backend._raw_motion = None
        backend._absolute_motion = MODULE.X11AbsoluteDragAccumulator(200.0)
        backend._expected_ue_pid = 1234
        backend._focus_identity = lambda: (True, "Matrix", frozenset({1234}))

        baseline = backend.poll()
        pressed = backend.poll()
        held = backend.poll()
        released = backend.poll()

        self.assertEqual(baseline.mouse_dx, 0.0)
        self.assertEqual(pressed.mouse_dx, 0.0)
        self.assertEqual(held.mouse_dx, 10.0)
        self.assertEqual(released.mouse_dx, 20.0)
        self.assertTrue(pressed.camera_dragging)
        self.assertTrue(held.camera_dragging)
        self.assertTrue(released.camera_dragging)

    def test_absolute_focus_loss_requires_release_and_fresh_press(self) -> None:
        accumulator = MODULE.X11AbsoluteDragAccumulator(200.0)
        self.assertEqual(
            accumulator.update(
                pointer=(0, 0), current_look_pressed=False, focused=True
            ),
            (0.0, 0.0, False),
        )
        accumulator.update(
            pointer=(0, 0), current_look_pressed=True, focused=True
        )
        self.assertEqual(
            accumulator.update(
                pointer=(10, 0), current_look_pressed=True, focused=True
            ),
            (10.0, 0.0, True),
        )
        self.assertEqual(
            accumulator.update(
                pointer=(20, 0), current_look_pressed=True, focused=False
            ),
            (0.0, 0.0, True),
        )
        self.assertEqual(
            accumulator.update(
                pointer=(30, 0), current_look_pressed=True, focused=True
            ),
            (0.0, 0.0, True),
        )
        self.assertEqual(
            accumulator.update(
                pointer=(30, 0), current_look_pressed=False, focused=True
            ),
            (0.0, 0.0, False),
        )
        accumulator.update(
            pointer=(30, 0), current_look_pressed=True, focused=True
        )
        self.assertEqual(
            accumulator.update(
                pointer=(35, 0), current_look_pressed=True, focused=True
            ),
            (5.0, 0.0, True),
        )
        self.assertEqual(accumulator.accepted_dx_total, 15.0)
        self.assertEqual(
            accumulator.drop_reason_counts,
            {"focus_lost": 1, "awaiting_release_before_fresh_press": 1},
        )
        self.assertEqual(accumulator.dropped_motion_events, 1)
        self.assertEqual(accumulator.dropped_dx_total, 10.0)

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

        left_ctrl_right_shift = backend.poll()
        pressed_codes.clear()
        pressed_codes.update(
            {
                backend._keycodes["ctrl_right"],
                backend._keycodes["shift_left"],
            }
        )
        right_ctrl_left_shift = backend.poll()

        for sample in (left_ctrl_right_shift, right_ctrl_left_shift):
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


@unittest.skipUnless(
    os.environ.get("MATRIX_RUN_X11_BADWINDOW_INTEGRATION") == "1",
    "set MATRIX_RUN_X11_BADWINDOW_INTEGRATION=1 under Xvfb",
)
class X11BadWindowIntegrationTest(unittest.TestCase):
    def test_destroyed_focused_window_does_not_exit_provider(self) -> None:
        library_name = ctypes.util.find_library("X11")
        if not library_name or not os.environ.get("DISPLAY"):
            self.skipTest("an X11 display and libX11 are required")
        x11 = ctypes.CDLL(library_name)
        signatures = {
            "XOpenDisplay": ([ctypes.c_char_p], ctypes.c_void_p),
            "XDefaultRootWindow": ([ctypes.c_void_p], ctypes.c_ulong),
            "XCreateSimpleWindow": (
                [
                    ctypes.c_void_p,
                    ctypes.c_ulong,
                    ctypes.c_int,
                    ctypes.c_int,
                    ctypes.c_uint,
                    ctypes.c_uint,
                    ctypes.c_uint,
                    ctypes.c_ulong,
                    ctypes.c_ulong,
                ],
                ctypes.c_ulong,
            ),
            "XMapWindow": (
                [ctypes.c_void_p, ctypes.c_ulong],
                ctypes.c_int,
            ),
            "XSetInputFocus": (
                [
                    ctypes.c_void_p,
                    ctypes.c_ulong,
                    ctypes.c_int,
                    ctypes.c_ulong,
                ],
                ctypes.c_int,
            ),
            "XDestroyWindow": (
                [ctypes.c_void_p, ctypes.c_ulong],
                ctypes.c_int,
            ),
            "XSync": (
                [ctypes.c_void_p, ctypes.c_int],
                ctypes.c_int,
            ),
            "XCloseDisplay": ([ctypes.c_void_p], ctypes.c_int),
        }
        for name, (argtypes, restype) in signatures.items():
            function = getattr(x11, name)
            function.argtypes = argtypes
            function.restype = restype

        creator = x11.XOpenDisplay(None)
        self.assertTrue(creator)
        backend = None
        window = 0
        try:
            root = x11.XDefaultRootWindow(creator)
            window = int(
                x11.XCreateSimpleWindow(
                    creator, root, 0, 0, 100, 100, 0, 0, 0
                )
            )
            self.assertGreater(window, 1)
            x11.XMapWindow(creator, window)
            x11.XSync(creator, 0)
            # RevertToParent=2, CurrentTime=0.
            x11.XSetInputFocus(creator, window, 2, 0)
            x11.XSync(creator, 0)

            backend = MODULE.X11KeyboardMouse(
                display_name=os.environ["DISPLAY"],
                focus_title_pattern=None,
                expected_ue_pid=None,
                look_button="left",
            )
            original_fetch_name = backend._fetch_name
            destroyed = False

            def destroy_then_query(focused_window: int):
                nonlocal destroyed
                if not destroyed:
                    self.assertEqual(focused_window, window)
                    x11.XDestroyWindow(creator, window)
                    x11.XSync(creator, 0)
                    destroyed = True
                return original_fetch_name(focused_window)

            backend._fetch_name = destroy_then_query

            self.assertEqual(
                backend._focus_identity(),
                (False, None, frozenset()),
            )
            self.assertTrue(destroyed)
            self.assertEqual(
                backend.pointer_telemetry["focus_badwindow_recoveries"], 1
            )
            self.assertEqual(
                backend.pointer_telemetry["last_focus_badwindow_resource"],
                window,
            )
        finally:
            if backend is not None:
                backend.close()
            x11.XCloseDisplay(creator)


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


class ProviderCleanupTest(unittest.TestCase):
    def test_failures_and_broken_logger_never_skip_resources_or_signals(self) -> None:
        events: list[str] = []

        def failing_step(label: str):
            def run() -> None:
                events.append(label)
                raise OSError(f"{label} failed")

            return run

        def resource(label: str, *, fail: bool = False):
            instance = mock.Mock()

            def close() -> None:
                events.append(label)
                if fail:
                    raise RuntimeError(f"{label} failed")

            instance.close.side_effect = close
            return instance

        gamepad = resource("gamepad")
        overlay = resource("overlay", fail=True)
        x11 = resource("x11")
        publisher = resource("publisher")
        external = resource("external")
        restored: list[int] = []

        def restore(signum: int, _handler: object) -> None:
            restored.append(signum)
            events.append(f"signal:{signum}")
            if signum == signal.SIGINT:
                raise OSError("SIGINT restore failed")

        cleanup = MODULE._CleanupCoordinator()
        with mock.patch("builtins.print", side_effect=OSError("stderr closed")), mock.patch.object(
            MODULE.signal,
            "signal",
            side_effect=restore,
        ):
            cleanup.run("publisher_release", failing_step("release"))
            cleanup.run("command_receipt", failing_step("receipt"))
            MODULE._close_provider_resources(
                cleanup,
                gamepad=gamepad,
                overlay=overlay,
                x11=x11,
                publisher=publisher,
                external_control=external,
                previous_handlers={
                    signal.SIGINT: object(),
                    signal.SIGTERM: object(),
                },
            )
            cleanup.run("status_write", failing_step("status"))

        for owned in (gamepad, overlay, x11, publisher, external):
            owned.close.assert_called_once_with()
        self.assertEqual(restored, [signal.SIGINT, signal.SIGTERM])
        self.assertEqual(
            [failure["step"] for failure in cleanup.failures],
            [
                "publisher_release",
                "command_receipt",
                "overlay_close",
                "signal_restore_SIGINT",
                "status_write",
            ],
        )
        return_code, exit_reason = MODULE._cleanup_outcome(
            cleanup,
            return_code=0,
            exit_reason="signal",
        )
        self.assertEqual(return_code, 1)
        self.assertEqual(exit_reason, "cleanup_error:publisher_release")
        self.assertEqual(
            events,
            [
                "release",
                "receipt",
                "gamepad",
                "overlay",
                "x11",
                "publisher",
                "external",
                f"signal:{signal.SIGINT}",
                f"signal:{signal.SIGTERM}",
                "status",
            ],
        )

    def test_status_write_failure_alone_is_a_nonzero_cleanup_outcome(self) -> None:
        cleanup = MODULE._CleanupCoordinator()
        with mock.patch("builtins.print"):
            cleanup.run(
                "status_write",
                lambda: (_ for _ in ()).throw(OSError("disk full")),
            )

        self.assertEqual(
            MODULE._cleanup_outcome(
                cleanup,
                return_code=0,
                exit_reason="max_seconds",
            ),
            (1, "cleanup_error:status_write"),
        )


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
            MODULE._JS_EVENT.pack(4, 1, MODULE._JS_EVENT_BUTTON, 0),
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
        self.assertTrue(sample.buttons_pressed)
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


class CameraYawSourceCliTest(unittest.TestCase):
    def test_provider_parser_accepts_an_open_game_command_fd(self) -> None:
        provider, runtime = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        try:
            with mock.patch.object(
                os.sys,
                "argv",
                [
                    "matrix_game_control_input.py",
                    "--game-command-fd",
                    str(provider.fileno()),
                    "--dry-run",
                ],
            ):
                args = MODULE._parse_args()
            MODULE._validate_args(args)
            self.assertEqual(args.game_command_fd, provider.fileno())
        finally:
            provider.close()
            runtime.close()

    def test_provider_external_control_endpoint_is_all_or_none_and_bounded(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            base = [
                "matrix_game_control_input.py",
                "--external-control-socket",
                os.fspath(root / "control.sock"),
                "--external-control-capability-file",
                os.fspath(root / "control.cap"),
                "--external-control-deadman-seconds",
                "0.10",
                "--dry-run",
            ]
            with mock.patch.object(os.sys, "argv", base):
                args = MODULE._parse_args()
            MODULE._validate_args(args)
            self.assertEqual(args.external_control_deadman_seconds, 0.10)

            with mock.patch.object(
                os.sys,
                "argv",
                [
                    "matrix_game_control_input.py",
                    "--external-control-socket",
                    os.fspath(root / "control.sock"),
                    "--dry-run",
                ],
            ):
                incomplete = MODULE._parse_args()
            with self.assertRaisesRegex(SystemExit, "all-or-none"):
                MODULE._validate_args(incomplete)

            too_slow = list(base)
            too_slow[too_slow.index("0.10")] = "0.16"
            with mock.patch.object(os.sys, "argv", too_slow):
                invalid = MODULE._parse_args()
            with self.assertRaisesRegex(SystemExit, r"\[0.01, 0.15\]"):
                MODULE._validate_args(invalid)

            collision = list(base)
            collision[
                collision.index(os.fspath(root / "control.sock"))
            ] = os.fspath(MODULE.DEFAULT_SOCKET)
            with mock.patch.object(os.sys, "argv", collision):
                conflicting = MODULE._parse_args()
            with self.assertRaisesRegex(SystemExit, "distinct"):
                MODULE._validate_args(conflicting)

    def test_provider_validation_rejects_a_closed_game_command_fd(self) -> None:
        provider, runtime = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        descriptor = provider.fileno()
        provider.close()
        self.addCleanup(runtime.close)
        with mock.patch.object(
            os.sys,
            "argv",
            [
                "matrix_game_control_input.py",
                "--game-command-fd",
                str(descriptor),
                "--dry-run",
            ],
        ):
            args = MODULE._parse_args()
        with self.assertRaisesRegex(SystemExit, "not open"):
            MODULE._validate_args(args)

    def test_provider_parser_keeps_three_x11_sources_distinct(self) -> None:
        for source in ("x11-mirror", "x11-core-gated", "x11-absolute"):
            with self.subTest(source=source), mock.patch.object(
                os.sys,
                "argv",
                ["matrix_game_control_input.py", "--camera-yaw-source", source],
            ):
                args = MODULE._parse_args()
                self.assertEqual(args.camera_yaw_source, source)

    def test_provider_parser_accepts_final_pov_state_file(self) -> None:
        with mock.patch.object(
            os.sys,
            "argv",
            [
                "matrix_game_control_input.py",
                "--camera-yaw-source",
                "ue-final-pov",
                "--ue-camera-state-file",
                "/run/user/1000/camera-state.bin",
                "--expected-ue-pid",
                "4242",
                "--dry-run",
            ],
        ):
            args = MODULE._parse_args()
        MODULE._validate_args(args)
        self.assertEqual(args.camera_yaw_source, "ue-final-pov")
        self.assertEqual(
            args.ue_camera_state_file,
            Path("/run/user/1000/camera-state.bin"),
        )

    def test_final_pov_requires_state_file_and_exact_pid(self) -> None:
        with mock.patch.object(
            os.sys,
            "argv",
            [
                "matrix_game_control_input.py",
                "--camera-yaw-source",
                "ue-final-pov",
                "--dry-run",
            ],
        ):
            args = MODULE._parse_args()
        with self.assertRaisesRegex(SystemExit, "expected-ue-pid"):
            MODULE._validate_args(args)


if __name__ == "__main__":
    unittest.main()
