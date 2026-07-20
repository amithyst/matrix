from __future__ import annotations

import builtins
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import unittest
from unittest import mock


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if os.fspath(SCRIPTS) not in sys.path:
    sys.path.insert(0, os.fspath(SCRIPTS))

import matrix_calibration_overlay as OVERLAY  # noqa: E402
import matrix_game_control_input as PROVIDER  # noqa: E402
import matrix_mc_commands as COMMANDS  # noqa: E402
import matrix_world_state as WORLD_STATE  # noqa: E402
import run_matrix_sonic as RUNTIME  # noqa: E402


class MatrixGameCommandEndToEndTest(unittest.TestCase):
    WORLD_ID = "town10:e2e"
    WORLD_REVISION = "a" * 64

    def setUp(self) -> None:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.state_path = Path(temporary.name) / "world-state.json"

        self.supervisor = PROVIDER.CalibrationOverlaySupervisor(
            state_file=Path(temporary.name) / "overlay-state.json",
            display_name=None,
            expected_ue_pid=os.getpid(),
        )
        intent_provider, intent_overlay = socket.socketpair(
            socket.AF_UNIX,
            socket.SOCK_SEQPACKET,
        )
        intent_provider.setblocking(False)
        self.supervisor._action_socket = intent_provider
        self.publisher = OVERLAY.PointerActionPublisher(
            file_descriptor=intent_overlay.detach(),
            session=self.supervisor._action_session,
        )

        command_provider, command_runtime = socket.socketpair(
            socket.AF_UNIX,
            socket.SOCK_SEQPACKET,
        )
        self.assertEqual(command_provider.family, socket.AF_UNIX)
        self.assertEqual(
            command_provider.getsockopt(socket.SOL_SOCKET, socket.SO_TYPE),
            socket.SOCK_SEQPACKET,
        )
        self.client = PROVIDER.GameCommandClient(command_provider.detach())
        self.world = RUNTIME._GameWorldStateRuntime(
            path=self.state_path,
            world_id=self.WORLD_ID,
            world_revision=self.WORLD_REVISION,
            checkpoint_seconds=0.75,
        )
        self.runtime = RUNTIME.GameCommandRuntime(command_runtime, self.world)

        self.addCleanup(self.runtime.close)
        self.addCleanup(self.client.close)
        self.addCleanup(self.publisher.close)
        self.addCleanup(self.supervisor.close)

    def _enter_editor_from_overlay(self) -> None:
        self.publisher.publish_command_edit(True)
        self.assertEqual(
            self.supervisor.drain_intents(),
            (PROVIDER.OverlayIntent(kind="command_edit", active=True),),
        )
        self.assertTrue(
            self.client.set_editing(
                True,
                panel_active=True,
                restart_requested=False,
            )
        )

    def _submit_from_overlay(
        self,
        command_text: str,
        expected_ast_type: type,
    ) -> COMMANDS.GameCommandRequest:
        self.publisher.publish_command_submit(command_text)

        assert self.supervisor._action_socket is not None
        raw_intent = self.supervisor._action_socket.recv(
            self.supervisor._MAX_INTENT_PACKET_BYTES + 1,
            socket.MSG_PEEK,
        )
        command_name = command_text.split(maxsplit=1)[0].encode("ascii")
        self.assertIn(command_name, raw_intent)
        self.assertEqual(
            self.supervisor.drain_intents(),
            (PROVIDER.OverlayIntent(kind="command_submit", command=command_text),),
        )

        self.assertTrue(
            self.client.submit(
                command_text,
                calibration_active=True,
                neutral_frame_ready=True,
                restart_requested=False,
            )
        )
        typed_packet = self.runtime.connection.recv(
            COMMANDS.MAX_COMMAND_PACKET_BYTES + 1,
            socket.MSG_PEEK,
        )
        self.assertNotIn(command_name, typed_packet)
        request = COMMANDS.decode_command_request(typed_packet)
        self.assertIsInstance(request.command, expected_ast_type)
        self.assertEqual(request.request_id, self.client.last_request_id)
        return request

    def _reload_state(self) -> WORLD_STATE.MatrixWorldState:
        return WORLD_STATE.WorldStateStore(
            self.state_path,
            world_id=self.WORLD_ID,
            world_revision=self.WORLD_REVISION,
        ).load()

    def test_overlay_provider_runtime_persistence_and_restart_response(self) -> None:
        current_pose = WORLD_STATE.WorldPose(10.0, 20.0, 0.8, 0.5)

        # A command must remain on the private sockets and typed dispatcher;
        # invoking a shell, subprocess, or eval is a test failure.
        forbidden = AssertionError("game command escaped the typed socket pipeline")
        with (
            mock.patch.object(builtins, "eval", side_effect=forbidden),
            mock.patch.object(os, "system", side_effect=forbidden),
            mock.patch.object(subprocess, "Popen", side_effect=forbidden),
        ):
            self._enter_editor_from_overlay()
            summon_request = self._submit_from_overlay(
                '/summon matrix:teleport_point ~1 ~-2 ~ {Tags:["XX"]}',
                COMMANDS.SummonTeleportPoint,
            )
            self.assertEqual(summon_request.sequence, 1)
            self.assertFalse(
                self.runtime.poll(current_pose=current_pose, command_allowed=True)
            )
            self.assertTrue(self.client.poll())

            summon_result = self.client.mapping()
            self.assertEqual(summon_result["status"], "success")
            self.assertEqual(summon_result["code"], "OK_SUMMONED")
            self.assertIs(summon_result["restart_required"], False)
            self.assertFalse(self.runtime.restart_requested)
            self.assertEqual(summon_result["data"]["position"], [11.0, 18.0, 0.8])

            after_summon = self._reload_state()
            self.assertEqual(len(after_summon.teleport_points), 1)
            point = after_summon.teleport_points[0]
            self.assertEqual(point.tags, ("XX",))
            self.assertEqual(point.pose, WORLD_STATE.WorldPose(11.0, 18.0, 0.8, 0.5))
            self.assertIsNone(after_summon.last_exit)

            teleport_request = self._submit_from_overlay(
                "/tp @s @e["
                "type=matrix:teleport_point,tag=XX,limit=1,sort=nearest]",
                COMMANDS.TeleportSelector,
            )
            self.assertEqual(teleport_request.sequence, 2)
            self.assertTrue(
                self.runtime.poll(current_pose=current_pose, command_allowed=True)
            )
            self.assertTrue(self.client.poll())

        teleport_result = self.client.mapping()
        self.assertEqual(teleport_result["status"], "restarting")
        self.assertEqual(teleport_result["code"], "OK_TELEPORT_RESTART")
        self.assertIs(teleport_result["restart_required"], True)
        self.assertTrue(self.runtime.restart_requested)
        self.assertEqual(teleport_result["data"]["entity_id"], point.entity_id)

        after_teleport = self._reload_state()
        self.assertEqual(
            after_teleport.last_exit,
            WORLD_STATE.WorldPose(11.0, 18.0, 0.8, 0.5),
        )
        self.assertEqual(after_teleport.resume_source, "teleport_command")
        self.assertEqual(len(after_teleport.teleport_points), 1)
        self.assertEqual(self.runtime.commands_executed, 2)
        self.assertEqual(self.runtime.requests_received, 2)

    def test_malformed_typed_request_fails_closed_without_permanent_pending(self) -> None:
        self._enter_editor_from_overlay()
        with mock.patch.object(PROVIDER, "encode_command_request", return_value=b"{}"):
            self.assertTrue(
                self.client.submit(
                    "/tp @s 1 2 3",
                    calibration_active=True,
                    neutral_frame_ready=True,
                    restart_requested=False,
                )
            )
        request_id = self.client.last_request_id
        self.assertIsNotNone(request_id)
        self.assertTrue(self.client.in_flight)

        with self.assertRaisesRegex(RuntimeError, "invalid game command request"):
            self.runtime.poll(
                current_pose=WORLD_STATE.WorldPose(10.0, 20.0, 0.8, 0.5),
                command_allowed=True,
            )
        self.assertEqual(self.runtime.protocol_errors, 1)
        self.assertEqual(self.runtime.commands_executed, 0)
        self.assertFalse(self.state_path.exists())

        # Production reaches this close through the main-loop exception and
        # cleanup boundary.  EOF must resolve the provider's local pending latch
        # without guessing whether a successfully sent record was executed.
        self.runtime.close()
        self.assertTrue(self.client.poll())
        self.assertFalse(self.client.in_flight)
        self.assertFalse(self.client.available)
        self.assertTrue(self.client.outcome_unknown)
        self.assertEqual(self.client.last_request_id, request_id)
        self.assertEqual(self.client.code, "E_COMMAND_OUTCOME_UNKNOWN")


if __name__ == "__main__":
    unittest.main()
