from __future__ import annotations

import json
import os
from pathlib import Path
import socket
import stat
import tempfile
import unittest
from unittest import mock


import sys

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if os.fspath(SCRIPTS) not in sys.path:
    sys.path.insert(0, os.fspath(SCRIPTS))

import matrix_external_control as MODULE  # noqa: E402


def neutral_mapping() -> dict[str, object]:
    return MODULE.ExternalInputState.neutral().to_mapping()


class Clock:
    def __init__(self, value: float = 10.0) -> None:
        self.value = value

    def __call__(self) -> float:
        return self.value


class BrokerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        root = Path(self.temporary.name)
        self.clock = Clock()
        self.broker = MODULE.ExternalControlBroker(
            root / "control.sock",
            root / "control.cap",
            clock=self.clock,
        )
        self.broker.open()
        self.clients: list[socket.socket] = []

    def tearDown(self) -> None:
        for client in self.clients:
            client.close()
        self.broker.close()
        self.temporary.cleanup()

    def connect(self) -> socket.socket:
        client = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        client.settimeout(1.0)
        client.connect(os.fspath(self.broker.path))
        self.clients.append(client)
        self.broker.poll(now=self.clock.value)
        return client

    def request(
        self,
        client: socket.socket,
        sequence: int,
        operation: str,
        payload: dict[str, object],
        *,
        capability: str | None = None,
    ) -> dict[str, object]:
        packet = {
            "protocol": MODULE.PROTOCOL,
            "kind": "request",
            "sequence": sequence,
            "capability": self.broker.capability if capability is None else capability,
            "operation": operation,
            "payload": payload,
        }
        client.send(json.dumps(packet, separators=(",", ":")).encode())
        self.broker.poll(now=self.clock.value)
        return json.loads(client.recv(MODULE.MAX_PACKET_BYTES))

    def acquire(self, client: socket.socket, sequence: int = 1) -> str:
        response = self.request(client, sequence, "lease.acquire", {})
        self.assertTrue(response["ok"], response)
        self.assertEqual(response["code"], "OK_LEASE")
        return response["data"]["lease_id"]

    def test_endpoint_and_capability_are_private_and_removed_on_close(self) -> None:
        self.assertTrue(stat.S_ISSOCK(self.broker.path.stat().st_mode))
        self.assertEqual(stat.S_IMODE(self.broker.path.stat().st_mode), 0o600)
        self.assertEqual(
            stat.S_IMODE(self.broker.capability_file.stat().st_mode), 0o600
        )
        capability = self.broker.capability_file.read_text(encoding="ascii").strip()
        self.assertEqual(capability, self.broker.capability)
        self.assertRegex(capability, r"^[0-9a-f]{64}$")
        self.broker.close()
        self.assertFalse(self.broker.path.exists())
        self.assertFalse(self.broker.capability_file.exists())

    def test_full_state_replace_and_relative_mouse_delta_is_consumed_once(self) -> None:
        client = self.connect()
        lease = self.acquire(client)
        state = neutral_mapping()
        state["keyboard"]["w"] = True
        state["keyboard"]["alt"] = True
        state["mouse"]["buttons"]["left"] = True
        state["mouse"]["dx"] = 12.5
        state["mouse"]["dy"] = -3.0
        state["gamepad"]["connected"] = True
        state["gamepad"]["axes"]["right"] = 0.25
        response = self.request(
            client,
            2,
            "input.replace",
            {"lease_id": lease, "state": state},
        )
        self.assertTrue(response["ok"])

        first = self.broker.sample(now=self.clock.value)
        second = self.broker.sample(now=self.clock.value)
        self.assertTrue(first.keyboard["w"])
        self.assertTrue(first.keyboard["alt"])
        self.assertTrue(first.mouse_buttons["left"])
        self.assertEqual((first.mouse_dx, first.mouse_dy), (12.5, -3.0))
        self.assertEqual((second.mouse_dx, second.mouse_dy), (0.0, 0.0))
        self.assertTrue(second.keyboard["w"])
        self.assertEqual(second.gamepad_axes["right"], 0.25)

    def test_deadman_revokes_lease_zeros_state_and_allows_another_client(self) -> None:
        first = self.connect()
        lease = self.acquire(first)
        state = neutral_mapping()
        state["keyboard"]["w"] = True
        self.request(first, 2, "input.replace", {"lease_id": lease, "state": state})

        self.clock.value += 0.149
        self.assertTrue(self.broker.sample(now=self.clock.value).keyboard["w"])
        self.clock.value += 0.001
        self.assertFalse(self.broker.sample(now=self.clock.value).keyboard["w"])
        self.assertFalse(self.broker.lease_active)
        self.assertEqual(self.broker.deadman_stops, 1)

        second = self.connect()
        response = self.request(second, 1, "lease.acquire", {})
        self.assertTrue(response["ok"])

    def test_single_lease_conflict_release_and_disconnect(self) -> None:
        first = self.connect()
        second = self.connect()
        lease = self.acquire(first)
        conflict = self.request(second, 1, "lease.acquire", {})
        self.assertFalse(conflict["ok"])
        self.assertEqual(conflict["code"], "E_LEASE_BUSY")

        released = self.request(first, 2, "lease.release", {"lease_id": lease})
        self.assertTrue(released["ok"])
        second_lease = self.acquire(second, sequence=2)
        self.assertTrue(second_lease)
        second.close()
        self.broker.poll(now=self.clock.value)
        self.assertFalse(self.broker.lease_active)

    def test_command_queue_is_bounded_typed_and_cleared_by_local_override(self) -> None:
        client = self.connect()
        lease = self.acquire(client)
        response = self.request(
            client,
            2,
            "command.submit",
            {"lease_id": lease, "command": "/tp @s ~ ~ ~"},
        )
        self.assertTrue(response["ok"])
        command = self.broker.drain_commands()[0]
        self.assertEqual(command.command, "/tp @s ~ ~ ~")
        self.assertEqual(command.request_sequence, 2)
        self.assertEqual(command.peer_pid, os.getpid())

        self.request(
            client,
            3,
            "command.submit",
            {"lease_id": lease, "command": "/tp @s ^ ^ ^1"},
        )
        self.broker.local_override("physical_keyboard")
        self.assertEqual(self.broker.drain_commands(), ())
        self.assertFalse(self.broker.lease_active)
        self.assertEqual(self.broker.local_overrides, 1)

    def test_command_receipt_covers_admission_terminal_result_and_cancellation(self) -> None:
        client = self.connect()
        lease = self.acquire(client)
        queued = self.request(
            client,
            2,
            "command.submit",
            {"lease_id": lease, "command": "/tp @s ~ ~ ~"},
        )
        command_id = queued["data"]["command_id"]
        command = self.broker.drain_commands(limit=1)[0]
        self.assertEqual(command.command_id, command_id)
        admitted = self.request(
            client,
            3,
            "command.result",
            {"command_id": command_id},
        )["data"]
        self.assertEqual(admitted["state"], "admitted")
        self.assertFalse(admitted["terminal"])
        self.broker.complete_command(
            command,
            {
                "ok": True,
                "outcome_unknown": False,
                "code": "OK_TELEPORT_RESTART",
                "message": "saved",
            },
        )
        completed = self.request(
            client,
            4,
            "command.result",
            {"command_id": command_id},
        )["data"]
        self.assertEqual(completed["state"], "completed")
        self.assertTrue(completed["terminal"])
        self.assertTrue(completed["result"]["ok"])

        queued = self.request(
            client,
            5,
            "command.submit",
            {"lease_id": lease, "command": "/tp @s ^ ^ ^1"},
        )
        cancelled_id = queued["data"]["command_id"]
        self.broker.local_override("physical_keyboard")
        cancelled = self.request(
            client,
            6,
            "command.result",
            {"command_id": cancelled_id},
        )["data"]
        self.assertEqual(cancelled["state"], "cancelled")
        self.assertTrue(cancelled["terminal"])
        self.assertTrue(cancelled["authority_revoked"])

    def test_lease_renew_never_silently_reacquires_expired_authority(self) -> None:
        client = self.connect()
        lease = self.acquire(client)
        renewed = self.request(
            client,
            2,
            "lease.renew",
            {"lease_id": lease},
        )
        self.assertTrue(renewed["ok"])
        self.assertEqual(renewed["data"]["lease_id"], lease)
        epoch = renewed["data"]["authority_epoch"]
        self.clock.value += 0.151
        expired = self.request(
            client,
            3,
            "lease.renew",
            {"lease_id": lease},
        )
        self.assertFalse(expired["ok"])
        self.assertEqual(expired["code"], "E_LEASE")
        self.assertFalse(self.broker.lease_active)
        self.assertEqual(self.broker.stale_lease_rejections, 1)
        self.assertEqual(self.broker.protocol_errors, 0)
        reacquired = self.request(client, 4, "lease.acquire", {})
        self.assertTrue(reacquired["ok"])
        self.assertNotEqual(reacquired["data"]["lease_id"], lease)
        self.assertGreater(reacquired["data"]["authority_epoch"], epoch)

    def test_admitted_command_can_finish_with_explicit_unknown_outcome(self) -> None:
        client = self.connect()
        lease = self.acquire(client)
        queued = self.request(
            client,
            2,
            "command.submit",
            {"lease_id": lease, "command": "/tp @s ~ ~ ~"},
        )
        command = self.broker.drain_commands(limit=1)[0]
        self.broker.complete_command(
            command,
            {
                "ok": None,
                "outcome_unknown": True,
                "code": "E_COMMAND_OUTCOME_UNKNOWN",
                "message": "runtime channel closed",
            },
        )
        receipt = self.request(
            client,
            3,
            "command.result",
            {"command_id": queued["data"]["command_id"]},
        )["data"]
        self.assertEqual(receipt["state"], "outcome_unknown")
        self.assertTrue(receipt["terminal"])
        self.assertIsNone(receipt["result"]["ok"])

    def test_data_modify_updates_only_whitelisted_input_under_active_lease(self) -> None:
        client = self.connect()
        self.acquire(client)
        self.broker.apply_data_modify(
            "control.input.keyboard.w", True, now=self.clock.value
        )
        self.broker.apply_data_modify(
            "control.input.gamepad.right", -0.5, now=self.clock.value
        )
        state = self.broker.sample(now=self.clock.value)
        self.assertTrue(state.keyboard["w"])
        self.assertTrue(state.gamepad_connected)
        self.assertEqual(state.gamepad_axes["right"], -0.5)
        with self.assertRaises(MODULE.ExternalControlError):
            self.broker.apply_data_modify(
                "control.input.keyboard.space", True, now=self.clock.value
            )
        self.broker.local_override("test")
        with self.assertRaisesRegex(MODULE.ExternalControlError, "lease"):
            self.broker.apply_data_modify(
                "control.input.keyboard.w", True, now=self.clock.value
            )

    def test_auth_sequence_schema_and_strict_json_fail_closed(self) -> None:
        client = self.connect()
        auth = self.request(
            client, 1, "lease.acquire", {}, capability="0" * 64
        )
        self.assertFalse(auth["ok"])
        self.assertEqual(auth["code"], "E_AUTH")
        lease = self.acquire(client, sequence=2)
        replay = self.request(client, 2, "status.get", {})
        self.assertFalse(replay["ok"])
        self.assertEqual(replay["code"], "E_SEQUENCE")

        client.send(
            b'{"protocol":"matrix-external-control/v1","protocol":"x",'
            b'"kind":"request","sequence":3,"capability":"x",'
            b'"operation":"status.get","payload":{}}'
        )
        self.broker.poll(now=self.clock.value)
        duplicate = json.loads(client.recv(MODULE.MAX_PACKET_BYTES))
        self.assertFalse(duplicate["ok"])
        self.assertEqual(duplicate["code"], "E_JSON_DUPLICATE")

        malformed_state = neutral_mapping()
        del malformed_state["keyboard"]["w"]
        invalid = self.request(
            client,
            4,
            "input.replace",
            {"lease_id": lease, "state": malformed_state},
        )
        self.assertFalse(invalid["ok"])
        self.assertEqual(invalid["code"], "E_INPUT_SCHEMA")

    def test_status_is_authenticated_but_does_not_require_a_lease(self) -> None:
        client = self.connect()
        response = self.request(client, 1, "status.get", {})
        self.assertTrue(response["ok"])
        self.assertFalse(response["data"]["lease_active"])
        self.assertEqual(response["data"]["protocol"], MODULE.PROTOCOL)

    def test_same_uid_gate_rejects_peer_before_protocol_processing(self) -> None:
        client = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        client.settimeout(0.1)
        client.connect(os.fspath(self.broker.path))
        self.clients.append(client)
        with mock.patch.object(MODULE.os, "getuid", return_value=os.getuid() + 1):
            self.broker.poll(now=self.clock.value)
        with self.assertRaises((ConnectionResetError, BrokenPipeError, socket.timeout)):
            client.send(b"x")
            client.recv(1)
        self.assertEqual(self.broker.rejected_peers, 1)

    def test_unauthenticated_idle_clients_cannot_permanently_fill_all_slots(self) -> None:
        idle = [self.connect() for _ in range(MODULE.MAX_CLIENTS)]
        authenticated = self.connect()
        response = self.request(authenticated, 1, "status.get", {})
        self.assertTrue(response["ok"])
        self.assertLessEqual(
            response["data"]["connected_clients"],
            MODULE.MAX_CLIENTS,
        )
        with self.assertRaises((BrokenPipeError, ConnectionResetError, socket.timeout)):
            idle[0].send(b"x")
            idle[0].recv(1)


class InputValidationTest(unittest.TestCase):
    def test_state_schema_types_ranges_and_nonfinite_values_are_strict(self) -> None:
        mutations = []
        missing = neutral_mapping()
        del missing["keyboard"]["alt"]
        mutations.append(missing)
        bool_axis = neutral_mapping()
        bool_axis["gamepad"]["axes"]["forward"] = True
        mutations.append(bool_axis)
        range_axis = neutral_mapping()
        range_axis["gamepad"]["axes"]["forward"] = 1.01
        mutations.append(range_axis)
        nan_mouse = neutral_mapping()
        nan_mouse["mouse"]["dx"] = float("nan")
        mutations.append(nan_mouse)
        extra = neutral_mapping()
        extra["extra"] = False
        mutations.append(extra)
        for value in mutations:
            with self.subTest(value=value), self.assertRaises(
                MODULE.ExternalControlError
            ):
                MODULE.ExternalInputState.from_mapping(value)

    def test_refuses_relative_paths_and_preexisting_non_socket_target(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            with self.assertRaisesRegex(ValueError, "absolute"):
                MODULE.ExternalControlBroker(Path("x"), root / "cap")
            endpoint = root / "endpoint"
            endpoint.write_text("owned", encoding="utf-8")
            broker = MODULE.ExternalControlBroker(endpoint, root / "cap")
            with self.assertRaisesRegex(RuntimeError, "non-socket"):
                broker.open()
            self.assertEqual(endpoint.read_text(encoding="utf-8"), "owned")

    def test_refuses_live_socket_and_close_does_not_unlink_replacement_inode(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            endpoint = root / "endpoint.sock"
            live = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            live.bind(os.fspath(endpoint))
            live.listen(1)
            broker = MODULE.ExternalControlBroker(endpoint, root / "cap")
            try:
                with self.assertRaisesRegex(RuntimeError, "already active"):
                    broker.open()
                self.assertTrue(endpoint.exists())
            finally:
                live.close()
                endpoint.unlink(missing_ok=True)

            broker = MODULE.ExternalControlBroker(endpoint, root / "cap")
            broker.open()
            moved = root / "owned-old.sock"
            endpoint.rename(moved)
            replacement = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            replacement.bind(os.fspath(endpoint))
            replacement.listen(1)
            try:
                broker.close()
                self.assertTrue(endpoint.exists())
            finally:
                replacement.close()
                endpoint.unlink(missing_ok=True)
                moved.unlink(missing_ok=True)


if __name__ == "__main__":
    unittest.main()
