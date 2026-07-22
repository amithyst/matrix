from __future__ import annotations

import json
import os
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest import mock


import sys

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if os.fspath(SCRIPTS) not in sys.path:
    sys.path.insert(0, os.fspath(SCRIPTS))

import matrix_external_control as EXTERNAL  # noqa: E402
import matrix_game_control_input as PROVIDER  # noqa: E402
import matrixctl as MODULE  # noqa: E402


class MatrixCtlHelpersTest(unittest.TestCase):
    def test_profile_endpoint_is_stable_and_rejects_path_traversal(self) -> None:
        endpoint, capability = MODULE.default_endpoint("trna")
        self.assertTrue(endpoint.is_absolute())
        self.assertEqual(endpoint.name, "trna.sock")
        self.assertEqual(capability.name, "trna.cap")
        with self.assertRaisesRegex(ValueError, "profile"):
            MODULE.default_endpoint("../trna")

    def test_typed_state_builders_cover_keyboard_mouse_and_gamepad(self) -> None:
        keyboard = MODULE._state_with_keyboard("w", ("alt",))
        self.assertTrue(keyboard.keyboard["w"])
        self.assertTrue(keyboard.keyboard["alt"])
        mouse = MODULE._state_with_mouse(12.0, -3.0, "left")
        self.assertEqual((mouse.mouse_dx, mouse.mouse_dy), (12.0, -3.0))
        self.assertTrue(mouse.mouse_buttons["left"])

        class Args:
            forward = 0.75
            right = -0.25
            look_yaw = 0.5
            look_pitch = -0.5

        gamepad = MODULE._state_with_gamepad(Args())
        self.assertTrue(gamepad.gamepad_connected)
        self.assertEqual(gamepad.gamepad_axes["forward"], 0.75)

    def test_modifier_only_gap_preserves_double_tap_speed_tier(self) -> None:
        for modifier in ("alt", "ctrl", "shift"):
            with self.subTest(modifier=modifier):
                detector = PROVIDER.KeyboardDoubleTapDetector(0.30)
                pressed = MODULE._state_with_keyboard("w", (modifier,))
                gap = MODULE._state_with_keyboard(None, (modifier,))

                def sample(state: EXTERNAL.ExternalInputState):
                    return PROVIDER.external_input_samples(
                        state,
                        focus=PROVIDER.KeyboardMouseSample(focused=True),
                        look_button="left",
                    )[0]

                self.assertFalse(
                    detector.update(sample(pressed), now_s=1.00, enabled=True)
                )
                self.assertFalse(
                    detector.update(sample(gap), now_s=1.08, enabled=True)
                )
                self.assertTrue(
                    detector.update(sample(pressed), now_s=1.16, enabled=True)
                )

    def test_resolved_paths_prefer_game_env_and_accept_legacy_env(self) -> None:
        class Args:
            profile = "trna"
            socket = None
            capability_file = None

        with mock.patch.dict(
            os.environ,
            {
                "MATRIX_GAME_EXTERNAL_CONTROL_SOCKET": "/new/control.sock",
                "MATRIX_GAME_EXTERNAL_CONTROL_CAPABILITY_FILE": "/new/control.cap",
                "MATRIX_EXTERNAL_CONTROL_SOCKET": "/legacy/control.sock",
                "MATRIX_EXTERNAL_CONTROL_CAPABILITY_FILE": "/legacy/control.cap",
            },
            clear=True,
        ):
            self.assertEqual(
                MODULE._resolved_paths(Args()),
                (Path("/new/control.sock"), Path("/new/control.cap")),
            )

        with mock.patch.dict(
            os.environ,
            {
                "MATRIX_EXTERNAL_CONTROL_SOCKET": "/legacy/control.sock",
                "MATRIX_EXTERNAL_CONTROL_CAPABILITY_FILE": "/legacy/control.cap",
            },
            clear=True,
        ):
            self.assertEqual(
                MODULE._resolved_paths(Args()),
                (Path("/legacy/control.sock"), Path("/legacy/control.cap")),
            )

    def test_command_receipts_require_typed_authority_revoked(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            command_id = "a" * 32
            client = MODULE.MatrixControlClient(
                root / "control.sock",
                root / "control.cap",
            )
            with mock.patch.object(
                client,
                "request",
                return_value={
                    "data": {
                        "command_id": command_id,
                        "terminal": False,
                        "state": "admitted",
                        "authority_revoked": "false",
                    }
                },
            ):
                with self.assertRaisesRegex(RuntimeError, "receipt is malformed"):
                    client.command_result(command_id)

            client.receipt_directory.mkdir(mode=0o700)
            receipt_path = client.receipt_directory / f"{command_id}.json"
            receipt_path.write_text(
                json.dumps(
                    {
                        "schema": EXTERNAL.COMMAND_RECEIPT_SCHEMA,
                        "written_unix_ns": 1,
                        "receipt": {
                            "command_id": command_id,
                            "terminal": True,
                            "state": "completed",
                            "authority_revoked": 0,
                        },
                    }
                ),
                encoding="utf-8",
            )
            receipt_path.chmod(0o600)
            with self.assertRaisesRegex(RuntimeError, "identity is invalid"):
                client.persistent_command_result(command_id)

    def test_endpoint_failure_uses_durable_terminal_or_reports_unknown(self) -> None:
        command_id = "b" * 32
        terminal = {
            "command_id": command_id,
            "terminal": True,
            "state": "completed",
            "authority_revoked": True,
            "result": {"ok": True},
        }

        class Clock:
            now = 0.0

            def read(self) -> float:
                return self.now

            def sleep(self, seconds: float) -> None:
                self.now += seconds

        class Client:
            def __init__(self, durable: dict[str, object] | None) -> None:
                self.durable = durable

            @staticmethod
            def command_result(_command_id: str) -> dict[str, object]:
                raise BrokenPipeError("endpoint closed")

            @staticmethod
            def refresh(_lease_id: str) -> None:
                raise AssertionError("endpoint failure must not renew")

            def persistent_command_result(
                self,
                _command_id: str,
            ) -> dict[str, object] | None:
                return self.durable

        clock = Clock()
        receipt, lease_available = MODULE._wait_for_command_terminal(
            Client(terminal),
            "lease",
            command_id,
            hold_seconds=1.0,
            refresh_seconds=0.01,
            clock=clock.read,
            sleeper=clock.sleep,
        )
        self.assertEqual(receipt, terminal)
        self.assertFalse(lease_available)

        clock = Clock()
        with self.assertRaisesRegex(RuntimeError, "E_COMMAND_OUTCOME_UNKNOWN"):
            MODULE._wait_for_command_terminal(
                Client(None),
                "lease",
                command_id,
                hold_seconds=1.0,
                refresh_seconds=0.01,
                clock=clock.read,
                sleeper=clock.sleep,
            )
        self.assertGreaterEqual(clock.now, 0.50)


@unittest.skipUnless(
    hasattr(__import__("socket"), "SOCK_SEQPACKET"),
    "Unix SOCK_SEQPACKET is required",
)
class MatrixCtlBrokerIntegrationTest(unittest.TestCase):
    def test_client_status_lease_input_command_and_release(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            broker = EXTERNAL.ExternalControlBroker(
                root / "control.sock",
                root / "control.cap",
            )
            broker.open()
            stop = threading.Event()
            failures: list[BaseException] = []

            def serve() -> None:
                try:
                    while not stop.is_set():
                        broker.poll()
                        time.sleep(0.001)
                except BaseException as exc:
                    failures.append(exc)

            thread = threading.Thread(target=serve, daemon=True)
            thread.start()
            try:
                with MODULE.MatrixControlClient(
                    broker.path,
                    broker.capability_file,
                ) as client:
                    status = client.request("status.get", {})
                    self.assertFalse(status["data"]["lease_active"])
                    lease_id, deadman = client.acquire()
                    self.assertLessEqual(deadman, 0.15)
                    client.refresh(lease_id)
                    state = MODULE._state_with_keyboard("w", ("shift",))
                    client.replace(lease_id, state)
                    self.assertTrue(broker.sample().keyboard["w"])
                    queued = client.command(
                        lease_id,
                        "/data modify entity @s "
                        "control.input.keyboard.w set value false",
                    )
                    self.assertEqual(queued["code"], "OK_COMMAND_QUEUED")
                    command_id = queued["data"]["command_id"]
                    deadline = time.monotonic() + 1.0
                    commands = ()
                    while not commands and time.monotonic() < deadline:
                        commands = broker.drain_commands(limit=1)
                        time.sleep(0.001)
                    self.assertEqual(len(commands), 1)
                    broker.complete_command(
                        commands[0],
                        {
                            "ok": True,
                            "outcome_unknown": False,
                            "code": "OK_DATA_INPUT_MODIFIED",
                            "message": "modified",
                        },
                    )
                    receipt = client.command_result(command_id)
                    self.assertTrue(receipt["terminal"])
                    self.assertEqual(receipt["state"], "completed")
                    durable = client.persistent_command_result(command_id)
                    self.assertIsNotNone(durable)
                    self.assertEqual(durable["state"], "completed")
                    durable_path = (
                        broker.receipt_directory / f"{command_id}.json"
                    )
                    self.assertEqual(durable_path.stat().st_mode & 0o777, 0o600)
                    client.release(lease_id)
                    self.assertFalse(broker.lease_active)
            finally:
                stop.set()
                thread.join(timeout=1.0)
                broker.close()
            self.assertEqual(failures, [])

    def test_admitted_command_survives_override_and_later_completion(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            broker = EXTERNAL.ExternalControlBroker(
                root / "control.sock",
                root / "control.cap",
            )
            broker.open()
            stop = threading.Event()
            failures: list[BaseException] = []

            def serve() -> None:
                try:
                    while not stop.is_set():
                        broker.poll()
                        time.sleep(0.001)
                except BaseException as exc:
                    failures.append(exc)

            server = threading.Thread(target=serve, daemon=True)
            server.start()
            completion: threading.Thread | None = None
            try:
                with MODULE.MatrixControlClient(
                    broker.path,
                    broker.capability_file,
                ) as client:
                    lease_id, _deadman = client.acquire()
                    queued = client.command(lease_id, "/tp @s ~ ~ ~")
                    command_id = queued["data"]["command_id"]
                    deadline = time.monotonic() + 1.0
                    commands = ()
                    while not commands and time.monotonic() < deadline:
                        commands = broker.drain_commands(limit=1)
                        time.sleep(0.001)
                    self.assertEqual(len(commands), 1)
                    command = commands[0]

                    original_result = client.command_result
                    original_refresh = client.refresh
                    refresh_attempted = threading.Event()
                    result_calls = 0

                    def result_then_override(
                        requested_id: str,
                    ) -> dict[str, object]:
                        nonlocal result_calls
                        receipt = original_result(requested_id)
                        result_calls += 1
                        if result_calls == 1:
                            self.assertEqual(receipt["state"], "admitted")
                            self.assertFalse(receipt["terminal"])
                            self.assertFalse(receipt["authority_revoked"])
                            broker.local_override("physical_keyboard")
                        return receipt

                    def refresh_after_override(requested_lease: str) -> None:
                        try:
                            original_refresh(requested_lease)
                        finally:
                            refresh_attempted.set()

                    def complete_later() -> None:
                        if not refresh_attempted.wait(timeout=1.0):
                            failures.append(
                                AssertionError("lease refresh was not attempted")
                            )
                            return
                        time.sleep(0.03)
                        broker.complete_command(
                            command,
                            {
                                "ok": True,
                                "outcome_unknown": False,
                                "code": "OK_TELEPORT_RESTART",
                                "message": "saved",
                            },
                        )

                    completion = threading.Thread(
                        target=complete_later,
                        daemon=True,
                    )
                    completion.start()
                    with mock.patch.object(
                        client,
                        "command_result",
                        side_effect=result_then_override,
                    ), mock.patch.object(
                        client,
                        "refresh",
                        side_effect=refresh_after_override,
                    ) as refresh:
                        receipt, lease_available = (
                            MODULE._wait_for_command_terminal(
                                client,
                                lease_id,
                                command_id,
                                hold_seconds=1.0,
                                refresh_seconds=0.01,
                            )
                        )

                    completion.join(timeout=1.0)
                    self.assertFalse(completion.is_alive())
                    self.assertEqual(refresh.call_count, 1)
                    self.assertFalse(lease_available)
                    self.assertTrue(receipt["terminal"])
                    self.assertEqual(receipt["state"], "completed")
                    self.assertTrue(receipt["authority_revoked"])
                    self.assertTrue(receipt["result"]["ok"])
                    durable = client.persistent_command_result(command_id)
                    self.assertIsNotNone(durable)
                    self.assertTrue(durable["authority_revoked"])
            finally:
                if completion is not None:
                    completion.join(timeout=1.0)
                stop.set()
                server.join(timeout=1.0)
                broker.close()
            self.assertEqual(failures, [])


if __name__ == "__main__":
    unittest.main()
