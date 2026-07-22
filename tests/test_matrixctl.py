from __future__ import annotations

from collections.abc import Callable
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
import matrix_game_control as CONTROL  # noqa: E402
import matrix_game_control_input as PROVIDER  # noqa: E402
import matrixctl as MODULE  # noqa: E402


class MatrixCtlHelpersTest(unittest.TestCase):
    def test_capability_reader_requires_a_private_owned_regular_file(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            capability = root / "control.cap"
            token = "a" * 64
            capability.write_text(token + "\n", encoding="ascii")
            capability.chmod(0o600)
            self.assertEqual(MODULE._read_capability(capability), token)

            capability.chmod(0o640)
            with self.assertRaisesRegex(PermissionError, "private owned regular"):
                MODULE._read_capability(capability)

            capability.chmod(0o600)
            with mock.patch.object(
                MODULE.os,
                "getuid",
                return_value=os.getuid() + 1,
            ), self.assertRaisesRegex(PermissionError, "private owned regular"):
                MODULE._read_capability(capability)

            link = root / "linked.cap"
            link.symlink_to(capability)
            with self.assertRaises(OSError):
                MODULE._read_capability(link)

            with self.assertRaisesRegex(PermissionError, "private owned regular"):
                MODULE._read_capability(root)

    def test_capability_reader_rejects_malformed_content(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            capability = Path(temporary) / "control.cap"
            capability.write_bytes(b"not-a-capability\n")
            capability.chmod(0o600)
            with self.assertRaisesRegex(RuntimeError, "malformed"):
                MODULE._read_capability(capability)

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

        connected_neutral = MODULE._connected_neutral_gamepad_state()
        self.assertTrue(connected_neutral.gamepad_connected)
        self.assertFalse(any(connected_neutral.gamepad_buttons.values()))
        self.assertTrue(
            all(value == 0.0 for value in connected_neutral.gamepad_axes.values())
        )

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

    def test_held_state_is_replaced_once_then_renewed_on_absolute_cadence(self) -> None:
        class Clock:
            now = 0.0

            def read(self) -> float:
                return self.now

            def sleep(self, seconds: float) -> None:
                self.now += seconds

        client = mock.Mock()
        state = MODULE._state_with_keyboard("w", ("alt",))
        clock = Clock()

        MODULE._hold_state(
            client,
            "lease",
            state,
            seconds=0.16,
            refresh_seconds=0.05,
            clock=clock.read,
            sleeper=clock.sleep,
        )

        client.replace.assert_called_once_with("lease", state)
        self.assertEqual(client.refresh.call_count, 3)
        client.refresh.assert_has_calls([mock.call("lease")] * 3)
        self.assertAlmostEqual(clock.now, 0.16)

    def test_wait_only_refreshes_fast_enough_for_a_short_deadman(self) -> None:
        class Clock:
            now = 0.0

            def read(self) -> float:
                return self.now

            def sleep(self, seconds: float) -> None:
                self.now += seconds

        clock = Clock()
        refresh_times: list[float] = []
        client = mock.Mock()
        client.refresh.side_effect = lambda _lease_id: refresh_times.append(
            clock.now
        )
        deadman_seconds = 0.012
        refresh_seconds = deadman_seconds / 3.0

        MODULE._wait_with_lease_refresh(
            client,
            "lease",
            seconds=0.04,
            refresh_seconds=refresh_seconds,
            clock=clock.read,
            sleeper=clock.sleep,
        )

        self.assertEqual(client.refresh.call_count, 9)
        client.replace.assert_not_called()
        cadence_points = [0.0, *refresh_times, clock.now]
        for previous, current in zip(cadence_points, cadence_points[1:]):
            self.assertLessEqual(
                current - previous,
                refresh_seconds + 1e-12,
            )
        self.assertAlmostEqual(clock.now, 0.04)

    def test_neutral_warmup_renews_until_all_provider_frames_are_covered(
        self,
    ) -> None:
        class Clock:
            now = 0.0

            def read(self) -> float:
                return self.now

            def sleep(self, seconds: float) -> None:
                self.now += seconds

        client = mock.Mock()
        neutral = EXTERNAL.ExternalInputState.neutral()
        clock = Clock()

        MODULE._hold_state(
            client,
            "lease",
            neutral,
            seconds=MODULE._NEUTRAL_WARMUP_SECONDS,
            refresh_seconds=0.05,
            clock=clock.read,
            sleeper=clock.sleep,
        )

        client.replace.assert_called_once_with("lease", neutral)
        self.assertEqual(client.refresh.call_count, 2)
        client.refresh.assert_has_calls([mock.call("lease")] * 2)
        self.assertAlmostEqual(clock.now, MODULE._NEUTRAL_WARMUP_SECONDS)

    @mock.patch.object(MODULE, "_hold_state")
    @mock.patch.object(MODULE, "MatrixControlClient")
    @mock.patch.object(MODULE, "_resolved_paths")
    @mock.patch.object(MODULE, "_parse_args")
    def test_main_renews_no_slower_than_50ms_then_neutralizes_and_releases(
        self,
        parse_args,
        resolved_paths,
        client_type,
        hold_state,
    ) -> None:
        parse_args.return_value = type(
            "Args",
            (),
            {
                "action": "key",
                "profile": "trna",
                "socket": None,
                "capability_file": None,
                "timeout": 1.0,
                "seconds": 1.0,
                "key": "w",
                "modifier": ["alt"],
                "double": False,
                "tap_gap": 0.08,
            },
        )()
        resolved_paths.return_value = (
            Path("/run/user/1000/control.sock"),
            Path("/run/user/1000/control.cap"),
        )
        client = client_type.return_value.__enter__.return_value
        client.acquire.return_value = ("lease", 0.15)

        self.assertEqual(MODULE.main(), 0)

        self.assertEqual(hold_state.call_count, 2)
        warmup, action = hold_state.call_args_list
        self.assertEqual(
            warmup.args,
            (
                client,
                "lease",
                MODULE._state_with_keyboard(None, ("alt",)),
            ),
        )
        self.assertEqual(
            warmup.kwargs["seconds"], MODULE._NEUTRAL_WARMUP_SECONDS
        )
        self.assertEqual(
            action.args,
            (client, "lease", MODULE._state_with_keyboard("w", ("alt",))),
        )
        self.assertEqual(action.kwargs["seconds"], 1.0)
        for call in (warmup, action):
            self.assertLessEqual(call.kwargs["refresh_seconds"], 0.05)
        client.replace.assert_called_once_with(
            "lease",
            EXTERNAL.ExternalInputState.neutral(),
        )
        client.release.assert_called_once_with("lease")

    @mock.patch.object(MODULE, "_hold_state")
    @mock.patch.object(MODULE, "MatrixControlClient")
    @mock.patch.object(MODULE, "_resolved_paths")
    @mock.patch.object(MODULE, "_parse_args")
    def test_main_double_tap_uses_modifier_only_warmup_before_both_taps(
        self,
        parse_args,
        resolved_paths,
        client_type,
        hold_state,
    ) -> None:
        parse_args.return_value = type(
            "Args",
            (),
            {
                "action": "key",
                "profile": "trna",
                "socket": None,
                "capability_file": None,
                "timeout": 1.0,
                "seconds": 0.25,
                "key": "w",
                "modifier": ["alt"],
                "double": True,
                "tap_gap": 0.08,
            },
        )()
        resolved_paths.return_value = (
            Path("/run/user/1000/control.sock"),
            Path("/run/user/1000/control.cap"),
        )
        client = client_type.return_value.__enter__.return_value
        client.acquire.return_value = ("lease", 0.15)

        self.assertEqual(MODULE.main(), 0)

        pressed = MODULE._state_with_keyboard("w", ("alt",))
        modifier_only = MODULE._state_with_keyboard(None, ("alt",))
        self.assertEqual(hold_state.call_count, 4)
        warmup, first_tap, gap, second_tap = hold_state.call_args_list
        self.assertEqual(
            warmup.args,
            (client, "lease", modifier_only),
        )
        self.assertEqual(
            warmup.kwargs["seconds"], MODULE._NEUTRAL_WARMUP_SECONDS
        )
        self.assertEqual(first_tap.args, (client, "lease", pressed))
        self.assertEqual(first_tap.kwargs["seconds"], 0.04)
        self.assertEqual(gap.args, (client, "lease", modifier_only))
        self.assertEqual(gap.kwargs["seconds"], 0.08)
        self.assertEqual(second_tap.args, (client, "lease", pressed))
        self.assertEqual(second_tap.kwargs["seconds"], 0.25)
        for call in hold_state.call_args_list:
            self.assertLessEqual(call.kwargs["refresh_seconds"], 0.05)
        client.replace.assert_called_once_with(
            "lease",
            EXTERNAL.ExternalInputState.neutral(),
        )
        client.release.assert_called_once_with("lease")

    @mock.patch.object(MODULE, "_hold_state")
    @mock.patch.object(MODULE, "MatrixControlClient")
    @mock.patch.object(MODULE, "_resolved_paths")
    @mock.patch.object(MODULE, "_parse_args")
    def test_main_gamepad_connects_centered_before_action_then_disconnects(
        self,
        parse_args,
        resolved_paths,
        client_type,
        hold_state,
    ) -> None:
        parse_args.return_value = type(
            "Args",
            (),
            {
                "action": "gamepad",
                "profile": "trna",
                "socket": None,
                "capability_file": None,
                "timeout": 1.0,
                "forward": 0.75,
                "right": -0.25,
                "look_yaw": 0.5,
                "look_pitch": -0.5,
                "seconds": 1.0,
            },
        )()
        resolved_paths.return_value = (
            Path("/run/user/1000/control.sock"),
            Path("/run/user/1000/control.cap"),
        )
        client = client_type.return_value.__enter__.return_value
        client.acquire.return_value = ("lease", 0.012)

        self.assertEqual(MODULE.main(), 0)

        self.assertEqual(hold_state.call_count, 2)
        warmup, action = hold_state.call_args_list
        connected_neutral = MODULE._connected_neutral_gamepad_state()
        requested = MODULE._state_with_gamepad(parse_args.return_value)
        self.assertEqual(warmup.args, (client, "lease", connected_neutral))
        self.assertEqual(
            warmup.kwargs["seconds"], MODULE._NEUTRAL_WARMUP_SECONDS
        )
        self.assertEqual(action.args, (client, "lease", requested))
        self.assertEqual(action.kwargs["seconds"], 1.0)
        for call in (warmup, action):
            self.assertEqual(call.kwargs["refresh_seconds"], 0.004)

        full_neutral = EXTERNAL.ExternalInputState.neutral()
        self.assertFalse(full_neutral.gamepad_connected)
        client.replace.assert_called_once_with("lease", full_neutral)
        client.release.assert_called_once_with("lease")

    def test_connected_gamepad_warmup_crosses_provider_and_core_rearm(
        self,
    ) -> None:
        class Args:
            forward = 0.75
            right = 0.0
            look_yaw = 0.0
            look_pitch = 0.0

        physical_focus = PROVIDER.KeyboardMouseSample(
            focused=True,
            focus_title="Matrix",
            focus_pid=42,
        )
        disconnected_neutral = EXTERNAL.ExternalInputState.neutral()
        connected_neutral = MODULE._connected_neutral_gamepad_state()
        active_stick = MODULE._state_with_gamepad(Args())

        def exercise(
            warmup: EXTERNAL.ExternalInputState,
        ) -> tuple[
            list[tuple[bool, CONTROL.RobotMotionCommand]],
            Callable[
                [EXTERNAL.ExternalInputState],
                tuple[bool, CONTROL.RobotMotionCommand],
            ],
        ]:
            core = CONTROL.GameControlCore()
            previous_connected = False
            sequence = 0
            now_s = 10.0

            def provider_frame(
                state: EXTERNAL.ExternalInputState,
            ) -> tuple[bool, CONTROL.RobotMotionCommand]:
                nonlocal previous_connected, sequence, now_s
                sequence += 1
                now_s += 0.02
                keyboard, gamepad = PROVIDER.external_input_samples(
                    state,
                    focus=physical_focus,
                    look_button="left",
                )
                frame_source = PROVIDER.external_frame_input_source(
                    state,
                    configured_source="auto",
                )
                input_available = PROVIDER.gamepad_input_available(
                    frame_source,
                    connected=gamepad.connected,
                    previous_connected=previous_connected,
                )
                previous_connected = gamepad.connected
                snapshot = PROVIDER.build_snapshot(
                    sequence=sequence,
                    timestamp_monotonic_s=now_s,
                    keyboard=keyboard,
                    gamepad=gamepad,
                    input_source=frame_source,
                    camera_yaw_rad=0.0,
                    camera_available=True,
                    input_available=input_available,
                )
                core.accept_snapshot(snapshot, received_at_s=now_s)
                return input_available, core.command(now_s=now_s, dt_s=0.02)

            # Model the preceding physical/provider frame, then enough warmup
            # frames to span both the hotplug edge and neutral rearm.
            provider_frame(disconnected_neutral)
            observations = [provider_frame(warmup), provider_frame(warmup)]
            observations.extend(
                (provider_frame(active_stick), provider_frame(active_stick))
            )
            return observations, provider_frame

        old, _ = exercise(disconnected_neutral)
        self.assertTrue(old[2][1].safe_stop)
        self.assertEqual(old[2][1].reason, "focus_lost")
        self.assertTrue(old[3][1].safe_stop)
        self.assertEqual(old[3][1].reason, "awaiting_neutral")

        current, provider_frame = exercise(connected_neutral)
        # The provider rejects exactly the connected edge, then a second
        # connected-and-centered frame safely rearms the core.
        self.assertFalse(current[0][0])
        self.assertEqual(current[0][1].reason, "focus_lost")
        self.assertTrue(current[1][0])
        self.assertEqual(current[1][1].mode, "idle")
        for input_available, command in current[2:]:
            self.assertTrue(input_available)
            self.assertFalse(command.safe_stop)
            self.assertIn(command.mode, {"move", "turn"})

        # Full-neutral cleanup disconnects the virtual controller.  That edge
        # deadmans immediately, and reconnecting with a displaced stick cannot
        # bypass the centered-stick requirement.
        cleanup_available, cleanup = provider_frame(disconnected_neutral)
        self.assertFalse(cleanup_available)
        self.assertTrue(cleanup.safe_stop)
        self.assertEqual(cleanup.reason, "focus_lost")
        reconnect_available, reconnect = provider_frame(active_stick)
        self.assertFalse(reconnect_available)
        self.assertTrue(reconnect.safe_stop)
        self.assertEqual(reconnect.reason, "focus_lost")
        available_again, still_blocked = provider_frame(active_stick)
        self.assertTrue(available_again)
        self.assertTrue(still_blocked.safe_stop)
        self.assertEqual(still_blocked.reason, "awaiting_neutral")

    def test_main_double_tap_activates_detector_after_external_source_rearm(
        self,
    ) -> None:
        for modifiers in ((), ("alt",), ("ctrl",), ("shift",)):
            with self.subTest(modifiers=modifiers):
                detector = PROVIDER.KeyboardDoubleTapDetector(0.30)
                focus = PROVIDER.KeyboardMouseSample(focused=True)
                now_s = 1.0
                # Model the provider's preceding physical-input frame so the
                # first external frame must traverse the real source-change
                # reset path before the two W edges arrive.
                self.assertFalse(
                    detector.update(
                        focus,
                        now_s=now_s - 0.02,
                        enabled=True,
                        source_id="physical",
                    )
                )

                def observe_hold(
                    _client,
                    _lease_id,
                    state,
                    *,
                    seconds,
                    refresh_seconds,
                ) -> None:
                    del refresh_seconds
                    nonlocal now_s
                    keyboard, _gamepad = PROVIDER.external_input_samples(
                        state,
                        focus=focus,
                        look_button="left",
                    )
                    detector.update(
                        keyboard,
                        now_s=now_s,
                        enabled=True,
                        source_id="external",
                    )
                    now_s += seconds

                args = type(
                    "Args",
                    (),
                    {
                        "action": "key",
                        "profile": "trna",
                        "socket": None,
                        "capability_file": None,
                        "timeout": 1.0,
                        "seconds": 0.25,
                        "key": "w",
                        "modifier": list(modifiers),
                        "double": True,
                        "tap_gap": 0.08,
                    },
                )()
                endpoint = Path("/run/user/1000/control.sock")
                capability = Path("/run/user/1000/control.cap")
                with mock.patch.object(
                    MODULE,
                    "_parse_args",
                    return_value=args,
                ), mock.patch.object(
                    MODULE,
                    "_resolved_paths",
                    return_value=(endpoint, capability),
                ), mock.patch.object(
                    MODULE,
                    "MatrixControlClient",
                ) as client_type, mock.patch.object(
                    MODULE,
                    "_hold_state",
                    side_effect=observe_hold,
                ):
                    client = client_type.return_value.__enter__.return_value
                    client.acquire.return_value = ("lease", 0.15)
                    self.assertEqual(MODULE.main(), 0)

                self.assertEqual(detector.activations, 1)
                self.assertEqual(detector.telemetry["source_id"], "external")
                self.assertEqual(detector.telemetry["boost_key"], "w")

    @mock.patch.object(MODULE, "_wait_with_lease_refresh")
    @mock.patch.object(MODULE, "_hold_state")
    @mock.patch.object(MODULE, "MatrixControlClient")
    @mock.patch.object(MODULE, "_resolved_paths")
    @mock.patch.object(MODULE, "_parse_args")
    def test_mouse_delta_visibility_renews_short_lease_without_repeating_delta(
        self,
        parse_args,
        resolved_paths,
        client_type,
        hold_state,
        wait_with_refresh,
    ) -> None:
        parse_args.return_value = type(
            "Args",
            (),
            {
                "action": "mouse",
                "profile": "trna",
                "socket": None,
                "capability_file": None,
                "timeout": 1.0,
                "dx": 12.0,
                "dy": -3.0,
                "button": "left",
                "seconds": 0.25,
            },
        )()
        resolved_paths.return_value = (
            Path("/run/user/1000/control.sock"),
            Path("/run/user/1000/control.cap"),
        )
        client = client_type.return_value.__enter__.return_value
        client.acquire.return_value = ("lease", 0.012)

        self.assertEqual(MODULE.main(), 0)

        refresh_seconds = 0.012 / 3.0
        wait_with_refresh.assert_called_once_with(
            client,
            "lease",
            seconds=0.04,
            refresh_seconds=refresh_seconds,
        )
        delta = MODULE._state_with_mouse(12.0, -3.0, "left")
        neutral = EXTERNAL.ExternalInputState.neutral()
        self.assertEqual(
            client.replace.call_args_list,
            [mock.call("lease", delta), mock.call("lease", neutral)],
        )
        self.assertEqual(hold_state.call_count, 2)
        warmup, held = hold_state.call_args_list
        self.assertEqual(warmup.args, (client, "lease", neutral))
        self.assertEqual(
            held.args,
            (client, "lease", MODULE._state_with_mouse(0.0, 0.0, "left")),
        )
        self.assertAlmostEqual(held.kwargs["seconds"], 0.21)
        for call in hold_state.call_args_list:
            self.assertEqual(call.kwargs["refresh_seconds"], refresh_seconds)
        client.release.assert_called_once_with("lease")

    @mock.patch.object(
        MODULE,
        "_hold_state",
        side_effect=(None, KeyboardInterrupt),
    )
    @mock.patch.object(MODULE, "MatrixControlClient")
    @mock.patch.object(MODULE, "_resolved_paths")
    @mock.patch.object(MODULE, "_parse_args")
    def test_main_neutralizes_and_releases_after_interrupt(
        self,
        parse_args,
        resolved_paths,
        client_type,
        _hold_state,
    ) -> None:
        parse_args.return_value = type(
            "Args",
            (),
            {
                "action": "key",
                "profile": "trna",
                "socket": None,
                "capability_file": None,
                "timeout": 1.0,
                "seconds": 1.0,
                "key": "w",
                "modifier": [],
                "double": False,
                "tap_gap": 0.08,
            },
        )()
        resolved_paths.return_value = (
            Path("/run/user/1000/control.sock"),
            Path("/run/user/1000/control.cap"),
        )
        client = client_type.return_value.__enter__.return_value
        client.acquire.return_value = ("lease", 0.15)

        with self.assertRaises(KeyboardInterrupt):
            MODULE.main()

        self.assertEqual(_hold_state.call_count, 2)
        warmup, interrupted_action = _hold_state.call_args_list
        self.assertEqual(
            warmup.args,
            (client, "lease", EXTERNAL.ExternalInputState.neutral()),
        )
        self.assertEqual(
            warmup.kwargs["seconds"], MODULE._NEUTRAL_WARMUP_SECONDS
        )
        self.assertEqual(
            interrupted_action.args,
            (client, "lease", MODULE._state_with_keyboard("w", ())),
        )
        client.replace.assert_called_once_with(
            "lease",
            EXTERNAL.ExternalInputState.neutral(),
        )
        client.release.assert_called_once_with("lease")

    def test_typed_negative_response_preserves_error_code(self) -> None:
        client = MODULE.MatrixControlClient(
            Path("/tmp/control.sock"),
            Path("/tmp/control.cap"),
        )
        client._socket = mock.Mock()
        client._capability = "a" * 64
        client._socket.send.return_value = mock.ANY
        response = {
            "protocol": EXTERNAL.PROTOCOL,
            "kind": "response",
            "sequence": 1,
            "ok": False,
            "code": "E_LEASE",
            "message": "client does not own the active lease",
            "data": None,
        }
        encoded = json.dumps(response).encode("utf-8")
        client._socket.send.return_value = len(
            json.dumps(
                {
                    "protocol": EXTERNAL.PROTOCOL,
                    "kind": "request",
                    "sequence": 1,
                    "capability": "a" * 64,
                    "operation": "lease.renew",
                    "payload": {"lease_id": "lease"},
                },
                separators=(",", ":"),
                allow_nan=False,
            ).encode("utf-8")
        )
        client._socket.recv.return_value = encoded

        with self.assertRaises(MODULE.MatrixControlResponseError) as raised:
            client.request("lease.renew", {"lease_id": "lease"})
        self.assertEqual(raised.exception.code, "E_LEASE")

    @mock.patch.object(MODULE, "MatrixControlClient")
    @mock.patch.object(MODULE, "_resolved_paths")
    @mock.patch.object(MODULE, "_parse_args")
    def test_main_does_not_cleanup_with_a_revoked_lease(
        self,
        parse_args,
        resolved_paths,
        client_type,
    ) -> None:
        parse_args.return_value = type(
            "Args",
            (),
            {
                "action": "key",
                "profile": "trna",
                "socket": None,
                "capability_file": None,
                "timeout": 1.0,
                "seconds": 1.0,
                "key": "w",
                "modifier": ["alt"],
                "double": False,
                "tap_gap": 0.08,
            },
        )()
        resolved_paths.return_value = (
            Path("/run/user/1000/control.sock"),
            Path("/run/user/1000/control.cap"),
        )
        client = client_type.return_value.__enter__.return_value
        client.acquire.return_value = ("lease", 0.15)
        client.replace.side_effect = MODULE.MatrixControlResponseError(
            "E_LEASE",
            "client does not own the active lease",
        )

        with self.assertRaises(MODULE.MatrixControlResponseError):
            MODULE.main()

        client.replace.assert_called_once()
        client.release.assert_not_called()

    @mock.patch.object(MODULE, "_hold_state")
    @mock.patch.object(MODULE, "_wait_for_command_terminal")
    @mock.patch.object(MODULE, "MatrixControlClient")
    @mock.patch.object(MODULE, "_resolved_paths")
    @mock.patch.object(MODULE, "_parse_args")
    def test_rejected_command_with_revoked_authority_skips_stale_cleanup(
        self,
        parse_args,
        resolved_paths,
        client_type,
        wait_terminal,
        hold_state,
    ) -> None:
        parse_args.return_value = type(
            "Args",
            (),
            {
                "action": "command",
                "profile": "trna",
                "socket": None,
                "capability_file": None,
                "timeout": 1.0,
                "hold_seconds": 1.0,
                "command": "/tp @s ~ ~ ~",
            },
        )()
        resolved_paths.return_value = (
            Path("/run/user/1000/control.sock"),
            Path("/run/user/1000/control.cap"),
        )
        command_id = "a" * 32
        client = client_type.return_value.__enter__.return_value
        client.acquire.return_value = ("lease", 0.15)
        client.command.return_value = {"data": {"command_id": command_id}}
        wait_terminal.return_value = (
            {
                "command_id": command_id,
                "terminal": True,
                "state": "rejected",
                "authority_revoked": True,
                "result": {
                    "ok": False,
                    "code": "E_COMMAND_REJECTED",
                    "message": "rejected after authority revocation",
                },
            },
            False,
        )

        with self.assertRaisesRegex(RuntimeError, "E_COMMAND_REJECTED"):
            MODULE.main()

        hold_state.assert_called_once_with(
            client,
            "lease",
            EXTERNAL.ExternalInputState.neutral(),
            seconds=MODULE._NEUTRAL_WARMUP_SECONDS,
            refresh_seconds=mock.ANY,
        )
        client.command.assert_called_once_with("lease", "/tp @s ~ ~ ~")
        client.replace.assert_not_called()
        client.release.assert_not_called()

    @mock.patch.object(MODULE, "_hold_state")
    @mock.patch.object(MODULE, "_wait_for_command_terminal")
    @mock.patch.object(MODULE, "MatrixControlClient")
    @mock.patch.object(MODULE, "_resolved_paths")
    @mock.patch.object(MODULE, "_parse_args")
    def test_unknown_command_outcome_after_lease_loss_skips_stale_cleanup(
        self,
        parse_args,
        resolved_paths,
        client_type,
        wait_terminal,
        hold_state,
    ) -> None:
        parse_args.return_value = type(
            "Args",
            (),
            {
                "action": "command",
                "profile": "trna",
                "socket": None,
                "capability_file": None,
                "timeout": 1.0,
                "hold_seconds": 1.0,
                "command": "/tp @s ~ ~ ~",
            },
        )()
        resolved_paths.return_value = (
            Path("/run/user/1000/control.sock"),
            Path("/run/user/1000/control.cap"),
        )
        command_id = "b" * 32
        client = client_type.return_value.__enter__.return_value
        client.acquire.return_value = ("lease", 0.15)
        client.command.return_value = {"data": {"command_id": command_id}}
        wait_terminal.side_effect = MODULE.MatrixCommandOutcomeUnknownError(
            f"E_COMMAND_OUTCOME_UNKNOWN: no terminal receipt for {command_id}",
            lease_available=False,
        )

        with self.assertRaises(MODULE.MatrixCommandOutcomeUnknownError):
            MODULE.main()

        hold_state.assert_called_once_with(
            client,
            "lease",
            EXTERNAL.ExternalInputState.neutral(),
            seconds=MODULE._NEUTRAL_WARMUP_SECONDS,
            refresh_seconds=mock.ANY,
        )
        client.command.assert_called_once_with("lease", "/tp @s ~ ~ ~")
        client.replace.assert_not_called()
        client.release.assert_not_called()

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

    def test_unknown_command_outcome_carries_revoked_lease_state(self) -> None:
        command_id = "c" * 32

        class Clock:
            now = 0.0

            def read(self) -> float:
                return self.now

            def sleep(self, seconds: float) -> None:
                self.now += seconds

        class Client:
            def __init__(self, *, authority_revoked: bool, renew_fails: bool) -> None:
                self.authority_revoked = authority_revoked
                self.renew_fails = renew_fails

            def command_result(self, _command_id: str) -> dict[str, object]:
                return {
                    "command_id": command_id,
                    "terminal": False,
                    "state": "admitted",
                    "authority_revoked": self.authority_revoked,
                }

            def refresh(self, _lease_id: str) -> None:
                if self.renew_fails:
                    raise MODULE.MatrixControlResponseError(
                        "E_LEASE",
                        "client does not own the active lease",
                    )

            @staticmethod
            def persistent_command_result(
                _command_id: str,
            ) -> dict[str, object] | None:
                return None

        for authority_revoked, renew_fails in ((False, True), (True, False)):
            with self.subTest(
                authority_revoked=authority_revoked,
                renew_fails=renew_fails,
            ):
                clock = Clock()
                with self.assertRaises(
                    MODULE.MatrixCommandOutcomeUnknownError
                ) as raised:
                    MODULE._wait_for_command_terminal(
                        Client(
                            authority_revoked=authority_revoked,
                            renew_fails=renew_fails,
                        ),
                        "lease",
                        command_id,
                        hold_seconds=0.02,
                        refresh_seconds=0.01,
                        clock=clock.read,
                        sleeper=clock.sleep,
                    )
                self.assertFalse(raised.exception.lease_available)
                if renew_fails:
                    self.assertIsInstance(
                        raised.exception.__cause__,
                        MODULE.MatrixControlResponseError,
                    )


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
