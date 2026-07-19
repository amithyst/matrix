from __future__ import annotations

import importlib.util
import json
import math
import os
from pathlib import Path
import socket
import stat
import sys
import tempfile
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "matrix_game_control.py"
SPEC = importlib.util.spec_from_file_location("matrix_game_control", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def snapshot(
    *,
    sequence: int = 1,
    timestamp: float = 10.0,
    focused: bool = True,
    yaw: float = 0.0,
    pressed: tuple[str, ...] = (),
    stick: tuple[float, float] = (0.0, 0.0),
    speed_modifiers: tuple[str, ...] = (),
):
    keys = {name: name in pressed for name in ("w", "a", "s", "d", "q", "e", "v")}
    keys.update(
        ctrl="ctrl" in speed_modifiers,
        shift="shift" in speed_modifiers,
    )
    return MODULE.InputSnapshot.from_mapping(
        {
            "protocol": MODULE.PROTOCOL_NAME,
            "sequence": sequence,
            "timestamp_monotonic_s": timestamp,
            "focused": focused,
            "camera_yaw_rad": yaw,
            "keys": keys,
            "move_stick": {"right": stick[0], "forward": stick[1]},
        }
    )


def immediate_config(**overrides):
    values = {
        "max_speed_mps": 0.3,
        "max_acceleration_mps2": 1000.0,
        "max_deceleration_mps2": 1000.0,
        "max_turn_rate_rad_s": 1000.0,
        "max_step_s": 1.0,
    }
    values.update(overrides)
    return MODULE.ControlConfig(**values)


def armed_core(config=None):
    core = MODULE.GameControlCore(config)
    core.accept_snapshot(
        snapshot(sequence=0, timestamp=9.99), received_at_s=9.99
    )
    return core


class InputProtocolTest(unittest.TestCase):
    def test_snapshot_packet_round_trip(self) -> None:
        original = snapshot(pressed=("w", "q"), stick=(0.25, -0.5), yaw=1.25)
        payload = MODULE.encode_input_packet(original)
        decoded = MODULE.decode_input_packet(payload)
        self.assertEqual(decoded, original)
        self.assertLessEqual(len(payload), MODULE.MAX_PACKET_BYTES)

    def test_v2_requires_both_keyboard_speed_modifiers(self) -> None:
        value = snapshot().to_mapping()
        self.assertEqual(value["protocol"], "matrix-game-input/v2")
        del value["keys"]["ctrl"]
        with self.assertRaisesRegex(MODULE.InputProtocolError, "missing fields: ctrl"):
            MODULE.InputSnapshot.from_mapping(value)

        value = snapshot().to_mapping()
        value["protocol"] = "matrix-game-input/v1"
        with self.assertRaisesRegex(MODULE.InputProtocolError, "v2"):
            MODULE.InputSnapshot.from_mapping(value)

    def test_schema_is_an_exact_whitelist(self) -> None:
        value = snapshot().to_mapping()
        value["shell_command"] = "do not accept arbitrary actions"
        with self.assertRaisesRegex(MODULE.InputProtocolError, "unknown fields"):
            MODULE.InputSnapshot.from_mapping(value)

        value = snapshot().to_mapping()
        del value["focused"]
        with self.assertRaisesRegex(MODULE.InputProtocolError, "missing fields"):
            MODULE.InputSnapshot.from_mapping(value)

        value = snapshot().to_mapping()
        value["keys"]["space"] = True
        with self.assertRaisesRegex(MODULE.InputProtocolError, "unknown fields"):
            MODULE.InputSnapshot.from_mapping(value)

    def test_schema_rejects_ambiguous_and_nonfinite_types(self) -> None:
        value = snapshot().to_mapping()
        value["sequence"] = True
        with self.assertRaisesRegex(MODULE.InputProtocolError, "integer"):
            MODULE.InputSnapshot.from_mapping(value)

        value = snapshot().to_mapping()
        value["move_stick"]["right"] = 1.01
        with self.assertRaisesRegex(MODULE.InputProtocolError, r"\[-1, 1\]"):
            MODULE.InputSnapshot.from_mapping(value)

        payload = MODULE.encode_input_packet(snapshot())
        nan_payload = payload.replace(b'"camera_yaw_rad":0.0', b'"camera_yaw_rad":NaN')
        with self.assertRaisesRegex(MODULE.InputProtocolError, "not allowed"):
            MODULE.decode_input_packet(nan_payload)

        value = snapshot().to_mapping()
        value["camera_yaw_rad"] = 10**10_000
        with self.assertRaisesRegex(MODULE.InputProtocolError, "finite"):
            MODULE.InputSnapshot.from_mapping(value)

    def test_json_duplicate_fields_are_rejected(self) -> None:
        payload = MODULE.encode_input_packet(snapshot())
        duplicate = payload.replace(
            b'"focused":true', b'"focused":true,"focused":false'
        )
        with self.assertRaisesRegex(MODULE.InputProtocolError, "duplicate"):
            MODULE.decode_input_packet(duplicate)

    def test_packet_size_and_utf8_are_bounded(self) -> None:
        with self.assertRaisesRegex(MODULE.InputProtocolError, "byte limit"):
            MODULE.decode_input_packet(b"x" * (MODULE.MAX_PACKET_BYTES + 1))
        with self.assertRaisesRegex(MODULE.InputProtocolError, "UTF-8"):
            MODULE.decode_input_packet(b"\xff")
        nested = (b"[" * 1100) + b"0" + (b"]" * 1100)
        with self.assertRaisesRegex(MODULE.InputProtocolError, "valid JSON"):
            MODULE.decode_input_packet(nested)


class MovementMathTest(unittest.TestCase):
    def test_camera_yaw_projects_only_on_horizontal_plane(self) -> None:
        x, y = MODULE.camera_relative_to_world(
            right=0.0, forward=1.0, camera_yaw_rad=math.pi / 2.0
        )
        self.assertAlmostEqual(x, 0.0, places=7)
        self.assertAlmostEqual(y, 1.0, places=7)

        x, y = MODULE.camera_relative_to_world(
            right=1.0, forward=0.0, camera_yaw_rad=math.pi / 2.0
        )
        self.assertAlmostEqual(x, 1.0, places=7)
        self.assertAlmostEqual(y, 0.0, places=7)

        x, y = MODULE.camera_relative_to_world(
            right=1.0, forward=0.0, camera_yaw_rad=0.0
        )
        self.assertAlmostEqual(x, 0.0, places=7)
        self.assertAlmostEqual(y, -1.0, places=7)

    def test_radial_deadzone_preserves_direction_and_remaps_magnitude(self) -> None:
        self.assertEqual(
            MODULE.apply_radial_deadzone(right=0.1, forward=0.0, deadzone=0.2),
            (0.0, 0.0),
        )
        right, forward = MODULE.apply_radial_deadzone(
            right=0.3, forward=0.4, deadzone=0.2
        )
        self.assertAlmostEqual(right, 0.225)
        self.assertAlmostEqual(forward, 0.3)
        self.assertAlmostEqual(math.hypot(right, forward), 0.375)

        right, forward = MODULE.apply_radial_deadzone(
            right=1.0, forward=1.0, deadzone=0.2
        )
        self.assertAlmostEqual(math.hypot(right, forward), 1.0)

    def test_native_gait_boundary_ties_follow_requested_tier(self) -> None:
        select = MODULE.native_locomotion_mode_for_speed
        self.assertEqual(select(0.10, requested_mode=1), 1)
        self.assertEqual(select(0.80, requested_mode=1), 1)
        self.assertEqual(select(0.79995, requested_mode=2), 1)
        self.assertEqual(select(0.80, requested_mode=2), 2)
        self.assertEqual(select(2.49, requested_mode=3), 2)
        self.assertEqual(select(2.49995, requested_mode=3), 2)
        self.assertEqual(select(2.50, requested_mode=2), 2)
        self.assertEqual(select(2.50, requested_mode=3), 3)
        with self.assertRaisesRegex(ValueError, "below native SLOW_WALK"):
            select(0.05, requested_mode=1)


class GameControlCoreTest(unittest.TestCase):
    def test_library_defaults_match_safe_runtime_profile(self) -> None:
        config = MODULE.ControlConfig()
        self.assertEqual(config.max_speed_mps, 0.30)
        self.assertEqual(config.max_acceleration_mps2, 1.20)
        self.assertEqual(config.max_deceleration_mps2, 2.40)
        self.assertEqual(config.max_turn_rate_rad_s, 2.50)

    def test_gait_configuration_cannot_make_slow_tier_unreachable(self) -> None:
        with self.assertRaisesRegex(ValueError, "minimum == start"):
            MODULE.ControlConfig(gait_start_speed_mps=0.12)
        with self.assertRaisesRegex(ValueError, "start < stop"):
            MODULE.ControlConfig(
                gait_start_heading_error_rad=math.radians(31.0),
                gait_stop_heading_error_rad=math.radians(30.0),
            )
        with self.assertRaisesRegex(ValueError, "must not overlap"):
            MODULE.ControlConfig(gait_stop_speed_mps=0.099)
        with self.assertRaisesRegex(ValueError, "native SLOW_WALK minimum"):
            MODULE.ControlConfig(
                min_gait_speed_mps=0.05,
                gait_start_speed_mps=0.05,
                gait_stop_speed_mps=0.04,
            )
        self.assertEqual(MODULE.ControlConfig(max_speed_mps=0.8).max_speed_mps, 0.8)
        with self.assertRaisesRegex(ValueError, "SLOW_WALK maximum"):
            MODULE.ControlConfig(max_speed_mps=0.81)

    def test_w_follows_camera_and_orients_to_movement(self) -> None:
        core = armed_core(immediate_config())
        core.accept_snapshot(
            snapshot(
                yaw=math.pi / 2.0,
                pressed=("w",),
                speed_modifiers=("shift",),
            ),
            received_at_s=10.0,
        )
        first = core.command(now_s=10.0, dt_s=0.1)
        self.assertAlmostEqual(first.speed_mps, 0.10)
        self.assertEqual(first.locomotion_mode, MODULE.SONIC_SLOW_WALK_MODE)
        command = core.command(now_s=10.0, dt_s=0.1)
        self.assertEqual(command.mode, "move")
        self.assertAlmostEqual(command.speed_mps, 2.5)
        self.assertEqual(command.locomotion_mode, MODULE.SONIC_RUN_MODE)
        self.assertAlmostEqual(command.movement[0], 0.0, places=7)
        self.assertAlmostEqual(command.movement[1], 1.0, places=7)
        self.assertEqual(command.movement, command.facing)

    def test_s_walks_forward_after_turning_to_camera_back(self) -> None:
        core = armed_core(immediate_config())
        core.accept_snapshot(snapshot(pressed=("s",)), received_at_s=10.0)
        command = core.command(now_s=10.0, dt_s=0.1)
        self.assertAlmostEqual(command.movement[0], -1.0, places=7)
        self.assertAlmostEqual(command.movement[1], 0.0, places=7)
        self.assertEqual(command.movement, command.facing)

    def test_wasd_diagonal_is_normalized_and_does_not_gain_speed(self) -> None:
        cardinal = armed_core(immediate_config())
        cardinal.accept_snapshot(snapshot(pressed=("w",)), received_at_s=10.0)
        cardinal_command = cardinal.command(now_s=10.0, dt_s=0.1)

        diagonal = armed_core(immediate_config())
        diagonal.accept_snapshot(snapshot(pressed=("w", "d")), received_at_s=10.0)
        diagonal_command = diagonal.command(now_s=10.0, dt_s=0.1)

        self.assertAlmostEqual(diagonal_command.speed_mps, cardinal_command.speed_mps)
        self.assertAlmostEqual(
            math.hypot(*diagonal_command.movement[:2]), 1.0, places=7
        )
        self.assertAlmostEqual(
            diagonal_command.movement[0], math.sqrt(0.5), places=7
        )
        self.assertAlmostEqual(
            diagonal_command.movement[1], -math.sqrt(0.5), places=7
        )

    def test_keyboard_uses_native_hold_to_walk_and_run_gaits(self) -> None:
        config = immediate_config(max_speed_mps=0.3)

        def command_for(modifiers: tuple[str, ...]):
            core = armed_core(config)
            core.accept_snapshot(
                snapshot(pressed=("w",), speed_modifiers=modifiers),
                received_at_s=10.0,
            )
            core.command(now_s=10.0, dt_s=0.1)
            return core.command(now_s=10.0, dt_s=0.1)

        slow = command_for(("ctrl",))
        walk = command_for(())
        run = command_for(("shift",))
        self.assertEqual((slow.locomotion_mode, slow.speed_mps), (1, 0.10))
        self.assertEqual((walk.locomotion_mode, walk.speed_mps), (2, 0.80))
        self.assertEqual((run.locomotion_mode, run.speed_mps), (3, 2.50))
        # The slower modifier wins an accidental overlap.
        conflict = command_for(("ctrl", "shift"))
        self.assertEqual((conflict.locomotion_mode, conflict.speed_mps), (1, 0.10))

    def test_modifiers_without_direction_are_native_idle(self) -> None:
        for modifiers in (("ctrl",), ("shift",), ("ctrl", "shift")):
            core = armed_core(immediate_config(max_speed_mps=0.3))
            core.accept_snapshot(
                snapshot(speed_modifiers=modifiers), received_at_s=10.0
            )
            command = core.command(now_s=10.0, dt_s=0.1)
            self.assertEqual(command.locomotion_mode, MODULE.SONIC_IDLE_MODE)
            self.assertEqual(command.speed_mps, 0.0)
            self.assertEqual(command.mode, "idle")

    def test_native_gait_boundaries_follow_acceleration_and_downshift(self) -> None:
        core = armed_core(
            MODULE.ControlConfig(
                max_speed_mps=0.3,
                max_acceleration_mps2=1.0,
                max_deceleration_mps2=1.0,
                max_turn_rate_rad_s=100.0,
                max_step_s=2.0,
            )
        )
        core.accept_snapshot(
            snapshot(pressed=("w",), speed_modifiers=("shift",)),
            received_at_s=10.0,
        )
        entered = core.command(now_s=10.0, dt_s=0.1)
        walking = core.command(now_s=10.0, dt_s=0.7)
        running = core.command(now_s=10.0, dt_s=1.7)
        self.assertEqual((entered.locomotion_mode, entered.speed_mps), (1, 0.1))
        self.assertEqual(walking.locomotion_mode, 2)
        self.assertAlmostEqual(walking.speed_mps, 0.8)
        self.assertEqual(running.locomotion_mode, 3)
        self.assertAlmostEqual(running.speed_mps, 2.5)

        core.accept_snapshot(
            snapshot(
                sequence=2,
                timestamp=10.01,
                pressed=("w",),
                speed_modifiers=("ctrl",),
            ),
            received_at_s=10.01,
        )
        downshifted = core.command(now_s=10.01, dt_s=1.0)
        precise = core.command(now_s=10.01, dt_s=1.0)
        self.assertEqual(downshifted.locomotion_mode, 2)
        self.assertAlmostEqual(downshifted.speed_mps, 1.5)
        self.assertEqual(precise.locomotion_mode, 1)
        self.assertAlmostEqual(precise.speed_mps, 0.5)

    def test_small_measured_heading_error_still_reaches_keyboard_native_gaits(self) -> None:
        for modifiers, expected_mode, expected_speed in (
            ((), MODULE.SONIC_WALK_MODE, 0.8),
            (("shift",), MODULE.SONIC_RUN_MODE, 2.5),
        ):
            core = armed_core(immediate_config(max_speed_mps=0.3))
            core.synchronize_heading(math.radians(5.0))
            core.accept_snapshot(
                snapshot(pressed=("w",), speed_modifiers=modifiers),
                received_at_s=10.0,
            )
            core.command(now_s=10.0, dt_s=0.1)
            settled = core.command(now_s=10.0, dt_s=0.1)
            self.assertEqual(settled.locomotion_mode, expected_mode)
            self.assertAlmostEqual(settled.speed_mps, expected_speed)

    def test_keyboard_modifiers_do_not_quantize_gamepad_speed(self) -> None:
        core = armed_core(immediate_config(max_speed_mps=0.3))
        core.accept_snapshot(
            snapshot(
                stick=(0.0, 0.575),
                speed_modifiers=("ctrl", "shift"),
            ),
            received_at_s=10.0,
        )

        core.command(now_s=10.0, dt_s=0.1)
        moving = core.command(now_s=10.0, dt_s=0.1)

        self.assertAlmostEqual(moving.speed_mps, 0.20)

    def test_analog_full_stick_respects_configured_slow_walk_cap(self) -> None:
        core = armed_core(immediate_config(max_speed_mps=0.8))
        core.accept_snapshot(snapshot(stick=(0.0, 1.0)), received_at_s=10.0)
        core.command(now_s=10.0, dt_s=0.1)
        moving = core.command(now_s=10.0, dt_s=0.1)
        self.assertEqual(moving.locomotion_mode, MODULE.SONIC_SLOW_WALK_MODE)
        self.assertAlmostEqual(moving.speed_mps, 0.8)

    def test_keyboard_run_to_analog_clamps_to_configured_cap_immediately(self) -> None:
        core = armed_core(immediate_config(max_speed_mps=0.3))
        core.accept_snapshot(
            snapshot(pressed=("w",), speed_modifiers=("shift",)),
            received_at_s=10.0,
        )
        core.command(now_s=10.0, dt_s=0.1)
        running = core.command(now_s=10.0, dt_s=0.1)
        self.assertEqual((running.locomotion_mode, running.speed_mps), (3, 2.5))

        core.accept_snapshot(
            snapshot(sequence=2, timestamp=10.01, stick=(0.0, 1.0)),
            received_at_s=10.01,
        )
        analog = core.command(now_s=10.01, dt_s=0.02)
        self.assertEqual(analog.locomotion_mode, MODULE.SONIC_SLOW_WALK_MODE)
        self.assertAlmostEqual(analog.speed_mps, 0.3)

    def test_keyboard_tier_changes_keep_acceleration_limits(self) -> None:
        core = armed_core(
            MODULE.ControlConfig(
                max_speed_mps=0.3,
                max_acceleration_mps2=1.0,
                max_deceleration_mps2=0.5,
                max_turn_rate_rad_s=100.0,
                max_step_s=1.0,
            )
        )
        core.accept_snapshot(
            snapshot(pressed=("w",), speed_modifiers=("ctrl",)),
            received_at_s=10.0,
        )
        self.assertAlmostEqual(
            core.command(now_s=10.0, dt_s=0.1).speed_mps, 0.10
        )

        core.accept_snapshot(
            snapshot(
                sequence=2,
                timestamp=10.01,
                pressed=("w",),
                speed_modifiers=("shift",),
            ),
            received_at_s=10.01,
        )
        self.assertAlmostEqual(
            core.command(now_s=10.01, dt_s=0.1).speed_mps, 0.20
        )

        core.accept_snapshot(
            snapshot(
                sequence=3,
                timestamp=10.02,
                pressed=("w",),
                speed_modifiers=("ctrl",),
            ),
            received_at_s=10.02,
        )
        self.assertAlmostEqual(
            core.command(now_s=10.02, dt_s=0.1).speed_mps, 0.15
        )

    def test_keyboard_wins_over_left_stick_while_held(self) -> None:
        core = armed_core(immediate_config())
        core.accept_snapshot(
            snapshot(pressed=("w",), stick=(1.0, 0.0)), received_at_s=10.0
        )
        core.command(now_s=10.0, dt_s=0.1)
        command = core.command(now_s=10.0, dt_s=0.1)
        self.assertAlmostEqual(command.movement[0], 1.0)
        self.assertAlmostEqual(command.movement[1], 0.0)

    def test_opposing_digital_keys_block_analog_fallback(self) -> None:
        core = armed_core(immediate_config())
        core.accept_snapshot(
            snapshot(pressed=("w", "s"), stick=(1.0, 0.0)),
            received_at_s=10.0,
        )
        command = core.command(now_s=10.0, dt_s=0.1)
        self.assertEqual(command.mode, "idle")
        self.assertEqual(command.movement, (0.0, 0.0, 0.0))

    def test_left_stick_preserves_analog_speed_after_deadzone(self) -> None:
        core = armed_core(immediate_config(stick_deadzone=0.2))
        core.accept_snapshot(snapshot(stick=(0.0, 0.6)), received_at_s=10.0)
        first = core.command(now_s=10.0, dt_s=0.1)
        self.assertAlmostEqual(first.speed_mps, 0.10)
        command = core.command(now_s=10.0, dt_s=0.1)
        # (0.6 - 0.2) / (1 - 0.2) = 0.5 input.  Analog travel spans the
        # interval from the 0.10 m/s native floor to the 0.30 m/s default cap.
        self.assertAlmostEqual(command.speed_mps, 0.20)
        self.assertAlmostEqual(command.movement[0], 1.0)
        self.assertAlmostEqual(command.movement[1], 0.0)

    def test_a_and_d_orient_to_camera_left_and_right(self) -> None:
        left = armed_core(immediate_config())
        left.accept_snapshot(snapshot(pressed=("a",)), received_at_s=10.0)
        left_command = left.command(now_s=10.0, dt_s=0.1)
        self.assertAlmostEqual(left_command.movement[0], 0.0, places=7)
        self.assertAlmostEqual(left_command.movement[1], 1.0, places=7)

        right = armed_core(immediate_config())
        right.accept_snapshot(snapshot(pressed=("d",)), received_at_s=10.0)
        right_command = right.command(now_s=10.0, dt_s=0.1)
        self.assertAlmostEqual(right_command.movement[0], 0.0, places=7)
        self.assertAlmostEqual(right_command.movement[1], -1.0, places=7)

    def test_q_and_e_never_contribute_to_locomotion(self) -> None:
        for key in ("q", "e", "q", "e"):
            core = armed_core(immediate_config())
            core.accept_snapshot(snapshot(pressed=(key,)), received_at_s=10.0)
            command = core.command(now_s=10.0, dt_s=0.1)
            self.assertEqual(command.mode, "idle")
            self.assertEqual(command.movement, (0.0, 0.0, 0.0))
            self.assertEqual(command.speed_mps, 0.0)

    def test_turn_and_acceleration_are_rate_limited(self) -> None:
        config = MODULE.ControlConfig(
            max_speed_mps=0.3,
            max_acceleration_mps2=1.0,
            max_deceleration_mps2=2.0,
            max_turn_rate_rad_s=1.0,
            max_step_s=0.1,
        )
        core = armed_core(config)
        core.accept_snapshot(
            snapshot(yaw=math.pi / 2.0, pressed=("w",)), received_at_s=10.0
        )
        command = core.command(now_s=10.0, dt_s=1.0)
        self.assertAlmostEqual(math.atan2(command.facing[1], command.facing[0]), 0.1)
        self.assertEqual(command.speed_mps, 0.0)
        self.assertEqual(command.mode, "idle")

        core.accept_snapshot(
            snapshot(sequence=2, timestamp=10.1, yaw=math.pi / 2.0, pressed=("w",)),
            received_at_s=10.1,
        )
        still_turning = core.command(now_s=10.1, dt_s=0.1)
        self.assertAlmostEqual(
            math.atan2(still_turning.facing[1], still_turning.facing[0]),
            0.2,
        )
        self.assertEqual(still_turning.mode, "idle")

        forward = armed_core(config)
        forward.accept_snapshot(snapshot(pressed=("w",)), received_at_s=10.0)
        gait_entry = forward.command(now_s=10.0, dt_s=0.1)
        self.assertAlmostEqual(gait_entry.speed_mps, 0.1)
        moving = forward.command(now_s=10.0, dt_s=0.1)
        self.assertAlmostEqual(moving.speed_mps, 0.2)
        self.assertEqual(moving.movement, moving.facing)

    def test_physical_heading_feedback_gates_translation_during_reversal(self) -> None:
        core = armed_core(immediate_config())
        core.synchronize_heading(0.0)
        core.accept_snapshot(snapshot(pressed=("s",)), received_at_s=10.0)

        turning = core.command(now_s=10.0, dt_s=0.1)

        self.assertEqual(turning.mode, "idle")
        self.assertEqual(turning.speed_mps, 0.0)
        self.assertAlmostEqual(abs(core.heading_rad), math.pi)
        self.assertEqual(core.measured_heading_rad, 0.0)

        core.synchronize_heading(math.pi)
        core.accept_snapshot(
            snapshot(sequence=2, timestamp=10.01, pressed=("s",)),
            received_at_s=10.01,
        )
        moving = core.command(now_s=10.01, dt_s=0.1)
        self.assertEqual(moving.mode, "move")
        self.assertGreater(moving.speed_mps, 0.0)

    def test_mid_turn_retarget_waits_for_command_and_body_alignment(self) -> None:
        config = MODULE.ControlConfig(
            max_speed_mps=0.3,
            max_acceleration_mps2=100.0,
            max_deceleration_mps2=100.0,
            max_turn_rate_rad_s=1.0,
            input_timeout_s=10.0,
            max_snapshot_age_s=10.0,
            max_step_s=0.1,
        )
        core = MODULE.GameControlCore(config, initial_heading_rad=math.pi)
        core.accept_snapshot(
            snapshot(sequence=0, timestamp=9.99), received_at_s=9.99
        )
        # The body already faces the new camera-forward request, but the
        # rate-limited planner target still points near the old reverse heading.
        core.synchronize_heading(math.pi / 2.0)
        core.accept_snapshot(
            snapshot(sequence=1, yaw=math.pi / 2.0, pressed=("w",)),
            received_at_s=10.0,
        )

        still_turning = core.command(now_s=10.0, dt_s=0.1)

        self.assertEqual(core.measured_heading_rad, math.pi / 2.0)
        self.assertGreater(
            abs(MODULE.wrap_angle_rad(core.heading_rad - (math.pi / 2.0))),
            math.radians(80.0),
        )
        self.assertEqual(still_turning.mode, "idle")
        self.assertEqual(still_turning.speed_mps, 0.0)

    def test_safety_stop_holds_measured_heading_not_stale_turn_target(self) -> None:
        core = MODULE.GameControlCore(
            immediate_config(), initial_heading_rad=math.pi
        )
        core.synchronize_heading(0.0)
        core.accept_snapshot(
            snapshot(sequence=0, timestamp=9.99), received_at_s=9.99
        )
        core.accept_snapshot(
            snapshot(
                sequence=1,
                timestamp=10.0,
                focused=False,
                pressed=("s",),
            ),
            received_at_s=10.0,
        )

        stopped = core.command(now_s=10.0, dt_s=0.1)

        self.assertTrue(stopped.safe_stop)
        self.assertEqual(stopped.reason, "focus_lost")
        self.assertEqual(stopped.speed_mps, 0.0)
        self.assertAlmostEqual(core.heading_rad, 0.0)
        self.assertAlmostEqual(stopped.facing[0], 1.0)
        self.assertAlmostEqual(stopped.facing[1], 0.0)

        core.synchronize_heading(math.radians(35.0))
        still_stopped = core.command(now_s=10.01, dt_s=0.01)
        self.assertTrue(still_stopped.safe_stop)
        self.assertAlmostEqual(core.heading_rad, 0.0)
        self.assertAlmostEqual(still_stopped.facing[0], 1.0)
        self.assertAlmostEqual(still_stopped.facing[1], 0.0)

    def test_active_gait_does_not_hold_minimum_speed_in_wrong_heading(self) -> None:
        core = armed_core(
            MODULE.ControlConfig(
                max_speed_mps=0.3,
                max_acceleration_mps2=100.0,
                max_deceleration_mps2=0.1,
                max_turn_rate_rad_s=100.0,
            )
        )
        core.synchronize_heading(0.0)
        core.accept_snapshot(snapshot(pressed=("w",)), received_at_s=10.0)
        self.assertEqual(core.command(now_s=10.0, dt_s=0.1).mode, "move")

        core.accept_snapshot(
            snapshot(sequence=2, timestamp=10.01, pressed=("s",)),
            received_at_s=10.01,
        )
        turning = core.command(now_s=10.01, dt_s=0.02)
        self.assertEqual(turning.mode, "idle")
        self.assertEqual(turning.speed_mps, 0.0)

    def test_antipodal_camera_noise_keeps_one_turn_direction(self) -> None:
        config = MODULE.ControlConfig(
            max_speed_mps=0.3,
            max_acceleration_mps2=1.0,
            max_deceleration_mps2=2.0,
            max_turn_rate_rad_s=1.0,
            max_step_s=0.1,
        )
        core = armed_core(config)
        headings = []
        for sequence, yaw in enumerate((1e-4, -1e-4, 1e-4, -1e-4), start=1):
            now = 10.0 + (sequence * 0.01)
            core.accept_snapshot(
                snapshot(
                    sequence=sequence,
                    timestamp=now,
                    yaw=yaw,
                    pressed=("s",),
                ),
                received_at_s=now,
            )
            core.command(now_s=now, dt_s=0.1)
            headings.append(core.heading_rad)
        self.assertTrue(
            all(later > earlier for earlier, later in zip(headings, headings[1:]))
        )

    def test_direction_release_hard_idles_in_one_frame(self) -> None:
        core = armed_core()
        core.accept_snapshot(snapshot(pressed=("w",)), received_at_s=10.0)
        moving = core.command(now_s=10.0, dt_s=0.1)
        self.assertGreater(moving.speed_mps, 0.0)

        core.accept_snapshot(snapshot(sequence=2, timestamp=10.1), received_at_s=10.1)
        stopping = core.command(now_s=10.1, dt_s=0.1)
        self.assertEqual(stopping.speed_mps, 0.0)
        self.assertEqual(stopping.locomotion_mode, MODULE.SONIC_IDLE_MODE)
        self.assertEqual(stopping.mode, "idle")

    def test_run_key_release_requests_idle_in_one_control_frame(self) -> None:
        core = armed_core(immediate_config(max_speed_mps=0.3))
        core.accept_snapshot(
            snapshot(pressed=("w",), speed_modifiers=("shift",)),
            received_at_s=10.0,
        )
        core.command(now_s=10.0, dt_s=0.1)
        running = core.command(now_s=10.0, dt_s=0.1)
        self.assertEqual((running.locomotion_mode, running.speed_mps), (3, 2.5))

        core.accept_snapshot(snapshot(sequence=2, timestamp=10.01), received_at_s=10.01)
        stopped = core.command(now_s=10.01, dt_s=0.02)
        self.assertEqual(stopped.locomotion_mode, MODULE.SONIC_IDLE_MODE)
        self.assertEqual(stopped.speed_mps, 0.0)
        self.assertEqual(stopped.mode, "idle")

    def test_mid_turn_direction_release_holds_measured_heading(self) -> None:
        core = armed_core(
            MODULE.ControlConfig(
                max_speed_mps=0.3,
                max_acceleration_mps2=1.0,
                max_deceleration_mps2=1.0,
                max_turn_rate_rad_s=1.0,
                max_step_s=0.1,
            )
        )
        core.synchronize_heading(0.0)
        core.accept_snapshot(snapshot(pressed=("s",)), received_at_s=10.0)
        turning = core.command(now_s=10.0, dt_s=0.1)
        self.assertGreater(abs(math.atan2(turning.facing[1], turning.facing[0])), 0.0)

        core.accept_snapshot(snapshot(sequence=2, timestamp=10.01), received_at_s=10.01)
        stopped = core.command(now_s=10.01, dt_s=0.02)
        self.assertEqual(stopped.locomotion_mode, MODULE.SONIC_IDLE_MODE)
        self.assertEqual(stopped.speed_mps, 0.0)
        self.assertAlmostEqual(stopped.facing[0], 1.0)
        self.assertAlmostEqual(stopped.facing[1], 0.0)

        # Planner lookahead can keep the physical body rotating after key-up.
        # IDLE must retain the one release-frame target instead of chasing that
        # residual rotation and triggering a fresh native replan every frame.
        core.synchronize_heading(math.radians(40.0))
        core.accept_snapshot(
            snapshot(sequence=3, timestamp=10.02), received_at_s=10.02
        )
        held = core.command(now_s=10.02, dt_s=0.02)
        self.assertEqual(held.mode, "idle")
        self.assertAlmostEqual(math.atan2(held.facing[1], held.facing[0]), 0.0)

        # A new movement interval clears the stop latch.  Its next release can
        # capture a new measured heading exactly once.
        core.accept_snapshot(
            snapshot(sequence=4, timestamp=10.03, pressed=("w",)),
            received_at_s=10.03,
        )
        core.command(now_s=10.03, dt_s=0.02)
        core.synchronize_heading(math.radians(25.0))
        core.accept_snapshot(
            snapshot(sequence=5, timestamp=10.04), received_at_s=10.04
        )
        relatched = core.command(now_s=10.04, dt_s=0.02)
        self.assertAlmostEqual(
            math.atan2(relatched.facing[1], relatched.facing[0]),
            math.radians(25.0),
        )

    def test_slow_walk_never_publishes_below_native_gait_minimum(self) -> None:
        core = armed_core(
            MODULE.ControlConfig(
                max_speed_mps=0.3,
                max_acceleration_mps2=1.2,
                max_deceleration_mps2=2.4,
                max_turn_rate_rad_s=2.5,
            )
        )
        core.accept_snapshot(snapshot(pressed=("w",)), received_at_s=10.0)
        starting = [
            core.command(now_s=10.0 + (index * 0.02), dt_s=0.02)
            for index in range(5)
        ]
        self.assertTrue(all(command.speed_mps == 0.0 for command in starting[:4]))
        self.assertAlmostEqual(starting[4].speed_mps, 0.1)
        after_entry = core.command(now_s=10.1, dt_s=0.02)
        self.assertAlmostEqual(after_entry.speed_mps - starting[4].speed_mps, 0.024)

        core.accept_snapshot(
            snapshot(sequence=2, timestamp=10.1), received_at_s=10.1
        )
        stopping = [
            core.command(now_s=10.1 + (index * 0.02), dt_s=0.02)
            for index in range(8)
        ]
        published = [command.speed_mps for command in stopping if command.speed_mps]
        self.assertTrue(all(speed >= 0.1 for speed in published))
        self.assertEqual(stopping[-1].mode, "idle")

    def test_gait_hysteresis_rejects_heading_noise_at_start_and_stop(self) -> None:
        core = armed_core(
            MODULE.ControlConfig(
                max_speed_mps=0.3,
                max_acceleration_mps2=100.0,
                max_deceleration_mps2=100.0,
                max_turn_rate_rad_s=100.0,
            )
        )
        core.accept_snapshot(
            snapshot(pressed=("w",), speed_modifiers=("shift",)),
            received_at_s=10.0,
        )
        core.synchronize_heading(0.0)
        self.assertEqual(core.command(now_s=10.0, dt_s=0.1).mode, "move")

        # Once active, noise near the 30-degree stop edge stops exactly once;
        # it cannot restart until alignment crosses the tighter 15-degree edge.
        stop_error = math.radians(30.0)
        active_modes = []
        for sequence, delta in enumerate((-0.002, 0.002, -0.002, 0.002), start=2):
            now = 10.0 + sequence * 0.01
            core.synchronize_heading(stop_error + delta)
            core.accept_snapshot(
                snapshot(
                    sequence=sequence,
                    timestamp=now,
                    pressed=("w",),
                    speed_modifiers=("shift",),
                ),
                received_at_s=now,
            )
            active_modes.append(core.command(now_s=now, dt_s=0.02).mode)
        self.assertEqual(active_modes, ["move", "idle", "idle", "idle"])

        # A request outside the 15-degree start edge stays idle. Crossing that
        # edge starts once, and drifting just outside it remains active because
        # the wider 30-degree stop edge has not been crossed.
        sequence = 6
        now = 10.06
        core.synchronize_heading(math.radians(16.0))
        core.accept_snapshot(
            snapshot(
                sequence=sequence,
                timestamp=now,
                pressed=("w",),
                speed_modifiers=("shift",),
            ),
            received_at_s=now,
        )
        self.assertEqual(core.command(now_s=now, dt_s=0.02).mode, "idle")

        restart_modes = []
        for sequence, error_deg in enumerate((15.1, 14.9, 15.1), start=7):
            now = 10.0 + sequence * 0.01
            core.synchronize_heading(math.radians(error_deg))
            core.accept_snapshot(
                snapshot(
                    sequence=sequence,
                    timestamp=now,
                    pressed=("w",),
                    speed_modifiers=("shift",),
                ),
                received_at_s=now,
            )
            restart_modes.append(core.command(now_s=now, dt_s=0.02).mode)
        self.assertEqual(restart_modes, ["idle", "move", "move"])

    def test_slow_walk_tolerates_small_measured_heading_error(self) -> None:
        config = immediate_config(max_speed_mps=0.3)
        aligned = armed_core(config)
        aligned.synchronize_heading(math.radians(5.0))
        aligned.accept_snapshot(
            snapshot(pressed=("w",), speed_modifiers=("ctrl",)),
            received_at_s=10.0,
        )
        slow = aligned.command(now_s=10.0, dt_s=0.1)
        self.assertEqual(slow.mode, "move")
        self.assertAlmostEqual(slow.speed_mps, 0.10)

        turning = armed_core(config)
        turning.synchronize_heading(math.radians(20.0))
        turning.accept_snapshot(
            snapshot(pressed=("w",), speed_modifiers=("ctrl",)),
            received_at_s=10.0,
        )
        self.assertEqual(turning.command(now_s=10.0, dt_s=0.1).mode, "idle")

    def test_small_effective_stick_input_can_enter_native_gait(self) -> None:
        core = armed_core(
            MODULE.ControlConfig(
                max_speed_mps=0.3,
                max_acceleration_mps2=100.0,
                max_deceleration_mps2=100.0,
                max_turn_rate_rad_s=100.0,
            )
        )
        # Raw 0.16 is only just above the default 0.15 radial deadzone.  It must
        # still enter the native gait without creating a second large deadzone,
        # while retaining a small amount of analog resolution above the floor.
        now = 10.0
        core.accept_snapshot(
            snapshot(sequence=1, timestamp=now, stick=(0.0, 0.16)),
            received_at_s=now,
        )
        first = core.command(now_s=now, dt_s=0.1)
        self.assertAlmostEqual(first.speed_mps, 0.10)
        moving = core.command(now_s=now, dt_s=0.1)
        self.assertEqual(moving.mode, "move")
        effective_magnitude = (0.16 - 0.15) / (1.0 - 0.15)
        expected_speed = 0.10 + (0.30 - 0.10) * effective_magnitude
        self.assertAlmostEqual(moving.speed_mps, expected_speed)

    def test_stick_uses_full_analog_range_above_native_gait_floor(self) -> None:
        core = armed_core(
            MODULE.ControlConfig(
                max_speed_mps=0.3,
                max_acceleration_mps2=100.0,
                max_deceleration_mps2=100.0,
                max_turn_rate_rad_s=100.0,
            )
        )
        # A raw magnitude of 0.575 maps to exactly 0.5 after the 0.15 radial
        # deadzone.  Half stick should therefore be halfway between the native
        # 0.10 m/s floor and the configured 0.30 m/s maximum, not remain pinned
        # to the floor as it did under max(minimum, maximum * magnitude).
        now = 10.0
        core.accept_snapshot(
            snapshot(sequence=1, timestamp=now, stick=(0.0, 0.575)),
            received_at_s=now,
        )

        first = core.command(now_s=now, dt_s=0.1)
        self.assertAlmostEqual(first.speed_mps, 0.10)
        moving = core.command(now_s=now, dt_s=0.1)

        self.assertEqual(moving.mode, "move")
        self.assertAlmostEqual(moving.speed_mps, 0.20)

    def test_v_is_edge_triggered_and_free_camera_forces_zero(self) -> None:
        core = armed_core(immediate_config())
        core.accept_snapshot(snapshot(pressed=("w",)), received_at_s=10.0)
        self.assertGreater(core.command(now_s=10.0, dt_s=0.1).speed_mps, 0.0)

        core.accept_snapshot(
            snapshot(sequence=2, timestamp=10.01, pressed=("w", "v")),
            received_at_s=10.01,
        )
        command = core.command(now_s=10.01, dt_s=0.1)
        self.assertTrue(core.free_camera)
        self.assertTrue(command.safe_stop)
        self.assertEqual(command.reason, "free_camera")
        self.assertEqual(command.speed_mps, 0.0)
        self.assertEqual(command.locomotion_mode, MODULE.SONIC_IDLE_MODE)

        # Holding V does not repeatedly toggle.
        core.accept_snapshot(
            snapshot(sequence=3, timestamp=10.02, pressed=("v",)),
            received_at_s=10.02,
        )
        self.assertTrue(core.free_camera)

        core.accept_snapshot(snapshot(sequence=4, timestamp=10.03), received_at_s=10.03)
        core.accept_snapshot(
            snapshot(sequence=5, timestamp=10.04, pressed=("v",)),
            received_at_s=10.04,
        )
        self.assertFalse(core.free_camera)

    def test_focus_loss_bypasses_deceleration(self) -> None:
        core = armed_core(immediate_config())
        core.accept_snapshot(snapshot(pressed=("w",)), received_at_s=10.0)
        self.assertGreater(core.command(now_s=10.0, dt_s=0.1).speed_mps, 0.0)
        core.accept_snapshot(
            snapshot(sequence=2, timestamp=10.01, focused=False, pressed=("w",)),
            received_at_s=10.01,
        )
        command = core.command(now_s=10.01, dt_s=0.001)
        self.assertEqual(command.mode, "deadman")
        self.assertEqual(command.reason, "focus_lost")
        self.assertEqual(command.speed_mps, 0.0)
        self.assertEqual(command.locomotion_mode, MODULE.SONIC_IDLE_MODE)

        core.accept_snapshot(
            snapshot(sequence=3, timestamp=10.02, focused=True, pressed=("w",)),
            received_at_s=10.02,
        )
        command = core.command(now_s=10.02, dt_s=0.1)
        self.assertEqual(command.reason, "awaiting_neutral")
        self.assertEqual(command.speed_mps, 0.0)

        core.accept_snapshot(
            snapshot(sequence=4, timestamp=10.03, focused=True),
            received_at_s=10.03,
        )
        self.assertEqual(core.command(now_s=10.03, dt_s=0.1).mode, "idle")
        core.accept_snapshot(
            snapshot(sequence=5, timestamp=10.04, focused=True, pressed=("w",)),
            received_at_s=10.04,
        )
        self.assertEqual(core.command(now_s=10.04, dt_s=0.1).mode, "move")

    def test_no_input_and_timeout_are_deadman_stops(self) -> None:
        core = MODULE.GameControlCore()
        command = core.command(now_s=10.0, dt_s=0.01)
        self.assertEqual(command.reason, "no_input")
        self.assertTrue(command.safe_stop)

        core.accept_snapshot(snapshot(pressed=("w",)), received_at_s=10.0)
        command = core.command(now_s=10.0, dt_s=0.1)
        self.assertEqual(command.reason, "awaiting_neutral")

        core.accept_snapshot(snapshot(sequence=2, timestamp=10.01), received_at_s=10.01)
        self.assertEqual(core.command(now_s=10.01, dt_s=0.01).mode, "idle")
        core.accept_snapshot(
            snapshot(sequence=3, timestamp=10.02, pressed=("w",)),
            received_at_s=10.02,
        )
        self.assertGreater(core.command(now_s=10.02, dt_s=0.1).speed_mps, 0.0)
        command = core.command(now_s=10.17, dt_s=0.01)
        self.assertEqual(command.reason, "input_timeout")
        self.assertEqual(command.speed_mps, 0.0)
        self.assertEqual(command.locomotion_mode, MODULE.SONIC_IDLE_MODE)

        core.accept_snapshot(
            snapshot(sequence=4, timestamp=10.171, pressed=("w",)),
            received_at_s=10.171,
        )
        command = core.command(now_s=10.171, dt_s=0.01)
        self.assertEqual(command.reason, "awaiting_neutral")

    def test_startup_stick_requires_neutral_before_arming(self) -> None:
        core = MODULE.GameControlCore(immediate_config())
        core.accept_snapshot(snapshot(stick=(0.0, 1.0)), received_at_s=10.0)
        self.assertEqual(
            core.command(now_s=10.0, dt_s=0.1).reason,
            "awaiting_neutral",
        )
        core.accept_snapshot(
            snapshot(sequence=2, timestamp=10.01, stick=(0.0, 0.0)),
            received_at_s=10.01,
        )
        self.assertEqual(core.command(now_s=10.01, dt_s=0.1).mode, "idle")

    def test_replay_stale_and_future_snapshots_do_not_refresh_deadman(self) -> None:
        core = MODULE.GameControlCore()
        core.accept_snapshot(snapshot(sequence=7, timestamp=10.0), received_at_s=10.0)
        with self.assertRaisesRegex(MODULE.InputRejectedError, "increase"):
            core.accept_snapshot(snapshot(sequence=7, timestamp=10.1), received_at_s=10.1)
        with self.assertRaisesRegex(MODULE.InputRejectedError, "stale"):
            core.accept_snapshot(snapshot(sequence=8, timestamp=10.0), received_at_s=10.3)
        with self.assertRaisesRegex(MODULE.InputRejectedError, "future"):
            core.accept_snapshot(snapshot(sequence=8, timestamp=10.3), received_at_s=10.2)

        command = core.command(now_s=10.21, dt_s=0.01)
        self.assertEqual(command.reason, "input_timeout")


@unittest.skipUnless(
    hasattr(socket, "SOCK_SEQPACKET") and hasattr(socket, "SO_PEERCRED"),
    "Linux Unix seqpacket credentials are required",
)
class UnixSeqpacketInputServerTest(unittest.TestCase):
    def test_repeated_accept_timeout_does_not_repeat_socket_mutation(self) -> None:
        class EmptyServerSocket:
            def __init__(self) -> None:
                self.timeouts: list[float | None] = []

            def settimeout(self, value: float | None) -> None:
                self.timeouts.append(value)

            @staticmethod
            def accept():
                raise BlockingIOError()

        server = MODULE.UnixSeqpacketInputServer("/tmp/not-opened.sock")
        fake = EmptyServerSocket()
        server._socket = fake
        for _ in range(2):
            with self.assertRaises(BlockingIOError):
                server.accept(timeout_s=0.0)
        self.assertEqual(fake.timeouts, [0.0])

    def test_reliable_local_receiver_checks_peer_and_decodes_packet(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "matrix-input.sock"
            with MODULE.UnixSeqpacketInputServer(path) as server:
                mode = stat.S_IMODE(path.stat().st_mode)
                self.assertEqual(mode, 0o600)

                client = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
                try:
                    client.connect(os.fspath(path))
                    client.sendall(MODULE.encode_input_packet(snapshot()))
                    with server.accept(timeout_s=1.0) as connection:
                        received = connection.receive(timeout_s=1.0)
                        self.assertEqual(received, snapshot())
                        self.assertEqual(connection.credentials.uid, os.getuid())
                finally:
                    client.close()
            self.assertFalse(path.exists())

    def test_server_refuses_to_replace_existing_path(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "matrix-input.sock"
            path.write_text("owner data", encoding="utf-8")
            server = MODULE.UnixSeqpacketInputServer(path)
            with self.assertRaises(FileExistsError):
                server.open()
            self.assertEqual(path.read_text(encoding="utf-8"), "owner data")


if __name__ == "__main__":
    unittest.main()
