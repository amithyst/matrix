from __future__ import annotations

from dataclasses import replace
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "run_matrix_sonic.py"
SPEC = importlib.util.spec_from_file_location("run_matrix_sonic", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
GAME_CONTROL = sys.modules["matrix_game_control"]
MC_COMMANDS = sys.modules["matrix_mc_commands"]
WORLD_STATE = sys.modules["matrix_world_state"]


class MatrixSonicRuntimeTest(unittest.TestCase):
    @staticmethod
    def snapshot(
        *,
        step_index: int = 0,
        sim_time: float = 0.0,
        qpos_len: int = 36,
        qvel_len: int = 35,
        ctrl_len: int = 29,
        torque_len: int = 29,
        fall_detected: bool = False,
        reset_count: int = 0,
        last_reset_reason: str | None = None,
        low_cmd_fresh: bool = False,
        low_cmd_received: bool = False,
        low_cmd_age_s: float | None = None,
        elastic_band_scale: float = 0.0,
    ) -> SimpleNamespace:
        return SimpleNamespace(
            step_index=step_index,
            sim_time=sim_time,
            qpos=[0.0] * qpos_len,
            qvel=[0.0] * qvel_len,
            ctrl=[0.0] * ctrl_len,
            applied_torque=[0.0] * torque_len,
            fall_detected=fall_detected,
            reset_count=reset_count,
            last_reset_reason=last_reset_reason,
            low_cmd_fresh=low_cmd_fresh,
            low_cmd_received=low_cmd_received,
            low_cmd_age_s=low_cmd_age_s,
            elastic_band_scale=elastic_band_scale,
        )

    @classmethod
    def snapshot_with_yaw(cls, yaw_rad: float, **kwargs) -> SimpleNamespace:
        snapshot = cls.snapshot(**kwargs)
        snapshot.qpos[3] = math.cos(yaw_rad / 2.0)
        snapshot.qpos[6] = math.sin(yaw_rad / 2.0)
        return snapshot

    @staticmethod
    def game_input_snapshot(
        sequence: int,
        timestamp_monotonic_s: float,
        *,
        w: bool = False,
        camera_yaw_rad: float = 0.0,
    ):
        return GAME_CONTROL.InputSnapshot.from_mapping(
            {
                "protocol": GAME_CONTROL.PROTOCOL_NAME,
                "sequence": sequence,
                "timestamp_monotonic_s": timestamp_monotonic_s,
                "focused": True,
                "camera_yaw_rad": camera_yaw_rad,
                "keys": {
                    "w": w,
                    "a": False,
                    "s": False,
                    "d": False,
                    "q": False,
                    "e": False,
                    "v": False,
                    "ctrl": False,
                    "shift": False,
                },
                "move_stick": {"right": 0.0, "forward": 0.0},
            }
        )

    @staticmethod
    def game_command_request(
        command_text: str,
        *,
        sequence: int,
        request_character: str,
        session: str = "a" * 32,
    ):
        parsed = MC_COMMANDS.parse_mc_command(command_text)
        return MC_COMMANDS.GameCommandRequest(
            session=session,
            sequence=sequence,
            request_id="cmd-" + request_character * 32,
            command=parsed.command,
        )

    @staticmethod
    def process_is_running(pid: int) -> bool:
        try:
            state = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8").split()[2]
        except (FileNotFoundError, IndexError, OSError):
            return False
        return state != "Z"

    def test_planner_endpoint_requires_loopback_tcp(self) -> None:
        self.assertEqual(MODULE._loopback_zmq_port("tcp://127.0.0.1:5556"), 5556)
        self.assertEqual(MODULE._loopback_zmq_port("tcp://[::1]:6000"), 6000)
        for endpoint in (
            "tcp://0.0.0.0:5556",
            "tcp://192.168.1.2:5556",
            "udp://127.0.0.1:5556",
            "tcp://127.0.0.1",
        ):
            with self.subTest(endpoint=endpoint):
                with self.assertRaises(ValueError):
                    MODULE._loopback_zmq_port(endpoint)

    def test_root_up_z_is_one_for_upright_quaternion(self) -> None:
        self.assertAlmostEqual(MODULE._root_up_z([0, 0, 0, 1, 0, 0, 0]), 1.0)

    def test_root_up_z_is_negative_for_upside_down_quaternion(self) -> None:
        self.assertAlmostEqual(MODULE._root_up_z([0, 0, 0, 0, 1, 0, 0]), -1.0)

    def test_world_safe_checkpoint_rejects_fall_motion_but_allows_horizontal_walk(self) -> None:
        def upright_snapshot() -> SimpleNamespace:
            snapshot = self.snapshot()
            snapshot.qpos[2] = 0.8
            snapshot.qpos[3] = 1.0
            return snapshot

        walking = upright_snapshot()
        walking.qvel[0] = 2.5
        walking.qvel[1] = -2.5
        self.assertTrue(MODULE._snapshot_world_upright(walking))

        unsafe_velocities = (
            (2, MODULE._WORLD_SAFE_MAX_VERTICAL_SPEED_M_S + 0.01),
            (2, math.nan),
            (3, MODULE._WORLD_SAFE_MAX_TILT_RATE_RAD_S + 0.01),
            (3, math.inf),
            (4, -MODULE._WORLD_SAFE_MAX_TILT_RATE_RAD_S - 0.01),
            (4, -math.inf),
        )
        for index, value in unsafe_velocities:
            with self.subTest(qvel_index=index, value=value):
                snapshot = upright_snapshot()
                snapshot.qvel[index] = value
                self.assertFalse(MODULE._snapshot_world_upright(snapshot))

        tilted = upright_snapshot()
        tilt_radians = math.acos(MODULE._WORLD_SAFE_MIN_ROOT_UP_Z - 0.01)
        tilted.qpos[3] = math.cos(tilt_radians / 2.0)
        tilted.qpos[4] = math.sin(tilt_radians / 2.0)
        self.assertFalse(MODULE._snapshot_world_upright(tilted))

    def test_root_yaw_uses_normalized_mujoco_wxyz_quaternion(self) -> None:
        half = math.pi / 4.0
        qpos = [
            0.0,
            0.0,
            0.0,
            2.0 * math.cos(half),
            0.0,
            0.0,
            2.0 * math.sin(half),
        ]
        self.assertAlmostEqual(MODULE._root_yaw_rad(qpos), math.pi / 2.0)
        with self.assertRaisesRegex(ValueError, "zero"):
            MODULE._root_yaw_rad([0.0] * 7)

    def test_heading_anchor_captures_only_first_fresh_lowcmd_edge(self) -> None:
        initial = self.snapshot_with_yaw(
            0.25,
            step_index=3,
            sim_time=0.015,
            low_cmd_fresh=False,
        )
        telemetry = MODULE._HeadingAnchorTelemetry(0.25, initial)
        self.assertIsNone(
            telemetry.status_fields()["root_yaw_first_fresh_lowcmd_rad"]
        )

        still_stale = self.snapshot_with_yaw(
            0.30,
            step_index=4,
            sim_time=0.020,
            low_cmd_fresh=False,
        )
        self.assertFalse(telemetry.observe(still_stale, wall_elapsed_s=0.10))
        first_fresh = self.snapshot_with_yaw(
            0.40,
            step_index=5,
            sim_time=0.025,
            low_cmd_fresh=True,
        )
        self.assertTrue(telemetry.observe(first_fresh, wall_elapsed_s=0.125))
        captured = telemetry.status_fields()
        self.assertEqual(captured["heading_anchor_source"], "initial_snapshot")
        self.assertEqual(captured["root_yaw_initial_rad"], 0.25)
        self.assertEqual(captured["root_yaw_first_fresh_lowcmd_rad"], 0.4)
        self.assertEqual(captured["root_yaw_startup_delta_rad"], 0.15)
        self.assertEqual(captured["first_fresh_lowcmd_step_index"], 5)
        self.assertEqual(captured["first_fresh_lowcmd_sim_time_s"], 0.025)
        self.assertEqual(captured["first_fresh_lowcmd_wall_elapsed_s"], 0.125)

        stale_again = self.snapshot_with_yaw(
            1.0,
            step_index=6,
            sim_time=0.030,
            low_cmd_fresh=False,
        )
        fresh_again = self.snapshot_with_yaw(
            1.2,
            step_index=7,
            sim_time=0.035,
            low_cmd_fresh=True,
        )
        self.assertFalse(telemetry.observe(stale_again, wall_elapsed_s=0.15))
        self.assertFalse(telemetry.observe(fresh_again, wall_elapsed_s=0.175))
        self.assertEqual(telemetry.status_fields(), captured)

    def test_heading_anchor_records_initially_fresh_snapshot_explicitly(self) -> None:
        initial = self.snapshot_with_yaw(
            -0.75,
            step_index=12,
            sim_time=0.06,
            low_cmd_fresh=True,
        )
        telemetry = MODULE._HeadingAnchorTelemetry(-0.75, initial)

        self.assertEqual(
            telemetry.status_fields(),
            {
                "heading_anchor_source": "initial_snapshot",
                "root_yaw_initial_rad": -0.75,
                "root_yaw_first_fresh_lowcmd_rad": -0.75,
                "root_yaw_startup_delta_rad": 0.0,
                "first_fresh_lowcmd_step_index": 12,
                "first_fresh_lowcmd_sim_time_s": 0.06,
                "first_fresh_lowcmd_wall_elapsed_s": 0.0,
            },
        )

    def test_heading_anchor_startup_delta_wraps_across_pi(self) -> None:
        initial_yaw = math.pi - 0.05
        telemetry = MODULE._HeadingAnchorTelemetry(
            initial_yaw,
            self.snapshot_with_yaw(initial_yaw, low_cmd_fresh=False),
        )
        fresh_yaw = -math.pi + 0.05
        telemetry.observe(
            self.snapshot_with_yaw(
                fresh_yaw,
                step_index=1,
                sim_time=0.005,
                low_cmd_fresh=True,
            ),
            wall_elapsed_s=0.02,
        )

        self.assertAlmostEqual(
            telemetry.status_fields()["root_yaw_startup_delta_rad"], 0.1
        )

    def test_heading_anchor_telemetry_does_not_change_control_command(self) -> None:
        core = GAME_CONTROL.GameControlCore(
            GAME_CONTROL.ControlConfig(
                max_speed_mps=0.3,
                max_acceleration_mps2=100.0,
                max_deceleration_mps2=100.0,
                max_turn_rate_rad_s=100.0,
                max_step_s=1.0,
            )
        )
        initial_yaw = 0.5
        current_yaw = 0.75
        measured_heading = GAME_CONTROL.wrap_angle_rad(current_yaw - initial_yaw)
        core.synchronize_heading(measured_heading)

        def input_snapshot(sequence: int, timestamp: float, *, w: bool):
            return GAME_CONTROL.InputSnapshot.from_mapping(
                {
                    "protocol": GAME_CONTROL.PROTOCOL_NAME,
                    "sequence": sequence,
                    "timestamp_monotonic_s": timestamp,
                    "focused": True,
                    "camera_yaw_rad": 0.4,
                    "keys": {
                        "w": w,
                        "a": False,
                        "s": False,
                        "d": False,
                        "q": False,
                        "e": False,
                        "v": False,
                        "ctrl": False,
                        "shift": False,
                    },
                    "move_stick": {"right": 0.0, "forward": 0.0},
                }
            )

        core.accept_snapshot(input_snapshot(1, 10.0, w=False), received_at_s=10.0)
        self.assertEqual(core.command(now_s=10.0, dt_s=0.1).mode, "idle")
        core.accept_snapshot(input_snapshot(2, 10.01, w=True), received_at_s=10.01)
        heading_before_telemetry = core.heading_rad

        telemetry = MODULE._HeadingAnchorTelemetry(
            initial_yaw,
            self.snapshot_with_yaw(initial_yaw, low_cmd_fresh=False),
        )
        telemetry.observe(
            self.snapshot_with_yaw(
                current_yaw,
                step_index=4,
                sim_time=0.02,
                low_cmd_fresh=True,
            ),
            wall_elapsed_s=0.1,
        )

        self.assertAlmostEqual(core.heading_rad, heading_before_telemetry)
        command = core.command(now_s=10.01, dt_s=0.1)
        self.assertEqual(command.mode, "move")
        expected_heading = GAME_CONTROL.wrap_angle_rad(
            measured_heading + GAME_CONTROL.MAX_MEASURED_FACING_LEAD_RAD
        )
        self.assertAlmostEqual(command.movement[0], math.cos(expected_heading))
        self.assertAlmostEqual(command.movement[1], math.sin(expected_heading))

        source_lines = [
            line.strip()
            for line in SCRIPT_PATH.read_text(encoding="utf-8").splitlines()
        ]
        self.assertEqual(
            source_lines.count(
                "status.update(heading_anchor_telemetry.status_fields())"
            ),
            1,
        )
        self.assertEqual(
            source_lines.count(
                "final_status.update(heading_anchor_telemetry.status_fields())"
            ),
            1,
        )

    def test_game_control_waits_for_fresh_lowcmd_and_released_startup_band(
        self,
    ) -> None:
        core = GAME_CONTROL.GameControlCore(
            GAME_CONTROL.ControlConfig(
                max_speed_mps=0.3,
                max_acceleration_mps2=100.0,
                max_deceleration_mps2=100.0,
                max_turn_rate_rad_s=100.0,
                max_step_s=1.0,
            )
        )
        startup = self.snapshot(low_cmd_fresh=False, elastic_band_scale=1.0)
        gate = MODULE._GameSonicReadinessGate(startup)

        def command_for(
            sonic_snapshot: SimpleNamespace,
            input_sequence: int,
            now_s: float,
            *,
            w: bool,
        ):
            gate.begin_frame(sonic_snapshot, core)
            core.accept_snapshot(
                self.game_input_snapshot(input_sequence, now_s, w=w),
                received_at_s=now_s,
            )
            return gate.apply(core.command(now_s=now_s, dt_s=0.02), core)

        startup_stop = command_for(startup, 1, 10.0, w=False)
        self.assertEqual(startup_stop.reason, "sonic_not_ready")
        self.assertTrue(startup_stop.safe_stop)
        self.assertEqual(startup_stop.speed_mps, 0.0)

        # A fresh LowCmd is not sufficient while the startup restraint still
        # has a material scale. Holding W during this phase stays hard-zero.
        band_fading = self.snapshot(
            low_cmd_fresh=True,
            elastic_band_scale=(
                2.0 * MODULE._GameSonicReadinessGate.ELASTIC_BAND_ZERO_ABS_TOL
            ),
        )
        fading_stop = command_for(band_fading, 2, 10.01, w=True)
        self.assertEqual(fading_stop.reason, "sonic_not_ready")
        self.assertEqual(fading_stop.speed_mps, 0.0)

        # Near-zero is considered released, but W held across readiness cannot
        # bypass the core's neutral re-arm latch.
        ready = self.snapshot(
            low_cmd_fresh=True,
            elastic_band_scale=(
                0.5 * MODULE._GameSonicReadinessGate.ELASTIC_BAND_ZERO_ABS_TOL
            ),
        )
        held_at_release = command_for(ready, 3, 10.02, w=True)
        self.assertEqual(held_at_release.reason, "awaiting_neutral")
        self.assertEqual(held_at_release.speed_mps, 0.0)

        self.assertEqual(command_for(ready, 4, 10.03, w=False).mode, "idle")
        moving = command_for(ready, 5, 10.04, w=True)
        self.assertEqual(moving.mode, "move")
        self.assertGreater(moving.speed_mps, 0.0)

    def test_game_control_lowcmd_dropout_requires_neutral_after_recovery(
        self,
    ) -> None:
        core = GAME_CONTROL.GameControlCore(
            GAME_CONTROL.ControlConfig(
                max_speed_mps=0.3,
                max_acceleration_mps2=100.0,
                max_deceleration_mps2=100.0,
                max_turn_rate_rad_s=100.0,
                max_step_s=1.0,
            )
        )
        ready = self.snapshot(low_cmd_fresh=True, elastic_band_scale=0.0)
        gate = MODULE._GameSonicReadinessGate(ready)

        def command_for(
            sonic_snapshot: SimpleNamespace,
            input_sequence: int,
            now_s: float,
            *,
            w: bool,
        ):
            gate.begin_frame(sonic_snapshot, core)
            core.accept_snapshot(
                self.game_input_snapshot(input_sequence, now_s, w=w),
                received_at_s=now_s,
            )
            return gate.apply(core.command(now_s=now_s, dt_s=0.02), core)

        self.assertEqual(command_for(ready, 1, 20.0, w=False).mode, "idle")
        self.assertEqual(command_for(ready, 2, 20.01, w=True).mode, "move")

        stale = self.snapshot(low_cmd_fresh=False, elastic_band_scale=0.0)
        with mock.patch.object(
            core, "invalidate_input", wraps=core.invalidate_input
        ) as invalidate_input:
            dropped = command_for(stale, 3, 20.02, w=True)
        self.assertEqual(
            invalidate_input.call_args_list,
            [mock.call("low_cmd_stale"), mock.call("sonic_not_ready")],
        )
        self.assertEqual(dropped.reason, "sonic_not_ready")
        self.assertEqual(dropped.speed_mps, 0.0)

        # The provider keeps reporting W, but fresh LowCmd recovery alone must
        # not restart locomotion. A neutral frame is required first.
        held_after_recovery = command_for(ready, 4, 20.03, w=True)
        self.assertEqual(held_after_recovery.reason, "awaiting_neutral")
        self.assertEqual(held_after_recovery.speed_mps, 0.0)
        self.assertEqual(command_for(ready, 5, 20.04, w=False).mode, "idle")
        resumed = command_for(ready, 6, 20.05, w=True)
        self.assertEqual(resumed.mode, "move")
        self.assertGreater(resumed.speed_mps, 0.0)

    def test_not_ready_batch_neutral_then_w_cannot_prearm_recovery(self) -> None:
        core = GAME_CONTROL.GameControlCore(
            GAME_CONTROL.ControlConfig(
                max_speed_mps=0.3,
                max_acceleration_mps2=100.0,
                max_deceleration_mps2=100.0,
                max_turn_rate_rad_s=100.0,
                max_step_s=1.0,
            )
        )
        stale = self.snapshot(low_cmd_fresh=False, elastic_band_scale=0.0)
        gate = MODULE._GameSonicReadinessGate(stale)
        core.synchronize_heading(0.0)
        gate.begin_frame(stale, core)

        # Model two packets drained in one control poll: neutral first clears
        # the core latch, then held W becomes the latest snapshot.
        core.accept_snapshot(
            self.game_input_snapshot(1, 30.0, w=False),
            received_at_s=30.0,
        )
        core.accept_snapshot(
            self.game_input_snapshot(
                2,
                30.001,
                w=True,
                camera_yaw_rad=math.pi / 2.0,
            ),
            received_at_s=30.001,
        )
        candidate = core.command(now_s=30.001, dt_s=0.02)
        stopped = gate.apply(candidate, core)
        self.assertEqual(stopped.reason, "sonic_not_ready")
        self.assertEqual(stopped.speed_mps, 0.0)
        self.assertAlmostEqual(stopped.facing[0], 1.0)
        self.assertAlmostEqual(stopped.facing[1], 0.0)
        self.assertAlmostEqual(core.heading_rad, 0.0)

        ready = self.snapshot(low_cmd_fresh=True, elastic_band_scale=0.0)
        gate.begin_frame(ready, core)
        core.accept_snapshot(
            self.game_input_snapshot(3, 30.02, w=True),
            received_at_s=30.02,
        )
        recovered = gate.apply(
            core.command(now_s=30.02, dt_s=0.02),
            core,
        )
        self.assertEqual(recovered.reason, "awaiting_neutral")
        self.assertEqual(recovered.speed_mps, 0.0)

        core.accept_snapshot(
            self.game_input_snapshot(4, 30.03, w=False),
            received_at_s=30.03,
        )
        neutral = gate.apply(
            core.command(now_s=30.03, dt_s=0.02),
            core,
        )
        self.assertEqual(neutral.mode, "idle")
        self.assertAlmostEqual(neutral.facing[0], 1.0)
        self.assertAlmostEqual(neutral.facing[1], 0.0)

    def test_readiness_and_snapshot_validation_reject_non_boolean_freshness(
        self,
    ) -> None:
        for invalid in (1, "false", None):
            with self.subTest(invalid=invalid):
                snapshot = self.snapshot(low_cmd_fresh=invalid)
                self.assertFalse(
                    MODULE._GameSonicReadinessGate.snapshot_ready(snapshot)
                )
                self.assertEqual(
                    MODULE._snapshot_validation_error(snapshot),
                    f"snapshot_invalid_low_cmd_fresh:{invalid!r}",
                )
        for invalid in (0, "0", "0.0", True):
            with self.subTest(invalid_elastic_band=invalid):
                snapshot = self.snapshot(
                    low_cmd_fresh=True,
                    elastic_band_scale=invalid,
                )
                self.assertFalse(
                    MODULE._GameSonicReadinessGate.snapshot_ready(snapshot)
                )
                self.assertEqual(
                    MODULE._snapshot_validation_error(snapshot),
                    f"snapshot_invalid_elastic_band_scale:{invalid!r}",
                )

    def test_sonic_fall_recovery_holds_idle_until_stably_upright(self) -> None:
        gate = MODULE._GameFallRecoveryGate(timeout_s=5.0)

        def pose(*, root_z: float, upright: bool) -> SimpleNamespace:
            snapshot = self.snapshot(
                fall_detected=True,
                low_cmd_fresh=True,
                elastic_band_scale=0.0,
            )
            snapshot.qpos[2] = root_z
            snapshot.qpos[3] = 1.0 if upright else 0.0
            snapshot.qpos[4] = 0.0 if upright else 1.0
            return snapshot

        self.assertEqual(
            gate.observe(pose(root_z=0.19, upright=False), now_s=10.0),
            "entered",
        )
        self.assertTrue(gate.recovering)
        self.assertTrue(gate.current_fallen)
        self.assertEqual(gate.episodes, 1)
        self.assertEqual(gate.last_entry_source, "sonic_fall_detected")
        self.assertEqual(gate.native_mode, 5)
        self.assertEqual(gate.target_height, 0.4)

        # Height alone is insufficient: the base must also be upright and the
        # native readiness contract must stay healthy for a continuous second.
        self.assertIsNone(
            gate.observe(pose(root_z=0.72, upright=False), now_s=10.5)
        )
        self.assertIsNone(
            gate.observe(pose(root_z=0.72, upright=True), now_s=11.0)
        )
        self.assertIsNone(
            gate.observe(pose(root_z=0.72, upright=True), now_s=11.99)
        )
        self.assertIsNone(
            gate.observe(pose(root_z=0.72, upright=True), now_s=12.0)
        )
        self.assertEqual(
            gate.observe(pose(root_z=0.72, upright=True), now_s=13.0),
            "recovered",
        )
        self.assertFalse(gate.recovering)
        self.assertEqual(gate.recoveries, 1)
        self.assertAlmostEqual(gate.last_duration_s, 3.0)
        status = gate.status(now_s=13.0)
        self.assertEqual(status["state"], "monitoring")
        self.assertEqual(status["policy_command"], "KNEEL_TWO_LEGS_TO_IDLE")
        self.assertTrue(status["recovered_requires_neutral"])

        # SONIC's historical fall flag remains sticky. A later current-height
        # fall must still start a distinct recovery episode.
        self.assertEqual(
            gate.observe(pose(root_z=0.18, upright=False), now_s=14.0),
            "entered",
        )
        self.assertEqual(gate.episodes, 2)

    def test_sonic_fall_recovery_timeout_is_telemetry_not_runtime_exit(self) -> None:
        gate = MODULE._GameFallRecoveryGate(timeout_s=2.0)
        fallen = self.snapshot(
            fall_detected=True,
            low_cmd_fresh=True,
            elastic_band_scale=0.0,
        )
        fallen.qpos[2] = 0.15
        fallen.qpos[4] = 1.0
        self.assertEqual(gate.observe(fallen, now_s=1.0), "entered")
        self.assertIsNone(gate.observe(fallen, now_s=3.1))
        self.assertTrue(gate.recovering)
        self.assertTrue(gate.timed_out)
        self.assertEqual(gate.status(now_s=3.1)["state"], "recovering_timeout")

    def test_sonic_fall_recovery_debounces_side_fall_above_native_height(self) -> None:
        gate = MODULE._GameFallRecoveryGate(timeout_s=5.0)
        side_fallen = self.snapshot(
            fall_detected=False,
            low_cmd_fresh=True,
            elastic_band_scale=0.0,
        )
        side_fallen.qpos[2] = 0.34
        side_fallen.qpos[3] = 0.0
        side_fallen.qpos[4] = 1.0

        self.assertIsNone(gate.observe(side_fallen, now_s=1.0))
        self.assertFalse(gate.current_fallen)
        self.assertTrue(gate.pose_candidate)
        self.assertFalse(gate.recovering)
        self.assertIsNone(gate.observe(side_fallen, now_s=1.34))
        self.assertEqual(gate.observe(side_fallen, now_s=1.36), "entered")
        self.assertTrue(gate.recovering)
        self.assertEqual(gate.last_entry_source, "pose_debounce")
        status = gate.status(now_s=1.36)
        self.assertEqual(status["last_entry_source"], "pose_debounce")
        self.assertEqual(status["pose_trigger_height_m"], 0.45)
        self.assertEqual(status["pose_trigger_up_z"], 0.5)
        self.assertEqual(status["pose_trigger_hold_s"], 0.35)

    def test_sonic_fall_recovery_pose_debounce_resets_when_pose_clears(self) -> None:
        gate = MODULE._GameFallRecoveryGate(timeout_s=5.0)

        def pose(*, root_z: float, upright: bool) -> SimpleNamespace:
            snapshot = self.snapshot(
                fall_detected=False,
                low_cmd_fresh=True,
                elastic_band_scale=0.0,
            )
            snapshot.qpos[2] = root_z
            snapshot.qpos[3] = 1.0 if upright else 0.0
            snapshot.qpos[4] = 0.0 if upright else 1.0
            return snapshot

        self.assertIsNone(
            gate.observe(pose(root_z=0.34, upright=False), now_s=1.0)
        )
        self.assertIsNone(
            gate.observe(pose(root_z=0.70, upright=True), now_s=1.2)
        )
        self.assertIsNone(
            gate.observe(pose(root_z=0.34, upright=False), now_s=1.5)
        )
        self.assertIsNone(
            gate.observe(pose(root_z=0.34, upright=False), now_s=1.84)
        )
        self.assertEqual(
            gate.observe(pose(root_z=0.34, upright=False), now_s=1.86),
            "entered",
        )

        upright_low = MODULE._GameFallRecoveryGate(timeout_s=5.0)
        self.assertIsNone(
            upright_low.observe(pose(root_z=0.40, upright=True), now_s=3.0)
        )
        self.assertIsNone(
            upright_low.observe(pose(root_z=0.40, upright=True), now_s=4.0)
        )
        self.assertFalse(upright_low.pose_candidate)
        self.assertFalse(upright_low.recovering)

        high_side = MODULE._GameFallRecoveryGate(timeout_s=5.0)
        self.assertIsNone(
            high_side.observe(pose(root_z=0.45, upright=False), now_s=5.0)
        )
        self.assertIsNone(
            high_side.observe(pose(root_z=0.45, upright=False), now_s=6.0)
        )
        self.assertFalse(high_side.pose_candidate)
        self.assertFalse(high_side.recovering)

    def test_sonic_fall_recovery_redebounces_later_side_fall(self) -> None:
        gate = MODULE._GameFallRecoveryGate(timeout_s=10.0)

        def pose(*, root_z: float, upright: bool) -> SimpleNamespace:
            snapshot = self.snapshot(
                fall_detected=False,
                low_cmd_fresh=True,
                elastic_band_scale=0.0,
            )
            snapshot.qpos[2] = root_z
            snapshot.qpos[3] = 1.0 if upright else 0.0
            snapshot.qpos[4] = 0.0 if upright else 1.0
            return snapshot

        side = pose(root_z=0.34, upright=False)
        upright = pose(root_z=0.72, upright=True)
        self.assertIsNone(gate.observe(side, now_s=1.0))
        self.assertEqual(gate.observe(side, now_s=1.36), "entered")
        self.assertIsNone(gate.observe(upright, now_s=3.50))
        self.assertEqual(gate.observe(upright, now_s=4.50), "recovered")
        self.assertEqual(gate.recoveries, 1)

        self.assertIsNone(gate.observe(side, now_s=5.0))
        self.assertFalse(gate.recovering)
        self.assertEqual(gate.observe(side, now_s=5.36), "entered")
        self.assertEqual(gate.episodes, 2)
        self.assertEqual(gate.last_entry_source, "pose_debounce")

    def test_sonic_fall_recovery_validation_is_interactive_only(self) -> None:
        allowed = SimpleNamespace(
            game_fall_recovery="sonic",
            game_fall_recovery_timeout=15.0,
            control_source="game",
            fail_on_fall=False,
            qualified_runtime=False,
        )
        MODULE._validate_game_fall_recovery(allowed)

        invalid = (
            ("control", {"control_source": "planner"}),
            ("conflicts", {"fail_on_fall": True}),
            ("qualified", {"qualified_runtime": True}),
            ("positive", {"game_fall_recovery_timeout": 0.0}),
        )
        for expected, changes in invalid:
            values = vars(allowed).copy()
            values.update(changes)
            with self.subTest(changes=changes), self.assertRaisesRegex(
                SystemExit, expected
            ):
                MODULE._validate_game_fall_recovery(SimpleNamespace(**values))

    def test_physical_recovery_validation_requires_local_worker_and_models(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            worker = root / "worker.py"
            model = root / "prone-v1.onnx"
            fallback = root / "prone-v2.onnx"
            amp_config = root / "amp.json"
            amp_model = root / "amp.onnx"
            for path in (worker, model, fallback, amp_config, amp_model):
                path.write_bytes(b"test")
            values = {
                "game_fall_recovery": "physical",
                "game_fall_recovery_timeout": 20.0,
                "control_source": "game",
                "fail_on_fall": False,
                "qualified_runtime": False,
                "physical_recovery_worker": worker,
                "physical_recovery_python": sys.executable,
                "physical_recovery_model": model,
                "physical_recovery_fallback_model": [fallback],
                "physical_recovery_amp_config": amp_config,
                "physical_recovery_amp_model": amp_model,
                "physical_recovery_amp_config_sha256": "1" * 64,
                "physical_recovery_amp_model_sha256": "2" * 64,
                "physical_recovery_control_socket": root / "control.sock",
                "physical_recovery_sonic_control_socket": root / "sonic.sock",
                "physical_recovery_fallback_after_seconds": 8.0,
                "physical_recovery_stable_hold_seconds": 1.5,
                "physical_recovery_timeout_seconds": 30.0,
                "physical_recovery_sonic_prewarm_timeout_seconds": 35.0,
            }
            MODULE._validate_game_fall_recovery(SimpleNamespace(**values))

            values["physical_recovery_model"] = root / "missing.onnx"
            with self.assertRaisesRegex(SystemExit, "model is missing"):
                MODULE._validate_game_fall_recovery(SimpleNamespace(**values))

    def test_absolute_physics_pacing_compensates_sleep_overshoot(self) -> None:
        with mock.patch.object(
            MODULE.time, "perf_counter", side_effect=[9.996, 10.00025]
        ), mock.patch.object(MODULE.time, "sleep") as sleep:
            next_deadline = MODULE._pace_absolute_deadline(10.0, 0.005)

        sleep.assert_called_once()
        self.assertAlmostEqual(sleep.call_args.args[0], 0.004)
        self.assertAlmostEqual(next_deadline, 10.005)
        self.assertIn(
            "simulator.step_once(rate_limit=False)",
            SCRIPT_PATH.read_text(encoding="utf-8"),
        )

    def test_absolute_physics_pacing_resets_after_sustained_overrun(self) -> None:
        with mock.patch.object(
            MODULE.time, "perf_counter", return_value=10.011
        ), mock.patch.object(MODULE.time, "sleep") as sleep:
            next_deadline = MODULE._pace_absolute_deadline(10.0, 0.005)

        sleep.assert_not_called()
        self.assertAlmostEqual(next_deadline, 10.016)

    def test_absolute_physics_pacing_does_not_accumulate_sleep_overshoot(self) -> None:
        clock = [0.0]

        def sleep_with_overshoot(duration: float) -> None:
            clock[0] += duration + 0.00025

        deadline = 0.005
        with mock.patch.object(
            MODULE.time, "perf_counter", side_effect=lambda: clock[0]
        ), mock.patch.object(MODULE.time, "sleep", side_effect=sleep_with_overshoot):
            for _ in range(200):
                deadline = MODULE._pace_absolute_deadline(deadline, 0.005)

        self.assertAlmostEqual(clock[0], 1.00025)
        self.assertAlmostEqual(deadline, 1.005)

    def test_parse_args_accepts_low_preset_and_rejects_off_table_scale(self) -> None:
        with mock.patch.object(
            sys,
            "argv",
            [
                "run_matrix_sonic.py",
                "--model",
                os.fspath(SCRIPT_PATH),
                "--sonic-root",
                "/tmp",
                "--game-applied-mouse-speed-scale",
                "0.01",
            ],
        ):
            parsed = MODULE._parse_args()
        self.assertEqual(parsed.game_applied_mouse_speed_scale, 0.01)

        with mock.patch.object(
            sys,
            "argv",
            [
                "run_matrix_sonic.py",
                "--model",
                os.fspath(SCRIPT_PATH),
                "--sonic-root",
                "/tmp",
                "--game-applied-mouse-speed-scale",
                "0.15",
            ],
        ), self.assertRaises(SystemExit):
            MODULE._parse_args()

    def test_parse_args_exposes_all_explicit_x11_camera_sources(self) -> None:
        for source in ("x11-mirror", "x11-core-gated", "x11-absolute"):
            with self.subTest(source=source), mock.patch.object(
                sys,
                "argv",
                [
                    "run_matrix_sonic.py",
                    "--model",
                    os.fspath(SCRIPT_PATH),
                    "--sonic-root",
                    "/tmp",
                    "--game-camera-yaw-source",
                    source,
                ],
            ):
                parsed = MODULE._parse_args()
                self.assertEqual(parsed.game_camera_yaw_source, source)

    def test_parse_args_exposes_final_pov_state_file(self) -> None:
        with mock.patch.object(
            sys,
            "argv",
            [
                "run_matrix_sonic.py",
                "--model",
                os.fspath(SCRIPT_PATH),
                "--sonic-root",
                "/tmp",
                "--game-camera-yaw-source",
                "ue-final-pov",
                "--game-ue-camera-state-file",
                "/run/user/1000/camera-state.bin",
            ],
        ):
            parsed = MODULE._parse_args()
        self.assertEqual(parsed.game_camera_yaw_source, "ue-final-pov")
        self.assertEqual(
            parsed.game_ue_camera_state_file,
            Path("/run/user/1000/camera-state.bin"),
        )

    def test_qualified_acceptance_rejects_weaker_lock_gates(self) -> None:
        lock = json.loads(
            (REPO_ROOT / "config/runtime/matrix-sonic.lock.json").read_text(
                encoding="utf-8"
            )
        )["acceptance"]
        base = {
            "qualified_runtime": True,
            "min_active_seconds": lock["active_lowcmd_seconds_min"],
            "min_displacement_m": lock["root_displacement_xy_min_m"],
            "min_physics_hz": lock["physics_hz_min"],
            "min_rtf": lock["rtf_min"],
            "low_cmd_fresh_timeout_seconds": lock[
                "low_cmd_fresh_timeout_seconds"
            ],
            "max_resets": lock["instability_resets_max"],
            "fail_on_fall": True,
        }
        MODULE._validate_qualified_acceptance(SimpleNamespace(**base))

        weaker_values = {
            "min_active_seconds": 0.0,
            "min_displacement_m": 0.0,
            "min_physics_hz": 0.0,
            "min_rtf": 0.0,
            "low_cmd_fresh_timeout_seconds": 1.0,
            "max_resets": lock["instability_resets_max"] + 1,
            "fail_on_fall": False,
        }
        for argument, weaker in weaker_values.items():
            values = dict(base)
            values[argument] = weaker
            with self.subTest(argument=argument), self.assertRaisesRegex(
                SystemExit, argument
            ):
                MODULE._validate_qualified_acceptance(SimpleNamespace(**values))

    def test_qualified_game_rejects_provider_bypass_and_fixed_camera(self) -> None:
        valid = SimpleNamespace(
            qualified_runtime=True,
            control_source="game",
            no_game_input_provider=False,
            game_camera_yaw_source="x11-mirror",
            game_look_button="right",
            game_focus_title="matrix",
            ue_pid=4242,
            game_mouse_sensitivity_deg=0.12,
            game_input_provider=REPO_ROOT / "scripts/matrix_game_control_input.py",
            game_input_provider_python=sys.executable,
        )
        MODULE._validate_qualified_game_control(valid)

        bypass = SimpleNamespace(**vars(valid))
        bypass.no_game_input_provider = True
        with self.assertRaisesRegex(SystemExit, "supervised input provider"):
            MODULE._validate_qualified_game_control(bypass)

        fixed = SimpleNamespace(**vars(valid))
        fixed.game_camera_yaw_source = "fixed"
        with self.assertRaisesRegex(SystemExit, "fixed camera yaw"):
            MODULE._validate_qualified_game_control(fixed)

        fixed.qualified_runtime = False
        MODULE._validate_qualified_game_control(fixed)

        wrong_script = SimpleNamespace(**vars(valid))
        wrong_script.game_input_provider = REPO_ROOT / "scripts/run_matrix_sonic.py"
        with self.assertRaisesRegex(SystemExit, "bundled input provider"):
            MODULE._validate_qualified_game_control(wrong_script)

        wrong_python = SimpleNamespace(**vars(valid))
        wrong_python.game_input_provider_python = "/tmp/unverified-python"
        with self.assertRaisesRegex(SystemExit, "verified runtime Python"):
            MODULE._validate_qualified_game_control(wrong_python)

        zero_sensitivity = SimpleNamespace(**vars(valid))
        zero_sensitivity.game_mouse_sensitivity_deg = 0.0
        with self.assertRaisesRegex(SystemExit, "positive mouse sensitivity"):
            MODULE._validate_qualified_game_control(zero_sensitivity)
        for source in ("x11-core-gated", "x11-absolute", "ue-final-pov"):
            with self.subTest(source=source):
                candidate = SimpleNamespace(**vars(valid))
                candidate.game_camera_yaw_source = source
                with self.assertRaisesRegex(
                    SystemExit, "experimental camera yaw sources"
                ):
                    MODULE._validate_qualified_game_control(candidate)

    def test_game_control_status_records_camera_claim_and_calibration(self) -> None:
        args = SimpleNamespace(
            game_input_source="auto",
            game_max_speed=0.30,
            game_max_acceleration=1.20,
            game_max_deceleration=2.40,
            game_max_turn_rate=2.50,
            game_stick_deadzone=0.15,
            game_input_timeout=0.15,
            game_max_snapshot_age=0.15,
            game_max_future_skew=0.05,
            game_camera_yaw_source="x11-mirror",
            game_look_button="right",
            game_focus_title="matrix",
            ue_pid=4242,
            game_camera_yaw_sign=-1,
            game_camera_yaw_offset_deg=90.0,
            game_initial_camera_yaw_deg=5.0,
            game_mouse_sensitivity_deg=0.12,
            game_applied_mouse_profile="remote",
            game_applied_mouse_speed_scale=0.01,
            game_carla_host="127.0.0.2",
            game_carla_port=2100,
            gamepad_look_yaw_rate_deg_s=140.0,
            gamepad_look_pitch_rate_deg_s=95.0,
            gamepad_look_deadzone=0.13,
            gamepad_look_min_pitch_deg=-70.0,
            gamepad_look_max_pitch_deg=50.0,
        )

        status = MODULE._game_control_status_fields(args)

        self.assertEqual(status["input_source_requested"], "auto")
        self.assertEqual(status["input_protocol"], "matrix-game-input/v2")
        self.assertEqual(status["input_source_effective"], "keyboard")
        self.assertEqual(status["camera_yaw_source"], "x11-mirror")
        self.assertEqual(status["camera_look_button"], "right")
        self.assertEqual(status["expected_ue_pid"], 4242)
        self.assertEqual(
            status["camera_yaw_observation"],
            "xinput2_raw_motion_mirror",
        )
        self.assertEqual(
            status["camera_yaw_truth_scope"],
            "xi2_raw_input_mirror_not_final_view",
        )
        self.assertEqual(
            status["native_gait"],
            "IDLE/SLOW_WALK/WALK/RUN selected by movement tier",
        )
        self.assertEqual(
            status["native_gait_modes"],
            {"IDLE": 0, "SLOW_WALK": 1, "WALK": 2, "RUN": 3},
        )
        self.assertEqual(status["keyboard_slow_speed_mps"], 0.10)
        self.assertEqual(status["keyboard_walk_speed_mps"], 0.80)
        self.assertEqual(status["keyboard_run_speed_mps"], 2.50)
        self.assertEqual(status["maximum_speed_mps"], 0.30)
        self.assertEqual(status["analog_maximum_speed_mps"], 0.30)
        self.assertEqual(status["keyboard_maximum_target_speed_mps"], 2.50)
        self.assertEqual(status["maximum_acceleration_mps2"], 1.20)
        self.assertEqual(status["maximum_deceleration_mps2"], 2.40)
        self.assertEqual(status["maximum_turn_rate_rad_s"], 2.50)
        self.assertEqual(status["stick_deadzone"], 0.15)
        self.assertEqual(status["input_timeout_s"], 0.15)
        self.assertEqual(status["camera_yaw_sign"], -1)
        self.assertEqual(status["camera_yaw_offset_deg"], 90.0)
        self.assertEqual(status["mouse_sensitivity_deg_per_px"], 0.12)
        self.assertEqual(status["visible_mouse_backend"], "sdl-relative-speed-scale")
        self.assertEqual(status["applied_mouse_profile"], "remote")
        self.assertEqual(status["applied_mouse_speed_scale"], 0.01)
        self.assertEqual(status["mouse_sensitivity_base_deg_per_px"], 0.12)
        self.assertEqual(status["mouse_sensitivity_effective_deg_per_px"], 0.0012)
        self.assertEqual(
            status["mouse_sensitivity_units"],
            "degrees_per_xi2_raw_unit",
        )
        self.assertEqual(
            status["mouse_sensitivity_base_deg_per_raw_unit"], 0.12
        )
        self.assertEqual(
            status["mouse_sensitivity_effective_deg_per_raw_unit"], 0.0012
        )
        self.assertEqual(status["carla_host"], "127.0.0.2")
        self.assertEqual(status["carla_port"], 2100)
        self.assertFalse(status["visible_follow_camera_verified"])
        self.assertTrue(status["external_visual_evidence_required"])
        self.assertEqual(
            status["qualification_scope"],
            "runtime_input_and_motion_path_only",
        )

        core_args = SimpleNamespace(**vars(args))
        core_args.game_camera_yaw_source = "x11-core-gated"
        core_status = MODULE._game_control_status_fields(core_args)
        self.assertEqual(
            core_status["camera_yaw_observation"],
            "xinput2_raw_motion_core_button_level_gate",
        )
        self.assertEqual(
            core_status["camera_yaw_truth_scope"],
            "xi2_raw_motion_core_button_gate_not_final_view",
        )
        self.assertEqual(
            core_status["button_gate_truth_scope"],
            "xquerypointer_core_button_level_sampled_not_event_ordered",
        )
        self.assertTrue(core_status["experimental"])
        self.assertFalse(core_status["legacy"])
        self.assertEqual(
            core_status["mouse_sensitivity_units"],
            "degrees_per_xi2_raw_unit",
        )

        absolute_args = SimpleNamespace(**vars(args))
        absolute_args.game_camera_yaw_source = "x11-absolute"
        absolute_status = MODULE._game_control_status_fields(absolute_args)
        self.assertEqual(
            absolute_status["camera_yaw_observation"],
            "xquerypointer_root_absolute_delta",
        )
        self.assertEqual(
            absolute_status["camera_yaw_truth_scope"],
            "x11_absolute_pointer_delta_mirror_not_final_view",
        )
        self.assertEqual(
            absolute_status["button_gate_truth_scope"],
            "xquerypointer_core_level_sampled_at_50hz",
        )
        self.assertTrue(absolute_status["experimental"])
        self.assertTrue(absolute_status["legacy"])
        self.assertEqual(
            absolute_status["mouse_sensitivity_units"],
            "degrees_per_x11_root_pixel",
        )
        self.assertEqual(
            absolute_status["mouse_sensitivity_effective_deg_per_unit"],
            0.0012,
        )

        final_pov_args = SimpleNamespace(**vars(args))
        final_pov_args.game_camera_yaw_source = "ue-final-pov"
        final_pov_args.game_ue_camera_state_file = Path(
            "/run/user/1000/camera-state.bin"
        )
        final_pov_status = MODULE._game_control_status_fields(final_pov_args)
        self.assertEqual(final_pov_status["input_source_effective"], "auto")
        self.assertEqual(
            final_pov_status["camera_yaw_observation"],
            "ue_player_camera_manager_final_pov_state",
        )
        self.assertEqual(
            final_pov_status["camera_yaw_truth_scope"],
            "player_camera_manager_final_pov",
        )
        self.assertEqual(
            final_pov_status["button_gate_truth_scope"],
            "xquerypointer_core_level_or_xi2_raw_button_edges",
        )
        self.assertTrue(final_pov_status["experimental"])
        self.assertFalse(final_pov_status["visible_follow_camera_verified"])
        self.assertEqual(
            final_pov_status["ue_camera_state_file"],
            "/run/user/1000/camera-state.bin",
        )

        applied_offset = MODULE._effective_game_camera_yaw_offset_deg(
            source="ue-final-pov",
            configured_offset_deg=90.0,
            initial_root_yaw_rad=math.pi / 2.0,
        )
        self.assertAlmostEqual(applied_offset, 0.0)
        compensated_status = MODULE._game_control_status_fields(
            final_pov_args,
            applied_camera_yaw_offset_deg=applied_offset,
            initial_root_yaw_rad=math.pi / 2.0,
        )
        self.assertAlmostEqual(compensated_status["camera_yaw_offset_deg"], 0.0)
        self.assertEqual(
            compensated_status["camera_yaw_offset_configured_deg"], 90.0
        )
        self.assertAlmostEqual(
            compensated_status["camera_yaw_initial_root_compensation_deg"],
            -90.0,
        )
        self.assertAlmostEqual(
            compensated_status["initial_root_yaw_rad_for_camera"],
            math.pi / 2.0,
        )
        self.assertEqual(
            MODULE._effective_game_camera_yaw_offset_deg(
                source="x11-mirror",
                configured_offset_deg=90.0,
                initial_root_yaw_rad=math.pi / 2.0,
            ),
            90.0,
        )

    def test_acceptance_rejects_fall_and_short_lowcmd(self) -> None:
        failures = MODULE._acceptance_failures(
            unstable=False,
            fall_detected=True,
            fail_on_fall=True,
            active_lowcmd=True,
            active_elapsed_s=12.0,
            min_active_seconds=30.0,
            physics_step_hz=200.0,
            min_physics_hz=195.0,
            rtf=1.0,
            min_rtf=0.95,
        )
        self.assertEqual(failures[0], "fall_detected")
        self.assertTrue(failures[1].startswith("active_lowcmd_too_short:"))

    def test_acceptance_allows_interactive_run_without_minimum(self) -> None:
        self.assertEqual(
            MODULE._acceptance_failures(
                unstable=False,
                fall_detected=False,
                fail_on_fall=True,
                active_lowcmd=False,
                active_elapsed_s=0.0,
                min_active_seconds=0.0,
                physics_step_hz=0.0,
                min_physics_hz=0.0,
                rtf=0.0,
                min_rtf=0.0,
            ),
            [],
        )

    def test_qualified_game_acceptance_requires_clean_exercised_input(self) -> None:
        self.assertEqual(
            MODULE._game_input_acceptance_failures(
                accepted_connections=1,
                packets_applied=10,
                moving_command_frames=5,
                protocol_errors=0,
                rejected_packets=0,
                peer_pid_mismatches=0,
                connected_at_boundary=True,
                input_age_s=0.05,
                maximum_boundary_age_s=0.17,
                safe_stop_at_boundary=False,
            ),
            [],
        )
        self.assertEqual(
            MODULE._game_input_acceptance_failures(
                accepted_connections=0,
                packets_applied=0,
                moving_command_frames=0,
                protocol_errors=2,
                rejected_packets=3,
                peer_pid_mismatches=4,
                connected_at_boundary=False,
                input_age_s=None,
                maximum_boundary_age_s=0.17,
                safe_stop_at_boundary=True,
            ),
            [
                "game_input_no_connection",
                "game_input_no_applied_packets",
                "game_input_no_moving_command_frames",
                "game_input_protocol_errors:2",
                "game_input_rejected_packets:3",
                "game_input_peer_pid_mismatches:4",
                "game_input_disconnected_at_boundary",
                "game_input_stale_at_boundary",
                "game_input_safe_stop_at_boundary",
            ],
        )

    def test_acceptance_rejects_stale_lowcmd_and_slow_physics(self) -> None:
        failures = MODULE._acceptance_failures(
            unstable=False,
            fall_detected=False,
            fail_on_fall=True,
            active_lowcmd=False,
            active_elapsed_s=45.0,
            min_active_seconds=30.0,
            physics_step_hz=190.0,
            min_physics_hz=195.0,
            rtf=0.90,
            min_rtf=0.95,
        )
        self.assertEqual(
            failures,
            [
                "lowcmd_not_fresh_at_exit",
                "physics_hz_too_low:190.000<195.000",
                "rtf_too_low:0.9000<0.9500",
            ],
        )

    def test_child_exit_is_not_misclassified_as_numerical_instability(self) -> None:
        failures = MODULE._acceptance_failures(
            unstable=False,
            fall_detected=False,
            fail_on_fall=False,
            active_lowcmd=False,
            active_elapsed_s=0.0,
            min_active_seconds=0.0,
            physics_step_hz=200.0,
            min_physics_hz=0.0,
            rtf=1.0,
            min_rtf=0.0,
            failed_child=("deploy", 17),
        )
        self.assertEqual(failures, ["native_child_exit:deploy:17"])
        self.assertNotIn("numerical_instability", failures)

    def test_fake_ue_exit_42_invalidates_otherwise_passing_status(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fake_ue = root / "fake-ue"
            fake_ue.write_text("#!/usr/bin/env bash\nexit 42\n", encoding="utf-8")
            fake_ue.chmod(0o755)
            ue_result = subprocess.run([str(fake_ue)], check=False)
            self.assertEqual(ue_result.returncode, 42)

            failure_file = root / "failure.json"
            failure_file.write_text(
                json.dumps({"name": "ue", "exit_code": ue_result.returncode}),
                encoding="utf-8",
            )
            failure = MODULE._read_external_failure(failure_file)
            self.assertEqual(failure, ("ue", 42))

            status = root / "status.json"
            status.write_text(
                json.dumps(
                    {
                        "acceptance_failures": [],
                        "completed": True,
                        "passed": True,
                        "termination_reason": "max_seconds",
                    }
                ),
                encoding="utf-8",
            )
            assert failure is not None
            MODULE._record_external_child_failure(status, failure)
            payload = json.loads(status.read_text(encoding="utf-8"))

            self.assertFalse(payload["passed"])
            self.assertFalse(payload["completed"])
            self.assertEqual(payload["failed_child_name"], "ue")
            self.assertEqual(payload["failed_child_exit_code"], 42)
            self.assertEqual(payload["termination_reason"], "child_exit")
            self.assertIn("native_child_exit:ue:42", payload["acceptance_failures"])

    def test_late_ue_failure_creates_missing_final_status(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            status = Path(temporary) / "status.json"

            MODULE._record_external_child_failure(status, ("ue", 23))

            payload = json.loads(status.read_text(encoding="utf-8"))
            self.assertFalse(payload["passed"])
            self.assertFalse(payload["completed"])
            self.assertEqual(payload["failed_child_name"], "ue")
            self.assertEqual(payload["failed_child_exit_code"], 23)
            self.assertEqual(payload["termination_reason"], "child_exit")
            self.assertEqual(
                payload["acceptance_failures"], ["native_child_exit:ue:23"]
            )

    def test_normal_ue_lifecycle_without_failure_record_is_not_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            failure_file = Path(temporary) / "failure.json"
            self.assertIsNone(MODULE._read_external_failure(failure_file))

        run_sim = (REPO_ROOT / "scripts/run_sim.sh").read_text(encoding="utf-8")
        supervisor = (REPO_ROOT / "scripts/supervise_matrix_ue.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("coproc MATRIX_UE_SUPERVISOR", run_sim)
        self.assertIn('wait "$UE_SUPERVISOR_PID"', run_sim)
        self.assertNotIn('kill -0 "$ue_pid"', run_sim)
        self.assertNotIn('PIDS+=("$UE_PID")', run_sim)
        self.assertNotIn("UE_EXPECTED_STOP_FILE", run_sim)
        self.assertIn('--camera-state-file "$UE_CAMERA_STATE_FILE"', run_sim)
        self.assertIn('--camera-layout "$MATRIX_UE_CAMERA_LAYOUT"', run_sim)
        self.assertIn('--game-ue-camera-state-file "$UE_CAMERA_STATE_FILE"', run_sim)
        self.assertNotIn("--camera-executable", run_sim)
        self.assertIn("os.WNOWAIT", supervisor)
        self.assertIn("start_new_session=True", supervisor)
        self.assertIn("signal.SIGKILL", supervisor)

    def test_ue_supervisor_classifies_unexpected_and_expected_exit(self) -> None:
        supervisor = REPO_ROOT / "scripts/supervise_matrix_ue.py"
        cases = (
            ("unexpected", ["/bin/sh", "-c", "exit 42"], 42, 42),
            ("expected", ["/bin/sh", "-c", "while :; do sleep 1; done"], 0, None),
        )
        for name, command, expected_code, expected_failure in cases:
            with self.subTest(case=name), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                pid_file = root / "ue.pid"
                failure_file = root / "failure.json"
                process = subprocess.Popen(
                    [
                        sys.executable,
                        str(supervisor),
                        "--pid-file",
                        str(pid_file),
                        "--failure-file",
                        str(failure_file),
                        "--log",
                        str(root / "ue.log"),
                        "--expected-parent-pid",
                        str(os.getpid()),
                        "--",
                        *command,
                    ],
                    stdin=subprocess.PIPE,
                )
                try:
                    deadline = time.monotonic() + 3.0
                    while not pid_file.exists() and time.monotonic() < deadline:
                        time.sleep(0.01)
                    self.assertTrue(pid_file.exists(), "supervisor did not publish UE PID")
                    if name == "unexpected":
                        deadline = time.monotonic() + 3.0
                        while not failure_file.exists() and time.monotonic() < deadline:
                            time.sleep(0.01)
                    assert process.stdin is not None
                    process.stdin.write(b"stop\n")
                    process.stdin.flush()
                    process.stdin.close()
                    self.assertEqual(process.wait(timeout=6.0), expected_code)
                    failure = (
                        json.loads(failure_file.read_text(encoding="utf-8"))
                        if failure_file.exists()
                        else None
                    )
                    if expected_failure is None:
                        self.assertIsNone(failure)
                    else:
                        self.assertEqual(
                            failure, {"name": "ue", "exit_code": expected_failure}
                        )
                finally:
                    if process.poll() is None:
                        process.kill()
                        process.wait(timeout=3.0)

    def test_malformed_ue_failure_never_falls_back_to_exit_zero(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            failure_file = Path(temporary) / "failure.json"
            failure_file.write_text('{"name":"ue"}\n', encoding="utf-8")
            self.assertEqual(
                MODULE._read_external_failure(failure_file),
                ("ue", MODULE._UNKNOWN_EXTERNAL_EXIT_CODE),
            )
            self.assertNotEqual(MODULE._UNKNOWN_EXTERNAL_EXIT_CODE, 0)

    def test_acceptance_enforces_optional_displacement_gate(self) -> None:
        failures = MODULE._acceptance_failures(
            unstable=False,
            fall_detected=False,
            fail_on_fall=False,
            active_lowcmd=False,
            active_elapsed_s=0.0,
            min_active_seconds=0.0,
            physics_step_hz=200.0,
            min_physics_hz=0.0,
            rtf=1.0,
            min_rtf=0.0,
            root_displacement_xy_m=0.49,
            min_displacement_m=0.5,
        )
        self.assertEqual(failures, ["root_displacement_too_small:0.490<0.500"])

    def test_acceptance_enforces_directional_final_x_gate(self) -> None:
        failures = MODULE._acceptance_failures(
            unstable=False,
            fall_detected=False,
            fail_on_fall=False,
            active_lowcmd=True,
            active_elapsed_s=30.0,
            min_active_seconds=30.0,
            physics_step_hz=200.0,
            min_physics_hz=195.0,
            rtf=1.0,
            min_rtf=0.95,
            root_displacement_xy_m=10.0,
            min_displacement_m=0.5,
            root_final_x=114.0,
            min_final_x=128.0,
        )
        self.assertEqual(failures, ["final_x_too_small:114.000<128.000"])

    def test_acceptance_rejects_lateral_distance_without_forward_progress(self) -> None:
        failures = MODULE._acceptance_failures(
            unstable=False,
            fall_detected=False,
            fail_on_fall=False,
            active_lowcmd=True,
            active_elapsed_s=30.0,
            min_active_seconds=30.0,
            physics_step_hz=200.0,
            min_physics_hz=195.0,
            rtf=1.0,
            min_rtf=0.95,
            root_displacement_xy_m=5.0,
            min_displacement_m=0.5,
            root_final_x=128.1,
            min_final_x=128.0,
            root_displacement_x_m=0.1,
            min_forward_x_m=4.0,
        )
        self.assertEqual(failures, ["forward_x_too_small:0.100<4.000"])

    def test_acceptance_rejects_authoritative_reset_count(self) -> None:
        failures = MODULE._acceptance_failures(
            unstable=False,
            fall_detected=False,
            fail_on_fall=True,
            active_lowcmd=True,
            active_elapsed_s=30.0,
            min_active_seconds=30.0,
            physics_step_hz=200.0,
            min_physics_hz=195.0,
            rtf=1.0,
            min_rtf=0.95,
            reset_count=1,
            max_resets=0,
        )
        self.assertEqual(failures, ["reset_count_exceeded:1>0"])

    def test_snapshot_requires_exact_native_dimensions(self) -> None:
        self.assertIsNone(MODULE._snapshot_validation_error(self.snapshot()))
        for field, kwargs, expected in (
            ("qpos", {"qpos_len": 37}, "qpos=37,expected=36"),
            ("qvel", {"qvel_len": 34}, "qvel=34,expected=35"),
            ("ctrl", {"ctrl_len": 30}, "ctrl=30,expected=29"),
            (
                "applied_torque",
                {"torque_len": 28},
                "applied_torque=28,expected=29",
            ),
        ):
            with self.subTest(field=field):
                error = MODULE._snapshot_validation_error(self.snapshot(**kwargs))
                self.assertEqual(error, f"snapshot_dimension:{expected}")

    def test_snapshot_accepts_explicit_inventory_dimensions(self) -> None:
        dimensions = {
            "qpos": 92,
            "qvel": 83,
            "ctrl": 29,
            "applied_torque": 29,
        }
        snapshot = self.snapshot(qpos_len=92, qvel_len=83)
        self.assertIsNone(
            MODULE._snapshot_validation_error(snapshot, expected_dims=dimensions)
        )
        invalid = dict(dimensions)
        invalid["unexpected"] = 1
        self.assertEqual(
            MODULE._snapshot_validation_error(snapshot, expected_dims=invalid),
            "snapshot_expected_dimensions_invalid",
        )

    def test_creative_inventory_extends_only_unactuated_freejoints(self) -> None:
        body_joints = [f"body_joint_{index}" for index in range(29)]
        actuators = [f"body_actuator_{index}" for index in range(29)]
        inventory_joint = "creative_item__prop__0__freejoint"
        joint_names = ["floating_base_joint", *body_joints, inventory_joint]

        class ActuatorTransmission:
            def __getitem__(self, key):
                index, column = key
                self.assert_column(column)
                return index + 1

            @staticmethod
            def assert_column(column):
                if column != 0:
                    raise AssertionError(column)

        model = SimpleNamespace(
            nq=43,
            nv=41,
            nu=29,
            njnt=len(joint_names),
            jnt_type=[0, *([3] * 29), 0],
            actuator_trnid=ActuatorTransmission(),
            joint=lambda index: SimpleNamespace(name=joint_names[index]),
            actuator=lambda index: SimpleNamespace(name=actuators[index]),
        )
        config = {
            "BODY_JOINT_NAMES": body_joints,
            "BODY_ACTUATOR_NAMES": actuators,
        }
        MODULE._validate_creative_inventory_sonic_contract(
            model,
            config,
            (inventory_joint,),
        )

        model.jnt_type[-1] = 3
        with self.assertRaisesRegex(ValueError, "must all be freejoints"):
            MODULE._validate_creative_inventory_sonic_contract(
                model,
                config,
                (inventory_joint,),
            )

    def test_snapshot_step_must_advance_once_and_time_must_increase(self) -> None:
        previous = self.snapshot(step_index=10, sim_time=1.0)
        self.assertIsNone(
            MODULE._snapshot_validation_error(
                self.snapshot(step_index=11, sim_time=1.005), previous
            )
        )
        self.assertEqual(
            MODULE._snapshot_validation_error(
                self.snapshot(step_index=12, sim_time=1.005), previous
            ),
            "snapshot_step_index_not_sequential:12,expected=11",
        )
        self.assertEqual(
            MODULE._snapshot_validation_error(
                self.snapshot(step_index=11, sim_time=1.0), previous
            ),
            "snapshot_sim_time_not_increasing:1.0,previous=1.0",
        )

    def test_snapshot_rejects_non_finite_physics_vectors(self) -> None:
        for field in ("qpos", "qvel", "ctrl", "applied_torque"):
            with self.subTest(field=field):
                snapshot = self.snapshot()
                getattr(snapshot, field)[2] = math.nan
                self.assertEqual(
                    MODULE._snapshot_validation_error(snapshot),
                    f"snapshot_non_finite:{field}[2]=nan",
                )

    def test_snapshot_validates_authoritative_fall_reset_fields(self) -> None:
        self.assertIsNone(
            MODULE._snapshot_validation_error(
                self.snapshot(
                    fall_detected=True,
                    reset_count=1,
                    last_reset_reason="fall",
                )
            )
        )
        invalid_fall = self.snapshot()
        invalid_fall.fall_detected = 1
        self.assertEqual(
            MODULE._snapshot_validation_error(invalid_fall),
            "snapshot_invalid_fall_detected:1",
        )
        self.assertEqual(
            MODULE._snapshot_validation_error(self.snapshot(reset_count=-1)),
            "snapshot_invalid_reset_count:-1",
        )
        self.assertEqual(
            MODULE._snapshot_validation_error(
                self.snapshot(step_index=1, sim_time=0.005, reset_count=1),
                self.snapshot(step_index=0, sim_time=0.0, reset_count=2),
            ),
            "snapshot_reset_count_decreased:1,previous=2",
        )

    def test_snapshot_validates_lowcmd_and_startup_band_fields(self) -> None:
        invalid_received = self.snapshot()
        invalid_received.low_cmd_received = 1
        self.assertEqual(
            MODULE._snapshot_validation_error(invalid_received),
            "snapshot_invalid_low_cmd_received:1",
        )
        self.assertEqual(
            MODULE._snapshot_validation_error(self.snapshot(low_cmd_age_s=-0.1)),
            "snapshot_invalid_low_cmd_age_s:-0.1",
        )
        self.assertEqual(
            MODULE._snapshot_validation_error(
                self.snapshot(elastic_band_scale=math.nan)
            ),
            "snapshot_invalid_elastic_band_scale:nan",
        )

    def test_only_normal_bounded_completion_can_pass(self) -> None:
        completed = MODULE._qualification_state(
            max_seconds=120.0,
            termination_reason="max_seconds",
            failures=[],
            runtime_verified=True,
        )
        self.assertTrue(completed["passed"])
        self.assertTrue(completed["completed"])

        scenario_complete = MODULE._qualification_state(
            max_seconds=360.0,
            termination_reason="scenario_complete",
            failures=[],
            runtime_verified=False,
        )
        self.assertTrue(scenario_complete["passed"])
        self.assertTrue(scenario_complete["completed"])
        self.assertFalse(scenario_complete["qualification_attempted"])
        self.assertFalse(scenario_complete["interrupted"])
        self.assertEqual(scenario_complete["acceptance_failures"], [])

        bounded_signal = MODULE._qualification_state(
            max_seconds=120.0,
            termination_reason="signal",
            failures=[],
            runtime_verified=True,
        )
        self.assertFalse(bounded_signal["passed"])
        self.assertIn("run_interrupted", bounded_signal["acceptance_failures"])

        interactive_signal = MODULE._qualification_state(
            max_seconds=0.0,
            termination_reason="signal",
            failures=[],
            runtime_verified=False,
        )
        self.assertFalse(interactive_signal["passed"])
        self.assertTrue(interactive_signal["interrupted"])
        self.assertEqual(interactive_signal["acceptance_failures"], [])

        unverified = MODULE._qualification_state(
            max_seconds=30.0,
            termination_reason="max_seconds",
            failures=[],
            runtime_verified=False,
        )
        self.assertFalse(unverified["passed"])
        self.assertIn(
            "runtime_not_verified_for_qualification",
            unverified["acceptance_failures"],
        )

    def test_qualified_runtime_consumes_matching_verifier_receipt(self) -> None:
        lock = REPO_ROOT / "config/runtime/matrix-sonic.lock.json"
        matrix_commit = subprocess.run(
            ["git", "-C", str(REPO_ROOT), "rev-parse", "HEAD"],
            text=True,
            capture_output=True,
            check=True,
        ).stdout.strip()
        with tempfile.TemporaryDirectory() as temporary:
            receipt_path = Path(temporary) / "receipt.json"
            active_lock = json.loads(lock.read_text(encoding="utf-8"))
            required_checks = [
                "Matrix source commit",
                "Matrix tracked source clean",
                "Matrix ignored source overlays absent",
                "native runtime Python",
                "native runtime Python prefix",
                "native runtime Python isolation",
                "native SONIC source clean",
                "native SONIC ignored source overlays absent",
                "native SONIC Git checkout required",
                "native SONIC commit",
                "native SONIC Python API",
                "gear_sonic import origin",
                "SONIC deploy dependency closure",
                "Matrix UE dependency closure",
                "TensorRT ABI",
            ]
            payload = {
                "passed": True,
                "checks": [
                    {"name": name, "ok": True} for name in required_checks
                ],
                "profile": "trna",
                "lock": str(lock),
                "lock_sha256": MODULE._sha256_file(lock),
                "matrix_root": str(REPO_ROOT),
                "matrix_commit": matrix_commit,
                "sonic_root": "/sonic",
                "runtime_root": "/runtime",
                "python": str((REPO_ROOT / ".venv-audit/bin/python").absolute()),
                "python_prefix": str((REPO_ROOT / ".venv-audit").absolute()),
                "pico_python": None,
                "pico_wheel": None,
                "full_hashes": True,
                "sonic_git_checkout": True,
                "qualification_eligible": True,
                "verification_flags": {
                    "fast": False,
                    "skip_dynamic": False,
                    "skip_installed_assets": False,
                    "require_git_sonic": True,
                },
                "verification_inventory": {
                    "runtime_files_expected": len(active_lock["runtime_files"]),
                    "runtime_files_checked": len(active_lock["runtime_files"]),
                    "runtime_trees_expected": len(active_lock["runtime_trees"]),
                    "runtime_trees_checked": len(active_lock["runtime_trees"]),
                    "installed_files_expected": len(
                        active_lock["matrix_release"]["installed_files"]
                    ),
                    "installed_files_checked": len(
                        active_lock["matrix_release"]["installed_files"]
                    ),
                    "installed_trees_expected": len(
                        active_lock["matrix_release"]["installed_trees"]
                    ),
                    "installed_trees_checked": len(
                        active_lock["matrix_release"]["installed_trees"]
                    ),
                    "dynamic_checks_performed": True,
                },
                "qualification_required_checks": required_checks,
                "missing_qualification_checks": [],
                "launch_roots": MODULE._expected_receipt_roots(
                    "trna", Path("/runtime"), Path("/sonic")
                ),
                "launch_environment": {
                    "ld_library_path": os.environ.get("LD_LIBRARY_PATH", ""),
                    "pythonpath": os.environ.get("PYTHONPATH", ""),
                    "tensorrt_root": os.environ.get("TensorRT_ROOT", ""),
                    "python_pycache_prefix": os.environ.get(
                        "PYTHONPYCACHEPREFIX", ""
                    ),
                    "python_dont_write_bytecode": os.environ.get(
                        "PYTHONDONTWRITEBYTECODE", ""
                    ),
                },
            }
            receipt_path.write_text(json.dumps(payload), encoding="utf-8")
            args = SimpleNamespace(
                qualified_runtime=True,
                runtime_lock_sha256=payload["lock_sha256"],
                matrix_commit=matrix_commit,
                verification_receipt=receipt_path,
                qualification_profile="trna",
                sonic_root=Path("/sonic"),
                control_source="planner",
                pico_python=None,
            )
            def validate_receipt():
                with (
                    mock.patch.object(
                        MODULE.sys,
                        "executable",
                        str(REPO_ROOT / ".venv-audit/bin/python"),
                    ),
                    mock.patch.object(
                        MODULE.sys, "prefix", str(REPO_ROOT / ".venv-audit")
                    ),
                ):
                    return MODULE._validate_qualification_receipt(args)

            self.assertEqual(validate_receipt(), payload)
            self.assertEqual(args.verification_receipt, receipt_path.resolve())
            self.assertEqual(
                args.verification_receipt_sha256,
                MODULE._sha256_file(receipt_path),
            )

            payload["full_hashes"] = False
            receipt_path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(SystemExit, "receipt does not match"):
                validate_receipt()

            payload["full_hashes"] = True
            payload["verification_flags"]["skip_dynamic"] = True
            receipt_path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(SystemExit, "receipt does not match"):
                validate_receipt()

            payload["verification_flags"]["skip_dynamic"] = False
            payload["passed"] = False
            receipt_path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(SystemExit, "receipt does not match"):
                validate_receipt()

    def test_qualified_model_rejects_receipt_model_root_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            model = root / "scene.xml"
            model.write_text("<mujoco/>\n", encoding="utf-8")
            (root / "manifest.json").write_text(
                json.dumps(
                    {
                        "canonical_model": "/different/model.xml",
                        "canonical_meshes": "/different/meshes",
                    }
                ),
                encoding="utf-8",
            )
            args = SimpleNamespace(
                qualified_runtime=True,
                sonic_root=Path("/sonic"),
                scenario_layout_sha256=None,
            )
            receipt = {
                "launch_roots": MODULE._expected_receipt_roots(
                    "trna", Path("/runtime"), Path("/sonic")
                )
            }
            with self.assertRaisesRegex(SystemExit, "canonical path"):
                MODULE._validate_qualified_model(args, model, receipt)

    def test_native_config_uses_matrix_model_and_waits_for_lowcmd(self) -> None:
        args = SimpleNamespace(
            dds_interface="lo",
            physics_hz=200.0,
            startup_band=True,
            startup_band_hold=4.0,
            startup_band_fade=3.0,
            low_cmd_fresh_timeout_seconds=0.1,
        )
        kwargs = MODULE._native_config_kwargs(args, Path("/tmp/matrix.xml"))
        self.assertEqual(kwargs["robot_scene"], "/tmp/matrix.xml")
        self.assertEqual(kwargs["interface"], "lo")
        self.assertEqual(kwargs["sim_frequency"], 200)
        self.assertTrue(kwargs["elastic_band_release_enabled"])
        self.assertTrue(kwargs["elastic_band_wait_for_lowcmd"])
        self.assertEqual(kwargs["elastic_band_hold_seconds"], 4.0)
        self.assertEqual(kwargs["elastic_band_fade_seconds"], 3.0)
        self.assertEqual(kwargs["low_cmd_fresh_timeout_seconds"], 0.1)
        self.assertFalse(kwargs["with_hands"])
        self.assertFalse(kwargs["reset_on_fall"])

    def test_disabling_startup_band_requests_immediate_release(self) -> None:
        args = SimpleNamespace(
            dds_interface="lo",
            physics_hz=200.0,
            startup_band=False,
            startup_band_hold=4.0,
            startup_band_fade=3.0,
            low_cmd_fresh_timeout_seconds=0.1,
        )
        kwargs = MODULE._native_config_kwargs(args, Path("/tmp/matrix.xml"))
        self.assertTrue(kwargs["elastic_band_release_enabled"])
        self.assertFalse(kwargs["elastic_band_wait_for_lowcmd"])
        self.assertEqual(kwargs["elastic_band_hold_seconds"], 0.0)
        self.assertEqual(kwargs["elastic_band_fade_seconds"], 0.0)

    def test_native_planner_uses_sonic_wire_builders(self) -> None:
        class FakeSocket:
            def __init__(self) -> None:
                self.bound = None
                self.sent = []
                self.close_calls = 0

            def setsockopt(self, *_args) -> None:
                pass

            def bind(self, endpoint) -> None:
                self.bound = endpoint

            def send(self, payload) -> None:
                self.sent.append(payload)

            def close(self, **_kwargs) -> None:
                self.close_calls += 1

        socket = FakeSocket()

        class FakeContext:
            @classmethod
            def instance(cls):
                return cls()

            def socket(self, _kind):
                return socket

        fake_zmq = SimpleNamespace(Context=FakeContext, PUB=1, LINGER=2)
        commands = []
        planners = []

        def build_command_message(**kwargs):
            commands.append(kwargs)
            return b"command"

        def build_planner_message(**kwargs):
            planners.append(kwargs)
            return b"planner"

        client = MODULE.NativePlannerClient(
            "tcp://127.0.0.1:5556",
            zmq_module=fake_zmq,
            build_command_message=build_command_message,
            build_planner_message=build_planner_message,
        )
        client.send_velocity(
            1.0,
            0.0,
            0.5,
            dt=0.2,
        )

        self.assertEqual(socket.bound, "tcp://127.0.0.1:5556")
        self.assertEqual(socket.sent, [b"command", b"planner"])
        self.assertTrue(commands[0]["start"])
        self.assertEqual(planners[0]["mode"], 2)
        with mock.patch.object(MODULE.time, "sleep"):
            client.request_deploy_stop()
        self.assertEqual(socket.sent[-3:], [b"command"] * 3)
        self.assertEqual(socket.close_calls, 0)
        self.assertFalse(commands[-1]["start"])
        self.assertTrue(commands[-1]["stop"])
        self.assertAlmostEqual(planners[0]["movement"][0], math.cos(0.1))
        self.assertAlmostEqual(planners[0]["movement"][1], math.sin(0.1))
        self.assertAlmostEqual(planners[0]["facing"][0], math.cos(0.1))
        self.assertAlmostEqual(planners[0]["facing"][1], math.sin(0.1))
        self.assertEqual(planners[0]["speed"], 1.0)

        client.send_game_command(
            MODULE.RobotMotionCommand(
                sequence=9,
                movement=(0.0, 1.0, 0.0),
                facing=(0.0, 1.0, 0.0),
                speed_mps=0.3,
                locomotion_mode=MODULE.SONIC_SLOW_WALK_MODE,
                mode="move",
                safe_stop=False,
                reason=None,
            )
        )
        self.assertEqual(planners[1]["mode"], 1)
        self.assertEqual(planners[1]["movement"], [0.0, 1.0, 0.0])
        self.assertEqual(planners[1]["facing"], [0.0, 1.0, 0.0])
        self.assertEqual(planners[1]["speed"], 0.3)

        for sequence, native_mode, speed in (
            (10, MODULE.SONIC_WALK_MODE, 0.8),
            (11, MODULE.SONIC_RUN_MODE, 2.5),
        ):
            client.send_game_command(
                MODULE.RobotMotionCommand(
                    sequence=sequence,
                    movement=(0.0, 1.0, 0.0),
                    facing=(0.0, 1.0, 0.0),
                    speed_mps=speed,
                    locomotion_mode=native_mode,
                    mode="move",
                    safe_stop=False,
                    reason=None,
                )
            )
            self.assertEqual(planners[-1]["mode"], native_mode)
            self.assertEqual(planners[-1]["speed"], speed)

        client.send_game_command(
            MODULE.RobotMotionCommand(
                sequence=12,
                movement=(0.0, 0.0, 0.0),
                facing=(0.0, 1.0, 0.0),
                speed_mps=0.0,
                locomotion_mode=MODULE.SONIC_IDLE_MODE,
                mode="deadman",
                safe_stop=True,
                reason="sonic_not_ready",
            )
        )
        self.assertTrue(commands[4]["start"])
        self.assertFalse(commands[4]["stop"])
        self.assertEqual(planners[4]["mode"], 0)
        self.assertEqual(planners[4]["movement"], [0.0, 0.0, 0.0])
        self.assertEqual(planners[4]["speed"], -1.0)

        client.send_recovery_posture(
            locomotion_mode=5,
            height=0.4,
            facing=(0.0, 1.0, 0.0),
        )
        self.assertEqual(planners[5]["mode"], 5)
        self.assertEqual(planners[5]["movement"], [0.0, 0.0, 0.0])
        self.assertEqual(planners[5]["facing"], [0.0, 1.0, 0.0])
        self.assertEqual(planners[5]["speed"], -1.0)
        self.assertEqual(planners[5]["height"], 0.4)

        client.send_recovery_posture(
            locomotion_mode=0,
            height=-1.0,
            facing=(1.0, 0.0, 0.0),
        )
        self.assertEqual(planners[6]["mode"], 0)
        self.assertEqual(planners[6]["height"], -1.0)
        with self.assertRaisesRegex(ValueError, "IDLE or KNEEL"):
            client.send_recovery_posture(
                locomotion_mode=7,
                height=0.4,
                facing=(1.0, 0.0, 0.0),
            )

        with self.assertRaisesRegex(ValueError, "SLOW_WALK"):
            client.send_game_command(
                MODULE.RobotMotionCommand(
                    sequence=13,
                    movement=(1.0, 0.0, 0.0),
                    facing=(1.0, 0.0, 0.0),
                    speed_mps=0.81,
                    locomotion_mode=MODULE.SONIC_SLOW_WALK_MODE,
                    mode="move",
                    safe_stop=False,
                    reason=None,
                )
            )

        for sequence, native_mode, speed, gait_name in (
            (14, MODULE.SONIC_WALK_MODE, 0.79995, "WALK"),
            (15, MODULE.SONIC_RUN_MODE, 2.49995, "RUN"),
        ):
            with self.subTest(native_mode=native_mode), self.assertRaisesRegex(
                ValueError, gait_name
            ):
                client.send_game_command(
                    MODULE.RobotMotionCommand(
                        sequence=sequence,
                        movement=(1.0, 0.0, 0.0),
                        facing=(1.0, 0.0, 0.0),
                        speed_mps=speed,
                        locomotion_mode=native_mode,
                        mode="move",
                        safe_stop=False,
                        reason=None,
                    )
                )

        with mock.patch.object(MODULE.time, "sleep") as sleep:
            client.close()
        self.assertTrue(commands[-1]["stop"])
        self.assertEqual(socket.sent[-3:], [b"command"] * 3)
        self.assertEqual(sleep.call_count, 3)

    def test_native_planner_yaw_only_remains_idle(self) -> None:
        class FakeSocket:
            def setsockopt(self, *_args) -> None:
                pass

            def bind(self, _endpoint) -> None:
                pass

            def send(self, _payload) -> None:
                pass

            def close(self, **_kwargs) -> None:
                pass

        socket = FakeSocket()

        class FakeContext:
            @classmethod
            def instance(cls):
                return cls()

            def socket(self, _kind):
                return socket

        planners = []
        client = MODULE.NativePlannerClient(
            "tcp://127.0.0.1:5556",
            zmq_module=SimpleNamespace(Context=FakeContext, PUB=1, LINGER=2),
            build_command_message=lambda **_kwargs: b"command",
            build_planner_message=lambda **kwargs: planners.append(kwargs) or b"planner",
        )
        client.send_velocity(0.0, 0.0, 1.0, dt=0.1)
        self.assertEqual(planners[0]["mode"], 0)
        self.assertEqual(planners[0]["movement"], [0.0, 0.0, 0.0])
        self.assertEqual(planners[0]["speed"], -1.0)
        self.assertAlmostEqual(planners[0]["facing"][0], math.cos(0.1))
        client.close()

    @unittest.skipUnless(
        hasattr(socket, "SOCK_SEQPACKET") and hasattr(socket, "SO_PEERCRED"),
        "Linux Unix seqpacket credentials are required",
    )
    def test_game_input_runtime_applies_authenticated_camera_relative_input(self) -> None:
        config = GAME_CONTROL.ControlConfig(
            max_speed_mps=0.3,
            max_acceleration_mps2=100.0,
            max_deceleration_mps2=100.0,
            max_turn_rate_rad_s=100.0,
            max_step_s=1.0,
        )
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "game.sock"
            runtime = MODULE.GameInputRuntime(
                path,
                GAME_CONTROL.GameControlCore(config),
            )
            runtime.open()
            client = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            try:
                client.connect(os.fspath(path))
                neutral = GAME_CONTROL.InputSnapshot.from_mapping(
                    {
                        "protocol": GAME_CONTROL.PROTOCOL_NAME,
                        "sequence": 7,
                        "timestamp_monotonic_s": 10.0,
                        "focused": True,
                        "camera_yaw_rad": math.pi / 2.0,
                        "keys": {
                            "w": False,
                            "a": False,
                            "s": False,
                            "d": False,
                            "q": False,
                            "e": False,
                            "v": False,
                            "ctrl": False,
                            "shift": False,
                        },
                        "move_stick": {"right": 0.0, "forward": 0.0},
                    }
                )
                client.sendall(GAME_CONTROL.encode_input_packet(neutral))
                self.assertEqual(
                    runtime.poll(now_s=10.0, dt_s=0.1).mode,
                    "idle",
                )
                moving = GAME_CONTROL.InputSnapshot.from_mapping(
                    {
                        **neutral.to_mapping(),
                        "sequence": 8,
                        "timestamp_monotonic_s": 10.01,
                        "keys": {**neutral.to_mapping()["keys"], "w": True},
                    }
                )
                client.sendall(GAME_CONTROL.encode_input_packet(moving))
                command = runtime.poll(now_s=10.01, dt_s=0.1)
                self.assertEqual(command.mode, "move")
                self.assertAlmostEqual(command.movement[0], 0.0, places=7)
                self.assertAlmostEqual(command.movement[1], 1.0, places=7)
                self.assertEqual(command.movement, command.facing)
                telemetry = runtime.telemetry(now_s=10.05)
                self.assertTrue(telemetry["connected"])
                self.assertEqual(telemetry["packets_applied"], 2)
                self.assertEqual(telemetry["sequence"], 8)

                # The final duration check may end the main loop with a packet
                # already queued.  A zero-dt boundary poll must still observe
                # focus loss before acceptance telemetry is captured.
                focus_lost = GAME_CONTROL.InputSnapshot.from_mapping(
                    {
                        **neutral.to_mapping(),
                        "sequence": 9,
                        "timestamp_monotonic_s": 10.06,
                        "focused": False,
                        "keys": {**neutral.to_mapping()["keys"], "w": True},
                    }
                )
                client.sendall(GAME_CONTROL.encode_input_packet(focus_lost))
                boundary_command = runtime.poll(now_s=10.06, dt_s=0.0)
                boundary_telemetry = runtime.telemetry(now_s=10.06)
                self.assertTrue(boundary_command.safe_stop)
                self.assertEqual(boundary_command.reason, "focus_lost")
                self.assertTrue(boundary_telemetry["safe_stop"])
                self.assertEqual(boundary_telemetry["packets_applied"], 3)
            finally:
                client.close()
                runtime.close()
            self.assertFalse(path.exists())

    def test_game_command_runtime_persists_summon_and_teleport_response(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            state_path = Path(temporary) / "world-state.json"
            runtime_socket, provider_socket = socket.socketpair(
                socket.AF_UNIX,
                socket.SOCK_SEQPACKET,
            )
            provider_socket.settimeout(1.0)
            world = MODULE._GameWorldStateRuntime(
                path=state_path,
                world_id="town10:test",
                world_revision="a" * 64,
                checkpoint_seconds=0.75,
            )
            runtime = MODULE.GameCommandRuntime(runtime_socket, world)
            current_pose = WORLD_STATE.WorldPose(10.0, 20.0, 0.8, 0.5)
            try:
                summon = self.game_command_request(
                    '/summon matrix:teleport_point ~1 ~-2 ~ {Tags:["XX"]}',
                    sequence=1,
                    request_character="b",
                )
                provider_socket.send(MC_COMMANDS.encode_command_request(summon))

                self.assertFalse(
                    runtime.poll(current_pose=current_pose, command_allowed=True)
                )
                summon_response = MC_COMMANDS.decode_command_response(
                    provider_socket.recv(MC_COMMANDS.MAX_COMMAND_PACKET_BYTES)
                )
                self.assertTrue(summon_response.ok)
                self.assertEqual(summon_response.code, "OK_SUMMONED")
                self.assertFalse(summon_response.restart_required)
                self.assertEqual(
                    summon_response.data["position"],
                    [11.0, 18.0, 0.8],
                )
                persisted = WORLD_STATE.WorldStateStore(
                    state_path,
                    world_id="town10:test",
                    world_revision="a" * 64,
                ).load()
                self.assertEqual(len(persisted.teleport_points), 1)
                self.assertIsNone(persisted.last_exit)

                teleport = self.game_command_request(
                    "/tp @s @e["
                    "type=matrix:teleport_point,tag=XX,limit=1,sort=nearest]",
                    sequence=2,
                    request_character="c",
                )
                provider_socket.send(MC_COMMANDS.encode_command_request(teleport))

                # The successful response must be queued before poll asks the
                # supervisor to cold-restart the complete runtime generation.
                self.assertTrue(
                    runtime.poll(current_pose=current_pose, command_allowed=True)
                )
                teleport_response = MC_COMMANDS.decode_command_response(
                    provider_socket.recv(MC_COMMANDS.MAX_COMMAND_PACKET_BYTES)
                )
                self.assertTrue(teleport_response.ok)
                self.assertEqual(teleport_response.code, "OK_TELEPORT_RESTART")
                self.assertTrue(teleport_response.restart_required)
                self.assertTrue(runtime.restart_requested)

                reloaded = WORLD_STATE.WorldStateStore(
                    state_path,
                    world_id="town10:test",
                    world_revision="a" * 64,
                ).load()
                self.assertEqual(
                    reloaded.last_exit,
                    WORLD_STATE.WorldPose(11.0, 18.0, 0.8, 0.5),
                )
                self.assertEqual(runtime.commands_executed, 2)
            finally:
                provider_socket.close()
                runtime.close()

    def test_required_world_checkpoint_surfaces_durable_write_failure(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            world = MODULE._GameWorldStateRuntime(
                path=Path(temporary) / "world-state.json",
                world_id="town10:test",
                world_revision="a" * 64,
                checkpoint_seconds=0.75,
            )
            snapshot = self.snapshot()
            snapshot.qpos[2] = 0.8
            snapshot.qpos[3] = 1.0
            failure = WORLD_STATE.WorldStateError("simulated fsync failure")

            with mock.patch.object(world.store, "save", side_effect=failure):
                with self.assertRaisesRegex(
                    WORLD_STATE.WorldStateError,
                    "simulated fsync failure",
                ):
                    world.checkpoint(
                        snapshot,
                        now_s=1.0,
                        force=True,
                        required=True,
                    )

            self.assertEqual(world.last_error, "simulated fsync failure")
            self.assertEqual(world.checkpoint_count, 0)
            self.assertIsNone(world.state.last_exit)

    def test_game_command_runtime_waits_for_policy_writer_ack(self) -> None:
        runtime_socket, provider_socket = socket.socketpair(
            socket.AF_UNIX,
            socket.SOCK_SEQPACKET,
        )
        provider_socket.setblocking(False)

        class PolicySlots:
            def __init__(self):
                self.transition_id = None
                self.result = None

            @staticmethod
            def strategy_loadout_mapping():
                return {
                    "version": 1,
                    "available": True,
                    "status": "ready",
                    "active_slot": "locomotion",
                    "pending": None,
                    "slots": [],
                    "resident_models": [],
                }

            def request_policy_slot_assignment(self, command, *, transition_id):
                self.transition_id = transition_id
                self.command = command
                return None

            def poll_policy_slot_assignment(self, transition_id):
                self.assert_transition = transition_id
                return self.result

        slots = PolicySlots()
        runtime = MODULE.GameCommandRuntime(
            runtime_socket,
            None,
            policy_slots=slots,
        )
        request = self.game_command_request(
            "/policy recovery host",
            sequence=1,
            request_character="f",
        )
        pose = WORLD_STATE.WorldPose(1.0, 2.0, 0.8, 0.0)
        try:
            provider_socket.send(MC_COMMANDS.encode_command_request(request))
            self.assertFalse(runtime.poll(current_pose=pose, command_allowed=True))
            self.assertEqual(slots.command, request.command)
            self.assertEqual(slots.transition_id, request.request_id)
            with self.assertRaises(BlockingIOError):
                provider_socket.recv(4096)

            loadout = slots.strategy_loadout_mapping()
            slots.result = (
                True,
                "OK_POLICY_SLOT_ASSIGNED",
                "assigned",
                loadout,
            )
            self.assertFalse(runtime.poll(current_pose=pose, command_allowed=True))
            response = MC_COMMANDS.decode_command_response(
                provider_socket.recv(4096)
            )
            self.assertTrue(response.ok)
            self.assertEqual(response.code, "OK_POLICY_SLOT_ASSIGNED")
            self.assertEqual(response.data["strategy_loadout"], loadout)
            self.assertEqual(runtime.policy_changes_executed, 1)
        finally:
            provider_socket.close()
            runtime.close()

    def test_game_command_runtime_rejects_commands_until_panel_safe_stop(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            state_path = Path(temporary) / "world-state.json"
            runtime_socket, provider_socket = socket.socketpair(
                socket.AF_UNIX,
                socket.SOCK_SEQPACKET,
            )
            provider_socket.settimeout(1.0)
            world = MODULE._GameWorldStateRuntime(
                path=state_path,
                world_id="town10:test",
                world_revision="a" * 64,
                checkpoint_seconds=0.75,
            )
            runtime = MODULE.GameCommandRuntime(runtime_socket, world)
            request = self.game_command_request(
                '/summon matrix:teleport_point ~ ~ ~ {Tags:["blocked"]}',
                sequence=1,
                request_character="b",
            )
            try:
                provider_socket.send(MC_COMMANDS.encode_command_request(request))
                self.assertFalse(
                    runtime.poll(
                        current_pose=WORLD_STATE.WorldPose(1.0, 2.0, 0.8, 0.0),
                        command_allowed=False,
                    )
                )
                response = MC_COMMANDS.decode_command_response(
                    provider_socket.recv(MC_COMMANDS.MAX_COMMAND_PACKET_BYTES)
                )
                self.assertFalse(response.ok)
                self.assertEqual(response.code, "E_NOT_PAUSED")
                self.assertFalse(response.restart_required)
                self.assertEqual(world.state.teleport_points, ())
                self.assertFalse(state_path.exists())
                self.assertEqual(runtime.rejected_commands, 1)
                self.assertEqual(runtime.commands_executed, 0)
            finally:
                provider_socket.close()
                runtime.close()

    def test_game_command_runtime_rejects_out_of_order_and_reused_ids(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            state_path = Path(temporary) / "world-state.json"
            runtime_socket, provider_socket = socket.socketpair(
                socket.AF_UNIX,
                socket.SOCK_SEQPACKET,
            )
            provider_socket.settimeout(1.0)
            world = MODULE._GameWorldStateRuntime(
                path=state_path,
                world_id="town10:test",
                world_revision="a" * 64,
                checkpoint_seconds=0.75,
            )
            runtime = MODULE.GameCommandRuntime(runtime_socket, world)
            pose = WORLD_STATE.WorldPose(1.0, 2.0, 0.8, 0.0)
            requests = (
                self.game_command_request(
                    '/summon matrix:teleport_point ~ ~ ~ {Tags:["first"]}',
                    sequence=3,
                    request_character="b",
                ),
                self.game_command_request(
                    '/summon matrix:teleport_point ~ ~ ~ {Tags:["stale"]}',
                    sequence=2,
                    request_character="c",
                ),
                self.game_command_request(
                    '/summon matrix:teleport_point ~ ~ ~ {Tags:["reused"]}',
                    sequence=4,
                    request_character="b",
                ),
            )
            try:
                responses = []
                for request in requests:
                    provider_socket.send(MC_COMMANDS.encode_command_request(request))
                    self.assertFalse(
                        runtime.poll(current_pose=pose, command_allowed=True)
                    )
                    responses.append(
                        MC_COMMANDS.decode_command_response(
                            provider_socket.recv(MC_COMMANDS.MAX_COMMAND_PACKET_BYTES)
                        )
                    )

                self.assertTrue(responses[0].ok)
                self.assertEqual(responses[0].code, "OK_SUMMONED")
                self.assertFalse(responses[1].ok)
                self.assertEqual(responses[1].code, "E_PROTOCOL_IDENTITY")
                self.assertIn("sequence", responses[1].message)
                self.assertFalse(responses[2].ok)
                self.assertEqual(responses[2].code, "E_PROTOCOL_IDENTITY")
                self.assertIn("request_id", responses[2].message)
                self.assertEqual(len(world.state.teleport_points), 1)
                self.assertEqual(runtime.commands_executed, 1)
                self.assertEqual(runtime.protocol_errors, 2)
                self.assertEqual(runtime.last_sequence, 4)
            finally:
                provider_socket.close()
                runtime.close()

    @mock.patch.object(MODULE.subprocess, "Popen")
    def test_native_process_group_runs_locked_binary_directly(self, popen) -> None:
        process = mock.Mock()
        popen.return_value = process
        group = MODULE.NativeProcessGroup(Path("/sonic"), {})
        group.start_deploy(interface="lo", zmq_port=6000)

        guarded_command = popen.call_args.args[0]
        self.assertEqual(guarded_command[0], sys.executable)
        self.assertEqual(
            Path(guarded_command[1]).name,
            "exec_with_parent_death_signal.py",
        )
        self.assertIn("--exec-command", guarded_command[: guarded_command.index("--")])
        command = guarded_command[guarded_command.index("--") + 1 :]
        self.assertEqual(command[0], "/sonic/gear_sonic_deploy/target/release/g1_deploy_onnx_ref")
        self.assertNotIn("deploy.sh", command)
        self.assertEqual(command[1], "lo")
        self.assertIn("--disable-crc-check", command)
        self.assertEqual(
            command[command.index("--planner-backend") + 1],
            "onnx",
        )
        self.assertEqual(command[command.index("--zmq-port") + 1], "6000")
        self.assertEqual(
            group.env["FASTRTPS_DEFAULT_PROFILES_FILE"],
            "/sonic/gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/config/fastrtps_profile.xml",
        )
        self.assertEqual(group.env["ROS_LOCALHOST_ONLY"], "1")
        self.assertEqual(group.env["PYTHONNOUSERSITE"], "1")
        self.assertEqual(group.env["PYTHONPATH"], "/sonic")

    @mock.patch.object(MODULE.subprocess, "Popen")
    def test_replacement_deploy_receives_private_writer_gate(self, popen) -> None:
        popen.return_value = mock.Mock(pid=4001)
        group = MODULE.NativeProcessGroup(Path("/sonic"), {})
        group.start_deploy(
            interface="lo",
            zmq_port=6000,
            writer_control_socket=Path("/run/user/1000/sonic.sock"),
            physical_reentry=True,
        )
        guarded = popen.call_args.args[0]
        command = guarded[guarded.index("--") + 1 :]
        self.assertEqual(
            command[command.index("--writer-control-socket") + 1],
            "/run/user/1000/sonic.sock",
        )
        self.assertIn("--writer-reentry", command)
        self.assertEqual(
            command[command.index("--planner-backend") + 1],
            "onnx",
        )
        self.assertEqual(
            command[command.index("--writer-reentry-hold-seconds") + 1],
            "0.5",
        )
        self.assertEqual(
            command[command.index("--writer-reentry-align-seconds") + 1],
            "5.0",
        )
        self.assertEqual(
            command[command.index("--writer-reentry-align-fraction") + 1],
            "0.0",
        )
        self.assertEqual(
            command[command.index("--writer-reentry-settle-seconds") + 1],
            "1.0",
        )
        self.assertEqual(
            command[command.index("--writer-reentry-blend-seconds") + 1],
            "6.0",
        )
        self.assertEqual(group.deploy_pid(), 4001)

    @mock.patch.object(MODULE.subprocess, "Popen")
    def test_physical_reentry_requires_writer_gate(self, popen) -> None:
        group = MODULE.NativeProcessGroup(Path("/sonic"), {})

        with self.assertRaisesRegex(RuntimeError, "requires a writer gate"):
            group.start_deploy(
                interface="lo",
                zmq_port=6000,
                physical_reentry=True,
            )

        popen.assert_not_called()

    @mock.patch.object(MODULE.subprocess, "Popen")
    def test_deploy_only_restart_advances_generation_and_preserves_game_input(
        self, popen
    ) -> None:
        first = mock.Mock(pid=4101)
        second = mock.Mock(pid=4102)
        game_input = mock.Mock(pid=4200)
        popen.side_effect = (first, second)
        returncodes = {4101: None, 4102: None, 4200: None}

        group = MODULE.NativeProcessGroup(Path("/sonic"), {})
        with mock.patch.object(
            MODULE,
            "_peek_child_returncode",
            side_effect=lambda process: returncodes[process.pid],
        ), mock.patch.object(MODULE.os, "killpg") as killpg:
            self.assertEqual(group.start_deploy(interface="lo", zmq_port=6000), 1)
            group.children.append(("game-input", game_input))
            group.begin_deploy_stop()
            returncodes[4101] = 0

            self.assertIsNone(group.failed_child())
            self.assertFalse(group.deploy_alive())
            self.assertEqual(group.start_deploy(interface="lo", zmq_port=6000), 2)
            self.assertTrue(group.deploy_alive())

        killpg.assert_not_called()
        self.assertEqual(group.deploy_generation, 2)
        self.assertIn(("game-input", game_input), group.children)

    @mock.patch.object(MODULE.subprocess, "Popen")
    def test_recovery_worker_command_is_writer_gated_host_process(self, popen) -> None:
        process = mock.Mock(pid=4300)
        popen.return_value = process
        group = MODULE.NativeProcessGroup(Path("/sonic"), {})

        group.start_recovery_policy(
            "/policy/python",
            Path("/matrix/scripts/matrix_sonic_host_worker.py"),
            interface="lo",
            control_socket=Path("/run/user/1000/recovery.sock"),
            model=Path("/models/prone-v1.onnx"),
            fallback_models=(Path("/models/prone-v2.onnx"),),
            fallback_after_s=8.0,
            amp_config=Path("/models/amp.json"),
            amp_model=Path("/models/amp.onnx"),
            amp_config_sha256="1" * 64,
            amp_model_sha256="2" * 64,
            initial_controller="amp",
        )

        guarded = popen.call_args.args[0]
        self.assertIn("--exec-command", guarded[: guarded.index("--")])
        command = guarded[guarded.index("--") + 1 :]
        self.assertEqual(
            command[:3],
            [
                "/policy/python",
                "-u",
                "/matrix/scripts/matrix_sonic_host_worker.py",
            ],
        )
        self.assertEqual(command[command.index("--interface") + 1], "lo")
        self.assertEqual(
            command[command.index("--control-socket") + 1],
            "/run/user/1000/recovery.sock",
        )
        self.assertEqual(
            command[command.index("--initial-controller") + 1], "amp"
        )
        self.assertEqual(
            command[command.index("--fallback-model") + 1],
            "/models/prone-v2.onnx",
        )
        self.assertEqual(
            command[command.index("--amp-hold-config") + 1],
            "/models/amp.json",
        )

    @unittest.skipUnless(
        hasattr(socket, "SOCK_SEQPACKET") and hasattr(socket, "SO_PEERCRED"),
        "Linux Unix seqpacket credentials are required",
    )
    def test_recovery_worker_control_enforces_ready_before_go(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "recovery.sock"
            control = MODULE._RecoveryWorkerControl(path)
            control.open()
            control.bind_expected_peer_pid(os.getpid())
            peer = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            try:
                peer.connect(str(path))
                peer.send(
                    json.dumps(
                        {
                            "schema": "matrix.sonic_host_worker.control.v1",
                            "event": "READY_NO_WRITER",
                            "writer_created": False,
                        }
                    ).encode("utf-8")
                )
                control.poll()
                self.assertTrue(control.ready)
                self.assertFalse(control.first_write)

                control.send("GO")
                command = json.loads(peer.recv(4096).decode("utf-8"))
                self.assertEqual(command["command"], "GO")
                episode_id = command["episode_id"]
                peer.send(
                    json.dumps(
                        {
                            "schema": "matrix.sonic_host_worker.control.v1",
                            "event": "FIRST_WRITE",
                            "episode_id": episode_id,
                        }
                    ).encode("utf-8")
                )
                control.poll()
                self.assertTrue(control.first_write)
                peer.send(
                    json.dumps(
                        {
                            "schema": "matrix.sonic_host_worker.control.v1",
                            "event": "POLICY_FALLBACK_DUE",
                            "requires_supervisor_authorization": True,
                            "policy_index": 0,
                            "next_policy_index": 1,
                            "episode_id": episode_id,
                        }
                    ).encode("utf-8")
                )
                control.poll()
                self.assertTrue(control.fallback_due)

                control.send("ADVANCE_POLICY")
                command = json.loads(peer.recv(4096).decode("utf-8"))
                self.assertEqual(command["command"], "ADVANCE_POLICY")
                transition_id = command["transition_id"]
                peer.send(
                    json.dumps(
                        {
                            "schema": "matrix.sonic_host_worker.control.v1",
                            "event": "POLICY_SWITCH",
                            "from_policy_index": 0,
                            "to_policy_index": 1,
                            "physical_continuation": True,
                            "episode_id": episode_id,
                            "transition_id": transition_id,
                        }
                    ).encode("utf-8")
                )
                control.poll()
                self.assertTrue(control.fallback_due)
                peer.send(
                    json.dumps(
                        {
                            "schema": "matrix.sonic_host_worker.control.v1",
                            "event": "POLICY_SWITCH_FIRST_WRITE",
                            "from_policy_index": 0,
                            "to_policy_index": 1,
                            "physical_continuation": True,
                            "writer_reused": True,
                            "episode_id": episode_id,
                            "transition_id": transition_id,
                        }
                    ).encode("utf-8")
                )
                control.poll()
                self.assertFalse(control.fallback_due)
                self.assertEqual(
                    control.telemetry()["last_policy_switch"]["to_policy_index"],
                    1,
                )
                control.send("STOP")
                self.assertEqual(
                    json.loads(peer.recv(4096).decode("utf-8"))["command"],
                    "STOP",
                )
                peer.send(
                    json.dumps(
                        {
                            "schema": "matrix.sonic_host_worker.control.v1",
                            "event": "STOPPED",
                            "episode_id": episode_id,
                        }
                    ).encode("utf-8")
                )
                control.poll()
                control.reset_for_start()
                completed = control.telemetry()["completed_episodes"]
                self.assertEqual(len(completed), 1)
                self.assertTrue(completed[0]["first_write"])
                self.assertTrue(completed[0]["stopped"])
                self.assertEqual(
                    completed[0]["last_policy_switch"]["to_policy_index"], 1
                )
            finally:
                peer.close()
                control.close()
            self.assertFalse(path.exists())

    @unittest.skipUnless(
        hasattr(socket, "SOCK_SEQPACKET") and hasattr(socket, "SO_PEERCRED"),
        "Linux Unix seqpacket credentials are required",
    )
    def test_joint_hold_supersedes_in_flight_fallback_due(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "recovery.sock"
            control = MODULE._RecoveryWorkerControl(path)
            control.open()
            control.bind_expected_peer_pid(os.getpid())
            peer = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            try:
                peer.connect(str(path))
                peer.send(
                    json.dumps(
                        {
                            "schema": "matrix.sonic_host_worker.control.v1",
                            "event": "READY_NO_WRITER",
                            "writer_created": False,
                        }
                    ).encode("utf-8")
                )
                control.poll()
                control.send("GO")
                go = json.loads(peer.recv(4096).decode("utf-8"))
                episode_id = go["episode_id"]
                peer.send(
                    json.dumps(
                        {
                            "schema": "matrix.sonic_host_worker.control.v1",
                            "event": "FIRST_WRITE",
                            "episode_id": episode_id,
                        }
                    ).encode("utf-8")
                )
                control.poll()

                # Queue the worker event but deliberately do not poll it.  The
                # opposite socket direction has no shared ordering, so this is
                # the real race seen when the stable gate requests hold.
                peer.send(
                    json.dumps(
                        {
                            "schema": "matrix.sonic_host_worker.control.v1",
                            "event": "POLICY_FALLBACK_DUE",
                            "requires_supervisor_authorization": True,
                            "policy_index": 0,
                            "next_policy_index": 1,
                            "episode_id": episode_id,
                        }
                    ).encode("utf-8")
                )
                control.send("ENTER_JOINT_HOLD")
                hold = json.loads(peer.recv(4096).decode("utf-8"))
                self.assertEqual(hold["command"], "ENTER_JOINT_HOLD")
                self.assertEqual(hold["hold_kind"], "joint_pose")

                control.poll()
                self.assertFalse(control.fallback_due)
                self.assertEqual(len(control.superseded_fallback_due_events), 1)
                self.assertEqual(
                    control.superseded_fallback_due_events[0]["superseded_by"],
                    "joint_pose",
                )

                peer.send(
                    json.dumps(
                        {
                            "schema": "matrix.sonic_host_worker.control.v1",
                            "event": "JOINT_HOLD_FIRST_WRITE",
                            "episode_id": episode_id,
                            "transition_id": hold["transition_id"],
                            "writer_reused": True,
                            "measured_joint_target": True,
                            "measured_joint_count": 29,
                            "capture_once": True,
                            "lowstate_capture_age_s": 0.01,
                            "target_velocity_zero": True,
                            "feedforward_torque_zero": True,
                        }
                    ).encode("utf-8")
                )
                control.poll()
                self.assertTrue(control.joint_hold_first_write)

                control.send("STOP")
                self.assertEqual(
                    json.loads(peer.recv(4096).decode("utf-8"))["command"],
                    "STOP",
                )
                peer.send(
                    json.dumps(
                        {
                            "schema": "matrix.sonic_host_worker.control.v1",
                            "event": "STOPPED",
                            "episode_id": episode_id,
                        }
                    ).encode("utf-8")
                )
                control.poll()
            finally:
                peer.close()
                control.close()

    @unittest.skipUnless(
        hasattr(socket, "SOCK_SEQPACKET") and hasattr(socket, "SO_PEERCRED"),
        "Linux Unix seqpacket credentials are required",
    )
    def test_recovery_worker_control_rejects_unexpected_peer_pid(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "recovery.sock"
            control = MODULE._RecoveryWorkerControl(path)
            control.open()
            control.bind_expected_peer_pid(os.getpid() + 100000)
            peer = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            try:
                peer.connect(str(path))
                with self.assertRaisesRegex(RuntimeError, "peer PID mismatch"):
                    control.poll()
                self.assertEqual(control.peer_pid_mismatches, 1)
                self.assertIsNone(control.connection)
            finally:
                peer.close()
                control.close()

    @unittest.skipUnless(
        hasattr(socket, "SOCK_SEQPACKET") and hasattr(socket, "SO_PEERCRED"),
        "Linux Unix seqpacket credentials are required",
    )
    def test_sonic_writer_control_tracks_hard_writer_fence(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "sonic.sock"
            control = MODULE._SonicWriterControl(path)
            control.open()
            control.bind_expected_peer_pid(os.getpid())
            peer = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            try:
                peer.connect(str(path))

                def event(
                    name: str,
                    created: bool,
                    *,
                    authorized: bool | None = None,
                ) -> bytes:
                    payload = {
                        "schema": "matrix.sonic_deploy.control.v1",
                        "event": name,
                        "writer_scope": "rt/lowcmd",
                        "lowcmd_writer_created": created,
                    }
                    if authorized is not None:
                        payload["write_authorized"] = authorized
                    return json.dumps(payload).encode("utf-8")

                peer.send(event("READY_NO_LOWCMD_WRITER", False))
                peer.send(event("SHADOW_READY_NO_LOWCMD_WRITER", False))
                control.poll()
                self.assertTrue(control.ready)
                self.assertTrue(control.shadow_ready)
                self.assertFalse(control.writer_created)
                control.send("GO")
                self.assertEqual(peer.recv(4096), b"GO")
                peer.send(event("WRITER_CREATED", True))
                peer.send(event("FIRST_WRITE", True))
                control.poll()
                self.assertTrue(control.writer_created)
                self.assertTrue(control.first_write)
                self.assertTrue(control.current_first_write)
                peer.send(
                    event(
                        "REENTRY_ALIGNMENT_COMPLETE",
                        True,
                        authorized=True,
                    )
                )
                peer.send(
                    event(
                        "REENTRY_SAFE_IDLE_HOLD_ACTIVE",
                        True,
                        authorized=True,
                    )
                )
                control.poll()
                self.assertTrue(control.reentry_alignment_complete)
                self.assertTrue(control.reentry_safe_idle_hold_active)
                self.assertFalse(control.reentry_policy_full_control)
                self.assertTrue(
                    control.telemetry()["reentry_safe_idle_hold_active"]
                )
                peer.send(
                    event(
                        "REENTRY_POLICY_FULL_CONTROL",
                        True,
                        authorized=True,
                    )
                )
                control.poll()
                self.assertTrue(control.reentry_policy_full_control)
                control.send("STOP")
                self.assertEqual(peer.recv(4096), b"STOP")
                peer.send(
                    json.dumps(
                        {
                            "schema": "matrix.sonic_deploy.control.v1",
                            "event": "WRITER_REVOKED",
                            "writer_scope": "rt/lowcmd",
                            "lowcmd_writer_created": True,
                            "write_authorized": False,
                        }
                    ).encode("utf-8")
                )
                control.poll()
                self.assertTrue(control.writer_created)
                self.assertTrue(control.writer_revoked)
                self.assertFalse(control.current_first_write)
                peer.send(event("STOPPED", False))
                control.poll()
                self.assertTrue(control.stopped)
                self.assertFalse(control.writer_created)
            finally:
                peer.close()
                control.close()

    @unittest.skipUnless(
        hasattr(socket, "SOCK_SEQPACKET") and hasattr(socket, "SO_PEERCRED"),
        "Linux Unix seqpacket credentials are required",
    )
    def test_resident_sonic_pause_resume_advances_writer_epoch(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "sonic.sock"
            control = MODULE._SonicWriterControl(
                path,
                require_authority_epoch=True,
            )
            control.open()
            control.bind_expected_peer_pid(os.getpid())
            peer = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            try:
                peer.connect(str(path))

                def event(
                    name: str,
                    created: bool,
                    authorized: bool,
                    epoch: int,
                ) -> bytes:
                    return json.dumps(
                        {
                            "schema": "matrix.sonic_deploy.control.v1",
                            "event": name,
                            "writer_scope": "rt/lowcmd",
                            "lowcmd_writer_created": created,
                            "write_authorized": authorized,
                            "authority_epoch": epoch,
                        }
                    ).encode("utf-8")

                peer.send(event("READY_NO_LOWCMD_WRITER", False, False, 0))
                control.poll()
                control.send("GO")
                self.assertEqual(peer.recv(4096), b"GO")
                peer.send(event("WRITER_CREATED", True, True, 1))
                peer.send(event("FIRST_WRITE", True, True, 1))
                control.poll()
                self.assertTrue(control.current_first_write)
                self.assertEqual(control.authority_epoch, 1)

                control.send("PAUSE")
                self.assertEqual(peer.recv(4096), b"PAUSE")
                peer.send(event("WRITER_PAUSED", True, False, 1))
                control.poll()
                self.assertTrue(control.paused)
                self.assertFalse(control.current_first_write)
                self.assertFalse(control.stopped)
                self.assertFalse(control.writer_revoked)

                control.send("RESUME")
                self.assertEqual(peer.recv(4096), b"RESUME")
                peer.send(event("WRITER_RESUMED", True, True, 2))
                control.poll()
                self.assertFalse(control.current_first_write)
                self.assertEqual(control.authority_epoch, 2)
                peer.send(event("FIRST_WRITE", True, True, 2))
                control.poll()
                self.assertTrue(control.current_first_write)
                self.assertEqual(control.resume_count, 1)
                self.assertFalse(control.stopped)
                self.assertFalse(control.writer_revoked)
            finally:
                peer.close()
                control.close()

    def test_resident_worker_ready_requires_gpu_policy_attestation(self) -> None:
        control = MODULE._RecoveryWorkerControl(
            Path("/unused/recovery.sock"),
            require_resident_attestation=True,
        )
        incomplete = json.dumps(
            {
                "schema": "matrix.sonic_host_worker.control.v1",
                "event": "READY_NO_WRITER",
                "writer_created": False,
            }
        ).encode("utf-8")
        with self.assertRaisesRegex(RuntimeError, "provider attestation"):
            control._handle_packet(incomplete)

        policies = [
            {
                "name": "host:prone",
                "execution_provider": "CUDAExecutionProvider",
                "warmed": True,
            },
            {
                "name": "amp:walk_run_getup",
                "execution_provider": "CUDAExecutionProvider",
                "warmed": True,
            },
            {
                "name": "kungfu:1307_recovery",
                "execution_provider": "CUDAExecutionProvider",
                "warmed": True,
            },
        ]
        ready = json.dumps(
            {
                "schema": "matrix.sonic_host_worker.control.v1",
                "event": "READY_NO_WRITER",
                "writer_created": False,
                "execution_provider": "CUDAExecutionProvider",
                "resident_policies": policies,
                "resident_policy_count": len(policies),
                "registered_policy_ids": ["host", "amp", "kungfu"],
                "initial_policy_id": "kungfu",
                "models_loaded_once": True,
                "models_warmed": True,
            }
        ).encode("utf-8")
        control._handle_packet(ready)
        self.assertTrue(control.ready)
        self.assertTrue(control.paused)
        self.assertEqual(control.execution_provider, "CUDAExecutionProvider")
        self.assertEqual(len(control.resident_policies), 3)
        self.assertEqual(control.registered_policy_ids, ["host", "amp", "kungfu"])
        self.assertEqual(control.initial_policy_id, "kungfu")

    def test_recovery_worker_control_tracks_resident_paused_writer(self) -> None:
        control = MODULE._RecoveryWorkerControl(Path("/unused/recovery.sock"))
        control.ready = True
        control.episode_id = 2
        control.pause_sent = True

        control._handle_packet(
            json.dumps(
                {
                    "schema": "matrix.sonic_host_worker.control.v1",
                    "event": "PAUSED_RESIDENT_WRITER",
                    "episode_id": 2,
                    "writer_created": True,
                    "write_authorized": False,
                    "writer_reused": True,
                }
            ).encode("utf-8")
        )
        control._handle_packet(
            json.dumps(
                {
                    "schema": "matrix.sonic_host_worker.control.v1",
                    "event": "STATUS",
                    "controller": "PAUSED_RESIDENT_WRITER",
                    "writer_created": True,
                    "write_authorized": False,
                }
            ).encode("utf-8")
        )

        telemetry = control.telemetry()
        self.assertTrue(telemetry["resident_paused"])
        self.assertTrue(telemetry["resident_writer_created"])
        self.assertFalse(telemetry["paused_no_writer"])

    def test_sonic_writer_safe_idle_hold_rejects_invalid_attestations(
        self,
    ) -> None:
        def packet(
            *,
            created: object = True,
            authorized: object = True,
        ) -> bytes:
            return json.dumps(
                {
                    "schema": "matrix.sonic_deploy.control.v1",
                    "event": "REENTRY_SAFE_IDLE_HOLD_ACTIVE",
                    "writer_scope": "rt/lowcmd",
                    "lowcmd_writer_created": created,
                    "write_authorized": authorized,
                }
            ).encode("utf-8")

        def eligible_control() -> MODULE._SonicWriterControl:
            control = MODULE._SonicWriterControl(Path("/unused/sonic.sock"))
            control.shadow_ready = True
            control.reentry_alignment_complete = True
            control.first_write = True
            control.writer_created = True
            return control

        invalid_states = (
            ("without shadow readiness", {"shadow_ready": False}, packet()),
            (
                "before alignment",
                {"reentry_alignment_complete": False},
                packet(),
            ),
            ("before first write", {"first_write": False}, packet()),
            ("without writer", {"writer_created": False}, packet()),
            ("payload reports no writer", {}, packet(created=False)),
            ("payload writer flag is not bool", {}, packet(created=1)),
            ("without authority", {}, packet(authorized=False)),
            ("authority flag is not bool", {}, packet(authorized=1)),
            ("after revocation", {"writer_revoked": True}, packet()),
        )
        for label, state, event_packet in invalid_states:
            control = eligible_control()
            for name, value in state.items():
                setattr(control, name, value)
            with self.subTest(label=label), self.assertRaises(RuntimeError):
                control._handle_packet(event_packet)
            self.assertFalse(control.reentry_safe_idle_hold_active)
            self.assertFalse(control.reentry_policy_full_control)

    def test_sonic_writer_safe_idle_hold_is_distinct_and_can_upgrade(
        self,
    ) -> None:
        def packet(event: str) -> bytes:
            return json.dumps(
                {
                    "schema": "matrix.sonic_deploy.control.v1",
                    "event": event,
                    "writer_scope": "rt/lowcmd",
                    "lowcmd_writer_created": True,
                    "write_authorized": True,
                }
            ).encode("utf-8")

        control = MODULE._SonicWriterControl(Path("/unused/sonic.sock"))
        control.shadow_ready = True
        control.reentry_alignment_complete = True
        control.first_write = True
        control.writer_created = True

        safe_hold = packet("REENTRY_SAFE_IDLE_HOLD_ACTIVE")
        control._handle_packet(safe_hold)
        self.assertTrue(control.reentry_safe_idle_hold_active)
        self.assertFalse(control.reentry_policy_full_control)
        with self.assertRaisesRegex(RuntimeError, "duplicate"):
            control._handle_packet(safe_hold)

        control._handle_packet(packet("REENTRY_POLICY_FULL_CONTROL"))
        self.assertTrue(control.reentry_policy_full_control)

        control.reset_for_start()
        self.assertFalse(control.reentry_safe_idle_hold_active)
        self.assertFalse(control.reentry_policy_full_control)

    def test_sonic_writer_rejects_safe_idle_hold_after_full_control(
        self,
    ) -> None:
        control = MODULE._SonicWriterControl(Path("/unused/sonic.sock"))
        control.shadow_ready = True
        control.reentry_alignment_complete = True
        control.first_write = True
        control.writer_created = True
        control.reentry_policy_full_control = True
        packet = json.dumps(
            {
                "schema": "matrix.sonic_deploy.control.v1",
                "event": "REENTRY_SAFE_IDLE_HOLD_ACTIVE",
                "writer_scope": "rt/lowcmd",
                "lowcmd_writer_created": True,
                "write_authorized": True,
            }
        ).encode("utf-8")

        with self.assertRaisesRegex(RuntimeError, "regressed"):
            control._handle_packet(packet)
        self.assertFalse(control.reentry_safe_idle_hold_active)
        self.assertTrue(control.reentry_policy_full_control)

    def test_physical_recovery_observation_reports_safe_idle_hold(self) -> None:
        coordinator = MODULE._PhysicalRecoveryCoordinator.__new__(
            MODULE._PhysicalRecoveryCoordinator
        )
        coordinator.worker = SimpleNamespace(
            poll=lambda: None,
            error=None,
            fallback_due=False,
            last_status=None,
            first_write=False,
            amp_hold_first_write=False,
            joint_hold_first_write=False,
            ready_recent=lambda **kwargs: False,
        )
        coordinator.sonic_writer = SimpleNamespace(
            poll=lambda: None,
            error=None,
            ready=False,
            expected_peer_pid=4321,
            shadow_ready=True,
            writer_created=True,
            writer_revoked=False,
            current_first_write=True,
            reentry_policy_full_control=False,
            reentry_safe_idle_hold_active=True,
        )
        coordinator.fsm = mock.Mock()
        coordinator.fsm.state = MODULE.RecoveryState.GAME_SONIC
        coordinator.fsm.step.return_value = MODULE.RecoveryOutput(
            previous_state=MODULE.RecoveryState.GAME_SONIC,
            state=MODULE.RecoveryState.GAME_SONIC,
        )
        coordinator.handoff_mode = "amp"
        coordinator.replacement_sonic_ready_s = None
        coordinator.replacement_sonic_first_fresh_s = None
        coordinator.command_frame_epoch = 0
        coordinator.last_output = None
        coordinator._maybe_authorize_policy_advance = mock.Mock()
        coordinator._fall_level = mock.Mock(return_value=False)
        snapshot = self.snapshot(
            low_cmd_fresh=True,
            low_cmd_received=True,
            low_cmd_age_s=0.01,
        )
        snapshot.qpos[2] = 0.7
        snapshot.qpos[3] = 1.0
        processes = SimpleNamespace(
            deploy_generation=8,
            deploy_alive=lambda: True,
            recovery_policy_alive=lambda: False,
        )

        coordinator.observe(
            snapshot,
            now_s=1.0,
            neutral_confirmed=False,
            foot_contact=True,
            grounded_contact=True,
            processes=processes,
        )

        observation = coordinator.fsm.step.call_args.args[0]
        self.assertTrue(observation.deploy_safe_idle_hold)
        self.assertFalse(observation.deploy_policy_full_control)
        self.assertTrue(observation.deploy_first_write)

    def test_stabilizing_to_wait_neutral_reanchors_game_core_once(self) -> None:
        core = mock.Mock(spec=GAME_CONTROL.GameControlCore)
        measured_heading = math.radians(100.0)
        transition = MODULE.RecoveryOutput(
            previous_state=MODULE.RecoveryState.SONIC_STABILIZING,
            state=MODULE.RecoveryState.WAIT_NEUTRAL,
            inhibit_game_input=True,
        )
        steady_wait = MODULE.RecoveryOutput(
            previous_state=MODULE.RecoveryState.WAIT_NEUTRAL,
            state=MODULE.RecoveryState.WAIT_NEUTRAL,
            inhibit_game_input=True,
        )

        self.assertTrue(
            MODULE._reanchor_game_heading_after_recovery_transition(
                transition,
                core,
                measured_heading_rad=measured_heading,
            )
        )
        self.assertFalse(
            MODULE._reanchor_game_heading_after_recovery_transition(
                steady_wait,
                core,
                measured_heading_rad=math.radians(-40.0),
            )
        )

        core.reanchor_heading.assert_called_once_with(measured_heading)

    def test_stabilizing_to_wait_neutral_captures_restart_anchor_once(self) -> None:
        coordinator = MODULE._PhysicalRecoveryCoordinator.__new__(
            MODULE._PhysicalRecoveryCoordinator
        )
        coordinator.worker = SimpleNamespace(
            poll=lambda: None,
            error=None,
            last_status=None,
            first_write=False,
            amp_hold_first_write=False,
            joint_hold_first_write=False,
            ready_recent=lambda **kwargs: False,
        )
        coordinator.sonic_writer = SimpleNamespace(
            poll=lambda: None,
            error=None,
            ready=False,
            expected_peer_pid=4321,
            shadow_ready=True,
            writer_created=True,
            writer_revoked=False,
            stopped=False,
            current_first_write=True,
            reentry_policy_full_control=False,
            reentry_safe_idle_hold_active=True,
        )
        coordinator.fsm = mock.Mock()
        coordinator.fsm.state = MODULE.RecoveryState.SONIC_STABILIZING
        outputs = iter(
            (
                MODULE.RecoveryOutput(
                    previous_state=MODULE.RecoveryState.SONIC_STABILIZING,
                    state=MODULE.RecoveryState.WAIT_NEUTRAL,
                    inhibit_game_input=True,
                ),
                MODULE.RecoveryOutput(
                    previous_state=MODULE.RecoveryState.WAIT_NEUTRAL,
                    state=MODULE.RecoveryState.WAIT_NEUTRAL,
                    inhibit_game_input=True,
                ),
            )
        )

        def transition(_observation):
            output = next(outputs)
            coordinator.fsm.state = output.state
            return output

        coordinator.fsm.step.side_effect = transition
        coordinator.handoff_mode = "amp"
        coordinator.replacement_sonic_ready_s = None
        coordinator.replacement_sonic_first_fresh_s = None
        coordinator.last_output = None
        coordinator.last_transition_s = None
        coordinator.episodes = 0
        coordinator.recoveries = 0
        coordinator.initial_root_yaw_rad = math.radians(10.0)
        coordinator.restarted_root_yaw_rad = None
        coordinator.command_frame_rotation_rad = 0.0
        coordinator.command_frame_epoch = 0
        coordinator.reframe_limited_frames = 0
        coordinator.last_reframe_limited = False
        coordinator.last_reframe_heading_error_rad = 0.0
        coordinator.previous_sonic_writer_revoked = False
        coordinator.previous_sonic_stopped = False
        coordinator._maybe_authorize_policy_advance = mock.Mock()
        coordinator._fall_level = mock.Mock(return_value=False)
        processes = SimpleNamespace(
            deploy_generation=8,
            deploy_alive=lambda: True,
            recovery_policy_alive=lambda: False,
        )

        first_heading = math.radians(100.0)
        first_snapshot = self.snapshot_with_yaw(
            first_heading,
            low_cmd_fresh=True,
            low_cmd_received=True,
            low_cmd_age_s=0.01,
        )
        first_snapshot.qpos[2] = 0.7
        second_snapshot = self.snapshot_with_yaw(
            math.radians(-40.0),
            low_cmd_fresh=True,
            low_cmd_received=True,
            low_cmd_age_s=0.01,
        )
        second_snapshot.qpos[2] = 0.7

        with mock.patch.object(
            coordinator,
            "_capture_restart_anchor",
            wraps=coordinator._capture_restart_anchor,
        ) as capture_restart_anchor:
            first = coordinator.observe(
                first_snapshot,
                now_s=1.0,
                neutral_confirmed=False,
                foot_contact=True,
                grounded_contact=True,
                processes=processes,
            )
            captured_rotation = coordinator.command_frame_rotation_rad
            second = coordinator.observe(
                second_snapshot,
                now_s=1.02,
                neutral_confirmed=False,
                foot_contact=True,
                grounded_contact=True,
                processes=processes,
            )

        self.assertIs(first.previous_state, MODULE.RecoveryState.SONIC_STABILIZING)
        self.assertIs(first.state, MODULE.RecoveryState.WAIT_NEUTRAL)
        self.assertIs(second.previous_state, MODULE.RecoveryState.WAIT_NEUTRAL)
        self.assertIs(second.state, MODULE.RecoveryState.WAIT_NEUTRAL)
        capture_restart_anchor.assert_called_once_with(first_snapshot.qpos)
        self.assertEqual(coordinator.command_frame_epoch, 1)
        self.assertEqual(coordinator.last_wire_facing_heading_rad, 0.0)
        self.assertAlmostEqual(coordinator.restarted_root_yaw_rad, first_heading)
        self.assertAlmostEqual(
            captured_rotation,
            -MODULE.wrap_angle_rad(
                first_heading - coordinator.initial_root_yaw_rad
            ),
        )
        self.assertAlmostEqual(
            coordinator.command_frame_rotation_rad, captured_rotation
        )

    @unittest.skipUnless(
        hasattr(socket, "SOCK_SEQPACKET") and hasattr(socket, "SO_PEERCRED"),
        "Linux Unix seqpacket credentials are required",
    )
    def test_sonic_writer_control_accepts_internal_fail_closed_cleanup(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "sonic-fatal.sock"
            control = MODULE._SonicWriterControl(path)
            control.open()
            control.bind_expected_peer_pid(os.getpid())
            peer = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            try:
                peer.connect(str(path))

                def event(
                    name: str,
                    created: bool,
                    *,
                    authorized: bool | None = None,
                ) -> bytes:
                    payload = {
                        "schema": "matrix.sonic_deploy.control.v1",
                        "event": name,
                        "writer_scope": "rt/lowcmd",
                        "lowcmd_writer_created": created,
                    }
                    if authorized is not None:
                        payload["write_authorized"] = authorized
                    return json.dumps(payload).encode("utf-8")

                peer.send(event("READY_NO_LOWCMD_WRITER", False))
                control.poll()
                self.assertFalse(control.shadow_ready)
                control.send("GO")
                self.assertEqual(peer.recv(4096), b"GO")
                peer.send(event("WRITER_CREATED", True))
                peer.send(event("FIRST_WRITE", True))
                control.poll()

                peer.send(
                    event(
                        "WRITER_FAILED_CLOSED",
                        True,
                        authorized=False,
                    )
                )
                control.poll()
                self.assertTrue(control.writer_failed_closed)
                self.assertTrue(control.writer_revoked)
                self.assertFalse(control.current_first_write)
                self.assertIn("failed closed", control.error)

                # Internal fatal cleanup is allowed to finish without a
                # supervisor STOP, but it must attest that the publisher is
                # gone and authority remains revoked.
                peer.send(event("STOPPED", False, authorized=False))
                control.poll()
                self.assertTrue(control.stopped)
                self.assertFalse(control.writer_created)
            finally:
                peer.close()
                control.close()

    def test_restarted_deploy_reframes_game_command_from_new_yaw_anchor(self) -> None:
        coordinator = MODULE._PhysicalRecoveryCoordinator.__new__(
            MODULE._PhysicalRecoveryCoordinator
        )
        coordinator.initial_root_yaw_rad = 0.0
        coordinator.restarted_root_yaw_rad = None
        coordinator.command_frame_rotation_rad = 0.0
        coordinator.command_frame_epoch = 0
        coordinator.last_wire_facing_heading_rad = None
        coordinator.reframe_limited_frames = 0
        coordinator.last_reframe_limited = False
        coordinator.last_reframe_heading_error_rad = 0.0
        half = math.pi / 4.0
        coordinator._capture_restart_anchor(
            [0.0, 0.0, 0.7, math.cos(half), 0.0, 0.0, math.sin(half)]
        )
        command = GAME_CONTROL.RobotMotionCommand(
            sequence=5,
            movement=(1.0, 0.0, 0.0),
            facing=(0.0, 1.0, 0.0),
            speed_mps=1.0,
            locomotion_mode=GAME_CONTROL.SONIC_RUN_MODE,
            mode="move",
            safe_stop=False,
            reason="active",
        )

        reframed = coordinator.reframe_game_command(command)

        self.assertAlmostEqual(coordinator.restarted_root_yaw_rad, math.pi / 2.0)
        self.assertAlmostEqual(coordinator.command_frame_rotation_rad, -math.pi / 2.0)
        self.assertAlmostEqual(reframed.movement[0], 0.0, places=7)
        self.assertAlmostEqual(reframed.movement[1], -1.0, places=7)
        self.assertAlmostEqual(reframed.facing[0], 1.0, places=7)
        self.assertAlmostEqual(reframed.facing[1], 0.0, places=7)
        self.assertEqual(reframed.speed_mps, command.speed_mps)

        bootstrap = coordinator.sonic_bootstrap_command(command)
        self.assertEqual(bootstrap.movement, (0.0, 0.0, 0.0))
        self.assertEqual(bootstrap.facing, (1.0, 0.0, 0.0))
        self.assertEqual(bootstrap.locomotion_mode, GAME_CONTROL.SONIC_IDLE_MODE)
        self.assertTrue(bootstrap.safe_stop)

        # Even an external moving candidate is replaced by the exact IDLE
        # bootstrap while WAIT_NEUTRAL has not observed policy full control.
        coordinator.sonic_writer = SimpleNamespace(
            reentry_policy_full_control=False
        )
        waiting = MODULE.RecoveryOutput(
            previous_state=MODULE.RecoveryState.WAIT_NEUTRAL,
            state=MODULE.RecoveryState.WAIT_NEUTRAL,
            inhibit_game_input=True,
        )
        gated = coordinator.recovery_wire_command(
            command,
            waiting,
            measured_heading_rad=math.pi / 2.0,
            dt_s=0.02,
        )
        self.assertEqual(gated, bootstrap)

    def test_resident_recovery_holds_sonic_at_live_body_heading(self) -> None:
        coordinator = MODULE._PhysicalRecoveryCoordinator.__new__(
            MODULE._PhysicalRecoveryCoordinator
        )
        coordinator.command_frame_rotation_rad = 0.0
        coordinator.last_wire_facing_heading_rad = None
        command = GAME_CONTROL.RobotMotionCommand(
            sequence=9,
            movement=(1.0, 0.0, 0.0),
            facing=(0.0, 1.0, 0.0),
            speed_mps=1.0,
            locomotion_mode=GAME_CONTROL.SONIC_RUN_MODE,
            mode="move",
            safe_stop=False,
            reason="active",
        )
        output = MODULE.ResidentRecoveryOutput(
            previous_state=MODULE.ResidentRecoveryState.KUNGFU_STARTING,
            state=MODULE.ResidentRecoveryState.KUNGFU_RECOVERING,
            inhibit_game_input=True,
        )
        measured_heading = math.radians(125.0)

        held = coordinator.recovery_wire_command(
            command,
            output,
            measured_heading_rad=measured_heading,
            dt_s=0.02,
        )

        self.assertEqual(held.movement, (0.0, 0.0, 0.0))
        self.assertEqual(held.speed_mps, 0.0)
        self.assertEqual(held.locomotion_mode, GAME_CONTROL.SONIC_IDLE_MODE)
        self.assertTrue(held.safe_stop)
        self.assertEqual(held.reason, "physical_recovery_resident_sonic_hold")
        self.assertAlmostEqual(held.facing[0], math.cos(measured_heading))
        self.assertAlmostEqual(held.facing[1], math.sin(measured_heading))
        self.assertAlmostEqual(
            coordinator.last_wire_facing_heading_rad,
            measured_heading,
        )

    def test_policy_fallback_uses_full_pose_grace_and_low_dynamics(self) -> None:
        coordinator = MODULE._PhysicalRecoveryCoordinator.__new__(
            MODULE._PhysicalRecoveryCoordinator
        )
        coordinator.worker = mock.Mock()
        coordinator.worker.fallback_due = True
        coordinator.worker.connection = object()
        coordinator.worker.go_sent = True
        coordinator.worker.stop_sent = False
        coordinator.worker.amp_hold_sent = False
        coordinator.worker.joint_hold_sent = False
        coordinator.policy_fallback_last_near_upright_s = None
        coordinator.policy_fallback_quiet_since_s = None
        coordinator.policy_advance_requested = False
        coordinator.policy_advances = 0

        def evaluate(
            now_s,
            *,
            root_z,
            root_up,
            linear=0.0,
            angular=0.0,
            joint_rms=0.0,
        ):
            coordinator._maybe_authorize_policy_advance(
                now_s=now_s,
                root_z_m=root_z,
                root_up_z=root_up,
                root_linear_speed_m_s=linear,
                root_angular_speed_rad_s=angular,
                joint_velocity_rms_rad_s=joint_rms,
                grounded_contact=True,
                recovery_state=MODULE.RecoveryState.POLICY_RECOVERING,
                policy_alive=True,
                worker_controller="HOST_GETUP",
            )

        # True standing uses both height and orientation, so a policy that has
        # just reached z=.75/up=.99 is not replaced at its timeout.
        evaluate(10.0, root_z=0.75, root_up=0.99)
        coordinator.worker.send.assert_not_called()

        # Falling out of upright grace or moving rapidly still cannot trigger
        # a hard policy switch.
        evaluate(12.1, root_z=0.25, root_up=0.2, angular=3.0, joint_rms=2.0)
        evaluate(12.2, root_z=0.25, root_up=0.2)
        coordinator.worker.send.assert_not_called()

        # Only a continuous grounded low-energy window authorizes the next
        # already-loaded policy.
        evaluate(12.71, root_z=0.25, root_up=0.2)
        coordinator.worker.send.assert_called_once_with("ADVANCE_POLICY")
        self.assertTrue(coordinator.policy_advance_requested)
        self.assertEqual(coordinator.policy_advances, 1)

    def test_low_supine_pose_is_not_mistaken_for_upright(self) -> None:
        coordinator = MODULE._PhysicalRecoveryCoordinator.__new__(
            MODULE._PhysicalRecoveryCoordinator
        )
        coordinator.worker = mock.Mock()
        coordinator.worker.fallback_due = True
        coordinator.worker.connection = object()
        coordinator.worker.go_sent = True
        coordinator.worker.stop_sent = False
        coordinator.worker.amp_hold_sent = False
        coordinator.worker.joint_hold_sent = False
        coordinator.policy_fallback_last_near_upright_s = None
        coordinator.policy_fallback_quiet_since_s = None
        coordinator.policy_advance_requested = False
        coordinator.policy_advances = 0
        arguments = {
            "root_z_m": 0.12,
            "root_up_z": 0.99,
            "root_linear_speed_m_s": 0.0,
            "root_angular_speed_rad_s": 0.0,
            "joint_velocity_rms_rad_s": 0.0,
            "grounded_contact": True,
            "recovery_state": MODULE.RecoveryState.POLICY_RECOVERING,
            "policy_alive": True,
            "worker_controller": "HOST_GETUP",
        }

        coordinator._maybe_authorize_policy_advance(now_s=20.0, **arguments)
        coordinator._maybe_authorize_policy_advance(now_s=20.51, **arguments)

        coordinator.worker.send.assert_called_once_with("ADVANCE_POLICY")

    def test_initial_sonic_uses_writer_gate_and_goes_only_after_ready(self) -> None:
        coordinator = MODULE._PhysicalRecoveryCoordinator.__new__(
            MODULE._PhysicalRecoveryCoordinator
        )
        coordinator.sonic_writer = mock.Mock()
        coordinator.sonic_writer.path = Path("/run/user/1000/initial-sonic.sock")
        coordinator.sonic_writer.ready = False
        coordinator.sonic_writer.writer_created = False
        coordinator.sonic_writer.first_write = False
        coordinator.initial_sonic_gate_pending = False
        processes = mock.Mock()
        processes.deploy_pid.return_value = 4321
        planner = mock.Mock()
        output = MODULE.RecoveryOutput(
            previous_state=MODULE.RecoveryState.GAME_SONIC,
            state=MODULE.RecoveryState.GAME_SONIC,
        )

        self.assertEqual(
            coordinator.prepare_initial_sonic_gate(),
            Path("/run/user/1000/initial-sonic.sock"),
        )
        coordinator.bind_initial_sonic_gate(processes=processes)
        coordinator.execute(output, processes=processes, planner=planner)
        coordinator.sonic_writer.send.assert_not_called()

        coordinator.sonic_writer.ready = True
        coordinator.execute(output, processes=processes, planner=planner)
        coordinator.sonic_writer.send.assert_called_once_with("GO")
        self.assertFalse(coordinator.initial_sonic_gate_pending)
        coordinator.sonic_writer.reset_for_start.assert_called_once_with()
        coordinator.sonic_writer.bind_expected_peer_pid.assert_called_once_with(4321)

    def test_sonic_stop_execute_is_idempotent_for_consecutive_outputs(self) -> None:
        coordinator = MODULE._PhysicalRecoveryCoordinator.__new__(
            MODULE._PhysicalRecoveryCoordinator
        )
        coordinator.initial_sonic_gate_pending = False
        writer = MODULE._SonicWriterControl(Path("/unused/sonic-stop.sock"))
        writer.reset_for_start()
        writer.expected_peer_pid = 4321
        writer.connection = mock.Mock()
        writer.connection.send.return_value = len(b"STOP")
        coordinator.sonic_writer = writer
        processes = mock.Mock()
        planner = mock.Mock()
        output = MODULE.RecoveryOutput(
            previous_state=MODULE.RecoveryState.SONIC_STOP_REQUESTED,
            state=MODULE.RecoveryState.FAILED,
            request_sonic_stop=True,
            fail_closed=True,
            failure_reason="timeout_sonic_stop_requested",
        )

        coordinator.execute(output, processes=processes, planner=planner)
        coordinator.execute(output, processes=processes, planner=planner)

        self.assertTrue(writer.stop_sent)
        writer.connection.send.assert_called_once_with(b"STOP")
        self.assertEqual(processes.begin_deploy_stop.call_count, 2)
        planner.request_deploy_stop.assert_not_called()
        with self.assertRaisesRegex(RuntimeError, "STOP was already sent"):
            writer.send("STOP")

    def test_sonic_stop_timeout_failure_does_not_resend_protocol_stop(self) -> None:
        machine = MODULE.SingleWriterRecoveryFSM(
            MODULE.RecoveryConfig(sonic_stop_timeout_s=0.2)
        )
        observation = MODULE.RecoveryInput(
            now_s=0.0,
            fall_detected=False,
            root_z_m=0.30,
            root_up_z=0.20,
            root_linear_speed_m_s=0.0,
            root_angular_speed_rad_s=0.0,
            joint_velocity_rms_rad_s=0.0,
            lowcmd_fresh=True,
            lowcmd_age_s=0.01,
            deploy_alive=True,
            deploy_generation=7,
            deploy_process_ready=False,
            deploy_writer_ready=False,
            deploy_writer_created=True,
            deploy_writer_revoked=False,
            deploy_first_write=False,
            deploy_policy_full_control=False,
            deploy_safe_idle_hold=False,
            policy_alive=False,
            policy_ready=False,
            policy_first_write=False,
            policy_hold_first_write=False,
            reset_count=0,
            foot_contact=True,
            grounded_contact=True,
            neutral_confirmed=False,
        )
        machine.step(observation)
        first_stop = machine.step(
            replace(observation, now_s=0.1, fall_detected=True)
        )
        self.assertTrue(first_stop.request_sonic_stop)

        coordinator = MODULE._PhysicalRecoveryCoordinator.__new__(
            MODULE._PhysicalRecoveryCoordinator
        )
        coordinator.initial_sonic_gate_pending = False
        writer = MODULE._SonicWriterControl(Path("/unused/sonic-timeout.sock"))
        writer.reset_for_start()
        writer.expected_peer_pid = 4321
        writer.connection = mock.Mock()
        writer.connection.send.return_value = len(b"STOP")
        coordinator.sonic_writer = writer
        processes = mock.Mock()
        planner = mock.Mock()

        coordinator.execute(first_stop, processes=processes, planner=planner)
        timed_out = machine.step(
            replace(observation, now_s=0.31, fall_detected=True)
        )
        self.assertEqual(timed_out.state, MODULE.RecoveryState.FAILED)
        self.assertEqual(
            timed_out.failure_reason, "timeout_sonic_stop_requested"
        )
        self.assertTrue(timed_out.request_sonic_stop)
        coordinator.execute(timed_out, processes=processes, planner=planner)

        writer.connection.send.assert_called_once_with(b"STOP")
        self.assertEqual(processes.begin_deploy_stop.call_count, 2)
        planner.request_deploy_stop.assert_not_called()

    def test_policy_stop_marks_expected_exit_before_protocol_stop(self) -> None:
        coordinator = MODULE._PhysicalRecoveryCoordinator.__new__(
            MODULE._PhysicalRecoveryCoordinator
        )
        coordinator.initial_sonic_gate_pending = False
        coordinator.worker = mock.Mock()
        coordinator.worker.connection = object()
        processes = mock.Mock()
        output = MODULE.RecoveryOutput(
            previous_state=MODULE.RecoveryState.POLICY_AMP_HOLDING,
            state=MODULE.RecoveryState.POLICY_STOP_REQUESTED,
            request_policy_stop=True,
        )

        coordinator.execute(output, processes=processes, planner=mock.Mock())

        coordinator.worker.send.assert_called_once_with("STOP")
        processes.begin_recovery_policy_stop.assert_called_once_with()

    def test_disconnected_policy_stop_marks_expected_exit(self) -> None:
        coordinator = MODULE._PhysicalRecoveryCoordinator.__new__(
            MODULE._PhysicalRecoveryCoordinator
        )
        coordinator.initial_sonic_gate_pending = False
        coordinator.worker = mock.Mock()
        coordinator.worker.connection = None
        processes = mock.Mock()
        output = MODULE.RecoveryOutput(
            previous_state=MODULE.RecoveryState.POLICY_AMP_HOLDING,
            state=MODULE.RecoveryState.POLICY_STOP_REQUESTED,
            request_policy_stop=True,
        )

        coordinator.execute(output, processes=processes, planner=mock.Mock())

        coordinator.worker.send.assert_not_called()
        processes.begin_recovery_policy_stop.assert_called_once_with()

    def test_writer_free_prewarm_start_stays_in_policy_phase(self) -> None:
        coordinator = MODULE._PhysicalRecoveryCoordinator.__new__(
            MODULE._PhysicalRecoveryCoordinator
        )
        coordinator.sonic_writer = SimpleNamespace(
            writer_created=False,
            first_write=False,
        )
        processes = SimpleNamespace(
            deploy_generation=8,
            deploy_alive=lambda: True,
        )
        output = MODULE.RecoveryOutput(
            previous_state=MODULE.RecoveryState.POLICY_RECOVERING,
            state=MODULE.RecoveryState.POLICY_RECOVERING,
            inhibit_game_input=True,
        )

        coordinator.verify_writer_free_prewarm_start(
            output,
            processes=processes,
            previous_generation=7,
        )

        for bad_output, bad_processes, writer_created in (
            (output, SimpleNamespace(deploy_generation=7, deploy_alive=lambda: True), False),
            (output, SimpleNamespace(deploy_generation=8, deploy_alive=lambda: False), False),
            (
                MODULE.RecoveryOutput(
                    previous_state=MODULE.RecoveryState.POLICY_RECOVERING,
                    state=MODULE.RecoveryState.SONIC_STABILIZING,
                    inhibit_game_input=True,
                ),
                processes,
                False,
            ),
            (output, processes, True),
        ):
            coordinator.sonic_writer.writer_created = writer_created
            with self.subTest(
                state=bad_output.state,
                generation=bad_processes.deploy_generation,
                alive=bad_processes.deploy_alive(),
                writer_created=writer_created,
            ), self.assertRaises(RuntimeError):
                coordinator.verify_writer_free_prewarm_start(
                    bad_output,
                    processes=bad_processes,
                    previous_generation=7,
                )

    def test_replacement_sonic_uses_measured_pose_writer_reentry(self) -> None:
        coordinator = MODULE._PhysicalRecoveryCoordinator.__new__(
            MODULE._PhysicalRecoveryCoordinator
        )
        coordinator.initial_sonic_gate_pending = False
        coordinator.zmq_port = 5556
        coordinator.interface = "lo"
        coordinator.worker = mock.Mock()
        coordinator.sonic_writer = mock.Mock()
        coordinator.sonic_writer.path = Path("/run/user/1000/replacement.sock")
        processes = mock.Mock()
        processes.deploy_pid.return_value = 4321
        output = MODULE.RecoveryOutput(
            previous_state=MODULE.RecoveryState.POLICY_RECOVERING,
            state=MODULE.RecoveryState.POLICY_RECOVERING,
            start_sonic=True,
            inhibit_game_input=True,
        )

        coordinator.execute(output, processes=processes, planner=mock.Mock())

        processes.start_deploy.assert_called_once_with(
            interface="lo",
            zmq_port=5556,
            writer_control_socket=Path("/run/user/1000/replacement.sock"),
            physical_reentry=True,
        )
        coordinator.sonic_writer.bind_expected_peer_pid.assert_called_once_with(4321)

    @mock.patch.object(MODULE.subprocess, "Popen")
    def test_native_process_group_starts_the_exact_game_input_adapter(self, popen) -> None:
        process = mock.Mock()
        process.pid = 4243
        popen.return_value = process
        group = MODULE.NativeProcessGroup(Path("/sonic"), {})
        command_parent, command_child = socket.socketpair(
            socket.AF_UNIX,
            socket.SOCK_SEQPACKET,
        )
        command_fd = command_child.fileno()
        try:
            provider_pid = group.start_game_input(
                "/runtime/python",
                Path("/matrix/scripts/matrix_game_control_input.py"),
                input_socket=Path("/run/user/1000/matrix-game.sock"),
                input_source="auto",
                camera_yaw_source="x11-mirror",
                look_button="left",
                initial_camera_yaw_deg=5.0,
                mouse_sensitivity_deg=0.12,
                mouse_settings_file=Path(
                    "/home/user/.config/matrix/mouse-control.json"
                ),
                applied_mouse_profile="remote",
                applied_mouse_speed_scale=0.5,
                restart_request_file=Path("/run/user/1000/matrix/restart.json"),
                restart_capability_file=Path("/run/user/1000/matrix/capability"),
                restart_launcher_pid=4000,
                camera_yaw_sign=-1,
                camera_yaw_offset_deg=90.0,
                carla_host="127.0.0.2",
                carla_port=2100,
                gamepad_look_yaw_rate_deg_s=140.0,
                gamepad_look_pitch_rate_deg_s=95.0,
                gamepad_look_deadzone=0.13,
                gamepad_look_min_pitch_deg=-70.0,
                gamepad_look_max_pitch_deg=50.0,
                focus_title="matrix",
                expected_ue_pid=4242,
                status_file=Path("/matrix/outputs/game-input.json"),
                command_fd=command_fd,
            )
        finally:
            command_parent.close()
            command_child.close()

        guarded = popen.call_args.args[0]
        self.assertIn("--exec-command", guarded[: guarded.index("--")])
        self.assertEqual(provider_pid, 4243)
        command = guarded[guarded.index("--") + 1 :]
        self.assertEqual(command[:3], [
            "/runtime/python",
            "-u",
            "/matrix/scripts/matrix_game_control_input.py",
        ])
        self.assertEqual(
            command[command.index("--socket") + 1],
            "/run/user/1000/matrix-game.sock",
        )
        self.assertEqual(
            command[command.index("--camera-yaw-source") + 1], "x11-mirror"
        )
        self.assertEqual(command[command.index("--camera-yaw-sign") + 1], "-1")
        self.assertEqual(
            command[command.index("--camera-yaw-offset-deg") + 1], "90.0"
        )
        self.assertEqual(command[command.index("--expected-ue-pid") + 1], "4242")
        self.assertEqual(
            command[command.index("--game-command-fd") + 1],
            str(command_fd),
        )
        self.assertNotIn("--ue-camera-state-file", command)
        self.assertEqual(
            command[command.index("--mouse-settings-file") + 1],
            "/home/user/.config/matrix/mouse-control.json",
        )
        self.assertEqual(
            command[command.index("--applied-mouse-profile") + 1], "remote"
        )
        self.assertEqual(
            command[command.index("--applied-mouse-speed-scale") + 1], "0.5"
        )
        self.assertEqual(
            command[command.index("--restart-launcher-pid") + 1], "4000"
        )
        self.assertEqual(command[command.index("--carla-host") + 1], "127.0.0.2")
        self.assertEqual(command[command.index("--carla-port") + 1], "2100")
        self.assertEqual(
            command[command.index("--gamepad-look-yaw-rate-deg-s") + 1],
            "140.0",
        )
        self.assertEqual(
            command[command.index("--gamepad-look-pitch-rate-deg-s") + 1],
            "95.0",
        )
        self.assertEqual(
            command[command.index("--gamepad-look-deadzone") + 1], "0.13"
        )
        self.assertEqual(
            command[command.index("--gamepad-look-min-pitch-deg") + 1],
            "-70.0",
        )
        self.assertEqual(
            command[command.index("--gamepad-look-max-pitch-deg") + 1],
            "50.0",
        )
        self.assertEqual(popen.call_args.kwargs["cwd"], Path("/matrix"))
        self.assertEqual(popen.call_args.kwargs["pass_fds"], (command_fd,))

    @mock.patch.object(MODULE.subprocess, "Popen")
    def test_native_process_group_forwards_final_pov_state_file(self, popen) -> None:
        process = mock.Mock()
        process.pid = 4243
        popen.return_value = process
        group = MODULE.NativeProcessGroup(Path("/sonic"), {})
        group.start_game_input(
            "/runtime/python",
            Path("/matrix/scripts/matrix_game_control_input.py"),
            input_socket=Path("/run/user/1000/matrix-game.sock"),
            input_source="keyboard",
            camera_yaw_source="ue-final-pov",
            ue_camera_state_file=Path("/run/user/1000/camera-state.bin"),
            look_button="left",
            initial_camera_yaw_deg=0.0,
            mouse_sensitivity_deg=0.12,
            camera_yaw_sign=-1,
            camera_yaw_offset_deg=0.0,
            carla_host="127.0.0.1",
            carla_port=2000,
            gamepad_look_yaw_rate_deg_s=120.0,
            gamepad_look_pitch_rate_deg_s=90.0,
            gamepad_look_deadzone=0.12,
            gamepad_look_min_pitch_deg=-80.0,
            gamepad_look_max_pitch_deg=60.0,
            focus_title="matrix",
            expected_ue_pid=4242,
            status_file=None,
        )

        guarded = popen.call_args.args[0]
        command = guarded[guarded.index("--") + 1 :]
        self.assertEqual(
            command[command.index("--camera-yaw-source") + 1],
            "ue-final-pov",
        )
        self.assertEqual(
            command[command.index("--ue-camera-state-file") + 1],
            "/run/user/1000/camera-state.bin",
        )

    def test_process_group_prepends_sonic_to_existing_pythonpath(self) -> None:
        group = MODULE.NativeProcessGroup(
            Path("/sonic"), {"PYTHONPATH": "/locked/site"}
        )
        self.assertEqual(
            group.env["PYTHONPATH"],
            f"/sonic{MODULE.os.pathsep}/locked/site",
        )

    def test_process_group_passes_the_exact_host_lock_to_guardian(self) -> None:
        with tempfile.TemporaryFile() as lock_stream, mock.patch.object(
            MODULE.subprocess, "Popen"
        ) as popen:
            lock_fd = lock_stream.fileno()
            group = MODULE.NativeProcessGroup(
                Path("/sonic"),
                {"MATRIX_SONIC_HOST_LOCK_FD": str(lock_fd)},
            )
            group.start_deploy(interface="lo", zmq_port=6000)
            self.assertEqual(popen.call_args.kwargs["pass_fds"], (lock_fd,))

    @mock.patch.object(MODULE.time, "sleep")
    @mock.patch.object(MODULE, "_peek_child_returncode", side_effect=[None, 0])
    def test_native_deploy_gets_a_graceful_stop_window(
        self, _peek, sleep
    ) -> None:
        process = mock.Mock()
        group = MODULE.NativeProcessGroup(Path("/sonic"), {})
        group.children.append(("deploy", process))

        self.assertTrue(group.wait_for_child("deploy", timeout=2.0))
        sleep.assert_called_once()

    @mock.patch.object(MODULE.subprocess, "Popen")
    def test_pico_uses_its_locked_python_and_planner_port(self, popen) -> None:
        popen.return_value = mock.Mock()
        group = MODULE.NativeProcessGroup(Path("/sonic"), {})
        group.start_pico("/pico/bin/python", port=6000)
        guarded_command = popen.call_args.args[0]
        command = guarded_command[guarded_command.index("--") + 1 :]
        self.assertEqual(command[0], "/pico/bin/python")
        self.assertEqual(command[1], "-u")
        self.assertEqual(command[command.index("--port") + 1], "6000")

    def test_parent_death_guardian_kills_native_process_group(self) -> None:
        guardian = REPO_ROOT / "scripts/exec_with_parent_death_signal.py"
        child_code = "\n".join(
            (
                "import os",
                "from pathlib import Path",
                "import subprocess",
                "import sys",
                "import time",
                "grandchild = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])",
                "Path(sys.argv[1]).write_text(f'{os.getpid()} {grandchild.pid}', encoding='utf-8')",
                "time.sleep(60)",
            )
        )
        supervisor_code = "\n".join(
            (
                "import os",
                "import subprocess",
                "import sys",
                "import time",
                "process = subprocess.Popen([sys.executable, sys.argv[1], '--expected-parent', str(os.getpid()), '--', sys.executable, '-c', sys.argv[2], sys.argv[3]], start_new_session=True)",
                "print(process.pid, flush=True)",
                "time.sleep(60)",
            )
        )

        with tempfile.TemporaryDirectory() as temporary:
            pid_file = Path(temporary) / "native-pids"
            supervisor = subprocess.Popen(
                [
                    sys.executable,
                    "-c",
                    supervisor_code,
                    str(guardian),
                    child_code,
                    str(pid_file),
                ],
                stdout=subprocess.PIPE,
                text=True,
            )
            assert supervisor.stdout is not None
            group_id = int(supervisor.stdout.readline().strip())
            try:
                deadline = time.monotonic() + 5.0
                while not pid_file.is_file() and time.monotonic() < deadline:
                    time.sleep(0.02)
                self.assertTrue(pid_file.is_file(), "guarded child did not start")
                native_pids = [
                    int(value) for value in pid_file.read_text(encoding="utf-8").split()
                ]

                os.kill(supervisor.pid, signal.SIGKILL)
                supervisor.wait(timeout=5.0)
                deadline = time.monotonic() + 5.0
                while (
                    any(self.process_is_running(pid) for pid in native_pids)
                    and time.monotonic() < deadline
                ):
                    time.sleep(0.05)
                self.assertFalse(
                    any(self.process_is_running(pid) for pid in native_pids),
                    f"native process group survived supervisor death: {native_pids}",
                )
            finally:
                if supervisor.poll() is None:
                    supervisor.kill()
                    supervisor.wait(timeout=5.0)
                supervisor.stdout.close()
                try:
                    os.killpg(group_id, signal.SIGKILL)
                except ProcessLookupError:
                    pass

    def test_parent_death_guardian_exec_mode_preserves_leaf_pid(self) -> None:
        guardian = REPO_ROOT / "scripts/exec_with_parent_death_signal.py"
        process = subprocess.Popen(
            [
                sys.executable,
                os.fspath(guardian),
                "--expected-parent",
                str(os.getpid()),
                "--exec-command",
                "--",
                sys.executable,
                "-c",
                "import os; print(os.getpid(), flush=True)",
            ],
            stdout=subprocess.PIPE,
            text=True,
            start_new_session=True,
        )
        assert process.stdout is not None
        try:
            executed_pid = int(process.stdout.readline().strip())
            self.assertEqual(executed_pid, process.pid)
            self.assertEqual(process.wait(timeout=5.0), 0)
        finally:
            process.stdout.close()
            if process.poll() is None:
                process.kill()
                process.wait(timeout=5.0)

    def test_parent_death_guardian_exec_mode_hard_kills_stuck_leaf(self) -> None:
        guardian = REPO_ROOT / "scripts/exec_with_parent_death_signal.py"
        leaf_code = "\n".join(
            (
                "import os",
                "from pathlib import Path",
                "import signal",
                "import sys",
                "import time",
                "signal.signal(signal.SIGTERM, lambda *_args: None)",
                "Path(sys.argv[1]).write_text(str(os.getpid()), encoding='utf-8')",
                "time.sleep(60)",
            )
        )
        supervisor_code = "\n".join(
            (
                "import os",
                "import subprocess",
                "import sys",
                "import time",
                "process = subprocess.Popen([sys.executable, sys.argv[1], '--expected-parent', str(os.getpid()), '--exec-command', '--', sys.executable, '-c', sys.argv[2], sys.argv[3]], start_new_session=True)",
                "print(process.pid, flush=True)",
                "time.sleep(60)",
            )
        )

        with tempfile.TemporaryDirectory() as temporary:
            pid_file = Path(temporary) / "leaf-pid"
            supervisor = subprocess.Popen(
                [
                    sys.executable,
                    "-c",
                    supervisor_code,
                    os.fspath(guardian),
                    leaf_code,
                    os.fspath(pid_file),
                ],
                stdout=subprocess.PIPE,
                text=True,
            )
            assert supervisor.stdout is not None
            leaf_pid = int(supervisor.stdout.readline().strip())
            try:
                deadline = time.monotonic() + 5.0
                while not pid_file.is_file() and time.monotonic() < deadline:
                    time.sleep(0.02)
                self.assertTrue(pid_file.is_file(), "exec leaf did not start")
                self.assertEqual(int(pid_file.read_text(encoding="utf-8")), leaf_pid)

                os.kill(supervisor.pid, signal.SIGKILL)
                supervisor.wait(timeout=5.0)
                deadline = time.monotonic() + 5.0
                while self.process_is_running(leaf_pid) and time.monotonic() < deadline:
                    time.sleep(0.05)
                self.assertFalse(
                    self.process_is_running(leaf_pid),
                    f"exec leaf survived supervisor death: {leaf_pid}",
                )
            finally:
                if supervisor.poll() is None:
                    supervisor.kill()
                    supervisor.wait(timeout=5.0)
                supervisor.stdout.close()
                try:
                    os.killpg(leaf_pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass

    def test_supervisor_receives_signal_when_run_sim_parent_is_sigkilled(self) -> None:
        child_code = "\n".join(
            (
                "import importlib.util",
                "import os",
                "from pathlib import Path",
                "import sys",
                "import time",
                "script = Path(sys.argv[1]).resolve()",
                "sys.path.insert(0, str(script.parent))",
                "spec = importlib.util.spec_from_file_location('guarded_runner', script)",
                "module = importlib.util.module_from_spec(spec)",
                "spec.loader.exec_module(module)",
                "module._arm_supervisor_parent_death(os.getppid())",
                "Path(sys.argv[2]).write_text(str(os.getpid()), encoding='utf-8')",
                "time.sleep(60)",
            )
        )
        parent_code = "\n".join(
            (
                "import subprocess",
                "import sys",
                "import time",
                "child = subprocess.Popen([sys.executable, '-c', sys.argv[1], sys.argv[2], sys.argv[3]])",
                "print(child.pid, flush=True)",
                "time.sleep(60)",
            )
        )

        with tempfile.TemporaryDirectory() as temporary:
            pid_file = Path(temporary) / "supervisor-pid"
            parent = subprocess.Popen(
                [
                    sys.executable,
                    "-c",
                    parent_code,
                    child_code,
                    str(SCRIPT_PATH),
                    str(pid_file),
                ],
                stdout=subprocess.PIPE,
                text=True,
            )
            assert parent.stdout is not None
            supervisor_pid = int(parent.stdout.readline().strip())
            try:
                deadline = time.monotonic() + 5.0
                while not pid_file.is_file() and time.monotonic() < deadline:
                    time.sleep(0.02)
                self.assertTrue(pid_file.is_file(), "supervisor did not arm PDEATHSIG")

                os.kill(parent.pid, signal.SIGKILL)
                parent.wait(timeout=5.0)
                deadline = time.monotonic() + 5.0
                while (
                    self.process_is_running(supervisor_pid)
                    and time.monotonic() < deadline
                ):
                    time.sleep(0.05)
                self.assertFalse(
                    self.process_is_running(supervisor_pid),
                    "supervisor survived run_sim parent death",
                )
            finally:
                if parent.poll() is None:
                    parent.kill()
                    parent.wait(timeout=5.0)
                parent.stdout.close()
                try:
                    os.kill(supervisor_pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass

    @mock.patch.object(MODULE, "_peek_child_returncode", return_value=7)
    @mock.patch.object(MODULE.os, "killpg")
    def test_process_group_close_signals_group_after_leader_exit(
        self, killpg, _peek
    ) -> None:
        process = mock.Mock(pid=4321)

        def signal_group(_process_group, signum):
            if signum == MODULE.signal.SIGKILL:
                return None

        killpg.side_effect = signal_group
        group = MODULE.NativeProcessGroup(Path("/sonic"), {})
        group.children.append(("deploy", process))

        group.close()

        self.assertIn(mock.call(4321, MODULE.signal.SIGTERM), killpg.call_args_list)
        self.assertIn(mock.call(4321, MODULE.signal.SIGKILL), killpg.call_args_list)
        process.wait.assert_called_once_with(timeout=2.0)

    def test_process_group_close_kills_group_before_exact_reap(self) -> None:
        events = []
        observed = iter((None, 0))
        process = mock.Mock(pid=4321)
        process.wait.side_effect = lambda **_kwargs: events.append("wait") or 0
        group = MODULE.NativeProcessGroup(Path("/sonic"), {})
        group.children.append(("deploy", process))

        def peek(_process):
            events.append("peek")
            return next(observed)

        def signal_group(_process_group, signum):
            events.append(
                "term" if signum == MODULE.signal.SIGTERM else "kill"
            )

        with (
            mock.patch.object(MODULE, "_peek_child_returncode", side_effect=peek),
            mock.patch.object(MODULE.os, "killpg", side_effect=signal_group),
        ):
            group.close()

        self.assertEqual(events, ["peek", "term", "peek", "kill", "wait"])

    @mock.patch.object(MODULE.time, "monotonic", side_effect=[0.0, 6.0])
    @mock.patch.object(MODULE, "_peek_child_returncode", return_value=None)
    @mock.patch.object(MODULE.os, "killpg")
    def test_process_group_close_reports_child_after_sigkill(
        self, _killpg, _peek, _monotonic
    ) -> None:
        process = mock.Mock(pid=4321)
        process.wait.side_effect = subprocess.TimeoutExpired("child", 2.0)
        group = MODULE.NativeProcessGroup(Path("/sonic"), {})
        group.children.append(("deploy", process))

        with self.assertRaisesRegex(RuntimeError, "did not exit after SIGKILL"):
            group.close()

    def test_cleanup_failure_invalidates_written_status(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            status = Path(temporary) / "status.json"
            status.write_text(
                json.dumps(
                    {
                        "acceptance_failures": [],
                        "passed": True,
                        "termination_reason": "max_seconds",
                    }
                ),
                encoding="utf-8",
            )

            MODULE._record_cleanup_failure(status, ["native processes: alive"])

            payload = json.loads(status.read_text(encoding="utf-8"))
            self.assertFalse(payload["passed"])
            self.assertIn("cleanup_failure", payload["acceptance_failures"])
            self.assertEqual(payload["termination_reason"], "cleanup_failure")

    def test_process_group_boundary_observes_exit_without_reaping(self) -> None:
        for exit_code in (0, 42):
            with self.subTest(exit_code=exit_code):
                process = subprocess.Popen(
                    [sys.executable, "-c", f"raise SystemExit({exit_code})"],
                    start_new_session=True,
                )
                group = MODULE.NativeProcessGroup(Path("/sonic"), {})
                group.children.append(("deploy", process))
                deadline = time.monotonic() + 5.0
                while MODULE._peek_child_returncode(process) is None:
                    if time.monotonic() >= deadline:
                        self.fail("native child did not exit")
                    time.sleep(0.01)

                self.assertEqual(
                    group.begin_expected_stop(), ("deploy", exit_code)
                )
                self.assertIsNone(process.returncode)
                group.close()

    def test_process_group_boundary_authorizes_later_stop(self) -> None:
        process = subprocess.Popen(
            [sys.executable, "-c", "import time; time.sleep(60)"],
            start_new_session=True,
        )
        group = MODULE.NativeProcessGroup(Path("/sonic"), {})
        group.children.append(("deploy", process))
        try:
            self.assertIsNone(group.begin_expected_stop())
            self.assertIsNone(group.failed_child())
        finally:
            group.close()

    def test_process_group_close_kills_term_ignoring_descendant_before_reap(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            pid_file = Path(temporary) / "grandchild.pid"
            leader_code = "\n".join(
                (
                    "from pathlib import Path",
                    "import subprocess,sys,time",
                    "code='import signal,time; signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(60)'",
                    "child=subprocess.Popen([sys.executable, '-c', code])",
                    "Path(sys.argv[1]).write_text(str(child.pid), encoding='utf-8')",
                    "time.sleep(60)",
                )
            )
            process = subprocess.Popen(
                [sys.executable, "-c", leader_code, str(pid_file)],
                start_new_session=True,
            )
            deadline = time.monotonic() + 5.0
            while not pid_file.is_file():
                if time.monotonic() >= deadline:
                    process.kill()
                    process.wait(timeout=5.0)
                    self.fail("native grandchild pid was not published")
                time.sleep(0.01)
            grandchild_pid = int(pid_file.read_text(encoding="utf-8"))
            group = MODULE.NativeProcessGroup(Path("/sonic"), {})
            group.children.append(("deploy", process))

            self.assertIsNone(group.begin_expected_stop())
            group.close()

            deadline = time.monotonic() + 5.0
            while self.process_is_running(grandchild_pid) and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertFalse(self.process_is_running(grandchild_pid))

    def test_startup_failure_closes_simulator_and_started_children(self) -> None:
        events = []
        simulator = mock.Mock()
        simulator.get_state_snapshot.return_value = self.snapshot()
        simulator.close.side_effect = lambda: events.append("simulator-close")

        process_group = mock.Mock()
        process_group.failed_child.return_value = None
        process_group.start_pico.side_effect = lambda *_args, **_kwargs: events.append(
            "pico-start"
        )

        def fail_deploy(**_kwargs):
            events.append("deploy-start")
            raise RuntimeError("deploy failed")

        process_group.start_deploy.side_effect = fail_deploy
        process_group.close.side_effect = lambda: events.append("processes-close")

        fake_numpy = ModuleType("numpy")
        fake_numpy.float64 = float
        fake_numpy.asarray = lambda values, dtype=None: list(values)
        fake_zmq = ModuleType("zmq")
        run_sim_loop = ModuleType("gear_sonic.scripts.run_sim_loop")
        run_sim_loop.create_simulator = lambda _config: simulator
        configs = ModuleType("gear_sonic.utils.mujoco_sim.configs")
        configs.SimLoopConfig = lambda **kwargs: kwargs
        planner_sender = ModuleType(
            "gear_sonic.utils.teleop.zmq.zmq_planner_sender"
        )
        planner_sender.build_command_message = lambda **_kwargs: b"command"
        planner_sender.build_planner_message = lambda **_kwargs: b"planner"
        render_protocol = ModuleType("matrix_render_protocol")
        render_protocol.MatrixRenderPublisher = mock.Mock()
        render_protocol.packet_size = lambda **_kwargs: 0

        fake_modules = {
            "numpy": fake_numpy,
            "zmq": fake_zmq,
            "gear_sonic.scripts.run_sim_loop": run_sim_loop,
            "gear_sonic.utils.mujoco_sim.configs": configs,
            "gear_sonic.utils.teleop.zmq.zmq_planner_sender": planner_sender,
            "matrix_render_protocol": render_protocol,
        }
        for package_name in (
            "gear_sonic",
            "gear_sonic.scripts",
            "gear_sonic.utils",
            "gear_sonic.utils.mujoco_sim",
            "gear_sonic.utils.teleop",
            "gear_sonic.utils.teleop.zmq",
        ):
            package = ModuleType(package_name)
            package.__path__ = []
            fake_modules[package_name] = package

        args = SimpleNamespace(
            model=SCRIPT_PATH,
            sonic_root=Path("/sonic"),
            control_source="pico",
            planner_bind="tcp://127.0.0.1:5556",
            pico_python="/pico/bin/python",
            dds_interface="lo",
            render_host="127.0.0.1",
            render_port=9999,
            no_render_sync=True,
            physics_hz=200.0,
            control_hz=50.0,
            max_seconds=1.0,
            fail_on_fall=False,
            min_active_seconds=0.0,
            min_displacement_m=0.0,
            min_final_x=None,
            min_forward_x_m=0.0,
            low_cmd_fresh_timeout_seconds=0.1,
            min_physics_hz=0.0,
            min_rtf=0.0,
            max_resets=0,
            walk_after=-1.0,
            vx=0.3,
            vy=0.0,
            yaw_rate=0.0,
            status_file=None,
            qualified_runtime=False,
            qualification_profile=None,
            runtime_lock_sha256=None,
            scenario_layout_sha256=None,
            matrix_commit=None,
            verification_receipt=None,
            expected_parent_pid=None,
            external_failure_file=None,
            ue_pid=None,
            print_every=2.0,
            startup_band=False,
            startup_band_hold=4.0,
            startup_band_fade=3.0,
        )

        def record_signal(signum, handler):
            events.append(("signal", int(signum), handler))

        with (
            mock.patch.dict(MODULE.sys.modules, fake_modules),
            mock.patch.object(MODULE, "_parse_args", return_value=args),
            mock.patch.object(
                MODULE, "_configure_native_runtime", return_value=Path("/sonic")
            ),
            mock.patch.object(MODULE, "_sonic_commit", return_value="deadbeef"),
            mock.patch.object(
                MODULE, "NativeProcessGroup", return_value=process_group
            ),
            mock.patch.object(MODULE.signal, "getsignal", return_value="previous"),
            mock.patch.object(MODULE.signal, "signal", side_effect=record_signal),
        ):
            with self.assertRaisesRegex(RuntimeError, "deploy failed"):
                MODULE.main()

        self.assertEqual([event[0] for event in events[:2]], ["signal", "signal"])
        self.assertLess(events.index("pico-start"), events.index("deploy-start"))
        self.assertIn("processes-close", events)
        self.assertIn("simulator-close", events)

    def test_preexisting_ue_exit_zero_prevents_native_children(self) -> None:
        class FakeArray(list):
            def copy(self):
                return FakeArray(self)

            def __sub__(self, other):
                return FakeArray(
                    left - right for left, right in zip(self, other, strict=True)
                )

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            failure_file = root / "failure.json"
            failure_file.write_text(
                json.dumps({"name": "ue", "exit_code": 0}),
                encoding="utf-8",
            )
            status_file = root / "status.json"

            simulator = mock.Mock()
            simulator.get_state_snapshot.return_value = self.snapshot()
            process_group = mock.Mock()
            process_group.failed_child.return_value = None
            process_group.begin_expected_stop.return_value = None

            fake_numpy = ModuleType("numpy")
            fake_numpy.float64 = float
            fake_numpy.asarray = lambda values, dtype=None: FakeArray(values)
            fake_numpy.linalg = SimpleNamespace(
                norm=lambda values: math.sqrt(sum(value * value for value in values))
            )
            fake_zmq = ModuleType("zmq")
            run_sim_loop = ModuleType("gear_sonic.scripts.run_sim_loop")
            run_sim_loop.create_simulator = lambda _config: simulator
            configs = ModuleType("gear_sonic.utils.mujoco_sim.configs")
            configs.SimLoopConfig = lambda **kwargs: kwargs
            planner_sender = ModuleType(
                "gear_sonic.utils.teleop.zmq.zmq_planner_sender"
            )
            planner_sender.build_command_message = lambda **_kwargs: b"command"
            planner_sender.build_planner_message = lambda **_kwargs: b"planner"
            render_protocol = ModuleType("matrix_render_protocol")
            render_protocol.MatrixRenderPublisher = mock.Mock()
            render_protocol.packet_size = lambda **_kwargs: 0

            fake_modules = {
                "numpy": fake_numpy,
                "zmq": fake_zmq,
                "gear_sonic.scripts.run_sim_loop": run_sim_loop,
                "gear_sonic.utils.mujoco_sim.configs": configs,
                "gear_sonic.utils.teleop.zmq.zmq_planner_sender": planner_sender,
                "matrix_render_protocol": render_protocol,
            }
            for package_name in (
                "gear_sonic",
                "gear_sonic.scripts",
                "gear_sonic.utils",
                "gear_sonic.utils.mujoco_sim",
                "gear_sonic.utils.teleop",
                "gear_sonic.utils.teleop.zmq",
            ):
                package = ModuleType(package_name)
                package.__path__ = []
                fake_modules[package_name] = package

            args = SimpleNamespace(
                model=SCRIPT_PATH,
                sonic_root=Path("/sonic"),
                control_source="pico",
                planner_bind="tcp://127.0.0.1:5556",
                pico_python="/pico/bin/python",
                dds_interface="lo",
                render_host="127.0.0.1",
                render_port=9999,
                no_render_sync=True,
                physics_hz=200.0,
                control_hz=50.0,
                max_seconds=0.0,
                fail_on_fall=False,
                min_active_seconds=0.0,
                min_displacement_m=0.0,
                min_final_x=None,
                min_forward_x_m=0.0,
                low_cmd_fresh_timeout_seconds=0.1,
                min_physics_hz=0.0,
                min_rtf=0.0,
                max_resets=0,
                walk_after=-1.0,
                vx=0.3,
                vy=0.0,
                yaw_rate=0.0,
                status_file=status_file,
                qualified_runtime=False,
                qualification_profile=None,
                runtime_lock_sha256=None,
                scenario_layout_sha256=None,
                matrix_commit=None,
                verification_receipt=None,
                expected_parent_pid=None,
                external_failure_file=failure_file,
                ue_pid=4321,
                print_every=2.0,
                startup_band=False,
                startup_band_hold=4.0,
                startup_band_fade=3.0,
            )

            with (
                mock.patch.dict(MODULE.sys.modules, fake_modules),
                mock.patch.object(MODULE, "_parse_args", return_value=args),
                mock.patch.object(
                    MODULE, "_configure_native_runtime", return_value=Path("/sonic")
                ),
                mock.patch.object(MODULE, "_sonic_commit", return_value="deadbeef"),
                mock.patch.object(
                    MODULE, "NativeProcessGroup", return_value=process_group
                ),
                mock.patch.object(MODULE.signal, "getsignal", return_value="previous"),
                mock.patch.object(MODULE.signal, "signal"),
            ):
                result = MODULE.main()

            self.assertEqual(result, 2)
            process_group.start_pico.assert_not_called()
            process_group.start_deploy.assert_not_called()
            process_group.close.assert_called_once_with()
            simulator.close.assert_called_once_with()
            payload = json.loads(status_file.read_text(encoding="utf-8"))
            self.assertFalse(payload["passed"])
            self.assertFalse(payload["completed"])
            self.assertEqual(payload["failed_child_name"], "ue")
            self.assertEqual(payload["failed_child_exit_code"], 0)
            self.assertEqual(payload["termination_reason"], "child_exit")
            self.assertEqual(payload["ue_pid"], 4321)
            self.assertIn("native_child_exit:ue:0", payload["acceptance_failures"])

            # Reuse the complete main fixture to inject an exit precisely at
            # the authoritative native pre-stop boundary.
            failure_file.unlink()
            status_file.unlink()
            process_group.reset_mock()
            process_group.failed_child.return_value = None
            process_group.begin_expected_stop.return_value = ("deploy", 0)
            args.max_seconds = 1e-9
            with (
                mock.patch.dict(MODULE.sys.modules, fake_modules),
                mock.patch.object(MODULE, "_parse_args", return_value=args),
                mock.patch.object(
                    MODULE, "_configure_native_runtime", return_value=Path("/sonic")
                ),
                mock.patch.object(MODULE, "_sonic_commit", return_value="deadbeef"),
                mock.patch.object(
                    MODULE, "NativeProcessGroup", return_value=process_group
                ),
                mock.patch.object(MODULE.signal, "getsignal", return_value="previous"),
                mock.patch.object(MODULE.signal, "signal"),
            ):
                boundary_result = MODULE.main()

            self.assertEqual(boundary_result, 2)
            boundary_payload = json.loads(status_file.read_text(encoding="utf-8"))
            self.assertFalse(boundary_payload["passed"])
            self.assertFalse(boundary_payload["completed"])
            self.assertEqual(boundary_payload["failed_child_name"], "deploy")
            self.assertEqual(boundary_payload["failed_child_exit_code"], 0)
            self.assertEqual(boundary_payload["termination_reason"], "child_exit")
            self.assertIn(
                "native_child_exit:deploy:0",
                boundary_payload["acceptance_failures"],
            )

            # A harness-owned in-process completion is a clean scenario
            # boundary, not a signal and not a formal max-seconds qualification.
            status_file.unlink()
            process_group.reset_mock()
            simulator.reset_mock()
            simulator.get_state_snapshot.return_value = self.snapshot(
                low_cmd_fresh=True,
                low_cmd_received=True,
                low_cmd_age_s=0.01,
            )
            process_group.failed_child.return_value = None
            process_group.begin_expected_stop.return_value = None
            args.max_seconds = 360.0
            completion_event = MODULE.threading.Event()
            completion_event.set()
            with (
                mock.patch.dict(MODULE.sys.modules, fake_modules),
                mock.patch.object(MODULE, "_parse_args", return_value=args),
                mock.patch.object(
                    MODULE, "_configure_native_runtime", return_value=Path("/sonic")
                ),
                mock.patch.object(MODULE, "_sonic_commit", return_value="deadbeef"),
                mock.patch.object(
                    MODULE, "NativeProcessGroup", return_value=process_group
                ),
                mock.patch.object(MODULE.signal, "getsignal", return_value="previous"),
                mock.patch.object(MODULE.signal, "signal"),
            ):
                scenario_result = MODULE.main(completion_event=completion_event)

            self.assertEqual(scenario_result, 0)
            scenario_payload = json.loads(
                status_file.read_text(encoding="utf-8")
            )
            self.assertTrue(scenario_payload["passed"])
            self.assertTrue(scenario_payload["completed"])
            self.assertFalse(scenario_payload["qualification_attempted"])
            self.assertFalse(scenario_payload["interrupted"])
            self.assertEqual(
                scenario_payload["termination_reason"], "scenario_complete"
            )
            self.assertEqual(scenario_payload["acceptance_failures"], [])

            # The outer runtime deadline is authoritative when it and the
            # harness event are both ready at the same loop boundary.
            status_file.unlink()
            process_group.reset_mock()
            simulator.reset_mock()
            simulator.get_state_snapshot.return_value = self.snapshot(
                low_cmd_fresh=True,
                low_cmd_received=True,
                low_cmd_age_s=0.01,
            )
            process_group.failed_child.return_value = None
            process_group.begin_expected_stop.return_value = None
            args.max_seconds = 1e-12
            with (
                mock.patch.dict(MODULE.sys.modules, fake_modules),
                mock.patch.object(MODULE, "_parse_args", return_value=args),
                mock.patch.object(
                    MODULE, "_configure_native_runtime", return_value=Path("/sonic")
                ),
                mock.patch.object(MODULE, "_sonic_commit", return_value="deadbeef"),
                mock.patch.object(
                    MODULE, "NativeProcessGroup", return_value=process_group
                ),
                mock.patch.object(MODULE.signal, "getsignal", return_value="previous"),
                mock.patch.object(MODULE.signal, "signal"),
            ):
                deadline_result = MODULE.main(completion_event=completion_event)

            self.assertEqual(deadline_result, 2)
            deadline_payload = json.loads(status_file.read_text(encoding="utf-8"))
            self.assertEqual(deadline_payload["termination_reason"], "max_seconds")
            self.assertTrue(deadline_payload["qualification_attempted"])
            self.assertIn(
                "runtime_not_verified_for_qualification",
                deadline_payload["acceptance_failures"],
            )


if __name__ == "__main__":
    unittest.main()
