from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = REPO_ROOT / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
SCRIPT = SCRIPT_DIR / "matrix_bfm_teacher_adapter.py"
SPEC = importlib.util.spec_from_file_location(
    "matrix_bfm_teacher_adapter",
    SCRIPT,
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class MatrixBfmTeacherAdapterTest(unittest.TestCase):
    @staticmethod
    def core():
        core = MODULE.BfmTeacherCore.__new__(MODULE.BfmTeacherCore)
        core.command_module = SimpleNamespace(
            CommandSample=lambda **fields: SimpleNamespace(**fields)
        )
        return core

    @staticmethod
    def sample(*, safe_stop: bool, mode: str):
        return SimpleNamespace(
            movement=np.asarray((1.0, 0.0, 0.0), dtype=np.float64),
            facing=np.asarray((0.0, 1.0, 0.0), dtype=np.float64),
            desired_facing=np.asarray((0.0, 1.0, 0.0), dtype=np.float64),
            root_yaw=0.0,
            speed_mps=0.8,
            locomotion_mode=2,
            mode=mode,
            safe_stop=safe_stop,
        )

    def test_safe_stop_is_zero_velocity_zero_yaw_stand(self) -> None:
        command = self.core()._command(
            self.sample(safe_stop=True, mode="deadman")
        )

        self.assertEqual(command.vx, 0.0)
        self.assertEqual(command.vy, 0.0)
        self.assertEqual(command.yaw_rate, 0.0)
        self.assertEqual(command.gait, "stand")
        self.assertTrue(command.stop_latched)

    def test_turn_command_keeps_heading_control(self) -> None:
        sample = self.sample(safe_stop=False, mode="turn")
        sample.movement = np.zeros(3, dtype=np.float64)
        sample.speed_mps = 0.0
        command = self.core()._command(sample)

        self.assertEqual(
            command.vx,
            MODULE.TURN_REFERENCE_FORWARD_MPS,
        )
        self.assertEqual(
            command.vy,
            0.0,
        )
        self.assertGreater(command.yaw_rate, 0.0)
        self.assertEqual(command.gait, "walk")
        self.assertFalse(command.stop_latched)

    def test_turn_reference_seed_does_not_activate_lateral_gait(self) -> None:
        sample = self.sample(safe_stop=False, mode="turn")
        sample.movement = np.zeros(3, dtype=np.float64)
        sample.facing = np.asarray((0.0, -1.0, 0.0), dtype=np.float64)
        sample.desired_facing = sample.facing.copy()
        sample.speed_mps = 0.0

        command = self.core()._command(sample)

        self.assertEqual(
            command.vx,
            MODULE.TURN_REFERENCE_FORWARD_MPS,
        )
        self.assertEqual(
            command.vy,
            0.0,
        )
        self.assertLess(command.yaw_rate, 0.0)
        self.assertEqual(command.gait, "walk")

    def test_turn_yaw_uses_damped_matrix_wire_facing(self) -> None:
        sample = self.sample(safe_stop=False, mode="turn")
        sample.movement = np.zeros(3, dtype=np.float64)
        sample.facing = np.asarray(
            (math.cos(0.15), math.sin(0.15), 0.0),
            dtype=np.float64,
        )
        # The final camera facing is intentionally farther away; the adapter
        # must consume Matrix's already bounded wire-facing vector.
        sample.desired_facing = np.asarray((0.0, 1.0, 0.0), dtype=np.float64)
        sample.speed_mps = 0.0
        lowstate = SimpleNamespace(
            body_gyro_rad_s=np.asarray((0.0, 0.0, 0.5), dtype=np.float64)
        )

        command = self.core()._command(sample, lowstate)

        self.assertAlmostEqual(
            command.yaw_rate,
            (
                0.15
                - MODULE.TURN_COMMAND_YAW_DAMPING_SECONDS * 0.5
            )
            * MODULE.FORMAL_COMMAND_YAW_GAIN,
        )
        self.assertLess(
            command.yaw_rate,
            MODULE.TURN_COMMAND_YAW_LIMIT_RAD_S,
        )

        sample.facing = np.asarray(
            (math.cos(1.0), math.sin(1.0), 0.0),
            dtype=np.float64,
        )
        command = self.core()._command(
            sample,
            SimpleNamespace(body_gyro_rad_s=np.zeros(3, dtype=np.float64)),
        )
        self.assertEqual(
            command.yaw_rate,
            MODULE.TURN_COMMAND_YAW_LIMIT_RAD_S,
        )

    def test_idle_mode_ignores_camera_facing_error(self) -> None:
        command = self.core()._command(
            self.sample(safe_stop=False, mode="idle")
        )

        self.assertEqual(command.vx, 0.0)
        self.assertEqual(command.vy, 0.0)
        self.assertEqual(command.yaw_rate, 0.0)
        self.assertEqual(command.gait, "stand")
        self.assertFalse(command.stop_latched)

    def test_resident_lowcmd_publisher_reuses_latest_target(self) -> None:
        class FakeStore:
            def get(self):
                return SimpleNamespace(
                    received_monotonic=1.0,
                    mode_pr=0,
                    mode_machine=5,
                )

        class FakeDds:
            def __init__(self) -> None:
                self.targets = []
                self.commands = []

            def make_low_cmd(self, target, _config, _state):
                self.targets.append(target.copy())
                return tuple(float(value) for value in target)

            def write(self, publisher, command):
                self.commands.append((publisher, command))
                return True

        class FakeHandoff:
            state = MODULE.HandoffStateMachine.ACTIVE
            publisher = object()

            def __init__(self) -> None:
                self.successful_writes = 0

            def record_successful_write(self) -> None:
                self.successful_writes += 1

        dds = FakeDds()
        handoff = FakeHandoff()
        target = np.arange(MODULE.NUM_JOINTS, dtype=np.float32)
        publisher = MODULE._ResidentLowCmdPublisher(
            dds=dds,
            state_store=FakeStore(),
            handoff=handoff,
            policy_config=SimpleNamespace(),
            target_supplier=lambda: target,
            writer_lock=threading.Lock(),
            monotonic=lambda: 1.01,
        )

        self.assertTrue(publisher._publish_once(1.005))
        self.assertTrue(publisher._publish_once(1.0055))
        target = target + 1.0
        self.assertTrue(publisher._publish_once(1.006))

        self.assertEqual(handoff.successful_writes, 3)
        self.assertEqual(len(dds.commands), 3)
        self.assertEqual(len(dds.targets), 2)
        self.assertIs(dds.commands[0][1], dds.commands[1][1])
        np.testing.assert_allclose(
            dds.targets[0],
            np.arange(MODULE.NUM_JOINTS, dtype=np.float32),
        )
        np.testing.assert_allclose(
            dds.targets[1],
            np.arange(MODULE.NUM_JOINTS, dtype=np.float32) + 1.0,
        )
        telemetry = publisher.telemetry(now=1.02)
        self.assertEqual(telemetry["lowcmd_publish_count"], 3)
        self.assertEqual(telemetry["lowcmd_command_build_count"], 2)
        self.assertEqual(telemetry["lowcmd_command_reuse_count"], 1)
        self.assertAlmostEqual(
            telemetry["lowcmd_publish_last_age_ms"],
            10.0,
        )

    def test_resident_lowcmd_publisher_reuses_cached_command_during_transient(
        self,
    ) -> None:
        state = SimpleNamespace(
            received_monotonic=1.0,
            mode_pr=0,
            mode_machine=5,
        )
        handoff = SimpleNamespace(
            state=MODULE.HandoffStateMachine.ACTIVE,
            publisher=object(),
            record_successful_write=lambda: None,
        )
        commands = []
        dds = SimpleNamespace(
            make_low_cmd=lambda target, *_args: tuple(target),
            write=lambda _publisher, command: commands.append(command) or True,
        )
        publisher = MODULE._ResidentLowCmdPublisher(
            dds=dds,
            state_store=SimpleNamespace(
                get=lambda: state
            ),
            handoff=handoff,
            policy_config=SimpleNamespace(),
            target_supplier=lambda: np.zeros(
                MODULE.NUM_JOINTS,
                dtype=np.float32,
            ),
            writer_lock=threading.Lock(),
            monotonic=lambda: 1.2,
        )

        self.assertTrue(publisher._publish_once(1.05))
        self.assertTrue(publisher._publish_once(1.2))
        self.assertIs(commands[0], commands[1])
        telemetry = publisher.telemetry(now=1.2)
        self.assertEqual(telemetry["lowcmd_stale_command_reuse_count"], 1)
        self.assertAlmostEqual(
            telemetry["lowcmd_stale_lowstate_max_age_ms"],
            200.0,
        )

    def test_resident_lowcmd_publisher_rejects_state_beyond_stale_grace(
        self,
    ) -> None:
        state = SimpleNamespace(
            received_monotonic=1.0,
            mode_pr=0,
            mode_machine=5,
        )
        handoff = SimpleNamespace(
            state=MODULE.HandoffStateMachine.ACTIVE,
            publisher=object(),
            record_successful_write=lambda: None,
        )
        publisher = MODULE._ResidentLowCmdPublisher(
            dds=SimpleNamespace(
                make_low_cmd=lambda target, *_args: tuple(target),
                write=lambda *_args: True,
            ),
            state_store=SimpleNamespace(get=lambda: state),
            handoff=handoff,
            policy_config=SimpleNamespace(),
            target_supplier=lambda: np.zeros(
                MODULE.NUM_JOINTS,
                dtype=np.float32,
            ),
            writer_lock=threading.Lock(),
            monotonic=lambda: 2.0,
        )

        self.assertTrue(publisher._publish_once(1.05))
        with self.assertRaisesRegex(RuntimeError, "beyond transient grace"):
            publisher._publish_once(
                1.0
                + MODULE.LOWSTATE_MAX_AGE_S
                + MODULE.TRANSIENT_INPUT_STALE_GRACE_S
                + 0.001
            )

    @staticmethod
    def inference_core():
        class FakeTeacher:
            def __init__(self) -> None:
                self.reset_count = 0

            def reset(self) -> None:
                self.reset_count += 1

            def step(self, _plan, _observation, _height_map):
                return np.ones(MODULE.NUM_JOINTS, dtype=np.float32)

        class FakeStream:
            def __init__(self) -> None:
                self.reset_count = 0
                self.target_speed = 0.0
                self.future_xy_delta_m = 0.0

            def reset(self) -> None:
                self.reset_count += 1
                self.target_speed = 0.0
                self.future_xy_delta_m = 0.0

            def sample(
                self,
                command,
                _root_position,
                _root_yaw,
                _height_field,
            ):
                if command.gait != "stand":
                    self.target_speed = math.hypot(command.vx, command.vy)
                    self.future_xy_delta_m = 1.0
                else:
                    self.target_speed = 0.0
                    self.future_xy_delta_m = 0.0
                plan = SimpleNamespace(
                    future_qpos=np.zeros((10, 36), dtype=np.float32),
                    target_speed=self.target_speed,
                )
                plan.future_qpos[-1, 0] = self.future_xy_delta_m
                return SimpleNamespace(
                    plan=plan,
                    replanned=False,
                    replan_reason=None,
                    plan_index=1,
                    root_error_before_m=0.0,
                    pending_rebuild=False,
                )

        core = MODULE.BfmTeacherCore.__new__(MODULE.BfmTeacherCore)
        core.command_module = SimpleNamespace(
            CommandSample=lambda **fields: SimpleNamespace(**fields)
        )
        core.teacher_module = SimpleNamespace(
            MUJOCO_TO_ISAACLAB=np.arange(MODULE.NUM_JOINTS),
            RobotObservation=lambda **fields: SimpleNamespace(**fields),
        )
        core.reference_module = SimpleNamespace(
            LocalTerrainHeightField=lambda *_args: object()
        )
        core.teacher = FakeTeacher()
        core.stream = FakeStream()
        core.previous_action = np.zeros(MODULE.NUM_JOINTS, dtype=np.float32)
        core.last_reset_count = 0
        core.last_world_sequence = None
        core.reference_motion_active = False
        core.reference_start_resets = 0
        core.reference_stop_resets = 0
        core.reference_transition = None
        core.reference_hold_target = None
        core.activation_blend_steps = 4
        core.activation_origin = None
        core.activation_step = 0
        core.default_joint_pos = np.zeros(MODULE.NUM_JOINTS, dtype=np.float32)
        core.action_scale = np.ones(MODULE.NUM_JOINTS, dtype=np.float32)
        core.isaac_to_matrix = np.arange(MODULE.NUM_JOINTS)
        return core

    @staticmethod
    def world(sequence: int = 1):
        return SimpleNamespace(
            sequence=sequence,
            reset_count=0,
            root_position=np.asarray((0.0, 0.0, 0.8), dtype=np.float64),
            root_yaw=0.0,
            height_map_z=np.zeros((11, 11), dtype=np.float64),
            movement=np.zeros(3, dtype=np.float64),
            facing=np.asarray((1.0, 0.0, 0.0), dtype=np.float64),
            speed_mps=0.0,
            locomotion_mode=0,
            mode="idle",
            safe_stop=True,
        )

    @staticmethod
    def lowstate(joint_value: float = 0.25):
        return SimpleNamespace(
            quaternion_wxyz=np.asarray(
                (1.0, 0.0, 0.0, 0.0),
                dtype=np.float32,
            ),
            body_gyro_rad_s=np.zeros(3, dtype=np.float32),
            joint_pos_rad=np.full(
                MODULE.NUM_JOINTS,
                joint_value,
                dtype=np.float32,
            ),
            joint_vel_rad_s=np.zeros(MODULE.NUM_JOINTS, dtype=np.float32),
        )

    def test_activation_starts_at_current_joint_pose_without_teleport(self) -> None:
        core = self.inference_core()
        lowstate = self.lowstate()

        core.prepare_activation(lowstate)
        target, status = core.step(self.world(), lowstate, active=True)

        np.testing.assert_allclose(target, lowstate.joint_pos_rad)
        np.testing.assert_allclose(
            core.previous_action,
            lowstate.joint_pos_rad,
        )
        self.assertEqual(status["activation_blend_fraction"], 0.0)
        self.assertEqual(status["published_target_delta_rms_rad"], 0.0)

    def test_standby_preview_does_not_accumulate_unapplied_action(self) -> None:
        core = self.inference_core()
        core.previous_action.fill(7.0)

        target, status = core.step(
            self.world(),
            self.lowstate(),
            active=False,
        )

        np.testing.assert_allclose(target, np.ones(MODULE.NUM_JOINTS))
        np.testing.assert_allclose(
            core.previous_action,
            np.zeros(MODULE.NUM_JOINTS),
        )
        self.assertTrue(status["shadow_preview"])
        self.assertEqual(core.teacher.reset_count, 1)

    def test_motion_to_idle_discards_stale_walking_reference(self) -> None:
        core = self.inference_core()
        lowstate = self.lowstate()
        moving = self.world(sequence=1)
        moving.safe_stop = False
        moving.mode = "move"
        moving.speed_mps = 0.8
        moving.locomotion_mode = 2
        moving.movement = np.asarray((1.0, 0.0, 0.0), dtype=np.float64)

        _, moving_status = core.step(moving, lowstate, active=True)
        self.assertEqual(moving_status["reference_target_speed_mps"], 0.8)
        self.assertEqual(moving_status["reference_future_xy_delta_m"], 1.0)

        stopped = self.world(sequence=2)
        target, stopped_status = core.step(stopped, lowstate, active=True)

        self.assertEqual(core.stream.reset_count, 0)
        self.assertEqual(core.reference_start_resets, 1)
        self.assertEqual(core.reference_stop_resets, 1)
        self.assertTrue(stopped_status["reference_stop_reset"])
        self.assertEqual(stopped_status["reference_stop_reset_count"], 1)
        self.assertEqual(stopped_status["reference_target_speed_mps"], 0.0)
        self.assertEqual(stopped_status["reference_future_xy_delta_m"], 0.0)
        np.testing.assert_allclose(target, lowstate.joint_pos_rad)

    def test_idle_to_motion_rebuilds_reference_from_current_pose(self) -> None:
        core = self.inference_core()
        lowstate = self.lowstate(joint_value=0.37)
        core.step(self.world(sequence=1), lowstate, active=True)
        moving = self.world(sequence=2)
        moving.safe_stop = False
        moving.mode = "move"
        moving.speed_mps = 0.8
        moving.locomotion_mode = 2
        moving.movement = np.asarray((1.0, 0.0, 0.0), dtype=np.float64)

        target, status = core.step(moving, lowstate, active=True)

        self.assertEqual(core.stream.reset_count, 0)
        self.assertEqual(core.teacher.reset_count, 1)
        self.assertEqual(core.reference_start_resets, 1)
        self.assertTrue(status["reference_start_reset"])
        self.assertEqual(status["reference_start_reset_count"], 1)
        self.assertEqual(status["reference_target_speed_mps"], 0.8)
        self.assertEqual(status["reference_future_xy_delta_m"], 1.0)
        np.testing.assert_allclose(target, lowstate.joint_pos_rad)

    def test_motion_start_waits_for_background_buffer_before_history_reset(
        self,
    ) -> None:
        class DelayedStartStream:
            def __init__(self) -> None:
                self.calls = 0

            def sample(self, _command, *_args):
                self.calls += 1
                moving = self.calls >= 2
                plan = SimpleNamespace(
                    future_qpos=np.zeros((10, 36), dtype=np.float32),
                    target_speed=0.8 if moving else 0.0,
                )
                plan.future_qpos[-1, 0] = 1.0 if moving else 0.0
                return SimpleNamespace(
                    plan=plan,
                    replanned=False,
                    replan_reason=None,
                    plan_index=self.calls,
                    root_error_before_m=0.0,
                    pending_rebuild=not moving,
                    buffer_swapped=moving,
                )

        core = self.inference_core()
        lowstate = self.lowstate(joint_value=0.31)
        core.step(self.world(sequence=1), lowstate, active=True)
        core.stream = DelayedStartStream()
        moving = self.world(sequence=2)
        moving.safe_stop = False
        moving.mode = "move"
        moving.speed_mps = 0.8
        moving.locomotion_mode = 2
        moving.movement = np.asarray((1.0, 0.0, 0.0), dtype=np.float64)
        resets_before = core.teacher.reset_count

        held_target, pending = core.step(moving, lowstate, active=True)

        self.assertEqual(core.teacher.reset_count, resets_before)
        self.assertEqual(pending["reference_transition"], "starting")
        self.assertTrue(pending["reference_pending_rebuild"])
        self.assertFalse(pending["reference_transition_completed"])
        self.assertTrue(pending["reference_transition_holding"])
        np.testing.assert_allclose(held_target, lowstate.joint_pos_rad)

        moving.sequence = 3
        target, swapped = core.step(moving, lowstate, active=True)

        self.assertEqual(core.teacher.reset_count, resets_before + 1)
        self.assertIsNone(swapped["reference_transition"])
        self.assertTrue(swapped["reference_buffer_swapped"])
        self.assertTrue(swapped["reference_transition_completed"])
        np.testing.assert_allclose(target, lowstate.joint_pos_rad)

    def test_motion_stop_holds_observed_pose_until_stand_buffer_swaps(
        self,
    ) -> None:
        class DelayedStopStream:
            def __init__(self) -> None:
                self.calls = 0

            def sample(self, _command, *_args):
                self.calls += 1
                swapped = self.calls >= 2
                plan = SimpleNamespace(
                    future_qpos=np.zeros((10, 36), dtype=np.float32),
                    target_speed=0.0 if swapped else 0.8,
                )
                plan.future_qpos[-1, 0] = 0.0 if swapped else 1.0
                return SimpleNamespace(
                    plan=plan,
                    replanned=False,
                    replan_reason=None,
                    plan_index=self.calls,
                    root_error_before_m=0.0,
                    pending_rebuild=not swapped,
                    buffer_swapped=swapped,
                )

        core = self.inference_core()
        lowstate = self.lowstate(joint_value=0.42)
        core.reference_motion_active = True
        core.stream = DelayedStopStream()
        resets_before = core.teacher.reset_count

        held_target, pending = core.step(
            self.world(sequence=1),
            lowstate,
            active=True,
        )

        self.assertEqual(core.teacher.reset_count, resets_before)
        self.assertTrue(pending["reference_transition_holding"])
        self.assertEqual(pending["reference_transition"], "stopping")
        np.testing.assert_allclose(held_target, lowstate.joint_pos_rad)
        np.testing.assert_allclose(
            core.previous_action,
            np.zeros(MODULE.NUM_JOINTS),
        )

        stopped = self.world(sequence=2)
        resumed_target, swapped = core.step(stopped, lowstate, active=True)

        self.assertEqual(core.teacher.reset_count, resets_before + 1)
        self.assertFalse(swapped["reference_transition_holding"])
        self.assertTrue(swapped["reference_transition_completed"])
        self.assertIsNone(swapped["reference_transition"])
        np.testing.assert_allclose(resumed_target, lowstate.joint_pos_rad)


if __name__ == "__main__":
    unittest.main()
