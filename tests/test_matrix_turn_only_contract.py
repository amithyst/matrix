from __future__ import annotations

from enum import Enum, auto
import importlib.util
import math
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
CORE_PATH = REPO_ROOT / "scripts" / "matrix_game_control.py"
RUNTIME_PATH = REPO_ROOT / "scripts" / "run_matrix_sonic.py"


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


CORE = _load_module("matrix_game_control", CORE_PATH)


def _load_runtime_target():
    """Load the isolated runtime edit without importing recovery dependencies."""

    class RecoveryState(Enum):
        GAME_SONIC = auto()
        POLICY_AMP_HOLDING = auto()
        POLICY_AMP_HOLD_REQUESTED = auto()
        POLICY_GETUP_STABLE = auto()
        POLICY_RECOVERING = auto()
        SONIC_RESTARTING = auto()
        SONIC_STABILIZING = auto()
        SONIC_STOP_REQUESTED = auto()
        WAIT_NEUTRAL = auto()

    class ResidentRecoveryState(Enum):
        GAME_SONIC = auto()

    mouse_settings = ModuleType("matrix_mouse_settings")
    mouse_settings.canonical_remote_speed_scale = float
    contacts = ModuleType("matrix_mujoco_contacts")
    contacts.has_external_foot_support = lambda *_args, **_kwargs: False
    contacts.has_external_ground_support = lambda *_args, **_kwargs: False
    recovery = ModuleType("matrix_sonic_recovery")
    recovery.RecoveryState = RecoveryState
    recovery.ResidentRecoveryState = ResidentRecoveryState
    for name in (
        "RecoveryConfig",
        "RecoveryInput",
        "RecoveryOutput",
        "ResidentPolicyRecoveryFSM",
        "ResidentRecoveryInput",
        "ResidentRecoveryOutput",
        "SingleWriterRecoveryFSM",
    ):
        setattr(recovery, name, type(name, (), {}))

    temporary_modules = {
        "matrix_mouse_settings": mouse_settings,
        "matrix_mujoco_contacts": contacts,
        "matrix_sonic_recovery": recovery,
    }
    previous = {name: sys.modules.get(name) for name in temporary_modules}
    try:
        sys.modules.update(temporary_modules)
        return _load_module("matrix_turn_only_runtime_target", RUNTIME_PATH)
    finally:
        for name, prior in previous.items():
            if prior is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = prior


RUNTIME = _load_runtime_target()


def snapshot(
    sequence: int,
    timestamp: float,
    *,
    yaw: float = 0.0,
    w: bool = False,
    shift: bool = False,
):
    return CORE.InputSnapshot.from_mapping(
        {
            "protocol": CORE.PROTOCOL_NAME,
            "sequence": sequence,
            "timestamp_monotonic_s": timestamp,
            "focused": True,
            "camera_yaw_rad": yaw,
            "keys": {
                "w": w,
                "a": False,
                "s": False,
                "d": False,
                "q": False,
                "e": False,
                "v": False,
                "ctrl": False,
                "alt": False,
                "shift": shift,
            },
            "keyboard_boost": False,
            "move_stick": {"right": 0.0, "forward": 0.0},
        }
    )


def armed_core():
    core = CORE.GameControlCore(
        CORE.ControlConfig(
            max_speed_mps=0.3,
            max_acceleration_mps2=1000.0,
            max_deceleration_mps2=1000.0,
            max_turn_rate_rad_s=1000.0,
            max_step_s=1.0,
        )
    )
    core.accept_snapshot(snapshot(0, 9.99), received_at_s=9.99)
    core.synchronize_heading(0.0)
    return core


def recovery_coordinator(*, rotation_rad: float):
    coordinator = RUNTIME._PhysicalRecoveryCoordinator.__new__(
        RUNTIME._PhysicalRecoveryCoordinator
    )
    coordinator.command_frame_rotation_rad = rotation_rad
    coordinator.command_frame_epoch = 1
    coordinator.last_wire_facing_heading_rad = None
    coordinator.reframe_limited_frames = 0
    coordinator.last_reframe_limited = False
    coordinator.last_reframe_heading_error_rad = 0.0
    return coordinator


class TurnOnlyCoreContractTest(unittest.TestCase):
    def test_reanchor_heading_resets_stale_command_once(self) -> None:
        core = CORE.GameControlCore(initial_heading_rad=0.0)
        core.accept_snapshot(snapshot(0, 9.99), received_at_s=9.99)
        recovered_heading = math.radians(150.0)

        core.reanchor_heading(recovered_heading)
        idle = core.command(now_s=10.0, dt_s=0.0)

        self.assertAlmostEqual(core.heading_rad, recovered_heading)
        self.assertAlmostEqual(core.measured_heading_rad, recovered_heading)
        self.assertAlmostEqual(
            math.atan2(idle.facing[1], idle.facing[0]), recovered_heading
        )
        # Ordinary feedback remains feedback-only after the lifecycle reset;
        # stopped IDLE does not chase residual physical yaw.
        core.synchronize_heading(math.radians(155.0))
        held = core.command(now_s=10.01, dt_s=0.0)
        self.assertAlmostEqual(
            math.atan2(held.facing[1], held.facing[0]), recovered_heading
        )

    def test_turn_target_cannot_run_ahead_of_stationary_measured_heading(self) -> None:
        core = CORE.GameControlCore(
            CORE.ControlConfig(
                max_speed_mps=0.3,
                max_acceleration_mps2=1000.0,
                max_deceleration_mps2=1000.0,
                max_turn_rate_rad_s=2.5,
                input_timeout_s=10.0,
                max_snapshot_age_s=10.0,
                max_step_s=0.1,
            ),
            initial_heading_rad=math.radians(150.0),
        )
        core.accept_snapshot(snapshot(0, 9.99), received_at_s=9.99)
        core.synchronize_heading(math.radians(150.0))
        core.accept_snapshot(
            snapshot(1, 10.0, yaw=0.0, w=True),
            received_at_s=10.0,
        )

        commands = [
            core.command(now_s=10.0 + index * 0.02, dt_s=0.02)
            for index in range(40)
        ]
        expected = math.radians(150.0) - 2.5 * 0.02
        self.assertTrue(all(command.mode == "turn" for command in commands))
        self.assertTrue(all(command.speed_mps == 0.0 for command in commands))
        self.assertTrue(
            all(command.movement == (0.0, 0.0, 0.0) for command in commands)
        )
        for command in commands:
            self.assertAlmostEqual(
                math.atan2(command.facing[1], command.facing[0]),
                expected,
            )

    def test_turn_target_advances_only_when_measured_heading_advances(self) -> None:
        core = CORE.GameControlCore(
            CORE.ControlConfig(
                max_speed_mps=0.3,
                max_acceleration_mps2=1000.0,
                max_deceleration_mps2=1000.0,
                max_turn_rate_rad_s=2.5,
                input_timeout_s=10.0,
                max_snapshot_age_s=10.0,
                max_step_s=0.1,
            ),
            initial_heading_rad=math.radians(150.0),
        )
        core.accept_snapshot(snapshot(0, 9.99), received_at_s=9.99)
        core.synchronize_heading(math.radians(150.0))
        core.accept_snapshot(
            snapshot(1, 10.0, yaw=0.0, w=True),
            received_at_s=10.0,
        )

        first = core.command(now_s=10.0, dt_s=0.02)
        core.synchronize_heading(math.radians(147.0))
        second = core.command(now_s=10.02, dt_s=0.02)

        self.assertAlmostEqual(
            math.atan2(first.facing[1], first.facing[0]),
            math.radians(150.0) - 0.05,
        )
        self.assertAlmostEqual(
            math.atan2(second.facing[1], second.facing[0]),
            math.radians(147.0) - 0.05,
        )

    def test_unaligned_turn_then_neutral_then_aligned_walk_and_run(self) -> None:
        for shift, expected_mode, expected_speed in (
            (False, CORE.SONIC_WALK_MODE, 0.8),
            (True, CORE.SONIC_RUN_MODE, 2.5),
        ):
            with self.subTest(shift=shift):
                core = armed_core()
                core.accept_snapshot(
                    snapshot(1, 10.0, yaw=math.pi / 2.0, w=True, shift=shift),
                    received_at_s=10.0,
                )

                turning = core.command(now_s=10.0, dt_s=0.1)
                self.assertEqual(turning.mode, "turn")
                self.assertEqual(
                    turning.locomotion_mode, CORE.SONIC_IDLE_MODE
                )
                self.assertEqual(turning.speed_mps, 0.0)
                self.assertEqual(turning.movement, (0.0, 0.0, 0.0))
                self.assertFalse(turning.safe_stop)

                core.accept_snapshot(
                    snapshot(2, 10.01, yaw=math.pi / 2.0),
                    received_at_s=10.01,
                )
                neutral = core.command(now_s=10.01, dt_s=0.1)
                self.assertEqual(neutral.mode, "idle")
                self.assertEqual(neutral.locomotion_mode, CORE.SONIC_IDLE_MODE)
                self.assertEqual(neutral.speed_mps, 0.0)
                self.assertEqual(neutral.movement, (0.0, 0.0, 0.0))

                core.synchronize_heading(math.pi / 2.0)
                core.accept_snapshot(
                    snapshot(3, 10.02, yaw=math.pi / 2.0, w=True, shift=shift),
                    received_at_s=10.02,
                )
                gait_entry = core.command(now_s=10.02, dt_s=0.1)
                aligned = core.command(now_s=10.02, dt_s=0.1)
                self.assertEqual(
                    gait_entry.locomotion_mode, CORE.SONIC_SLOW_WALK_MODE
                )
                self.assertEqual(gait_entry.speed_mps, 0.1)
                self.assertEqual(aligned.mode, "move")
                self.assertEqual(aligned.locomotion_mode, expected_mode)
                self.assertEqual(aligned.speed_mps, expected_speed)
                self.assertEqual(aligned.movement, aligned.facing)


class TurnOnlyRuntimeContractTest(unittest.TestCase):
    @staticmethod
    def planner_client(planner_frames: list[dict[str, object]]):
        class FakeSocket:
            def setsockopt(self, *_args) -> None:
                pass

            def bind(self, _endpoint: str) -> None:
                pass

            def send(self, _payload: bytes) -> None:
                pass

            def close(self, **_kwargs) -> None:
                pass

        fake_socket = FakeSocket()

        class FakeContext:
            @classmethod
            def instance(cls):
                return cls()

            def socket(self, _kind: int):
                return fake_socket

        return RUNTIME.NativePlannerClient(
            "tcp://127.0.0.1:5556",
            zmq_module=SimpleNamespace(Context=FakeContext, PUB=1, LINGER=2),
            build_command_message=lambda **_kwargs: b"command",
            build_planner_message=lambda **kwargs: (
                planner_frames.append(kwargs) or b"planner"
            ),
        )

    def test_turn_only_wire_contract_and_moving_frame_accounting(self) -> None:
        core = armed_core()
        core.accept_snapshot(
            snapshot(1, 10.0, yaw=math.pi / 2.0, w=True),
            received_at_s=10.0,
        )
        turning = core.command(now_s=10.0, dt_s=0.1)

        planner_frames: list[dict[str, object]] = []
        planner = self.planner_client(planner_frames)
        planner.send_game_command(turning)
        self.assertEqual(turning.locomotion_mode, CORE.SONIC_IDLE_MODE)
        self.assertEqual(planner_frames[-1]["mode"], CORE.SONIC_WALK_MODE)
        self.assertEqual(planner_frames[-1]["speed"], -1.0)
        self.assertEqual(planner_frames[-1]["movement"], [0.0, 0.0, 0.0])
        self.assertEqual(planner_frames[-1]["facing"], list(turning.facing))

        core.accept_snapshot(snapshot(2, 10.01), received_at_s=10.01)
        neutral = core.command(now_s=10.01, dt_s=0.1)
        planner.send_game_command(neutral)
        self.assertEqual(planner_frames[-1]["mode"], CORE.SONIC_IDLE_MODE)
        self.assertEqual(planner_frames[-1]["speed"], -1.0)
        self.assertEqual(planner_frames[-1]["movement"], [0.0, 0.0, 0.0])

        with tempfile.TemporaryDirectory() as temporary:
            runtime = RUNTIME.GameInputRuntime(
                Path(temporary) / "game.sock",
                core,
            )
            runtime.record_published_command(turning)
            runtime.record_published_command(neutral)
            self.assertEqual(runtime.moving_command_frames, 0)

            core.synchronize_heading(math.pi / 2.0)
            core.accept_snapshot(
                snapshot(3, 10.02, yaw=math.pi / 2.0, w=True),
                received_at_s=10.02,
            )
            core.command(now_s=10.02, dt_s=0.1)
            walking = core.command(now_s=10.02, dt_s=0.1)
            runtime.record_published_command(walking)
            self.assertEqual(runtime.moving_command_frames, 1)

    def test_recovery_reframe_caps_composed_core_target_at_wire_boundary(self) -> None:
        recovered_heading = math.radians(150.0)
        core = CORE.GameControlCore(
            CORE.ControlConfig(
                max_speed_mps=0.3,
                max_acceleration_mps2=1000.0,
                max_deceleration_mps2=1000.0,
                max_turn_rate_rad_s=2.5,
                input_timeout_s=10.0,
                max_snapshot_age_s=10.0,
                max_step_s=0.1,
            )
        )
        core.accept_snapshot(snapshot(0, 9.99), received_at_s=9.99)
        core.reanchor_heading(recovered_heading)
        core.accept_snapshot(
            snapshot(1, 10.0, yaw=0.0, w=True),
            received_at_s=10.0,
        )
        world_command = core.command(now_s=10.0, dt_s=0.02)
        coordinator = recovery_coordinator(rotation_rad=-recovered_heading)

        wire_command = coordinator.reframe_game_command(
            world_command,
            measured_heading_rad=recovered_heading,
            dt_s=0.02,
        )

        self.assertEqual(world_command.reason, "aligning_heading")
        self.assertEqual(wire_command.mode, "turn")
        self.assertEqual(
            wire_command.reason,
            "recovery_heading_slew_limited",
        )
        self.assertEqual(
            wire_command.locomotion_mode, CORE.SONIC_IDLE_MODE
        )
        self.assertEqual(wire_command.speed_mps, 0.0)
        self.assertEqual(wire_command.movement, (0.0, 0.0, 0.0))
        wire_heading = math.atan2(
            wire_command.facing[1], wire_command.facing[0]
        )
        self.assertAlmostEqual(
            wire_heading,
            -coordinator.WIRE_MAX_HEADING_STEP_RAD,
        )
        self.assertLessEqual(
            abs(wire_heading), coordinator.WIRE_MAX_HEADING_STEP_RAD
        )
        self.assertTrue(coordinator.last_reframe_limited)
        self.assertEqual(coordinator.reframe_limited_frames, 1)

        planner_frames: list[dict[str, object]] = []
        planner = self.planner_client(planner_frames)
        planner.send_game_command(wire_command)
        self.assertEqual(planner_frames[-1]["mode"], CORE.SONIC_WALK_MODE)
        self.assertEqual(planner_frames[-1]["movement"], [0.0, 0.0, 0.0])
        self.assertEqual(planner_frames[-1]["speed"], -1.0)

    def test_recovery_reframe_neutral_holds_measured_deploy_heading(self) -> None:
        recovered_heading = math.radians(150.0)
        coordinator = recovery_coordinator(rotation_rad=-recovered_heading)
        reanchored_neutral = CORE.RobotMotionCommand(
            sequence=1,
            movement=(0.0, 0.0, 0.0),
            facing=(
                math.cos(recovered_heading),
                math.sin(recovered_heading),
                0.0,
            ),
            speed_mps=0.0,
            locomotion_mode=CORE.SONIC_IDLE_MODE,
            mode="idle",
            safe_stop=False,
            reason=None,
        )

        reframed = coordinator.reframe_game_command(
            reanchored_neutral,
            measured_heading_rad=recovered_heading,
            dt_s=0.02,
        )

        self.assertEqual(reframed.mode, "idle")
        self.assertEqual(reframed.locomotion_mode, CORE.SONIC_IDLE_MODE)
        self.assertEqual(reframed.speed_mps, 0.0)
        self.assertEqual(reframed.movement, (0.0, 0.0, 0.0))
        self.assertAlmostEqual(
            math.atan2(reframed.facing[1], reframed.facing[0]), 0.0
        )
        self.assertFalse(coordinator.last_reframe_limited)

    def test_recovery_reframe_uses_short_arc_across_pi(self) -> None:
        coordinator = recovery_coordinator(rotation_rad=0.0)
        command = CORE.RobotMotionCommand(
            sequence=1,
            movement=(0.0, 0.0, 0.0),
            facing=(
                math.cos(math.radians(-179.0)),
                math.sin(math.radians(-179.0)),
                0.0,
            ),
            speed_mps=0.0,
            locomotion_mode=CORE.SONIC_IDLE_MODE,
            mode="turn",
            safe_stop=False,
            reason="aligning_heading",
            desired_facing=(
                math.cos(math.radians(-179.0)),
                math.sin(math.radians(-179.0)),
                0.0,
            ),
        )

        reframed = coordinator.reframe_game_command(
            command,
            measured_heading_rad=math.radians(179.0),
            dt_s=0.02,
        )
        output_heading = math.atan2(reframed.facing[1], reframed.facing[0])
        self.assertAlmostEqual(
            CORE.wrap_angle_rad(output_heading - math.radians(179.0)),
            coordinator.WIRE_MAX_HEADING_STEP_RAD,
        )

    def test_recovery_wire_facing_is_slew_limited_across_measured_yaw_jumps(
        self,
    ) -> None:
        coordinator = recovery_coordinator(rotation_rad=0.0)
        coordinator.last_wire_facing_heading_rad = 0.0
        command = CORE.RobotMotionCommand(
            sequence=1,
            movement=(0.0, 0.0, 0.0),
            facing=(-1.0, 0.0, 0.0),
            speed_mps=0.0,
            locomotion_mode=CORE.SONIC_IDLE_MODE,
            mode="turn",
            safe_stop=False,
            reason="aligning_heading",
            desired_facing=(math.cos(-1.0), math.sin(-1.0), 0.0),
        )

        previous_heading = 0.0
        for measured_heading in (0.0, 1.2, -1.2, 2.4, -2.4):
            reframed = coordinator.reframe_game_command(
                command,
                measured_heading_rad=measured_heading,
                dt_s=0.02,
            )
            wire_heading = math.atan2(
                reframed.facing[1], reframed.facing[0]
            )
            self.assertLessEqual(
                abs(CORE.wrap_angle_rad(wire_heading - previous_heading)),
                coordinator.WIRE_MAX_HEADING_STEP_RAD + 1e-12,
            )
            self.assertEqual(reframed.mode, "turn")
            self.assertEqual(reframed.movement, (0.0, 0.0, 0.0))
            self.assertEqual(reframed.speed_mps, 0.0)
            previous_heading = wire_heading

    def test_recovery_wire_target_accumulates_physical_lead_without_translation(
        self,
    ) -> None:
        coordinator = recovery_coordinator(rotation_rad=0.0)
        coordinator.last_wire_facing_heading_rad = 0.0
        command = CORE.RobotMotionCommand(
            sequence=1,
            movement=(0.0, 0.0, 0.0),
            facing=(math.cos(-0.05), math.sin(-0.05), 0.0),
            speed_mps=0.0,
            locomotion_mode=CORE.SONIC_IDLE_MODE,
            mode="turn",
            safe_stop=False,
            reason="aligning_heading",
            desired_facing=(math.cos(-1.0), math.sin(-1.0), 0.0),
        )

        headings = []
        frame_count = int(
            coordinator.WIRE_TURN_LEAD_WINDOW_RAD
            / coordinator.WIRE_MAX_HEADING_STEP_RAD
        )
        for _ in range(frame_count + 2):
            reframed = coordinator.reframe_game_command(
                command,
                measured_heading_rad=0.0,
                dt_s=0.02,
            )
            headings.append(
                math.atan2(reframed.facing[1], reframed.facing[0])
            )
            self.assertEqual(reframed.mode, "turn")
            self.assertEqual(reframed.movement, (0.0, 0.0, 0.0))
            self.assertEqual(reframed.speed_mps, 0.0)

        self.assertEqual(len(headings), frame_count + 2)
        self.assertAlmostEqual(headings[0], -0.02)
        self.assertAlmostEqual(headings[1], -0.04)
        self.assertAlmostEqual(
            headings[frame_count - 1],
            -coordinator.WIRE_TURN_LEAD_WINDOW_RAD,
        )
        self.assertAlmostEqual(
            headings[-1],
            -coordinator.WIRE_TURN_LEAD_WINDOW_RAD,
        )
        for previous, current in zip([0.0, *headings], headings):
            self.assertLessEqual(
                abs(CORE.wrap_angle_rad(current - previous)),
                coordinator.WIRE_MAX_HEADING_STEP_RAD + 1e-12,
            )

    def test_recovery_move_waits_for_wire_target_then_translates(self) -> None:
        coordinator = recovery_coordinator(rotation_rad=0.0)
        coordinator.last_wire_facing_heading_rad = 0.0
        turn = CORE.RobotMotionCommand(
            sequence=1,
            movement=(0.0, 0.0, 0.0),
            facing=(math.cos(-0.05), math.sin(-0.05), 0.0),
            speed_mps=0.0,
            locomotion_mode=CORE.SONIC_IDLE_MODE,
            mode="turn",
            safe_stop=False,
            reason="aligning_heading",
            desired_facing=(math.cos(-1.0), math.sin(-1.0), 0.0),
        )
        for _ in range(25):
            reframed = coordinator.reframe_game_command(
                turn,
                measured_heading_rad=0.0,
                dt_s=0.02,
            )
            self.assertEqual(reframed.mode, "turn")
            self.assertEqual(reframed.movement, (0.0, 0.0, 0.0))
            self.assertEqual(reframed.speed_mps, 0.0)

        move = CORE.RobotMotionCommand(
            sequence=2,
            movement=(1.0, 0.0, 0.0),
            facing=(1.0, 0.0, 0.0),
            speed_mps=0.3,
            locomotion_mode=CORE.SONIC_SLOW_WALK_MODE,
            mode="move",
            safe_stop=False,
            reason=None,
            desired_facing=(1.0, 0.0, 0.0),
        )
        previous_heading = math.atan2(
            reframed.facing[1], reframed.facing[0]
        )
        for _ in range(19):
            waiting = coordinator.reframe_game_command(
                move,
                measured_heading_rad=0.0,
                dt_s=0.02,
            )
            waiting_heading = math.atan2(
                waiting.facing[1], waiting.facing[0]
            )
            self.assertLessEqual(
                abs(CORE.wrap_angle_rad(waiting_heading - previous_heading)),
                coordinator.WIRE_MAX_HEADING_STEP_RAD + 1e-12,
            )
            self.assertEqual(waiting.mode, "turn")
            self.assertEqual(waiting.movement, (0.0, 0.0, 0.0))
            self.assertEqual(waiting.speed_mps, 0.0)
            previous_heading = waiting_heading

        translated = coordinator.reframe_game_command(
            move,
            measured_heading_rad=0.0,
            dt_s=0.02,
        )
        self.assertEqual(translated.mode, "move")
        self.assertEqual(translated.movement, (1.0, 0.0, 0.0))
        self.assertEqual(translated.speed_mps, 0.3)

    def test_recovery_neutral_unwinds_active_wire_latch_toward_safe_target(
        self,
    ) -> None:
        coordinator = recovery_coordinator(rotation_rad=0.0)
        coordinator.last_wire_facing_heading_rad = 0.37
        neutral = CORE.RobotMotionCommand(
            sequence=2,
            movement=(0.0, 0.0, 0.0),
            facing=(math.cos(-1.0), math.sin(-1.0), 0.0),
            speed_mps=0.0,
            locomotion_mode=CORE.SONIC_IDLE_MODE,
            mode="idle",
            safe_stop=False,
            reason=None,
        )

        reframed = coordinator.reframe_game_command(
            neutral,
            measured_heading_rad=2.0,
            dt_s=0.02,
        )

        self.assertEqual(reframed.mode, "idle")
        self.assertAlmostEqual(
            math.atan2(reframed.facing[1], reframed.facing[0]), 0.35
        )
        self.assertAlmostEqual(coordinator.last_wire_facing_heading_rad, 0.35)
        self.assertEqual(reframed.movement, (0.0, 0.0, 0.0))
        self.assertEqual(reframed.speed_mps, 0.0)

    def test_recovery_active_neutral_active_wire_steps_are_all_bounded(self) -> None:
        coordinator = recovery_coordinator(rotation_rad=0.0)
        coordinator.last_wire_facing_heading_rad = 0.0
        active = CORE.RobotMotionCommand(
            sequence=1,
            movement=(0.0, 0.0, 0.0),
            facing=(-1.0, 0.0, 0.0),
            speed_mps=0.0,
            locomotion_mode=CORE.SONIC_IDLE_MODE,
            mode="turn",
            safe_stop=False,
            reason="aligning_heading",
            desired_facing=(-1.0, 0.0, 0.0),
        )
        neutral = CORE.RobotMotionCommand(
            sequence=2,
            movement=(0.0, 0.0, 0.0),
            facing=(math.cos(-2.0), math.sin(-2.0), 0.0),
            speed_mps=0.0,
            locomotion_mode=CORE.SONIC_IDLE_MODE,
            mode="deadman",
            safe_stop=True,
            reason="physical_fall_recovery",
        )

        first_active = coordinator.reframe_game_command(
            active,
            measured_heading_rad=0.0,
            dt_s=0.02,
        )
        first_heading = math.atan2(
            first_active.facing[1], first_active.facing[0]
        )
        stopped = coordinator.reframe_game_command(
            neutral,
            measured_heading_rad=1.5,
            dt_s=0.02,
        )
        stopped_heading = math.atan2(stopped.facing[1], stopped.facing[0])
        latch_after_stop = coordinator.last_wire_facing_heading_rad
        second_active = coordinator.reframe_game_command(
            active,
            measured_heading_rad=1.5,
            dt_s=0.02,
        )
        second_heading = math.atan2(
            second_active.facing[1], second_active.facing[0]
        )

        self.assertLessEqual(
            abs(CORE.wrap_angle_rad(first_heading - 0.0)),
            coordinator.WIRE_MAX_HEADING_STEP_RAD + 1e-12,
        )
        self.assertAlmostEqual(
            CORE.wrap_angle_rad(stopped_heading - first_heading),
            -coordinator.WIRE_MAX_HEADING_STEP_RAD,
        )
        self.assertAlmostEqual(latch_after_stop, stopped_heading)
        self.assertLessEqual(
            abs(CORE.wrap_angle_rad(second_heading - stopped_heading)),
            coordinator.WIRE_MAX_HEADING_STEP_RAD + 1e-12,
        )

    def test_recovery_turn_lead_never_overshoots_true_desired_heading(
        self,
    ) -> None:
        coordinator = recovery_coordinator(rotation_rad=0.0)
        coordinator.last_wire_facing_heading_rad = 0.0
        desired_heading = 0.2
        command = CORE.RobotMotionCommand(
            sequence=1,
            movement=(9.0, 0.0, 0.0),
            facing=(math.cos(0.05), math.sin(0.05), 0.0),
            speed_mps=9.0,
            locomotion_mode=CORE.SONIC_RUN_MODE,
            mode="turn",
            safe_stop=False,
            reason="aligning_heading",
            desired_facing=(
                math.cos(desired_heading),
                math.sin(desired_heading),
                0.0,
            ),
        )

        headings = []
        for _ in range(20):
            reframed = coordinator.reframe_game_command(
                command,
                measured_heading_rad=0.0,
                dt_s=0.02,
            )
            headings.append(math.atan2(reframed.facing[1], reframed.facing[0]))
            self.assertEqual(reframed.mode, "turn")
            self.assertEqual(reframed.movement, (0.0, 0.0, 0.0))
            self.assertEqual(reframed.speed_mps, 0.0)
            self.assertEqual(
                reframed.locomotion_mode, CORE.SONIC_IDLE_MODE
            )

        self.assertAlmostEqual(headings[-1], desired_heading)
        self.assertTrue(
            all(heading <= desired_heading + 1e-12 for heading in headings)
        )


if __name__ == "__main__":
    unittest.main()
