from __future__ import annotations

from collections import deque
from contextlib import redirect_stdout
import importlib.util
import io
import json
from pathlib import Path
import socket
import sys
import tempfile
import unittest

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "matrix_external_state_relay.py"
SPEC = importlib.util.spec_from_file_location(
    "matrix_external_state_relay", SCRIPT_PATH
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


# Frozen in Leo's ee8745ad runtime.  matrix_state_sink.py receives this order
# from IsaacLab and converts it to SMP/Matrix order before emitting its 588-byte
# datagram.  The relay must therefore preserve, rather than repeat, this map.
ISAACLAB_JOINT_ORDER = (
    "left_hip_pitch_joint",
    "right_hip_pitch_joint",
    "waist_yaw_joint",
    "left_hip_roll_joint",
    "right_hip_roll_joint",
    "waist_roll_joint",
    "left_hip_yaw_joint",
    "right_hip_yaw_joint",
    "waist_pitch_joint",
    "left_knee_joint",
    "right_knee_joint",
    "left_shoulder_pitch_joint",
    "right_shoulder_pitch_joint",
    "left_ankle_pitch_joint",
    "right_ankle_pitch_joint",
    "left_shoulder_roll_joint",
    "right_shoulder_roll_joint",
    "left_ankle_roll_joint",
    "right_ankle_roll_joint",
    "left_shoulder_yaw_joint",
    "right_shoulder_yaw_joint",
    "left_elbow_joint",
    "right_elbow_joint",
    "left_wrist_roll_joint",
    "right_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "right_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_wrist_yaw_joint",
)

MATRIX_MUJOCO_JOINT_ORDER = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)

ISAACLAB_TO_MATRIX_SOURCE_INDICES = (
    0,
    3,
    6,
    9,
    13,
    17,
    1,
    4,
    7,
    10,
    14,
    18,
    2,
    5,
    8,
    11,
    15,
    19,
    21,
    23,
    25,
    27,
    12,
    16,
    20,
    22,
    24,
    26,
    28,
)


def render_state(
    sim_time: float,
    *,
    position: float = 0.0,
    quaternion: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
) -> MODULE.MujocoRenderState:
    qpos = np.zeros(36, dtype=np.float64)
    qpos[:3] = position
    qpos[3:7] = quaternion
    qpos[7:] = position + np.arange(29, dtype=np.float64)
    return MODULE.MujocoRenderState(
        sim_time=sim_time,
        qpos=qpos,
        qvel=position + np.arange(35, dtype=np.float64),
        ctrl=np.empty(0, dtype=np.float64),
    )


def boundary_state(x: float, y: float) -> MODULE.MujocoRenderState:
    state = render_state(0.0)
    state.qpos[0] = x
    state.qpos[1] = y
    return state


class MatrixExternalStateRelayTest(unittest.TestCase):
    def test_relay_converts_frozen_588_byte_state_to_canonical_820_bytes(self) -> None:
        source = render_state(0.0)
        source_packet = MODULE.pack_mujoco_state(
            source.sim_time, source.qpos, source.qvel, source.ctrl
        )
        self.assertEqual(len(source_packet), 588)
        self.assertEqual(MODULE.packet_size(nq=36, nv=35, nu=0), 588)

        decoded = MODULE.unpack_mujoco_state(source_packet)
        MODULE.validate_source_state(decoded, nq=36, nv=35, nu=0)
        output = MODULE.matrix_output_state(decoded, output_nu=29, ctrl_fill=0.0)
        output_packet = MODULE.pack_mujoco_state(
            output.sim_time, output.qpos, output.qvel, output.ctrl
        )

        self.assertEqual(len(output_packet), 820)
        self.assertEqual(MODULE.packet_size(nq=36, nv=35, nu=29), 820)
        relayed = MODULE.unpack_mujoco_state(output_packet)
        np.testing.assert_array_equal(relayed.qpos, source.qpos)
        np.testing.assert_array_equal(relayed.qvel, source.qvel)
        np.testing.assert_array_equal(relayed.ctrl, np.zeros(29))

    def test_relay_preserves_upstream_isaaclab_to_mujoco_mapping(self) -> None:
        derived_indices = tuple(
            ISAACLAB_JOINT_ORDER.index(name) for name in MATRIX_MUJOCO_JOINT_ORDER
        )
        self.assertEqual(derived_indices, ISAACLAB_TO_MATRIX_SOURCE_INDICES)

        isaac_joint_pos = np.arange(29, dtype=np.float64)
        isaac_joint_vel = 100.0 + isaac_joint_pos
        qpos = np.concatenate(
            (
                np.asarray((1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0)),
                isaac_joint_pos[list(derived_indices)],
            )
        )
        qvel = np.concatenate(
            (
                np.asarray((0.1, 0.2, 0.3, 0.4, 0.5, 0.6)),
                isaac_joint_vel[list(derived_indices)],
            )
        )
        source = MODULE.MujocoRenderState(
            sim_time=0.5,
            qpos=qpos,
            qvel=qvel,
            ctrl=np.empty(0, dtype=np.float64),
        )

        output = MODULE.matrix_output_state(source, output_nu=29, ctrl_fill=0.0)

        np.testing.assert_array_equal(output.qpos[7:], qpos[7:])
        np.testing.assert_array_equal(output.qvel[6:], qvel[6:])
        np.testing.assert_array_equal(
            output.qpos[7:], isaac_joint_pos[list(ISAACLAB_TO_MATRIX_SOURCE_INDICES)]
        )
        np.testing.assert_array_equal(
            output.qvel[6:], isaac_joint_vel[list(ISAACLAB_TO_MATRIX_SOURCE_INDICES)]
        )

    def test_sequence_tracker_rejects_duplicate_and_counts_gap(self) -> None:
        tracker = MODULE.SequenceTracker(50.0)
        stats = MODULE.RelayStats()

        self.assertTrue(tracker.observe(0.00, stats))
        self.assertTrue(tracker.observe(0.02, stats))
        self.assertFalse(tracker.observe(0.02, stats))
        self.assertTrue(tracker.observe(0.06, stats))
        self.assertFalse(tracker.observe(0.04, stats))
        self.assertFalse(tracker.observe(0.081, stats))

        self.assertEqual(stats.first_sequence, 0)
        self.assertEqual(stats.last_sequence, 3)
        self.assertEqual(stats.duplicates, 1)
        self.assertEqual(stats.sequence_gaps, 1)
        self.assertEqual(stats.out_of_order, 1)
        self.assertEqual(stats.non_grid_time, 1)

    def test_wall_time_lookup_interpolates_only_strictly_inside_history(self) -> None:
        left = render_state(0.0, position=0.0)
        right = render_state(
            0.02,
            position=2.0,
            quaternion=(0.0, 0.0, 0.0, 1.0),
        )
        history = deque(((10.0, left), (12.0, right)))

        before, before_interpolated = MODULE.state_at_wall_time(history, 9.0)
        at_left, at_left_interpolated = MODULE.state_at_wall_time(history, 10.0)
        middle, middle_interpolated = MODULE.state_at_wall_time(history, 11.0)
        at_right, at_right_interpolated = MODULE.state_at_wall_time(history, 12.0)
        after, after_interpolated = MODULE.state_at_wall_time(history, 13.0)

        self.assertIs(before, left)
        self.assertIs(at_left, left)
        self.assertIs(at_right, right)
        self.assertIs(after, right)
        self.assertFalse(before_interpolated)
        self.assertFalse(at_left_interpolated)
        self.assertTrue(middle_interpolated)
        self.assertFalse(at_right_interpolated)
        self.assertFalse(after_interpolated)
        self.assertAlmostEqual(middle.sim_time, 0.01)
        np.testing.assert_allclose(middle.qpos[:3], np.ones(3), atol=1.0e-12)
        np.testing.assert_allclose(
            middle.qpos[3:7],
            np.asarray((2**-0.5, 0.0, 0.0, 2**-0.5)),
            atol=1.0e-12,
        )
        np.testing.assert_allclose(middle.qvel, (left.qvel + right.qvel) / 2.0)

    def test_interpolation_rejects_extrapolation_and_empty_history(self) -> None:
        left = render_state(0.0)
        right = render_state(0.02, position=1.0)

        for alpha in (-0.01, 1.01):
            with self.subTest(alpha=alpha), self.assertRaisesRegex(
                ValueError, "alpha must be in"
            ):
                MODULE.interpolate_state(left, right, alpha)
        with self.assertRaisesRegex(ValueError, "history is empty"):
            MODULE.state_at_wall_time(deque(), 0.0)

    def test_collision_bounds_reject_invalid_geometry_and_margins(self) -> None:
        invalid = (
            (10.0, 0.0, 0.0, 100.0, 20.0, 10.0),
            (0.0, 100.0, 0.0, 100.0, 10.0, 10.0),
            (0.0, 100.0, 0.0, 100.0, 10.0, 20.0),
            (0.0, 100.0, 0.0, 100.0, float("inf"), 10.0),
        )
        for values in invalid:
            with self.subTest(values=values), self.assertRaises(ValueError):
                MODULE.CollisionBounds(*values)

    def test_boundary_warning_latches_once_before_stop_margin(self) -> None:
        guard = MODULE.BoundaryGuard(
            MODULE.CollisionBounds(0.0, 100.0, 0.0, 100.0, 20.0, 10.0),
            keyboard_socket=None,
        )
        output = io.StringIO()
        try:
            with redirect_stdout(output):
                guard.observe(boundary_state(81.0, 50.0), wall_time=0.0)
                guard.observe(boundary_state(82.0, 50.0), wall_time=0.1)
        finally:
            guard.close()

        self.assertEqual(guard.stats.warning_events, 1)
        self.assertEqual(guard.stats.stop_events, 0)
        self.assertEqual(guard.stats.command_errors, 0)
        self.assertIn("collision boundary approaching", output.getvalue())

    def test_boundary_stop_sends_space_pulse_and_repeats_at_250_ms(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            socket_path = Path(temporary) / "keyboard.sock"
            receiver = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            receiver.bind(str(socket_path))
            receiver.settimeout(0.02)
            guard = MODULE.BoundaryGuard(
                MODULE.CollisionBounds(
                    0.0, 100.0, 0.0, 100.0, 20.0, 10.0
                ),
                keyboard_socket=socket_path,
                repeat_s=0.25,
            )
            output = io.StringIO()
            try:
                with redirect_stdout(output):
                    guard.observe(boundary_state(90.0, 50.0), wall_time=0.0)
                first = json.loads(receiver.recv(4096))
                second = json.loads(receiver.recv(4096))
                self.assertEqual(first, {"key": "SPACE", "pressed": True})
                self.assertEqual(second, {"key": "SPACE", "pressed": False})

                with redirect_stdout(output):
                    guard.observe(boundary_state(91.0, 50.0), wall_time=0.249)
                with self.assertRaises(socket.timeout):
                    receiver.recv(4096)

                with redirect_stdout(output):
                    guard.observe(boundary_state(91.0, 50.0), wall_time=0.25)
                repeated_press = json.loads(receiver.recv(4096))
                repeated_release = json.loads(receiver.recv(4096))
                self.assertEqual(
                    (repeated_press, repeated_release),
                    (
                        {"key": "SPACE", "pressed": True},
                        {"key": "SPACE", "pressed": False},
                    ),
                )
            finally:
                guard.close()
                receiver.close()

        self.assertEqual(guard.stats.warning_events, 1)
        self.assertEqual(guard.stats.stop_events, 1)
        self.assertEqual(guard.stats.stop_pulses, 2)
        self.assertEqual(guard.stats.command_errors, 0)
        self.assertEqual(guard.stats.first_stop_root_xy, [90.0, 50.0])
        self.assertEqual(guard.stats.minimum_edge_distance_m, 9.0)

    def test_boundary_hard_violation_latches_once(self) -> None:
        guard = MODULE.BoundaryGuard(
            MODULE.CollisionBounds(0.0, 100.0, 0.0, 100.0, 20.0, 10.0),
            keyboard_socket=None,
        )
        output = io.StringIO()
        try:
            with redirect_stdout(output):
                guard.observe(boundary_state(100.5, 50.0), wall_time=0.0)
                guard.observe(boundary_state(101.0, 50.0), wall_time=0.1)
        finally:
            guard.close()

        self.assertEqual(guard.stats.hard_violations, 1)
        self.assertEqual(guard.stats.stop_events, 1)
        self.assertEqual(guard.stats.minimum_edge_distance_m, -1.0)
        self.assertIn("crossed the finite Moon collision boundary", output.getvalue())

    def test_boundary_stop_without_keyboard_socket_records_command_error(self) -> None:
        guard = MODULE.BoundaryGuard(
            MODULE.CollisionBounds(0.0, 100.0, 0.0, 100.0, 20.0, 10.0),
            keyboard_socket=None,
            repeat_s=0.25,
        )
        output = io.StringIO()
        try:
            with redirect_stdout(output):
                guard.observe(boundary_state(90.0, 50.0), wall_time=0.0)
                guard.observe(boundary_state(90.0, 50.0), wall_time=0.249)
                guard.observe(boundary_state(90.0, 50.0), wall_time=0.25)
        finally:
            guard.close()

        self.assertEqual(guard.stats.stop_events, 1)
        self.assertEqual(guard.stats.stop_pulses, 0)
        self.assertEqual(guard.stats.command_errors, 2)

    def test_boundary_stop_with_unbound_socket_records_command_error(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            missing_socket = Path(temporary) / "missing-keyboard.sock"
            guard = MODULE.BoundaryGuard(
                MODULE.CollisionBounds(
                    0.0, 100.0, 0.0, 100.0, 20.0, 10.0
                ),
                keyboard_socket=missing_socket,
            )
            output = io.StringIO()
            try:
                with redirect_stdout(output):
                    guard.observe(boundary_state(90.0, 50.0), wall_time=0.0)
            finally:
                guard.close()

        self.assertEqual(guard.stats.stop_events, 1)
        self.assertEqual(guard.stats.stop_pulses, 0)
        self.assertEqual(guard.stats.command_errors, 1)
        self.assertIn("boundary stop command failed", output.getvalue())


if __name__ == "__main__":
    unittest.main()
