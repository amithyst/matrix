from __future__ import annotations

import math
import unittest

import numpy as np

import matrix_kungfu_recovery as recovery


class FakeRunner:
    def __init__(self, action: np.ndarray):
        self.action = np.asarray(action, dtype=np.float32)
        self.observations: list[np.ndarray] = []

    def __call__(self, observation: np.ndarray) -> np.ndarray:
        self.observations.append(np.asarray(observation, dtype=np.float32).copy())
        return self.action.copy()


def yaw_quaternion(angle: float) -> np.ndarray:
    return np.asarray(
        (math.cos(angle / 2.0), 0.0, 0.0, math.sin(angle / 2.0)),
        dtype=np.float32,
    )


def reference(
    *, yaw: float = 0.0, waist_yaw: float = 0.0
) -> recovery.KungFuReference:
    joints = recovery.DEFAULT_JOINT_POS.copy()
    joints[12] = waist_yaw
    root = yaw_quaternion(yaw)
    return recovery.KungFuReference(
        frame=12,
        joint_pos=joints,
        joint_vel=np.zeros(recovery.NUM_JOINTS, dtype=np.float32),
        root_quaternion_wxyz=root,
        torso_quaternion_wxyz=recovery._torso_quaternion(root, joints),
        source_frames=100,
        source_fps=50.0,
    )


class KungFuRecoveryPolicyTests(unittest.TestCase):
    def test_yaw_alignment_makes_matching_torso_orientation_identity(self) -> None:
        runner = FakeRunner(np.zeros(recovery.NUM_JOINTS, dtype=np.float32))
        policy = recovery.KungFuRecoveryPolicy(
            runner=runner,
            reference=reference(yaw=-1.2),
        )
        joints = recovery.DEFAULT_JOINT_POS.copy()
        policy.start(base_quat=yaw_quaternion(0.7), joint_pos=joints)
        orientation = policy.orientation_observation(
            base_quat=yaw_quaternion(0.7), joint_pos=joints
        )
        np.testing.assert_allclose(
            orientation,
            np.asarray((1.0, 0.0, 0.0, 1.0, 0.0, 0.0), dtype=np.float32),
            atol=1e-5,
        )

    def test_yaw_alignment_uses_reference_root_not_reference_torso(self) -> None:
        runner = FakeRunner(np.zeros(recovery.NUM_JOINTS, dtype=np.float32))
        policy = recovery.KungFuRecoveryPolicy(
            runner=runner,
            reference=reference(yaw=-1.2, waist_yaw=0.3),
            reference_mode=recovery.REFERENCE_MODE_SEQUENCE,
        )
        joints = recovery.DEFAULT_JOINT_POS.copy()
        policy.start(base_quat=yaw_quaternion(0.7), joint_pos=joints)
        orientation = policy.orientation_observation(
            base_quat=yaw_quaternion(0.7), joint_pos=joints
        )
        c = math.cos(0.3)
        s = math.sin(0.3)
        np.testing.assert_allclose(
            orientation,
            np.asarray((c, -s, s, c, 0.0, 0.0), dtype=np.float32),
            atol=1e-5,
        )

    def test_observation_matches_public_154_field_layout(self) -> None:
        runner = FakeRunner(np.zeros(recovery.NUM_JOINTS, dtype=np.float32))
        ref = reference()
        policy = recovery.KungFuRecoveryPolicy(runner=runner, reference=ref)
        joints = recovery.DEFAULT_JOINT_POS + np.linspace(
            -0.1, 0.1, recovery.NUM_JOINTS, dtype=np.float32
        )
        velocities = np.linspace(
            -1.0, 1.0, recovery.NUM_JOINTS, dtype=np.float32
        )
        gyro = np.asarray((0.1, -0.2, 0.3), dtype=np.float32)
        policy.start(base_quat=(1.0, 0.0, 0.0, 0.0), joint_pos=joints)
        observation = policy.build_observation(
            base_quat=(1.0, 0.0, 0.0, 0.0),
            base_ang_vel=gyro,
            joint_pos=joints,
            joint_vel=velocities,
        )
        self.assertEqual(observation.shape, (154,))
        np.testing.assert_array_equal(observation[0:29], ref.joint_pos)
        np.testing.assert_array_equal(observation[29:58], ref.joint_vel)
        np.testing.assert_array_equal(observation[64:67], gyro)
        np.testing.assert_allclose(
            observation[67:96], joints - recovery.DEFAULT_JOINT_POS
        )
        np.testing.assert_array_equal(observation[96:125], velocities)
        np.testing.assert_array_equal(
            observation[125:154], np.zeros(29, dtype=np.float32)
        )

    def test_action_maps_to_position_pd_target_and_history(self) -> None:
        action = np.linspace(-1.0, 1.0, recovery.NUM_JOINTS, dtype=np.float32)
        runner = FakeRunner(action)
        policy = recovery.KungFuRecoveryPolicy(
            runner=runner,
            reference=reference(),
        )
        policy.start(
            base_quat=(1.0, 0.0, 0.0, 0.0),
            joint_pos=recovery.DEFAULT_JOINT_POS,
        )
        first = policy.infer(
            base_quat=(1.0, 0.0, 0.0, 0.0),
            base_ang_vel=np.zeros(3, dtype=np.float32),
            joint_pos=recovery.DEFAULT_JOINT_POS,
            joint_vel=np.zeros(29, dtype=np.float32),
        )
        np.testing.assert_allclose(
            first.target_joint_pos,
            recovery.DEFAULT_JOINT_POS + recovery.ACTION_SCALE * action,
        )
        second = policy.infer(
            base_quat=(1.0, 0.0, 0.0, 0.0),
            base_ang_vel=np.zeros(3, dtype=np.float32),
            joint_pos=recovery.DEFAULT_JOINT_POS,
            joint_vel=np.zeros(29, dtype=np.float32),
        )
        np.testing.assert_array_equal(second.observation[125:154], action)

    def test_inference_advances_exact_reference_frames_and_start_resets(self) -> None:
        frames = 3
        joint_pos_sequence = np.stack(
            [
                recovery.DEFAULT_JOINT_POS + np.float32(0.01 * index)
                for index in range(frames)
            ]
        )
        joint_vel_sequence = np.stack(
            [
                np.full(
                    recovery.NUM_JOINTS,
                    np.float32(0.1 * (index + 1)),
                    dtype=np.float32,
                )
                for index in range(frames)
            ]
        )
        root_quaternion_sequence = np.stack(
            [yaw_quaternion(0.05 * index) for index in range(frames)]
        )
        initial = recovery.KungFuReference(
            frame=0,
            joint_pos=joint_pos_sequence[0].copy(),
            joint_vel=joint_vel_sequence[0].copy(),
            root_quaternion_wxyz=root_quaternion_sequence[0].copy(),
            torso_quaternion_wxyz=recovery._torso_quaternion(
                root_quaternion_sequence[0], joint_pos_sequence[0]
            ),
            source_frames=frames,
            source_fps=50.0,
            joint_pos_sequence=joint_pos_sequence,
            joint_vel_sequence=joint_vel_sequence,
            root_quaternion_sequence_wxyz=root_quaternion_sequence,
        )
        policy = recovery.KungFuRecoveryPolicy(
            runner=FakeRunner(np.zeros(29, dtype=np.float32)),
            reference=initial,
            reference_mode=recovery.REFERENCE_MODE_SEQUENCE,
        )
        policy.start(
            base_quat=(1.0, 0.0, 0.0, 0.0),
            joint_pos=recovery.DEFAULT_JOINT_POS,
        )
        first = policy.infer(
            base_quat=(1.0, 0.0, 0.0, 0.0),
            base_ang_vel=np.zeros(3, dtype=np.float32),
            joint_pos=recovery.DEFAULT_JOINT_POS,
            joint_vel=np.zeros(29, dtype=np.float32),
        )
        second = policy.infer(
            base_quat=(1.0, 0.0, 0.0, 0.0),
            base_ang_vel=np.zeros(3, dtype=np.float32),
            joint_pos=recovery.DEFAULT_JOINT_POS,
            joint_vel=np.zeros(29, dtype=np.float32),
        )
        np.testing.assert_array_equal(first.observation[0:29], joint_pos_sequence[0])
        np.testing.assert_array_equal(first.observation[29:58], joint_vel_sequence[0])
        np.testing.assert_array_equal(second.observation[0:29], joint_pos_sequence[1])
        np.testing.assert_array_equal(second.observation[29:58], joint_vel_sequence[1])
        self.assertEqual(policy.reference.frame, 2)
        self.assertFalse(policy.reference_is_frozen)

        policy.start(
            base_quat=(1.0, 0.0, 0.0, 0.0),
            joint_pos=recovery.DEFAULT_JOINT_POS,
        )
        self.assertEqual(policy.reference.frame, 0)

    def test_control_gain_scale_preserves_critical_damping_ratio(self) -> None:
        config = recovery.KungFuControlConfig.create(4.0)
        np.testing.assert_allclose(config.kp, recovery.KPS * 4.0)
        np.testing.assert_allclose(config.kd, recovery.KDS * 2.0)

    def test_inference_requires_explicit_start(self) -> None:
        policy = recovery.KungFuRecoveryPolicy(
            runner=FakeRunner(np.zeros(29, dtype=np.float32)),
            reference=reference(),
        )
        with self.assertRaisesRegex(RuntimeError, "start"):
            policy.infer(
                base_quat=(1.0, 0.0, 0.0, 0.0),
                base_ang_vel=(0.0, 0.0, 0.0),
                joint_pos=recovery.DEFAULT_JOINT_POS,
                joint_vel=np.zeros(29, dtype=np.float32),
            )


if __name__ == "__main__":
    unittest.main()
