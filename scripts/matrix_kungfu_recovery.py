#!/usr/bin/env python3
"""KungFuAthleteBot 29-DoF G1 physical fall-recovery policy adapter.

The adapter reproduces the public Unitree RL Mjlab real-deployment observation
and action contracts.  It consumes only IMU and joint feedback and returns a
29-D position-PD target.  It has no simulator handle and cannot reset, reload,
teleport, or write qpos/qvel.
"""

from __future__ import annotations

import importlib
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

import numpy as np

from matrix_policy_runtime import create_inference_session


NUM_JOINTS = 29
OBSERVATION_WIDTH = 154
POLICY_HZ = 50.0
DEFAULT_REFERENCE_FRAME = 0
TORSO_BODY_INDEX = 15
REFERENCE_MODE_SEQUENCE = "sequence"
REFERENCE_MODE_FROZEN = "frozen"

# Exact G1 29-DoF order used by Unitree hardware, Matrix DDS, the public 1307
# motion, and the exported ONNX.
JOINT_NAMES = (
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

DEFAULT_JOINT_POS = np.asarray(
    (
        -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
        -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
        0.0, 0.0, 0.0,
        0.35, 0.18, 0.0, 0.87, 0.0, 0.0, 0.0,
        0.35, -0.18, 0.0, 0.87, 0.0, 0.0, 0.0,
    ),
    dtype=np.float32,
)

# Public deploy/robots/g1 mimic gains and action mapping for checkpoint 1307.
KPS = np.asarray(
    (
        40.2, 99.1, 40.2, 99.1, 28.5, 28.5,
        40.2, 99.1, 40.2, 99.1, 28.5, 28.5,
        40.2, 28.5, 28.5,
        14.3, 14.3, 14.3, 14.3, 14.3, 16.8, 16.8,
        14.3, 14.3, 14.3, 14.3, 14.3, 16.8, 16.8,
    ),
    dtype=np.float32,
)
KDS = np.asarray(
    (
        2.6, 6.3, 2.6, 6.3, 1.8, 1.8,
        2.6, 6.3, 2.6, 6.3, 1.8, 1.8,
        2.6, 1.8, 1.8,
        0.9, 0.9, 0.9, 0.9, 0.9, 1.1, 1.1,
        0.9, 0.9, 0.9, 0.9, 0.9, 1.1, 1.1,
    ),
    dtype=np.float32,
)
ACTION_SCALE = np.asarray(
    (
        0.55, 0.35, 0.55, 0.35, 0.44, 0.44,
        0.55, 0.35, 0.55, 0.35, 0.44, 0.44,
        0.55, 0.44, 0.44,
        0.44, 0.44, 0.44, 0.44, 0.44, 0.07, 0.07,
        0.44, 0.44, 0.44, 0.44, 0.44, 0.07, 0.07,
    ),
    dtype=np.float32,
)


def _finite_vector(value: Any, size: int, label: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float32)
    if result.shape != (size,):
        raise ValueError(f"{label} must have shape ({size},), got {result.shape}")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{label} contains a non-finite value")
    return result.copy()


def _normalize_quaternion(value: Any) -> np.ndarray:
    result = _finite_vector(value, 4, "quaternion_wxyz")
    norm = float(np.linalg.norm(result))
    if norm <= 1e-8:
        raise ValueError("quaternion_wxyz has zero norm")
    return result / np.float32(norm)


def _quat_conjugate(value: Any) -> np.ndarray:
    w, x, y, z = _normalize_quaternion(value)
    return np.asarray((w, -x, -y, -z), dtype=np.float32)


def _quat_mul(left: Any, right: Any) -> np.ndarray:
    aw, ax, ay, az = _normalize_quaternion(left)
    bw, bx, by, bz = _normalize_quaternion(right)
    return _normalize_quaternion(
        np.asarray(
            (
                aw * bw - ax * bx - ay * by - az * bz,
                aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
            ),
            dtype=np.float32,
        )
    )


def _axis_angle_quaternion(axis: int, angle: float) -> np.ndarray:
    half = 0.5 * float(angle)
    result = np.zeros(4, dtype=np.float32)
    result[0] = math.cos(half)
    result[axis + 1] = math.sin(half)
    return result


def _torso_quaternion(base_quat: Any, joint_pos: Any) -> np.ndarray:
    joints = _finite_vector(joint_pos, NUM_JOINTS, "joint_pos")
    result = _normalize_quaternion(base_quat)
    result = _quat_mul(result, _axis_angle_quaternion(2, joints[12]))
    result = _quat_mul(result, _axis_angle_quaternion(0, joints[13]))
    result = _quat_mul(result, _axis_angle_quaternion(1, joints[14]))
    return result


def _yaw_quaternion(value: Any) -> np.ndarray:
    w, x, y, z = _normalize_quaternion(value)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.asarray((math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)), dtype=np.float32)


def _quat_to_matrix(value: Any) -> np.ndarray:
    w, x, y, z = _normalize_quaternion(value)
    return np.asarray(
        (
            (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
            (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
            (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
        ),
        dtype=np.float32,
    )


class PolicyRunner(Protocol):
    def __call__(self, observation: np.ndarray) -> np.ndarray: ...


class KungFuOnnxRunner:
    """ONNX Runtime adapter enforcing the public 154 -> 29 contract."""

    def __init__(self, model_path: Path, *, execution_provider: str = "cpu"):
        self.path = model_path.resolve()
        external_data = self.path.with_name(f"{self.path.name}.data")
        if not external_data.is_file():
            raise ValueError(f"KungFu ONNX external data is missing: {external_data}")
        ort = importlib.import_module("onnxruntime")
        self.session, self.execution_provider = create_inference_session(
            ort,
            str(self.path),
            execution_provider,
        )
        inputs = self.session.get_inputs()
        outputs = self.session.get_outputs()
        if len(inputs) != 1 or len(outputs) != 1:
            raise ValueError("KungFu ONNX must have exactly one input and one output")
        if inputs[0].shape[-1] != OBSERVATION_WIDTH or outputs[0].shape[-1] != NUM_JOINTS:
            raise ValueError(
                f"unexpected KungFu ONNX contract: {inputs[0].shape!r} -> "
                f"{outputs[0].shape!r}; expected 154 -> 29"
            )
        self.input_name = inputs[0].name
        self.output_name = outputs[0].name
        # Force device allocations while the worker is still writer-free.
        self(np.zeros(OBSERVATION_WIDTH, dtype=np.float32))

    def __call__(self, observation: np.ndarray) -> np.ndarray:
        batch = np.asarray(observation, dtype=np.float32).reshape(1, OBSERVATION_WIDTH)
        output = self.session.run([self.output_name], {self.input_name: batch})[0]
        return _finite_vector(np.asarray(output).reshape(-1), NUM_JOINTS, "KungFu action")


@dataclass(frozen=True)
class KungFuReference:
    frame: int
    joint_pos: np.ndarray
    joint_vel: np.ndarray
    root_quaternion_wxyz: np.ndarray
    torso_quaternion_wxyz: np.ndarray
    source_frames: int
    source_fps: float
    joint_pos_sequence: np.ndarray | None = None
    joint_vel_sequence: np.ndarray | None = None
    root_quaternion_sequence_wxyz: np.ndarray | None = None

    @classmethod
    def load(
        cls,
        motion_path: Path,
        frame: int = DEFAULT_REFERENCE_FRAME,
        *,
        zero_reference_velocity: bool = False,
    ) -> "KungFuReference":
        data = np.load(motion_path.resolve(), allow_pickle=False)
        required = {
            "fps",
            "joint_pos",
            "joint_vel",
            "body_quat_w",
        }
        missing = required.difference(data.files)
        if missing:
            raise ValueError(f"KungFu motion is missing arrays: {sorted(missing)}")
        joint_pos = np.asarray(data["joint_pos"], dtype=np.float32)
        joint_vel = np.asarray(data["joint_vel"], dtype=np.float32)
        body_quat = np.asarray(data["body_quat_w"], dtype=np.float32)
        if joint_pos.ndim != 2 or joint_pos.shape[1] != NUM_JOINTS:
            raise ValueError(f"unexpected KungFu joint_pos shape: {joint_pos.shape}")
        if joint_vel.shape != joint_pos.shape:
            raise ValueError(f"unexpected KungFu joint_vel shape: {joint_vel.shape}")
        if body_quat.ndim != 3 or body_quat.shape[0] != joint_pos.shape[0] or body_quat.shape[2] != 4:
            raise ValueError(f"unexpected KungFu body_quat_w shape: {body_quat.shape}")
        if not (
            np.all(np.isfinite(joint_pos))
            and np.all(np.isfinite(joint_vel))
            and np.all(np.isfinite(body_quat[:, 0, :]))
        ):
            raise ValueError("KungFu motion contains a non-finite value")
        index = int(frame)
        if index < 0 or index >= joint_pos.shape[0]:
            raise ValueError(f"KungFu reference frame is out of range: {index}")
        fps = float(np.asarray(data["fps"]).reshape(-1)[0])
        if not math.isfinite(fps) or abs(fps - POLICY_HZ) > 1e-6:
            raise ValueError(f"KungFu motion must be 50 Hz, got {fps}")
        reference_joint_pos = _finite_vector(joint_pos[index], NUM_JOINTS, "reference joint_pos")
        sequence_joint_vel = joint_vel.copy()
        if zero_reference_velocity:
            sequence_joint_vel.fill(0.0)
        reference_joint_vel = _finite_vector(
            sequence_joint_vel[index], NUM_JOINTS, "reference joint_vel"
        )
        root_quat = _normalize_quaternion(body_quat[index, 0])
        # Match upstream State_Mimic.cpp exactly: root orientation followed by
        # waist yaw/roll/pitch, rather than depending on a body-index convention.
        torso_quat = _torso_quaternion(root_quat, reference_joint_pos)
        return cls(
            frame=index,
            joint_pos=reference_joint_pos,
            joint_vel=reference_joint_vel,
            root_quaternion_wxyz=root_quat,
            torso_quaternion_wxyz=torso_quat,
            source_frames=int(joint_pos.shape[0]),
            source_fps=fps,
            joint_pos_sequence=joint_pos.copy(),
            joint_vel_sequence=sequence_joint_vel,
            root_quaternion_sequence_wxyz=body_quat[:, 0, :].copy(),
        )

    @property
    def is_sequence(self) -> bool:
        return (
            self.joint_pos_sequence is not None
            and self.joint_vel_sequence is not None
            and self.root_quaternion_sequence_wxyz is not None
        )

    def at_frame(self, frame: int) -> "KungFuReference":
        """Return the exact 50 Hz reference frame, clamped like State_Mimic."""

        if not self.is_sequence:
            return self
        assert self.joint_pos_sequence is not None
        assert self.joint_vel_sequence is not None
        assert self.root_quaternion_sequence_wxyz is not None
        index = min(max(int(frame), 0), self.source_frames - 1)
        reference_joint_pos = _finite_vector(
            self.joint_pos_sequence[index], NUM_JOINTS, "reference joint_pos"
        )
        reference_joint_vel = _finite_vector(
            self.joint_vel_sequence[index], NUM_JOINTS, "reference joint_vel"
        )
        root_quat = _normalize_quaternion(
            self.root_quaternion_sequence_wxyz[index]
        )
        return KungFuReference(
            frame=index,
            joint_pos=reference_joint_pos,
            joint_vel=reference_joint_vel,
            root_quaternion_wxyz=root_quat,
            torso_quaternion_wxyz=_torso_quaternion(
                root_quat, reference_joint_pos
            ),
            source_frames=self.source_frames,
            source_fps=self.source_fps,
            joint_pos_sequence=self.joint_pos_sequence,
            joint_vel_sequence=self.joint_vel_sequence,
            root_quaternion_sequence_wxyz=(
                self.root_quaternion_sequence_wxyz
            ),
        )

    def next_frame(self) -> "KungFuReference":
        return self.at_frame(self.frame + 1)


@dataclass(frozen=True)
class KungFuControlConfig:
    kp: np.ndarray
    kd: np.ndarray

    @classmethod
    def create(cls, gain_scale: float = 1.0) -> "KungFuControlConfig":
        scale = float(gain_scale)
        if not math.isfinite(scale) or scale <= 0.0:
            raise ValueError("gain_scale must be finite and positive")
        return cls(
            kp=(KPS * np.float32(scale)).astype(np.float32),
            kd=(KDS * np.float32(math.sqrt(scale))).astype(np.float32),
        )


@dataclass(frozen=True)
class KungFuPolicyOutput:
    target_joint_pos: np.ndarray
    raw_action: np.ndarray
    observation: np.ndarray


class KungFuRecoveryPolicy:
    """Pure state-feedback recovery policy tracking the public 50 Hz motion."""

    def __init__(
        self,
        *,
        runner: PolicyRunner,
        reference: KungFuReference,
        config: KungFuControlConfig | None = None,
        reference_mode: str = REFERENCE_MODE_FROZEN,
    ) -> None:
        if reference_mode not in {
            REFERENCE_MODE_SEQUENCE,
            REFERENCE_MODE_FROZEN,
        }:
            raise ValueError(f"unsupported KungFu reference mode: {reference_mode}")
        self.runner = runner
        self._initial_reference = reference
        self.reference = reference
        self.reference_mode = reference_mode
        self.config = config or KungFuControlConfig.create()
        self.previous_action = np.zeros(NUM_JOINTS, dtype=np.float32)
        self._reference_alignment: np.ndarray | None = None

    @classmethod
    def from_artifacts(
        cls,
        *,
        model_path: Path,
        motion_path: Path,
        reference_frame: int = DEFAULT_REFERENCE_FRAME,
        gain_scale: float = 1.0,
        execution_provider: str = "cpu",
    ) -> "KungFuRecoveryPolicy":
        # Frame zero is the exact public State_Mimic deployment.  A positive
        # frame intentionally selects a fixed recovery target for controlled
        # A/B fallback experiments; its reference velocity is zeroed.
        reference_mode = (
            REFERENCE_MODE_SEQUENCE
            if reference_frame == 0
            else REFERENCE_MODE_FROZEN
        )
        return cls(
            runner=KungFuOnnxRunner(
                model_path,
                execution_provider=execution_provider,
            ),
            reference=KungFuReference.load(
                motion_path,
                reference_frame,
                zero_reference_velocity=(
                    reference_mode == REFERENCE_MODE_FROZEN
                ),
            ),
            config=KungFuControlConfig.create(gain_scale),
            reference_mode=reference_mode,
        )

    def start(self, *, base_quat: Any, joint_pos: Any) -> None:
        self.reference = self._initial_reference
        current_torso = _torso_quaternion(base_quat, joint_pos)
        robot_yaw = _yaw_quaternion(current_torso)
        # Sequence mode matches upstream State_Mimic exactly: current torso yaw
        # is aligned to reference root yaw.  Frozen mode preserves the already
        # validated v18 recovery-target A/B semantics.
        reference_yaw = _yaw_quaternion(
            self.reference.root_quaternion_wxyz
            if self.reference_mode == REFERENCE_MODE_SEQUENCE
            else self.reference.torso_quaternion_wxyz
        )
        self._reference_alignment = _quat_mul(robot_yaw, _quat_conjugate(reference_yaw))
        self.previous_action.fill(0.0)

    @property
    def reference_is_frozen(self) -> bool:
        return self.reference_mode == REFERENCE_MODE_FROZEN

    def orientation_observation(self, *, base_quat: Any, joint_pos: Any) -> np.ndarray:
        if self._reference_alignment is None:
            raise RuntimeError("call start() before inference")
        current_torso = _torso_quaternion(base_quat, joint_pos)
        aligned_reference = _quat_mul(
            self._reference_alignment, self.reference.torso_quaternion_wxyz
        )
        # Upstream computes inverse(reference)*current, transposes the matrix,
        # then selects the first two columns.  This is inverse(current)*reference.
        current_to_reference = _quat_mul(
            _quat_conjugate(current_torso), aligned_reference
        )
        rotation = _quat_to_matrix(current_to_reference)
        return rotation[:, :2].reshape(-1).astype(np.float32, copy=False)

    def build_observation(
        self,
        *,
        base_quat: Any,
        base_ang_vel: Any,
        joint_pos: Any,
        joint_vel: Any,
    ) -> np.ndarray:
        positions = _finite_vector(joint_pos, NUM_JOINTS, "joint_pos")
        velocities = _finite_vector(joint_vel, NUM_JOINTS, "joint_vel")
        angular_velocity = _finite_vector(base_ang_vel, 3, "base_ang_vel")
        orientation = self.orientation_observation(
            base_quat=base_quat, joint_pos=positions
        )
        observation = np.concatenate(
            (
                self.reference.joint_pos,
                self.reference.joint_vel,
                orientation,
                angular_velocity,
                positions - DEFAULT_JOINT_POS,
                velocities,
                self.previous_action,
            )
        ).astype(np.float32, copy=False)
        if observation.shape != (OBSERVATION_WIDTH,):
            raise AssertionError(f"internal KungFu observation shape error: {observation.shape}")
        if not np.all(np.isfinite(observation)):
            raise ValueError("KungFu observation contains a non-finite value")
        return observation

    def infer(
        self,
        *,
        base_quat: Any,
        base_ang_vel: Any,
        joint_pos: Any,
        joint_vel: Any,
    ) -> KungFuPolicyOutput:
        observation = self.build_observation(
            base_quat=base_quat,
            base_ang_vel=base_ang_vel,
            joint_pos=joint_pos,
            joint_vel=joint_vel,
        )
        raw_action = _finite_vector(
            self.runner(observation[None, :]), NUM_JOINTS, "KungFu raw action"
        )
        target = DEFAULT_JOINT_POS + ACTION_SCALE * raw_action
        if not np.all(np.isfinite(target)):
            raise ValueError("KungFu target contains a non-finite value")
        self.previous_action[:] = raw_action
        output = KungFuPolicyOutput(
            target_joint_pos=target.astype(np.float32, copy=True),
            raw_action=raw_action.copy(),
            observation=observation.copy(),
        )
        # Official State_Mimic advances floor(t / 0.02) without interpolation
        # and clamps at the last frame.  The current frame produced this action;
        # the next 50 Hz inference consumes the following frame.
        if self.reference_mode == REFERENCE_MODE_SEQUENCE:
            self.reference = self.reference.next_frame()
        return output
