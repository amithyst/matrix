#!/usr/bin/env python3
"""Validate a G1 AMP get-up policy in MuJoCo using physical knockdowns only.

The robot is initialized once in its nominal standing pose.  After the
controller has settled, ``mj_applyFT`` supplies a short horizontal force at
the pelvis.  The recovery phase never writes qpos, calls mj_resetData, reloads
the model, or otherwise teleports the robot.

This script intentionally runs outside the Matrix/SONIC DDS process graph.  It
is the first gate for a policy that may later become a temporary LowCmd owner.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence

import mujoco
import numpy as np
import onnxruntime as ort

from matrix_mujoco_contacts import has_external_foot_support


DEFAULT_DIRECTIONS = ("forward", "backward", "left", "right")
KNOCKDOWN_VECTORS = {
    "forward": np.asarray((1.0, 0.0, 0.0), dtype=np.float64),
    "backward": np.asarray((-1.0, 0.0, 0.0), dtype=np.float64),
    "left": np.asarray((0.0, 1.0, 0.0), dtype=np.float64),
    "right": np.asarray((0.0, -1.0, 0.0), dtype=np.float64),
}


def _array(config: dict[str, Any], key: str, size: int) -> np.ndarray:
    value = np.asarray(config[key], dtype=np.float64)
    if value.shape != (size,):
        raise ValueError(f"{key} must contain {size} values, got {value.shape}")
    if not np.all(np.isfinite(value)):
        raise ValueError(f"{key} contains a non-finite value")
    return value


def _quat_rotate_inverse(quat_wxyz: np.ndarray, vector: np.ndarray) -> np.ndarray:
    """Rotate a world-frame vector into the quaternion's body frame."""

    quat = np.asarray(quat_wxyz, dtype=np.float64)
    norm = float(np.linalg.norm(quat))
    if norm <= 1e-12:
        raise ValueError("root quaternion has zero norm")
    w, x, y, z = quat / norm
    # R(q).T @ vector, expanded to avoid a scipy dependency in the worker.
    rotation_transpose = np.asarray(
        (
            (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y + w * z), 2.0 * (x * z - w * y)),
            (2.0 * (x * y - w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z + w * x)),
            (2.0 * (x * z + w * y), 2.0 * (y * z - w * x), 1.0 - 2.0 * (x * x + y * y)),
        ),
        dtype=np.float64,
    )
    return rotation_transpose @ vector


def _root_up_z(quat_wxyz: np.ndarray) -> float:
    quat = np.asarray(quat_wxyz, dtype=np.float64)
    quat /= max(float(np.linalg.norm(quat)), 1e-12)
    _w, x, y, _z = quat
    return float(1.0 - 2.0 * (x * x + y * y))


@dataclass(frozen=True)
class JointMapping:
    names: tuple[str, ...]
    joint_ids: np.ndarray
    qpos_addresses: np.ndarray
    qvel_addresses: np.ndarray
    actuator_ids: np.ndarray


def _joint_mapping(model: mujoco.MjModel, names: Sequence[str]) -> JointMapping:
    joint_ids: list[int] = []
    qpos_addresses: list[int] = []
    qvel_addresses: list[int] = []
    actuator_ids: list[int] = []

    actuator_by_joint: dict[int, int] = {}
    for actuator_id in range(model.nu):
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        if joint_id >= 0:
            actuator_by_joint.setdefault(joint_id, actuator_id)

    for name in names:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise ValueError(f"MuJoCo model is missing policy joint {name!r}")
        joint_type = int(model.jnt_type[joint_id])
        if joint_type not in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
            raise ValueError(f"policy joint {name!r} is not a one-DoF joint")
        if joint_id not in actuator_by_joint:
            raise ValueError(f"MuJoCo model has no motor actuator for policy joint {name!r}")
        joint_ids.append(joint_id)
        qpos_addresses.append(int(model.jnt_qposadr[joint_id]))
        qvel_addresses.append(int(model.jnt_dofadr[joint_id]))
        actuator_ids.append(actuator_by_joint[joint_id])

    if len(set(actuator_ids)) != len(actuator_ids):
        raise ValueError("policy joints do not map one-to-one onto MuJoCo actuators")
    return JointMapping(
        names=tuple(names),
        joint_ids=np.asarray(joint_ids, dtype=np.int32),
        qpos_addresses=np.asarray(qpos_addresses, dtype=np.int32),
        qvel_addresses=np.asarray(qvel_addresses, dtype=np.int32),
        actuator_ids=np.asarray(actuator_ids, dtype=np.int32),
    )


class AmpPolicy:
    def __init__(self, config: dict[str, Any], model_path: Path):
        self.joint_names = tuple(config["policy_joint_names"])
        self.num_actions = len(self.joint_names)
        self.default_joint_pos = _array(config, "default_joint_pos", self.num_actions)
        self.action_scale = _array(config, "action_scale", self.num_actions)
        self.kp = _array(config, "stiffness", self.num_actions)
        self.kd = _array(config, "damping", self.num_actions)
        self.effort_limit = _array(config, "effort_limit", self.num_actions)
        self.x1 = _array(config, "X1", self.num_actions)
        self.x2 = _array(config, "X2", self.num_actions)
        self.y1 = _array(config, "Y1", self.num_actions)
        self.y2 = _array(config, "Y2", self.num_actions)
        self.friction_static = _array(config, "Fs", self.num_actions)
        self.friction_dynamic = _array(config, "Fd", self.num_actions)
        self.friction_activation_velocity = _array(config, "Va", self.num_actions)
        self.action_clip = float(config.get("action_clip", 10.0))
        self.history_length = int(config["obs_config"]["history_length"])
        if self.history_length <= 0:
            raise ValueError("history_length must be positive")
        if not config.get("obs_joint_pos_relative", False):
            raise ValueError("this policy validator requires relative joint positions")
        expected_obs = [
            "RootAngVelB",
            "ProjectedGravityB",
            "Command",
            "JointPos",
            "JointVel",
            "PrevActions",
        ]
        actual_obs = [entry["name"] for entry in config["obs_config"]["policy"]]
        if actual_obs != expected_obs:
            raise ValueError(f"unsupported observation order: {actual_obs!r}")

        self.session = ort.InferenceSession(
            str(model_path), providers=["CPUExecutionProvider"]
        )
        inputs = self.session.get_inputs()
        outputs = self.session.get_outputs()
        if len(inputs) != 1 or len(outputs) < 1:
            raise ValueError("expected a single-input AMP ONNX policy")
        self.input_name = inputs[0].name
        self.output_name = outputs[0].name
        expected_width = (9 + 3 * self.num_actions) * self.history_length
        input_width = inputs[0].shape[-1]
        if isinstance(input_width, int) and input_width != expected_width:
            raise ValueError(
                f"ONNX input width {input_width} does not match expected {expected_width}"
            )

        self.last_action = np.zeros(self.num_actions, dtype=np.float32)
        self.history: list[np.ndarray] = []

    def reset_history(
        self,
        *,
        root_quat: np.ndarray,
        root_angvel: np.ndarray,
        joint_pos: np.ndarray,
        joint_vel: np.ndarray,
    ) -> None:
        self.last_action.fill(0.0)
        observation = self._observation(
            root_quat=root_quat,
            root_angvel=root_angvel,
            joint_pos=joint_pos,
            joint_vel=joint_vel,
        )
        self.history = [observation.copy() for _ in range(self.history_length)]

    def _observation(
        self,
        *,
        root_quat: np.ndarray,
        root_angvel: np.ndarray,
        joint_pos: np.ndarray,
        joint_vel: np.ndarray,
    ) -> np.ndarray:
        return np.concatenate(
            (
                np.asarray(root_angvel, dtype=np.float32),
                _quat_rotate_inverse(
                    root_quat, np.asarray((0.0, 0.0, -1.0), dtype=np.float64)
                ).astype(np.float32),
                np.zeros(3, dtype=np.float32),
                (joint_pos - self.default_joint_pos).astype(np.float32),
                np.asarray(joint_vel, dtype=np.float32),
                self.last_action,
            )
        )

    def infer(
        self,
        *,
        root_quat: np.ndarray,
        root_angvel: np.ndarray,
        joint_pos: np.ndarray,
        joint_vel: np.ndarray,
    ) -> np.ndarray:
        observation = self._observation(
            root_quat=root_quat,
            root_angvel=root_angvel,
            joint_pos=joint_pos,
            joint_vel=joint_vel,
        )
        if not self.history:
            self.history = [observation.copy() for _ in range(self.history_length)]
        else:
            self.history.pop(0)
            self.history.append(observation)
        policy_input = np.concatenate(self.history).astype(np.float32, copy=False)[None, :]
        raw_action = self.session.run(
            [self.output_name], {self.input_name: policy_input}
        )[0][0]
        if raw_action.shape != (self.num_actions,) or not np.all(np.isfinite(raw_action)):
            raise RuntimeError(f"policy returned invalid action shape/value: {raw_action!r}")
        self.last_action[:] = np.clip(raw_action, -self.action_clip, self.action_clip)
        return self.default_joint_pos + self.action_scale * self.last_action

    def torque(
        self,
        target: np.ndarray,
        joint_pos: np.ndarray,
        joint_vel: np.ndarray,
    ) -> np.ndarray:
        torque = self.kp * (target - joint_pos) - self.kd * joint_vel
        same_direction = joint_vel * torque > 0.0
        max_effort = np.minimum(
            np.where(same_direction, self.y1, self.y2), self.effort_limit
        )
        fast = np.abs(joint_vel) >= self.x1
        denominator = np.maximum(self.x2 - self.x1, 1e-6)
        speed_limited = np.maximum(
            (-max_effort / denominator) * (np.abs(joint_vel) - self.x1)
            + max_effort,
            0.0,
        )
        max_effort = np.where(fast, speed_limited, max_effort)
        torque = np.clip(torque, -max_effort, max_effort)
        activation = np.maximum(self.friction_activation_velocity, 1e-6)
        torque -= self.friction_static * np.tanh(joint_vel / activation)
        torque -= self.friction_dynamic * joint_vel
        return torque


def _free_joint_addresses(model: mujoco.MjModel) -> tuple[int, int]:
    for joint_id in range(model.njnt):
        if int(model.jnt_type[joint_id]) == mujoco.mjtJoint.mjJNT_FREE:
            return int(model.jnt_qposadr[joint_id]), int(model.jnt_dofadr[joint_id])
    raise ValueError("MuJoCo model has no free root joint")


def _pelvis_body_id(model: mujoco.MjModel) -> int:
    for name in ("pelvis", "base"):
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id > 0:
            return int(body_id)
    raise ValueError("MuJoCo model has no pelvis/base body")


def _foot_body_ids(model: mujoco.MjModel) -> set[int]:
    result: set[int] = set()
    for name in (
        "left_ankle_roll_link",
        "right_ankle_roll_link",
        "left_foot",
        "right_foot",
    ):
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id > 0:
            result.add(int(body_id))
    return result


def _has_foot_contact(
    model: mujoco.MjModel, data: mujoco.MjData, foot_body_ids: set[int]
) -> bool:
    return has_external_foot_support(
        model,
        data,
        foot_body_ids=foot_body_ids,
        robot_root_body_id=_pelvis_body_id(model),
    )


def _initialize_nominal_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    mapping: JointMapping,
    config: dict[str, Any],
) -> None:
    """Perform the run's only qpos/qvel writes, before physical time starts."""

    initial = config["initial_state"]
    root_qpos, _root_dof = _free_joint_addresses(model)
    data.qpos[root_qpos : root_qpos + 3] = np.asarray(initial["root_pos"], dtype=float)
    data.qpos[root_qpos + 3 : root_qpos + 7] = np.asarray(
        initial["root_quat"], dtype=float
    )
    data.qpos[mapping.qpos_addresses] = np.asarray(
        initial["joint_pos_array"], dtype=float
    )
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)


def _direction_list(value: str) -> tuple[str, ...]:
    values = tuple(part.strip().lower() for part in value.split(",") if part.strip())
    unknown = [item for item in values if item not in KNOCKDOWN_VECTORS]
    if unknown:
        raise argparse.ArgumentTypeError(f"unknown knockdown directions: {unknown}")
    if not values:
        raise argparse.ArgumentTypeError("at least one knockdown direction is required")
    return values


def run_trial(
    *,
    xml_path: Path,
    config: dict[str, Any],
    model_path: Path,
    direction: str,
    physics_hz: float | None,
    settle_seconds: float,
    knockdown_force_newtons: float,
    knockdown_seconds: float,
    knockdown_substeps: int | None,
    recovery_seconds: float,
    stable_seconds: float,
    max_root_linear_speed: float,
    max_root_angular_speed: float,
    max_joint_speed_rms: float,
) -> dict[str, Any]:
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    if physics_hz is not None:
        if not math.isfinite(physics_hz) or physics_hz <= 0.0:
            raise ValueError("physics_hz must be positive and finite")
        model.opt.timestep = 1.0 / physics_hz
    data = mujoco.MjData(model)
    policy = AmpPolicy(config, model_path)
    mapping = _joint_mapping(model, policy.joint_names)
    root_qpos, root_dof = _free_joint_addresses(model)
    pelvis_body_id = _pelvis_body_id(model)
    foot_body_ids = _foot_body_ids(model)
    _initialize_nominal_pose(model, data, mapping, config)

    timestep = float(model.opt.timestep)
    control_dt = float(config["sim"].get("control_dt", 0.02))
    decimation = max(1, int(round(control_dt / timestep)))
    if decimation <= 0:
        raise ValueError("policy decimation must be positive")

    joint_pos = np.asarray(data.qpos[mapping.qpos_addresses], dtype=np.float64)
    joint_vel = np.asarray(data.qvel[mapping.qvel_addresses], dtype=np.float64)
    policy.reset_history(
        root_quat=np.asarray(data.qpos[root_qpos + 3 : root_qpos + 7]),
        root_angvel=np.asarray(data.qvel[root_dof + 3 : root_dof + 6]),
        joint_pos=joint_pos,
        joint_vel=joint_vel,
    )

    settle_steps = int(round(settle_seconds / timestep))
    knockdown_steps = (
        int(knockdown_substeps)
        if knockdown_substeps is not None
        else int(round(knockdown_seconds / timestep))
    )
    if knockdown_steps <= 0:
        raise ValueError("knockdown_substeps must be positive")
    recovery_steps = int(round(recovery_seconds / timestep))
    stable_steps_required = int(round(stable_seconds / timestep))
    total_steps = settle_steps + knockdown_steps + recovery_steps
    direction_vector = KNOCKDOWN_VECTORS[direction]
    force_vector = direction_vector * knockdown_force_newtons
    zero_torque = np.zeros(3, dtype=np.float64)
    target = policy.default_joint_pos.copy()

    min_root_z = math.inf
    min_root_up_z = math.inf
    fallen = False
    fall_step: int | None = None
    recovered_step: int | None = None
    stable_steps = 0
    foot_contact_during_stable = False
    samples: list[dict[str, float]] = []

    # No qpos/qvel assignments and no reset/reload are allowed beyond this line.
    for step in range(total_steps):
        joint_pos = np.asarray(data.qpos[mapping.qpos_addresses], dtype=np.float64)
        joint_vel = np.asarray(data.qvel[mapping.qvel_addresses], dtype=np.float64)
        if step % decimation == 0:
            target = policy.infer(
                root_quat=np.asarray(data.qpos[root_qpos + 3 : root_qpos + 7]),
                root_angvel=np.asarray(data.qvel[root_dof + 3 : root_dof + 6]),
                joint_pos=joint_pos,
                joint_vel=joint_vel,
            )

        torque = policy.torque(target, joint_pos, joint_vel)
        for policy_index, actuator_id in enumerate(mapping.actuator_ids):
            value = float(torque[policy_index])
            if bool(model.actuator_ctrllimited[actuator_id]):
                low, high = model.actuator_ctrlrange[actuator_id]
                value = min(max(value, float(low)), float(high))
            data.ctrl[actuator_id] = value

        data.qfrc_applied[:] = 0.0
        if settle_steps <= step < settle_steps + knockdown_steps:
            point = np.asarray(data.xpos[pelvis_body_id], dtype=np.float64).copy()
            mujoco.mj_applyFT(
                model,
                data,
                force_vector,
                zero_torque,
                point,
                pelvis_body_id,
                data.qfrc_applied,
            )
        mujoco.mj_step(model, data)

        root_z = float(data.qpos[root_qpos + 2])
        root_quat = np.asarray(data.qpos[root_qpos + 3 : root_qpos + 7])
        up_z = _root_up_z(root_quat)
        root_linear_speed = float(
            np.linalg.norm(data.qvel[root_dof : root_dof + 3])
        )
        root_angular_speed = float(
            np.linalg.norm(data.qvel[root_dof + 3 : root_dof + 6])
        )
        joint_speed_rms = float(
            math.sqrt(np.mean(np.square(data.qvel[mapping.qvel_addresses])))
        )
        min_root_z = min(min_root_z, root_z)
        min_root_up_z = min(min_root_up_z, up_z)

        after_force = step >= settle_steps + knockdown_steps
        if after_force and not fallen and (root_z < 0.45 or up_z < 0.5):
            fallen = True
            fall_step = step

        stable_now = (
            fallen
            and root_z >= 0.65
            and up_z >= 0.85
            and root_linear_speed < max_root_linear_speed
            and root_angular_speed < max_root_angular_speed
            and joint_speed_rms < max_joint_speed_rms
            and _has_foot_contact(model, data, foot_body_ids)
        )
        if stable_now:
            stable_steps += 1
            foot_contact_during_stable = True
            if stable_steps >= stable_steps_required and recovered_step is None:
                recovered_step = step
        else:
            stable_steps = 0

        if step % max(1, int(round(0.25 / timestep))) == 0:
            samples.append(
                {
                    "time_s": round(step * timestep, 6),
                    "root_z": root_z,
                    "root_up_z": up_z,
                    "root_linear_speed_mps": root_linear_speed,
                    "root_angular_speed_radps": root_angular_speed,
                    "joint_speed_rms_radps": joint_speed_rms,
                }
            )

    passed = fallen and recovered_step is not None
    final_root_quat = np.asarray(data.qpos[root_qpos + 3 : root_qpos + 7])
    final_joint_speed = np.asarray(data.qvel[mapping.qvel_addresses])
    return {
        "direction": direction,
        "passed": passed,
        "physical_knockdown_only": True,
        "qpos_writes_after_physics_start": 0,
        "reset_calls_after_physics_start": 0,
        "fallen": fallen,
        "fall_time_s": None if fall_step is None else fall_step * timestep,
        "recovered_time_s": (
            None if recovered_step is None else recovered_step * timestep
        ),
        "stable_hold_required_s": stable_seconds,
        "stability_thresholds": {
            "root_linear_speed_mps": max_root_linear_speed,
            "root_angular_speed_radps": max_root_angular_speed,
            "joint_speed_rms_radps": max_joint_speed_rms,
        },
        "foot_contact_during_stable": foot_contact_during_stable,
        "minimum_root_z": min_root_z,
        "minimum_root_up_z": min_root_up_z,
        "final_root_z": float(data.qpos[root_qpos + 2]),
        "final_root_up_z": _root_up_z(final_root_quat),
        "final_root_linear_speed_mps": float(
            np.linalg.norm(data.qvel[root_dof : root_dof + 3])
        ),
        "final_root_angular_speed_radps": float(
            np.linalg.norm(data.qvel[root_dof + 3 : root_dof + 6])
        ),
        "final_joint_speed_rms_radps": float(
            math.sqrt(np.mean(np.square(final_joint_speed)))
        ),
        "model_timestep_s": timestep,
        "policy_control_dt_s": control_dt,
        "policy_decimation": decimation,
        "knockdown_substeps": knockdown_steps,
        "knockdown_duration_s": knockdown_steps * timestep,
        "samples": samples,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xml", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument(
        "--directions",
        type=_direction_list,
        default=DEFAULT_DIRECTIONS,
        help="comma-separated: forward,backward,left,right",
    )
    parser.add_argument("--settle-seconds", type=float, default=2.0)
    parser.add_argument(
        "--physics-hz",
        type=float,
        help="override the MJCF timestep before creating MjData",
    )
    parser.add_argument("--knockdown-force-newtons", type=float, default=3400.0)
    parser.add_argument(
        "--knockdown-seconds",
        type=float,
        default=0.028,
        help="physical force duration when --knockdown-substeps is omitted",
    )
    parser.add_argument(
        "--knockdown-substeps",
        type=int,
        help="explicit physical mj_applyFT steps; overrides --knockdown-seconds",
    )
    parser.add_argument("--recovery-seconds", type=float, default=18.0)
    parser.add_argument("--stable-seconds", type=float, default=1.5)
    parser.add_argument("--max-root-linear-speed", type=float, default=0.15)
    parser.add_argument("--max-root-angular-speed", type=float, default=0.30)
    parser.add_argument("--max-joint-speed-rms", type=float, default=0.50)
    parser.add_argument("--output", type=Path)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    with args.config.open("r", encoding="utf-8") as handle:
        config = json.load(handle)

    trials: list[dict[str, Any]] = []
    for direction in args.directions:
        print(f"[AMP GET-UP] testing physical knockdown: {direction}", flush=True)
        trial = run_trial(
            xml_path=args.xml.resolve(),
            config=config,
            model_path=args.model.resolve(),
            direction=direction,
            physics_hz=args.physics_hz,
            settle_seconds=args.settle_seconds,
            knockdown_force_newtons=args.knockdown_force_newtons,
            knockdown_seconds=args.knockdown_seconds,
            knockdown_substeps=args.knockdown_substeps,
            recovery_seconds=args.recovery_seconds,
            stable_seconds=args.stable_seconds,
            max_root_linear_speed=args.max_root_linear_speed,
            max_root_angular_speed=args.max_root_angular_speed,
            max_joint_speed_rms=args.max_joint_speed_rms,
        )
        trials.append(trial)
        print(
            "[AMP GET-UP] "
            f"{direction}: passed={trial['passed']} "
            f"fallen={trial['fallen']} root_z={trial['final_root_z']:.3f} "
            f"up_z={trial['final_root_up_z']:.3f}",
            flush=True,
        )

    result = {
        "schema": "matrix.g1_amp_getup_validation.v1",
        "passed": all(trial["passed"] for trial in trials),
        "xml": str(args.xml.resolve()),
        "config": str(args.config.resolve()),
        "model": str(args.model.resolve()),
        "constraints": {
            "physical_knockdown_only": True,
            "qpos_writes_after_physics_start": 0,
            "reset_calls_after_physics_start": 0,
            "teleport_or_reload_during_recovery": False,
        },
        "trials": trials,
    }
    encoded = json.dumps(result, indent=2, sort_keys=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded + "\n", encoding="utf-8")
    print(encoded)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
