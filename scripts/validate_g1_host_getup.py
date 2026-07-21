#!/usr/bin/env python3
"""Validate a HoST G1 get-up policy after a purely physical knockdown.

An AMP controller keeps the robot standing while ``mj_applyFT`` knocks it
down.  Once a fallen pose is observed, the selected HoST policy takes over by
joint PD targets only.  Recovery never writes qpos/qvel or calls reset/reload.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any, Sequence

import mujoco
import numpy as np
import onnxruntime as ort

from validate_g1_amp_getup import (
    AmpPolicy,
    KNOCKDOWN_VECTORS,
    _foot_body_ids,
    _free_joint_addresses,
    _has_foot_contact,
    _initialize_nominal_pose,
    _joint_mapping,
    _pelvis_body_id,
    _quat_rotate_inverse,
    _root_up_z,
)


HOST_TO_MATRIX_INDICES = np.asarray(
    (*range(0, 13), *range(15, 20), *range(22, 27)), dtype=np.int32
)
MATRIX_JOINT_NAMES = (
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


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _host_gains() -> tuple[np.ndarray, np.ndarray]:
    kp = np.full(29, 100.0, dtype=np.float64)
    kd = np.full(29, 4.0, dtype=np.float64)
    for index in (0, 1, 2, 6, 7, 8):
        kp[index] = 150.0
    for index in (3, 9):
        kp[index] = 200.0
        kd[index] = 6.0
    for index in (4, 5, 10, 11):
        kp[index] = 40.0
        kd[index] = 2.0
    return kp, kd


class HostPolicy:
    """HoST's public 6x76 observation and incremental target contract."""

    def __init__(self, model_path: Path, *, action_rescale: float = 0.25):
        self.action_rescale = float(action_rescale)
        if not math.isfinite(self.action_rescale) or self.action_rescale <= 0.0:
            raise ValueError("action_rescale must be positive and finite")
        self.session = ort.InferenceSession(
            str(model_path), providers=["CPUExecutionProvider"]
        )
        inputs = self.session.get_inputs()
        outputs = self.session.get_outputs()
        if len(inputs) != 1 or len(outputs) != 1:
            raise ValueError("HoST policy must have exactly one input and one output")
        self.input_name = inputs[0].name
        self.output_name = outputs[0].name
        if inputs[0].shape[-1] != 456 or outputs[0].shape[-1] != 23:
            raise ValueError(
                f"unexpected HoST ONNX contract: {inputs[0].shape} -> {outputs[0].shape}"
            )
        self.last_action = np.zeros(23, dtype=np.float32)
        self.history: list[np.ndarray] = []

    def _observation(
        self,
        *,
        root_quat: np.ndarray,
        root_angvel: np.ndarray,
        joint_pos: np.ndarray,
        joint_vel: np.ndarray,
    ) -> np.ndarray:
        controlled_pos = np.asarray(joint_pos[HOST_TO_MATRIX_INDICES], dtype=np.float32)
        controlled_vel = np.asarray(joint_vel[HOST_TO_MATRIX_INDICES], dtype=np.float32)
        return np.concatenate(
            (
                np.asarray(root_angvel, dtype=np.float32) * 0.25,
                _quat_rotate_inverse(
                    root_quat, np.asarray((0.0, 0.0, -1.0), dtype=np.float64)
                ).astype(np.float32),
                controlled_pos,
                controlled_vel * 0.05,
                self.last_action,
                np.asarray((self.action_rescale,), dtype=np.float32),
            )
        )

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
        self.history = [observation.copy() for _ in range(6)]

    def infer(
        self,
        *,
        root_quat: np.ndarray,
        root_angvel: np.ndarray,
        joint_pos: np.ndarray,
        joint_vel: np.ndarray,
        held_joint_target: np.ndarray,
    ) -> np.ndarray:
        observation = self._observation(
            root_quat=root_quat,
            root_angvel=root_angvel,
            joint_pos=joint_pos,
            joint_vel=joint_vel,
        )
        if not self.history:
            self.history = [observation.copy() for _ in range(6)]
        else:
            self.history.pop(0)
            self.history.append(observation)
        policy_input = np.concatenate(self.history).astype(np.float32, copy=False)[None, :]
        action = self.session.run(
            [self.output_name], {self.input_name: policy_input}
        )[0][0]
        if action.shape != (23,) or not np.all(np.isfinite(action)):
            raise RuntimeError(f"HoST returned invalid action: {action!r}")
        self.last_action[:] = np.clip(action, -100.0, 100.0)
        target = np.asarray(held_joint_target, dtype=np.float64).copy()
        target[HOST_TO_MATRIX_INDICES] = (
            joint_pos[HOST_TO_MATRIX_INDICES]
            + self.action_rescale * self.last_action
        )
        return target


def _model_family(model_path: Path) -> str:
    name = model_path.name.lower()
    if "prone" in name:
        return "prone"
    if "supine" in name or "ground" in name:
        return "supine"
    return "unknown"


def run_trial(
    *,
    xml_path: Path,
    amp_config: dict[str, Any],
    amp_model_path: Path,
    host_model_paths: Sequence[Path],
    direction: str,
    physics_hz: float,
    settle_seconds: float,
    knockdown_force_newtons: float,
    knockdown_seconds: float,
    recovery_seconds: float,
    stable_seconds: float,
    action_rescale: float,
    fallback_after_seconds: float,
    takeover_delay_seconds: float,
) -> dict[str, Any]:
    if not math.isfinite(takeover_delay_seconds) or takeover_delay_seconds < 0.0:
        raise ValueError("takeover_delay_seconds must be non-negative and finite")
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    model.opt.timestep = 1.0 / float(physics_hz)
    data = mujoco.MjData(model)
    amp = AmpPolicy(amp_config, amp_model_path)
    if not host_model_paths:
        raise ValueError("at least one HoST model is required")
    host_policies = [
        HostPolicy(path, action_rescale=action_rescale)
        for path in host_model_paths
    ]
    host_index = 0
    host = host_policies[host_index]
    mapping = _joint_mapping(model, amp.joint_names)
    if mapping.names != MATRIX_JOINT_NAMES:
        raise ValueError("AMP/Matrix joint order differs from the HoST adapter contract")
    root_qpos, root_dof = _free_joint_addresses(model)
    pelvis_body_id = _pelvis_body_id(model)
    foot_body_ids = _foot_body_ids(model)
    _initialize_nominal_pose(model, data, mapping, amp_config)

    timestep = float(model.opt.timestep)
    control_dt = 0.02
    decimation = max(1, int(round(control_dt / timestep)))
    settle_steps = int(round(settle_seconds / timestep))
    knockdown_steps = max(1, int(round(knockdown_seconds / timestep)))
    recovery_steps = int(round(recovery_seconds / timestep))
    stable_steps_required = int(round(stable_seconds / timestep))
    takeover_delay_steps = max(0, int(round(takeover_delay_seconds / timestep)))
    total_steps = settle_steps + knockdown_steps + recovery_steps
    force_vector = KNOCKDOWN_VECTORS[direction] * knockdown_force_newtons
    zero_torque = np.zeros(3, dtype=np.float64)

    joint_pos = np.asarray(data.qpos[mapping.qpos_addresses], dtype=np.float64)
    joint_vel = np.asarray(data.qvel[mapping.qvel_addresses], dtype=np.float64)
    amp.reset_history(
        root_quat=np.asarray(data.qpos[root_qpos + 3 : root_qpos + 7]),
        root_angvel=np.asarray(data.qvel[root_dof + 3 : root_dof + 6]),
        joint_pos=joint_pos,
        joint_vel=joint_vel,
    )
    target = amp.default_joint_pos.copy()
    held_joint_target: np.ndarray | None = None
    host_active = False
    fall_detected_step: int | None = None
    host_started_step: int | None = None
    projected_gravity_at_takeover: list[float] | None = None
    policy_switches: list[dict[str, object]] = []
    fallen = False
    stable_steps = 0
    recovered_step: int | None = None
    min_root_z = math.inf
    min_root_up_z = math.inf
    kp, kd = _host_gains()

    # No qpos/qvel assignment and no reset/reload are allowed beyond this line.
    for step in range(total_steps):
        joint_pos = np.asarray(data.qpos[mapping.qpos_addresses], dtype=np.float64)
        joint_vel = np.asarray(data.qvel[mapping.qvel_addresses], dtype=np.float64)
        root_quat = np.asarray(data.qpos[root_qpos + 3 : root_qpos + 7])
        root_angvel = np.asarray(data.qvel[root_dof + 3 : root_dof + 6])
        root_z = float(data.qpos[root_qpos + 2])
        root_up_z = _root_up_z(root_quat)
        min_root_z = min(min_root_z, root_z)
        min_root_up_z = min(min_root_up_z, root_up_z)

        after_force_start = step >= settle_steps
        if after_force_start and (root_z < 0.45 or root_up_z < 0.5):
            fallen = True
            if fall_detected_step is None:
                fall_detected_step = step

        if (
            fall_detected_step is not None
            and not host_active
            and step - fall_detected_step >= takeover_delay_steps
        ):
            host_active = True
            host_started_step = step
            held_joint_target = joint_pos.copy()
            host.reset_history(
                root_quat=root_quat,
                root_angvel=root_angvel,
                joint_pos=joint_pos,
                joint_vel=joint_vel,
            )
            projected_gravity_at_takeover = _quat_rotate_inverse(
                root_quat, np.asarray((0.0, 0.0, -1.0), dtype=np.float64)
            ).tolist()

        if step % decimation == 0:
            if host_active:
                assert held_joint_target is not None
                assert host_started_step is not None
                elapsed_host_s = (step - host_started_step) * timestep
                next_switch_s = fallback_after_seconds * (host_index + 1)
                if (
                    host_index + 1 < len(host_policies)
                    and elapsed_host_s >= next_switch_s
                ):
                    host_index += 1
                    host = host_policies[host_index]
                    held_joint_target = joint_pos.copy()
                    host.reset_history(
                        root_quat=root_quat,
                        root_angvel=root_angvel,
                        joint_pos=joint_pos,
                        joint_vel=joint_vel,
                    )
                    policy_switches.append(
                        {
                            "time_s": step * timestep,
                            "model": str(host_model_paths[host_index]),
                        }
                    )
                target = host.infer(
                    root_quat=root_quat,
                    root_angvel=root_angvel,
                    joint_pos=joint_pos,
                    joint_vel=joint_vel,
                    held_joint_target=held_joint_target,
                )
            elif not fallen:
                target = amp.infer(
                    root_quat=root_quat,
                    root_angvel=root_angvel,
                    joint_pos=joint_pos,
                    joint_vel=joint_vel,
                )

        if host_active:
            torque = kp * (target - joint_pos) - kd * joint_vel
        else:
            torque = amp.torque(target, joint_pos, joint_vel)
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
        root_up_z = _root_up_z(root_quat)
        root_linear_speed = float(np.linalg.norm(data.qvel[root_dof : root_dof + 3]))
        root_angular_speed = float(
            np.linalg.norm(data.qvel[root_dof + 3 : root_dof + 6])
        )
        joint_speed_rms = float(
            math.sqrt(np.mean(np.square(data.qvel[mapping.qvel_addresses])))
        )
        stable_now = (
            host_active
            and root_z >= 0.65
            and root_up_z >= 0.85
            and root_linear_speed < 0.15
            and root_angular_speed < 0.50
            and joint_speed_rms < 0.50
            and _has_foot_contact(model, data, foot_body_ids)
        )
        if stable_now:
            stable_steps += 1
            if stable_steps >= stable_steps_required and recovered_step is None:
                recovered_step = step
                # The production supervisor stops the temporary LowCmd writer
                # as soon as this continuous stable window is satisfied.  End
                # the trial at the same boundary so a later timeout cannot
                # switch to a fallback after recovery has already completed.
                break
        else:
            stable_steps = 0

    final_quat = np.asarray(data.qpos[root_qpos + 3 : root_qpos + 7])
    return {
        "direction": direction,
        "models": [str(path) for path in host_model_paths],
        "model_families": [_model_family(path) for path in host_model_paths],
        "final_model_index": host_index,
        "policy_switches": policy_switches,
        "passed": fallen and recovered_step is not None,
        "fallen": fallen,
        "host_started_time_s": (
            None if host_started_step is None else host_started_step * timestep
        ),
        "fall_detected_time_s": (
            None if fall_detected_step is None else fall_detected_step * timestep
        ),
        "takeover_delay_s": takeover_delay_seconds,
        "projected_gravity_at_takeover": projected_gravity_at_takeover,
        "recovered_time_s": (
            None if recovered_step is None else recovered_step * timestep
        ),
        "minimum_root_z": min_root_z,
        "minimum_root_up_z": min_root_up_z,
        "final_root_z": float(data.qpos[root_qpos + 2]),
        "final_root_up_z": _root_up_z(final_quat),
        "final_root_linear_speed_mps": float(
            np.linalg.norm(data.qvel[root_dof : root_dof + 3])
        ),
        "final_root_angular_speed_radps": float(
            np.linalg.norm(data.qvel[root_dof + 3 : root_dof + 6])
        ),
        "final_joint_speed_rms_radps": float(
            math.sqrt(np.mean(np.square(data.qvel[mapping.qvel_addresses])))
        ),
        "physics_hz": physics_hz,
        "policy_hz": 1.0 / control_dt,
        "action_rescale": action_rescale,
        "physical_knockdown_only": True,
        "qpos_writes_after_physics_start": 0,
        "reset_calls_after_physics_start": 0,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xml", required=True, type=Path)
    parser.add_argument("--amp-config", required=True, type=Path)
    parser.add_argument("--amp-model", required=True, type=Path)
    parser.add_argument("--host-model", required=True, type=Path)
    parser.add_argument(
        "--host-fallback-model",
        action="append",
        type=Path,
        default=[],
        help="additional model tried physically after --fallback-after-seconds",
    )
    parser.add_argument(
        "--directions", default="forward,backward,left,right"
    )
    parser.add_argument("--physics-hz", type=float, default=200.0)
    parser.add_argument("--settle-seconds", type=float, default=2.0)
    parser.add_argument("--knockdown-force-newtons", type=float, default=3400.0)
    parser.add_argument("--knockdown-seconds", type=float, default=0.04)
    parser.add_argument("--recovery-seconds", type=float, default=18.0)
    parser.add_argument("--stable-seconds", type=float, default=1.5)
    parser.add_argument("--action-rescale", type=float, default=0.25)
    parser.add_argument("--fallback-after-seconds", type=float, default=8.0)
    parser.add_argument(
        "--takeover-delay-seconds",
        type=float,
        default=0.35,
        help=(
            "physical settling window before HoST takes LowCmd ownership; "
            "the last AMP target remains applied and simulator state is never edited"
        ),
    )
    parser.add_argument("--output", type=Path)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    directions = tuple(item.strip() for item in args.directions.split(",") if item.strip())
    if not directions or any(item not in KNOCKDOWN_VECTORS for item in directions):
        raise SystemExit("--directions must contain forward/backward/left/right")
    with args.amp_config.open("r", encoding="utf-8") as handle:
        amp_config = json.load(handle)
    host_model_paths = [args.host_model.resolve()]
    host_model_paths.extend(path.resolve() for path in args.host_fallback_model)
    trials = []
    for direction in directions:
        print(f"[HoST GET-UP] testing {direction}", flush=True)
        trial = run_trial(
            xml_path=args.xml.resolve(),
            amp_config=amp_config,
            amp_model_path=args.amp_model.resolve(),
            host_model_paths=host_model_paths,
            direction=direction,
            physics_hz=args.physics_hz,
            settle_seconds=args.settle_seconds,
            knockdown_force_newtons=args.knockdown_force_newtons,
            knockdown_seconds=args.knockdown_seconds,
            recovery_seconds=args.recovery_seconds,
            stable_seconds=args.stable_seconds,
            action_rescale=args.action_rescale,
            fallback_after_seconds=args.fallback_after_seconds,
            takeover_delay_seconds=args.takeover_delay_seconds,
        )
        trials.append(trial)
        print(
            f"[HoST GET-UP] {direction}: passed={trial['passed']} "
            f"z={trial['final_root_z']:.3f} up={trial['final_root_up_z']:.3f}",
            flush=True,
        )
    result = {
        "schema": "matrix.g1_host_getup_validation.v1",
        "passed": all(trial["passed"] for trial in trials),
        "inputs": {
            "xml": str(args.xml.resolve()),
            "xml_sha256": _file_sha256(args.xml.resolve()),
            "amp_config": str(args.amp_config.resolve()),
            "amp_config_sha256": _file_sha256(args.amp_config.resolve()),
            "amp_model": str(args.amp_model.resolve()),
            "amp_model_sha256": _file_sha256(args.amp_model.resolve()),
            "host_models": [str(path) for path in host_model_paths],
            "host_model_sha256": [
                _file_sha256(path) for path in host_model_paths
            ],
        },
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
