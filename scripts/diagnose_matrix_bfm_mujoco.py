#!/usr/bin/env python3
"""Run BFM-Teacher50k against a Matrix MuJoCo scene without UE or DDS.

This is an offline diagnostic only.  It aligns the robot once to the first
Robo-PFNN reference frame, then runs the same 50 Hz Teacher observation/action
contract and the same 200 Hz position-PD torque loop used by Matrix.  Variants
change one simulator contract at a time so policy, actuator, and passive-joint
differences can be isolated without touching the source MJCF.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time
from typing import Any

import mujoco
import numpy as np

# Some pinned upstream checkouts unfortunately contain tracked ``.pyc`` files.
# Keep diagnostics read-only with respect to every imported source tree.
sys.dont_write_bytecode = True

import matrix_bfm_teacher_adapter as adapter
from matrix_bfm_teacher_adapter import (
    BfmTeacherCore,
    G1_29_JOINT_NAMES,
    LowStateSnapshot,
    WorldSample,
)


POLICY_HZ = 50.0
PHYSICS_HZ = 200.0
CONTROL_DECIMATION = int(PHYSICS_HZ / POLICY_HZ)

VARIANTS = (
    "matrix",
    "official-effort",
    "official-armature",
    "passive-off",
    "official-actuator",
    "official-actuator-contact1",
)


def _yaw_wxyz(quaternion: np.ndarray) -> float:
    w, x, y, z = (float(value) for value in quaternion)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def _up_z_wxyz(quaternion: np.ndarray) -> float:
    _, x, y, _ = (float(value) for value in quaternion)
    return 1.0 - 2.0 * (x * x + y * y)


def _rotation_matrix_wxyz(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = (float(value) for value in quaternion)
    return np.asarray(
        (
            (
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ),
            (
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ),
            (
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ),
        ),
        dtype=np.float64,
    )


def _official_effort_limits() -> np.ndarray:
    result: list[float] = []
    for name in G1_29_JOINT_NAMES:
        if any(token in name for token in ("hip_pitch", "hip_roll", "knee")):
            result.append(139.0)
        elif "hip_yaw" in name or name == "waist_yaw_joint":
            result.append(88.0)
        elif "ankle_" in name or name in {
            "waist_roll_joint",
            "waist_pitch_joint",
        }:
            result.append(50.0)
        elif "wrist_pitch" in name or "wrist_yaw" in name:
            result.append(5.0)
        else:
            result.append(25.0)
    return np.asarray(result, dtype=np.float64)


def _official_armatures() -> np.ndarray:
    result: list[float] = []
    for name in G1_29_JOINT_NAMES:
        if any(token in name for token in ("hip_pitch", "hip_roll", "knee")):
            result.append(0.025101925)
        elif "hip_yaw" in name or name == "waist_yaw_joint":
            result.append(0.010177520)
        elif "ankle_" in name or name in {
            "waist_roll_joint",
            "waist_pitch_joint",
        }:
            result.append(2.0 * 0.003609725)
        elif "wrist_pitch" in name or "wrist_yaw" in name:
            result.append(0.00425)
        else:
            result.append(0.003609725)
    return np.asarray(result, dtype=np.float64)


def _joint_addresses(
    model: mujoco.MjModel,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    joint_ids = np.asarray(
        [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in G1_29_JOINT_NAMES
        ],
        dtype=np.int64,
    )
    if np.any(joint_ids < 0):
        missing = [
            name for name, joint_id in zip(G1_29_JOINT_NAMES, joint_ids) if joint_id < 0
        ]
        raise RuntimeError(f"scene is missing G1 joints: {missing}")
    qpos_addresses = model.jnt_qposadr[joint_ids].astype(np.int64)
    dof_addresses = model.jnt_dofadr[joint_ids].astype(np.int64)
    actuator_ids = np.asarray(
        [
            mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_ACTUATOR,
                name.removesuffix("_joint"),
            )
            for name in G1_29_JOINT_NAMES
        ],
        dtype=np.int64,
    )
    if np.any(actuator_ids < 0):
        missing = [
            name
            for name, actuator_id in zip(G1_29_JOINT_NAMES, actuator_ids)
            if actuator_id < 0
        ]
        raise RuntimeError(f"scene is missing G1 actuators: {missing}")
    return qpos_addresses, dof_addresses, actuator_ids


def _apply_variant(
    model: mujoco.MjModel,
    *,
    variant: str,
    dof_addresses: np.ndarray,
    actuator_ids: np.ndarray,
) -> np.ndarray:
    matrix_limits = np.max(np.abs(model.actuator_ctrlrange[actuator_ids]), axis=1)
    effort_limits = matrix_limits.copy()
    if variant in {
        "official-effort",
        "official-actuator",
        "official-actuator-contact1",
    }:
        effort_limits = _official_effort_limits()
        model.actuator_ctrlrange[actuator_ids, 0] = -effort_limits
        model.actuator_ctrlrange[actuator_ids, 1] = effort_limits
    if variant in {
        "official-armature",
        "official-actuator",
        "official-actuator-contact1",
    }:
        model.dof_armature[dof_addresses] = _official_armatures()
    if variant in {
        "passive-off",
        "official-actuator",
        "official-actuator-contact1",
    }:
        model.dof_damping[dof_addresses] = 0.0
        model.dof_frictionloss[dof_addresses] = 0.0
    if variant == "official-actuator-contact1":
        # Isaac's accepted flat baseline uses static/dynamic friction 1.0.
        # MuJoCo exposes a single sliding-friction coefficient per geom.
        model.geom_friction[:, 0] = 1.0
    return effort_limits


def _apply_controller_contract(
    core: BfmTeacherCore,
    model: mujoco.MjModel,
    *,
    dof_addresses: np.ndarray,
    contract: str,
) -> None:
    if contract == "formal7168":
        return
    armature = np.asarray(
        model.dof_armature[dof_addresses],
        dtype=np.float32,
    )
    if np.any(~np.isfinite(armature)) or np.any(armature <= 0.0):
        raise RuntimeError("model armature must be finite and positive")
    natural_frequency = 10.0 * 2.0 * math.pi
    damping_ratio = 2.0
    kp = armature * natural_frequency**2
    kd = 2.0 * damping_ratio * armature * natural_frequency
    if contract == "model-armature-full":
        effort = 4.0 * core.action_scale * core.kp
        core.action_scale = (0.25 * effort / kp).astype(np.float32)
    core.kp = kp.astype(np.float32)
    core.kd = kd.astype(np.float32)


def _core(args: argparse.Namespace) -> BfmTeacherCore:
    return BfmTeacherCore(
        model_path=args.teacher_onnx,
        realscan_root=args.realscan_root,
        robo_pfnn_root=args.robo_pfnn_root,
        weights_dir=args.weights_dir,
        g1_xml=args.g1_xml,
        formal_ik=args.formal_ik,
        execution_provider="cpu",
        activation_blend_seconds=args.activation_blend_seconds,
    )


def _stand_command(core: BfmTeacherCore):
    return core.command_module.CommandSample(
        vx=0.0,
        vy=0.0,
        yaw_rate=0.0,
        gait="stand",
        stop_latched=False,
    )


def _initial_reference(
    core: BfmTeacherCore,
    root_position: np.ndarray,
    root_yaw: float,
    height_map: np.ndarray,
):
    height_field = core.reference_module.LocalTerrainHeightField(
        root_position,
        root_yaw,
        height_map,
    )
    sample = core.stream.sample(
        _stand_command(core),
        root_position,
        root_yaw,
        height_field,
    )
    root_velocity_world = core.reference_module.qpos_root_velocity(
        sample.plan.qpos_50hz[0],
        sample.plan.qpos_50hz[1],
        POLICY_HZ,
    )
    return sample, root_velocity_world


def _lowstate(
    data: mujoco.MjData,
    *,
    qpos_addresses: np.ndarray,
    dof_addresses: np.ndarray,
    received_monotonic: float,
) -> LowStateSnapshot:
    return LowStateSnapshot.validated(
        quaternion_wxyz=data.qpos[3:7],
        body_gyro_rad_s=data.qvel[3:6],
        joint_pos_rad=data.qpos[qpos_addresses],
        joint_vel_rad_s=data.qvel[dof_addresses],
        received_monotonic=received_monotonic,
    )


def _capture_live_state(args: argparse.Namespace) -> dict[str, np.ndarray]:
    status = json.loads(args.live_status.read_text(encoding="utf-8"))
    root_position = np.asarray(status.get("root_xyz"), dtype=np.float64)
    if root_position.shape != (3,) or not np.isfinite(root_position).all():
        raise ValueError("live status root_xyz must be a finite 3-vector")

    state_store = adapter.LatestLowState()
    # UnitreeDdsRuntime creates only a LowState reader here.  It never creates
    # an rt/lowcmd writer unless create_publisher() is called.
    dds_runtime = adapter.UnitreeDdsRuntime(
        interface=args.dds_interface,
        state_store=state_store,
    )
    deadline = time.monotonic() + args.live_state_timeout_seconds
    state = None
    while time.monotonic() < deadline:
        state = state_store.get()
        if state is not None:
            break
        time.sleep(0.01)
    if state is None:
        raise TimeoutError("timed out waiting for live rt/lowstate")
    # Keep the reader live through the copy; it intentionally has no writer.
    _ = dds_runtime
    return {
        "root_position": root_position,
        "quaternion_wxyz": state.quaternion_wxyz.copy(),
        "body_gyro_rad_s": state.body_gyro_rad_s.copy(),
        "joint_pos_rad": state.joint_pos_rad.copy(),
        "joint_vel_rad_s": state.joint_vel_rad_s.copy(),
    }


def run_variant(args: argparse.Namespace, variant: str) -> dict[str, Any]:
    adapter.TURN_REFERENCE_FORWARD_MPS = args.turn_reference_forward_mps
    adapter.FORMAL_COMMAND_YAW_GAIN = args.command_yaw_gain
    adapter.FORMAL_COMMAND_YAW_LIMIT_RAD_S = args.command_yaw_limit_rad_s
    adapter.TURN_COMMAND_YAW_LIMIT_RAD_S = args.command_yaw_limit_rad_s
    adapter.TURN_COMMAND_YAW_DAMPING_SECONDS = args.yaw_damping_seconds
    core = _core(args)
    try:
        model = mujoco.MjModel.from_xml_path(str(args.scene))
        model.opt.timestep = 1.0 / PHYSICS_HZ
        data = mujoco.MjData(model)
        qpos_addresses, dof_addresses, actuator_ids = _joint_addresses(model)
        effort_limits = _apply_variant(
            model,
            variant=variant,
            dof_addresses=dof_addresses,
            actuator_ids=actuator_ids,
        )
        _apply_controller_contract(
            core,
            model,
            dof_addresses=dof_addresses,
            contract=args.controller_contract,
        )

        height_map = np.full((11, 11), args.ground_z, dtype=np.float64)
        anchor_position = model.qpos0[:3].copy()
        anchor_yaw = _yaw_wxyz(model.qpos0[3:7])
        mujoco.mj_resetData(model, data)
        if args.initialization == "aligned":
            initial_reference, root_velocity_world = _initial_reference(
                core,
                anchor_position,
                anchor_yaw,
                height_map,
            )
            reference_qpos = np.asarray(
                initial_reference.plan.future_qpos[0],
                dtype=np.float64,
            )
            reference_joint_vel = np.asarray(
                initial_reference.plan.future_joint_vel[0],
                dtype=np.float64,
            )
            data.qpos[:7] = reference_qpos[:7]
            data.qpos[qpos_addresses] = reference_qpos[7:]
            data.qvel[:3] = root_velocity_world[:3]
            # Isaac exposes root angular velocity in world coordinates.
            # MuJoCo's free-joint rotational qvel is body-local.
            data.qvel[3:6] = (
                _rotation_matrix_wxyz(reference_qpos[3:7]).T
                @ root_velocity_world[3:6]
            )
            data.qvel[dof_addresses] = reference_joint_vel
        elif args.initialization == "live-blend":
            live_state = args.live_state
            data.qpos[:3] = live_state["root_position"]
            data.qpos[3:7] = live_state["quaternion_wxyz"]
            data.qpos[qpos_addresses] = live_state["joint_pos_rad"]
            data.qvel[:3] = 0.0
            data.qvel[3:6] = live_state["body_gyro_rad_s"]
            data.qvel[dof_addresses] = live_state["joint_vel_rad_s"]
        mujoco.mj_forward(model, data)
        core.reset()
        if args.initialization in {"qpos0-blend", "live-blend"}:
            core.prepare_activation(
                _lowstate(
                    data,
                    qpos_addresses=qpos_addresses,
                    dof_addresses=dof_addresses,
                    received_monotonic=0.0,
                )
            )

        initial_root = data.qpos[:3].copy()
        command_heading = _yaw_wxyz(data.qpos[3:7]) + math.radians(
            args.command_heading_delta_deg
        )
        wire_heading = _yaw_wxyz(data.qpos[3:7])
        action_l2: list[float] = []
        action_max: list[float] = []
        torque_max: list[float] = []
        joint_error_rms: list[float] = []
        saturation_count = 0
        saturation_total = 0
        samples: list[dict[str, Any]] = []
        minimum_up_z = 1.0

        for step in range(args.steps):
            root_position = data.qpos[:3].copy()
            root_quaternion = data.qpos[3:7].copy()
            root_yaw = _yaw_wxyz(root_quaternion)
            heading_error = math.atan2(
                math.sin(command_heading - root_yaw),
                math.cos(command_heading - root_yaw),
            )
            wire_target_heading = root_yaw + float(
                np.clip(
                    heading_error,
                    -args.wire_lead_window_rad,
                    args.wire_lead_window_rad,
                )
            )
            wire_slew = math.atan2(
                math.sin(wire_target_heading - wire_heading),
                math.cos(wire_target_heading - wire_heading),
            )
            wire_heading += float(
                np.clip(
                    wire_slew,
                    -args.wire_max_step_rad,
                    args.wire_max_step_rad,
                )
            )
            wire_heading = math.atan2(
                math.sin(wire_heading),
                math.cos(wire_heading),
            )
            desired_facing = np.asarray(
                (
                    math.cos(command_heading),
                    math.sin(command_heading),
                    0.0,
                ),
                dtype=np.float64,
            )
            wire_facing = np.asarray(
                (
                    math.cos(wire_heading),
                    math.sin(wire_heading),
                    0.0,
                ),
                dtype=np.float64,
            )
            transition_to_move = (
                args.idle_steps_before_turn + args.turn_steps_before_move
            )
            moving = args.command_mode == "move" or (
                args.command_mode == "turn-move"
                and step >= args.turn_steps_before_move
            ) or (
                args.command_mode == "idle-turn-move"
                and step >= transition_to_move
            )
            turning = args.command_mode == "turn" or (
                args.command_mode == "turn-move"
                and step < args.turn_steps_before_move
            ) or (
                args.command_mode == "idle-turn-move"
                and args.idle_steps_before_turn
                <= step
                < transition_to_move
            )
            lowstate = _lowstate(
                data,
                qpos_addresses=qpos_addresses,
                dof_addresses=dof_addresses,
                received_monotonic=float(step) / POLICY_HZ,
            )
            world = WorldSample(
                sequence=step,
                received_monotonic=float(step) / POLICY_HZ,
                reset_count=0,
                root_position=root_position,
                root_yaw=root_yaw,
                height_map_z=height_map,
                movement=(
                    desired_facing.copy()
                    if moving
                    else np.zeros(3, dtype=np.float64)
                ),
                facing=(
                    desired_facing
                    if args.adapter_heading_source == "desired-facing"
                    else wire_facing
                ),
                desired_facing=desired_facing,
                speed_mps=args.command_speed_mps if moving else 0.0,
                locomotion_mode=2 if moving else 0,
                mode="move" if moving else "turn" if turning else "idle",
                safe_stop=False,
            )
            target, policy_status = core.step(world, lowstate, active=True)
            minimum_up_z = min(minimum_up_z, _up_z_wxyz(root_quaternion))
            action_l2.append(float(policy_status["raw_action_l2"]))
            action_max.append(float(policy_status["raw_action_max_abs"]))
            joint_error_rms.append(
                float(policy_status["reference_joint_error_rms_rad"])
            )

            for _ in range(CONTROL_DECIMATION):
                torque = (
                    core.kp.astype(np.float64)
                    * (target.astype(np.float64) - data.qpos[qpos_addresses])
                    - core.kd.astype(np.float64) * data.qvel[dof_addresses]
                )
                saturated = np.abs(torque) > effort_limits
                saturation_count += int(np.count_nonzero(saturated))
                saturation_total += int(torque.size)
                torque = np.clip(torque, -effort_limits, effort_limits)
                torque_max.append(float(np.max(np.abs(torque))))
                data.ctrl[actuator_ids] = torque
                mujoco.mj_step(model, data)

            if (
                step == 0
                or (step + 1) % args.sample_every == 0
                or step + 1 == args.steps
            ):
                displacement = float(
                    np.linalg.norm(data.qpos[:2] - initial_root[:2])
                )
                samples.append(
                    {
                        "step": step + 1,
                        "time_s": float(data.time),
                        "command_mode": (
                            "move" if moving else "turn" if turning else "idle"
                        ),
                        "root_xyz": data.qpos[:3].tolist(),
                        "xy_displacement_m": displacement,
                        "up_z": _up_z_wxyz(data.qpos[3:7]),
                        "root_yaw_rad": _yaw_wxyz(data.qpos[3:7]),
                        "command_heading_error_rad": policy_status[
                            "command_heading_error_rad"
                        ],
                        "command_yaw_rate_rad_s": policy_status[
                            "command_yaw_rate_rad_s"
                        ],
                        "command_speed_mps": policy_status["command_speed_mps"],
                        "action_l2": action_l2[-1],
                        "action_max_abs": action_max[-1],
                        "reference_joint_error_rms_rad": joint_error_rms[-1],
                    }
                )

        final_displacement = float(
            np.linalg.norm(data.qpos[:2] - initial_root[:2])
        )
        return {
            "variant": variant,
            "controller_contract": args.controller_contract,
            "initialization": args.initialization,
            "command_mode": args.command_mode,
            "idle_steps_before_turn": args.idle_steps_before_turn,
            "turn_steps_before_move": args.turn_steps_before_move,
            "command_heading_delta_deg": args.command_heading_delta_deg,
            "command_speed_mps": args.command_speed_mps,
            "command_yaw_gain": args.command_yaw_gain,
            "command_yaw_limit_rad_s": args.command_yaw_limit_rad_s,
            "yaw_damping_seconds": args.yaw_damping_seconds,
            "wire_lead_window_rad": args.wire_lead_window_rad,
            "wire_max_step_rad": args.wire_max_step_rad,
            "adapter_heading_source": args.adapter_heading_source,
            "turn_reference_forward_mps": args.turn_reference_forward_mps,
            "steps": args.steps,
            "duration_s": float(data.time),
            "initial_root_xyz": initial_root.tolist(),
            "final_root_xyz": data.qpos[:3].tolist(),
            "xy_displacement_m": final_displacement,
            "up_z": _up_z_wxyz(data.qpos[3:7]),
            "minimum_up_z": minimum_up_z,
            "final_root_yaw_rad": _yaw_wxyz(data.qpos[3:7]),
            "action_l2_mean": float(np.mean(action_l2)),
            "action_l2_max": float(np.max(action_l2)),
            "action_max_abs_max": float(np.max(action_max)),
            "torque_max_abs": float(np.max(torque_max)),
            "torque_saturation_fraction": (
                float(saturation_count) / float(saturation_total)
                if saturation_total
                else 0.0
            ),
            "reference_joint_error_rms_mean_rad": float(
                np.mean(joint_error_rms)
            ),
            "samples": samples,
        }
    finally:
        core.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--teacher-onnx", type=Path, required=True)
    parser.add_argument("--realscan-root", type=Path, required=True)
    parser.add_argument("--robo-pfnn-root", type=Path, required=True)
    parser.add_argument("--weights-dir", type=Path, required=True)
    parser.add_argument("--g1-xml", type=Path, required=True)
    parser.add_argument("--formal-ik", type=Path, required=True)
    parser.add_argument("--steps", type=int, default=300)
    parser.add_argument("--sample-every", type=int, default=50)
    parser.add_argument("--ground-z", type=float, default=0.0)
    parser.add_argument(
        "--command-mode",
        choices=("idle", "turn", "move", "turn-move", "idle-turn-move"),
        default="idle",
    )
    parser.add_argument("--idle-steps-before-turn", type=int, default=300)
    parser.add_argument("--turn-steps-before-move", type=int, default=200)
    parser.add_argument("--command-heading-delta-deg", type=float, default=-45.0)
    parser.add_argument("--command-speed-mps", type=float, default=0.8)
    parser.add_argument(
        "--command-yaw-gain",
        type=float,
        default=adapter.FORMAL_COMMAND_YAW_GAIN,
    )
    parser.add_argument(
        "--command-yaw-limit-rad-s",
        type=float,
        default=adapter.FORMAL_COMMAND_YAW_LIMIT_RAD_S,
    )
    parser.add_argument("--yaw-damping-seconds", type=float, default=0.0)
    parser.add_argument("--wire-lead-window-rad", type=float, default=0.05)
    parser.add_argument("--wire-max-step-rad", type=float, default=0.05)
    parser.add_argument(
        "--adapter-heading-source",
        choices=("desired-facing", "wire-facing"),
        default="desired-facing",
    )
    parser.add_argument(
        "--turn-reference-forward-mps",
        type=float,
        default=adapter.TURN_REFERENCE_FORWARD_MPS,
    )
    parser.add_argument(
        "--activation-blend-seconds",
        type=float,
        default=0.75,
    )
    parser.add_argument(
        "--controller-contract",
        choices=(
            "formal7168",
            "model-armature-gains",
            "model-armature-full",
        ),
        default="formal7168",
    )
    parser.add_argument(
        "--initialization",
        choices=("aligned", "qpos0-blend", "live-blend"),
        default="aligned",
    )
    parser.add_argument(
        "--live-status",
        type=Path,
        help="Matrix status JSON used with --initialization live-blend",
    )
    parser.add_argument("--dds-interface", default="lo")
    parser.add_argument("--live-state-timeout-seconds", type=float, default=2.0)
    parser.add_argument(
        "--variant",
        action="append",
        choices=VARIANTS,
        dest="variants",
    )
    args = parser.parse_args()
    if args.steps <= 0:
        parser.error("--steps must be positive")
    if args.sample_every <= 0:
        parser.error("--sample-every must be positive")
    if args.idle_steps_before_turn < 0:
        parser.error("--idle-steps-before-turn must be nonnegative")
    if args.turn_steps_before_move < 0:
        parser.error("--turn-steps-before-move must be nonnegative")
    if (
        args.command_mode == "turn-move"
        and args.turn_steps_before_move >= args.steps
    ):
        parser.error("--turn-steps-before-move must be smaller than --steps")
    if (
        args.command_mode == "idle-turn-move"
        and (
            args.idle_steps_before_turn >= args.steps
            or args.idle_steps_before_turn + args.turn_steps_before_move
            >= args.steps
        )
    ):
        parser.error(
            "idle plus turn steps must leave at least one move step"
        )
    for name in (
        "command_heading_delta_deg",
        "command_speed_mps",
        "command_yaw_gain",
        "command_yaw_limit_rad_s",
        "yaw_damping_seconds",
        "wire_lead_window_rad",
        "wire_max_step_rad",
        "turn_reference_forward_mps",
    ):
        if not math.isfinite(getattr(args, name)):
            parser.error(f"--{name.replace('_', '-')} must be finite")
    if args.command_speed_mps < 0.0:
        parser.error("--command-speed-mps must be nonnegative")
    if args.command_yaw_gain <= 0.0:
        parser.error("--command-yaw-gain must be positive")
    if args.command_yaw_limit_rad_s <= 0.0:
        parser.error("--command-yaw-limit-rad-s must be positive")
    if args.yaw_damping_seconds < 0.0:
        parser.error("--yaw-damping-seconds must be nonnegative")
    if not 0.0 < args.wire_lead_window_rad <= math.pi:
        parser.error("--wire-lead-window-rad must be in (0, pi]")
    if not 0.0 < args.wire_max_step_rad <= math.pi:
        parser.error("--wire-max-step-rad must be in (0, pi]")
    if args.turn_reference_forward_mps <= 0.0:
        parser.error("--turn-reference-forward-mps must be positive")
    if (
        not math.isfinite(args.activation_blend_seconds)
        or args.activation_blend_seconds <= 0.0
    ):
        parser.error("--activation-blend-seconds must be positive and finite")
    if args.initialization == "live-blend" and args.live_status is None:
        parser.error("--live-status is required with --initialization live-blend")
    if (
        not math.isfinite(args.live_state_timeout_seconds)
        or args.live_state_timeout_seconds <= 0.0
    ):
        parser.error("--live-state-timeout-seconds must be positive and finite")
    if not args.variants:
        args.variants = list(VARIANTS)
    return args


def main() -> int:
    args = parse_args()
    args.live_state = (
        _capture_live_state(args)
        if args.initialization == "live-blend"
        else None
    )
    results = []
    for variant in args.variants:
        result = run_variant(args, variant)
        results.append(result)
        print(json.dumps(result, sort_keys=True), flush=True)
    print(
        json.dumps(
            {
                "summary": [
                    {
                        "variant": result["variant"],
                        "initialization": result["initialization"],
                        "xy_displacement_m": result["xy_displacement_m"],
                        "up_z": result["up_z"],
                        "torque_saturation_fraction": result[
                            "torque_saturation_fraction"
                        ],
                    }
                    for result in results
                ]
            },
            sort_keys=True,
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
