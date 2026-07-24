#!/usr/bin/env python3
"""Resident BFM-Teacher50k locomotion adapter for Matrix.

The process keeps the terrain-aware Teacher and its pinned Robo-PFNN
reference generator warm while native SONIC owns ``rt/lowcmd``.  Writer
authority is granted only by the supervisor over an authenticated local
``SOCK_SEQPACKET`` control connection.  No simulator state is edited here.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib
import importlib.util
import json
import math
import os
import select
import socket
import subprocess
import sys
import threading
import time
import types
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Mapping

import numpy as np

from matrix_sonic_amp_worker import (
    G1_29_JOINT_NAMES,
    HandoffStateMachine,
    LatestLowState,
    LowStateSnapshot,
    NUM_JOINTS,
    PolicyConfig,
    UnitreeDdsRuntime,
    _advance_deadline,
    state_status,
)


POLICY_ID = "bfm-sonic-teacher50k"
CONTROL_SCHEMA = "matrix.bfm_teacher_worker.control.v1"
POLICY_HZ = 50.0
PUBLISH_HZ = 500.0
WORLD_SAMPLE_MAX_AGE_S = 0.15
LOWSTATE_MAX_AGE_S = 0.10
# A single delayed scheduler/DDS frame must not tear down an otherwise healthy
# resident writer.  In particular, loading a large Town10 asset can briefly
# delay both the simulator's LowState publisher and Matrix's world STATE packet
# by slightly more than the normal freshness budgets.  Reuse the last
# policy-consistent LowCmd while that shared input boundary catches up, but
# retain a bounded fail-closed deadline for a real disconnect.
TRANSIENT_INPUT_STALE_GRACE_S = 0.50
ACTION_CLIP = 20.0
# The pinned Robo-PFNN runtime divides commanded velocity by its 50 Hz label
# rate before choosing a direction from target velocity.  A yaw-only command
# with exactly zero velocity therefore leaves the trajectory in stand and
# silently ignores ``yaw_rate``.  Seed a physically negligible *forward*
# reference just above the strict post-division 1e-5 threshold (0.0005 m/s in
# command units).  Forward is important: a lateral seed sets PFNN's
# ``gait_side`` input to one and turns an in-place rotation into a full strafe.
# Matrix's authoritative world command remains zero-translation; this value
# exists only inside the reference generator so a turn-only request can
# produce a rotating pose.
TURN_REFERENCE_FORWARD_MPS = 0.00051
# The formal7168 collection path turns a requested heading into the canonical
# body-yaw command with a bounded P controller.  Matrix already provides a
# rate-limited wire-facing vector.  Consume that safety boundary directly,
# then predict body heading slightly forward from measured yaw velocity to
# damp the PFNN/Teacher closed-loop lag.  The final camera facing remains in
# the packet for observability, but bypassing wire-facing caused full-rate
# command reversals and live turn oscillation.
FORMAL_COMMAND_YAW_GAIN = 4.0
FORMAL_COMMAND_YAW_LIMIT_RAD_S = 1.5
TURN_COMMAND_YAW_LIMIT_RAD_S = 0.6
TURN_COMMAND_YAW_DAMPING_SECONDS = 0.1

BFM_SOURCE_COMMIT = "5e264ae2bee2315dc0522c48c64b4506977b2e25"
REALSCAN_SOURCE_COMMIT = "850a71bef1e1472aaeb3ff4cb9004d9848830cfc"
ROBO_PFNN_SOURCE_COMMIT = "eb1b8b8001a221d2147f8daa073ca447acc8649e"
TEACHER_ONNX_SHA256 = (
    "edbec19062d6c34621dd97df864c596d29937432d8a019dd949d03785d9cdc45"
)
TEACHER_CONFIG_SHA256 = (
    "e7bed95642a3627cc6f6cff416da784fe2d0841b697d0f34e7039fd73af10e3f"
)
ROBO_PFNN_WEIGHTS_TREE_SHA256 = (
    "d1d0a7255a2f8898e81522570a09a3b56624fd7b955a2d7d02b87800f47585cb"
)
ROBO_PFNN_G1_XML_SHA256 = (
    "8c586e4747da85804180fe44d8692e0fd8231356728b6327e256dca498087a78"
)
ROBO_PFNN_IK_SHA256 = (
    "c8776f1e7651a4f179ea75e17b9746c41fa77a15be2cacf5809fe648340a7ab2"
)

_ARMATURE_5020 = 0.003609725
_ARMATURE_7520_14 = 0.010177520
_ARMATURE_7520_22 = 0.025101925
_ARMATURE_4010 = 0.00425
_NATURAL_FREQ = 10.0 * 2.0 * math.pi
_DAMPING_RATIO = 2.0


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def directory_tree_sha256(path: Path) -> tuple[str, int]:
    """Match the formal7168 sorted ``sha256sum`` tree contract."""

    files = sorted(
        candidate for candidate in path.rglob("*") if candidate.is_file()
    )
    digest = hashlib.sha256()
    for candidate in files:
        relative = candidate.relative_to(path).as_posix()
        line = f"{file_sha256(candidate)}  ./{relative}\n"
        digest.update(line.encode("utf-8"))
    return digest.hexdigest(), len(files)


def require_file_sha256(path: Path, expected: str, label: str) -> None:
    if not path.is_file():
        raise FileNotFoundError(f"{label} is missing: {path}")
    actual = file_sha256(path)
    if actual != expected:
        raise ValueError(
            f"{label} SHA256 mismatch: expected={expected} actual={actual}"
        )


def require_source_checkout(path: Path, expected_commit: str, label: str) -> None:
    if not path.is_dir():
        raise FileNotFoundError(f"{label} source checkout is missing: {path}")
    result = subprocess.run(
        (
            "git",
            "-C",
            os.fspath(path),
            "rev-parse",
            "HEAD",
        ),
        check=False,
        capture_output=True,
        text=True,
        timeout=5.0,
        env={**os.environ, "GIT_OPTIONAL_LOCKS": "0"},
    )
    actual = result.stdout.strip()
    if result.returncode != 0 or actual != expected_commit:
        raise ValueError(
            f"{label} source commit mismatch: "
            f"expected={expected_commit} actual={actual or 'unavailable'}"
        )
    dirty = subprocess.run(
        (
            "git",
            "-C",
            os.fspath(path),
            "status",
            "--porcelain",
            "--untracked-files=no",
        ),
        check=False,
        capture_output=True,
        text=True,
        timeout=5.0,
        env={**os.environ, "GIT_OPTIONAL_LOCKS": "0"},
    )
    if dirty.returncode != 0 or dirty.stdout.strip():
        raise ValueError(f"{label} source checkout is dirty or unreadable")


def _read_f32(path: Path) -> np.ndarray:
    if not path.is_file():
        raise FileNotFoundError(path)
    return np.fromfile(path, dtype=np.float32)


class NumpyPfnnForward:
    """CPU implementation of the pinned four-bank cubic PFNN forward pass."""

    def __init__(self, weights_dir: Path) -> None:
        self.weights_dir = Path(weights_dir)
        self._lock = threading.Lock()
        phase_banks = 50

        def stack(prefix: str, rows: int, cols: int) -> np.ndarray:
            return np.stack(
                [
                    _read_f32(
                        self.weights_dir / f"{prefix}_{index:03d}.bin"
                    ).reshape(rows, cols)
                    for index in range(phase_banks)
                ],
                axis=0,
            )

        self.weights = (
            stack("W0", 512, 420),
            stack("W1", 512, 512),
            stack("W2", 383, 512),
        )
        self.biases = (
            stack("b0", 512, 1).reshape(phase_banks, 512),
            stack("b1", 512, 1).reshape(phase_banks, 512),
            stack("b2", 383, 1).reshape(phase_banks, 383),
        )
        self.xmean = _read_f32(self.weights_dir / "Xmean.bin")
        self.xstd_inv = 1.0 / _read_f32(self.weights_dir / "Xstd.bin")
        self.ymean = _read_f32(self.weights_dir / "Ymean.bin")
        self.ystd = _read_f32(self.weights_dir / "Ystd.bin")

    @staticmethod
    def _elu(values: np.ndarray) -> np.ndarray:
        result = values.copy()
        negative = result < 0.0
        result[negative] = np.expm1(result[negative])
        return result

    def predict(self, values: np.ndarray, phase: float) -> np.ndarray:
        phase_scaled = 50.0 * float(phase)
        key = int(math.floor(phase_scaled))
        mu = phase_scaled - key
        key1 = key % 50
        indices = np.asarray(
            ((key1 - 1) % 50, key1, (key1 + 1) % 50, (key1 + 2) % 50),
            dtype=np.int64,
        )
        mu2 = mu * mu
        mu3 = mu2 * mu
        coefficients = np.asarray(
            (
                -0.5 * mu3 + mu2 - 0.5 * mu,
                1.5 * mu3 - 2.5 * mu2 + 1.0,
                -1.5 * mu3 + 2.0 * mu2 + 0.5 * mu,
                0.5 * mu3 - 0.5 * mu2,
            ),
            dtype=np.float32,
        )
        with self._lock:
            hidden = (
                np.asarray(values, dtype=np.float32) - self.xmean
            ) * self.xstd_inv
            for layer, (weights, biases) in enumerate(
                zip(self.weights, self.biases)
            ):
                weight = np.tensordot(
                    coefficients,
                    weights[indices],
                    axes=(0, 0),
                )
                bias = np.tensordot(
                    coefficients,
                    biases[indices],
                    axes=(0, 0),
                )
                hidden = weight @ hidden + bias
                if layer < 2:
                    hidden = self._elu(hidden)
            output = hidden * self.ystd + self.ymean
            sincos = output[293:351].reshape(NUM_JOINTS, 2)
            norms = np.maximum(
                np.linalg.norm(sincos, axis=1, keepdims=True),
                1.0e-6,
            )
            output[293:351] = (sincos / norms).reshape(-1)
            return output.astype(np.float64, copy=False)


def _joint_control_vectors() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    stiffness_5020 = _ARMATURE_5020 * _NATURAL_FREQ**2
    stiffness_7520_14 = _ARMATURE_7520_14 * _NATURAL_FREQ**2
    stiffness_7520_22 = _ARMATURE_7520_22 * _NATURAL_FREQ**2
    stiffness_4010 = _ARMATURE_4010 * _NATURAL_FREQ**2
    damping_5020 = (
        2.0 * _DAMPING_RATIO * _ARMATURE_5020 * _NATURAL_FREQ
    )
    damping_7520_14 = (
        2.0 * _DAMPING_RATIO * _ARMATURE_7520_14 * _NATURAL_FREQ
    )
    damping_7520_22 = (
        2.0 * _DAMPING_RATIO * _ARMATURE_7520_22 * _NATURAL_FREQ
    )
    damping_4010 = (
        2.0 * _DAMPING_RATIO * _ARMATURE_4010 * _NATURAL_FREQ
    )

    kp: list[float] = []
    kd: list[float] = []
    effort: list[float] = []
    for name in G1_29_JOINT_NAMES:
        if any(token in name for token in ("hip_pitch", "hip_roll", "knee")):
            kp.append(stiffness_7520_22)
            kd.append(damping_7520_22)
            effort.append(139.0)
        elif "hip_yaw" in name or name == "waist_yaw_joint":
            kp.append(stiffness_7520_14)
            kd.append(damping_7520_14)
            effort.append(88.0)
        elif "ankle_" in name or name in {
            "waist_roll_joint",
            "waist_pitch_joint",
        }:
            kp.append(2.0 * stiffness_5020)
            kd.append(2.0 * damping_5020)
            effort.append(50.0)
        elif "wrist_pitch" in name or "wrist_yaw" in name:
            kp.append(stiffness_4010)
            kd.append(damping_4010)
            effort.append(5.0)
        else:
            kp.append(stiffness_5020)
            kd.append(damping_5020)
            effort.append(25.0)
    kp_array = np.asarray(kp, dtype=np.float32)
    kd_array = np.asarray(kd, dtype=np.float32)
    scale_array = 0.25 * np.asarray(effort, dtype=np.float32) / kp_array
    return kp_array, kd_array, scale_array


@dataclass(frozen=True)
class WorldSample:
    sequence: int
    received_monotonic: float
    reset_count: int
    root_position: np.ndarray
    root_yaw: float
    height_map_z: np.ndarray
    movement: np.ndarray
    facing: np.ndarray
    desired_facing: np.ndarray
    speed_mps: float
    locomotion_mode: int
    mode: str
    safe_stop: bool

    @classmethod
    def from_packet(
        cls,
        packet: Mapping[str, Any],
        *,
        received_monotonic: float,
    ) -> "WorldSample":
        root = np.asarray(packet.get("root_position"), dtype=np.float64)
        height = np.asarray(packet.get("height_map_z"), dtype=np.float64)
        movement = np.asarray(packet.get("movement"), dtype=np.float64)
        facing = np.asarray(packet.get("facing"), dtype=np.float64)
        desired_facing = np.asarray(
            packet.get("desired_facing", packet.get("facing")),
            dtype=np.float64,
        )
        if root.shape != (3,) or not np.isfinite(root).all():
            raise ValueError("STATE root_position must be a finite 3-vector")
        if height.shape == (121,):
            height = height.reshape(11, 11)
        if height.shape != (11, 11) or not np.isfinite(height).all():
            raise ValueError("STATE height_map_z must be a finite 11x11 grid")
        if movement.shape != (3,) or not np.isfinite(movement).all():
            raise ValueError("STATE movement must be a finite 3-vector")
        if facing.shape != (3,) or not np.isfinite(facing).all():
            raise ValueError("STATE facing must be a finite 3-vector")
        if desired_facing.shape != (3,) or not np.isfinite(desired_facing).all():
            raise ValueError("STATE desired_facing must be a finite 3-vector")
        root_yaw = float(packet.get("root_yaw"))
        speed = float(packet.get("speed_mps"))
        if not math.isfinite(root_yaw) or not math.isfinite(speed):
            raise ValueError("STATE yaw and speed must be finite")
        return cls(
            sequence=int(packet.get("sequence")),
            received_monotonic=float(received_monotonic),
            reset_count=int(packet.get("reset_count")),
            root_position=root,
            root_yaw=root_yaw,
            height_map_z=height,
            movement=movement,
            facing=facing,
            desired_facing=desired_facing,
            speed_mps=speed,
            locomotion_mode=int(packet.get("locomotion_mode")),
            mode=str(packet.get("mode")),
            safe_stop=bool(packet.get("safe_stop")),
        )


def _load_module_from_path(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load module from {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


class BfmTeacherCore:
    """Robo-PFNN reference plus Teacher ONNX and exact G1 action contract."""

    def __init__(
        self,
        *,
        model_path: Path,
        realscan_root: Path,
        robo_pfnn_root: Path,
        weights_dir: Path,
        g1_xml: Path,
        formal_ik: Path,
        execution_provider: str,
        activation_blend_seconds: float = 0.1,
    ) -> None:
        if (
            not math.isfinite(activation_blend_seconds)
            or activation_blend_seconds <= 0.0
        ):
            raise ValueError("activation_blend_seconds must be finite and positive")
        source = realscan_root / "src"
        if str(source) not in sys.path:
            sys.path.insert(0, str(source))
        # The Matrix runtime is Python 3.10.  RealScan's command dataclass only
        # imports its TOML config module for type construction; this adapter
        # never parses TOML.  Provide an explicit fail-closed compatibility
        # module instead of mutating the environment with an unpinned package.
        if "tomllib" not in sys.modules:
            try:
                importlib.import_module("tomllib")
            except ModuleNotFoundError:
                tomllib_stub = types.ModuleType("tomllib")

                def unsupported_toml(*_args, **_kwargs):
                    raise RuntimeError(
                        "TOML parsing is unavailable in the Matrix BFM runtime"
                    )

                tomllib_stub.load = unsupported_toml
                tomllib_stub.loads = unsupported_toml
                sys.modules["tomllib"] = tomllib_stub
        self.command_module = importlib.import_module(
            "bfm_sonic_realscan_play.command"
        )
        self.teacher_module = importlib.import_module(
            "bfm_sonic_realscan_play.teacher_onnx"
        )
        self.reference_module = importlib.import_module(
            "bfm_sonic_realscan_play.robo_pfnn_reference"
        )
        ik_module = _load_module_from_path(
            "_matrix_bfm_formal_pfnn_ik",
            formal_ik,
        )
        self.reference_module._load_formal_pfnn_ik = lambda _weights: ik_module

        providers = (
            ["CUDAExecutionProvider", "CPUExecutionProvider"]
            if execution_provider == "cuda"
            else ["CPUExecutionProvider"]
        )
        self.teacher = self.teacher_module.TeacherOnnxPolicy(
            model_path,
            providers=providers,
        )
        self.forward = NumpyPfnnForward(weights_dir)
        self.stream = self.reference_module.RoboPfnnReferenceStream(
            repo=robo_pfnn_root,
            weights=weights_dir,
            g1_xml=g1_xml,
            device="cpu",
        )
        # The upstream stream accepts a preloaded forward instance.  Supplying
        # the NumPy implementation avoids a Torch dependency in Matrix's DDS
        # runtime while preserving the same four-bank network calculation.
        self.stream._forward = self.forward
        self.previous_action = np.zeros(NUM_JOINTS, dtype=np.float32)
        self.last_reset_count: int | None = None
        self.last_world_sequence: int | None = None
        self.reference_motion_active = False
        self.reference_start_resets = 0
        self.reference_stop_resets = 0
        self.reference_transition: str | None = None
        self.reference_hold_target: np.ndarray | None = None
        self.activation_blend_steps = max(
            2,
            int(round(float(activation_blend_seconds) * POLICY_HZ)),
        )
        self.activation_origin: np.ndarray | None = None
        self.activation_step = 0
        self.kp, self.kd, self.action_scale = _joint_control_vectors()
        self.default_joint_pos = np.asarray(
            self.teacher_module.SMP_DEFAULT_QPOS,
            dtype=np.float32,
        )
        self.isaac_to_matrix = np.argsort(
            self.teacher_module.MUJOCO_TO_ISAACLAB
        )
        self.dds_config = PolicyConfig(
            default_joint_pos=self.default_joint_pos.copy(),
            action_scale=self.action_scale.copy(),
            kp=self.kp.copy(),
            kd=self.kd.copy(),
            action_clip=ACTION_CLIP,
        )

    def reset(self) -> None:
        self.teacher.reset()
        self.stream.reset()
        self.previous_action.fill(0.0)
        self.last_world_sequence = None
        self.reference_motion_active = False
        self.reference_transition = None
        self.reference_hold_target = None
        self.activation_origin = None
        self.activation_step = 0

    def prepare_activation(self, lowstate: LowStateSnapshot) -> None:
        """Start a policy-consistent, no-teleport handoff from current joints.

        The upstream closed-loop runner teleports the simulated robot to the
        first reference frame before its first Teacher inference.  Matrix must
        preserve world state across a hot policy switch, so it instead clears
        the actor history and ramps from the currently observed joint pose.
        """

        self.teacher.reset()
        self.previous_action.fill(0.0)
        self.last_world_sequence = None
        self.reference_motion_active = False
        self.reference_transition = None
        self.reference_hold_target = None
        self.activation_origin = lowstate.joint_pos_rad.astype(
            np.float32,
            copy=True,
        )
        self.activation_step = 0

    def enter_standby(self) -> None:
        """Discard actor state produced while another controller owns LowCmd."""

        self.teacher.reset()
        self.previous_action.fill(0.0)
        self.last_world_sequence = None
        self.reference_motion_active = False
        self.reference_transition = None
        self.reference_hold_target = None
        self.activation_origin = None
        self.activation_step = 0

    def close(self) -> None:
        self.stream.close()

    def _command(
        self,
        sample: WorldSample,
        lowstate: LowStateSnapshot | None = None,
    ):
        movement_xy = sample.movement[:2]
        norm = float(np.linalg.norm(movement_xy))
        if (
            sample.safe_stop
            or sample.mode not in {"move", "turn"}
            or sample.speed_mps <= 1.0e-6
        ):
            world_velocity = np.zeros(2, dtype=np.float64)
        elif norm > 1.0e-8:
            world_velocity = movement_xy / norm * sample.speed_mps
        else:
            world_velocity = np.zeros(2, dtype=np.float64)
        cosine = math.cos(sample.root_yaw)
        sine = math.sin(sample.root_yaw)
        local_vx = cosine * world_velocity[0] + sine * world_velocity[1]
        local_vy = -sine * world_velocity[0] + cosine * world_velocity[1]
        requested_facing = sample.facing
        facing_norm = float(np.linalg.norm(requested_facing[:2]))
        if (
            sample.safe_stop
            or sample.mode not in {"move", "turn"}
            or facing_norm <= 1.0e-8
        ):
            yaw_rate = 0.0
        else:
            facing_yaw = math.atan2(
                requested_facing[1],
                requested_facing[0],
            )
            heading_error = math.atan2(
                math.sin(facing_yaw - sample.root_yaw),
                math.cos(facing_yaw - sample.root_yaw),
            )
            yaw_limit = FORMAL_COMMAND_YAW_LIMIT_RAD_S
            if sample.mode == "turn":
                measured_yaw_rate = (
                    float(lowstate.body_gyro_rad_s[2])
                    if lowstate is not None
                    else 0.0
                )
                heading_error = math.atan2(
                    math.sin(
                        heading_error
                        - TURN_COMMAND_YAW_DAMPING_SECONDS
                        * measured_yaw_rate
                    ),
                    math.cos(
                        heading_error
                        - TURN_COMMAND_YAW_DAMPING_SECONDS
                        * measured_yaw_rate
                    ),
                )
                yaw_limit = TURN_COMMAND_YAW_LIMIT_RAD_S
            yaw_rate = float(
                np.clip(
                    heading_error * FORMAL_COMMAND_YAW_GAIN,
                    -yaw_limit,
                    yaw_limit,
                )
            )
        if (
            sample.mode == "turn"
            and not sample.safe_stop
            and abs(yaw_rate) > 1.0e-6
            and abs(local_vx) <= 1.0e-8
            and abs(local_vy) <= 1.0e-8
        ):
            local_vx = TURN_REFERENCE_FORWARD_MPS
        moving = (
            abs(local_vx) > 1.0e-6
            or abs(local_vy) > 1.0e-6
            or abs(yaw_rate) > 1.0e-6
        )
        gait = (
            "stand"
            if not moving
            else ("jog" if sample.locomotion_mode == 3 else "walk")
        )
        return self.command_module.CommandSample(
            vx=float(local_vx),
            vy=float(local_vy),
            yaw_rate=yaw_rate,
            gait=gait,
            stop_latched=bool(sample.safe_stop),
        )

    def step(
        self,
        world: WorldSample,
        lowstate: LowStateSnapshot,
        *,
        active: bool = True,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        if self.last_reset_count is None:
            self.last_reset_count = world.reset_count
        elif world.reset_count != self.last_reset_count:
            self.reset()
            self.last_reset_count = world.reset_count
        if self.last_world_sequence == world.sequence:
            raise RuntimeError("BFM Teacher received a duplicate world sequence")
        if not active:
            # A resident shadow may warm inference kernels and advance the
            # reference generator, but its actions are not applied to the
            # robot.  Repeating the current frame matches IsaacLab's first
            # history append and prevents fictitious action/history buildup.
            self.teacher.reset()
            self.previous_action.fill(0.0)

        height_field = self.reference_module.LocalTerrainHeightField(
            world.root_position,
            world.root_yaw,
            world.height_map_z,
        )
        command = self._command(world, lowstate)
        command_motion_active = bool(
            command.gait != "stand"
            or abs(float(command.vx)) > 1.0e-6
            or abs(float(command.vy)) > 1.0e-6
            or abs(float(command.yaw_rate)) > 1.0e-6
        )
        start_reference_reset = bool(
            not self.reference_motion_active and command_motion_active
        )
        stop_reference_reset = bool(
            self.reference_motion_active and not command_motion_active
        )
        if start_reference_reset:
            # Let RoboPfnnReferenceStream branch from its continuously tracked
            # stand cursor in the background.  A synchronous reset performs a
            # full PFNN warmup and future-buffer fill on the 50 Hz policy
            # thread; the independent 500 Hz DDS publisher then repeats the
            # old stand LowCmd for seconds and can apply a stale walk command
            # after the operator has already released the key.
            self.reference_transition = "starting"
            self.reference_hold_target = lowstate.joint_pos_rad.astype(
                np.float32,
                copy=True,
            )
            self.previous_action.fill(0.0)
            self.activation_origin = None
            self.activation_step = 0
            self.reference_start_resets += 1
        elif stop_reference_reset:
            # A background walk -> stand branch must not leave the old walking
            # target in control.  Hold the exact observed pose until the stand
            # buffer is ready, then restart Teacher history and blend from the
            # current physical joints.
            self.reference_transition = "stopping"
            self.reference_hold_target = lowstate.joint_pos_rad.astype(
                np.float32,
                copy=True,
            )
            self.previous_action.fill(0.0)
            self.activation_origin = None
            self.activation_step = 0
            self.reference_stop_resets += 1
        self.reference_motion_active = command_motion_active
        requested_facing = world.facing
        requested_facing_yaw = math.atan2(
            requested_facing[1],
            requested_facing[0],
        )
        command_raw_heading_error = (
            math.atan2(
                math.sin(requested_facing_yaw - world.root_yaw),
                math.cos(requested_facing_yaw - world.root_yaw),
            )
            if (
                not world.safe_stop
                and world.mode in {"move", "turn"}
                and float(np.linalg.norm(requested_facing[:2])) > 1.0e-8
            )
            else 0.0
        )
        command_heading_error = command_raw_heading_error
        if world.mode == "turn" and not world.safe_stop:
            command_heading_error = math.atan2(
                math.sin(
                    command_raw_heading_error
                    - TURN_COMMAND_YAW_DAMPING_SECONDS
                    * float(lowstate.body_gyro_rad_s[2])
                ),
                math.cos(
                    command_raw_heading_error
                    - TURN_COMMAND_YAW_DAMPING_SECONDS
                    * float(lowstate.body_gyro_rad_s[2])
                ),
            )
        final_facing = getattr(world, "desired_facing", world.facing)
        final_facing_norm = float(np.linalg.norm(final_facing[:2]))
        command_final_heading_error = (
            math.atan2(
                math.sin(
                    math.atan2(final_facing[1], final_facing[0])
                    - world.root_yaw
                ),
                math.cos(
                    math.atan2(final_facing[1], final_facing[0])
                    - world.root_yaw
                ),
            )
            if (
                not world.safe_stop
                and world.mode in {"move", "turn"}
                and final_facing_norm > 1.0e-8
            )
            else 0.0
        )
        reference = self.stream.sample(
            command,
            world.root_position,
            world.root_yaw,
            height_field,
        )
        reference_buffer_swapped = bool(
            getattr(reference, "buffer_swapped", False)
        )
        reference_pending_rebuild = bool(reference.pending_rebuild)
        transition_completed = bool(
            self.reference_transition is not None
            and (
                reference_buffer_swapped
                or not reference_pending_rebuild
            )
        )
        if transition_completed:
            self.teacher.reset()
            self.previous_action.fill(0.0)
            if active:
                self.activation_origin = lowstate.joint_pos_rad.astype(
                    np.float32,
                    copy=True,
                )
                self.activation_step = 0
            else:
                self.activation_origin = None
                self.activation_step = 0
            self.reference_transition = None
            self.reference_hold_target = None
        holding_reference_transition = bool(
            self.reference_transition in {"starting", "stopping"}
            and self.reference_hold_target is not None
        )
        matrix_to_isaac = self.teacher_module.MUJOCO_TO_ISAACLAB
        observation = self.teacher_module.RobotObservation(
            base_quat_wxyz=lowstate.quaternion_wxyz,
            base_ang_vel=lowstate.body_gyro_rad_s,
            joint_pos=lowstate.joint_pos_rad[matrix_to_isaac],
            joint_vel=lowstate.joint_vel_rad_s[matrix_to_isaac],
            previous_action=self.previous_action,
        )
        action_isaac = self.teacher.step(
            reference.plan,
            observation,
            world.height_map_z,
        )
        action_isaac = np.clip(
            action_isaac,
            -ACTION_CLIP,
            ACTION_CLIP,
        ).astype(np.float32)
        action_matrix = action_isaac[self.isaac_to_matrix]
        desired_target = (
            self.default_joint_pos + self.action_scale * action_matrix
        ).astype(np.float32)
        blend_fraction = 1.0
        target = desired_target
        if active and holding_reference_transition:
            target = self.reference_hold_target.copy()
            blend_fraction = 0.0
        elif active and self.activation_origin is not None:
            progress = min(
                1.0,
                self.activation_step / float(self.activation_blend_steps - 1),
            )
            blend_fraction = progress * progress * (3.0 - 2.0 * progress)
            target = (
                self.activation_origin
                + blend_fraction * (desired_target - self.activation_origin)
            ).astype(np.float32)
            self.activation_step += 1
            if progress >= 1.0:
                self.activation_origin = None
        if active:
            if holding_reference_transition:
                # The old reference mode is deliberately discarded while the
                # requested branch builds.  Do not feed the held posture back
                # as a fictitious Teacher action.
                self.previous_action.fill(0.0)
            else:
                # The actor's PrevActions observation must describe the target
                # that Matrix actually published, including the no-teleport
                # blend.
                applied_matrix = (
                    target - self.default_joint_pos
                ) / self.action_scale
                self.previous_action = np.clip(
                    applied_matrix[
                        self.teacher_module.MUJOCO_TO_ISAACLAB
                    ],
                    -ACTION_CLIP,
                    ACTION_CLIP,
                ).astype(np.float32)
        else:
            self.previous_action.fill(0.0)
        self.last_world_sequence = world.sequence
        desired_delta = desired_target - lowstate.joint_pos_rad
        published_delta = target - lowstate.joint_pos_rad
        reference_joint = np.asarray(
            reference.plan.future_qpos[0, 7:],
            dtype=np.float32,
        )
        reference_delta = reference_joint - lowstate.joint_pos_rad
        reference_future_xy_delta_m = float(
            np.linalg.norm(
                np.asarray(reference.plan.future_qpos[-1, :2], dtype=np.float64)
                - np.asarray(reference.plan.future_qpos[0, :2], dtype=np.float64)
            )
        )
        turn_reference_seeded = bool(
            world.mode == "turn"
            and not world.safe_stop
            and abs(command.yaw_rate) > 1.0e-6
            and math.isclose(
                command.vx,
                TURN_REFERENCE_FORWARD_MPS,
                rel_tol=0.0,
                abs_tol=1.0e-12,
            )
            and math.isclose(
                command.vy,
                0.0,
                rel_tol=0.0,
                abs_tol=1.0e-12,
            )
        )
        return target, {
            "reference_replanned": bool(reference.replanned),
            "reference_reason": reference.replan_reason,
            "reference_plan_index": int(reference.plan_index),
            "reference_root_error_m": float(reference.root_error_before_m),
            "reference_pending_rebuild": reference_pending_rebuild,
            "reference_buffer_swapped": reference_buffer_swapped,
            "reference_transition": self.reference_transition,
            "reference_transition_completed": transition_completed,
            "reference_transition_holding": holding_reference_transition,
            "reference_start_reset": start_reference_reset,
            "reference_start_reset_count": int(self.reference_start_resets),
            "reference_stop_reset": stop_reference_reset,
            "reference_stop_reset_count": int(self.reference_stop_resets),
            "command_gait": command.gait,
            "command_vx_mps": float(command.vx),
            "command_vy_mps": float(command.vy),
            "command_yaw_rate_rad_s": float(command.yaw_rate),
            "command_heading_error_rad": float(command_heading_error),
            "command_raw_heading_error_rad": float(
                command_raw_heading_error
            ),
            "command_final_heading_error_rad": float(
                command_final_heading_error
            ),
            "command_heading_source": "matrix_wire_facing",
            "command_yaw_gain": FORMAL_COMMAND_YAW_GAIN,
            "command_yaw_limit_rad_s": (
                TURN_COMMAND_YAW_LIMIT_RAD_S
                if world.mode == "turn"
                else FORMAL_COMMAND_YAW_LIMIT_RAD_S
            ),
            "command_yaw_damping_seconds": (
                TURN_COMMAND_YAW_DAMPING_SECONDS
                if world.mode == "turn"
                else 0.0
            ),
            "command_speed_mps": float(math.hypot(command.vx, command.vy)),
            "turn_reference_seeded": turn_reference_seeded,
            "turn_reference_forward_mps": (
                float(command.vx) if turn_reference_seeded else 0.0
            ),
            "world_input_mode": world.mode,
            "world_input_safe_stop": bool(world.safe_stop),
            "world_input_speed_mps": float(world.speed_mps),
            "world_input_locomotion_mode": int(world.locomotion_mode),
            "reference_target_speed_mps": float(reference.plan.target_speed),
            "reference_future_xy_delta_m": reference_future_xy_delta_m,
            "shadow_preview": not active,
            "activation_blend_fraction": float(blend_fraction),
            "activation_blend_steps": int(self.activation_blend_steps),
            "raw_action_l2": float(np.linalg.norm(action_isaac)),
            "raw_action_max_abs": float(np.max(np.abs(action_isaac))),
            "desired_target_delta_rms_rad": float(
                math.sqrt(np.mean(np.square(desired_delta)))
            ),
            "desired_target_delta_max_rad": float(
                np.max(np.abs(desired_delta))
            ),
            "published_target_delta_rms_rad": float(
                math.sqrt(np.mean(np.square(published_delta)))
            ),
            "published_target_delta_max_rad": float(
                np.max(np.abs(published_delta))
            ),
            "reference_joint_error_rms_rad": float(
                math.sqrt(np.mean(np.square(reference_delta)))
            ),
        }


def _connect_control(path: Path, timeout_s: float = 10.0) -> socket.socket:
    socket_type = getattr(socket, "SOCK_SEQPACKET", None)
    if socket_type is None:
        raise RuntimeError("BFM Teacher requires AF_UNIX/SOCK_SEQPACKET")
    connection = socket.socket(socket.AF_UNIX, socket_type)
    deadline = time.monotonic() + timeout_s
    while True:
        try:
            connection.connect(str(path))
            break
        except (FileNotFoundError, ConnectionRefusedError):
            if time.monotonic() >= deadline:
                connection.close()
                raise
            time.sleep(0.02)
    connection.setblocking(False)
    return connection


class _ResidentLowCmdPublisher:
    """Repeat the newest policy target independently of 50 Hz inference.

    This thread is the sole caller of ``dds.write``.  The shared writer lock
    linearizes GO/PAUSE/STOP against every write, preserving the resident
    writer fence while allowing the latest immutable target to be reused.
    """

    def __init__(
        self,
        *,
        dds: UnitreeDdsRuntime,
        state_store: LatestLowState,
        handoff: HandoffStateMachine,
        policy_config: PolicyConfig,
        target_supplier: Callable[[], np.ndarray | None],
        writer_lock: threading.Lock,
        publish_hz: float = PUBLISH_HZ,
        lowstate_timeout_s: float = LOWSTATE_MAX_AGE_S,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        if not math.isfinite(publish_hz) or publish_hz <= 0.0:
            raise ValueError("publish_hz must be finite and positive")
        if (
            not math.isfinite(lowstate_timeout_s)
            or lowstate_timeout_s <= 0.0
        ):
            raise ValueError("lowstate_timeout_s must be finite and positive")
        self.dds = dds
        self.state_store = state_store
        self.handoff = handoff
        self.policy_config = policy_config
        self.target_supplier = target_supplier
        self.writer_lock = writer_lock
        self.publish_period_s = 1.0 / publish_hz
        self.lowstate_timeout_s = lowstate_timeout_s
        self.monotonic = monotonic
        self._stop = threading.Event()
        self._wake = threading.Event()
        self._thread: threading.Thread | None = None
        self._error: BaseException | None = None
        self._stats_lock = threading.Lock()
        self._publish_count = 0
        self._last_publish_monotonic: float | None = None
        self._last_publish_gap_s: float | None = None
        self._max_publish_gap_s = 0.0
        self._command_build_count = 0
        self._command_reuse_count = 0
        self._stale_command_reuse_count = 0
        self._stale_lowstate_max_age_s = 0.0
        self._cached_target: np.ndarray | None = None
        self._cached_mode_pr: int | None = None
        self._cached_mode_machine: int | None = None
        self._cached_command: Any | None = None

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("BFM LowCmd publisher was already started")
        self._thread = threading.Thread(
            target=self._run,
            name="matrix-bfm-lowcmd",
            daemon=True,
        )
        self._thread.start()

    def wake(self) -> None:
        self._wake.set()

    def _publish_once(self, now: float) -> bool:
        with self.writer_lock:
            if self.handoff.state != HandoffStateMachine.ACTIVE:
                return False
            target = self.target_supplier()
            if target is None:
                raise RuntimeError("BFM Teacher active writer has no policy target")
            state = self.state_store.get()
            if state is None:
                raise RuntimeError("BFM Teacher LowState is unavailable before publish")
            lowstate_age_s = now - state.received_monotonic
            if not math.isfinite(lowstate_age_s) or lowstate_age_s < 0.0:
                raise RuntimeError(
                    "BFM Teacher LowState age is invalid before publish"
                )
            stale_lowstate = lowstate_age_s > self.lowstate_timeout_s
            if (
                stale_lowstate
                and lowstate_age_s
                > self.lowstate_timeout_s + TRANSIENT_INPUT_STALE_GRACE_S
            ):
                raise RuntimeError(
                    "BFM Teacher LowState remained stale beyond transient grace"
                )
            mode_pr = int(state.mode_pr)
            mode_machine = int(state.mode_machine)
            if stale_lowstate:
                if self._cached_command is None:
                    raise RuntimeError(
                        "BFM Teacher LowState became stale before first publish"
                    )
                # The 50 Hz policy loop pauses inference during this same
                # transient.  Keep publishing its last admitted target so the
                # simulator does not observe an artificial LowCmd dropout.
                command = self._cached_command
                rebuild_command = False
            else:
                rebuild_command = bool(
                    target is not self._cached_target
                    or mode_pr != self._cached_mode_pr
                    or mode_machine != self._cached_mode_machine
                    or self._cached_command is None
                )
                if rebuild_command:
                    command = self.dds.make_low_cmd(
                        target,
                        self.policy_config,
                        state,
                    )
                    self._cached_target = target
                    self._cached_mode_pr = mode_pr
                    self._cached_mode_machine = mode_machine
                    self._cached_command = command
                else:
                    command = self._cached_command
                    assert command is not None
            if not self.dds.write(self.handoff.publisher, command):
                return False
            self.handoff.record_successful_write()
        completed = self.monotonic()
        with self._stats_lock:
            if self._last_publish_monotonic is not None:
                gap = completed - self._last_publish_monotonic
                if math.isfinite(gap) and gap >= 0.0:
                    self._last_publish_gap_s = gap
                    self._max_publish_gap_s = max(
                        self._max_publish_gap_s,
                        gap,
                    )
            self._last_publish_monotonic = completed
            self._publish_count += 1
            if rebuild_command:
                self._command_build_count += 1
            else:
                self._command_reuse_count += 1
            if stale_lowstate:
                self._stale_command_reuse_count += 1
                self._stale_lowstate_max_age_s = max(
                    self._stale_lowstate_max_age_s,
                    lowstate_age_s,
                )
        return True

    def _run(self) -> None:
        next_publish = self.monotonic()
        try:
            while not self._stop.is_set():
                with self.writer_lock:
                    active = (
                        self.handoff.state == HandoffStateMachine.ACTIVE
                    )
                if not active:
                    self._wake.wait(0.02)
                    self._wake.clear()
                    next_publish = self.monotonic()
                    continue
                now = self.monotonic()
                remaining = next_publish - now
                if remaining > 0.0:
                    self._stop.wait(min(remaining, 0.02))
                    continue
                self._publish_once(now)
                completed = self.monotonic()
                next_publish = _advance_deadline(
                    next_publish,
                    self.publish_period_s,
                    completed,
                )
        except BaseException as exc:
            self._error = exc
            self._stop.set()
            self._wake.set()

    def raise_if_failed(self) -> None:
        if self._error is not None:
            raise RuntimeError(
                f"BFM Teacher LowCmd publisher failed: {self._error}"
            ) from self._error

    def telemetry(self, *, now: float) -> dict[str, object]:
        with self._stats_lock:
            last_publish = self._last_publish_monotonic
            last_gap = self._last_publish_gap_s
            max_gap = self._max_publish_gap_s
            count = self._publish_count
            command_build_count = self._command_build_count
            command_reuse_count = self._command_reuse_count
            stale_command_reuse_count = self._stale_command_reuse_count
            stale_lowstate_max_age_s = self._stale_lowstate_max_age_s
        thread = self._thread
        return {
            "lowcmd_publish_count": count,
            "lowcmd_publish_last_age_ms": (
                max(0.0, now - last_publish) * 1000.0
                if last_publish is not None
                else None
            ),
            "lowcmd_publish_last_gap_ms": (
                last_gap * 1000.0 if last_gap is not None else None
            ),
            "lowcmd_publish_max_gap_ms": max_gap * 1000.0,
            "lowcmd_command_build_count": command_build_count,
            "lowcmd_command_reuse_count": command_reuse_count,
            "lowcmd_stale_command_reuse_count": stale_command_reuse_count,
            "lowcmd_stale_lowstate_max_age_ms": (
                stale_lowstate_max_age_s * 1000.0
            ),
            "lowcmd_publish_thread_alive": bool(
                thread is not None and thread.is_alive()
            ),
        }

    def stop(self) -> None:
        self._stop.set()
        self._wake.set()
        thread = self._thread
        if thread is None:
            return
        thread.join(timeout=2.0)
        if thread.is_alive() and self._error is None:
            self._error = RuntimeError(
                "BFM Teacher LowCmd publisher did not stop"
            )


def run_worker(
    *,
    core: BfmTeacherCore,
    dds: UnitreeDdsRuntime,
    state_store: LatestLowState,
    control_socket: Path,
    execution_provider: str,
    monotonic: Callable[[], float] = time.monotonic,
) -> int:
    connection = _connect_control(control_socket)
    authority_epoch = 0
    latest_world: WorldSample | None = None
    latest_target: np.ndarray | None = None
    target_lock = threading.Lock()
    writer_lock = threading.Lock()
    event_lock = threading.Lock()
    latest_policy_status: dict[str, Any] = {}
    warmed = False
    stopped_event_sent = False

    def send_event(event: str, fields: Mapping[str, Any] | None = None) -> None:
        payload: dict[str, Any] = {
            "schema": CONTROL_SCHEMA,
            "event": event,
            "policy_id": POLICY_ID,
            "authority_epoch": authority_epoch,
        }
        payload.update(fields or {})
        packet = json.dumps(
            payload,
            separators=(",", ":"),
            sort_keys=True,
            allow_nan=False,
        ).encode("utf-8")
        with event_lock:
            written = connection.send(packet)
        if written != len(packet):
            raise RuntimeError("short BFM Teacher event packet")

    handoff = HandoffStateMachine(dds.create_publisher, send_event)
    handoff.announce_ready(
        {
            "execution_provider": execution_provider,
            "model_input_dim": 1790,
            "action_dim": NUM_JOINTS,
            "writer_scope": "rt/lowcmd",
            "models_loaded_once": True,
            "models_warmed": False,
        }
    )

    def latest_target_snapshot() -> np.ndarray | None:
        with target_lock:
            return latest_target

    publisher = _ResidentLowCmdPublisher(
        dds=dds,
        state_store=state_store,
        handoff=handoff,
        policy_config=core.dds_config,
        target_supplier=latest_target_snapshot,
        writer_lock=writer_lock,
        monotonic=monotonic,
    )
    publisher.start()
    now = monotonic()
    next_policy = now
    next_status = now
    policy_period = 1.0 / POLICY_HZ
    status_period = 1.0
    transient_stale_active = False
    transient_stale_events = 0
    transient_stale_max_overage_s = 0.0

    try:
        while handoff.state != HandoffStateMachine.STOPPED:
            publisher.raise_if_failed()
            now = monotonic()
            deadline = min(next_policy, next_status)
            timeout = max(0.0, min(0.02, deadline - now))
            readable, _writable, _errors = select.select(
                (connection,),
                (),
                (),
                timeout,
            )
            if readable:
                while True:
                    try:
                        packet = connection.recv(65536)
                    except BlockingIOError:
                        break
                    if not packet:
                        raise EOFError("BFM Teacher supervisor disconnected")
                    payload = json.loads(packet.decode("utf-8"))
                    if (
                        not isinstance(payload, dict)
                        or payload.get("schema") != CONTROL_SCHEMA
                    ):
                        raise RuntimeError(
                            "BFM Teacher received an unsupported control packet"
                        )
                    command = str(payload.get("command", "")).upper()
                    if command == "STATE":
                        latest_world = WorldSample.from_packet(
                            payload,
                            received_monotonic=monotonic(),
                        )
                        continue
                    if command == "GO":
                        requested_epoch = int(payload.get("authority_epoch"))
                        if requested_epoch != authority_epoch + 1:
                            raise RuntimeError(
                                "BFM Teacher authority epoch did not advance"
                            )
                        if not warmed or latest_target is None:
                            raise RuntimeError(
                                "BFM Teacher GO arrived before shadow warmup"
                            )
                        now = monotonic()
                        state = state_store.get()
                        if (
                            latest_world is None
                            or now - latest_world.received_monotonic
                            > WORLD_SAMPLE_MAX_AGE_S
                            or state is None
                            or now - state.received_monotonic
                            > LOWSTATE_MAX_AGE_S
                        ):
                            raise RuntimeError(
                                "BFM Teacher GO lacks fresh world/LowState input"
                            )
                        if not latest_world.safe_stop:
                            raise RuntimeError(
                                "BFM Teacher GO requires a safety-stop handoff"
                        )
                        core.prepare_activation(state)
                        started = monotonic()
                        target, latest_policy_status = core.step(
                            latest_world,
                            state,
                            active=True,
                        )
                        with target_lock:
                            latest_target = target
                        latest_policy_status["inference_ms"] = (
                            monotonic() - started
                        ) * 1000.0
                        authority_epoch = requested_epoch
                        with writer_lock:
                            handoff.command("GO")
                        publisher.wake()
                        next_policy = monotonic() + policy_period
                        continue
                    if command == "PAUSE":
                        with writer_lock:
                            handoff.command("PAUSE")
                        publisher.wake()
                        core.enter_standby()
                        continue
                    if command == "STOP":
                        with writer_lock:
                            handoff.command("STOP")
                        publisher.wake()
                        stopped_event_sent = True
                        continue
                    raise RuntimeError(
                        f"unsupported BFM Teacher command: {command!r}"
                    )

            now = monotonic()
            if now >= next_policy:
                state = state_store.get()
                world_age_s = (
                    now - latest_world.received_monotonic
                    if latest_world is not None
                    else None
                )
                state_age_s = (
                    now - state.received_monotonic
                    if state is not None
                    else None
                )
                world_fresh = (
                    world_age_s is not None
                    and 0.0 <= world_age_s <= WORLD_SAMPLE_MAX_AGE_S
                )
                state_fresh = (
                    state_age_s is not None
                    and 0.0 <= state_age_s <= LOWSTATE_MAX_AGE_S
                )
                if world_fresh and state_fresh:
                    transient_stale_active = False
                    assert latest_world is not None
                    assert state is not None
                    started = monotonic()
                    try:
                        target, latest_policy_status = core.step(
                            latest_world,
                            state,
                            active=(
                                handoff.state == HandoffStateMachine.ACTIVE
                            ),
                        )
                    except RuntimeError as exc:
                        if "duplicate world sequence" not in str(exc):
                            raise
                    else:
                        with target_lock:
                            latest_target = target
                        latest_policy_status["inference_ms"] = (
                            monotonic() - started
                        ) * 1000.0
                        if not warmed:
                            warmed = True
                            send_event(
                                "WARMED_NO_WRITER",
                                {
                                    "writer_created": False,
                                    "models_loaded_once": True,
                                    "models_warmed": True,
                                },
                            )
                elif handoff.state == HandoffStateMachine.ACTIVE:
                    ages = (world_age_s, state_age_s)
                    if any(
                        age is None
                        or not math.isfinite(age)
                        or age < 0.0
                        for age in ages
                    ):
                        raise RuntimeError(
                            "BFM Teacher active writer received invalid "
                            "world/LowState freshness"
                        )
                    assert world_age_s is not None
                    assert state_age_s is not None
                    stale_overage_s = max(
                        world_age_s - WORLD_SAMPLE_MAX_AGE_S,
                        state_age_s - LOWSTATE_MAX_AGE_S,
                        0.0,
                    )
                    if not transient_stale_active:
                        transient_stale_active = True
                        transient_stale_events += 1
                    transient_stale_max_overage_s = max(
                        transient_stale_max_overage_s,
                        stale_overage_s,
                    )
                    if stale_overage_s > TRANSIENT_INPUT_STALE_GRACE_S:
                        raise RuntimeError(
                            "BFM Teacher active writer lost fresh "
                            "world/LowState input beyond transient grace"
                        )
                else:
                    transient_stale_active = False
                next_policy = _advance_deadline(
                    next_policy,
                    policy_period,
                    now,
                )

            if now >= next_status:
                state = state_store.get()
                status: dict[str, Any] = {
                    "writer_created": handoff.publisher is not None,
                    "write_authorized": (
                        handoff.state == HandoffStateMachine.ACTIVE
                    ),
                    "controller": (
                        POLICY_ID
                        if handoff.state == HandoffStateMachine.ACTIVE
                        else handoff.state
                    ),
                    "models_loaded_once": True,
                    "models_warmed": warmed,
                    "transient_input_stale": transient_stale_active,
                    "transient_input_stale_events": transient_stale_events,
                    "transient_input_stale_grace_ms": (
                        TRANSIENT_INPUT_STALE_GRACE_S * 1000.0
                    ),
                    "transient_input_stale_max_overage_ms": (
                        transient_stale_max_overage_s * 1000.0
                    ),
                    "world_sample_sequence": (
                        latest_world.sequence
                        if latest_world is not None
                        else None
                    ),
                    **publisher.telemetry(now=monotonic()),
                    **latest_policy_status,
                }
                if state is not None:
                    status.update(state_status(state))
                send_event("STATUS", status)
                next_status = _advance_deadline(
                    next_status,
                    status_period,
                    now,
                )
        return 0
    except Exception as exc:
        try:
            send_event(
                "ERROR",
                {
                    "message": str(exc),
                    "writer_created": handoff.publisher is not None,
                },
            )
        except Exception:
            pass
        raise
    finally:
        publisher.stop()
        with writer_lock:
            handoff.close_writer()
        if not stopped_event_sent:
            try:
                send_event("STOPPED", {"writer_created": False})
            except Exception:
                pass
        connection.close()


def validate_artifacts(args: argparse.Namespace) -> None:
    require_source_checkout(
        args.bfm_source_root,
        BFM_SOURCE_COMMIT,
        "BFM-SONIC",
    )
    require_source_checkout(
        args.realscan_source_root,
        REALSCAN_SOURCE_COMMIT,
        "BFM RealScan adapter",
    )
    require_source_checkout(
        args.robo_pfnn_root,
        ROBO_PFNN_SOURCE_COMMIT,
        "Robo-PFNN",
    )
    require_file_sha256(
        args.model,
        TEACHER_ONNX_SHA256,
        "Teacher50k ONNX",
    )
    require_file_sha256(
        args.config,
        TEACHER_CONFIG_SHA256,
        "Teacher50k config",
    )
    require_file_sha256(
        args.g1_xml,
        ROBO_PFNN_G1_XML_SHA256,
        "Robo-PFNN G1 XML",
    )
    require_file_sha256(
        args.formal_ik,
        ROBO_PFNN_IK_SHA256,
        "formal7168 PFNN IK",
    )
    tree_sha256, file_count = directory_tree_sha256(args.weights)
    if file_count != 305 or tree_sha256 != ROBO_PFNN_WEIGHTS_TREE_SHA256:
        raise ValueError(
            "formal7168 Robo-PFNN weight tree mismatch: "
            f"files={file_count} expected_sha={ROBO_PFNN_WEIGHTS_TREE_SHA256} "
            f"actual_sha={tree_sha256}"
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--bfm-source-root", required=True, type=Path)
    parser.add_argument("--realscan-source-root", required=True, type=Path)
    parser.add_argument("--robo-pfnn-root", required=True, type=Path)
    parser.add_argument("--weights", required=True, type=Path)
    parser.add_argument("--g1-xml", required=True, type=Path)
    parser.add_argument("--formal-ik", required=True, type=Path)
    parser.add_argument("--interface", default="lo")
    parser.add_argument("--control-socket", type=Path)
    parser.add_argument(
        "--execution-provider",
        choices=("cuda", "cpu"),
        default="cuda",
    )
    parser.add_argument(
        "--activation-blend-seconds",
        type=float,
        default=0.1,
        help=(
            "smooth no-teleport takeover duration; the actor history records "
            "the actually published blended targets"
        ),
    )
    parser.add_argument(
        "--validate-only",
        action="store_true",
        help="Validate hashes, load both models, then exit without DDS",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    validate_artifacts(args)
    core = BfmTeacherCore(
        model_path=args.model,
        realscan_root=args.realscan_source_root,
        robo_pfnn_root=args.robo_pfnn_root,
        weights_dir=args.weights,
        g1_xml=args.g1_xml,
        formal_ik=args.formal_ik,
        execution_provider=args.execution_provider,
        activation_blend_seconds=args.activation_blend_seconds,
    )
    if args.validate_only:
        core.close()
        print(
            json.dumps(
                {
                    "policy_id": POLICY_ID,
                    "status": "validated",
                    "model_input_dim": 1790,
                    "action_dim": NUM_JOINTS,
                    "weights_tree_sha256": ROBO_PFNN_WEIGHTS_TREE_SHA256,
                },
                sort_keys=True,
            )
        )
        return 0
    if args.control_socket is None:
        raise SystemExit("--control-socket is required outside --validate-only")
    state_store = LatestLowState()
    dds = UnitreeDdsRuntime(
        interface=args.interface,
        state_store=state_store,
    )
    try:
        return run_worker(
            core=core,
            dds=dds,
            state_store=state_store,
            control_socket=args.control_socket,
            execution_provider=args.execution_provider,
        )
    finally:
        core.close()


if __name__ == "__main__":
    raise SystemExit(main())
