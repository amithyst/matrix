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

# A hot locomotion handoff is prepared while the existing controller still
# owns LowCmd.  The BFM worker resets its online PFNN stream onto the current
# physical root/terrain, runs four writer-free policy ticks, and admits the
# next authority epoch only if the target that would be published remains a
# small continuation of the measured joint pose.  These are writer-safety
# invariants, not policy/action-scale tuning parameters.
HOT_SWITCH_PREVIEW_STEPS = 4
HOT_SWITCH_MAX_TARGET_DELTA_RAD = 0.12
REFERENCE_SWAP_MAX_TARGET_DELTA_RAD = 0.03
# Ordinary W release stays inside the BFM controller. Delaying the stand
# command was only needed by the experimental BFM -> SONIC brake transaction;
# it diverges from the accepted BFM-3DGS press/release contract and destroys
# the continuous walk -> stand policy history.
STOP_HANDOFF_GRACE_STEPS = 0
HOT_SWITCH_MAX_REFERENCE_ROOT_ERROR_M = 0.25
HOT_SWITCH_LOW_CMD_MAX_AGE_S = 0.10

BFM_SOURCE_COMMIT = "5e264ae2bee2315dc0522c48c64b4506977b2e25"
REALSCAN_SOURCE_COMMIT = "850a71bef1e1472aaeb3ff4cb9004d9848830cfc"
ROBO_PFNN_SOURCE_COMMIT = "eb1b8b8001a221d2147f8daa073ca447acc8649e"
TEACHER_ONNX_SHA256 = (
    "243a839d325f7b214ff40367d0c2fb32d5a36c7ef0e869b85b70a428212f37b1"
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
CONTRACT_SOURCE_HASHES = {
    "bfm_source_commit": BFM_SOURCE_COMMIT,
    "realscan_source_commit": REALSCAN_SOURCE_COMMIT,
    "robo_pfnn_source_commit": ROBO_PFNN_SOURCE_COMMIT,
    "teacher_onnx_sha256": TEACHER_ONNX_SHA256,
    "teacher_config_sha256": TEACHER_CONFIG_SHA256,
    "robo_pfnn_weights_tree_sha256": ROBO_PFNN_WEIGHTS_TREE_SHA256,
    "robo_pfnn_g1_xml_sha256": ROBO_PFNN_G1_XML_SHA256,
    "formal7168_ik_sha256": ROBO_PFNN_IK_SHA256,
}
CONTRACT_DIMS = {
    "model_input_dim": 1790,
    "tokenizer_dim": 761,
    "command_dim": 580,
    "height_map_dim": 121,
    "orientation_dim": 60,
    "actor_observation_dim": 1029,
    "history_length": 10,
    "compatibility_zero_dim": 99,
    "action_dim": NUM_JOINTS,
}

# Keep the online reference stream byte-for-byte explicit with the accepted
# BFM-3DGS/RealScan base.toml contract.  RoboPfnnReferenceStream's library
# default for root_reanchor_threshold_m is intentionally more permissive
# (0.25 m); relying on that default lets a fast 0.9 m/s reference get much
# farther ahead of the Matrix G1 before it is pulled back.  The qualified
# interactive runner uses 0.15 m and passes every field explicitly.
REALSCAN_REFERENCE_CONTRACT = {
    "source_hz": 60.0,
    "output_hz": 50.0,
    "future_frames": 10,
    "future_stride_steps": 5,
    "buffer_frames": 47,
    "warmup_source_steps": 60,
    "root_reanchor_threshold_m": 0.15,
    "root_follow_deadband_m": 0.015,
    "root_follow_gain": 0.25,
    "root_follow_max_step_m": 0.015,
    "command_replan_linear_delta_mps": 0.05,
    "command_replan_yaw_delta_rad_s": 0.05,
    "pending_min_steps": 4,
    "pending_extra_frames": 8,
}


@dataclass(frozen=True)
class LowCmdTargetSnapshot:
    """Newest finite 29-DoF position target observed on ``rt/lowcmd``."""

    joint_pos_rad: np.ndarray
    received_monotonic: float

    @classmethod
    def validated(
        cls,
        *,
        joint_pos_rad: Any,
        received_monotonic: float,
    ) -> "LowCmdTargetSnapshot":
        values = np.asarray(joint_pos_rad, dtype=np.float32).reshape(-1)
        received = float(received_monotonic)
        if values.shape != (NUM_JOINTS,) or not np.all(np.isfinite(values)):
            raise ValueError("LowCmd target must be finite 29D")
        if not math.isfinite(received):
            raise ValueError("LowCmd target timestamp must be finite")
        return cls(values.copy(), received)


class LatestLowCmdTarget:
    """Thread-safe capture of the currently authoritative DDS position target."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._target: LowCmdTargetSnapshot | None = None

    def set(self, target: LowCmdTargetSnapshot) -> None:
        with self._lock:
            self._target = target

    def get(self) -> LowCmdTargetSnapshot | None:
        with self._lock:
            return self._target


class BfmUnitreeDdsRuntime(UnitreeDdsRuntime):
    """BFM DDS adapter that also observes the active controller's LowCmd.

    A measured joint pose is not a torque-continuous handoff anchor for a PD
    controller: commanding q_measured removes the position error that was
    supporting the robot against gravity.  Capture the actual active LowCmd
    target so BFM can begin from the same commanded pose without creating a
    second writer before GO.
    """

    def __init__(
        self,
        *,
        interface: str,
        state_store: LatestLowState,
        command_store: LatestLowCmdTarget,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        super().__init__(
            interface=interface,
            state_store=state_store,
            monotonic=monotonic,
        )
        channel = importlib.import_module("unitree_sdk2py.core.channel")
        messages = importlib.import_module("unitree_sdk2py.idl.unitree_hg.msg.dds_")
        self._command_store = command_store
        self._command_monotonic = monotonic
        self._command_subscriber = channel.ChannelSubscriber(
            "rt/lowcmd",
            messages.LowCmd_,
        )
        self._command_subscriber.Init(self._on_low_cmd, 10)

    def _on_low_cmd(self, message: Any) -> None:
        try:
            motors = message.motor_cmd
            if len(motors) < NUM_JOINTS:
                return
            # An authoritative locomotion command drives every G1 joint in
            # position mode.  Ignore zero/default or partially built packets.
            if any(int(motors[index].mode) != 1 for index in range(NUM_JOINTS)):
                return
            target = LowCmdTargetSnapshot.validated(
                joint_pos_rad=[
                    motors[index].q for index in range(NUM_JOINTS)
                ],
                received_monotonic=self._command_monotonic(),
            )
        except (AttributeError, IndexError, TypeError, ValueError):
            return
        self._command_store.set(target)

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
        elif (
            "hip_yaw" in name
            or name == "waist_yaw_joint"
        ):
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


def _handoff_input_is_neutral(
    world: WorldSample,
    *,
    allow_idle_neutral: bool,
) -> bool:
    """Validate the two explicit writer-fenced BFM handoff inputs."""

    if world.safe_stop:
        return True
    return bool(
        allow_idle_neutral
        and world.mode == "idle"
        and world.locomotion_mode == 0
        and abs(float(world.speed_mps)) <= 1.0e-6
        and float(np.linalg.norm(world.movement)) <= 1.0e-6
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
        reference_clip: Path | None = None,
        direct_start: bool = False,
        trace_file: Path | None = None,
        trace_ticks: int = 0,
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
        self.recorded_reference_module = importlib.import_module(
            "bfm_sonic_realscan_play.recorded_reference"
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
        if reference_clip is None:
            self.stream = self.reference_module.RoboPfnnReferenceStream(
                repo=robo_pfnn_root,
                weights=weights_dir,
                g1_xml=g1_xml,
                device="cuda:0",
                **REALSCAN_REFERENCE_CONTRACT,
            )
            self.reference_source = "robo_pfnn_formal7168"
        else:
            self.stream = self.recorded_reference_module.RecordedMotionReferenceStream(
                reference_clip,
            )
            self.reference_source = (
                f"formal7168_clip:{self.stream.motion_key}"
            )
        self.direct_start = bool(direct_start)
        self.direct_reference_start: dict[str, list[float]] | None = None
        self.previous_action = np.zeros(NUM_JOINTS, dtype=np.float32)
        self.last_reset_count: int | None = None
        self.last_world_sequence: int | None = None
        self.reference_motion_active = False
        self.last_motion_command: Any | None = None
        self.stop_handoff_grace_steps_remaining = 0
        self.reference_start_resets = 0
        self.reference_stop_resets = 0
        self.reference_transition: str | None = None
        self.reference_hold_target: np.ndarray | None = None
        self.reference_stop_blend_pending = False
        # The accepted BFM-3DGS loop keeps applying Teacher on the active
        # reference while Robo-PFNN builds a pending branch, then swaps that
        # branch atomically.  Freezing joints or clearing actor history during
        # the build changes the trained closed loop and can make root error
        # cross the hard-reanchor threshold immediately after W.
        self.canonical_reference_continuity = True
        self.idle_anchor_target: np.ndarray | None = None
        self.idle_anchor_enabled = True
        self.activation_blend_steps = max(
            2,
            int(round(float(activation_blend_seconds) * POLICY_HZ)),
        )
        self.activation_origin: np.ndarray | None = None
        self.activation_step = 0
        self.activation_target_step_limit_rad = HOT_SWITCH_MAX_TARGET_DELTA_RAD
        self.last_published_target: np.ndarray | None = None
        self.trace_file = trace_file
        self.trace_ticks = max(0, int(trace_ticks))
        self.trace_written = 0
        if self.trace_file is not None and self.trace_ticks > 0:
            self.trace_file.parent.mkdir(parents=True, exist_ok=True)
            self.trace_file.write_text("", encoding="utf-8")
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
        self.last_motion_command = None
        self.stop_handoff_grace_steps_remaining = 0
        self.reference_transition = None
        self.reference_hold_target = None
        self.reference_stop_blend_pending = False
        self.idle_anchor_target = None
        self.idle_anchor_enabled = True
        self.activation_origin = None
        self.activation_step = 0
        self.activation_target_step_limit_rad = HOT_SWITCH_MAX_TARGET_DELTA_RAD
        self.last_published_target = None
        self.direct_reference_start = None

    def prepare_activation(
        self,
        lowstate: LowStateSnapshot,
        prior_target: np.ndarray | None = None,
        *,
        idle_anchor_enabled: bool = True,
    ) -> None:
        """Start a policy-consistent handoff from the active LowCmd target.

        The upstream closed-loop runner teleports the simulated robot to the
        first reference frame before its first Teacher inference.  Matrix must
        preserve world state across a hot policy switch, so it clears actor
        history and ramps from the prior controller's observed LowCmd target.
        """

        self.teacher.reset()
        self.previous_action.fill(0.0)
        self.last_world_sequence = None
        self.reference_motion_active = False
        self.last_motion_command = None
        self.stop_handoff_grace_steps_remaining = 0
        self.reference_transition = None
        self.reference_hold_target = None
        self.reference_stop_blend_pending = False
        anchor = (
            lowstate.joint_pos_rad
            if prior_target is None
            else np.asarray(prior_target, dtype=np.float32)
        )
        if anchor.shape != (NUM_JOINTS,) or not np.all(np.isfinite(anchor)):
            raise ValueError("handoff anchor target must be finite 29D")
        self.idle_anchor_enabled = bool(idle_anchor_enabled)
        self.idle_anchor_target = (
            anchor.astype(np.float32, copy=True)
            if self.idle_anchor_enabled
            else None
        )
        self.activation_origin = anchor.astype(np.float32, copy=True)
        self.activation_step = 0
        self.activation_target_step_limit_rad = HOT_SWITCH_MAX_TARGET_DELTA_RAD
        self.last_published_target = anchor.astype(np.float32, copy=True)

    def prepare_handoff_activation(
        self,
        lowstate: LowStateSnapshot,
        prior_target: np.ndarray,
    ) -> None:
        """Re-root online PFNN and prepare a writer-free hot-switch preview."""

        self.stream.reset()
        self.prepare_activation(lowstate, prior_target)
        self.reference_transition = "handoff"
        self.reference_hold_target = np.asarray(
            prior_target,
            dtype=np.float32,
        ).copy()

    def prepare_direct_activation(self) -> None:
        """Reset actor history without perturbing the aligned reference cursor."""

        self.teacher.reset()
        self.previous_action.fill(0.0)
        self.last_world_sequence = None
        self.reference_motion_active = True
        self.last_motion_command = None
        self.stop_handoff_grace_steps_remaining = 0
        self.reference_transition = None
        self.reference_hold_target = None
        self.reference_stop_blend_pending = False
        self.idle_anchor_target = None
        self.idle_anchor_enabled = False
        self.activation_origin = None
        self.activation_step = 0
        self.activation_target_step_limit_rad = HOT_SWITCH_MAX_TARGET_DELTA_RAD
        self.last_published_target = None

    def prepare_aligned_initial_activation(self) -> None:
        """Start the canonical Teacher loop after an exact PFNN pose write.

        This is deliberately different from a live controller hot-switch.  The
        physical G1 has already been written to the first PFNN root, velocity,
        and joint state while every LowCmd writer is fenced.  Applying the
        hot-switch blend/step limiter after that exact alignment changes the
        Teacher action sequence and can keep the closed loop permanently one
        or more targets behind.  The accepted BFM-3DGS runner publishes the
        first Teacher target directly from the aligned state, so the Matrix
        initial-policy path must do the same.
        """

        self.prepare_direct_activation()
        # Direct runs are admitted with a non-zero command and therefore mark
        # the reference as moving.  The default game policy is admitted while
        # safely standing, so preserve the already-warmed stand branch instead
        # of manufacturing a motion-to-stand transition on the first tick.
        self.reference_motion_active = False
        self.last_motion_command = None
        self.stop_handoff_grace_steps_remaining = 0

    def enter_standby(self) -> None:
        """Discard actor state produced while another controller owns LowCmd."""

        self.teacher.reset()
        self.previous_action.fill(0.0)
        self.last_world_sequence = None
        self.reference_motion_active = False
        self.reference_transition = None
        self.reference_hold_target = None
        self.reference_stop_blend_pending = False
        self.idle_anchor_target = None
        self.idle_anchor_enabled = True
        self.activation_origin = None
        self.activation_step = 0
        self.activation_target_step_limit_rad = HOT_SWITCH_MAX_TARGET_DELTA_RAD
        self.last_published_target = None

    def _command_continuity_anchor(
        self,
        lowstate: LowStateSnapshot,
    ) -> np.ndarray:
        """Return the last applied target, with measured q only as cold fallback."""

        last_target = getattr(self, "last_published_target", None)
        if last_target is not None:
            return last_target.astype(np.float32, copy=True)
        return lowstate.joint_pos_rad.astype(np.float32, copy=True)

    def _prepare_realtime_rolling_command(self, command: Any) -> str | None:
        """Roll an ordinary command change without a PFNN publish barrier.

        The accepted RealScan stream builds a complete replacement branch in
        the background and then waits synchronously at its deterministic swap
        step. That is simulation-time safe, but Matrix physics keeps running:
        while ``Future.result()`` blocks, the independent LowCmd publisher can
        only repeat one stale walking target for hundreds of milliseconds.

        For ordinary, non-latched keyboard motion, keep the active cursor and
        let its existing 47-frame horizon roll into each new command one frame
        per policy tick. This preserves PFNN phase, Teacher history,
        PrevActions, and the 50 Hz writer. Explicit safety-stop latches retain
        the upstream branch/rebuild behavior.
        """

        if bool(getattr(command, "stop_latched", False)):
            return None
        stream = self.stream
        if not (
            hasattr(stream, "_frames")
            and hasattr(stream, "_last_branch_command")
        ):
            return None
        pending_cancelled = False
        pending = getattr(stream, "_pending", None)
        if pending is not None:
            # Root-anchor and terrain refreshes can start a pending branch even
            # when the keyboard command itself is unchanged. Cancel it before
            # the upstream four-step poll reaches Future.result(); the active
            # buffer has already received the continuous root translation and
            # keeps appending frames against the current height field.
            pending.cancel_event.set()
            pending.future.cancel()
            stream._pending = None
            pending_cancelled = True
            if hasattr(stream, "_discard_count"):
                stream._discard_count += 1
            if hasattr(stream, "_deferred_reason"):
                stream._deferred_reason = None
        previous = getattr(stream, "_last_branch_command", None)
        if previous is None:
            return "pending_cancel" if pending_cancelled else None
        change_reason = getattr(stream, "_change_reason", None)
        if callable(change_reason):
            reason = change_reason(command, previous)
        elif getattr(command, "gait", None) != getattr(previous, "gait", None):
            reason = "gait"
        else:
            reason = None
        if reason is None:
            return "pending_cancel" if pending_cancelled else None
        if hasattr(stream, "_deferred_reason"):
            stream._deferred_reason = None
        stream._last_branch_command = command
        return str(reason)

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
        requested_facing = sample.facing
        facing_norm = float(np.linalg.norm(requested_facing[:2]))
        # Match the accepted BFM-3DGS keyboard contract: W/Shift-W are PFNN
        # local forward walk/jog commands and carry no implicit yaw feedback.
        # Matrix has already expressed movement in the camera-facing world
        # frame, so rotate it back by that same facing.  A/D arrives as the
        # explicit ``turn`` mode and remains the sole yaw command surface.
        movement_frame_yaw = (
            math.atan2(requested_facing[1], requested_facing[0])
            if sample.mode == "move" and facing_norm > 1.0e-8
            else sample.root_yaw
        )
        cosine = math.cos(movement_frame_yaw)
        sine = math.sin(movement_frame_yaw)
        local_vx = cosine * world_velocity[0] + sine * world_velocity[1]
        local_vy = -sine * world_velocity[0] + cosine * world_velocity[1]
        if (
            sample.safe_stop
            or sample.mode != "turn"
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
            # A transient Matrix deadman frame is how the engine bridge
            # releases a synthetic/remote key lease. BFM-3DGS treats W release
            # as an ordinary non-latched stand command. Reserve the reference
            # stop latch for explicit safety-stop modes so deadman -> idle does
            # not manufacture a second, equivalent gait rebuild.
            stop_latched=bool(
                sample.safe_stop and sample.mode not in {"deadman", "idle"}
            ),
        )

    @staticmethod
    def _array_summary(values: Any) -> dict[str, Any]:
        array = np.asarray(values)
        finite = bool(np.all(np.isfinite(array)))
        summary: dict[str, Any] = {
            "shape": [int(value) for value in array.shape],
            "finite": finite,
        }
        if array.size:
            summary.update(
                {
                    "min": float(np.min(array)),
                    "max": float(np.max(array)),
                    "mean": float(np.mean(array)),
                }
            )
        return summary

    @staticmethod
    def _reference_plan_summary(plan: Any) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for name in ("future_qpos", "future_qvel", "target_speed"):
            value = getattr(plan, name, None)
            if value is None:
                result[name] = None
            elif name == "target_speed":
                result[name] = float(value)
            else:
                result[name] = BfmTeacherCore._array_summary(value)
        return result

    @staticmethod
    def _max_abs_entry(
        values: np.ndarray,
        names: list[str] | tuple[str, ...],
    ) -> dict[str, Any]:
        array = np.asarray(values, dtype=np.float64).reshape(-1)
        if array.size == 0:
            return {
                "max_abs": None,
                "argmax": None,
                "joint_name": None,
                "signed_value": None,
            }
        index = int(np.argmax(np.abs(array)))
        return {
            "max_abs": float(abs(array[index])),
            "argmax": index,
            "joint_name": names[index] if index < len(names) else None,
            "signed_value": float(array[index]),
        }

    @staticmethod
    def _yaw_from_wxyz(quaternion: np.ndarray) -> float | None:
        values = np.asarray(quaternion, dtype=np.float64).reshape(-1)
        if values.shape != (4,) or not np.all(np.isfinite(values)):
            return None
        norm = float(np.linalg.norm(values))
        if norm <= 1.0e-12:
            return None
        w, x, y, z = (values / norm).tolist()
        return float(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))

    def _joint_order_ledger(self) -> dict[str, Any]:
        matrix_names = list(G1_29_JOINT_NAMES)
        isaac_to_matrix = [int(value) for value in self.isaac_to_matrix.tolist()]
        matrix_to_isaac = [
            int(value) for value in self.teacher_module.MUJOCO_TO_ISAACLAB.tolist()
        ]
        isaac_names = [matrix_names[matrix_index] for matrix_index in isaac_to_matrix]
        return {
            "matrix_mujoco_actuator_order": [
                {
                    "matrix_index": matrix_index,
                    "joint_name": joint_name,
                    "isaac_index": matrix_to_isaac[matrix_index],
                }
                for matrix_index, joint_name in enumerate(matrix_names)
            ],
            "isaac_action_order": [
                {
                    "isaac_index": isaac_index,
                    "joint_name": joint_name,
                    "matrix_index": isaac_to_matrix[isaac_index],
                }
                for isaac_index, joint_name in enumerate(isaac_names)
            ],
        }

    def _reference_continuity_summary(
        self,
        plan: Any,
        lowstate: LowStateSnapshot,
    ) -> dict[str, Any]:
        raw_future_qpos = getattr(plan, "future_qpos", None)
        if raw_future_qpos is None:
            return {
                "valid": False,
                "foot_ik_observable": "unavailable",
            }
        future_qpos = np.asarray(raw_future_qpos, dtype=np.float64)
        if future_qpos.shape != (10, 36) or not np.all(np.isfinite(future_qpos)):
            return {
                "valid": False,
                "foot_ik_observable": "unavailable",
            }
        yaw_0 = self._yaw_from_wxyz(future_qpos[0, 3:7])
        yaw_1 = self._yaw_from_wxyz(future_qpos[1, 3:7])
        yaw_delta = (
            None
            if yaw_0 is None or yaw_1 is None
            else float(math.atan2(math.sin(yaw_1 - yaw_0), math.cos(yaw_1 - yaw_0)))
        )
        reference_joint = future_qpos[0, 7:].astype(np.float32)
        reference_joint_delta = reference_joint - lowstate.joint_pos_rad
        root_delta_01 = future_qpos[1, :3] - future_qpos[0, :3]
        return {
            "valid": True,
            "root_xyz_0": [float(value) for value in future_qpos[0, :3]],
            "root_xyz_1": [float(value) for value in future_qpos[1, :3]],
            "root_xyz_delta_01": [float(value) for value in root_delta_01],
            "root_xy_delta_01_m": float(np.linalg.norm(root_delta_01[:2])),
            "root_z_delta_01_m": float(root_delta_01[2]),
            "root_yaw_0_rad": yaw_0,
            "root_yaw_1_rad": yaw_1,
            "root_yaw_delta_01_rad": yaw_delta,
            "pelvis_quat_wxyz_0": [float(value) for value in future_qpos[0, 3:7]],
            "pelvis_quat_wxyz_1": [float(value) for value in future_qpos[1, 3:7]],
            "reference_joint_minus_current": self._max_abs_entry(
                reference_joint_delta,
                G1_29_JOINT_NAMES,
            ),
            "foot_ik_observable": (
                "future_qpos_after_robo_pfnn_formal7168_pelvis_and_foot_ik"
            ),
        }

    def _write_trace_tick(
        self,
        *,
        world: WorldSample,
        lowstate: LowStateSnapshot,
        active: bool,
        reference: Any,
        observation: Any,
        action_isaac: np.ndarray,
        action_matrix: np.ndarray,
        desired_target: np.ndarray,
        target: np.ndarray,
        previous_action_before: np.ndarray,
        status: Mapping[str, Any],
    ) -> None:
        trace_file = getattr(self, "trace_file", None)
        trace_ticks = int(getattr(self, "trace_ticks", 0))
        trace_written = int(getattr(self, "trace_written", 0))
        if (
            trace_file is None
            or trace_ticks <= 0
            or trace_written >= trace_ticks
        ):
            return
        matrix_names = list(G1_29_JOINT_NAMES)
        isaac_names = [
            matrix_names[int(matrix_index)]
            for matrix_index in self.isaac_to_matrix.tolist()
        ]
        previous_action_matrix = previous_action_before[self.isaac_to_matrix]
        action_delta_isaac = action_isaac - previous_action_before
        action_delta_matrix = action_matrix - previous_action_matrix
        desired_target_delta = desired_target - lowstate.joint_pos_rad
        published_target_delta = target - lowstate.joint_pos_rad
        record = {
            "schema": "matrix.bfm_teacher.policy_tick_trace.v1",
            "policy_id": POLICY_ID,
            "tick_index": trace_written,
            "active_writer": bool(active),
            "world_sequence": int(world.sequence),
            "world_sim_time_s": (
                float(world.sim_time_s)
                if getattr(world, "sim_time_s", None) is not None
                else None
            ),
            "world_reset_count": int(world.reset_count),
            "contract_dims": CONTRACT_DIMS,
            "source_hashes": CONTRACT_SOURCE_HASHES,
            "lowstate": {
                "base_quat_wxyz": self._array_summary(lowstate.quaternion_wxyz),
                "base_ang_vel": self._array_summary(lowstate.body_gyro_rad_s),
                "joint_pos": self._array_summary(lowstate.joint_pos_rad),
                "joint_vel": self._array_summary(lowstate.joint_vel_rad_s),
                "base_quat_wxyz_values": lowstate.quaternion_wxyz.tolist(),
                "base_ang_vel_values": lowstate.body_gyro_rad_s.tolist(),
                "joint_pos_values": lowstate.joint_pos_rad.tolist(),
                "joint_vel_values": lowstate.joint_vel_rad_s.tolist(),
            },
            "observation": {
                "base_quat_wxyz": self._array_summary(
                    observation.base_quat_wxyz
                ),
                "base_ang_vel": self._array_summary(observation.base_ang_vel),
                "joint_pos": self._array_summary(observation.joint_pos),
                "joint_vel": self._array_summary(observation.joint_vel),
                "previous_action": self._array_summary(previous_action_before),
            },
            "reference": {
                "plan": self._reference_plan_summary(reference.plan),
                "replanned": bool(reference.replanned),
                "reason": reference.replan_reason,
                "plan_index": int(reference.plan_index),
                "pending_rebuild": bool(reference.pending_rebuild),
                "buffer_swapped": bool(
                    getattr(reference, "buffer_swapped", False)
                ),
                "pending_build_ms": getattr(reference, "pending_build_ms", None),
                "pending_clone_ms": getattr(reference, "pending_clone_ms", None),
                "pending_advance_ms": getattr(
                    reference, "pending_advance_ms", None
                ),
                "pending_publish_ms": getattr(
                    reference, "pending_publish_ms", None
                ),
                "pending_block_ms": getattr(reference, "pending_block_ms", None),
                "pending_elapsed_steps": getattr(
                    reference, "pending_elapsed_steps", None
                ),
                "reference_ready": not bool(reference.pending_rebuild),
                "continuity": self._reference_continuity_summary(
                    reference.plan,
                    lowstate,
                ),
            },
            "height_map_z": self._array_summary(world.height_map_z),
            "action_isaac": self._array_summary(action_isaac),
            "action_matrix": self._array_summary(action_matrix),
            "desired_target": self._array_summary(desired_target),
            "published_target": self._array_summary(target),
            "action_isaac_values": action_isaac.tolist(),
            "action_matrix_values": action_matrix.tolist(),
            "desired_target_values": desired_target.tolist(),
            "published_target_values": target.tolist(),
            "reference_joint_values": np.asarray(
                reference.plan.future_qpos[0, 7:], dtype=np.float32
            ).tolist(),
            "joint_ledger": self._joint_order_ledger(),
            "joint_argmax": {
                "published_target_minus_current": self._max_abs_entry(
                    published_target_delta,
                    matrix_names,
                ),
                "desired_target_minus_current": self._max_abs_entry(
                    desired_target_delta,
                    matrix_names,
                ),
                "raw_action_delta_isaac": self._max_abs_entry(
                    action_delta_isaac,
                    isaac_names,
                ),
                "action_delta_matrix": self._max_abs_entry(
                    action_delta_matrix,
                    matrix_names,
                ),
            },
            "continuity": {
                "desired_target_delta_rms_rad": status[
                    "desired_target_delta_rms_rad"
                ],
                "desired_target_delta_max_rad": status[
                    "desired_target_delta_max_rad"
                ],
                "published_target_delta_rms_rad": status[
                    "published_target_delta_rms_rad"
                ],
                "published_target_delta_max_rad": status[
                    "published_target_delta_max_rad"
                ],
                "published_target_step_delta_rms_rad": status[
                    "published_target_step_delta_rms_rad"
                ],
                "published_target_step_delta_max_rad": status[
                    "published_target_step_delta_max_rad"
                ],
                "reference_joint_error_rms_rad": status[
                    "reference_joint_error_rms_rad"
                ],
            },
            "status": dict(status),
        }
        with trace_file.open("a", encoding="utf-8") as stream:
            json.dump(record, stream, sort_keys=True, allow_nan=False)
            stream.write("\n")
        self.trace_written = trace_written + 1

    def step(
        self,
        world: WorldSample,
        lowstate: LowStateSnapshot,
        *,
        active: bool = True,
        handoff_preview: bool = False,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        if handoff_preview and not active:
            raise ValueError("BFM handoff preview requires active policy state")
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
        raw_command_motion_active = bool(
            command.gait != "stand"
            or abs(float(command.vx)) > 1.0e-6
            or abs(float(command.vy)) > 1.0e-6
            or abs(float(command.yaw_rate)) > 1.0e-6
        )
        stop_handoff_grace_active = False
        if raw_command_motion_active:
            self.last_motion_command = command
            self.stop_handoff_grace_steps_remaining = STOP_HANDOFF_GRACE_STEPS
        elif (
            active
            and self.reference_motion_active
            and self.last_motion_command is not None
            and self.stop_handoff_grace_steps_remaining > 0
        ):
            # Keep the last qualified moving branch alive just long enough for
            # the resident-writer transaction to fence BFM and resume SONIC.
            # Entering PFNN stand immediately on key-up can destabilize the
            # moving MuJoCo plant before the writer acknowledgement arrives.
            command = self.last_motion_command
            self.stop_handoff_grace_steps_remaining -= 1
            stop_handoff_grace_active = True
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
        canonical_reference_continuity = bool(
            getattr(self, "canonical_reference_continuity", True)
        )
        if start_reference_reset and canonical_reference_continuity:
            # A completed/ongoing stand-branch settle must not throttle the
            # first requested motion branch.  BFM-3DGS publishes the new walk
            # actor target directly; keeping an old stand activation origin
            # here made the CPU-side PFNN build pause hold the wrong target.
            self.activation_origin = None
            self.activation_step = 0
            self.reference_stop_blend_pending = False
        elif stop_reference_reset and canonical_reference_continuity:
            self.reference_stop_blend_pending = True
        if start_reference_reset and not canonical_reference_continuity:
            # Let RoboPfnnReferenceStream branch from its continuously tracked
            # stand cursor in the background.  A synchronous reset performs a
            # full PFNN warmup and future-buffer fill on the 50 Hz policy
            # thread; the independent 500 Hz DDS publisher then repeats the
            # old stand LowCmd for seconds and can apply a stale walk command
            # after the operator has already released the key.
            self.reference_transition = "starting"
            self.reference_hold_target = self._command_continuity_anchor(lowstate)
            self.idle_anchor_target = None
            self.previous_action.fill(0.0)
            self.activation_origin = None
            self.activation_step = 0
            self.reference_start_resets += 1
        elif stop_reference_reset and not canonical_reference_continuity:
            # A background walk -> stand branch must not leave the old walking
            # target in control.  Hold the exact observed pose until the stand
            # buffer is ready, then restart Teacher history and blend from the
            # last applied command target.
            self.reference_transition = "stopping"
            self.reference_hold_target = self._command_continuity_anchor(lowstate)
            self.idle_anchor_target = None
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
        realtime_rolling_reason = (
            self._prepare_realtime_rolling_command(command)
            if active and canonical_reference_continuity
            else None
        )
        realtime_rolling_stop = bool(
            active
            and canonical_reference_continuity
            and stop_reference_reset
            and realtime_rolling_reason is not None
        )
        if realtime_rolling_stop:
            # No atomic branch swap follows this rolling transition, so there
            # is no swap target that needs the pending-hold/blend path.
            self.reference_stop_blend_pending = False
        reference = self.stream.sample(
            command,
            world.root_position,
            world.root_yaw,
            height_field,
        )
        if not active:
            qpos_50hz = np.asarray(reference.plan.qpos_50hz, dtype=np.float32)
            joint_vel_50hz = np.asarray(
                reference.plan.joint_vel_50hz,
                dtype=np.float32,
            )
            if qpos_50hz.ndim != 2 or qpos_50hz.shape[1] != 36:
                raise RuntimeError(
                    "direct BFM reference qpos must have shape (frames, 36)"
                )
            if qpos_50hz.shape[0] < 2 or joint_vel_50hz.shape[1:] != (29,):
                raise RuntimeError("direct BFM reference has no 50 Hz velocity frame")
            root_velocity = self.reference_module.qpos_root_velocity(
                qpos_50hz[0],
                qpos_50hz[1],
                POLICY_HZ,
            )
            qvel = np.concatenate(
                (root_velocity, joint_vel_50hz[0]),
            ).astype(np.float32)
            if qvel.shape != (35,) or not np.all(np.isfinite(qvel)):
                raise RuntimeError("direct BFM reference qvel must be finite 35D")
            self.direct_reference_start = {
                "qpos": [float(value) for value in qpos_50hz[0]],
                "qvel": [float(value) for value in qvel],
            }
        reference_buffer_swapped = bool(
            getattr(reference, "buffer_swapped", False)
        )
        reference_pending_rebuild = bool(reference.pending_rebuild)
        if (
            active
            and not canonical_reference_continuity
            and self.reference_transition is None
            and reference_pending_rebuild
            and not reference_buffer_swapped
        ):
            # Robo-PFNN can rebuild its future buffer while motion is already
            # active, for example after a root-anchor correction.  Until the
            # replacement buffer is swapped in, publish the observed pose and
            # keep actor history clean; otherwise Matrix can apply a one-frame
            # target jump from a half-rebuilt reference.
            self.reference_transition = "rebuilding"
            self.reference_hold_target = self._command_continuity_anchor(lowstate)
            self.previous_action.fill(0.0)
            self.activation_origin = None
            self.activation_step = 0
        transition_completed = bool(
            not canonical_reference_continuity
            and
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
                self.activation_origin = self._command_continuity_anchor(lowstate)
                self.activation_step = 0
            else:
                self.activation_origin = None
                self.activation_step = 0
            self.reference_transition = None
            self.reference_hold_target = None
        reference_swap_blend_started = False
        reference_root_anchor_swap = bool(
            reference_buffer_swapped
            and "root_anchor" in str(reference.replan_reason or "")
        )
        reference_stop_swap = bool(
            reference_buffer_swapped
            and not command_motion_active
            and getattr(self, "reference_stop_blend_pending", False)
        )
        if (
            active
            and canonical_reference_continuity
            and (reference_root_anchor_swap or reference_stop_swap)
            and getattr(self, "last_published_target", None) is not None
            and not (
                getattr(self, "idle_anchor_enabled", True)
                and not command_motion_active
            )
        ):
            # Keep the BFM-3DGS active/pending PFNN stream and Teacher history
            # continuous, but do not expose a branch-swap discontinuity as one
            # MuJoCo LowCmd step.  PhysX tolerates the canonical atomic swap;
            # Matrix observed 0.36-0.90 rad target jumps specifically when a
            # terrain/root-anchor branch swapped, followed immediately by a
            # fall.  Blend root-anchor swaps and real walk -> stand swaps from
            # the last target actually published.  The Teacher still advances
            # every tick and sees the applied blended action as PrevActions;
            # no PFNN or actor reset is introduced and terrain keeps updating.
            self.activation_origin = self.last_published_target.astype(
                np.float32,
                copy=True,
            )
            self.activation_step = 0
            self.activation_target_step_limit_rad = (
                REFERENCE_SWAP_MAX_TARGET_DELTA_RAD
            )
            if reference_stop_swap:
                self.reference_stop_blend_pending = False
            reference_swap_blend_started = True
        holding_reference_transition = bool(
            self.reference_transition in {"starting", "stopping", "rebuilding"}
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
        previous_action_before = self.previous_action.copy()
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
        prior_published_target = (
            getattr(self, "last_published_target", None).astype(
                np.float32,
                copy=True,
            )
            if getattr(self, "last_published_target", None) is not None
            else None
        )
        blend_fraction = 1.0
        target = desired_target
        canonical_stop_pending_hold = bool(
            active
            and canonical_reference_continuity
            and getattr(self, "reference_stop_blend_pending", False)
            and not command_motion_active
            and reference_pending_rebuild
            and not reference_buffer_swapped
            and prior_published_target is not None
        )
        idle_anchor_hold = bool(
            active
            and getattr(self, "idle_anchor_enabled", True)
            and not command_motion_active
            and not holding_reference_transition
        )
        activation_target_limited = False
        if idle_anchor_hold:
            if self.idle_anchor_target is None:
                self.idle_anchor_target = lowstate.joint_pos_rad.astype(
                    np.float32,
                    copy=True,
                )
            target = self.idle_anchor_target.copy()
            blend_fraction = 0.0
            # Keep the exact observed/applied pose for an ordinary hot switch
            # from an unrelated controller.  The initial PFNN-aligned path
            # disables this compatibility hold and retains the canonical
            # Teacher stand closed loop used by the validated runner.
            self.activation_step = 0
        elif active and holding_reference_transition:
            target = self.reference_hold_target.copy()
            blend_fraction = 0.0
        elif canonical_stop_pending_hold:
            # Continue publishing the last admitted walking target while the
            # stand branch is built.  Letting the actor consume the still-live
            # walking branch during these few pending frames produced target
            # jumps above 0.5 rad before the swap blend could even start.
            # Teacher/PFNN state still advances, and PrevActions below records
            # this exact held target, so the eventual swap remains continuous.
            target = prior_published_target.copy()
            blend_fraction = 0.0
            self.activation_step = 0
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
            continuity_anchor = (
                prior_published_target
                if prior_published_target is not None
                else self.activation_origin
            )
            target_step = target - continuity_anchor
            target_step_limit_rad = float(
                getattr(
                    self,
                    "activation_target_step_limit_rad",
                    HOT_SWITCH_MAX_TARGET_DELTA_RAD,
                )
            )
            if float(np.max(np.abs(target_step))) > target_step_limit_rad:
                target = (
                    continuity_anchor
                    + np.clip(
                        target_step,
                        -target_step_limit_rad,
                        target_step_limit_rad,
                    )
                ).astype(np.float32)
                activation_target_limited = True
            if progress >= 1.0 and not activation_target_limited:
                self.activation_origin = None
                self.activation_target_step_limit_rad = (
                    HOT_SWITCH_MAX_TARGET_DELTA_RAD
                )
        if active:
            if idle_anchor_hold or holding_reference_transition:
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
        if active:
            self.last_published_target = target.astype(np.float32, copy=True)
        self.last_world_sequence = world.sequence
        desired_delta = desired_target - lowstate.joint_pos_rad
        published_delta = target - lowstate.joint_pos_rad
        published_step_delta = (
            target - prior_published_target
            if prior_published_target is not None
            else np.zeros(NUM_JOINTS, dtype=np.float32)
        )
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
        status = {
            "reference_replanned": bool(reference.replanned),
            "reference_reason": reference.replan_reason,
            "reference_plan_index": int(reference.plan_index),
            "reference_root_error_m": float(reference.root_error_before_m),
            "reference_pending_rebuild": reference_pending_rebuild,
            "reference_buffer_swapped": reference_buffer_swapped,
            "reference_realtime_rolling_reason": realtime_rolling_reason,
            "reference_realtime_rolling_stop": realtime_rolling_stop,
            "reference_pending_build_ms": getattr(
                reference, "pending_build_ms", None
            ),
            "reference_pending_clone_ms": getattr(
                reference, "pending_clone_ms", None
            ),
            "reference_pending_advance_ms": getattr(
                reference, "pending_advance_ms", None
            ),
            "reference_pending_publish_ms": getattr(
                reference, "pending_publish_ms", None
            ),
            "reference_pending_block_ms": getattr(
                reference, "pending_block_ms", None
            ),
            "reference_pending_elapsed_steps": getattr(
                reference, "pending_elapsed_steps", None
            ),
            "reference_swap_blend_started": reference_swap_blend_started,
            "reference_stop_blend_pending": bool(
                getattr(self, "reference_stop_blend_pending", False)
            ),
            "reference_stop_pending_hold": canonical_stop_pending_hold,
            "reference_transition": self.reference_transition,
            "idle_anchor_hold": idle_anchor_hold,
            "idle_anchor_enabled": bool(
                getattr(self, "idle_anchor_enabled", True)
            ),
            "idle_anchor_initialized": self.idle_anchor_target is not None,
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
            "stop_handoff_grace_active": stop_handoff_grace_active,
            "stop_handoff_grace_steps_remaining": int(
                self.stop_handoff_grace_steps_remaining
            ),
            "reference_target_speed_mps": float(reference.plan.target_speed),
            "reference_future_xy_delta_m": reference_future_xy_delta_m,
            "shadow_preview": not active or handoff_preview,
            "handoff_preview": bool(handoff_preview),
            "activation_blend_fraction": float(blend_fraction),
            "activation_blend_steps": int(self.activation_blend_steps),
            "activation_settle_active": self.activation_origin is not None,
            "activation_target_limited": activation_target_limited,
            "activation_target_delta_limit_rad": float(
                getattr(
                    self,
                    "activation_target_step_limit_rad",
                    HOT_SWITCH_MAX_TARGET_DELTA_RAD,
                )
            ),
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
            "published_target_step_delta_rms_rad": float(
                math.sqrt(np.mean(np.square(published_step_delta)))
            ),
            "published_target_step_delta_max_rad": float(
                np.max(np.abs(published_step_delta))
            ),
            "reference_joint_error_rms_rad": float(
                math.sqrt(np.mean(np.square(reference_delta)))
            ),
        }
        self._write_trace_tick(
            world=world,
            lowstate=lowstate,
            active=active,
            reference=reference,
            observation=observation,
            action_isaac=action_isaac,
            action_matrix=action_matrix,
            desired_target=desired_target,
            target=target,
            previous_action_before=previous_action_before,
            status=status,
        )
        return target, status


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


def _handoff_is_writer_fenced(handoff: HandoffStateMachine) -> bool:
    """Return true only for a standby that cannot currently publish."""

    if handoff.state == HandoffStateMachine.WAITING:
        return handoff.publisher is None
    if handoff.state == HandoffStateMachine.PAUSED:
        return handoff.publisher is not None
    return False


def run_worker(
    *,
    core: BfmTeacherCore,
    dds: UnitreeDdsRuntime,
    state_store: LatestLowState,
    command_store: LatestLowCmdTarget,
    control_socket: Path,
    execution_provider: str,
    direct_start: bool = False,
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
    preparing_authority_epoch: int | None = None
    prepared_authority_epoch: int | None = None
    prepare_ready_steps = 0
    prepare_reference_buffer_swapped = False
    preparing_aligned_initial = False

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
            "direct_start": bool(direct_start),
            "reference_source": core.reference_source,
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
    # Initial-policy alignment consumes the freshest online-PFNN state from
    # STATUS after the prior writer is fenced.  Keep this control-plane sample
    # close to the 50 Hz reference without flooding the socket at policy rate.
    status_period = 0.1
    transient_stale_active = False
    preparing_allow_idle_neutral = False
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
                    if command == "PREPARE":
                        requested_epoch = int(payload.get("authority_epoch"))
                        aligned_initial = payload.get("aligned_initial", False)
                        allow_idle_neutral = payload.get(
                            "allow_idle_neutral", False
                        )
                        if type(aligned_initial) is not bool:
                            raise RuntimeError(
                                "BFM Teacher PREPARE has invalid alignment mode"
                            )
                        if type(allow_idle_neutral) is not bool:
                            raise RuntimeError(
                                "BFM Teacher PREPARE has invalid idle-neutral mode"
                            )
                        if aligned_initial and allow_idle_neutral:
                            raise RuntimeError(
                                "BFM Teacher PREPARE mixes initial and idle-neutral modes"
                            )
                        if direct_start:
                            raise RuntimeError(
                                "direct BFM does not use hot-switch preparation"
                            )
                        if requested_epoch != authority_epoch + 1:
                            raise RuntimeError(
                                "BFM Teacher prepare epoch did not advance"
                            )
                        if preparing_authority_epoch == requested_epoch:
                            # A duplicated local datagram must not kill the
                            # resident policy.  The original preparation
                            # remains writer-fenced and will emit its result.
                            continue
                        if prepared_authority_epoch == requested_epoch:
                            continue
                        if not _handoff_is_writer_fenced(handoff):
                            raise RuntimeError(
                                "BFM Teacher PREPARE requires writer-fenced standby"
                            )
                        if (
                            preparing_authority_epoch is not None
                            or prepared_authority_epoch is not None
                        ):
                            raise RuntimeError(
                                "BFM Teacher PREPARE authority epoch is busy"
                            )
                        now = monotonic()
                        state = state_store.get()
                        prior_command = command_store.get()
                        if (
                            not warmed
                            or latest_world is None
                            or now - latest_world.received_monotonic
                            > WORLD_SAMPLE_MAX_AGE_S
                            or state is None
                            or now - state.received_monotonic
                            > LOWSTATE_MAX_AGE_S
                            or (
                                not aligned_initial
                                and (
                                    prior_command is None
                                    or now - prior_command.received_monotonic
                                    > HOT_SWITCH_LOW_CMD_MAX_AGE_S
                                )
                            )
                        ):
                            raise RuntimeError(
                                "BFM Teacher PREPARE lacks fresh warmed inputs"
                            )
                        if not _handoff_input_is_neutral(
                            latest_world,
                            allow_idle_neutral=allow_idle_neutral,
                        ):
                            raise RuntimeError(
                                "BFM Teacher PREPARE requires a safety-stop or "
                                "authorized idle-neutral handoff"
                            )
                        if aligned_initial:
                            # Matrix has already aligned the physical G1 to the
                            # first online-PFNN reference while SONIC is fenced.
                            # Keep that reference cursor and use the canonical
                            # BFM-3DGS first target directly.  The simulator is
                            # already at the exact PFNN state, so a live-policy
                            # hot-switch blend would alter the trained closed
                            # loop rather than improve safety.
                            core.prepare_aligned_initial_activation()
                        else:
                            assert prior_command is not None
                            core.prepare_handoff_activation(
                                state,
                                prior_command.joint_pos_rad,
                            )
                        preparing_authority_epoch = requested_epoch
                        prepared_authority_epoch = None
                        preparing_aligned_initial = aligned_initial
                        preparing_allow_idle_neutral = allow_idle_neutral
                        prepare_ready_steps = 0
                        prepare_reference_buffer_swapped = False
                        next_policy = monotonic()
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
                        if (
                            not direct_start
                            and not _handoff_input_is_neutral(
                                latest_world,
                                allow_idle_neutral=(
                                    preparing_allow_idle_neutral
                                ),
                            )
                        ):
                            raise RuntimeError(
                                "BFM Teacher GO requires a safety-stop or "
                                "authorized idle-neutral handoff"
                            )
                        if direct_start:
                            core.prepare_direct_activation()
                        else:
                            if prepared_authority_epoch != requested_epoch:
                                raise RuntimeError(
                                    "BFM Teacher GO arrived before safe preparation"
                                )
                            if (
                                not preparing_aligned_initial
                                and (
                                latest_policy_status.get(
                                    "published_target_step_delta_max_rad"
                                )
                                is None
                                or float(
                                    latest_policy_status[
                                        "published_target_step_delta_max_rad"
                                    ]
                                )
                                > HOT_SWITCH_MAX_TARGET_DELTA_RAD + 1.0e-6
                                )
                            ):
                                raise RuntimeError(
                                    "BFM Teacher prepared target exceeded safety gate"
                                )
                        if direct_start:
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
                        preparing_authority_epoch = None
                        prepared_authority_epoch = None
                        preparing_aligned_initial = False
                        preparing_allow_idle_neutral = False
                        prepare_ready_steps = 0
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
                        preparing_authority_epoch = None
                        prepared_authority_epoch = None
                        preparing_aligned_initial = False
                        prepare_ready_steps = 0
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
                    if (
                        direct_start
                        and warmed
                        and handoff.state == HandoffStateMachine.WAITING
                    ):
                        next_policy = _advance_deadline(
                            next_policy,
                            policy_period,
                            now,
                        )
                        continue
                    started = monotonic()
                    try:
                        target, latest_policy_status = core.step(
                            latest_world,
                            state,
                            active=(
                                handoff.state == HandoffStateMachine.ACTIVE
                                or preparing_authority_epoch is not None
                                or prepared_authority_epoch is not None
                            ),
                            handoff_preview=(
                                handoff.state
                                in {
                                    HandoffStateMachine.WAITING,
                                    HandoffStateMachine.PAUSED,
                                }
                                and (
                                    preparing_authority_epoch is not None
                                    or prepared_authority_epoch is not None
                                )
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
                        if (
                            handoff.state
                            in {
                                HandoffStateMachine.WAITING,
                                HandoffStateMachine.PAUSED,
                            }
                            and (
                                preparing_authority_epoch is not None
                                or prepared_authority_epoch is not None
                            )
                        ):
                            prepare_reference_buffer_swapped |= bool(
                                latest_policy_status.get(
                                    "reference_buffer_swapped", False
                                )
                            )
                            reference_ready = bool(
                                not latest_policy_status.get(
                                    "reference_pending_rebuild", True
                                )
                                and not latest_policy_status.get(
                                    "reference_transition_holding", True
                                )
                                and float(
                                    latest_policy_status.get(
                                        "reference_root_error_m", math.inf
                                    )
                                )
                                <= HOT_SWITCH_MAX_REFERENCE_ROOT_ERROR_M
                            )
                            target_step_delta = float(
                                latest_policy_status.get(
                                    "published_target_step_delta_max_rad",
                                    math.inf,
                                )
                            )
                            if (
                                not preparing_aligned_initial
                                and (
                                target_step_delta
                                > HOT_SWITCH_MAX_TARGET_DELTA_RAD + 1.0e-6
                                )
                            ):
                                rejected_epoch = (
                                    preparing_authority_epoch
                                    if preparing_authority_epoch is not None
                                    else prepared_authority_epoch
                                )
                                send_event(
                                    "ACTIVATION_REJECTED",
                                    {
                                        "authority_epoch": rejected_epoch,
                                        "writer_created": (
                                            handoff.publisher is not None
                                        ),
                                        "writer_reused": (
                                            handoff.publisher is not None
                                        ),
                                        "write_authorized": False,
                                        "reason": "published_target_delta_exceeded",
                                        "target_delta_max_rad": target_step_delta,
                                        "target_step_delta_max_rad": (
                                            target_step_delta
                                        ),
                                        "target_delta_limit_rad": (
                                            HOT_SWITCH_MAX_TARGET_DELTA_RAD
                                        ),
                                        "reference_aligned": reference_ready,
                                    },
                                )
                                preparing_authority_epoch = None
                                prepared_authority_epoch = None
                                preparing_aligned_initial = False
                                preparing_allow_idle_neutral = False
                                prepare_ready_steps = 0
                                core.enter_standby()
                            elif preparing_authority_epoch is not None:
                                prepare_ready_steps = (
                                    prepare_ready_steps + 1
                                    if reference_ready
                                    else 0
                                )
                                required_preview_steps = (
                                    1
                                    if preparing_aligned_initial
                                    else HOT_SWITCH_PREVIEW_STEPS
                                )
                                if prepare_ready_steps >= required_preview_steps:
                                    prepared_authority_epoch = (
                                        preparing_authority_epoch
                                    )
                                    preparing_authority_epoch = None
                                    send_event(
                                        "ACTIVATION_PREPARED",
                                        {
                                            "authority_epoch": (
                                                prepared_authority_epoch
                                            ),
                                            "writer_created": (
                                                handoff.publisher is not None
                                            ),
                                            "writer_reused": (
                                                handoff.publisher is not None
                                            ),
                                            "write_authorized": False,
                                            "exact_initial_alignment": bool(
                                                preparing_aligned_initial
                                            ),
                                            "idle_neutral_handoff": bool(
                                                preparing_allow_idle_neutral
                                            ),
                                            "reference_aligned": True,
                                            "reference_pending_rebuild": False,
                                            "reference_buffer_swapped": (
                                                prepare_reference_buffer_swapped
                                            ),
                                            "preview_steps": prepare_ready_steps,
                                            "target_delta_max_rad": (
                                                target_step_delta
                                            ),
                                            "target_step_delta_max_rad": (
                                                target_step_delta
                                            ),
                                            "desired_target_delta_max_rad": float(
                                                latest_policy_status.get(
                                                    "desired_target_delta_max_rad",
                                                    math.inf,
                                                )
                                            ),
                                            "target_delta_limit_rad": (
                                                HOT_SWITCH_MAX_TARGET_DELTA_RAD
                                            ),
                                        },
                                    )
                        if not warmed:
                            direct_reference = core.direct_reference_start
                            if direct_reference is None:
                                raise RuntimeError(
                                    "BFM warmup produced no initial reference state"
                                )
                            warmed = True
                            send_event(
                                "WARMED_NO_WRITER",
                                {
                                    "writer_created": False,
                                    "models_loaded_once": True,
                                    "models_warmed": True,
                                    "direct_start": bool(direct_start),
                                    "reference_source": core.reference_source,
                                    "direct_initial_qpos": (
                                        direct_reference["qpos"]
                                        if direct_reference is not None
                                        else None
                                    ),
                                    "direct_initial_qvel": (
                                        direct_reference["qvel"]
                                        if direct_reference is not None
                                        else None
                                    ),
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
                observed_command = command_store.get()
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
                    "activation_preparing": (
                        preparing_authority_epoch is not None
                    ),
                    "activation_prepared": (
                        prepared_authority_epoch is not None
                    ),
                    "prepared_authority_epoch": prepared_authority_epoch,
                    "activation_preview_ready_steps": prepare_ready_steps,
                    "activation_preview_steps_required": (
                        1
                        if preparing_aligned_initial
                        else HOT_SWITCH_PREVIEW_STEPS
                    ),
                    "activation_exact_initial_alignment": bool(
                        preparing_aligned_initial
                    ),
                    "activation_target_delta_limit_rad": (
                        HOT_SWITCH_MAX_TARGET_DELTA_RAD
                    ),
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
                    "policy_trace_file": (
                        str(core.trace_file) if core.trace_file is not None else None
                    ),
                    "policy_trace_ticks_requested": int(core.trace_ticks),
                    "policy_trace_ticks_written": int(core.trace_written),
                    "direct_initial_qpos": (
                        core.direct_reference_start["qpos"]
                        if core.direct_reference_start is not None
                        else None
                    ),
                    "direct_initial_qvel": (
                        core.direct_reference_start["qvel"]
                        if core.direct_reference_start is not None
                        else None
                    ),
                    "observed_lowcmd_target": observed_command is not None,
                    "observed_lowcmd_target_age_ms": (
                        max(
                            0.0,
                            monotonic() - observed_command.received_monotonic,
                        )
                        * 1000.0
                        if observed_command is not None
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
    if args.reference_clip is not None:
        if not args.reference_clip.is_absolute():
            raise ValueError("direct BFM reference clip must be absolute")
        if not args.reference_clip_sha256:
            raise ValueError("direct BFM reference clip SHA256 is required")
        require_file_sha256(
            args.reference_clip,
            args.reference_clip_sha256,
            "formal7168 reference clip",
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
    parser.add_argument("--reference-clip", type=Path)
    parser.add_argument("--reference-clip-sha256")
    parser.add_argument("--direct-start", action="store_true")
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
            "smooth command-continuous takeover duration; actor history records "
            "the actually published blended targets"
        ),
    )
    parser.add_argument("--trace-file", type=Path)
    parser.add_argument("--trace-ticks", type=int, default=0)
    parser.add_argument(
        "--validate-only",
        action="store_true",
        help="Validate hashes, load both models, then exit without DDS",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.reference_clip is not None and not args.direct_start:
        raise SystemExit("--reference-clip requires --direct-start")
    if args.reference_clip_sha256 is not None and args.reference_clip is None:
        raise SystemExit("--reference-clip-sha256 requires --reference-clip")
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
        reference_clip=args.reference_clip,
        direct_start=args.direct_start,
        trace_file=args.trace_file,
        trace_ticks=args.trace_ticks,
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
    command_store = LatestLowCmdTarget()
    dds = BfmUnitreeDdsRuntime(
        interface=args.interface,
        state_store=state_store,
        command_store=command_store,
    )
    try:
        return run_worker(
            core=core,
            dds=dds,
            state_store=state_store,
            command_store=command_store,
            control_socket=args.control_socket,
            execution_provider=args.execution_provider,
            direct_start=args.direct_start,
        )
    finally:
        core.close()


if __name__ == "__main__":
    raise SystemExit(main())
