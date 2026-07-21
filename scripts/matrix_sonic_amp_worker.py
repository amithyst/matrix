#!/usr/bin/env python3
"""Temporarily own G1 LowCmd and run the 29-DoF AMP get-up policy.

This process is deliberately a *physical controller*, not a simulator state
editor.  It has no MuJoCo dependency and exposes no qpos, reset, reload, or
teleport operation.  Its only output is Unitree ``rt/lowcmd``.

The control-plane hand-off is intentionally strict:

* startup loads the ONNX model and subscribes to ``rt/lowstate``;
* it then connects to an AF_UNIX/SOCK_SEQPACKET endpoint and reports
  ``READY_NO_WRITER`` while no LowCmd publisher exists;
* only a ``GO`` packet may create the publisher;
* ``FIRST_WRITE`` is reported after the first successful DDS write; and
* ``PAUSE`` closes the writer while keeping every policy loaded;
* a later ``GO`` recreates the writer without reloading a model; and
* ``STOP`` performs final process shutdown.

Packets are UTF-8 JSON objects using ``matrix.sonic_amp_worker.control.v1``.
For a small amount of operator friendliness, the receiver also accepts the
literal packets ``GO`` and ``STOP``.
"""

from __future__ import annotations

import argparse
import importlib
import json
import math
import os
import select
import socket
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Mapping, Protocol, Sequence

import numpy as np

from matrix_policy_runtime import create_inference_session


NUM_JOINTS = 29
LOWCMD_SLOTS = 35
OBSERVATION_FRAME_WIDTH = 96
HISTORY_LENGTH = 4
OBSERVATION_WIDTH = OBSERVATION_FRAME_WIDTH * HISTORY_LENGTH
POLICY_HZ = 50.0
CONTROL_SCHEMA = "matrix.sonic_amp_worker.control.v1"

G1_29_JOINT_NAMES = (
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


def _finite_vector(value: Any, size: int, label: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float32)
    if result.shape != (size,):
        raise ValueError(f"{label} must contain {size} values, got {result.shape}")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{label} contains a non-finite value")
    return result.copy()


def _normalized_quaternion_wxyz(value: Any) -> np.ndarray:
    result = _finite_vector(value, 4, "quaternion_wxyz")
    norm = float(np.linalg.norm(result))
    if norm <= 1e-8:
        raise ValueError("quaternion_wxyz has zero norm")
    return result / norm


def projected_gravity_body(quaternion_wxyz: Any) -> np.ndarray:
    """Return world gravity direction (0, 0, -1) in the IMU body frame."""

    w, x, y, z = _normalized_quaternion_wxyz(quaternion_wxyz)
    rotation_transpose = np.asarray(
        (
            (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y + w * z), 2.0 * (x * z - w * y)),
            (2.0 * (x * y - w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z + w * x)),
            (2.0 * (x * z + w * y), 2.0 * (y * z - w * x), 1.0 - 2.0 * (x * x + y * y)),
        ),
        dtype=np.float32,
    )
    return rotation_transpose @ np.asarray((0.0, 0.0, -1.0), dtype=np.float32)


def root_up_z_from_imu(quaternion_wxyz: Any) -> float:
    """World-Z component of body up, derived solely from LowState IMU."""

    _w, x, y, _z = _normalized_quaternion_wxyz(quaternion_wxyz)
    return float(1.0 - 2.0 * (x * x + y * y))


@dataclass(frozen=True)
class PolicyConfig:
    default_joint_pos: np.ndarray
    action_scale: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    action_clip: float

    @classmethod
    def from_mapping(cls, config: Mapping[str, Any]) -> "PolicyConfig":
        joint_names = tuple(config.get("policy_joint_names", ()))
        if joint_names != G1_29_JOINT_NAMES:
            raise ValueError(
                "policy_joint_names must exactly match the Unitree G1 29-DoF DDS order"
            )

        observation_config = config.get("obs_config", {})
        if int(observation_config.get("history_length", 0)) != HISTORY_LENGTH:
            raise ValueError("AMP worker requires history_length=4")
        expected_observations = (
            "RootAngVelB",
            "ProjectedGravityB",
            "Command",
            "JointPos",
            "JointVel",
            "PrevActions",
        )
        actual_observations = tuple(
            item.get("name") for item in observation_config.get("policy", ())
        )
        if actual_observations != expected_observations:
            raise ValueError(
                f"unsupported AMP observation order: {actual_observations!r}"
            )
        if not bool(config.get("obs_joint_pos_relative", False)):
            raise ValueError("AMP worker requires relative joint-position observations")

        control_dt = float(config.get("sim", {}).get("control_dt", 0.0))
        if not math.isclose(control_dt, 1.0 / POLICY_HZ, abs_tol=1e-9):
            raise ValueError("AMP worker requires a 50 Hz (0.02 s) policy")
        action_clip = float(config.get("action_clip", 0.0))
        if not math.isfinite(action_clip) or action_clip <= 0.0:
            raise ValueError("action_clip must be finite and positive")
        return cls(
            default_joint_pos=_finite_vector(
                config.get("default_joint_pos"), NUM_JOINTS, "default_joint_pos"
            ),
            action_scale=_finite_vector(
                config.get("action_scale"), NUM_JOINTS, "action_scale"
            ),
            kp=_finite_vector(config.get("stiffness"), NUM_JOINTS, "stiffness"),
            kd=_finite_vector(config.get("damping"), NUM_JOINTS, "damping"),
            action_clip=action_clip,
        )


@dataclass(frozen=True)
class LowStateSnapshot:
    quaternion_wxyz: np.ndarray
    body_gyro_rad_s: np.ndarray
    joint_pos_rad: np.ndarray
    joint_vel_rad_s: np.ndarray
    received_monotonic: float
    mode_pr: int = 0
    mode_machine: int = 0

    @classmethod
    def validated(
        cls,
        *,
        quaternion_wxyz: Any,
        body_gyro_rad_s: Any,
        joint_pos_rad: Any,
        joint_vel_rad_s: Any,
        received_monotonic: float,
        mode_pr: int = 0,
        mode_machine: int = 0,
    ) -> "LowStateSnapshot":
        received = float(received_monotonic)
        if not math.isfinite(received):
            raise ValueError("received_monotonic must be finite")
        return cls(
            quaternion_wxyz=_normalized_quaternion_wxyz(quaternion_wxyz),
            body_gyro_rad_s=_finite_vector(
                body_gyro_rad_s, 3, "body_gyro_rad_s"
            ),
            joint_pos_rad=_finite_vector(joint_pos_rad, NUM_JOINTS, "joint_pos_rad"),
            joint_vel_rad_s=_finite_vector(
                joint_vel_rad_s, NUM_JOINTS, "joint_vel_rad_s"
            ),
            received_monotonic=received,
            mode_pr=int(mode_pr),
            mode_machine=int(mode_machine),
        )


@dataclass(frozen=True)
class PolicyOutput:
    raw_action: np.ndarray
    clipped_action: np.ndarray
    target_joint_pos: np.ndarray
    observation: np.ndarray


class PolicyRunner(Protocol):
    def __call__(self, observation: np.ndarray) -> np.ndarray: ...


class OnnxPolicyRunner:
    """Lazy ONNX Runtime wrapper; importing this module needs no ONNX install."""

    def __init__(self, model_path: Path, *, execution_provider: str = "cpu"):
        ort = importlib.import_module("onnxruntime")
        self._session, self.execution_provider = create_inference_session(
            ort,
            str(model_path),
            execution_provider,
        )
        inputs = self._session.get_inputs()
        outputs = self._session.get_outputs()
        if len(inputs) != 1 or not outputs:
            raise ValueError("AMP ONNX must expose one input and at least one output")
        input_width = inputs[0].shape[-1]
        if isinstance(input_width, int) and input_width != OBSERVATION_WIDTH:
            raise ValueError(
                f"AMP ONNX input width is {input_width}, expected {OBSERVATION_WIDTH}"
            )
        output_width = outputs[0].shape[-1]
        if isinstance(output_width, int) and output_width != NUM_JOINTS:
            raise ValueError(
                f"AMP ONNX output width is {output_width}, expected {NUM_JOINTS}"
            )
        self._input_name = inputs[0].name
        self._output_name = outputs[0].name
        # Materialize kernels and device allocations before READY is emitted.
        self(np.zeros(OBSERVATION_WIDTH, dtype=np.float32))

    def __call__(self, observation: np.ndarray) -> np.ndarray:
        batch = np.asarray(observation, dtype=np.float32).reshape(1, OBSERVATION_WIDTH)
        return np.asarray(
            self._session.run([self._output_name], {self._input_name: batch})[0]
        ).reshape(-1)


class AmpPolicyCore:
    """Pure, unit-testable 4-frame AMP observation and action pipeline."""

    def __init__(self, config: PolicyConfig, runner: PolicyRunner):
        self.config = config
        self.runner = runner
        self.previous_raw_action = np.zeros(NUM_JOINTS, dtype=np.float32)
        self._history: deque[np.ndarray] = deque(maxlen=HISTORY_LENGTH)

    def reset_history(self, state: LowStateSnapshot) -> None:
        self.previous_raw_action.fill(0.0)
        frame = self.observation_frame(state)
        self._history.clear()
        self._history.extend(frame.copy() for _ in range(HISTORY_LENGTH))

    def observation_frame(self, state: LowStateSnapshot) -> np.ndarray:
        frame = np.concatenate(
            (
                state.body_gyro_rad_s,
                projected_gravity_body(state.quaternion_wxyz),
                np.zeros(3, dtype=np.float32),
                state.joint_pos_rad - self.config.default_joint_pos,
                state.joint_vel_rad_s,
                self.previous_raw_action,
            )
        ).astype(np.float32, copy=False)
        if frame.shape != (OBSERVATION_FRAME_WIDTH,):
            raise AssertionError(f"internal AMP observation shape error: {frame.shape}")
        return frame

    def build_observation(self, state: LowStateSnapshot) -> np.ndarray:
        if not self._history:
            self.reset_history(state)
        else:
            self._history.append(self.observation_frame(state))
        # deque iteration is explicitly oldest -> newest.
        observation = np.concatenate(tuple(self._history)).astype(
            np.float32, copy=False
        )
        if observation.shape != (OBSERVATION_WIDTH,):
            raise AssertionError(f"internal AMP history shape error: {observation.shape}")
        return observation

    def infer(self, state: LowStateSnapshot) -> PolicyOutput:
        observation = self.build_observation(state)
        raw_action = _finite_vector(
            self.runner(observation[None, :]), NUM_JOINTS, "ONNX raw action"
        )
        clipped_action = np.clip(
            raw_action, -self.config.action_clip, self.config.action_clip
        ).astype(np.float32, copy=False)
        target = self.config.default_joint_pos + self.config.action_scale * clipped_action
        # PrevActions is the previous action-space value after the configured
        # actor clip, never the scaled q target.  This matches the upstream
        # PolicyRunner's ``lastActions`` buffer.  "Raw" here distinguishes
        # action space from joint-position targets, not pre-clip telemetry.
        self.previous_raw_action[:] = clipped_action
        return PolicyOutput(
            raw_action=raw_action.copy(),
            clipped_action=clipped_action.copy(),
            target_joint_pos=target.astype(np.float32, copy=False),
            observation=observation.copy(),
        )


class LatestLowState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._state: LowStateSnapshot | None = None

    def set(self, state: LowStateSnapshot) -> None:
        with self._lock:
            self._state = state

    def get(self) -> LowStateSnapshot | None:
        with self._lock:
            return self._state


def encode_packet(event: str, **fields: Any) -> bytes:
    payload = {"schema": CONTROL_SCHEMA, "event": str(event), **fields}
    return json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")


def decode_command(packet: bytes) -> str:
    try:
        text = packet.decode("utf-8").strip()
    except UnicodeDecodeError as exc:
        raise ValueError("control packet is not UTF-8") from exc
    if text.upper() in {"GO", "PAUSE", "STOP"}:
        return text.upper()
    try:
        payload = json.loads(text)
    except json.JSONDecodeError as exc:
        raise ValueError("control packet is neither GO/PAUSE/STOP nor JSON") from exc
    if not isinstance(payload, dict):
        raise ValueError("control packet JSON must be an object")
    schema = payload.get("schema")
    if schema not in (None, CONTROL_SCHEMA):
        raise ValueError(f"unsupported control schema: {schema!r}")
    command = str(payload.get("command", "")).upper()
    if command not in {"GO", "PAUSE", "STOP"}:
        raise ValueError(f"unsupported control command: {command!r}")
    return command


class HandoffStateMachine:
    """Resident publisher authority gate independent of DDS and sockets."""

    WAITING = "WAITING_NO_WRITER"
    PAUSED = "PAUSED_RESIDENT_WRITER"
    ACTIVE = "ACTIVE"
    STOPPED = "STOPPED"

    def __init__(
        self,
        publisher_factory: Callable[[], Any],
        event_sink: Callable[[str, Mapping[str, Any]], None],
    ) -> None:
        self._publisher_factory = publisher_factory
        self._event_sink = event_sink
        self.publisher: Any | None = None
        self.state = self.WAITING
        self.first_write_reported = False

    def announce_ready(self, fields: Mapping[str, Any] | None = None) -> None:
        if self.state != self.WAITING or self.publisher is not None:
            raise RuntimeError("READY_NO_WRITER may only be announced before GO")
        payload = dict(fields or {})
        payload["writer_created"] = False
        self._event_sink("READY_NO_WRITER", payload)

    def command(self, command: str) -> None:
        normalized = command.upper()
        if normalized == "GO":
            if self.state == self.WAITING:
                # This is the sole publisher-construction site.
                self.publisher = self._publisher_factory()
            elif self.state == self.PAUSED:
                if self.publisher is None:
                    raise RuntimeError("resident writer was lost while paused")
            elif self.state != self.ACTIVE:
                raise RuntimeError("GO received after STOP")
            if self.state != self.ACTIVE:
                self.state = self.ACTIVE
                self.first_write_reported = False
            return
        if normalized == "PAUSE":
            if self.state != self.ACTIVE:
                raise RuntimeError("PAUSE requires an active writer")
            if self.publisher is None:
                raise RuntimeError("PAUSE found no resident writer")
            # Commands and writes share this loop thread, so changing state is
            # the linearized write fence. Keep the DDS publisher resident and
            # close it exactly once at STOP/process cleanup.
            self.state = self.PAUSED
            self._event_sink(
                "PAUSED_RESIDENT_WRITER",
                {
                    "writer_created": True,
                    "write_authorized": False,
                    "writer_reused": True,
                },
            )
            return
        if normalized == "STOP":
            self.close_writer()
            self._event_sink("STOPPED", {"writer_created": False})
            return
        raise ValueError(f"unsupported command: {command!r}")

    def close_writer(self) -> None:
        """Idempotently revoke and close the publisher without socket I/O."""

        self._close_writer(next_state=self.STOPPED)

    def _close_writer(self, *, next_state: str) -> None:
        self.state = next_state
        publisher, self.publisher = self.publisher, None
        close = getattr(publisher, "Close", None)
        if callable(close):
            close()

    def record_successful_write(self) -> None:
        if self.state != self.ACTIVE or self.publisher is None:
            raise RuntimeError("cannot record a write without the active publisher")
        if not self.first_write_reported:
            self.first_write_reported = True
            self._event_sink("FIRST_WRITE", {"writer_created": True})


class HgLowCmdCrc:
    """Pure-Python Unitree HG LowCmd CRC compatible with SDK 1.0.1.

    Some Unitree Python wheels omit ``utils/lib/crc_amd64.so`` even though
    ``CRC.__init__`` unconditionally loads it.  The vendor module already
    ships this exact polynomial as its non-Linux fallback; keeping the small
    HG packer here avoids making physical recovery depend on missing package
    data while preserving the wire checksum.
    """

    _PACK_FORMAT = "<2B2x" + ("B3x5fI" * LOWCMD_SLOTS) + "5I"

    @staticmethod
    def _crc32_words(words: Sequence[int]) -> int:
        crc = 0xFFFFFFFF
        polynomial = 0x04C11DB7
        for current in words:
            bit = 1 << 31
            for _ in range(32):
                if crc & 0x80000000:
                    crc = ((crc << 1) & 0xFFFFFFFF) ^ polynomial
                else:
                    crc = (crc << 1) & 0xFFFFFFFF
                if int(current) & bit:
                    crc ^= polynomial
                bit >>= 1
        return crc

    def Crc(self, command: Any) -> int:
        values: list[int | float] = [
            int(command.mode_pr),
            int(command.mode_machine),
        ]
        if len(command.motor_cmd) < LOWCMD_SLOTS:
            raise ValueError("Unitree HG LowCmd has fewer than 35 motor slots")
        for index in range(LOWCMD_SLOTS):
            motor = command.motor_cmd[index]
            values.extend(
                (
                    int(motor.mode),
                    float(motor.q),
                    float(motor.dq),
                    float(motor.tau),
                    float(motor.kp),
                    float(motor.kd),
                    int(motor.reserve),
                )
            )
        reserve = tuple(int(value) for value in command.reserve)
        if len(reserve) != 4:
            raise ValueError("Unitree HG LowCmd reserve must contain four words")
        values.extend((*reserve, int(command.crc)))
        packed = struct.pack(self._PACK_FORMAT, *values)
        word_count_without_crc = (len(packed) // 4) - 1
        words = struct.unpack_from(
            f"<{word_count_without_crc}I", packed, 0
        )
        return self._crc32_words(words)


class _PublisherLease:
    """Release a closed SDK publisher from its resident DDS runtime."""

    def __init__(self, owner: "UnitreeDdsRuntime", publisher: Any) -> None:
        self._owner = owner
        self._publisher = publisher
        self.closed = False

    def Write(self, command: Any) -> Any:
        if self.closed:
            raise RuntimeError("cannot write through a closed LowCmd lease")
        return self._publisher.Write(command)

    def Close(self) -> None:
        if self.closed:
            return
        self.closed = True
        try:
            close = getattr(self._publisher, "Close", None)
            if callable(close):
                close()
        finally:
            self._owner._release_publisher(self)


class UnitreeDdsRuntime:
    """Lazy SDK adapter.  Constructor creates a subscriber, never a publisher."""

    def __init__(
        self,
        *,
        interface: str,
        state_store: LatestLowState,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        channel = importlib.import_module("unitree_sdk2py.core.channel")
        messages = importlib.import_module("unitree_sdk2py.idl.unitree_hg.msg.dds_")
        defaults = importlib.import_module("unitree_sdk2py.idl.default")
        self.crc_backend = "python_unitree_sdk_1_0_1"
        self._crc = HgLowCmdCrc()
        try:
            crc_module = importlib.import_module("unitree_sdk2py.utils.crc")
            self._crc = crc_module.CRC()
            self.crc_backend = "unitree_native"
        except (ImportError, AttributeError, OSError):
            # The SDK wheel can import successfully while omitting the shared
            # CRC library.  The compatible deterministic implementation above
            # remains active in that case.
            pass

        self._channel_publisher = channel.ChannelPublisher
        self._low_cmd_type = messages.LowCmd_
        self._low_cmd_factory = defaults.unitree_hg_msg_dds__LowCmd_
        self._state_store = state_store
        self._monotonic = monotonic
        self._publisher: _PublisherLease | None = None

        if interface:
            channel.ChannelFactoryInitialize(0, interface)
        else:
            channel.ChannelFactoryInitialize(0)
        # Only the LowState reader exists before GO.
        self._subscriber = channel.ChannelSubscriber("rt/lowstate", messages.LowState_)
        self._subscriber.Init(self._on_low_state, 10)

    def _on_low_state(self, message: Any) -> None:
        try:
            imu = message.imu_state
            state = LowStateSnapshot.validated(
                quaternion_wxyz=tuple(imu.quaternion),
                body_gyro_rad_s=tuple(imu.gyroscope),
                joint_pos_rad=tuple(message.motor_state[i].q for i in range(NUM_JOINTS)),
                joint_vel_rad_s=tuple(
                    message.motor_state[i].dq for i in range(NUM_JOINTS)
                ),
                received_monotonic=self._monotonic(),
                mode_pr=getattr(message, "mode_pr", 0),
                mode_machine=getattr(message, "mode_machine", 0),
            )
        except (AttributeError, IndexError, TypeError, ValueError):
            # A malformed or partially initialized DDS sample must never become
            # a policy input.  The main loop's LowState timeout remains active.
            return
        self._state_store.set(state)

    def create_publisher(self) -> Any:
        if self._publisher is not None:
            raise RuntimeError("LowCmd publisher already exists")
        publisher = self._channel_publisher("rt/lowcmd", self._low_cmd_type)
        publisher.Init()
        lease = _PublisherLease(self, publisher)
        self._publisher = lease
        return lease

    def _release_publisher(self, lease: _PublisherLease) -> None:
        if self._publisher is lease:
            self._publisher = None

    def make_low_cmd(
        self,
        target_joint_pos: np.ndarray,
        config: PolicyConfig,
        state: LowStateSnapshot,
    ) -> Any:
        command = self._low_cmd_factory()
        if hasattr(command, "mode_pr"):
            # The AMP model/config uses serial ankle pitch/roll coordinates.
            command.mode_pr = 0
        if hasattr(command, "mode_machine"):
            command.mode_machine = state.mode_machine
        if len(command.motor_cmd) < LOWCMD_SLOTS:
            raise RuntimeError(
                f"LowCmd exposes {len(command.motor_cmd)} motor slots, expected 35"
            )
        for index in range(LOWCMD_SLOTS):
            motor = command.motor_cmd[index]
            motor.mode = 0
            motor.q = 0.0
            motor.dq = 0.0
            motor.tau = 0.0
            motor.kp = 0.0
            motor.kd = 0.0
            if hasattr(motor, "reserve"):
                motor.reserve = 0
        for index in range(NUM_JOINTS):
            motor = command.motor_cmd[index]
            motor.mode = 1
            motor.q = float(target_joint_pos[index])
            motor.dq = 0.0
            motor.tau = 0.0
            motor.kp = float(config.kp[index])
            motor.kd = float(config.kd[index])
        if hasattr(command, "crc"):
            command.crc = self._crc.Crc(command)
        return command

    @staticmethod
    def write(publisher: Any, command: Any) -> bool:
        return publisher.Write(command) is not False


def state_status(state: LowStateSnapshot) -> dict[str, Any]:
    """Observable DDS status; intentionally contains no unavailable root z."""

    gyro_norm = float(np.linalg.norm(state.body_gyro_rad_s))
    joint_rms = float(math.sqrt(np.mean(np.square(state.joint_vel_rad_s))))
    return {
        "root_quaternion_wxyz": [float(value) for value in state.quaternion_wxyz],
        "projected_gravity_body": [
            float(value) for value in projected_gravity_body(state.quaternion_wxyz)
        ],
        "root_up_z_imu": root_up_z_from_imu(state.quaternion_wxyz),
        "body_gyro_rad_s": [float(value) for value in state.body_gyro_rad_s],
        "root_angular_speed_rad_s": gyro_norm,
        "joint_velocity_rms_rad_s": joint_rms,
        "root_z_available": False,
    }


def _connect_control(path: Path) -> socket.socket:
    socket_type = getattr(socket, "SOCK_SEQPACKET", None)
    if socket_type is None:
        raise RuntimeError("AF_UNIX/SOCK_SEQPACKET is unavailable")
    connection = socket.socket(socket.AF_UNIX, socket_type)
    connection.connect(str(path))
    return connection


def _advance_deadline(deadline: float, period: float, now: float) -> float:
    deadline += period
    if deadline <= now:
        return now + period
    return deadline


def run_worker(
    *,
    policy: AmpPolicyCore,
    dds: UnitreeDdsRuntime,
    state_store: LatestLowState,
    control: socket.socket,
    publish_hz: float,
    lowstate_timeout_s: float,
    status_hz: float,
    monotonic: Callable[[], float] = time.monotonic,
) -> int:
    if publish_hz < POLICY_HZ:
        raise ValueError("publish_hz must be at least the 50 Hz policy rate")
    if lowstate_timeout_s <= 0.0 or status_hz <= 0.0:
        raise ValueError("timeouts and status rate must be positive")

    def send_event(event: str, fields: Mapping[str, Any]) -> None:
        control.send(encode_packet(event, pid=os.getpid(), **dict(fields)))

    handoff = HandoffStateMachine(dds.create_publisher, send_event)
    handoff.announce_ready()
    publish_period = 1.0 / publish_hz
    policy_period = 1.0 / POLICY_HZ
    status_period = 1.0 / status_hz
    now = monotonic()
    next_publish = now
    next_policy = now
    next_status = now
    go_time: float | None = None
    latest_target: np.ndarray | None = None

    try:
        while handoff.state != HandoffStateMachine.STOPPED:
            now = monotonic()
            if handoff.state == HandoffStateMachine.ACTIVE:
                deadline = min(next_publish, next_policy, next_status)
                timeout = max(0.0, min(deadline - now, 0.05))
            else:
                timeout = 0.05
            readable, _writable, _exceptional = select.select([control], [], [], timeout)
            if readable:
                packet = control.recv(65536)
                if not packet:
                    # Loss of the supervisor is a safe implicit STOP.
                    handoff.command("STOP")
                    break
                try:
                    command = decode_command(packet)
                except ValueError as exc:
                    send_event("ERROR", {"message": str(exc)})
                    continue
                handoff.command(command)
                if command == "GO" and go_time is None:
                    go_time = monotonic()
                    next_publish = go_time
                    next_policy = go_time
                    next_status = go_time
                if command == "STOP":
                    break

            if handoff.state != HandoffStateMachine.ACTIVE:
                continue
            now = monotonic()
            state = state_store.get()
            if state is None:
                if go_time is not None and now - go_time > lowstate_timeout_s:
                    send_event("ERROR", {"message": "LowState unavailable after GO"})
                    handoff.command("STOP")
                    return 2
                continue
            state_age = now - state.received_monotonic
            if state_age > lowstate_timeout_s:
                send_event(
                    "ERROR",
                    {
                        "message": "LowState became stale",
                        "lowstate_age_s": state_age,
                    },
                )
                handoff.command("STOP")
                return 2

            if now >= next_policy:
                latest_target = policy.infer(state).target_joint_pos
                next_policy = _advance_deadline(next_policy, policy_period, now)
            if latest_target is not None and now >= next_publish:
                command = dds.make_low_cmd(latest_target, policy.config, state)
                if dds.write(handoff.publisher, command):
                    handoff.record_successful_write()
                next_publish = _advance_deadline(next_publish, publish_period, now)
            if now >= next_status:
                send_event("STATUS", state_status(state))
                next_status = _advance_deadline(next_status, status_period, now)
        return 0
    finally:
        # Socket loss or an event-send failure must revoke the DDS writer even
        # before process teardown/destructors run.
        handoff.close_writer()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--interface", required=True)
    parser.add_argument("--control-socket", required=True, type=Path)
    parser.add_argument("--publish-hz", type=float, default=500.0)
    parser.add_argument("--status-hz", type=float, default=5.0)
    parser.add_argument("--lowstate-timeout-seconds", type=float, default=0.25)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    with args.config.open("r", encoding="utf-8") as handle:
        raw_config = json.load(handle)
    config = PolicyConfig.from_mapping(raw_config)
    policy = AmpPolicyCore(config, OnnxPolicyRunner(args.model.resolve()))
    state_store = LatestLowState()
    # UnitreeDdsRuntime only constructs the LowState subscriber here.  Its
    # LowCmd publisher factory remains untouched until a supervisor sends GO.
    dds = UnitreeDdsRuntime(interface=args.interface, state_store=state_store)
    control = _connect_control(args.control_socket)
    try:
        return run_worker(
            policy=policy,
            dds=dds,
            state_store=state_store,
            control=control,
            publish_hz=args.publish_hz,
            lowstate_timeout_s=args.lowstate_timeout_seconds,
            status_hz=args.status_hz,
        )
    except (BrokenPipeError, ConnectionResetError):
        # Supervisor loss must terminate the writer rather than orphan it.
        return 3
    finally:
        control.close()


if __name__ == "__main__":
    sys.exit(main())
