#!/usr/bin/env python3
"""Run a physical G1 get-up policy as the temporary LowCmd owner.

HoST and AMP are used only as physical joint controllers.  This worker has no MuJoCo
state access and no reset, qpos, reload, or teleport path.  It listens to
LowState and loads every ONNX model before announcing ``READY_NO_WRITER``;
the LowCmd publisher is constructed only after a supervisor sends ``GO``.

The first controller can be KungFuAthleteBot recovery, AMP
``walk_run_getup``, or HoST ``prone_v1``.  In
AMP-first mode the zero-command AMP policy physically performs both get-up and
the subsequent dynamic hold.  In HoST-first mode, if the parent has not judged
the robot stable and sent ``STOP`` by ``--fallback-after-seconds``, the worker
physically continues from the current LowState with ``prone_v2``.  Switching
only resets policy observation history; it never changes simulated state.

When all AMP-hold artifacts are supplied, ``ENTER_AMP_HOLD`` atomically drops
the cached HoST target and cold-enters the preloaded AMP policy from the latest
LowState.  The AMP command observation is fixed at zero, so the same process
and same LowCmd publisher dynamically hold standing until ``STOP``.  The
supervisor receives ``AMP_HOLD_FIRST_WRITE`` only after the first AMP command
has been accepted by DDS.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import math
import os
import select
import socket
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Mapping, Protocol, Sequence

import numpy as np

from matrix_sonic_amp_worker import (
    AmpPolicyCore,
    CONTROL_SCHEMA as AMP_CONTROL_SCHEMA,
    G1_29_JOINT_NAMES,
    HandoffStateMachine,
    LatestLowState,
    LowStateSnapshot,
    NUM_JOINTS,
    OnnxPolicyRunner,
    PolicyConfig,
    UnitreeDdsRuntime,
    _advance_deadline,
    _finite_vector,
    projected_gravity_body,
    state_status,
)
from matrix_kungfu_recovery import (
    DEFAULT_REFERENCE_FRAME as KUNGFU_DEFAULT_REFERENCE_FRAME,
    KungFuRecoveryPolicy,
)
from matrix_policy_runtime import (
    ResidentPolicyAdapter,
    ResidentPolicyRegistry,
    create_inference_session,
    requested_provider_name,
)


HOST_DOF = 23
HOST_FRAME_WIDTH = 76
HOST_HISTORY_LENGTH = 6
HOST_OBSERVATION_WIDTH = HOST_FRAME_WIDTH * HOST_HISTORY_LENGTH
POLICY_HZ = 50.0
CONTROL_SCHEMA = "matrix.sonic_host_worker.control.v1"
HOST_GETUP_CONTROLLER = "HOST_GETUP"
KUNGFU_GETUP_CONTROLLER = "KUNGFU_GETUP"
AMP_GETUP_CONTROLLER = "AMP_GETUP"
AMP_ZERO_COMMAND_HOLD_CONTROLLER = "AMP_ZERO_COMMAND_HOLD"
JOINT_POSE_HOLD_CONTROLLER = "JOINT_POSE_HOLD"
POLICY_SWITCH_BLEND_S = 0.4
JOINT_HOLD_CAPTURE_MAX_AGE_S = 0.05
JOINT_HOLD_LOWSTATE_TIMEOUT_S = 1.0

# HoST public policies omit waist roll/pitch and the four wrist pitch/yaw
# joints.  These 23 indices preserve the Unitree G1/Matrix DDS order.
HOST_TO_MATRIX_INDICES = np.asarray(
    (*range(0, 13), *range(15, 20), *range(22, 27)), dtype=np.int32
)
HOST_HELD_MATRIX_INDICES = np.asarray((13, 14, 20, 21, 27, 28), dtype=np.int32)


def effective_lowstate_timeout_s(controller: str, configured_s: float) -> float:
    """Return the sensing deadline without weakening learned controllers."""

    if controller == JOINT_POSE_HOLD_CONTROLLER:
        return max(float(configured_s), JOINT_HOLD_LOWSTATE_TIMEOUT_S)
    return float(configured_s)


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def require_matching_sha256(path: Path, expected: str, label: str) -> str:
    """Validate one immutable recovery artifact before any writer is announced."""

    normalized = str(expected).strip().lower()
    if len(normalized) != 64 or any(
        character not in "0123456789abcdef" for character in normalized
    ):
        raise ValueError(f"{label} SHA256 must be 64 lowercase hex characters")
    actual = file_sha256(path)
    if actual != normalized:
        raise ValueError(
            f"{label} SHA256 mismatch: expected={normalized} actual={actual}"
        )
    return actual


def load_amp_hold_policy(
    *,
    config_path: Path,
    model_path: Path,
    config_sha256: str,
    model_sha256: str,
    execution_provider: str = "cpu",
) -> AmpPolicyCore:
    """Hash-, schema-, and ONNX-shape-check AMP before READY_NO_WRITER."""

    resolved_config = config_path.resolve()
    resolved_model = model_path.resolve()
    require_matching_sha256(resolved_config, config_sha256, "AMP hold config")
    require_matching_sha256(resolved_model, model_sha256, "AMP hold model")
    with resolved_config.open("r", encoding="utf-8") as handle:
        raw_config = json.load(handle)
    if not isinstance(raw_config, dict):
        raise ValueError("AMP hold config must be a JSON object")
    config = PolicyConfig.from_mapping(raw_config)
    # OnnxPolicyRunner construction validates 384 -> 29 before the worker
    # connects to its supervisor, let alone announces READY_NO_WRITER.
    return AmpPolicyCore(
        config,
        OnnxPolicyRunner(
            resolved_model,
            execution_provider=execution_provider,
        ),
    )


def load_kungfu_policy(
    *,
    model_path: Path,
    motion_path: Path,
    model_sha256: str,
    model_data_sha256: str,
    motion_sha256: str,
    reference_frame: int,
    gain_scale: float,
    execution_provider: str = "cpu",
) -> KungFuRecoveryPolicy:
    """Hash- and shape-check every public KungFu artifact before READY."""

    resolved_model = model_path.resolve()
    resolved_data = resolved_model.with_name(f"{resolved_model.name}.data")
    resolved_motion = motion_path.resolve()
    require_matching_sha256(resolved_model, model_sha256, "KungFu ONNX")
    require_matching_sha256(
        resolved_data, model_data_sha256, "KungFu ONNX external data"
    )
    require_matching_sha256(resolved_motion, motion_sha256, "KungFu motion")
    return KungFuRecoveryPolicy.from_artifacts(
        model_path=resolved_model,
        motion_path=resolved_motion,
        reference_frame=reference_frame,
        gain_scale=gain_scale,
        execution_provider=execution_provider,
    )


def host_gains() -> tuple[np.ndarray, np.ndarray]:
    """Return the gains used by the public HoST G1 MuJoCo deployment."""

    kp = np.full(NUM_JOINTS, 100.0, dtype=np.float32)
    kd = np.full(NUM_JOINTS, 4.0, dtype=np.float32)
    for index in (0, 1, 2, 6, 7, 8):
        kp[index] = 150.0
    for index in (3, 9):
        kp[index] = 200.0
        kd[index] = 6.0
    for index in (4, 5, 10, 11):
        kp[index] = 40.0
        kd[index] = 2.0
    return kp, kd


@dataclass(frozen=True)
class HostControlConfig:
    action_rescale: float
    action_clip: float
    kp: np.ndarray
    kd: np.ndarray

    @classmethod
    def create(
        cls,
        *,
        action_rescale: float = 0.25,
        action_clip: float = 100.0,
    ) -> "HostControlConfig":
        scale = float(action_rescale)
        clip = float(action_clip)
        if not math.isfinite(scale) or scale <= 0.0:
            raise ValueError("action_rescale must be finite and positive")
        if not math.isfinite(clip) or clip <= 0.0:
            raise ValueError("action_clip must be finite and positive")
        kp, kd = host_gains()
        return cls(action_rescale=scale, action_clip=clip, kp=kp, kd=kd)


class HostRunner(Protocol):
    label: str

    def __call__(self, observation: np.ndarray) -> np.ndarray: ...


class HostOnnxRunner:
    """Lazy ONNX wrapper enforcing HoST's public 456 -> 23 contract."""

    def __init__(self, model_path: Path, *, execution_provider: str = "cpu"):
        self.path = model_path.resolve()
        self.label = self.path.name
        ort = importlib.import_module("onnxruntime")
        self._session, self.execution_provider = create_inference_session(
            ort,
            str(self.path),
            execution_provider,
        )
        inputs = self._session.get_inputs()
        outputs = self._session.get_outputs()
        if len(inputs) != 1 or len(outputs) != 1:
            raise ValueError("HoST ONNX must expose exactly one input and one output")
        input_width = inputs[0].shape[-1]
        output_width = outputs[0].shape[-1]
        if input_width != HOST_OBSERVATION_WIDTH or output_width != HOST_DOF:
            raise ValueError(
                "unexpected HoST ONNX contract: "
                f"{inputs[0].shape!r} -> {outputs[0].shape!r}; expected 456 -> 23"
            )
        self._input_name = inputs[0].name
        self._output_name = outputs[0].name
        # Warm every policy before this process can attest resident readiness.
        self(np.zeros(HOST_OBSERVATION_WIDTH, dtype=np.float32))

    def __call__(self, observation: np.ndarray) -> np.ndarray:
        batch = np.asarray(observation, dtype=np.float32).reshape(
            1, HOST_OBSERVATION_WIDTH
        )
        return np.asarray(
            self._session.run([self._output_name], {self._input_name: batch})[0]
        ).reshape(-1)


@dataclass(frozen=True)
class HostPolicyOutput:
    raw_action: np.ndarray
    clipped_action: np.ndarray
    target_joint_pos: np.ndarray
    observation: np.ndarray


class HostPolicyCore:
    """Pure six-frame observation and incremental-position policy core."""

    def __init__(self, config: HostControlConfig, runner: HostRunner):
        self.config = config
        self.runner = runner
        self.previous_action = np.zeros(HOST_DOF, dtype=np.float32)
        self._history: deque[np.ndarray] = deque(maxlen=HOST_HISTORY_LENGTH)
        self._held_joint_target: np.ndarray | None = None

    def reset_history(self, state: LowStateSnapshot) -> None:
        self.previous_action.fill(0.0)
        self._held_joint_target = state.joint_pos_rad.copy()
        frame = self.observation_frame(state)
        self._history.clear()
        self._history.extend(frame.copy() for _ in range(HOST_HISTORY_LENGTH))

    def observation_frame(self, state: LowStateSnapshot) -> np.ndarray:
        controlled_pos = state.joint_pos_rad[HOST_TO_MATRIX_INDICES]
        controlled_vel = state.joint_vel_rad_s[HOST_TO_MATRIX_INDICES]
        frame = np.concatenate(
            (
                state.body_gyro_rad_s * np.float32(0.25),
                projected_gravity_body(state.quaternion_wxyz),
                controlled_pos,
                controlled_vel * np.float32(0.05),
                self.previous_action,
                np.asarray((self.config.action_rescale,), dtype=np.float32),
            )
        ).astype(np.float32, copy=False)
        if frame.shape != (HOST_FRAME_WIDTH,):
            raise AssertionError(f"internal HoST observation shape error: {frame.shape}")
        return frame

    def build_observation(self, state: LowStateSnapshot) -> np.ndarray:
        if not self._history:
            self.reset_history(state)
        else:
            self._history.append(self.observation_frame(state))
        observation = np.concatenate(tuple(self._history)).astype(
            np.float32, copy=False
        )
        if observation.shape != (HOST_OBSERVATION_WIDTH,):
            raise AssertionError(f"internal HoST history shape error: {observation.shape}")
        return observation

    def infer(self, state: LowStateSnapshot) -> HostPolicyOutput:
        observation = self.build_observation(state)
        raw_action = _finite_vector(
            self.runner(observation[None, :]), HOST_DOF, "HoST raw action"
        )
        clipped_action = np.clip(
            raw_action, -self.config.action_clip, self.config.action_clip
        ).astype(np.float32, copy=False)
        if self._held_joint_target is None:
            raise AssertionError("HoST held target was not initialized")
        target = self._held_joint_target.copy()
        # HoST's public deployment is delta-position control relative to the
        # physically observed joint positions at this policy tick.
        target[HOST_TO_MATRIX_INDICES] = (
            state.joint_pos_rad[HOST_TO_MATRIX_INDICES]
            + self.config.action_rescale * clipped_action
        )
        self.previous_action[:] = clipped_action
        return HostPolicyOutput(
            raw_action=raw_action.copy(),
            clipped_action=clipped_action.copy(),
            target_joint_pos=target.astype(np.float32, copy=False),
            observation=observation.copy(),
        )


class HostPolicyCascade:
    """Supervisor-authorized physical continuation across compatible HoST models.

    The worker can observe IMU/joints but not simulator root height or contacts,
    so its local timeout only announces that a fallback is due.  The Matrix
    supervisor owns the full physical state and must explicitly authorize the
    switch with ADVANCE_POLICY.
    """

    def __init__(
        self,
        *,
        config: HostControlConfig,
        runners: Sequence[HostRunner],
        fallback_after_s: float,
        policy_switch_blend_s: float = POLICY_SWITCH_BLEND_S,
    ) -> None:
        if not runners:
            raise ValueError("at least one HoST runner is required")
        timeout = float(fallback_after_s)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError("fallback_after_s must be finite and positive")
        self.config = config
        self.runners = tuple(runners)
        self.fallback_after_s = timeout
        self.policy_switch_blend_s = float(policy_switch_blend_s)
        if not math.isfinite(self.policy_switch_blend_s) or self.policy_switch_blend_s <= 0.0:
            raise ValueError("policy_switch_blend_s must be finite and positive")
        self.index = 0
        self.core = HostPolicyCore(config, self.runners[0])
        self.policy_started_monotonic: float | None = None
        self.fallback_pending = False
        self._switch_blend_origin: np.ndarray | None = None
        self._switch_blend_started_monotonic: float | None = None

    @property
    def label(self) -> str:
        return str(getattr(self.runners[self.index], "label", f"policy_{self.index}"))

    def start(self, state: LowStateSnapshot, now: float) -> None:
        self.index = 0
        self.core = HostPolicyCore(self.config, self.runners[0])
        self.core.reset_history(state)
        self.policy_started_monotonic = float(now)
        self.fallback_pending = False
        self._switch_blend_origin = None
        self._switch_blend_started_monotonic = None

    def maybe_request_fallback(
        self, state: LowStateSnapshot, now: float
    ) -> dict[str, Any] | None:
        if self.policy_started_monotonic is None:
            self.start(state, now)
            return None
        elapsed = float(now) - self.policy_started_monotonic
        if (
            self.index + 1 >= len(self.runners)
            or elapsed < self.fallback_after_s
            or self.fallback_pending
        ):
            return None
        self.fallback_pending = True
        status = state_status(state)
        return {
            "policy_index": self.index,
            "policy": self.label,
            "next_policy_index": self.index + 1,
            "next_policy": str(
                getattr(
                    self.runners[self.index + 1],
                    "label",
                    f"policy_{self.index + 1}",
                )
            ),
            "policy_elapsed_s": elapsed,
            "requires_supervisor_authorization": True,
            "root_up_z_imu": status["root_up_z_imu"],
            "root_angular_speed_rad_s": status["root_angular_speed_rad_s"],
            "joint_velocity_rms_rad_s": status["joint_velocity_rms_rad_s"],
        }

    def advance(self, state: LowStateSnapshot, now: float) -> dict[str, Any]:
        if not self.fallback_pending:
            raise RuntimeError("ADVANCE_POLICY arrived before fallback was due")
        if self.index + 1 >= len(self.runners):
            raise RuntimeError("ADVANCE_POLICY has no remaining fallback model")
        old_index = self.index
        old_label = self.label
        self.index += 1
        self.core = HostPolicyCore(self.config, self.runners[self.index])
        # This reset copies the *current LowState* for held joints/history.  It
        # does not and cannot write simulator state, so the transition remains
        # physically continuous.
        self.core.reset_history(state)
        self.policy_started_monotonic = float(now)
        self.fallback_pending = False
        self._switch_blend_origin = state.joint_pos_rad.copy()
        # Start the blend at the authorization boundary, not at the timestamp
        # of the LowState sample used to seed it.  A valid sample may already
        # be up to lowstate_timeout_s old; using its receive timestamp could
        # therefore skip most of a 0.4 s blend on the first command.
        self._switch_blend_started_monotonic = float(now)
        return {
            "from_policy_index": old_index,
            "from_policy": old_label,
            "to_policy_index": self.index,
            "to_policy": self.label,
            "physical_continuation": True,
            "target_blend_seconds": self.policy_switch_blend_s,
        }

    def infer(self, state: LowStateSnapshot) -> HostPolicyOutput:
        output = self.core.infer(state)
        if (
            self._switch_blend_origin is None
            or self._switch_blend_started_monotonic is None
        ):
            return output
        alpha = float(
            np.clip(
                (state.received_monotonic - self._switch_blend_started_monotonic)
                / self.policy_switch_blend_s,
                0.0,
                1.0,
            )
        )
        blended_target = (
            self._switch_blend_origin * (1.0 - alpha)
            + output.target_joint_pos * alpha
        ).astype(np.float32, copy=False)
        applied_action = (
            (
                blended_target[HOST_TO_MATRIX_INDICES]
                - state.joint_pos_rad[HOST_TO_MATRIX_INDICES]
            )
            / self.config.action_rescale
        ).astype(np.float32, copy=False)
        self.core.previous_action[:] = applied_action
        if alpha >= 1.0:
            self._switch_blend_origin = None
            self._switch_blend_started_monotonic = None
        return HostPolicyOutput(
            raw_action=output.raw_action,
            clipped_action=applied_action.copy(),
            target_joint_pos=blended_target,
            observation=output.observation,
        )


def encode_packet(event: str, **fields: Any) -> bytes:
    payload = {"schema": CONTROL_SCHEMA, "event": str(event), **fields}
    return json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")


def decode_command_envelope(packet: bytes) -> tuple[str, dict[str, Any]]:
    try:
        text = packet.decode("utf-8").strip()
    except UnicodeDecodeError as exc:
        raise ValueError("control packet is not UTF-8") from exc
    if text.upper() in {
        "GO",
        "PAUSE",
        "STOP",
        "ENTER_AMP_HOLD",
        "ENTER_JOINT_HOLD",
        "ADVANCE_POLICY",
    }:
        return text.upper(), {}
    try:
        payload = json.loads(text)
    except json.JSONDecodeError as exc:
        raise ValueError("control packet is neither GO/STOP nor JSON") from exc
    if not isinstance(payload, dict):
        raise ValueError("control packet JSON must be an object")
    schema = payload.get("schema")
    if schema not in (None, CONTROL_SCHEMA, AMP_CONTROL_SCHEMA):
        raise ValueError(f"unsupported control schema: {schema!r}")
    command = str(payload.get("command", "")).upper()
    if command not in {
        "GO",
        "PAUSE",
        "STOP",
        "ENTER_AMP_HOLD",
        "ENTER_JOINT_HOLD",
        "ADVANCE_POLICY",
    }:
        raise ValueError(f"unsupported control command: {command!r}")
    return command, dict(payload)


def decode_command(packet: bytes) -> str:
    """Compatibility wrapper returning only the decoded command name."""

    return decode_command_envelope(packet)[0]


def _connect_control(path: Path) -> socket.socket:
    socket_type = getattr(socket, "SOCK_SEQPACKET", None)
    if socket_type is None:
        raise RuntimeError("AF_UNIX/SOCK_SEQPACKET is unavailable")
    connection = socket.socket(socket.AF_UNIX, socket_type)
    connection.connect(str(path))
    return connection


def build_resident_policy_registry(
    *,
    cascade: HostPolicyCascade,
    amp_hold_policy: AmpPolicyCore | None,
    kungfu_policy: KungFuRecoveryPolicy | None,
    execution_provider: str,
) -> ResidentPolicyRegistry:
    """Register every loaded controller behind one policy-id dispatch API."""

    provider_name = requested_provider_name(execution_provider)
    registry = ResidentPolicyRegistry(provider_name)
    registry.register(
        ResidentPolicyAdapter(
            policy_id="host",
            controller=HOST_GETUP_CONTROLLER,
            execution_provider=provider_name,
            command_config=cascade.config,
            start_episode_fn=cascade.start,
            infer_target_fn=lambda state, _now: cascade.infer(
                state
            ).target_joint_pos,
            status_fields_fn=lambda now, _started: {
                "policy_index": cascade.index,
                "policy": cascade.label,
                "policy_elapsed_s": now
                - float(cascade.policy_started_monotonic),
                "action_rescale": cascade.config.action_rescale,
                "fallback_pending": cascade.fallback_pending,
            },
        )
    )
    if amp_hold_policy is not None:
        registry.register(
            ResidentPolicyAdapter(
                policy_id="amp",
                controller=AMP_GETUP_CONTROLLER,
                execution_provider=provider_name,
                command_config=amp_hold_policy.config,
                start_episode_fn=lambda state, _now: amp_hold_policy.reset_history(
                    state
                ),
                infer_target_fn=lambda state, _now: amp_hold_policy.infer(
                    state
                ).target_joint_pos,
                status_fields_fn=lambda now, started: {
                    "policy_index": None,
                    "policy": "amp_walk_run_getup",
                    "policy_elapsed_s": now - started,
                    "command": [0.0, 0.0, 0.0],
                },
            )
        )
    if kungfu_policy is not None:
        registry.register(
            ResidentPolicyAdapter(
                policy_id="kungfu",
                controller=KUNGFU_GETUP_CONTROLLER,
                execution_provider=provider_name,
                command_config=kungfu_policy.config,
                start_episode_fn=lambda state, _now: kungfu_policy.start(
                    base_quat=state.quaternion_wxyz,
                    joint_pos=state.joint_pos_rad,
                ),
                infer_target_fn=lambda state, _now: kungfu_policy.infer(
                    base_quat=state.quaternion_wxyz,
                    base_ang_vel=state.body_gyro_rad_s,
                    joint_pos=state.joint_pos_rad,
                    joint_vel=state.joint_vel_rad_s,
                ).target_joint_pos,
                status_fields_fn=lambda now, started: {
                    "policy_index": None,
                    "policy": "kungfu_1307_recovery",
                    "policy_elapsed_s": now - started,
                    "reference_frame": kungfu_policy.reference.frame,
                    "reference_frozen": kungfu_policy.reference_is_frozen,
                    "reference_source_frames": kungfu_policy.reference.source_frames,
                },
            )
        )
    return registry


def run_worker(
    *,
    cascade: HostPolicyCascade,
    amp_hold_policy: AmpPolicyCore | None,
    kungfu_policy: KungFuRecoveryPolicy | None = None,
    dds: UnitreeDdsRuntime,
    state_store: LatestLowState,
    control: socket.socket,
    publish_hz: float,
    lowstate_timeout_s: float,
    status_hz: float,
    initial_controller: str = "host",
    resident_policies: Sequence[Mapping[str, Any]] = (),
    execution_provider: str = "cpu",
    monotonic: Callable[[], float] = time.monotonic,
) -> int:
    if publish_hz < POLICY_HZ:
        raise ValueError("publish_hz must be at least the 50 Hz policy rate")
    if lowstate_timeout_s <= 0.0 or status_hz <= 0.0:
        raise ValueError("timeouts and status rate must be positive")

    active_episode_id: int | None = None

    def send_event(event: str, fields: Mapping[str, Any]) -> None:
        event_fields = dict(fields)
        if active_episode_id is not None:
            event_fields.setdefault("episode_id", active_episode_id)
        control.send(encode_packet(event, pid=os.getpid(), **event_fields))

    provider_name = requested_provider_name(execution_provider)
    policy_registry = build_resident_policy_registry(
        cascade=cascade,
        amp_hold_policy=amp_hold_policy,
        kungfu_policy=kungfu_policy,
        execution_provider=execution_provider,
    )
    initial_policy = policy_registry.require(initial_controller)
    resident_manifest = [dict(item) for item in resident_policies]
    if not resident_manifest:
        raise ValueError("resident policy manifest cannot be empty")
    if any(item.get("execution_provider") != provider_name for item in resident_manifest):
        raise ValueError("resident policy provider attestation is inconsistent")
    handoff = HandoffStateMachine(dds.create_publisher, send_event)
    handoff.announce_ready(
        {
            "execution_provider": provider_name,
            "resident_policies": resident_manifest,
            "resident_policy_count": len(resident_manifest),
            "registered_policy_ids": list(policy_registry.policy_ids),
            "initial_policy_id": initial_policy.policy_id,
            "models_loaded_once": True,
            "models_warmed": True,
        }
    )
    publish_period = 1.0 / publish_hz
    policy_period = 1.0 / POLICY_HZ
    status_period = 1.0 / status_hz
    now = monotonic()
    next_publish = now
    next_policy = now
    next_status = now
    go_time: float | None = None
    latest_target: np.ndarray | None = None
    controller = initial_policy.controller
    amp_started_monotonic: float | None = None
    kungfu_started_monotonic: float | None = None
    amp_hold_first_write_pending = False
    amp_hold_first_write_reported = False
    joint_hold_target: np.ndarray | None = None
    joint_hold_config: HostControlConfig | PolicyConfig | None = None
    joint_hold_started_monotonic: float | None = None
    joint_hold_capture_age_s: float | None = None
    joint_hold_transition_id: str | None = None
    joint_hold_previous_controller: str | None = None
    joint_hold_first_write_pending = False
    joint_hold_first_write_reported = False
    amp_hold_transition_id: str | None = None
    amp_hold_previous_controller: str | None = None
    amp_hold_history_reset = False
    pending_policy_switch_first_write: dict[str, Any] | None = None

    def reset_episode_controller(state: LowStateSnapshot, start_now: float) -> None:
        nonlocal controller
        nonlocal latest_target, amp_started_monotonic, kungfu_started_monotonic
        nonlocal amp_hold_first_write_pending, amp_hold_first_write_reported
        nonlocal joint_hold_target, joint_hold_config
        nonlocal joint_hold_started_monotonic, joint_hold_capture_age_s
        nonlocal joint_hold_transition_id, joint_hold_previous_controller
        nonlocal joint_hold_first_write_pending, joint_hold_first_write_reported
        nonlocal amp_hold_transition_id, amp_hold_previous_controller
        nonlocal amp_hold_history_reset, pending_policy_switch_first_write

        controller = initial_policy.controller
        latest_target = None
        amp_started_monotonic = None
        kungfu_started_monotonic = None
        amp_hold_first_write_pending = False
        amp_hold_first_write_reported = False
        joint_hold_target = None
        joint_hold_config = None
        joint_hold_started_monotonic = None
        joint_hold_capture_age_s = None
        joint_hold_transition_id = None
        joint_hold_previous_controller = None
        joint_hold_first_write_pending = False
        joint_hold_first_write_reported = False
        amp_hold_transition_id = None
        amp_hold_previous_controller = None
        amp_hold_history_reset = False
        pending_policy_switch_first_write = None
        active_policy = policy_registry.for_controller(controller)
        if active_policy is None:
            raise RuntimeError(
                f"initial resident controller is not registered: {controller!r}"
            )
        active_policy.start_episode(state, start_now)
        if controller == KUNGFU_GETUP_CONTROLLER:
            kungfu_started_monotonic = start_now
        elif controller == AMP_GETUP_CONTROLLER:
            amp_started_monotonic = start_now

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
                    handoff.command("STOP")
                    break
                try:
                    command, command_fields = decode_command_envelope(packet)
                except ValueError as exc:
                    send_event("ERROR", {"message": str(exc)})
                    continue
                if command in {"GO", "PAUSE", "STOP"}:
                    requested_episode_id = command_fields.get("episode_id")
                    if requested_episode_id is not None:
                        if (
                            type(requested_episode_id) is not int
                            or requested_episode_id <= 0
                        ):
                            send_event(
                                "ERROR",
                                {
                                    "message": (
                                        f"{command} episode_id must be a "
                                        "positive integer"
                                    )
                                },
                            )
                            continue
                        new_resident_episode = (
                            command == "GO"
                            and handoff.state
                            in {
                                HandoffStateMachine.WAITING,
                                HandoffStateMachine.PAUSED,
                            }
                        )
                        if (
                            active_episode_id is not None
                            and active_episode_id != requested_episode_id
                            and not new_resident_episode
                        ):
                            send_event(
                                "ERROR",
                                {"message": f"{command} episode_id mismatch"},
                            )
                            continue
                        active_episode_id = requested_episode_id
                    if command == "GO" and go_time is None:
                        start_now = monotonic()
                        start_state = state_store.get()
                        if start_state is None:
                            send_event(
                                "ERROR",
                                {"message": "LowState unavailable at resident GO"},
                            )
                            return 2
                        start_state_age = (
                            start_now - start_state.received_monotonic
                        )
                        if (
                            start_state_age < 0.0
                            or start_state_age > lowstate_timeout_s
                        ):
                            send_event(
                                "ERROR",
                                {
                                    "message": "LowState stale at resident GO",
                                    "lowstate_age_s": start_state_age,
                                },
                            )
                            return 2
                        # Every model remains loaded. Only per-episode observation
                        # history and physical reference alignment are reset.
                        reset_episode_controller(start_state, start_now)
                    handoff.command(command)
                    if command == "GO" and go_time is None:
                        go_time = monotonic()
                        next_publish = go_time
                        next_policy = go_time
                        next_status = go_time
                    if command == "PAUSE":
                        go_time = None
                        latest_target = None
                        active_episode_id = None
                    if command == "STOP":
                        break
                elif command == "ADVANCE_POLICY":
                    if handoff.state != HandoffStateMachine.ACTIVE:
                        send_event(
                            "ERROR",
                            {"message": "ADVANCE_POLICY requires an active writer"},
                        )
                        continue
                    if controller != HOST_GETUP_CONTROLLER:
                        send_event(
                            "ERROR",
                            {"message": "ADVANCE_POLICY requires HoST control"},
                        )
                        continue
                    requested_episode_id = command_fields.get("episode_id")
                    transition_id = command_fields.get("transition_id")
                    if active_episode_id is not None and requested_episode_id != active_episode_id:
                        send_event(
                            "ERROR",
                            {
                                "message": "ADVANCE_POLICY episode_id mismatch",
                                "expected_episode_id": active_episode_id,
                                "received_episode_id": requested_episode_id,
                            },
                        )
                        continue
                    if transition_id is not None and (
                        not isinstance(transition_id, str) or not transition_id
                    ):
                        send_event(
                            "ERROR",
                            {"message": "ADVANCE_POLICY transition_id must be non-empty"},
                        )
                        continue
                    transition_now = monotonic()
                    transition_state = state_store.get()
                    if transition_state is None:
                        send_event(
                            "ERROR",
                            {"message": "LowState unavailable at policy transition"},
                        )
                        continue
                    transition_state_age = (
                        transition_now - transition_state.received_monotonic
                    )
                    if (
                        transition_state_age < 0.0
                        or transition_state_age > lowstate_timeout_s
                    ):
                        send_event(
                            "ERROR",
                            {
                                "message": "LowState stale at policy transition",
                                "lowstate_age_s": transition_state_age,
                            },
                        )
                        continue
                    try:
                        switch = cascade.advance(transition_state, transition_now)
                    except RuntimeError as exc:
                        send_event("ERROR", {"message": str(exc)})
                        continue
                    switch.update(
                        {
                            "episode_id": active_episode_id,
                            "transition_id": transition_id,
                        }
                    )
                    latest_target = None
                    next_policy = transition_now
                    next_publish = transition_now
                    next_status = transition_now
                    pending_policy_switch_first_write = dict(switch)
                    send_event("POLICY_SWITCH", switch)
                elif command == "ENTER_JOINT_HOLD":
                    if handoff.state != HandoffStateMachine.ACTIVE:
                        send_event(
                            "ERROR",
                            {"message": "ENTER_JOINT_HOLD requires an active writer"},
                        )
                        continue
                    if controller == JOINT_POSE_HOLD_CONTROLLER:
                        # A duplicate request must not recapture a moving
                        # measured pose or reset the hold timer.
                        continue
                    if controller not in {
                        HOST_GETUP_CONTROLLER,
                        KUNGFU_GETUP_CONTROLLER,
                        AMP_GETUP_CONTROLLER,
                    }:
                        send_event(
                            "ERROR",
                            {
                                "message": (
                                    "ENTER_JOINT_HOLD requires an active "
                                    "get-up controller"
                                )
                            },
                        )
                        continue
                    if pending_policy_switch_first_write is not None:
                        send_event(
                            "ERROR",
                            {
                                "message": (
                                    "ENTER_JOINT_HOLD cannot interrupt an "
                                    "unacknowledged policy switch"
                                )
                            },
                        )
                        continue
                    requested_episode_id = command_fields.get("episode_id")
                    if (
                        active_episode_id is not None
                        and requested_episode_id != active_episode_id
                    ):
                        send_event(
                            "ERROR",
                            {"message": "ENTER_JOINT_HOLD episode_id mismatch"},
                        )
                        continue
                    transition_id = command_fields.get("transition_id")
                    if transition_id is not None and (
                        not isinstance(transition_id, str) or not transition_id
                    ):
                        send_event(
                            "ERROR",
                            {
                                "message": (
                                    "ENTER_JOINT_HOLD transition_id must be "
                                    "non-empty"
                                )
                            },
                        )
                        continue
                    transition_now = monotonic()
                    transition_state = state_store.get()
                    if transition_state is None:
                        send_event(
                            "ERROR",
                            {"message": "LowState unavailable at joint hold"},
                        )
                        continue
                    transition_state_age = (
                        transition_now - transition_state.received_monotonic
                    )
                    if (
                        transition_state_age < 0.0
                        or transition_state_age
                        > min(lowstate_timeout_s, JOINT_HOLD_CAPTURE_MAX_AGE_S)
                    ):
                        send_event(
                            "ERROR",
                            {
                                "message": "LowState stale at joint hold",
                                "lowstate_age_s": transition_state_age,
                                "maximum_capture_age_s": (
                                    JOINT_HOLD_CAPTURE_MAX_AGE_S
                                ),
                            },
                        )
                        continue
                    # Freeze the measured physical pose through the same DDS
                    # publisher and the same joint PD gains.  This lets the
                    # already-upright robot remain supported while replacement
                    # SONIC finishes writer-free prewarming.
                    joint_hold_previous_controller = controller
                    latest_target = None
                    joint_hold_target = transition_state.joint_pos_rad.copy()
                    joint_hold_config = (
                        cascade.config
                        if controller == HOST_GETUP_CONTROLLER
                        else (
                            kungfu_policy.config
                            if controller == KUNGFU_GETUP_CONTROLLER
                            else amp_hold_policy.config
                        )
                    )
                    joint_hold_started_monotonic = transition_now
                    joint_hold_capture_age_s = transition_state_age
                    joint_hold_transition_id = transition_id
                    joint_hold_first_write_pending = True
                    controller = JOINT_POSE_HOLD_CONTROLLER
                    next_policy = transition_now
                    next_publish = transition_now
                    next_status = transition_now
                elif command == "ENTER_AMP_HOLD":
                    if handoff.state != HandoffStateMachine.ACTIVE:
                        send_event(
                            "ERROR",
                            {"message": "ENTER_AMP_HOLD requires an active writer"},
                        )
                        continue
                    if controller == AMP_ZERO_COMMAND_HOLD_CONTROLLER:
                        # Do not reset policy history on duplicate delivery.
                        continue
                    if amp_hold_policy is None:
                        send_event(
                            "ERROR",
                            {"message": "AMP hold policy was not preloaded"},
                        )
                        handoff.command("STOP")
                        return 2
                    if controller not in {
                        HOST_GETUP_CONTROLLER,
                        KUNGFU_GETUP_CONTROLLER,
                        AMP_GETUP_CONTROLLER,
                    }:
                        send_event(
                            "ERROR",
                            {
                                "message": (
                                    "ENTER_AMP_HOLD requires an active "
                                    "get-up controller"
                                )
                            },
                        )
                        continue
                    if pending_policy_switch_first_write is not None:
                        send_event(
                            "ERROR",
                            {
                                "message": (
                                    "ENTER_AMP_HOLD cannot interrupt an "
                                    "unacknowledged policy switch"
                                )
                            },
                        )
                        continue
                    requested_episode_id = command_fields.get("episode_id")
                    if (
                        active_episode_id is not None
                        and requested_episode_id != active_episode_id
                    ):
                        send_event(
                            "ERROR",
                            {"message": "ENTER_AMP_HOLD episode_id mismatch"},
                        )
                        continue
                    transition_id = command_fields.get("transition_id")
                    if transition_id is not None and (
                        not isinstance(transition_id, str) or not transition_id
                    ):
                        send_event(
                            "ERROR",
                            {
                                "message": (
                                    "ENTER_AMP_HOLD transition_id must be "
                                    "non-empty"
                                )
                            },
                        )
                        continue
                    transition_now = monotonic()
                    transition_state = state_store.get()
                    if transition_state is None:
                        send_event(
                            "ERROR",
                            {"message": "LowState unavailable at AMP hold transition"},
                        )
                        handoff.command("STOP")
                        return 2
                    transition_state_age = (
                        transition_now - transition_state.received_monotonic
                    )
                    if (
                        transition_state_age < 0.0
                        or transition_state_age > lowstate_timeout_s
                    ):
                        send_event(
                            "ERROR",
                            {
                                "message": "LowState stale at AMP hold transition",
                                "lowstate_age_s": transition_state_age,
                            },
                        )
                        handoff.command("STOP")
                        return 2

                    # This is the atomic policy boundary.  The loop is the only
                    # LowCmd producer, so clearing this cache guarantees that no
                    # HoST target can be written after ENTER_AMP_HOLD.  AMP is
                    # cold-entered from the latest physical LowState; no qpos,
                    # reset, reload, or teleport API exists in this process.
                    amp_hold_previous_controller = controller
                    amp_hold_transition_id = transition_id
                    latest_target = None
                    amp_hold_history_reset = controller in {
                        HOST_GETUP_CONTROLLER,
                        KUNGFU_GETUP_CONTROLLER,
                    }
                    if amp_hold_history_reset:
                        # HoST and AMP have different observation histories;
                        # cold-enter AMP from the latest physical LowState.
                        amp_hold_policy.reset_history(transition_state)
                    # AMP-first already runs this exact zero-command policy.
                    # Preserve its recurrent history and previous action when
                    # merely relabelling get-up as the standing hold phase.
                    controller = AMP_ZERO_COMMAND_HOLD_CONTROLLER
                    amp_started_monotonic = transition_now
                    amp_hold_first_write_pending = True
                    next_policy = transition_now
                    next_publish = transition_now
                    next_status = transition_now
                else:  # pragma: no cover - decode_command is the authority.
                    raise AssertionError(f"unhandled command: {command}")

            if handoff.state != HandoffStateMachine.ACTIVE:
                now = monotonic()
                if now >= next_status:
                    standby_state = state_store.get()
                    resident_writer_created = handoff.publisher is not None
                    lowstate_age_s = (
                        None
                        if standby_state is None
                        else max(0.0, now - standby_state.received_monotonic)
                    )
                    send_event(
                        "STATUS",
                        {
                            "controller": (
                                "PAUSED_RESIDENT_WRITER"
                                if resident_writer_created
                                else "WRITER_FREE_STANDBY"
                            ),
                            "active_policy_id": None,
                            "registered_policy_ids": list(
                                policy_registry.policy_ids
                            ),
                            "writer_created": resident_writer_created,
                            "write_authorized": False,
                            "lowstate_available": standby_state is not None,
                            "lowstate_age_s": lowstate_age_s,
                            "execution_provider": provider_name,
                            "resident_policy_count": len(resident_manifest),
                            "models_loaded_once": True,
                            "models_warmed": True,
                        },
                    )
                    next_status = _advance_deadline(
                        next_status, status_period, now
                    )
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
            # TensorRT prewarm in the replacement SONIC can briefly starve
            # the simulator/LowState publisher for more than 250 ms.  A joint
            # hold has already captured one fresh pose and publishes a fixed
            # zero-dq/zero-feedforward PD command, so it can safely bridge that
            # scheduling gap.  Learned HoST/AMP control still fails closed at
            # the original, much tighter sensing deadline.
            state_timeout_s = effective_lowstate_timeout_s(
                controller, lowstate_timeout_s
            )
            if state_age > state_timeout_s:
                send_event(
                    "ERROR",
                    {
                        "message": "LowState became stale",
                        "lowstate_age_s": state_age,
                        "lowstate_timeout_s": state_timeout_s,
                        "controller": controller,
                    },
                )
                handoff.command("STOP")
                return 2

            if controller == HOST_GETUP_CONTROLLER:
                if cascade.policy_started_monotonic is None:
                    cascade.start(state, now)
                fallback_request = cascade.maybe_request_fallback(state, now)
                if fallback_request is not None:
                    send_event("POLICY_FALLBACK_DUE", fallback_request)
            if now >= next_policy:
                active_policy = policy_registry.for_controller(controller)
                if active_policy is not None:
                    latest_target = active_policy.infer_target(state, now)
                elif controller == JOINT_POSE_HOLD_CONTROLLER:
                    assert joint_hold_target is not None
                    latest_target = joint_hold_target.copy()
                else:
                    assert amp_hold_policy is not None
                    latest_target = amp_hold_policy.infer(state).target_joint_pos
                next_policy = _advance_deadline(next_policy, policy_period, now)
            if latest_target is not None and now >= next_publish:
                active_policy = policy_registry.for_controller(controller)
                if active_policy is not None:
                    command_config = active_policy.command_config
                elif controller == JOINT_POSE_HOLD_CONTROLLER:
                    command_config = joint_hold_config
                else:
                    assert amp_hold_policy is not None
                    command_config = amp_hold_policy.config
                assert command_config is not None
                command = dds.make_low_cmd(latest_target, command_config, state)
                if dds.write(handoff.publisher, command):
                    handoff.record_successful_write()
                    if pending_policy_switch_first_write is not None:
                        send_event(
                            "POLICY_SWITCH_FIRST_WRITE",
                            {
                                **pending_policy_switch_first_write,
                                "writer_reused": True,
                            },
                        )
                        pending_policy_switch_first_write = None
                    if joint_hold_first_write_pending:
                        send_event(
                            "JOINT_HOLD_FIRST_WRITE",
                            {
                                "controller": JOINT_POSE_HOLD_CONTROLLER,
                                "transition_id": joint_hold_transition_id,
                                "writer_reused": True,
                                "measured_joint_target": True,
                                "measured_joint_count": NUM_JOINTS,
                                "capture_once": True,
                                "lowstate_capture_age_s": (
                                    joint_hold_capture_age_s
                                ),
                                "target_velocity_zero": True,
                                "feedforward_torque_zero": True,
                                "previous_controller": (
                                    joint_hold_previous_controller
                                ),
                            },
                        )
                        joint_hold_first_write_pending = False
                        joint_hold_first_write_reported = True
                    if amp_hold_first_write_pending:
                        send_event(
                            "AMP_HOLD_FIRST_WRITE",
                            {
                                "controller": AMP_ZERO_COMMAND_HOLD_CONTROLLER,
                                "transition_id": amp_hold_transition_id,
                                "writer_reused": True,
                                "host_target_cleared": (
                                    amp_hold_previous_controller
                                    == HOST_GETUP_CONTROLLER
                                ),
                                "previous_controller": (
                                    amp_hold_previous_controller
                                ),
                                "history_reset_from_latest_lowstate": (
                                    amp_hold_history_reset
                                ),
                                "command": [0.0, 0.0, 0.0],
                            },
                        )
                        amp_hold_first_write_pending = False
                        amp_hold_first_write_reported = True
                next_publish = _advance_deadline(next_publish, publish_period, now)
            if now >= next_status:
                status = state_status(state)
                active_policy = policy_registry.for_controller(controller)
                if active_policy is not None:
                    status.update(active_policy.status_fields(now))
                    status["active_policy_id"] = active_policy.policy_id
                elif controller == JOINT_POSE_HOLD_CONTROLLER:
                    assert joint_hold_started_monotonic is not None
                    status.update(
                        {
                            "active_policy_id": "joint_pose_hold",
                            "policy_index": cascade.index,
                            "policy": "measured_joint_pose_hold",
                            "policy_elapsed_s": now
                            - joint_hold_started_monotonic,
                            "measured_joint_target": True,
                            "capture_once": True,
                            "lowstate_capture_age_s": joint_hold_capture_age_s,
                            "previous_controller": (
                                joint_hold_previous_controller
                            ),
                        }
                    )
                else:
                    assert amp_started_monotonic is not None
                    status.update(
                        {
                            "active_policy_id": "amp",
                            "policy_index": None,
                            "policy": (
                                "amp_walk_run_getup"
                                if controller == AMP_GETUP_CONTROLLER
                                else "amp_zero_command_hold"
                            ),
                            "policy_elapsed_s": now - amp_started_monotonic,
                            "command": [0.0, 0.0, 0.0],
                        }
                    )
                status.update(
                    {
                        "crc_backend": getattr(dds, "crc_backend", "unknown"),
                        "controller": controller,
                        "amp_hold_first_write": amp_hold_first_write_reported,
                        "joint_hold_first_write": (
                            joint_hold_first_write_reported
                        ),
                    }
                )
                send_event("STATUS", status)
                next_status = _advance_deadline(next_status, status_period, now)
        return 0
    finally:
        handoff.close_writer()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model", "--prone-v1-model", dest="model", required=True, type=Path
    )
    parser.add_argument(
        "--fallback-model",
        "--prone-v2-model",
        dest="fallback_models",
        action="append",
        default=[],
        type=Path,
    )
    parser.add_argument("--interface", required=True)
    parser.add_argument("--control-socket", required=True, type=Path)
    # HoST's official interactive/play deployment uses 0.30.  The 0.25 value
    # is its conservative evaluation setting and repeatedly reached, then
    # lost, the upright apex in the live DDS/SONIC chain.
    parser.add_argument("--action-rescale", type=float, default=0.30)
    parser.add_argument("--action-clip", type=float, default=100.0)
    parser.add_argument("--fallback-after-seconds", type=float, default=8.0)
    parser.add_argument(
        "--initial-controller",
        choices=("host", "amp", "kungfu"),
        default=os.environ.get(
            "MATRIX_PHYSICAL_RECOVERY_INITIAL_CONTROLLER", "host"
        ),
        help="physical get-up controller used immediately after GO",
    )
    parser.add_argument("--kungfu-model", type=Path)
    parser.add_argument("--kungfu-motion", type=Path)
    parser.add_argument(
        "--kungfu-model-sha256",
        default=os.environ.get("MATRIX_KUNGFU_RECOVERY_MODEL_SHA256", ""),
    )
    parser.add_argument(
        "--kungfu-model-data-sha256",
        default=os.environ.get(
            "MATRIX_KUNGFU_RECOVERY_MODEL_DATA_SHA256", ""
        ),
    )
    parser.add_argument(
        "--kungfu-motion-sha256",
        default=os.environ.get("MATRIX_KUNGFU_RECOVERY_MOTION_SHA256", ""),
    )
    parser.add_argument(
        "--kungfu-reference-frame",
        type=int,
        default=int(
            os.environ.get(
                "MATRIX_KUNGFU_RECOVERY_REFERENCE_FRAME",
                str(KUNGFU_DEFAULT_REFERENCE_FRAME),
            )
        ),
    )
    parser.add_argument("--kungfu-gain-scale", type=float, default=1.0)
    parser.add_argument("--amp-hold-config", type=Path)
    parser.add_argument("--amp-hold-model", type=Path)
    parser.add_argument(
        "--amp-hold-config-sha256",
        default=os.environ.get("MATRIX_PHYSICAL_RECOVERY_AMP_CONFIG_SHA256", ""),
    )
    parser.add_argument(
        "--amp-hold-model-sha256",
        default=os.environ.get("MATRIX_PHYSICAL_RECOVERY_AMP_MODEL_SHA256", ""),
    )
    parser.add_argument("--publish-hz", type=float, default=500.0)
    parser.add_argument("--status-hz", type=float, default=5.0)
    parser.add_argument("--lowstate-timeout-seconds", type=float, default=0.25)
    parser.add_argument(
        "--execution-provider",
        choices=("cuda", "cpu"),
        default=os.environ.get(
            "MATRIX_PHYSICAL_RECOVERY_EXECUTION_PROVIDER", "cpu"
        ),
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    config = HostControlConfig.create(
        action_rescale=args.action_rescale,
        action_clip=args.action_clip,
    )
    model_paths = [args.model.resolve()]
    model_paths.extend(path.resolve() for path in args.fallback_models)
    expected_primary_sha256 = os.environ.get(
        "MATRIX_PHYSICAL_RECOVERY_MODEL_SHA256", ""
    ).strip().lower()
    if expected_primary_sha256:
        if len(expected_primary_sha256) != 64 or any(
            char not in "0123456789abcdef" for char in expected_primary_sha256
        ):
            raise SystemExit("invalid MATRIX_PHYSICAL_RECOVERY_MODEL_SHA256")
        actual_sha256 = file_sha256(model_paths[0])
        if actual_sha256 != expected_primary_sha256:
            raise SystemExit(
                "physical recovery model SHA256 mismatch: "
                f"expected={expected_primary_sha256} actual={actual_sha256}"
            )
    # All models are loaded and shape-checked before READY_NO_WRITER.
    runners = [
        HostOnnxRunner(
            path,
            execution_provider=args.execution_provider,
        )
        for path in model_paths
    ]
    cascade = HostPolicyCascade(
        config=config,
        runners=runners,
        fallback_after_s=args.fallback_after_seconds,
    )
    amp_hold_arguments = (
        args.amp_hold_config,
        args.amp_hold_model,
        args.amp_hold_config_sha256,
        args.amp_hold_model_sha256,
    )
    if any(bool(value) for value in amp_hold_arguments) and not all(
        bool(value) for value in amp_hold_arguments
    ):
        raise SystemExit(
            "AMP hold requires --amp-hold-config, --amp-hold-model, "
            "--amp-hold-config-sha256, and --amp-hold-model-sha256"
        )
    amp_hold_policy = None
    if all(bool(value) for value in amp_hold_arguments):
        try:
            amp_hold_policy = load_amp_hold_policy(
                config_path=args.amp_hold_config,
                model_path=args.amp_hold_model,
                config_sha256=args.amp_hold_config_sha256,
                model_sha256=args.amp_hold_model_sha256,
                execution_provider=args.execution_provider,
            )
        except (OSError, ValueError) as exc:
            raise SystemExit(f"invalid AMP hold artifacts: {exc}") from exc
    if args.initial_controller == "amp" and amp_hold_policy is None:
        raise SystemExit("AMP-first recovery requires valid AMP artifacts")
    kungfu_policy = None
    kungfu_arguments = (
        args.kungfu_model,
        args.kungfu_motion,
        args.kungfu_model_sha256,
        args.kungfu_model_data_sha256,
        args.kungfu_motion_sha256,
    )
    if any(bool(value) for value in kungfu_arguments) and not all(
        bool(value) for value in kungfu_arguments
    ):
        raise SystemExit(
            "KungFu recovery requires --kungfu-model, --kungfu-motion, and "
            "all three KungFu SHA256 values"
        )
    if all(bool(value) for value in kungfu_arguments):
        try:
            kungfu_policy = load_kungfu_policy(
                model_path=args.kungfu_model,
                motion_path=args.kungfu_motion,
                model_sha256=args.kungfu_model_sha256,
                model_data_sha256=args.kungfu_model_data_sha256,
                motion_sha256=args.kungfu_motion_sha256,
                reference_frame=args.kungfu_reference_frame,
                gain_scale=args.kungfu_gain_scale,
                execution_provider=args.execution_provider,
            )
        except (OSError, ValueError) as exc:
            raise SystemExit(f"invalid KungFu recovery artifacts: {exc}") from exc
    if args.initial_controller == "kungfu" and kungfu_policy is None:
        raise SystemExit("KungFu-first recovery requires valid KungFu artifacts")
    state_store = LatestLowState()
    dds = UnitreeDdsRuntime(interface=args.interface, state_store=state_store)
    control = _connect_control(args.control_socket)
    resident_policies: list[dict[str, Any]] = [
        {
            "name": f"host:{runner.label}",
            "execution_provider": runner.execution_provider,
            "warmed": True,
        }
        for runner in runners
    ]
    if amp_hold_policy is not None:
        resident_policies.append(
            {
                "name": "amp:walk_run_getup",
                "execution_provider": getattr(
                    amp_hold_policy.runner, "execution_provider", None
                ),
                "warmed": True,
            }
        )
    if kungfu_policy is not None:
        resident_policies.append(
            {
                "name": "kungfu:1307_recovery",
                "execution_provider": getattr(
                    kungfu_policy.runner, "execution_provider", None
                ),
                "warmed": True,
            }
        )
    try:
        return run_worker(
            cascade=cascade,
            amp_hold_policy=amp_hold_policy,
            kungfu_policy=kungfu_policy,
            dds=dds,
            state_store=state_store,
            control=control,
            publish_hz=args.publish_hz,
            lowstate_timeout_s=args.lowstate_timeout_seconds,
            status_hz=args.status_hz,
            initial_controller=args.initial_controller,
            resident_policies=resident_policies,
            execution_provider=args.execution_provider,
        )
    except (BrokenPipeError, ConnectionResetError):
        return 3
    finally:
        control.close()


if __name__ == "__main__":
    sys.exit(main())
