#!/usr/bin/env python3
"""Validate the live SONIC -> HoST -> SONIC hand-off with a physical fall.

This harness wraps the production runtime without exposing a simulator reset
or pose-writing API.  It adds a short MuJoCo body force immediately before
``mj_step`` (after SONIC's elastic-band update), then supplies authenticated
neutral/W keyboard snapshots through the normal game-input socket.  Passing
therefore proves that a new SONIC generation can resume ordinary input after
the temporary HoST LowCmd writer physically gets the same robot state upright.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import socket
import sys
import threading
import time
from typing import Any, Sequence


_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

import matrix_game_control


_FORCE_DIRECTIONS = {
    "forward": (1.0, 0.0, 0.0),
    "backward": (-1.0, 0.0, 0.0),
    "left": (0.0, 1.0, 0.0),
    "right": (0.0, -1.0, 0.0),
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def _runtime_value(arguments: Sequence[str], option: str) -> str:
    matches = [index for index, value in enumerate(arguments) if value == option]
    if len(matches) != 1 or matches[0] + 1 >= len(arguments):
        raise ValueError(f"runtime arguments require exactly one {option}")
    return arguments[matches[0] + 1]


def _has_runtime_flag(arguments: Sequence[str], option: str) -> bool:
    return sum(value == option for value in arguments) == 1


def _runtime_deadline_seconds(arguments: Sequence[str]) -> float:
    """Require a finite outer deadline so validator failures cannot hang."""

    raw_value = _runtime_value(arguments, "--max-seconds")
    try:
        value = float(raw_value)
    except ValueError as exc:
        raise ValueError("runtime --max-seconds must be a number") from exc
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError("runtime --max-seconds must be finite and positive")
    return value


class PhysicalKnockdownProbe:
    """Inject one force pulse at the last possible pre-integration boundary."""

    def __init__(
        self,
        *,
        direction: str,
        force_newtons: float,
        duration_s: float,
        ready_hold_s: float,
    ) -> None:
        if direction not in _FORCE_DIRECTIONS:
            raise ValueError(f"unknown force direction: {direction}")
        for name, value in (
            ("force_newtons", force_newtons),
            ("duration_s", duration_s),
            ("ready_hold_s", ready_hold_s),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        self.direction = direction
        self.force_newtons = float(force_newtons)
        self.duration_s = float(duration_s)
        self.ready_hold_s = float(ready_hold_s)
        self.phase = "waiting_for_sonic_and_band_release"
        self.ready_ticks = 0
        self.ready_ticks_required: int | None = None
        self.force_ticks_required: int | None = None
        self.force_ticks_requested = 0
        self.force_ticks_applied = 0
        self.force_started_sim_time_s: float | None = None
        self.force_finished_sim_time_s: float | None = None
        self.minimum_root_z_m: float | None = None
        self.minimum_root_up_z: float | None = None
        self.fall_flag_seen = False
        self.pelvis_body_id: int | None = None
        self.pelvis_body_name: str | None = None
        self.sim_dt_s: float | None = None
        self._apply_this_step = False
        self._target_model: Any = None
        self._target_data: Any = None
        self._mujoco: Any = None
        self._original_mj_step: Any = None
        self._lock = threading.Lock()
        # These counters cover only direct operations performed by this
        # validation probe.  They are intentionally not described as a global
        # interception of every possible simulator consumer.  The production
        # snapshot's reset_count remains the independent runtime reset authority.
        self.direct_qpos_writes = 0
        self.direct_qvel_writes = 0
        self.direct_reset_calls = 0
        self.direct_reload_calls = 0
        self.direct_teleports = 0

    @staticmethod
    def _root_up_z(qpos: Sequence[float]) -> float:
        _w, x, y, _z = (float(value) for value in qpos[3:7])
        return 1.0 - 2.0 * (x * x + y * y)

    def attach(self, simulator: Any, mujoco_module: Any, numpy_module: Any) -> None:
        model = simulator.sim_env.mj_model
        data = simulator.sim_env.mj_data
        pelvis_id = -1
        pelvis_name = None
        for name in ("pelvis", "base"):
            candidate = int(
                mujoco_module.mj_name2id(
                    model, mujoco_module.mjtObj.mjOBJ_BODY, name
                )
            )
            if candidate > 0:
                pelvis_id = candidate
                pelvis_name = name
                break
        if pelvis_id <= 0:
            raise RuntimeError("MuJoCo model has no pelvis/base body")

        sim_dt = float(simulator.sim_dt)
        self.sim_dt_s = sim_dt
        self.ready_ticks_required = max(1, int(math.ceil(self.ready_hold_s / sim_dt)))
        self.force_ticks_required = max(1, int(round(self.duration_s / sim_dt)))
        self.pelvis_body_id = pelvis_id
        self.pelvis_body_name = pelvis_name
        self._target_model = model
        self._target_data = data
        self._mujoco = mujoco_module
        self._original_mj_step = mujoco_module.mj_step
        direction = numpy_module.asarray(
            _FORCE_DIRECTIONS[self.direction], dtype=numpy_module.float64
        )
        force = direction * self.force_newtons
        zero_torque = numpy_module.zeros(3, dtype=numpy_module.float64)

        def wrapped_mj_step(candidate_model: Any, candidate_data: Any, *args: Any, **kwargs: Any) -> Any:
            if (
                candidate_model is not self._target_model
                or candidate_data is not self._target_data
                or not self._apply_this_step
            ):
                return self._original_mj_step(
                    candidate_model, candidate_data, *args, **kwargs
                )
            # qfrc_applied is the documented external-force input. Preserve any
            # pre-existing generalized force and restore it after integration;
            # qpos/qvel and the scene are never written or reset.
            previous_qfrc = candidate_data.qfrc_applied.copy()
            point = candidate_data.xpos[pelvis_id].copy()
            try:
                mujoco_module.mj_applyFT(
                    candidate_model,
                    candidate_data,
                    force,
                    zero_torque,
                    point,
                    pelvis_id,
                    candidate_data.qfrc_applied,
                )
                with self._lock:
                    self.force_ticks_applied += 1
                return self._original_mj_step(
                    candidate_model, candidate_data, *args, **kwargs
                )
            finally:
                candidate_data.qfrc_applied[:] = previous_qfrc

        mujoco_module.mj_step = wrapped_mj_step
        original_step_once = simulator.step_once

        def wrapped_step_once(*, rate_limit: bool = False) -> Any:
            before = simulator.get_state_snapshot()
            with self._lock:
                if self.phase == "waiting_for_sonic_and_band_release":
                    ready = bool(before.low_cmd_fresh) and float(
                        before.elastic_band_scale
                    ) <= 1e-6
                    self.ready_ticks = self.ready_ticks + 1 if ready else 0
                    assert self.ready_ticks_required is not None
                    if self.ready_ticks >= self.ready_ticks_required:
                        self.phase = "applying_physical_force"
                        self.force_started_sim_time_s = float(before.sim_time)
                assert self.force_ticks_required is not None
                self._apply_this_step = (
                    self.phase == "applying_physical_force"
                    and self.force_ticks_requested < self.force_ticks_required
                )
                if self._apply_this_step:
                    self.force_ticks_requested += 1
            try:
                snapshot = original_step_once(rate_limit=rate_limit)
            finally:
                with self._lock:
                    self._apply_this_step = False
            with self._lock:
                if self.force_ticks_requested > 0:
                    root_z = float(snapshot.qpos[2])
                    root_up_z = self._root_up_z(snapshot.qpos)
                    self.minimum_root_z_m = (
                        root_z
                        if self.minimum_root_z_m is None
                        else min(self.minimum_root_z_m, root_z)
                    )
                    self.minimum_root_up_z = (
                        root_up_z
                        if self.minimum_root_up_z is None
                        else min(self.minimum_root_up_z, root_up_z)
                    )
                    self.fall_flag_seen |= bool(snapshot.fall_detected)
                if (
                    self.phase == "applying_physical_force"
                    and self.force_ticks_requested >= self.force_ticks_required
                ):
                    self.phase = "observing_physical_recovery"
                    self.force_finished_sim_time_s = float(snapshot.sim_time)
            return snapshot

        simulator.step_once = wrapped_step_once

    def close(self) -> None:
        if self._mujoco is not None and self._original_mj_step is not None:
            self._mujoco.mj_step = self._original_mj_step

    def telemetry(self) -> dict[str, Any]:
        with self._lock:
            expected_impulse = (
                self.force_newtons * self.force_ticks_applied * self.sim_dt_s
                if self.sim_dt_s is not None
                else 0.0
            )
            return {
                "direction": self.direction,
                "force_newtons": self.force_newtons,
                "duration_requested_s": self.duration_s,
                "ready_hold_s": self.ready_hold_s,
                "phase": self.phase,
                "sim_dt_s": self.sim_dt_s,
                "ready_ticks": self.ready_ticks,
                "ready_ticks_required": self.ready_ticks_required,
                "force_ticks_required": self.force_ticks_required,
                "force_ticks_requested": self.force_ticks_requested,
                "force_ticks_applied": self.force_ticks_applied,
                "physical_impulse_n_s": expected_impulse,
                "force_started_sim_time_s": self.force_started_sim_time_s,
                "force_finished_sim_time_s": self.force_finished_sim_time_s,
                "minimum_root_z_m": self.minimum_root_z_m,
                "minimum_root_up_z": self.minimum_root_up_z,
                "fall_flag_seen": self.fall_flag_seen,
                "pelvis_body_id": self.pelvis_body_id,
                "pelvis_body_name": self.pelvis_body_name,
                "injection_boundary": "after_elastic_band_before_mj_step",
                "mutation_counter_scope": (
                    "physical_knockdown_probe_direct_operations_only"
                ),
                "simulator_state_mutation_by_probe": any(
                    (
                        self.direct_qpos_writes,
                        self.direct_qvel_writes,
                        self.direct_reset_calls,
                        self.direct_reload_calls,
                        self.direct_teleports,
                    )
                ),
                "qpos_writes": self.direct_qpos_writes,
                "qvel_writes": self.direct_qvel_writes,
                "reset_calls": self.direct_reset_calls,
                "reload_calls": self.direct_reload_calls,
                "teleports": self.direct_teleports,
            }


class GameInputPeer:
    """Feed neutral input, then prove W works after recovery completes."""

    def __init__(
        self,
        *,
        socket_path: Path,
        status_path: Path,
        move_seconds: float,
        applied_hold_seconds: float,
        neutral_after_seconds: float,
        completion_event: threading.Event,
    ) -> None:
        self.socket_path = socket_path
        self.status_path = status_path
        self.move_seconds = float(move_seconds)
        self.applied_hold_seconds = float(applied_hold_seconds)
        self.neutral_after_seconds = float(neutral_after_seconds)
        self.completion_event = completion_event
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.connected = False
        self.packets_sent = 0
        self.neutral_packets_sent = 0
        self.movement_packets_sent = 0
        self.recovery_observed = False
        self.post_recovery_neutral_packets_sent = 0
        self.neutral_handshake_complete = False
        self.neutral_sequence_barrier: int | None = None
        self.last_neutral_ack_sequence: int | None = None
        self.neutral_rearm_count = 0
        self.last_game_input_stop_reason: str | None = None
        self.move_started_monotonic_s: float | None = None
        self.move_finished_monotonic_s: float | None = None
        self.move_applied_monotonic_s: float | None = None
        self.moving_command_frames_at_resume: int | None = None
        self.stop_requested_after_success = False
        self.stop_requested_after_attempt = False
        self.last_error: str | None = None
        self._sequence = max(1, time.monotonic_ns())
        self._client: socket.socket | None = None

    def start(self) -> None:
        self.thread = threading.Thread(
            target=self._run,
            name="matrix-recovery-e2e-game-input",
            daemon=True,
        )
        self.thread.start()

    def close(self) -> None:
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=2.0)
        if self._client is not None:
            self._client.close()
            self._client = None

    def _status(self) -> dict[str, Any] | None:
        try:
            payload = json.loads(self.status_path.read_text(encoding="utf-8"))
        except (FileNotFoundError, json.JSONDecodeError, OSError):
            return None
        return payload if isinstance(payload, dict) else None

    def _connect(self) -> bool:
        client = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        try:
            client.connect(os.fspath(self.socket_path))
        except OSError:
            client.close()
            return False
        self._client = client
        self.connected = True
        return True

    @staticmethod
    def _moving_command_frames(status: dict[str, Any] | None) -> int:
        if not isinstance(status, dict):
            return 0
        game_input = status.get("game_input", {})
        if not isinstance(game_input, dict):
            return 0
        frames = game_input.get("moving_command_frames", 0)
        if isinstance(frames, bool) or not isinstance(frames, int) or frames < 0:
            return 0
        return frames

    @staticmethod
    def _game_input_neutral_acknowledged(
        status: dict[str, Any] | None,
        *,
        sequence_barrier: int,
    ) -> bool:
        if not isinstance(status, dict):
            return False
        game_input = status.get("game_input")
        if not isinstance(game_input, dict):
            return False
        required_fields = {
            "mode",
            "safe_stop",
            "stop_reason",
            "locomotion_mode",
            "speed_mps",
            "sequence",
        }
        if not required_fields.issubset(game_input):
            return False
        locomotion_mode = game_input["locomotion_mode"]
        speed_mps = game_input["speed_mps"]
        sequence = game_input["sequence"]
        return (
            game_input["mode"] == "idle"
            and game_input["safe_stop"] is False
            and game_input["stop_reason"] is None
            and type(locomotion_mode) is int
            and locomotion_mode == 0
            and type(sequence) is int
            and sequence >= sequence_barrier
            and not isinstance(speed_mps, bool)
            and isinstance(speed_mps, (int, float))
            and math.isfinite(float(speed_mps))
            and float(speed_mps) == 0.0
        )

    @staticmethod
    def _game_input_requires_neutral_rearm(
        status: dict[str, Any] | None,
    ) -> bool:
        if not isinstance(status, dict):
            return False
        game_input = status.get("game_input")
        if not isinstance(game_input, dict):
            return False
        return (
            game_input.get("safe_stop") is True
            or game_input.get("stop_reason") == "awaiting_neutral"
        )

    def _movement_requested(
        self,
        status: dict[str, Any] | None,
        *,
        now: float,
    ) -> bool:
        recovery = (
            status.get("game_fall_recovery", {})
            if status is not None
            else {}
        )
        recovery_mode = (
            recovery.get("mode") if isinstance(recovery, dict) else None
        )
        recovery_state = (
            recovery.get("state") if isinstance(recovery, dict) else None
        )
        replacement_gate = (
            recovery.get("replacement_sonic_writer_gate", {})
            if isinstance(recovery, dict)
            else {}
        )
        physical_full_control = (
            isinstance(replacement_gate, dict)
            and replacement_gate.get("reentry_policy_full_control") is True
        )
        resumed = (
            recovery_mode == "physical"
            and recovery_state == "GAME_SONIC"
            and physical_full_control
        ) or (
            recovery_mode == "sonic" and recovery_state == "monitoring"
        )
        if (
            isinstance(recovery, dict)
            and int(recovery.get("recoveries", 0)) >= 1
            and resumed
        ):
            self.recovery_observed = True
            game_input = status.get("game_input") if status is not None else None
            if isinstance(game_input, dict) and "stop_reason" in game_input:
                stop_reason = game_input["stop_reason"]
                if stop_reason is None or isinstance(stop_reason, str):
                    self.last_game_input_stop_reason = stop_reason
            if (
                not self.neutral_handshake_complete
                and self.neutral_sequence_barrier is None
            ):
                # This is the sequence of the neutral packet _run sends after
                # this decision.  Requiring status to echo it prevents a stale
                # pre-recovery idle snapshot from being mistaken for the ACK.
                self.neutral_sequence_barrier = self._sequence
            if (
                not self.neutral_handshake_complete
                and self.neutral_sequence_barrier is not None
                and self._game_input_neutral_acknowledged(
                    status,
                    sequence_barrier=self.neutral_sequence_barrier,
                )
            ):
                self.neutral_handshake_complete = True
                self.last_neutral_ack_sequence = game_input["sequence"]
                self.neutral_sequence_barrier = None
                if self.move_started_monotonic_s is None:
                    self.move_started_monotonic_s = now
                    self.moving_command_frames_at_resume = (
                        self._moving_command_frames(status)
                    )

        moving_frames = self._moving_command_frames(status)
        if (
            self.move_started_monotonic_s is not None
            and self.moving_command_frames_at_resume is not None
            and moving_frames > self.moving_command_frames_at_resume
            and self.move_applied_monotonic_s is None
        ):
            self.move_applied_monotonic_s = now
        if (
            self.move_started_monotonic_s is not None
            and self.move_applied_monotonic_s is None
            and self.neutral_handshake_complete
            and self._game_input_requires_neutral_rearm(status)
        ):
            self.neutral_handshake_complete = False
            self.neutral_sequence_barrier = self._sequence
            self.neutral_rearm_count += 1
        waiting_for_applied = (
            self.move_started_monotonic_s is not None
            and self.move_applied_monotonic_s is None
            and self.neutral_handshake_complete
            and now - self.move_started_monotonic_s < self.move_seconds
        )
        proving_applied_motion = (
            self.move_applied_monotonic_s is not None
            and now - self.move_applied_monotonic_s < self.applied_hold_seconds
        )
        moving = waiting_for_applied or proving_applied_motion
        move_attempt_timed_out = (
            self.move_started_monotonic_s is not None
            and self.move_applied_monotonic_s is None
            and now - self.move_started_monotonic_s >= self.move_seconds
        )
        applied_hold_complete = (
            self.move_applied_monotonic_s is not None
            and not proving_applied_motion
        )
        if (
            (move_attempt_timed_out or applied_hold_complete)
            and self.move_finished_monotonic_s is None
        ):
            self.move_finished_monotonic_s = now
        return moving

    def _record_sent(self, *, moving: bool) -> None:
        self.packets_sent += 1
        if moving:
            self.movement_packets_sent += 1
        else:
            self.neutral_packets_sent += 1
            if (
                self.recovery_observed
                and not self.neutral_handshake_complete
            ):
                self.post_recovery_neutral_packets_sent += 1

    def _send(self, *, moving: bool) -> None:
        assert self._client is not None
        snapshot = matrix_game_control.InputSnapshot.from_mapping(
            {
                "protocol": matrix_game_control.PROTOCOL_NAME,
                "sequence": self._sequence,
                "timestamp_monotonic_s": time.monotonic(),
                "focused": True,
                "camera_yaw_rad": 0.0,
                "keys": {
                    "w": moving,
                    "a": False,
                    "s": False,
                    "d": False,
                    "q": False,
                    "e": False,
                    "v": False,
                    "ctrl": False,
                    "shift": False,
                },
                "move_stick": {"right": 0.0, "forward": 0.0},
            }
        )
        self._client.sendall(matrix_game_control.encode_input_packet(snapshot))
        self._sequence += 1
        self._record_sent(moving=moving)

    def _complete_attempt(self) -> None:
        self.stop_requested_after_attempt = True
        self.stop_requested_after_success = (
            self.move_applied_monotonic_s is not None
        )
        self.completion_event.set()

    def _run(self) -> None:
        try:
            while not self.stop_event.is_set() and self._client is None:
                if not self._connect():
                    self.stop_event.wait(0.02)
            while not self.stop_event.is_set() and self._client is not None:
                now = time.monotonic()
                status = self._status()
                moving = self._movement_requested(status, now=now)
                self._send(moving=moving)
                if (
                    self.move_finished_monotonic_s is not None
                    and now - self.move_finished_monotonic_s
                    >= self.neutral_after_seconds
                ):
                    self._complete_attempt()
                    return
                self.stop_event.wait(0.02)
        except (BrokenPipeError, ConnectionError, OSError, ValueError) as exc:
            if not self.stop_event.is_set():
                self.last_error = f"{type(exc).__name__}: {exc}"
                # A failed input attempt is terminal evidence, not a reason to
                # leave the in-process runtime waiting for its outer deadline.
                self._complete_attempt()

    def telemetry(self) -> dict[str, Any]:
        return {
            "connected": self.connected,
            "packets_sent": self.packets_sent,
            "neutral_packets_sent": self.neutral_packets_sent,
            "movement_packets_sent": self.movement_packets_sent,
            "recovery_observed": self.recovery_observed,
            "post_recovery_neutral_packets_sent": (
                self.post_recovery_neutral_packets_sent
            ),
            "neutral_handshake_complete": self.neutral_handshake_complete,
            "neutral_sequence_barrier": self.neutral_sequence_barrier,
            "last_neutral_ack_sequence": self.last_neutral_ack_sequence,
            "neutral_rearm_count": self.neutral_rearm_count,
            "last_game_input_stop_reason": self.last_game_input_stop_reason,
            "move_started": self.move_started_monotonic_s is not None,
            "move_finished": self.move_finished_monotonic_s is not None,
            "move_applied": self.move_applied_monotonic_s is not None,
            "moving_command_frames_at_resume": (
                self.moving_command_frames_at_resume
            ),
            "applied_hold_seconds": self.applied_hold_seconds,
            "stop_requested_after_attempt": self.stop_requested_after_attempt,
            "stop_requested_after_success": self.stop_requested_after_success,
            "last_error": self.last_error,
            "socket": str(self.socket_path),
        }


def _evaluate(
    status: dict[str, Any],
    probe: dict[str, Any],
    input_peer: dict[str, Any],
    *,
    runtime_return_code: int | None,
) -> list[str]:
    failures: list[str] = []
    recovery = status.get("game_fall_recovery", {})
    game_input = status.get("game_input", {})
    worker = recovery.get("worker", {}) if isinstance(recovery, dict) else {}
    worker_episode: dict[str, Any] = {}
    expected_worker_episode_id = (
        recovery.get("latest_completed_recovery_worker_episode_id")
        if isinstance(recovery, dict)
        else None
    )
    if isinstance(worker, dict) and type(expected_worker_episode_id) is int:
        completed = worker.get("completed_episodes", [])
        if isinstance(completed, list):
            for candidate in reversed(completed):
                if (
                    isinstance(candidate, dict)
                    and candidate.get("episode_id") == expected_worker_episode_id
                ):
                    worker_episode = candidate
                    break
    sonic_gate = (
        recovery.get("replacement_sonic_writer_gate", {})
        if isinstance(recovery, dict)
        else {}
    )

    def require(condition: bool, reason: str) -> None:
        if not condition:
            failures.append(reason)

    if type(runtime_return_code) is not int:
        failures.append("runtime_return_code_missing_or_invalid")
    elif runtime_return_code != 0:
        failures.append("runtime_return_code_nonzero")
    require(status.get("passed") is True, "runtime_status_not_passed")
    acceptance_failures = status.get("acceptance_failures")
    if not isinstance(acceptance_failures, list):
        failures.append("runtime_acceptance_failures_missing_or_invalid")
    elif acceptance_failures:
        failures.append("runtime_acceptance_failures_present")

    require(
        probe.get("force_ticks_applied") == probe.get("force_ticks_required"),
        "physical_force_not_fully_applied",
    )
    require(
        probe.get("mutation_counter_scope")
        == "physical_knockdown_probe_direct_operations_only",
        "probe_mutation_counter_scope_missing",
    )
    for field in (
        "qpos_writes",
        "qvel_writes",
        "reset_calls",
        "reload_calls",
        "teleports",
    ):
        require(probe.get(field) == 0, f"probe_{field}_nonzero")
    require(
        (
            probe.get("minimum_root_z_m") is not None
            and float(probe["minimum_root_z_m"]) < 0.45
        )
        or (
            probe.get("minimum_root_up_z") is not None
            and float(probe["minimum_root_up_z"]) < 0.5
        ),
        "physical_fall_not_observed",
    )
    require(isinstance(recovery, dict), "recovery_telemetry_missing")
    if isinstance(recovery, dict):
        recovery_mode = recovery.get("mode")
        require(
            recovery_mode in {"physical", "sonic"},
            "unsupported_recovery_mode",
        )
        require(int(recovery.get("episodes", 0)) >= 1, "recovery_episode_missing")
        require(int(recovery.get("recoveries", 0)) >= 1, "recovery_not_completed")
        if recovery_mode == "physical":
            handoff_mode = recovery.get("handoff_mode", "amp")
            require(
                handoff_mode in {"amp", "sonic"},
                "unsupported_physical_handoff_mode",
            )
            require(
                int(recovery.get("deploy_generation", 0)) >= 2,
                "sonic_not_restarted",
            )
            require(recovery.get("state") == "GAME_SONIC", "game_not_resumed")
            require(recovery.get("fail_closed") is False, "recovery_failed_closed")
            require(
                recovery.get("physical_only") is True,
                "physical_only_not_attested",
            )
            require(
                recovery.get("previous_sonic_writer_revoked") is True,
                "previous_sonic_hard_revocation_missing",
            )
            require(
                recovery.get("simulator_state_mutation") is False,
                "simulator_state_mutation_reported",
            )
            require(
                type(expected_worker_episode_id) is int
                and expected_worker_episode_id > 0,
                "completed_recovery_worker_episode_id_missing",
            )
            require(
                worker_episode.get("episode_id") == expected_worker_episode_id,
                "completed_recovery_worker_episode_not_found",
            )
            require(
                worker_episode.get("go_sent") is True,
                "host_writer_go_not_attested",
            )
            require(
                worker_episode.get("first_write") is True,
                "host_first_write_missing",
            )
            if handoff_mode == "amp":
                require(
                    worker_episode.get("amp_hold_first_write") is True,
                    "amp_dynamic_hold_first_write_missing",
                )
            else:
                require(
                    worker_episode.get("amp_hold_sent") is False
                    and worker_episode.get("joint_hold_sent") is False
                    and worker_episode.get("hold_kind") is None,
                    "direct_sonic_dynamic_handoff_not_attested",
                )
            require(
                worker_episode.get("stopped") is True,
                "physical_policy_not_stopped",
            )
            require(
                worker_episode.get("stop_sent") is True,
                "physical_policy_stop_not_attested",
            )
            command_history = worker_episode.get("command_history", [])
            switch_first_writes = worker_episode.get(
                "policy_switch_first_writes", []
            )
            advance_count = (
                sum(
                    1
                    for command in command_history
                    if isinstance(command, dict)
                    and command.get("command") == "ADVANCE_POLICY"
                )
                if isinstance(command_history, list)
                else -1
            )
            require(
                isinstance(switch_first_writes, list)
                and advance_count == len(switch_first_writes),
                "policy_switch_first_write_evidence_mismatch",
            )
            require(
                sonic_gate.get("ready_no_lowcmd_writer") is True,
                "replacement_sonic_writer_free_ready_missing",
            )
            require(
                sonic_gate.get("shadow_ready_no_lowcmd_writer") is True,
                "replacement_sonic_shadow_admission_missing",
            )
            require(
                sonic_gate.get("first_write") is True,
                "replacement_sonic_first_write_event_missing",
            )
            require(
                sonic_gate.get("reentry_alignment_complete") is True,
                "replacement_sonic_activation_freeze_missing",
            )
            require(
                sonic_gate.get("reentry_policy_full_control") is True,
                "replacement_sonic_full_control_missing",
            )
        elif recovery_mode == "sonic":
            require(recovery.get("state") == "monitoring", "game_not_resumed")
            require(recovery.get("policy_command") == "KNEEL_TWO_LEGS_TO_IDLE", "native_sonic_recovery_command_missing")
            require(recovery.get("timed_out") is False, "native_sonic_recovery_timed_out")
    require(status.get("active_lowcmd") is True, "replacement_sonic_lowcmd_not_fresh")
    root_xyz = status.get("root_xyz")
    require(
        isinstance(root_xyz, list)
        and len(root_xyz) >= 3
        and float(root_xyz[2]) >= 0.65,
        "final_robot_not_upright_height",
    )
    require(
        status.get("root_up_z") is not None
        and float(status["root_up_z"]) >= 0.85,
        "final_robot_not_upright_orientation",
    )
    require(
        status.get("current_fall_detected") is False,
        "final_fall_still_detected",
    )
    require(int(status.get("instability_resets", -1)) == 0, "reset_count_changed")
    require(status.get("numerical_error") is None, "runtime_numerical_error")
    require(status.get("failed_child_exit_code") is None, "managed_child_failed")
    require(input_peer.get("recovery_observed") is True, "input_peer_did_not_see_resume")
    require(int(input_peer.get("movement_packets_sent", 0)) > 0, "post_recovery_w_not_sent")
    require(
        isinstance(game_input, dict)
        and int(game_input.get("moving_command_frames", 0)) > 0,
        "post_recovery_w_not_applied",
    )
    require(input_peer.get("last_error") is None, "input_peer_error")
    return failures


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence", required=True, type=Path)
    parser.add_argument(
        "--knockdown-direction", choices=tuple(_FORCE_DIRECTIONS), default="forward"
    )
    parser.add_argument("--knockdown-force-newtons", type=float, default=3400.0)
    parser.add_argument("--knockdown-seconds", type=float, default=0.04)
    parser.add_argument("--ready-hold-seconds", type=float, default=0.5)
    parser.add_argument(
        "--post-recovery-move-seconds",
        type=float,
        default=12.0,
        help=(
            "maximum W hold while waiting for a published locomotion frame; "
            "W is released early after the first frame"
        ),
    )
    parser.add_argument(
        "--post-recovery-applied-hold-seconds",
        type=float,
        default=3.0,
        help=(
            "continue W after the first published locomotion frame so live "
            "shadow admission and the physical blend can complete"
        ),
    )
    parser.add_argument("--post-move-neutral-seconds", type=float, default=0.75)
    parser.add_argument("runtime_args", nargs=argparse.REMAINDER)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    runtime_args = list(args.runtime_args)
    if runtime_args and runtime_args[0] == "--":
        runtime_args.pop(0)
    if not runtime_args:
        raise SystemExit("runtime arguments are required after --")
    try:
        _runtime_deadline_seconds(runtime_args)
    except ValueError as exc:
        raise SystemExit(f"E2E recovery validation requires {exc}") from exc
    sonic_root = Path(_runtime_value(runtime_args, "--sonic-root")).resolve()
    status_path = Path(_runtime_value(runtime_args, "--status-file")).resolve()
    input_socket = Path(_runtime_value(runtime_args, "--game-input-socket"))
    model_path = Path(_runtime_value(runtime_args, "--model")).resolve()
    recovery_mode = _runtime_value(runtime_args, "--game-fall-recovery")
    recovery_model_path = (
        Path(_runtime_value(runtime_args, "--physical-recovery-model")).resolve()
        if recovery_mode == "physical"
        else None
    )
    recovery_worker_path = (
        Path(_runtime_value(runtime_args, "--physical-recovery-worker")).resolve()
        if recovery_mode == "physical"
        else None
    )
    if _runtime_value(runtime_args, "--control-source") != "game":
        raise SystemExit("E2E recovery validation requires --control-source game")
    if recovery_mode not in {"physical", "sonic"}:
        raise SystemExit("E2E recovery validation requires physical or sonic recovery")
    if not _has_runtime_flag(runtime_args, "--no-game-input-provider"):
        raise SystemExit("E2E validator owns input and requires --no-game-input-provider")
    if str(sonic_root) not in sys.path:
        sys.path.insert(0, str(sonic_root))

    status_path.unlink(missing_ok=True)
    probe = PhysicalKnockdownProbe(
        direction=args.knockdown_direction,
        force_newtons=args.knockdown_force_newtons,
        duration_s=args.knockdown_seconds,
        ready_hold_s=args.ready_hold_seconds,
    )
    completion_event = threading.Event()
    input_peer = GameInputPeer(
        socket_path=input_socket,
        status_path=status_path,
        move_seconds=args.post_recovery_move_seconds,
        applied_hold_seconds=args.post_recovery_applied_hold_seconds,
        neutral_after_seconds=args.post_move_neutral_seconds,
        completion_event=completion_event,
    )
    runtime_return_code: int | None = None
    runtime_exception: str | None = None
    status: dict[str, Any] = {}

    try:
        import mujoco
        import numpy as np
        from gear_sonic.scripts import run_sim_loop
        import run_matrix_sonic

        original_create_simulator = run_sim_loop.create_simulator

        def create_instrumented_simulator(config: Any) -> Any:
            simulator = original_create_simulator(config)
            probe.attach(simulator, mujoco, np)
            return simulator

        run_sim_loop.create_simulator = create_instrumented_simulator
        input_peer.start()
        previous_argv = sys.argv
        sys.argv = [str(_SCRIPT_DIR / "run_matrix_sonic.py"), *runtime_args]
        try:
            runtime_return_code = int(
                run_matrix_sonic.main(completion_event=completion_event)
            )
        except SystemExit as exc:
            runtime_return_code = int(exc.code) if isinstance(exc.code, int) else 2
            runtime_exception = None if exc.code in (None, 0) else str(exc.code)
        except Exception as exc:  # Preserve a machine-readable failed artifact.
            runtime_return_code = 2
            runtime_exception = f"{type(exc).__name__}: {exc}"
        finally:
            sys.argv = previous_argv
            run_sim_loop.create_simulator = original_create_simulator
    finally:
        input_peer.close()
        probe.close()
        try:
            decoded = json.loads(status_path.read_text(encoding="utf-8"))
        except (FileNotFoundError, json.JSONDecodeError, OSError):
            decoded = {}
        if isinstance(decoded, dict):
            status = decoded

    probe_status = probe.telemetry()
    input_status = input_peer.telemetry()
    failures = _evaluate(
        status,
        probe_status,
        input_status,
        runtime_return_code=runtime_return_code,
    )
    if runtime_exception is not None:
        failures.append("runtime_exception")
    evidence = {
        "schema": "matrix.sonic_physical_recovery_e2e.v1",
        "passed": not failures,
        "failures": failures,
        "constraints": {
            "physical_knockdown_only": True,
            "joint_pd_getup_only": True,
            "mutation_counter_scope": probe_status.get("mutation_counter_scope"),
            "global_qpos_qvel_write_observability": False,
            "authoritative_runtime_reset_counter_required": True,
            "qpos_writes": probe_status.get("qpos_writes"),
            "qvel_writes": probe_status.get("qvel_writes"),
            "reset_calls": probe_status.get("reset_calls"),
            "respawns": 0,
            "reloads": probe_status.get("reload_calls"),
            "teleports": probe_status.get("teleports"),
        },
        "runtime_return_code": runtime_return_code,
        "runtime_exception": runtime_exception,
        "runtime_status_file": str(status_path),
        "runtime_status_sha256": _sha256(status_path) if status_path.is_file() else None,
        "runtime_status": status,
        "inputs": {
            "model": str(model_path),
            "model_sha256": _sha256(model_path) if model_path.is_file() else None,
            "recovery_model": (
                str(recovery_model_path) if recovery_model_path is not None else None
            ),
            "recovery_model_sha256": (
                _sha256(recovery_model_path)
                if recovery_model_path is not None and recovery_model_path.is_file()
                else None
            ),
            "recovery_worker": (
                str(recovery_worker_path) if recovery_worker_path is not None else None
            ),
            "recovery_worker_sha256": (
                _sha256(recovery_worker_path)
                if recovery_worker_path is not None and recovery_worker_path.is_file()
                else None
            ),
            "runtime": str(_SCRIPT_DIR / "run_matrix_sonic.py"),
            "runtime_sha256": _sha256(_SCRIPT_DIR / "run_matrix_sonic.py"),
        },
        "knockdown_probe": probe_status,
        "game_input_peer": input_status,
        "runtime_arguments": runtime_args,
    }
    _atomic_json(args.evidence.resolve(), evidence)
    print(json.dumps(evidence, indent=2, sort_keys=True))
    return 0 if evidence["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
