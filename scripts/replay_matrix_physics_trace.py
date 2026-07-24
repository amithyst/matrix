#!/usr/bin/env python3
"""Replay a validated TwinBot MuJoCo trace into Matrix's loopback UE bridge."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import tempfile
import time
from typing import Any, Sequence


TRACE_SCHEMA = "twinbot.physics_trace.mujoco.v0"
STATUS_SCHEMA = "matrix.physics_trace_replay.status.v1"
SUMMARY_SCHEMA = "matrix.physics_trace_replay.summary.v1"
PHYSICS_EXECUTION = "offline_mujoco_persistent_world"
RENDER_MODE = "matrix_ue_trace_replay"
EXPECTED_DIMS = (57, 55, 43)
REPLAY_FPS = 25.0
RENDER_ADDRESS = ("127.0.0.1", 9999)
DEFAULT_MAX_TRACE_BYTES = 1024 * 1024 * 1024
REQUIRED_TRANSITION_SUBSEQUENCE = (
    "world_ready",
    "dock_with_pregrasp",
    "assisted_stance_settle",
    "pick_place_contact_stabilized",
    "contact_validated",
    "grasp_stabilizer_active",
    "cube_supported_on_worktop",
    "grasp_stabilizer_released",
)
_TIME_AND_SIZE = struct.Struct("<dI")
_SIZE = struct.Struct("<I")


class TraceValidationError(ValueError):
    """Raised when the input is not the accepted scene6 task trace."""


@dataclass(frozen=True)
class ValidatedTrace:
    path: Path
    sha256: str
    size_bytes: int
    trace_id: str
    model_path: Path
    model_sha256: str
    render_model_path: Path
    render_model_sha256: str
    frames: tuple[dict[str, Any], ...]
    first_time_s: float
    last_time_s: float
    dimensions: tuple[int, int, int]

    def inspection(self) -> dict[str, Any]:
        nq, nv, nu = self.dimensions
        return {
            "schema_id": "matrix.physics_trace_replay.inspection.v1",
            "valid": True,
            "physics_execution": PHYSICS_EXECUTION,
            "render_mode": RENDER_MODE,
            "trace": {
                "path": str(self.path),
                "sha256": self.sha256,
                "size_bytes": self.size_bytes,
                "schema_id": TRACE_SCHEMA,
                "physics_trace_id": self.trace_id,
            },
            "model": {
                "path": str(self.model_path),
                "sha256": self.model_sha256,
                "size_bytes": self.model_path.stat().st_size,
            },
            "render_robot_model": {
                "path": str(self.render_model_path),
                "sha256": self.render_model_sha256,
                "size_bytes": self.render_model_path.stat().st_size,
            },
            "dimensions": {"nq": nq, "nv": nv, "nu": nu},
            "source_frame_count": len(self.frames),
            "fps": REPLAY_FPS,
            "source_duration_s": round(len(self.frames) / REPLAY_FPS, 6),
            "trace_time_range_s": [self.first_time_s, self.last_time_s],
        }


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    if path.is_symlink() or path.is_dir():
        raise TraceValidationError(
            f"JSON output must not be a symlink or directory: {path}"
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        dir=path.parent,
        prefix=f".{path.name}.",
        delete=False,
    ) as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
        temporary = Path(stream.name)
    os.replace(temporary, path)


def _reject_constant(value: str) -> None:
    raise TraceValidationError(f"non-finite JSON number is forbidden: {value}")


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise TraceValidationError(f"duplicate JSON key is forbidden: {key}")
        result[key] = value
    return result


def _read_json(path: Path, *, max_bytes: int) -> dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        raise TraceValidationError(f"trace must be a regular non-symlink file: {path}")
    size = path.stat().st_size
    if size <= 0 or size > max_bytes:
        raise TraceValidationError(
            f"trace size must be in 1..{max_bytes} bytes, got {size}"
        )
    try:
        payload = json.loads(
            path.read_text(encoding="utf-8"),
            parse_constant=_reject_constant,
            object_pairs_hook=_unique_object,
        )
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise TraceValidationError(f"invalid trace JSON: {exc}") from exc
    if not isinstance(payload, dict):
        raise TraceValidationError("trace root must be an object")
    return payload


def _finite_number(value: Any, *, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TraceValidationError(f"{field} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise TraceValidationError(f"{field} must be finite")
    return result


def _vector(value: Any, *, length: int, field: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != length:
        actual = len(value) if isinstance(value, list) else type(value).__name__
        raise TraceValidationError(f"{field} shape must be {length}, got {actual}")
    return tuple(
        _finite_number(item, field=f"{field}[{index}]")
        for index, item in enumerate(value)
    )


def _required_mapping(value: Any, *, field: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise TraceValidationError(f"{field} must be an object")
    return value


def _resolve_model_path(
    trace_path: Path,
    configured: Any,
    override: Path | None,
) -> Path:
    if override is not None:
        candidates = [override.expanduser()]
    elif isinstance(configured, str) and configured:
        raw = Path(configured).expanduser()
        candidates = [raw, trace_path.parent / raw]
        candidates.extend(
            (
                trace_path.parent / "model" / raw.name,
                trace_path.parent / raw.name,
            )
        )
    else:
        raise TraceValidationError("trace model_path must be a non-empty string")
    for candidate in candidates:
        if candidate.is_symlink():
            continue
        try:
            resolved = candidate.resolve(strict=True)
        except OSError:
            continue
        if resolved.is_file() and not resolved.is_symlink():
            return resolved
    raise TraceValidationError(
        "trace model does not exist; pass --model explicitly: "
        + ", ".join(str(candidate) for candidate in candidates)
    )


def validate_trace(
    trace_path: Path,
    *,
    model_override: Path | None = None,
    max_bytes: int = DEFAULT_MAX_TRACE_BYTES,
) -> ValidatedTrace:
    trace_path = trace_path.expanduser()
    if trace_path.is_symlink():
        raise TraceValidationError(
            f"trace must be a regular non-symlink file: {trace_path}"
        )
    trace_path = trace_path.resolve()
    payload = _read_json(trace_path, max_bytes=max_bytes)
    if payload.get("schema_id") != TRACE_SCHEMA:
        raise TraceValidationError(
            f"trace schema must be {TRACE_SCHEMA!r}, got {payload.get('schema_id')!r}"
        )
    if payload.get("physics_backend") != "mujoco":
        raise TraceValidationError("physics_backend must be 'mujoco'")
    if payload.get("persistent_world_state") is not True:
        raise TraceValidationError("persistent_world_state must be true")
    if payload.get("status") != "succeeded":
        raise TraceValidationError("only a succeeded TwinBot task trace may be replayed")
    trace_id = payload.get("physics_trace_id")
    if not isinstance(trace_id, str) or not trace_id.strip():
        raise TraceValidationError("physics_trace_id must be a non-empty string")

    scene_context = _required_mapping(
        payload.get("scene_context"), field="scene_context"
    )
    required_scene_values = {
        "scene_number": 6,
        "map_name": "/Game/Maps/HouseWorld",
        "physics_execution": PHYSICS_EXECUTION,
        "intended_render_mode": RENDER_MODE,
    }
    for field, expected in required_scene_values.items():
        if scene_context.get(field) != expected:
            raise TraceValidationError(
                f"scene_context.{field} must be {expected!r}, "
                f"got {scene_context.get(field)!r}"
            )
    assistance = scene_context.get("manipulation_assistance")
    if assistance != "contact_gated_wrist_cube_weld_and_anchored_stance":
        raise TraceValidationError(
            "scene_context.manipulation_assistance must disclose the "
            "contact-gated weld and anchored stance"
        )

    control = _required_mapping(payload.get("control"), field="control")
    if control.get("controller") != "behavior_tree_controller_switching":
        raise TraceValidationError("unexpected TwinBot task controller")
    if control.get("mode") != "persistent_matrix_home_world_v0":
        raise TraceValidationError("unexpected TwinBot persistent-world mode")

    dimensions = payload.get("dimensions")
    if (
        not isinstance(dimensions, dict)
        or set(dimensions) != {"nq", "nv", "nu"}
        or any(
            isinstance(dimensions[name], bool)
            or not isinstance(dimensions[name], int)
            for name in ("nq", "nv", "nu")
        )
        or tuple(dimensions[name] for name in ("nq", "nv", "nu"))
        != EXPECTED_DIMS
    ):
        raise TraceValidationError("trace dimensions must be exactly 57/55/43")
    physics_timestep = _finite_number(
        payload.get("physics_timestep_s"), field="physics_timestep_s"
    )
    if not math.isclose(physics_timestep, 0.002, abs_tol=1e-12):
        raise TraceValidationError("physics_timestep_s must be 0.002")
    sample_fps = _finite_number(payload.get("sample_fps"), field="sample_fps")
    if not math.isclose(sample_fps, REPLAY_FPS, abs_tol=1e-9):
        raise TraceValidationError("sample_fps must be 25")

    raw_transitions = payload.get("transitions")
    if not isinstance(raw_transitions, list) or not raw_transitions:
        raise TraceValidationError("trace transitions must be a non-empty array")
    transition_phases: list[str] = []
    previous_transition_time: float | None = None
    for index, raw_transition in enumerate(raw_transitions):
        transition = _required_mapping(
            raw_transition, field=f"transitions[{index}]"
        )
        phase = transition.get("phase")
        if not isinstance(phase, str) or not phase:
            raise TraceValidationError(
                f"transitions[{index}].phase must be a non-empty string"
            )
        transition_time = _finite_number(
            transition.get("time_s"), field=f"transitions[{index}].time_s"
        )
        if transition_time < 0.0 or (
            previous_transition_time is not None
            and transition_time < previous_transition_time
        ):
            raise TraceValidationError("trace transition time regressed")
        previous_transition_time = transition_time
        transition_phases.append(phase)
    search_from = 0
    for required_phase in REQUIRED_TRANSITION_SUBSEQUENCE:
        try:
            found = transition_phases.index(required_phase, search_from)
        except ValueError as exc:
            raise TraceValidationError(
                "trace is missing ordered task transition: " + required_phase
            ) from exc
        search_from = found + 1

    raw_frames = payload.get("frames")
    if not isinstance(raw_frames, list) or not raw_frames:
        raise TraceValidationError("trace frames must be a non-empty array")
    nq, nv, nu = EXPECTED_DIMS
    frames: list[dict[str, Any]] = []
    previous_time: float | None = None
    for index, raw_frame in enumerate(raw_frames):
        frame = _required_mapping(raw_frame, field=f"frames[{index}]")
        sim_time = _finite_number(
            frame.get("time_s"), field=f"frames[{index}].time_s"
        )
        if sim_time < 0.0:
            raise TraceValidationError(f"frames[{index}].time_s must be non-negative")
        if previous_time is not None and sim_time < previous_time:
            raise TraceValidationError(
                f"frames[{index}].time_s regressed from {previous_time} to {sim_time}"
            )
        previous_time = sim_time
        qpos = _vector(frame.get("qpos"), length=nq, field=f"frames[{index}].qpos")
        qvel = _vector(frame.get("qvel"), length=nv, field=f"frames[{index}].qvel")
        ctrl = _vector(frame.get("ctrl"), length=nu, field=f"frames[{index}].ctrl")
        phase = frame.get("controller_phase")
        if not isinstance(phase, str) or not phase:
            raise TraceValidationError(
                f"frames[{index}].controller_phase must be a non-empty string"
            )
        frames.append(
            {
                "time_s": sim_time,
                "qpos": qpos,
                "qvel": qvel,
                "ctrl": ctrl,
                "controller_phase": phase,
            }
        )
    assert previous_time is not None
    model_path = _resolve_model_path(
        trace_path, payload.get("model_path"), model_override
    )
    render_model_path = _resolve_model_path(
        trace_path, payload.get("render_robot_model_path"), None
    )
    declared_render_hash = payload.get("render_robot_model_sha256")
    render_model_sha256 = _sha256(render_model_path)
    if declared_render_hash != render_model_sha256:
        raise TraceValidationError(
            "render_robot_model_sha256 does not match render_robot_model_path"
        )
    return ValidatedTrace(
        path=trace_path,
        sha256=_sha256(trace_path),
        size_bytes=trace_path.stat().st_size,
        trace_id=trace_id,
        model_path=model_path,
        model_sha256=_sha256(model_path),
        render_model_path=render_model_path,
        render_model_sha256=render_model_sha256,
        frames=tuple(frames),
        first_time_s=float(frames[0]["time_s"]),
        last_time_s=float(frames[-1]["time_s"]),
        dimensions=EXPECTED_DIMS,
    )


def pack_render_packet(frame: dict[str, Any]) -> bytes:
    """Pack one frame using Matrix's little-endian variable-vector protocol."""

    qpos = frame["qpos"]
    qvel = frame["qvel"]
    ctrl = frame["ctrl"]
    payload = bytearray(_TIME_AND_SIZE.pack(float(frame["time_s"]), len(qpos)))
    for values in (qpos,):
        payload.extend(struct.pack(f"<{len(values)}d", *values))
    payload.extend(_SIZE.pack(len(qvel)))
    payload.extend(struct.pack(f"<{len(qvel)}d", *qvel))
    payload.extend(_SIZE.pack(len(ctrl)))
    payload.extend(struct.pack(f"<{len(ctrl)}d", *ctrl))
    return bytes(payload)


def _process_start_ticks(pid: int) -> str:
    try:
        fields = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8").split()
    except OSError as exc:
        raise RuntimeError(f"UE process {pid} is unavailable") from exc
    if len(fields) < 22 or fields[2] == "Z":
        raise RuntimeError(f"UE process {pid} is not live")
    return fields[21]


def _ue_is_same_process(pid: int, start_ticks: str) -> bool:
    try:
        fields = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8").split()
    except OSError:
        return False
    return len(fields) >= 22 and fields[2] != "Z" and fields[21] == start_ticks


class ReplayInterrupted(RuntimeError):
    pass


class ReplayFinalHoldStopped(RuntimeError):
    """The launcher stopped replay after every source trace frame was sent."""


def replay(
    validated: ValidatedTrace,
    *,
    status_path: Path,
    summary_path: Path,
    pre_roll_s: float,
    final_hold_s: float,
    ue_pid: int | None,
) -> dict[str, Any]:
    if pre_roll_s < 0.0 or not math.isfinite(pre_roll_s):
        raise ValueError("pre-roll must be a non-negative finite number")
    if final_hold_s < 0.0 or not math.isfinite(final_hold_s):
        raise ValueError("final hold must be a non-negative finite number")
    ue_start_ticks = _process_start_ticks(ue_pid) if ue_pid is not None else None
    pre_roll_packets = round(pre_roll_s * REPLAY_FPS)
    final_hold_packets = round(final_hold_s * REPLAY_FPS)
    trace_packets = len(validated.frames)
    expected_packets = pre_roll_packets + trace_packets + final_hold_packets
    interval_s = 1.0 / REPLAY_FPS
    started_monotonic = time.monotonic()
    started_at = datetime.now(timezone.utc)
    stop_requested = False
    packet_count = 0
    current_source_index = 0
    trace_packets_sent = 0
    trace_complete = False

    def request_stop(_signum: int, _frame: Any) -> None:
        nonlocal stop_requested
        stop_requested = True

    previous_handlers = {
        signum: signal.signal(signum, request_stop)
        for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)
    }

    inspection = validated.inspection()

    def status(*, active: bool, completed: bool, passed: bool | None) -> dict[str, Any]:
        elapsed = max(0.0, time.monotonic() - started_monotonic)
        nq, nv, nu = validated.dimensions
        return {
            "schema_id": STATUS_SCHEMA,
            "updated_at": datetime.now(timezone.utc).isoformat(),
            "physics_execution": PHYSICS_EXECUTION,
            "render_mode": RENDER_MODE,
            "backend": RENDER_MODE,
            "scene": "matrix_house_scene6",
            "scene_number": 6,
            "map_name": "/Game/Maps/HouseWorld",
            "active_lowcmd": active,
            "active_lowcmd_semantics": (
                "legacy_recorder_readiness_gate_no_dds_lowcmd"
            ),
            "dds_lowcmd_active": False,
            "active_elapsed_s": round(elapsed, 6),
            "physics_step_hz": None,
            "rtf": None,
            "physics_metrics_source": "offline_trace_not_measured_during_replay",
            "replay_fps": REPLAY_FPS,
            "replay_rtf": 1.0,
            "completed": completed,
            "passed": passed,
            "frame_index": current_source_index,
            "frame_count": trace_packets,
            "packet_count": packet_count,
            "expected_packet_count": expected_packets,
            "dimensions": {"nq": nq, "nv": nv, "nu": nu},
            "trace": inspection["trace"],
            "model": inspection["render_robot_model"]["path"],
            "model_provenance": inspection["render_robot_model"],
            "scene_model_provenance": inspection["model"],
            "manipulation_assistance": (
                "contact_gated_wrist_cube_weld_and_anchored_stance"
            ),
        }

    def check_runtime() -> None:
        if stop_requested:
            if trace_complete:
                raise ReplayFinalHoldStopped(
                    "launcher stopped replay after all source frames were sent"
                )
            raise ReplayInterrupted("trace replay was interrupted")
        if (
            ue_pid is not None
            and ue_start_ticks is not None
            and not _ue_is_same_process(ue_pid, ue_start_ticks)
        ):
            raise ReplayInterrupted("supervised UE process exited during trace replay")

    def sleep_until(deadline: float) -> None:
        while True:
            check_runtime()
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return
            time.sleep(min(remaining, 0.04))

    first = validated.frames[0]
    last = validated.frames[-1]
    sequence = itertools.chain(
        (("pre_roll", 0, first) for _ in range(pre_roll_packets)),
        (
            ("trace", index, frame)
            for index, frame in enumerate(validated.frames)
        ),
        (
            ("final_hold", trace_packets - 1, last)
            for _ in range(final_hold_packets)
        ),
    )

    failure: str | None = None
    clean_final_hold_stop = False
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        _atomic_json(status_path, status(active=True, completed=False, passed=None))
        deadline = time.monotonic()
        last_status_write = 0.0
        for phase, source_index, frame in sequence:
            check_runtime()
            current_source_index = source_index
            payload = pack_render_packet(frame)
            sent = sender.sendto(payload, RENDER_ADDRESS)
            if sent != len(payload):
                raise RuntimeError(f"partial UDP send: {sent}/{len(payload)} bytes")
            packet_count += 1
            if phase == "trace":
                trace_packets_sent += 1
                trace_complete = trace_packets_sent == trace_packets
            now = time.monotonic()
            if now - last_status_write >= 0.2 or packet_count == expected_packets:
                _atomic_json(
                    status_path,
                    status(active=True, completed=False, passed=None),
                )
                last_status_write = now
            deadline += interval_s
            sleep_until(deadline)
    except ReplayFinalHoldStopped:
        clean_final_hold_stop = True
    except (OSError, RuntimeError) as exc:
        failure = str(exc)
    finally:
        sender.close()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)

    passed = (
        failure is None
        and trace_complete
        and (clean_final_hold_stop or packet_count == expected_packets)
    )
    finished_at = datetime.now(timezone.utc)
    wall_duration_s = max(0.0, time.monotonic() - started_monotonic)
    summary = {
        "schema_id": SUMMARY_SCHEMA,
        "passed": passed,
        "failure": failure,
        "completion": (
            "trace_complete_final_hold_stopped_by_launcher"
            if clean_final_hold_stop
            else "scheduled_replay_complete"
            if passed
            else "failed"
        ),
        "physics_execution": PHYSICS_EXECUTION,
        "render_mode": RENDER_MODE,
        "manipulation_assistance": (
            "contact_gated_wrist_cube_weld_and_anchored_stance"
        ),
        "started_at": started_at.isoformat(),
        "finished_at": finished_at.isoformat(),
        "wall_duration_s": round(wall_duration_s, 6),
        "fps": REPLAY_FPS,
        "udp": {"host": RENDER_ADDRESS[0], "port": RENDER_ADDRESS[1]},
        "trace": inspection["trace"],
        "model": inspection["render_robot_model"],
        "scene_model": inspection["model"],
        "dimensions": inspection["dimensions"],
        "source_frame_count": trace_packets,
        "source_duration_s": inspection["source_duration_s"],
        "trace_time_range_s": inspection["trace_time_range_s"],
        "packets": {
            "pre_roll": pre_roll_packets,
            "trace": trace_packets,
            "trace_sent": trace_packets_sent,
            "final_hold": final_hold_packets,
            "expected": expected_packets,
            "sent": packet_count,
        },
        "status_path": str(status_path),
    }
    _atomic_json(summary_path, summary)
    _atomic_json(
        status_path,
        status(active=False, completed=True, passed=passed),
    )
    return summary


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--model", type=Path)
    inspection = parser.add_mutually_exclusive_group()
    inspection.add_argument("--inspect", action="store_true")
    inspection.add_argument("--inspect-frame-count", action="store_true")
    parser.add_argument("--status-file", type=Path)
    parser.add_argument("--summary", type=Path)
    parser.add_argument("--pre-roll", type=float, default=2.0)
    parser.add_argument("--final-hold", type=float, default=6.0)
    parser.add_argument("--ue-pid", type=int)
    parser.add_argument("--max-trace-bytes", type=int, default=DEFAULT_MAX_TRACE_BYTES)
    args = parser.parse_args(argv)
    if args.max_trace_bytes <= 0:
        parser.error("--max-trace-bytes must be positive")
    if args.pre_roll < 0.0 or not math.isfinite(args.pre_roll):
        parser.error("--pre-roll must be a non-negative finite number")
    if args.final_hold < 0.0 or not math.isfinite(args.final_hold):
        parser.error("--final-hold must be a non-negative finite number")
    if args.ue_pid is not None and args.ue_pid <= 0:
        parser.error("--ue-pid must be positive")
    if not (args.inspect or args.inspect_frame_count) and (
        args.status_file is None or args.summary is None
    ):
        parser.error("replay requires --status-file and --summary")
    return args


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    try:
        validated = validate_trace(
            args.trace,
            model_override=args.model,
            max_bytes=args.max_trace_bytes,
        )
        if args.inspect:
            print(json.dumps(validated.inspection(), ensure_ascii=False, sort_keys=True))
            return 0
        if args.inspect_frame_count:
            print(len(validated.frames))
            return 0
        summary = replay(
            validated,
            status_path=args.status_file.expanduser().resolve(),
            summary_path=args.summary.expanduser().resolve(),
            pre_roll_s=args.pre_roll,
            final_hold_s=args.final_hold,
            ue_pid=args.ue_pid,
        )
    except (OSError, TraceValidationError, ValueError, RuntimeError) as exc:
        print(f"[matrix-trace-replay] ERROR: {exc}", file=os.sys.stderr)
        return 2
    print(
        "[matrix-trace-replay] "
        f"passed={summary['passed']} packets={summary['packets']['sent']}/"
        f"{summary['packets']['expected']} summary={args.summary}",
        flush=True,
    )
    return 0 if summary["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
