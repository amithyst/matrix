#!/usr/bin/env python3
"""Relay Leo's BFM Isaac state socket into Matrix's UDP render protocol.

The BFM runtime publishes one actual post-physics state per 50 Hz control
tick.  Matrix's cooked UE process consumes the same qpos/qvel ABI on UDP 9999.
This relay validates the frozen 588-byte input, tracks its simulated-time
sequence, pads only the canonical visual RobotPack's unused ctrl vector, and
optionally smooths wall-time delivery.
Interpolation is presentation-only and is never counted as a physics/control
tick.
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
import select
import signal
import socket
import sys
import time
from typing import Deque

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from matrix_render_protocol import (  # noqa: E402
    MujocoRenderState,
    pack_mujoco_state,
    packet_size,
    unpack_mujoco_state,
)
from matrix_bfm_isaac_command import send_key_event  # noqa: E402


STATUS_SCHEMA = "matrix_bfm_isaac_relay_status.v1"
BOOTSTRAP_SCHEMA = "matrix_bfm_isaac_bootstrap_state.v1"


@dataclass
class RelayStats:
    received: int = 0
    forwarded: int = 0
    invalid: int = 0
    sequence_gaps: int = 0
    duplicates: int = 0
    out_of_order: int = 0
    non_grid_time: int = 0
    prefill_forwarded: int = 0
    interpolated_forwarded: int = 0
    held_forwarded: int = 0
    first_sequence: int | None = None
    last_sequence: int | None = None
    input_bytes_min: int | None = None
    input_bytes_max: int | None = None


@dataclass(frozen=True)
class CollisionBounds:
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    warning_margin: float
    stop_margin: float

    def __post_init__(self) -> None:
        values = (
            self.x_min,
            self.x_max,
            self.y_min,
            self.y_max,
            self.warning_margin,
            self.stop_margin,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("collision bounds must be finite")
        if self.x_min >= self.x_max or self.y_min >= self.y_max:
            raise ValueError("collision bounds must be ordered")
        half_short_side = 0.5 * min(
            self.x_max - self.x_min, self.y_max - self.y_min
        )
        if not 0.0 < self.stop_margin < self.warning_margin < half_short_side:
            raise ValueError(
                "collision margins must satisfy 0 < stop < warning < half-span"
            )

    def edge_distance(self, x: float, y: float) -> float:
        return min(
            x - self.x_min,
            self.x_max - x,
            y - self.y_min,
            self.y_max - y,
        )


@dataclass
class BoundaryGuardStats:
    armed: bool = False
    warning_events: int = 0
    stop_events: int = 0
    stop_pulses: int = 0
    command_errors: int = 0
    hard_violations: int = 0
    first_stop_root_xy: list[float] | None = None
    minimum_edge_distance_m: float | None = None


class BoundaryGuard:
    """Latch SPACE before the robot reaches the finite Moon collision edge."""

    def __init__(
        self,
        bounds: CollisionBounds,
        *,
        keyboard_socket: Path | None,
        repeat_s: float = 0.25,
    ) -> None:
        if not math.isfinite(repeat_s) or repeat_s <= 0.0:
            raise ValueError("boundary safety repeat must be positive and finite")
        self.bounds = bounds
        self.keyboard_socket = keyboard_socket
        self.repeat_s = float(repeat_s)
        self.stats = BoundaryGuardStats(armed=True)
        self._warning_latched = False
        self._stop_latched = False
        self._hard_latched = False
        self._next_pulse_wall = 0.0
        self._sender = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)

    def observe(self, state: MujocoRenderState, *, wall_time: float) -> None:
        x = float(state.qpos[0])
        y = float(state.qpos[1])
        distance = self.bounds.edge_distance(x, y)
        previous_minimum = self.stats.minimum_edge_distance_m
        self.stats.minimum_edge_distance_m = (
            distance
            if previous_minimum is None
            else min(previous_minimum, distance)
        )
        if distance <= self.bounds.warning_margin and not self._warning_latched:
            self._warning_latched = True
            self.stats.warning_events += 1
            print(
                "[WARN] Moon collision boundary approaching "
                f"root=({x:.3f},{y:.3f}) edge_distance_m={distance:.3f}",
                flush=True,
            )
        if distance <= self.bounds.stop_margin:
            if not self._stop_latched:
                self._stop_latched = True
                self.stats.stop_events += 1
                self.stats.first_stop_root_xy = [x, y]
                print(
                    "[WARN] Moon collision boundary safety stop latched "
                    f"root=({x:.3f},{y:.3f}) edge_distance_m={distance:.3f}",
                    flush=True,
                )
            if wall_time >= self._next_pulse_wall:
                self._next_pulse_wall = wall_time + self.repeat_s
                if self.keyboard_socket is None:
                    self.stats.command_errors += 1
                else:
                    try:
                        send_key_event(
                            self._sender, self.keyboard_socket, "SPACE", True
                        )
                        send_key_event(
                            self._sender, self.keyboard_socket, "SPACE", False
                        )
                    except OSError as exc:
                        self.stats.command_errors += 1
                        print(
                            "[ERROR] Moon boundary stop command failed: "
                            f"{exc}",
                            flush=True,
                        )
                    else:
                        self.stats.stop_pulses += 1
        if distance < 0.0 and not self._hard_latched:
            self._hard_latched = True
            self.stats.hard_violations += 1
            print(
                "[ERROR] G1 crossed the finite Moon collision boundary "
                f"root=({x:.3f},{y:.3f}) edge_distance_m={distance:.3f}",
                flush=True,
            )

    def close(self) -> None:
        self._sender.close()


class SequenceTracker:
    """Validate the source's exact 50 Hz simulated-time sequence."""

    def __init__(self, control_hz: float, *, tolerance_s: float = 1.0e-6) -> None:
        if not math.isfinite(control_hz) or control_hz <= 0.0:
            raise ValueError("control_hz must be positive and finite")
        if not math.isfinite(tolerance_s) or tolerance_s < 0.0:
            raise ValueError("tolerance_s must be non-negative and finite")
        self.control_hz = float(control_hz)
        self.tolerance_s = float(tolerance_s)
        self.last_sequence: int | None = None

    def observe(self, sim_time: float, stats: RelayStats) -> bool:
        if not math.isfinite(sim_time) or sim_time < 0.0:
            stats.non_grid_time += 1
            return False
        sequence = int(round(sim_time * self.control_hz))
        expected_time = sequence / self.control_hz
        if abs(sim_time - expected_time) > self.tolerance_s:
            stats.non_grid_time += 1
            return False
        if self.last_sequence is None:
            stats.first_sequence = sequence
            if sequence != 0:
                stats.sequence_gaps += sequence
        elif sequence == self.last_sequence:
            stats.duplicates += 1
            return False
        elif sequence < self.last_sequence:
            stats.out_of_order += 1
            return False
        elif sequence > self.last_sequence + 1:
            stats.sequence_gaps += sequence - self.last_sequence - 1
        self.last_sequence = sequence
        stats.last_sequence = sequence
        return True


def _slerp_wxyz(left: np.ndarray, right: np.ndarray, alpha: float) -> np.ndarray:
    q0 = np.asarray(left, dtype=np.float64).copy()
    q1 = np.asarray(right, dtype=np.float64).copy()
    norm0 = float(np.linalg.norm(q0))
    norm1 = float(np.linalg.norm(q1))
    if norm0 < 1.0e-12 or norm1 < 1.0e-12:
        raise ValueError("cannot interpolate a zero-length root quaternion")
    q0 /= norm0
    q1 /= norm1
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 *= -1.0
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        result = (1.0 - alpha) * q0 + alpha * q1
        return result / np.linalg.norm(result)
    angle = math.acos(dot)
    sine = math.sin(angle)
    return (
        math.sin((1.0 - alpha) * angle) / sine * q0
        + math.sin(alpha * angle) / sine * q1
    )


def interpolate_state(
    left: MujocoRenderState,
    right: MujocoRenderState,
    alpha: float,
) -> MujocoRenderState:
    if not 0.0 <= alpha <= 1.0:
        raise ValueError("interpolation alpha must be in [0, 1]")
    if (
        left.qpos.shape != right.qpos.shape
        or left.qvel.shape != right.qvel.shape
        or left.ctrl.shape != right.ctrl.shape
    ):
        raise ValueError("cannot interpolate states with different dimensions")
    qpos = (1.0 - alpha) * left.qpos + alpha * right.qpos
    if qpos.size >= 7:
        qpos[3:7] = _slerp_wxyz(left.qpos[3:7], right.qpos[3:7], alpha)
    return MujocoRenderState(
        sim_time=(1.0 - alpha) * left.sim_time + alpha * right.sim_time,
        qpos=qpos,
        qvel=(1.0 - alpha) * left.qvel + alpha * right.qvel,
        ctrl=(1.0 - alpha) * left.ctrl + alpha * right.ctrl,
    )


def state_at_wall_time(
    history: Deque[tuple[float, MujocoRenderState]], target: float
) -> tuple[MujocoRenderState, bool]:
    if not history:
        raise ValueError("state history is empty")
    if target <= history[0][0]:
        return history[0][1], False
    if target >= history[-1][0]:
        return history[-1][1], False
    for index in range(1, len(history)):
        right_time, right = history[index]
        if target <= right_time:
            left_time, left = history[index - 1]
            span = right_time - left_time
            alpha = 1.0 if span <= 1.0e-9 else (target - left_time) / span
            return interpolate_state(left, right, alpha), True
    return history[-1][1], False


def validate_source_state(
    state: MujocoRenderState,
    *,
    nq: int,
    nv: int,
    nu: int,
) -> None:
    dimensions = (state.qpos.size, state.qvel.size, state.ctrl.size)
    if dimensions != (nq, nv, nu):
        raise ValueError(
            "source dimensions do not match frozen contract: "
            f"expected={nq}/{nv}/{nu} actual={dimensions[0]}/{dimensions[1]}/{dimensions[2]}"
        )
    if state.qpos.size >= 7:
        norm = float(np.linalg.norm(state.qpos[3:7]))
        if not 0.99 <= norm <= 1.01:
            raise ValueError(f"source root quaternion is not unit length: {norm}")


def matrix_output_state(
    state: MujocoRenderState, *, output_nu: int, ctrl_fill: float
) -> MujocoRenderState:
    if output_nu < 0:
        raise ValueError("output_nu must be non-negative")
    if not math.isfinite(ctrl_fill):
        raise ValueError("ctrl_fill must be finite")
    return MujocoRenderState(
        sim_time=state.sim_time,
        qpos=state.qpos.copy(),
        qvel=state.qvel.copy(),
        ctrl=np.full((output_nu,), ctrl_fill, dtype=np.float64),
    )


def load_bootstrap_state(
    path: Path, *, nq: int, nv: int, output_nu: int, ctrl_fill: float
) -> bytes:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("schema") != BOOTSTRAP_SCHEMA:
        raise ValueError("unsupported bootstrap state schema")
    if set(payload) != {"schema", "sim_time", "qpos", "qvel"}:
        raise ValueError("bootstrap state must contain schema/sim_time/qpos/qvel only")
    state = MujocoRenderState(
        sim_time=float(payload["sim_time"]),
        qpos=np.asarray(payload["qpos"], dtype=np.float64),
        qvel=np.asarray(payload["qvel"], dtype=np.float64),
        ctrl=np.empty((0,), dtype=np.float64),
    )
    validate_source_state(state, nq=nq, nv=nv, nu=0)
    output = matrix_output_state(state, output_nu=output_nu, ctrl_fill=ctrl_fill)
    return pack_mujoco_state(
        output.sim_time, output.qpos, output.qvel, output.ctrl
    )


def atomic_write_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp.{os.getpid()}")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate and relay BFM Isaac states to Matrix UDP 9999"
    )
    parser.add_argument("--unix-socket", type=Path, required=True)
    parser.add_argument("--status-file", type=Path, required=True)
    parser.add_argument("--udp-host", default="127.0.0.1")
    parser.add_argument("--udp-port", type=int, default=9999)
    parser.add_argument("--input-nq", type=int, default=36)
    parser.add_argument("--input-nv", type=int, default=35)
    parser.add_argument("--input-nu", type=int, default=0)
    parser.add_argument("--output-nu", type=int, default=29)
    parser.add_argument("--ctrl-fill", type=float, default=0.0)
    parser.add_argument("--control-hz", type=float, default=50.0)
    parser.add_argument("--output-hz", type=float, default=50.0)
    parser.add_argument("--interpolation-buffer-s", type=float, default=0.10)
    parser.add_argument("--no-interpolate", action="store_true")
    parser.add_argument("--bootstrap-state", type=Path)
    parser.add_argument("--bootstrap-hz", type=float, default=50.0)
    parser.add_argument("--safety-keyboard-socket", type=Path)
    parser.add_argument("--collision-x-min", type=float)
    parser.add_argument("--collision-x-max", type=float)
    parser.add_argument("--collision-y-min", type=float)
    parser.add_argument("--collision-y-max", type=float)
    parser.add_argument("--collision-warning-margin", type=float, default=20.0)
    parser.add_argument("--collision-stop-margin", type=float, default=10.0)
    parser.add_argument("--boundary-safety-repeat-s", type=float, default=0.25)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    for name in ("input_nq", "input_nv", "input_nu", "output_nu"):
        if getattr(args, name) < 0:
            parser.error(f"--{name.replace('_', '-')} must be non-negative")
    for name in ("control_hz", "output_hz", "bootstrap_hz"):
        value = float(getattr(args, name))
        if not math.isfinite(value) or value <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be positive and finite")
    if (
        not math.isfinite(args.interpolation_buffer_s)
        or args.interpolation_buffer_s < 0.0
    ):
        parser.error("--interpolation-buffer-s must be non-negative and finite")
    if not 1 <= args.udp_port <= 65535:
        parser.error("--udp-port must be in [1, 65535]")

    raw_collision_bounds = (
        args.collision_x_min,
        args.collision_x_max,
        args.collision_y_min,
        args.collision_y_max,
    )
    if any(value is not None for value in raw_collision_bounds) and not all(
        value is not None for value in raw_collision_bounds
    ):
        parser.error("collision x/y min/max must be supplied together")
    boundary_guard: BoundaryGuard | None = None
    if all(value is not None for value in raw_collision_bounds):
        try:
            bounds = CollisionBounds(
                x_min=float(args.collision_x_min),
                x_max=float(args.collision_x_max),
                y_min=float(args.collision_y_min),
                y_max=float(args.collision_y_max),
                warning_margin=float(args.collision_warning_margin),
                stop_margin=float(args.collision_stop_margin),
            )
            boundary_guard = BoundaryGuard(
                bounds,
                keyboard_socket=args.safety_keyboard_socket,
                repeat_s=float(args.boundary_safety_repeat_s),
            )
        except ValueError as exc:
            parser.error(str(exc))

    expected_input_bytes = packet_size(
        nq=args.input_nq, nv=args.input_nv, nu=args.input_nu
    )
    expected_output_bytes = packet_size(
        nq=args.input_nq, nv=args.input_nv, nu=args.output_nu
    )
    bootstrap_packet: bytes | None = None
    if args.bootstrap_state is not None:
        try:
            bootstrap_packet = load_bootstrap_state(
                args.bootstrap_state,
                nq=args.input_nq,
                nv=args.input_nv,
                output_nu=args.output_nu,
                ctrl_fill=args.ctrl_fill,
            )
        except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
            parser.error(str(exc))
        if len(bootstrap_packet) != expected_output_bytes:
            parser.error("bootstrap packet does not match output wire contract")

    args.unix_socket.parent.mkdir(parents=True, exist_ok=True)
    args.unix_socket.unlink(missing_ok=True)
    receiver = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(str(args.unix_socket))
    args.unix_socket.chmod(0o600)
    receiver.setblocking(False)

    running = True

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal running
        running = False

    previous_handlers = {
        signum: signal.signal(signum, request_stop)
        for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)
    }
    stats = RelayStats()
    tracker = SequenceTracker(args.control_hz)
    history: Deque[tuple[float, MujocoRenderState]] = deque()
    first_host_packet = False
    started_wall = time.monotonic()
    next_bootstrap = started_wall
    next_output = started_wall
    output_period = 1.0 / args.output_hz
    bootstrap_period = 1.0 / args.bootstrap_hz
    print(
        "matrix-bfm-isaac-relay ready "
        f"socket={args.unix_socket} target={args.udp_host}:{args.udp_port} "
        f"input={args.input_nq}/{args.input_nv}/{args.input_nu}/{expected_input_bytes} "
        f"output={args.input_nq}/{args.input_nv}/{args.output_nu}/{expected_output_bytes} "
        f"control_hz={args.control_hz:g} output_hz={args.output_hz:g} "
        f"interpolate={not args.no_interpolate}",
        flush=True,
    )
    try:
        while running:
            now = time.monotonic()
            if first_host_packet and not args.no_interpolate:
                deadline = next_output
            elif not first_host_packet and bootstrap_packet is not None:
                deadline = next_bootstrap
            else:
                deadline = now + 0.25
            timeout = min(0.25, max(0.0, deadline - now))
            readable, _, _ = select.select([receiver], [], [], timeout)
            if readable:
                while running:
                    try:
                        datagram = receiver.recv(65535)
                    except BlockingIOError:
                        break
                    stats.input_bytes_min = (
                        len(datagram)
                        if stats.input_bytes_min is None
                        else min(stats.input_bytes_min, len(datagram))
                    )
                    stats.input_bytes_max = (
                        len(datagram)
                        if stats.input_bytes_max is None
                        else max(stats.input_bytes_max, len(datagram))
                    )
                    try:
                        if len(datagram) != expected_input_bytes:
                            raise ValueError("source packet byte size mismatch")
                        state = unpack_mujoco_state(datagram)
                        validate_source_state(
                            state,
                            nq=args.input_nq,
                            nv=args.input_nv,
                            nu=args.input_nu,
                        )
                    except ValueError:
                        stats.invalid += 1
                        continue
                    if not tracker.observe(state.sim_time, stats):
                        continue
                    stats.received += 1
                    if boundary_guard is not None:
                        boundary_guard.observe(state, wall_time=time.monotonic())
                    first_host_packet = True
                    arrival = time.monotonic()
                    if args.no_interpolate:
                        output = matrix_output_state(
                            state,
                            output_nu=args.output_nu,
                            ctrl_fill=args.ctrl_fill,
                        )
                        sender.sendto(
                            pack_mujoco_state(
                                output.sim_time,
                                output.qpos,
                                output.qvel,
                                output.ctrl,
                            ),
                            (args.udp_host, args.udp_port),
                        )
                        stats.forwarded += 1
                    else:
                        history.append((arrival, state))
                        if len(history) == 1:
                            next_output = arrival

            now = time.monotonic()
            if not first_host_packet:
                if bootstrap_packet is not None and now >= next_bootstrap:
                    sender.sendto(bootstrap_packet, (args.udp_host, args.udp_port))
                    stats.prefill_forwarded += 1
                    next_bootstrap += bootstrap_period
                    if next_bootstrap <= now:
                        next_bootstrap = now + bootstrap_period
                continue

            if not args.no_interpolate and history and now >= next_output:
                target = now - args.interpolation_buffer_s
                state, was_interpolated = state_at_wall_time(history, target)
                output = matrix_output_state(
                    state, output_nu=args.output_nu, ctrl_fill=args.ctrl_fill
                )
                datagram = pack_mujoco_state(
                    output.sim_time, output.qpos, output.qvel, output.ctrl
                )
                if len(datagram) != expected_output_bytes:
                    raise RuntimeError("output packet byte size drifted from contract")
                sender.sendto(datagram, (args.udp_host, args.udp_port))
                stats.forwarded += 1
                if was_interpolated:
                    stats.interpolated_forwarded += 1
                else:
                    stats.held_forwarded += 1
                while len(history) > 2 and history[1][0] <= target:
                    history.popleft()
                next_output += output_period
                if next_output <= now:
                    next_output = now + output_period
    finally:
        stopped_wall = time.monotonic()
        receiver.close()
        sender.close()
        if boundary_guard is not None:
            boundary_guard.close()
        args.unix_socket.unlink(missing_ok=True)
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        status: dict[str, object] = {
            "schema": STATUS_SCHEMA,
            "ok": (
                stats.invalid == 0
                and stats.sequence_gaps == 0
                and stats.duplicates == 0
                and stats.out_of_order == 0
                and stats.non_grid_time == 0
                and (
                    boundary_guard is None
                    or (
                        boundary_guard.stats.command_errors == 0
                        and boundary_guard.stats.hard_violations == 0
                    )
                )
            ),
            "wall_seconds": stopped_wall - started_wall,
            "control_hz_expected": args.control_hz,
            "input_contract": {
                "nq": args.input_nq,
                "nv": args.input_nv,
                "nu": args.input_nu,
                "packet_bytes": expected_input_bytes,
            },
            "output_contract": {
                "nq": args.input_nq,
                "nv": args.input_nv,
                "nu": args.output_nu,
                "packet_bytes": expected_output_bytes,
                "ctrl_fill": args.ctrl_fill,
            },
            "interpolation_enabled": not args.no_interpolate,
            "interpolation_buffer_s": args.interpolation_buffer_s,
            "stats": asdict(stats),
            "boundary_guard": (
                asdict(boundary_guard.stats)
                if boundary_guard is not None
                else asdict(BoundaryGuardStats())
            ),
        }
        atomic_write_json(args.status_file, status)
        print(json.dumps(status, sort_keys=True), flush=True)
    return 0 if status["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
