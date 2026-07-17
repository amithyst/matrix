#!/usr/bin/env python3
"""Run native gear_sonic MuJoCo physics and mirror it into a Matrix UE map."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import time
from typing import Any, Callable, TypedDict
from urllib.parse import urlsplit
import uuid


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--sonic-root", type=Path, required=True)
    parser.add_argument(
        "--control-source", choices=("planner", "pico", "external"), default="planner"
    )
    parser.add_argument("--planner-bind", default="tcp://127.0.0.1:5556")
    parser.add_argument(
        "--pico-python",
        default=None,
        help="Interpreter for SONIC's PICO manager (defaults to the simulator Python)",
    )
    parser.add_argument("--dds-interface", default="lo")
    parser.add_argument("--render-host", default="127.0.0.1")
    parser.add_argument("--render-port", type=int, default=9999)
    parser.add_argument(
        "--no-render-sync",
        action="store_true",
        help="Run physics/control without publishing state to a Matrix UE process",
    )
    parser.add_argument("--physics-hz", type=float, default=200.0)
    parser.add_argument("--control-hz", type=float, default=50.0)
    parser.add_argument("--max-seconds", type=float, default=0.0)
    parser.add_argument(
        "--fail-on-fall",
        action="store_true",
        help="Return non-zero when SONIC's authoritative fall flag is set",
    )
    parser.add_argument(
        "--min-active-seconds",
        type=float,
        default=0.0,
        help="Return non-zero unless native SONIC has one continuous fresh-lowcmd streak this long",
    )
    parser.add_argument(
        "--min-displacement-m",
        type=float,
        default=0.0,
        help="Minimum final world-XY root displacement required for acceptance",
    )
    parser.add_argument(
        "--min-final-x",
        type=float,
        default=None,
        help="Optional minimum final world-X root coordinate for directional acceptance",
    )
    parser.add_argument(
        "--min-forward-x-m",
        type=float,
        default=0.0,
        help="Minimum signed final-minus-initial world-X displacement",
    )
    parser.add_argument(
        "--low-cmd-fresh-timeout-seconds",
        type=float,
        default=0.1,
        help="Maximum native DDS lowcmd age that counts as fresh (default: 0.1)",
    )
    parser.add_argument("--min-physics-hz", type=float, default=0.0)
    parser.add_argument("--min-rtf", type=float, default=0.0)
    parser.add_argument("--max-resets", type=int, default=0)
    parser.add_argument("--walk-after", type=float, default=-1.0)
    parser.add_argument("--vx", type=float, default=0.30)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--yaw-rate", type=float, default=0.0)
    parser.add_argument("--status-file", type=Path)
    parser.add_argument("--qualified-runtime", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--qualification-profile", default=None, help=argparse.SUPPRESS)
    parser.add_argument("--runtime-lock-sha256", default=None, help=argparse.SUPPRESS)
    parser.add_argument("--scenario-layout-sha256", default=None, help=argparse.SUPPRESS)
    parser.add_argument("--matrix-commit", default=None, help=argparse.SUPPRESS)
    parser.add_argument(
        "--verification-receipt", type=Path, default=None, help=argparse.SUPPRESS
    )
    parser.add_argument(
        "--expected-parent-pid",
        type=int,
        default=None,
        help=argparse.SUPPRESS,
    )
    parser.add_argument("--print-every", type=float, default=2.0)
    parser.add_argument(
        "--startup-band",
        action="store_true",
        help="Wait for native DDS lowcmd, hold the root, then fade the elastic band",
    )
    parser.add_argument("--startup-band-hold", type=float, default=4.0)
    parser.add_argument("--startup-band-fade", type=float, default=3.0)
    return parser.parse_args()


def _atomic_json(path: Path | None, payload: dict[str, object]) -> None:
    if path is None:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", dir=path.parent, prefix=f".{path.name}.", delete=False
    ) as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
        temporary_path = Path(stream.name)
    os.replace(temporary_path, path)


def _arm_supervisor_parent_death(expected_parent_pid: int | None) -> None:
    if expected_parent_pid is None:
        return
    if expected_parent_pid <= 1:
        raise SystemExit("--expected-parent-pid must identify a live launcher")
    from exec_with_parent_death_signal import _arm_parent_death_signal

    _arm_parent_death_signal(signal.SIGTERM)
    if os.getppid() != expected_parent_pid:
        print(
            "matrix-sonic-runtime ERROR launcher exited before supervisor startup",
            file=sys.stderr,
        )
        raise SystemExit(125)


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _validate_qualification_receipt(args: argparse.Namespace) -> None:
    if not args.qualified_runtime:
        return
    matrix_root = Path(__file__).resolve().parents[1]
    lock_path = matrix_root / "config/runtime/matrix-sonic.lock.json"
    if _sha256_file(lock_path) != args.runtime_lock_sha256:
        raise SystemExit("qualified runtime lock SHA does not match the active lock")
    commit = subprocess.run(
        ["git", "-C", str(matrix_root), "rev-parse", "HEAD"],
        text=True,
        capture_output=True,
        check=False,
    )
    if (
        commit.returncode != 0
        or commit.stdout.strip() != args.matrix_commit
    ):
        raise SystemExit("qualified runtime requires the recorded Matrix commit")

    receipt_path = args.verification_receipt
    if receipt_path is None or not receipt_path.is_file() or receipt_path.is_symlink():
        raise SystemExit("--qualified-runtime requires a regular verification receipt")
    try:
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise SystemExit(f"invalid runtime verification receipt: {exc}") from exc
    checks = receipt.get("checks") if isinstance(receipt, dict) else None
    receipt_ok = (
        receipt.get("passed") is True
        and isinstance(checks, list)
        and bool(checks)
        and all(isinstance(item, dict) and item.get("ok") is True for item in checks)
        and receipt.get("profile") == args.qualification_profile
        and receipt.get("lock_sha256") == args.runtime_lock_sha256
        and Path(str(receipt.get("lock", ""))).resolve() == lock_path.resolve()
        and Path(str(receipt.get("matrix_root", ""))).resolve() == matrix_root
        and receipt.get("matrix_commit") == args.matrix_commit
        and Path(str(receipt.get("sonic_root", ""))).resolve()
        == args.sonic_root.resolve()
    )
    if not receipt_ok:
        raise SystemExit("runtime verification receipt does not match this launch")
    args.verification_receipt = receipt_path.resolve()
    args.verification_receipt_sha256 = _sha256_file(args.verification_receipt)


def _root_up_z(qpos) -> float:
    """Diagnostic world-Z component of the floating base's local up axis."""
    _, x, y, _ = [float(value) for value in qpos[3:7]]
    return 1.0 - 2.0 * (x * x + y * y)


def _pace_absolute_deadline(deadline_s: float, period_s: float) -> float:
    """Wait for one absolute tick and return the following tick deadline.

    A relative ``sleep(period - work)`` accumulates scheduler overshoot on
    every 5 ms SONIC step. Keeping the deadline on an absolute timeline lets
    the following sleep compensate for that overshoot while still resetting
    after a sustained overrun instead of emitting a catch-up burst.
    """
    now = time.perf_counter()
    remaining_s = deadline_s - now
    if remaining_s > 0.0:
        time.sleep(remaining_s)
        now = time.perf_counter()
    if now - deadline_s > 2.0 * period_s:
        return now + period_s
    return deadline_s + period_s


def _acceptance_failures(
    *,
    unstable: bool,
    fall_detected: bool,
    fail_on_fall: bool,
    active_lowcmd: bool,
    active_elapsed_s: float,
    min_active_seconds: float,
    physics_step_hz: float,
    min_physics_hz: float,
    rtf: float,
    min_rtf: float,
    failed_child: tuple[str, int] | None = None,
    root_displacement_xy_m: float = 0.0,
    min_displacement_m: float = 0.0,
    root_final_x: float = 0.0,
    min_final_x: float | None = None,
    root_displacement_x_m: float = 0.0,
    min_forward_x_m: float = 0.0,
    reset_count: int = 0,
    max_resets: int = 0,
) -> list[str]:
    failures = []
    if unstable:
        failures.append("numerical_instability")
    if failed_child is not None:
        name, code = failed_child
        failures.append(f"native_child_exit:{name}:{code}")
    if fail_on_fall and fall_detected:
        failures.append("fall_detected")
    if min_active_seconds > 0.0 and not active_lowcmd:
        failures.append("lowcmd_not_fresh_at_exit")
    if active_elapsed_s + 1e-6 < min_active_seconds:
        failures.append(
            f"active_lowcmd_too_short:{active_elapsed_s:.3f}<{min_active_seconds:.3f}"
        )
    if min_displacement_m > 0.0 and (
        not math.isfinite(root_displacement_xy_m)
        or root_displacement_xy_m + 1e-6 < min_displacement_m
    ):
        failures.append(
            "root_displacement_too_small:"
            f"{root_displacement_xy_m:.3f}<{min_displacement_m:.3f}"
        )
    if min_final_x is not None and (
        not math.isfinite(root_final_x) or root_final_x + 1e-6 < min_final_x
    ):
        failures.append(f"final_x_too_small:{root_final_x:.3f}<{min_final_x:.3f}")
    if min_forward_x_m > 0.0 and (
        not math.isfinite(root_displacement_x_m)
        or root_displacement_x_m + 1e-6 < min_forward_x_m
    ):
        failures.append(
            "forward_x_too_small:"
            f"{root_displacement_x_m:.3f}<{min_forward_x_m:.3f}"
        )
    if reset_count > max_resets:
        failures.append(f"reset_count_exceeded:{reset_count}>{max_resets}")
    if min_physics_hz > 0.0 and (
        not math.isfinite(physics_step_hz)
        or physics_step_hz + 1e-6 < min_physics_hz
    ):
        failures.append(
            f"physics_hz_too_low:{physics_step_hz:.3f}<{min_physics_hz:.3f}"
        )
    if min_rtf > 0.0 and (not math.isfinite(rtf) or rtf + 1e-6 < min_rtf):
        failures.append(f"rtf_too_low:{rtf:.4f}<{min_rtf:.4f}")
    return failures


_EXPECTED_SNAPSHOT_DIMS = {
    "qpos": 36,
    "qvel": 35,
    "ctrl": 29,
    "applied_torque": 29,
}


def _snapshot_validation_error(snapshot, previous_snapshot=None) -> str | None:
    """Return a precise native snapshot invariant violation, if any."""
    for field, expected in _EXPECTED_SNAPSHOT_DIMS.items():
        values = getattr(snapshot, field, None)
        if values is None:
            return f"snapshot_missing_field:{field}"
        try:
            actual = len(values)
        except TypeError:
            return f"snapshot_field_not_sized:{field}"
        if actual != expected:
            return f"snapshot_dimension:{field}={actual},expected={expected}"

    try:
        step_index = int(snapshot.step_index)
    except (TypeError, ValueError, OverflowError):
        return f"snapshot_invalid_step_index:{snapshot.step_index!r}"
    if step_index != snapshot.step_index:
        return f"snapshot_invalid_step_index:{snapshot.step_index!r}"

    try:
        sim_time = float(snapshot.sim_time)
    except (TypeError, ValueError, OverflowError):
        return f"snapshot_invalid_sim_time:{snapshot.sim_time!r}"
    if not math.isfinite(sim_time):
        return f"snapshot_non_finite:sim_time={sim_time!r}"

    if type(getattr(snapshot, "fall_detected", None)) is not bool:
        return f"snapshot_invalid_fall_detected:{getattr(snapshot, 'fall_detected', None)!r}"
    try:
        reset_count = int(snapshot.reset_count)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return f"snapshot_invalid_reset_count:{getattr(snapshot, 'reset_count', None)!r}"
    if reset_count != snapshot.reset_count or reset_count < 0:
        return f"snapshot_invalid_reset_count:{snapshot.reset_count!r}"
    last_reset_reason = getattr(snapshot, "last_reset_reason", None)
    if last_reset_reason is not None and not isinstance(last_reset_reason, str):
        return f"snapshot_invalid_last_reset_reason:{last_reset_reason!r}"

    for field in _EXPECTED_SNAPSHOT_DIMS:
        for index, value in enumerate(getattr(snapshot, field)):
            try:
                finite = math.isfinite(float(value))
            except (TypeError, ValueError, OverflowError):
                finite = False
            if not finite:
                return f"snapshot_non_finite:{field}[{index}]={value!r}"

    if previous_snapshot is not None:
        expected_step_index = int(previous_snapshot.step_index) + 1
        if step_index != expected_step_index:
            return (
                "snapshot_step_index_not_sequential:"
                f"{step_index},expected={expected_step_index}"
            )
        previous_sim_time = float(previous_snapshot.sim_time)
        if sim_time <= previous_sim_time:
            return (
                "snapshot_sim_time_not_increasing:"
                f"{sim_time!r},previous={previous_sim_time!r}"
            )
        if reset_count < int(previous_snapshot.reset_count):
            return (
                "snapshot_reset_count_decreased:"
                f"{reset_count},previous={int(previous_snapshot.reset_count)}"
            )
    return None


class _QualificationState(TypedDict):
    acceptance_failures: list[str]
    qualification_attempted: bool
    completed: bool
    interrupted: bool
    passed: bool


def _qualification_state(
    *,
    max_seconds: float,
    termination_reason: str,
    failures: list[str],
    runtime_verified: bool,
) -> _QualificationState:
    """Classify a finalized run without treating operator stops as passes."""
    acceptance_failures = list(failures)
    attempted = max_seconds > 0.0
    if attempted and not runtime_verified:
        acceptance_failures.append("runtime_not_verified_for_qualification")
    if attempted and termination_reason == "signal":
        acceptance_failures.append("run_interrupted")
    if termination_reason == "unknown":
        acceptance_failures.append("unknown_termination")
    completed = termination_reason == "max_seconds"
    return {
        "acceptance_failures": acceptance_failures,
        "qualification_attempted": attempted,
        "completed": completed,
        "interrupted": termination_reason == "signal",
        "passed": completed and not acceptance_failures,
    }


def _configure_native_runtime(args: argparse.Namespace) -> Path:
    sonic_root = args.sonic_root.resolve()
    required = (
        sonic_root / "gear_sonic/scripts/run_sim_loop.py",
        sonic_root / "gear_sonic/utils/mujoco_sim/configs.py",
        sonic_root / "gear_sonic/utils/teleop/zmq/zmq_planner_sender.py",
        sonic_root / "gear_sonic_deploy/target/release/g1_deploy_onnx_ref",
        sonic_root / "gear_sonic_deploy/policy/release/observation_config.yaml",
        sonic_root / "gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/config/fastrtps_profile.xml",
    )
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise SystemExit(
            "Native SONIC checkout/runtime is incomplete:\n  " + "\n  ".join(missing)
        )
    motion_data = sonic_root / "gear_sonic_deploy/reference/example"
    if not motion_data.is_dir():
        raise SystemExit(f"Native SONIC motion data is missing: {motion_data}")
    sys.path.insert(0, str(sonic_root))
    return sonic_root


def _sonic_commit(sonic_root: Path) -> str:
    marker = sonic_root / "SONIC_COMMIT"
    if marker.is_file():
        return marker.read_text(encoding="utf-8").strip().splitlines()[0]
    result = subprocess.run(
        ["git", "-C", str(sonic_root), "rev-parse", "HEAD"],
        text=True,
        capture_output=True,
        check=False,
    )
    return result.stdout.strip() if result.returncode == 0 else "unknown"


def _native_config_kwargs(args: argparse.Namespace, model_path: Path) -> dict[str, Any]:
    release_enabled = True
    return {
        "interface": args.dds_interface,
        "sim_frequency": int(round(args.physics_hz)),
        "enable_onscreen": False,
        "enable_offscreen": False,
        "robot_scene": str(model_path),
        "elastic_band_release_enabled": release_enabled,
        "elastic_band_hold_seconds": args.startup_band_hold if args.startup_band else 0.0,
        "elastic_band_fade_seconds": args.startup_band_fade if args.startup_band else 0.0,
        "elastic_band_wait_for_lowcmd": bool(args.startup_band),
        "low_cmd_fresh_timeout_seconds": args.low_cmd_fresh_timeout_seconds,
        "with_hands": False,
        "reset_on_fall": False,
    }


def _loopback_zmq_port(endpoint: str) -> int:
    try:
        parsed = urlsplit(endpoint)
        port = parsed.port
    except ValueError as exc:
        raise ValueError(f"invalid planner endpoint: {endpoint}") from exc
    if parsed.scheme != "tcp" or parsed.path or parsed.query or parsed.fragment:
        raise ValueError(f"planner endpoint must be tcp://HOST:PORT: {endpoint}")
    if parsed.hostname not in {"127.0.0.1", "localhost", "::1"}:
        raise ValueError(f"planner endpoint must bind loopback: {endpoint}")
    if port is None or not 1 <= port <= 65535:
        raise ValueError(f"planner endpoint has invalid port: {endpoint}")
    return port


class NativePlannerClient:
    """Thin socket lifecycle around SONIC's canonical ZMQ wire builders."""

    def __init__(
        self,
        endpoint: str,
        *,
        zmq_module,
        build_command_message: Callable[..., bytes],
        build_planner_message: Callable[..., bytes],
    ) -> None:
        self._build_command_message = build_command_message
        self._build_planner_message = build_planner_message
        self._context = zmq_module.Context.instance()
        self._socket = self._context.socket(zmq_module.PUB)
        try:
            self._socket.setsockopt(zmq_module.LINGER, 0)
            self._socket.bind(endpoint)
        except Exception:
            self._socket.close(linger=0)
            raise
        self._heading: float | None = None

    @staticmethod
    def _wrap_angle(value: float) -> float:
        return math.atan2(math.sin(value), math.cos(value))

    def send_velocity(
        self,
        vx: float,
        vy: float,
        yaw_rate: float,
        *,
        dt: float,
        start: bool = True,
    ) -> None:
        if self._heading is None:
            # SONIC normalizes the planner context to a zero-yaw frame. Keep
            # this command frame independent from a Matrix XML world spawn.
            self._heading = 0.0
        self._heading = self._wrap_angle(self._heading + yaw_rate * dt)
        speed = math.hypot(vx, vy)
        moving = speed > 1e-6
        if moving:
            local_x = vx / speed
            local_y = vy / speed
            cosine = math.cos(self._heading)
            sine = math.sin(self._heading)
            movement = [
                cosine * local_x - sine * local_y,
                sine * local_x + cosine * local_y,
                0.0,
            ]
        else:
            movement = [0.0, 0.0, 0.0]
        facing = [math.cos(self._heading), math.sin(self._heading), 0.0]
        self._socket.send(
            self._build_command_message(
                start=start,
                stop=False,
                planner=True,
                delta_heading=None,
            )
        )
        self._socket.send(
            self._build_planner_message(
                mode=2 if moving else 0,
                movement=movement,
                facing=facing,
                speed=speed if moving else -1.0,
                height=-1.0,
            )
        )

    def close(self) -> None:
        stop_error = None
        try:
            stop_message = self._build_command_message(
                start=False,
                stop=True,
                planner=True,
                delta_heading=None,
            )
            # The native deploy binary exits through its ZMQ stop state and
            # does not install SIGTERM handlers. Repeat the native stop frame
            # briefly before the supervisor escalates to process-group signals.
            for _ in range(3):
                self._socket.send(stop_message)
                time.sleep(0.02)
        except Exception as exc:
            stop_error = exc
        finally:
            self._socket.close(linger=0)
        if stop_error is not None:
            raise RuntimeError(f"failed to send native planner stop: {stop_error}")


class NativeProcessGroup:
    """Own native SONIC deploy/PICO children without touching unrelated processes."""

    def __init__(self, sonic_root: Path, env: dict[str, str]) -> None:
        self.sonic_root = sonic_root
        self.env = env
        self.guardian = Path(__file__).with_name(
            "exec_with_parent_death_signal.py"
        ).resolve()
        if not self.guardian.is_file():
            raise RuntimeError(f"native parent-death guardian is missing: {self.guardian}")
        profile = (
            sonic_root
            / "gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/config/fastrtps_profile.xml"
        )
        self.env["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(profile)
        self.env["ROS_LOCALHOST_ONLY"] = "1"
        self.env["PYTHONNOUSERSITE"] = "1"
        existing_pythonpath = self.env.get("PYTHONPATH")
        self.env["PYTHONPATH"] = str(sonic_root) + (
            os.pathsep + existing_pythonpath if existing_pythonpath else ""
        )
        self.pass_fds: tuple[int, ...] = ()
        lock_fd_value = self.env.get("MATRIX_SONIC_HOST_LOCK_FD")
        if lock_fd_value:
            try:
                lock_fd = int(lock_fd_value)
                os.fstat(lock_fd)
            except (OSError, ValueError) as exc:
                raise RuntimeError(
                    f"invalid MATRIX_SONIC_HOST_LOCK_FD={lock_fd_value!r}: {exc}"
                ) from exc
            self.pass_fds = (lock_fd,)
        self.children: list[tuple[str, subprocess.Popen[bytes]]] = []
        self._closed = False

    def _start(self, name: str, command: list[str], cwd: Path) -> None:
        guarded_command = [
            sys.executable,
            str(self.guardian),
            "--expected-parent",
            str(os.getpid()),
            "--",
            *command,
        ]
        process = subprocess.Popen(
            guarded_command,
            cwd=cwd,
            env=self.env,
            pass_fds=self.pass_fds,
            start_new_session=True,
        )
        self.children.append((name, process))

    def start_pico(self, python: str, *, port: int) -> None:
        self._start(
            "pico-manager",
            [
                python,
                "-u",
                str(self.sonic_root / "gear_sonic/scripts/pico_manager_thread_server.py"),
                "--manager",
                "--port",
                str(port),
            ],
            self.sonic_root,
        )

    def start_deploy(self, *, interface: str, zmq_port: int) -> None:
        deploy_root = self.sonic_root / "gear_sonic_deploy"
        self._start(
            "deploy",
            [
                str(deploy_root / "target/release/g1_deploy_onnx_ref"),
                interface,
                "policy/release/model_decoder.onnx",
                "reference/example",
                "--obs-config",
                "policy/release/observation_config.yaml",
                "--encoder-file",
                "policy/release/model_encoder.onnx",
                "--planner-file",
                "planner/target_vel/V2/planner_sonic.onnx",
                "--input-type",
                "zmq_manager",
                "--output-type",
                "all",
                "--zmq-host",
                "localhost",
                "--zmq-port",
                str(zmq_port),
                "--disable-crc-check",
            ],
            deploy_root,
        )

    def failed_child(self) -> tuple[str, int] | None:
        for name, process in self.children:
            code = process.poll()
            if code is not None:
                return name, code
        return None

    def wait_for_child(self, name: str, *, timeout: float) -> bool:
        """Give a native child time to finish its own stop/cleanup path."""
        matching = [process for child_name, process in self.children if child_name == name]
        if not matching:
            return True
        deadline = time.monotonic() + max(timeout, 0.0)
        while any(process.poll() is None for process in matching):
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            time.sleep(min(0.05, remaining))
        return True

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        process_groups = {process.pid for _, process in self.children}
        for process_group in process_groups:
            try:
                # The session leader may already have exited while one of its
                # descendants remains. Always address the whole process group.
                os.killpg(process_group, signal.SIGTERM)
            except (ProcessLookupError, PermissionError):
                pass

        remaining_groups = self._wait_for_groups(
            process_groups, deadline=time.monotonic() + 5.0
        )
        for process_group in remaining_groups:
            try:
                os.killpg(process_group, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
        surviving_groups = self._wait_for_groups(
            remaining_groups, deadline=time.monotonic() + 2.0
        )

        for _, process in reversed(self.children):
            try:
                process.wait(timeout=0.0)
            except subprocess.TimeoutExpired:
                # Both group waits above are bounded; never hang cleanup on a
                # misbehaving or unobservable child.
                pass
        if surviving_groups:
            formatted = ",".join(str(value) for value in sorted(surviving_groups))
            raise RuntimeError(f"native process groups survived SIGKILL: {formatted}")

    def _wait_for_groups(self, groups: set[int], *, deadline: float) -> set[int]:
        remaining = set(groups)
        while remaining:
            for _, process in self.children:
                process.poll()
            alive = set()
            for process_group in remaining:
                try:
                    os.killpg(process_group, 0)
                except ProcessLookupError:
                    pass
                except PermissionError:
                    alive.add(process_group)
                else:
                    alive.add(process_group)
            remaining = alive
            wait_s = min(0.05, deadline - time.monotonic())
            if not remaining or wait_s <= 0.0:
                break
            time.sleep(wait_s)
        return remaining


def _close_runtime_resource(name: str, resource) -> str | None:
    if resource is None:
        return None
    try:
        resource.close()
    except Exception as exc:
        print(f"matrix-sonic-runtime ERROR closing {name}: {exc}", flush=True)
        return f"{name}: {exc}"
    return None


def _record_cleanup_failure(path: Path | None, errors: list[str]) -> None:
    if path is None or not path.is_file():
        return
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        print(
            f"matrix-sonic-runtime ERROR reading status for cleanup failure: {exc}",
            flush=True,
        )
        return
    if not isinstance(payload, dict):
        return
    failures = payload.get("acceptance_failures")
    if not isinstance(failures, list):
        failures = []
    if "cleanup_failure" not in failures:
        failures.append("cleanup_failure")
    payload["acceptance_failures"] = failures
    payload["cleanup_errors"] = errors
    payload["passed"] = False
    payload["pre_cleanup_termination_reason"] = payload.get("termination_reason")
    payload["termination_reason"] = "cleanup_failure"
    _atomic_json(path, payload)


def main() -> int:
    args = _parse_args()
    args.verification_receipt_sha256 = None
    _arm_supervisor_parent_death(args.expected_parent_pid)
    run_id = uuid.uuid4().hex
    model_path = args.model.resolve()
    if not model_path.is_file():
        raise SystemExit(f"composed Matrix model is missing: {model_path}")
    if args.physics_hz <= 0.0 or args.control_hz <= 0.0:
        raise SystemExit("--physics-hz and --control-hz must be positive")
    if not math.isfinite(args.max_seconds) or args.max_seconds < 0.0:
        raise SystemExit("--max-seconds must be non-negative and finite")
    if args.min_active_seconds < 0.0:
        raise SystemExit("--min-active-seconds must be non-negative")
    if not math.isfinite(args.min_displacement_m) or args.min_displacement_m < 0.0:
        raise SystemExit("--min-displacement-m must be non-negative and finite")
    if args.min_final_x is not None and not math.isfinite(args.min_final_x):
        raise SystemExit("--min-final-x must be finite")
    if not math.isfinite(args.min_forward_x_m) or args.min_forward_x_m < 0.0:
        raise SystemExit("--min-forward-x-m must be non-negative and finite")
    if args.min_physics_hz < 0.0 or args.min_rtf < 0.0:
        raise SystemExit("--min-physics-hz and --min-rtf must be non-negative")
    if args.max_resets < 0:
        raise SystemExit("--max-resets must be non-negative")
    sha256_pattern = r"[0-9a-f]{64}"
    if args.qualified_runtime:
        if not args.qualification_profile:
            raise SystemExit("--qualified-runtime requires --qualification-profile")
        if re.fullmatch(sha256_pattern, args.runtime_lock_sha256 or "") is None:
            raise SystemExit("--qualified-runtime requires --runtime-lock-sha256")
        if re.fullmatch(r"[0-9a-f]{40}", args.matrix_commit or "") is None:
            raise SystemExit("--qualified-runtime requires --matrix-commit")
        if args.verification_receipt is None:
            raise SystemExit("--qualified-runtime requires --verification-receipt")
        if (
            args.scenario_layout_sha256 is not None
            and re.fullmatch(sha256_pattern, args.scenario_layout_sha256) is None
        ):
            raise SystemExit("--scenario-layout-sha256 must be a lowercase SHA256")
    elif any(
        value is not None
        for value in (
            args.qualification_profile,
            args.runtime_lock_sha256,
            args.scenario_layout_sha256,
            args.matrix_commit,
            args.verification_receipt,
        )
    ):
        raise SystemExit("qualification metadata requires --qualified-runtime")
    _validate_qualification_receipt(args)
    if (
        not math.isfinite(args.low_cmd_fresh_timeout_seconds)
        or args.low_cmd_fresh_timeout_seconds <= 0.0
    ):
        raise SystemExit("--low-cmd-fresh-timeout-seconds must be positive and finite")
    if args.dds_interface != "lo":
        raise SystemExit("native Matrix SONIC requires --dds-interface lo")
    try:
        planner_port = _loopback_zmq_port(args.planner_bind)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    if args.startup_band_hold < 0.0 or args.startup_band_fade < 0.0:
        raise SystemExit("startup band hold/fade durations must be non-negative")
    sonic_root = _configure_native_runtime(args)
    sonic_commit = _sonic_commit(sonic_root)
    try:
        import numpy as np
        import zmq
        from gear_sonic.scripts.run_sim_loop import create_simulator
        from gear_sonic.utils.mujoco_sim.configs import SimLoopConfig
        from gear_sonic.utils.teleop.zmq.zmq_planner_sender import (
            build_command_message,
            build_planner_message,
        )
        from matrix_render_protocol import MatrixRenderPublisher, packet_size
    except ImportError as exc:
        raise SystemExit(
            f"Native SONIC runtime dependency is missing: {exc}. "
            "Use the SONIC commit pinned by config/runtime/matrix-sonic.lock.json."
        ) from exc

    config = SimLoopConfig(**_native_config_kwargs(args, model_path))
    simulator = create_simulator(config)
    renderer = None
    planner = None
    processes = None
    previous_signal_handlers: dict[int, Any] = {}
    running = True
    termination_reason: str | None = None
    termination_signal: int | None = None
    child_failure: tuple[str, int] | None = None
    numerical_error: str | None = None

    def request_stop(signum, _frame) -> None:
        nonlocal running, termination_reason, termination_signal
        running = False
        if termination_reason is None:
            termination_reason = "signal"
            termination_signal = int(signum)

    try:
        # Install handlers before any child is started. Everything after the
        # simulator construction is inside this cleanup boundary.
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handler = signal.getsignal(signum)
            signal.signal(signum, request_stop)
            previous_signal_handlers[int(signum)] = previous_handler

        snapshot = simulator.get_state_snapshot()
        initial_snapshot_error = _snapshot_validation_error(snapshot)
        if initial_snapshot_error is not None:
            raise SystemExit(
                f"invalid native SONIC initial snapshot: {initial_snapshot_error}"
            )
        qpos = snapshot.qpos

        physics_hz = float(args.physics_hz)
        substeps_float = physics_hz / args.control_hz
        substeps = int(round(substeps_float))
        if substeps <= 0 or not math.isclose(
            substeps_float, substeps, rel_tol=0.0, abs_tol=1e-6
        ):
            raise SystemExit(
                f"control_hz={args.control_hz} must divide "
                f"physics_hz={physics_hz} exactly"
            )

        initial_root_xy = np.asarray(qpos[:2], dtype=np.float64).copy()
        renderer = (
            None
            if args.no_render_sync
            else MatrixRenderPublisher(args.render_host, args.render_port)
        )
        processes = NativeProcessGroup(sonic_root, os.environ.copy())
        if running and args.control_source == "planner":
            planner = NativePlannerClient(
                args.planner_bind,
                zmq_module=zmq,
                build_command_message=build_command_message,
                build_planner_message=build_planner_message,
            )
        elif running and args.control_source == "pico":
            processes.start_pico(
                args.pico_python or sys.executable, port=planner_port
            )
        if running:
            processes.start_deploy(
                interface=args.dds_interface, zmq_port=planner_port
            )

        def poll_failed_child() -> bool:
            nonlocal child_failure, running, termination_reason
            failure = processes.failed_child()
            if failure is None:
                return False
            if child_failure is None:
                child_failure = failure
                name, code = failure
                print(
                    "matrix-sonic-runtime ERROR native SONIC child exited: "
                    f"{name}={code}",
                    flush=True,
                )
            running = False
            if termination_reason != "numerical_instability":
                termination_reason = "child_exit"
            return True

        expected_packet_size = packet_size(
            nq=len(snapshot.qpos), nv=len(snapshot.qvel), nu=len(snapshot.ctrl)
        )
        render_target = (
            "disabled"
            if renderer is None
            else f"{args.render_host}:{args.render_port}"
        )
        print(
            "matrix-sonic-runtime "
            f"backend=gear_sonic_native sonic_commit={sonic_commit} "
            f"model={model_path} nq={len(snapshot.qpos)} nv={len(snapshot.qvel)} "
            f"nu={len(snapshot.ctrl)} physics_hz={physics_hz:.1f} "
            f"control_hz={args.control_hz:.1f} substeps={substeps} "
            f"render={render_target} packet_bytes={expected_packet_size} "
            f"control_source={args.control_source}",
            flush=True,
        )

        started_wall = time.perf_counter()
        physics_period_s = 1.0 / physics_hz
        next_physics_wall = started_wall + physics_period_s
        next_print = started_wall
        last_print_wall = started_wall
        last_render_count = 0
        last_physics_steps = 0
        control_frames = 0
        active_frames = 0
        physics_steps = 0
        instability_resets = int(snapshot.reset_count)
        unstable = False
        fall_detected = bool(snapshot.fall_detected)
        min_root_z = float(snapshot.qpos[2])
        active_started_wall = None
        longest_active_elapsed_s = 0.0
        walking = False
        while running:
            # Poll on both sides of the duration gate so a child exit at the
            # acceptance boundary cannot be mistaken for normal completion.
            if poll_failed_child():
                break
            frame_wall = time.perf_counter()
            elapsed_wall = frame_wall - started_wall
            if args.max_seconds > 0.0 and elapsed_wall >= args.max_seconds:
                termination_reason = "max_seconds"
                poll_failed_child()
                break

            active_elapsed = (
                frame_wall - active_started_wall
                if active_started_wall is not None
                else 0.0
            )
            walking = (
                active_started_wall is not None
                and args.walk_after >= 0.0
                and active_elapsed >= args.walk_after
            )
            if planner is not None:
                planner.send_velocity(
                    args.vx if walking else 0.0,
                    args.vy if walking else 0.0,
                    args.yaw_rate if walking else 0.0,
                    dt=1.0 / args.control_hz,
                )

            for _ in range(substeps):
                if not running:
                    break
                # Keep native DDS lowstate and MuJoCo cadence at 200 Hz instead
                # of emitting four back-to-back steps once per 50 Hz frame.
                # Matrix owns the absolute deadline because SONIC's relative
                # per-step sleep otherwise accumulates scheduler overshoot.
                next_snapshot = simulator.step_once(rate_limit=False)
                physics_steps += 1
                step_error = _snapshot_validation_error(next_snapshot, snapshot)
                if step_error is not None:
                    unstable = True
                    running = False
                    termination_reason = "numerical_instability"
                    numerical_error = step_error
                    print(
                        "matrix-sonic-runtime ERROR invalid native SONIC step: "
                        f"{step_error}",
                        flush=True,
                    )
                    break
                snapshot = next_snapshot
                instability_resets = int(snapshot.reset_count)
                if args.fail_on_fall and bool(snapshot.fall_detected):
                    fall_detected = True
                    running = False
                    termination_reason = "fall_detected"
                    break
                if instability_resets > args.max_resets:
                    running = False
                    termination_reason = "reset_detected"
                    break
                next_physics_wall = _pace_absolute_deadline(
                    next_physics_wall, physics_period_s
                )

            if not running:
                break
            if poll_failed_child():
                break

            active_lowcmd = bool(snapshot.low_cmd_fresh)
            low_cmd_age_s = snapshot.low_cmd_age_s
            freshness_sample_wall = time.perf_counter()
            if active_lowcmd:
                if active_started_wall is None:
                    active_started_wall = freshness_sample_wall
                active_elapsed = freshness_sample_wall - active_started_wall
                longest_active_elapsed_s = max(
                    longest_active_elapsed_s, active_elapsed
                )
                active_frames += 1
            else:
                active_started_wall = None
                active_elapsed = 0.0

            root_z = float(snapshot.qpos[2])
            root_up_z = _root_up_z(snapshot.qpos)
            min_root_z = min(min_root_z, root_z)
            # SONIC is the sole fall authority. Height and orientation remain
            # diagnostics only, so Matrix cannot disagree with its snapshot.
            fall_detected = fall_detected or bool(snapshot.fall_detected)

            if renderer is not None:
                renderer.send(
                    snapshot.sim_time,
                    snapshot.qpos,
                    snapshot.qvel,
                    snapshot.ctrl,
                )
            control_frames += 1

            now = time.perf_counter()
            if now >= next_print:
                window_wall = max(now - last_print_wall, 1e-9)
                render_count = renderer.packet_count if renderer is not None else 0
                window_render = render_count - last_render_count
                window_physics_steps = physics_steps - last_physics_steps
                status = {
                    "active_elapsed_s": round(
                        now - active_started_wall, 3
                    ) if active_started_wall is not None else 0.0,
                    "active_lowcmd_longest_s": round(longest_active_elapsed_s, 3),
                    "active_lowcmd": active_lowcmd,
                    "backend": "gear_sonic_native",
                    "control_frames": control_frames,
                    "control_hz": args.control_hz,
                    "elapsed_wall_s": round(now - started_wall, 3),
                    "model": str(model_path),
                    "nu": len(snapshot.ctrl),
                    "low_cmd_age_s": (
                        round(float(low_cmd_age_s), 6)
                        if low_cmd_age_s is not None
                        else None
                    ),
                    "low_cmd_fresh_timeout_s": args.low_cmd_fresh_timeout_seconds,
                    "low_cmd_received": bool(snapshot.low_cmd_received),
                    "fall_detected": fall_detected,
                    "min_displacement_m": args.min_displacement_m,
                    "min_final_x": args.min_final_x,
                    "min_forward_x_m": args.min_forward_x_m,
                    "min_root_z": round(min_root_z, 5),
                    "physics_hz_target": physics_hz,
                    "physics_step_hz": round(window_physics_steps / window_wall, 3),
                    "render_hz": round(window_render / window_wall, 3),
                    "render_packet_bytes": expected_packet_size,
                    "render_sync_enabled": renderer is not None,
                    "ue_state_sync_hz": round(window_render / window_wall, 3),
                    "root_xyz": [round(float(value), 5) for value in snapshot.qpos[:3]],
                    "root_displacement_xy_m": round(
                        float(
                            np.linalg.norm(
                                np.asarray(snapshot.qpos[:2]) - initial_root_xy
                            )
                        ),
                        5,
                    ),
                    "root_up_z": round(root_up_z, 5),
                    "run_id": run_id,
                    "runtime_lock_sha256": args.runtime_lock_sha256,
                    "runtime_verified": bool(args.qualified_runtime),
                    "qualification_profile": args.qualification_profile,
                    "scenario_layout_sha256": args.scenario_layout_sha256,
                    "matrix_commit": args.matrix_commit,
                    "verification_receipt": (
                        str(args.verification_receipt)
                        if args.verification_receipt is not None
                        else None
                    ),
                    "verification_receipt_sha256": args.verification_receipt_sha256,
                    "rtf": round(
                        (window_physics_steps / physics_hz) / window_wall,
                        4,
                    ),
                    "sim_time_s": round(float(snapshot.sim_time), 4),
                    "sonic_commit": sonic_commit,
                    "sonic_step_index": int(snapshot.step_index),
                    "instability_resets": instability_resets,
                    "last_reset_reason": snapshot.last_reset_reason,
                    "max_resets": args.max_resets,
                    "startup_band_enabled": bool(args.startup_band),
                    "startup_band_hold_s": args.startup_band_hold,
                    "startup_band_fade_s": args.startup_band_fade,
                    "startup_band_scale": round(float(snapshot.elastic_band_scale), 5),
                    "walking_commanded": walking,
                }
                print(
                    f"matrix-sonic-runtime status={json.dumps(status, sort_keys=True)}",
                    flush=True,
                )
                _atomic_json(args.status_file, status)
                last_print_wall = now
                last_render_count = render_count
                last_physics_steps = physics_steps
                next_print = now + max(args.print_every, 0.1)

        # One final poll closes the race between the last loop poll and final
        # status publication, including the max_seconds boundary.
        poll_failed_child()
        if termination_reason is None:
            termination_reason = "signal" if not running else "unknown"

        finished_wall = time.perf_counter()
        elapsed_wall_s = finished_wall - started_wall
        active_lowcmd = bool(snapshot.low_cmd_fresh)
        active_elapsed_s = 0.0
        if active_lowcmd and active_started_wall is not None:
            active_elapsed_s = finished_wall - active_started_wall
        longest_active_elapsed_s = max(longest_active_elapsed_s, active_elapsed_s)
        physics_step_hz_aggregate = physics_steps / max(elapsed_wall_s, 1e-9)
        rtf_aggregate = (physics_steps / physics_hz) / max(elapsed_wall_s, 1e-9)
        render_count = renderer.packet_count if renderer is not None else 0
        render_hz_aggregate = render_count / max(elapsed_wall_s, 1e-9)
        root_z = float(snapshot.qpos[2])
        root_up_z = _root_up_z(snapshot.qpos)
        min_root_z = min(min_root_z, root_z)
        fall_detected = fall_detected or bool(snapshot.fall_detected)
        instability_resets = int(snapshot.reset_count)
        root_displacement_xy_m = float(
            np.linalg.norm(np.asarray(snapshot.qpos[:2]) - initial_root_xy)
        )
        acceptance_failures = _acceptance_failures(
            unstable=unstable,
            fall_detected=fall_detected,
            fail_on_fall=args.fail_on_fall,
            active_lowcmd=active_lowcmd,
            active_elapsed_s=longest_active_elapsed_s,
            min_active_seconds=args.min_active_seconds,
            physics_step_hz=physics_step_hz_aggregate,
            min_physics_hz=args.min_physics_hz,
            rtf=rtf_aggregate,
            min_rtf=args.min_rtf,
            failed_child=child_failure,
            root_displacement_xy_m=root_displacement_xy_m,
            min_displacement_m=args.min_displacement_m,
            root_final_x=float(snapshot.qpos[0]),
            min_final_x=args.min_final_x,
            root_displacement_x_m=float(snapshot.qpos[0]) - float(initial_root_xy[0]),
            min_forward_x_m=args.min_forward_x_m,
            reset_count=instability_resets,
            max_resets=args.max_resets,
        )
        qualification = _qualification_state(
            max_seconds=args.max_seconds,
            termination_reason=termination_reason,
            failures=acceptance_failures,
            runtime_verified=bool(args.qualified_runtime),
        )
        acceptance_failures = qualification["acceptance_failures"]
        qualification_attempted = bool(qualification["qualification_attempted"])
        completed = bool(qualification["completed"])
        passed = bool(qualification["passed"])
        interrupted = bool(qualification["interrupted"])

        failed_child_name = child_failure[0] if child_failure is not None else None
        failed_child_code = child_failure[1] if child_failure is not None else None
        final_status = {
            "acceptance_failures": acceptance_failures,
            "active_elapsed_s": round(active_elapsed_s, 3),
            "active_frames": active_frames,
            "active_lowcmd": active_lowcmd,
            "active_lowcmd_longest_s": round(longest_active_elapsed_s, 3),
            "backend": "gear_sonic_native",
            "completed": completed,
            "control_frames": control_frames,
            "control_hz": args.control_hz,
            "elapsed_wall_s": round(elapsed_wall_s, 3),
            "failed_child_exit_code": failed_child_code,
            "failed_child_name": failed_child_name,
            "fall_detected": fall_detected,
            "instability_resets": instability_resets,
            "interrupted": interrupted,
            "last_reset_reason": snapshot.last_reset_reason,
            "low_cmd_age_s": (
                round(float(snapshot.low_cmd_age_s), 6)
                if snapshot.low_cmd_age_s is not None
                else None
            ),
            "low_cmd_fresh_timeout_s": args.low_cmd_fresh_timeout_seconds,
            "low_cmd_received": bool(snapshot.low_cmd_received),
            "min_physics_hz": args.min_physics_hz,
            "min_displacement_m": args.min_displacement_m,
            "min_final_x": args.min_final_x,
            "min_forward_x_m": args.min_forward_x_m,
            "min_root_z": round(min_root_z, 5),
            "min_rtf": args.min_rtf,
            "max_resets": args.max_resets,
            "matrix_commit": args.matrix_commit,
            "verification_receipt": (
                str(args.verification_receipt)
                if args.verification_receipt is not None
                else None
            ),
            "verification_receipt_sha256": args.verification_receipt_sha256,
            "model": str(model_path),
            "nq": len(snapshot.qpos),
            "nu": len(snapshot.ctrl),
            "numerical_error": numerical_error,
            "nv": len(snapshot.qvel),
            "passed": passed,
            "physics_hz_target": physics_hz,
            "physics_step_hz": round(physics_step_hz_aggregate, 3),
            "physics_step_hz_aggregate": round(physics_step_hz_aggregate, 3),
            "physics_steps": physics_steps,
            "qualification_attempted": qualification_attempted,
            "qualification_profile": args.qualification_profile,
            "render_hz": round(render_hz_aggregate, 3),
            "render_packet_bytes": expected_packet_size,
            "render_sync_enabled": renderer is not None,
            "root_displacement_xy_m": round(root_displacement_xy_m, 5),
            "root_final_x": round(float(snapshot.qpos[0]), 5),
            "root_displacement_x_m": round(
                float(snapshot.qpos[0]) - float(initial_root_xy[0]), 5
            ),
            "root_up_z": round(root_up_z, 5),
            "root_xyz": [round(float(value), 5) for value in snapshot.qpos[:3]],
            "rtf": round(rtf_aggregate, 4),
            "rtf_aggregate": round(rtf_aggregate, 4),
            "run_id": run_id,
            "runtime_lock_sha256": args.runtime_lock_sha256,
            "runtime_verified": bool(args.qualified_runtime),
            "scenario_layout_sha256": args.scenario_layout_sha256,
            "sim_time_s": round(float(snapshot.sim_time), 4),
            "sonic_commit": sonic_commit,
            "sonic_step_index": int(snapshot.step_index),
            "startup_band_enabled": bool(args.startup_band),
            "startup_band_fade_s": args.startup_band_fade,
            "startup_band_hold_s": args.startup_band_hold,
            "startup_band_scale": round(float(snapshot.elastic_band_scale), 5),
            "termination_reason": termination_reason,
            "termination_signal": termination_signal,
            "ue_state_sync_hz": round(render_hz_aggregate, 3),
            "walking_commanded": walking,
        }
        _atomic_json(args.status_file, final_status)
        print(
            "matrix-sonic-runtime stopped "
            f"wall_s={elapsed_wall_s:.2f} sim_s={snapshot.sim_time:.2f} "
            f"frames={control_frames} active_frames={active_frames} "
            f"reason={termination_reason} passed={passed} "
            f"failures={acceptance_failures}",
            flush=True,
        )
        if passed or (not qualification_attempted and interrupted and not acceptance_failures):
            return 0
        return 2
    finally:
        active_exception = sys.exc_info()[0] is not None
        cleanup_errors = []
        error = _close_runtime_resource("planner", planner)
        if error is not None:
            cleanup_errors.append(error)
        if planner is not None and processes is not None:
            try:
                processes.wait_for_child("deploy", timeout=2.0)
            except Exception as exc:
                print(
                    "matrix-sonic-runtime ERROR waiting for native deploy stop: "
                    f"{exc}",
                    flush=True,
                )
                cleanup_errors.append(f"native deploy stop wait: {exc}")
        for name, resource in (
            ("native processes", processes),
            ("renderer", renderer),
            ("simulator", simulator),
        ):
            error = _close_runtime_resource(name, resource)
            if error is not None:
                cleanup_errors.append(error)
        for signum, previous_handler in previous_signal_handlers.items():
            try:
                signal.signal(signum, previous_handler)
            except (OSError, ValueError) as exc:
                print(
                    "matrix-sonic-runtime ERROR restoring signal handler "
                    f"{signum}: {exc}",
                    flush=True,
                )
                cleanup_errors.append(
                    f"signal handler {signum}: {exc}"
                )
        if cleanup_errors:
            _record_cleanup_failure(args.status_file, cleanup_errors)
            if not active_exception:
                raise RuntimeError(
                    "native cleanup failed: " + "; ".join(cleanup_errors)
                )


if __name__ == "__main__":
    raise SystemExit(main())
