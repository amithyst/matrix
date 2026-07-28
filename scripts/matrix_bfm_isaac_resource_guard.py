#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import asdict, dataclass
import fcntl
import json
import math
import os
from pathlib import Path
import secrets
import signal
import subprocess
import time
from typing import Sequence

from matrix_bfm_isaac_instance_ledger import (
    NONCE_ENV,
    ProcessIdentity,
    atomic_write,
    process_identity,
    process_nonce,
)


DEFAULT_FOREIGN_ROOTS = (
    "/home/trna/matrix",
)

ANCHOR_ROLES = (
    "run_matrix_sonic.sh",
    "run_sim.sh",
    "supervise_matrix_ue.py",
    "zsibot_mujoco_ue",
    "robot_mujoco",
    "run_matrix_sonic.py",
)
SCRIPT_INTERPRETERS = {
    "bash",
    "dash",
    "python",
    "python3",
    "sh",
    "zsh",
}


@dataclass(frozen=True, slots=True)
class ForeignProcess:
    pid: int
    executable: str
    cwd: str
    command: str


@dataclass(frozen=True, slots=True)
class ResourceSnapshot:
    allowed: bool
    reason: str
    free_vram_mib: int | None
    available_ram_mib: int
    load_1m: float
    cpu_count: int
    host_udp_9999_busy: bool
    foreign_processes: tuple[ForeignProcess, ...]

    def to_json(self) -> str:
        return json.dumps(asdict(self), indent=2, sort_keys=True)


def _read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace")
    except (FileNotFoundError, PermissionError, ProcessLookupError, OSError):
        return ""


def _read_link(path: Path) -> str:
    try:
        return os.readlink(path)
    except (FileNotFoundError, PermissionError, ProcessLookupError, OSError):
        return ""


def _matches_root(value: str, root: str) -> bool:
    if not value:
        return False
    if root.endswith("-"):
        return value.startswith(root)
    return value == root or value.startswith(root + "/")


def _is_anchor(value: str) -> bool:
    basename = Path(value).name
    return any(basename == role or basename.startswith(role + "-") for role in ANCHOR_ROLES)


def _absolute_candidate(value: str, cwd: str) -> str:
    if not value:
        return ""
    path = Path(value)
    if path.is_absolute():
        return os.path.normpath(value)
    if not cwd:
        return ""
    return os.path.normpath(str(Path(cwd) / path))


def _invocation_candidates(
    executable: str,
    cwd: str,
    tokens: Sequence[str],
) -> tuple[str, ...]:
    """Return only paths that can own the process, never arbitrary arguments.

    A process can be anchored by its native executable, argv[0], or the script
    passed to a known interpreter.  Treating every argv token as executable
    provenance causes read-only commands such as ``rg run_sim.sh`` to look like
    a running Matrix instance.
    """

    candidates = [executable]
    if tokens:
        candidates.append(_absolute_candidate(tokens[0], cwd))
        executable_name = Path(tokens[0]).name
        if executable_name in SCRIPT_INTERPRETERS:
            for token in tokens[1:]:
                if token in {"-c", "-m"}:
                    break
                if token.startswith("-"):
                    continue
                candidates.append(_absolute_candidate(token, cwd))
                break
    return tuple(dict.fromkeys(candidate for candidate in candidates if candidate))


def scan_foreign_processes(
    *,
    own_root: str,
    proc_root: Path = Path("/proc"),
    foreign_roots: Sequence[str] = DEFAULT_FOREIGN_ROOTS,
) -> tuple[ForeignProcess, ...]:
    found: list[ForeignProcess] = []
    try:
        entries = tuple(proc_root.iterdir())
    except OSError:
        return ()
    for entry in entries:
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        executable = _read_link(entry / "exe")
        cwd = _read_link(entry / "cwd")
        try:
            raw = (entry / "cmdline").read_bytes()
        except (FileNotFoundError, PermissionError, ProcessLookupError, OSError):
            raw = b""
        tokens = tuple(
            token.decode("utf-8", errors="replace")
            for token in raw.split(b"\0")
            if token
        )
        candidates = _invocation_candidates(executable, cwd, tokens)
        if any(_matches_root(candidate, own_root) for candidate in candidates):
            continue
        rooted_anchor = any(
            _matches_root(candidate, root) and _is_anchor(candidate)
            for candidate in candidates
            for root in foreign_roots
        )
        if not rooted_anchor:
            continue
        found.append(
            ForeignProcess(
                pid=pid,
                executable=executable,
                cwd=cwd,
                command=" ".join(tokens),
            )
        )
    return tuple(sorted(found, key=lambda item: item.pid))


def query_free_vram_mib() -> int | None:
    try:
        result = subprocess.run(
            [
                "nvidia-smi",
                "--query-gpu=memory.free",
                "--format=csv,noheader,nounits",
            ],
            check=True,
            capture_output=True,
            text=True,
            timeout=5,
        )
        return min(int(line.strip()) for line in result.stdout.splitlines() if line.strip())
    except (OSError, ValueError, subprocess.SubprocessError):
        return None


def available_ram_mib(meminfo: Path = Path("/proc/meminfo")) -> int:
    for line in _read_text(meminfo).splitlines():
        if line.startswith("MemAvailable:"):
            return int(line.split()[1]) // 1024
    return 0


def host_udp_9999_busy() -> bool:
    try:
        result = subprocess.run(
            ["ss", "-H", "-lun", "sport = :9999"],
            check=False,
            capture_output=True,
            text=True,
            timeout=5,
        )
        return bool(result.stdout.strip())
    except (OSError, subprocess.SubprocessError):
        return False


def snapshot(
    *,
    own_root: str,
    min_free_vram_mib: int,
    min_available_ram_mib: int,
    enforce_resources: bool = True,
    include_port_hint: bool = True,
) -> ResourceSnapshot:
    foreign = scan_foreign_processes(own_root=own_root)
    port_busy = host_udp_9999_busy() if include_port_hint else False
    free_vram = query_free_vram_mib()
    ram = available_ram_mib()
    load_1m = os.getloadavg()[0]
    cpu_count = os.cpu_count() or 1

    if foreign or port_busy:
        allowed, reason = False, "foreign_matrix_active"
    elif enforce_resources and free_vram is None:
        allowed, reason = False, "gpu_inventory_unavailable"
    elif enforce_resources and free_vram < min_free_vram_mib:
        allowed, reason = False, "insufficient_free_vram"
    elif enforce_resources and ram < min_available_ram_mib:
        allowed, reason = False, "insufficient_available_ram"
    else:
        allowed, reason = True, "resources_available"
    return ResourceSnapshot(
        allowed=allowed,
        reason=reason,
        free_vram_mib=free_vram,
        available_ram_mib=ram,
        load_1m=load_1m,
        cpu_count=cpu_count,
        host_udp_9999_busy=port_busy,
        foreign_processes=foreign,
    )


def write_status(path: Path, payload: dict[str, object]) -> None:
    atomic_write(path, payload)


def stop_process_group(process: subprocess.Popen[bytes], grace_s: float = 4.0) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=grace_s)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        return
    process.wait(timeout=5)


def nonce_processes(
    nonce: str,
    *,
    uid: int | None = None,
    proc_root: Path = Path("/proc"),
) -> tuple[ProcessIdentity, ...]:
    expected_uid = os.getuid() if uid is None else int(uid)
    try:
        entries = tuple(proc_root.iterdir())
    except OSError:
        return ()
    found = []
    for entry in entries:
        if not entry.name.isdigit():
            continue
        identity = process_identity(int(entry.name), proc_root)
        if identity is None or identity.uid != expected_uid:
            continue
        if process_nonce(identity.pid, proc_root) == nonce:
            found.append(identity)
    return tuple(sorted(found, key=lambda item: item.pid))


def signal_nonce_processes(nonce: str, signal_value: int) -> tuple[int, ...]:
    signaled = []
    for recorded in nonce_processes(nonce):
        pidfd = None
        try:
            pidfd = os.pidfd_open(recorded.pid)
            if process_identity(recorded.pid) != recorded:
                continue
            if process_nonce(recorded.pid) != nonce:
                continue
            signal.pidfd_send_signal(pidfd, signal_value)
            signaled.append(recorded.pid)
        except (ProcessLookupError, PermissionError):
            continue
        finally:
            if pidfd is not None:
                os.close(pidfd)
    return tuple(signaled)


def terminate_nonce_processes(nonce: str, grace_s: float = 3.0) -> tuple[int, ...]:
    targeted = list(signal_nonce_processes(nonce, signal.SIGTERM))
    deadline = time.monotonic() + max(0.0, grace_s)
    while time.monotonic() < deadline:
        if not nonce_processes(nonce):
            return tuple(dict.fromkeys(targeted))
        time.sleep(0.05)
    targeted.extend(signal_nonce_processes(nonce, signal.SIGKILL))
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and nonce_processes(nonce):
        time.sleep(0.05)
    remaining = nonce_processes(nonce)
    if remaining:
        raise RuntimeError(
            "nonce cleanup failed for PIDs: "
            + ",".join(str(item.pid) for item in remaining)
        )
    return tuple(dict.fromkeys(targeted))


def run_guarded(
    command: Sequence[str],
    *,
    own_root: str,
    status_path: Path,
    min_free_vram_mib: int,
    min_available_ram_mib: int,
    runtime_floor_vram_mib: int,
    runtime_floor_ram_mib: int,
    lock_path: Path,
    cleanup_command: Sequence[str] | None = None,
    interval_s: float = 0.25,
    shutdown_grace_s: float = 40.0,
) -> int:
    if not math.isfinite(interval_s) or interval_s <= 0.0:
        raise ValueError("resource guard interval must be positive and finite")
    if not math.isfinite(shutdown_grace_s) or shutdown_grace_s <= 0.0:
        raise ValueError("resource guard shutdown grace must be positive and finite")
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    lock_handle = lock_path.open("w")
    try:
        fcntl.flock(lock_handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock_handle.close()
        write_status(status_path, {"state": "yielded_before_start", "reason": "guard_locked"})
        return 75

    process: subprocess.Popen[bytes] | None = None
    run_nonce = secrets.token_hex(16)
    yielded: tuple[str, ResourceSnapshot] | None = None
    terminate_signal = 0
    previous_handlers: dict[int, object] = {}

    def request_stop(signum: int, _frame: object) -> None:
        nonlocal terminate_signal
        terminate_signal = signum

    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        previous_handlers[signum] = signal.signal(signum, request_stop)
    try:
        stable: list[ResourceSnapshot] = []
        for sample_index in range(3):
            current = snapshot(
                own_root=own_root,
                min_free_vram_mib=min_free_vram_mib,
                min_available_ram_mib=min_available_ram_mib,
            )
            stable.append(current)
            if not current.allowed:
                write_status(
                    status_path,
                    {"state": "yielded_before_start", "snapshot": asdict(current)},
                )
                return 75
            if sample_index < 2:
                time.sleep(0.5)

        environment = os.environ.copy()
        environment[NONCE_ENV] = run_nonce
        process = subprocess.Popen(tuple(command), start_new_session=True, env=environment)
        write_status(
            status_path,
            {
                "state": "running",
                "pid": process.pid,
                "run_nonce": run_nonce,
                "snapshot": asdict(stable[-1]),
            },
        )
        vram_pressure_samples = 0
        ram_pressure_samples = 0
        while process.poll() is None and not terminate_signal:
            time.sleep(interval_s)
            current = snapshot(
                own_root=own_root,
                min_free_vram_mib=runtime_floor_vram_mib,
                min_available_ram_mib=runtime_floor_ram_mib,
                include_port_hint=False,
            )
            if current.foreign_processes:
                yielded = ("foreign_matrix_active", current)
                break
            vram_pressure_samples = (
                vram_pressure_samples + 1
                if current.free_vram_mib is None
                or current.free_vram_mib < runtime_floor_vram_mib
                else 0
            )
            ram_pressure_samples = (
                ram_pressure_samples + 1
                if current.available_ram_mib < runtime_floor_ram_mib
                else 0
            )
            if vram_pressure_samples >= 3:
                yielded = ("runtime_vram_pressure", current)
                break
            if ram_pressure_samples >= 5:
                yielded = ("runtime_ram_pressure", current)
                break

        if yielded is not None:
            reason, current = yielded
            write_status(
                status_path,
                {
                    "state": "yield_requested",
                    "pid": process.pid,
                    "run_nonce": run_nonce,
                    "reason": reason,
                    "snapshot": asdict(current),
                },
            )
        elif terminate_signal:
            write_status(
                status_path,
                {
                    "state": "stop_requested",
                    "pid": process.pid,
                    "run_nonce": run_nonce,
                    "signal": terminate_signal,
                },
            )
        if yielded is not None or terminate_signal:
            stop_process_group(process, grace_s=shutdown_grace_s)
        if yielded is not None:
            reason, current = yielded
            write_status(
                status_path,
                {
                    "state": "yielded_during_run",
                    "pid": process.pid,
                    "run_nonce": run_nonce,
                    "reason": reason,
                    "snapshot": asdict(current),
                },
            )
            return 75
        if terminate_signal:
            write_status(
                status_path,
                {
                    "state": "stopped_by_signal",
                    "pid": process.pid,
                    "run_nonce": run_nonce,
                    "signal": terminate_signal,
                },
            )
            return 128 + terminate_signal
        return_code = int(process.returncode or 0)
        write_status(
            status_path,
            {
                "state": "completed",
                "pid": process.pid,
                "run_nonce": run_nonce,
                "return_code": return_code,
            },
        )
        return return_code
    finally:
        if process is not None and process.poll() is None:
            stop_process_group(process, grace_s=shutdown_grace_s)
        signal_nonce_processes(run_nonce, signal.SIGTERM)
        if process is not None and cleanup_command:
            cleanup_environment = os.environ.copy()
            cleanup_environment["MATRIX_EXPECTED_NONCE"] = run_nonce
            try:
                subprocess.run(
                    tuple(cleanup_command),
                    env=cleanup_environment,
                    check=False,
                    timeout=8,
                )
            except subprocess.TimeoutExpired:
                pass
        try:
            terminate_nonce_processes(run_nonce, grace_s=0.5)
        except RuntimeError as exc:
            remaining = nonce_processes(run_nonce)
            write_status(
                status_path,
                {
                    "state": "cleanup_failed",
                    "run_nonce": run_nonce,
                    "reason": str(exc),
                    "remaining_pids": [item.pid for item in remaining],
                },
            )
            raise
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        fcntl.flock(lock_handle, fcntl.LOCK_UN)
        lock_handle.close()

import argparse


def _main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Cooperative Matrix resource guard")
    parser.add_argument("--own-root", required=True)
    parser.add_argument("--min-free-vram-mib", type=int, default=12_288)
    parser.add_argument("--min-available-ram-mib", type=int, default=12_288)
    subparsers = parser.add_subparsers(dest="action", required=True)
    check = subparsers.add_parser("check")
    check.add_argument("--ignore-host-udp-9999", action="store_true")
    run = subparsers.add_parser("run")
    run.add_argument("--status", type=Path, required=True)
    run.add_argument("--runtime-floor-vram-mib", type=int, default=1_536)
    run.add_argument("--runtime-floor-ram-mib", type=int, default=3_072)
    run.add_argument("--lock", type=Path, required=True)
    run.add_argument("--cleanup-script", type=Path)
    run.add_argument("--interval", type=float, default=0.25)
    run.add_argument("--shutdown-grace", type=float, default=40.0)
    run.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)

    if args.action == "check":
        current = snapshot(
            own_root=args.own_root,
            min_free_vram_mib=args.min_free_vram_mib,
            min_available_ram_mib=args.min_available_ram_mib,
            include_port_hint=not args.ignore_host_udp_9999,
        )
        print(current.to_json())
        return 0 if current.allowed else 75

    command = tuple(args.command)
    if command and command[0] == "--":
        command = command[1:]
    if not command:
        parser.error("run requires a command after --")
    if args.interval <= 0.0 or args.shutdown_grace <= 0.0:
        parser.error("--interval and --shutdown-grace must be positive")
    cleanup_command = (
        ("bash", str(args.cleanup_script)) if args.cleanup_script else None
    )
    return run_guarded(
        command,
        own_root=args.own_root,
        status_path=args.status,
        min_free_vram_mib=args.min_free_vram_mib,
        min_available_ram_mib=args.min_available_ram_mib,
        runtime_floor_vram_mib=args.runtime_floor_vram_mib,
        runtime_floor_ram_mib=args.runtime_floor_ram_mib,
        lock_path=args.lock,
        cleanup_command=cleanup_command,
        interval_s=args.interval,
        shutdown_grace_s=args.shutdown_grace,
    )


if __name__ == "__main__":
    raise SystemExit(_main())
