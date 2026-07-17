#!/usr/bin/env python3
"""Own and supervise the Matrix UE process without polling a reusable PID."""

from __future__ import annotations

import argparse
from functools import partial
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import tempfile
import time
from typing import BinaryIO

from exec_with_parent_death_signal import _arm_parent_death_signal


UNKNOWN_EXIT_CODE = 255
SPAWN_FAILURE_EXIT_CODE = 127


def _normalized_returncode(returncode: int) -> int:
    if returncode >= 0:
        return min(returncode, 255)
    return min(128 + abs(returncode), 255)


def _atomic_text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        dir=path.parent,
        prefix=f".{path.name}.",
        delete=False,
    ) as stream:
        stream.write(value)
        temporary = Path(stream.name)
    os.replace(temporary, path)


def _publish_failure(path: Path, returncode: int) -> None:
    payload = {
        "name": "ue",
        "exit_code": _normalized_returncode(returncode),
    }
    _atomic_text(path, json.dumps(payload, separators=(",", ":")) + "\n")


def _peek_returncode(process: subprocess.Popen[bytes]) -> int | None:
    """Observe a direct child without reaping it, keeping its PID/PGID reserved."""
    flags = os.WEXITED | os.WNOHANG | os.WNOWAIT
    result = os.waitid(os.P_PID, process.pid, flags)
    if result is None:
        return None
    if result.si_code == os.CLD_EXITED:
        return int(result.si_status)
    return -int(result.si_status)


def _signal_process_group(process: subprocess.Popen[bytes], signum: int) -> None:
    try:
        os.killpg(process.pid, signum)
    except ProcessLookupError:
        pass


def _reap_process_group(
    process: subprocess.Popen[bytes],
    *,
    returncode: int | None,
    term_grace_seconds: float,
) -> int:
    """Stop residual group members while the unreaped leader reserves its PGID."""
    _signal_process_group(process, signal.SIGTERM)
    deadline = time.monotonic() + term_grace_seconds
    observed = returncode
    while observed is None and time.monotonic() < deadline:
        time.sleep(0.02)
        observed = _peek_returncode(process)
    # The wrapper normally execs UE. If it left descendants behind, kill them
    # before reaping the group leader so its numeric PGID cannot be reused.
    _signal_process_group(process, signal.SIGKILL)
    waited = process.wait()
    return observed if observed is not None else waited


def _arm_ue_parent_death_signal(expected_parent_pid: int) -> None:
    # SIGKILL closes the last orphan window if the supervisor itself is killed.
    # Recheck after prctl to close the fork-to-prctl parent-death race.
    _arm_parent_death_signal(signal.SIGKILL)
    if os.getppid() != expected_parent_pid:
        os._exit(125)


def _control_event(stream: BinaryIO, timeout: float) -> str | None:
    readable, _, _ = select.select([stream], [], [], timeout)
    if not readable:
        return None
    data = os.read(stream.fileno(), 4096)
    if not data:
        return "eof"
    if b"stop" in data.split():
        return "stop"
    return None


def supervise(args: argparse.Namespace) -> int:
    stop_requested = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    for signum in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
        signal.signal(signum, request_stop)

    _arm_parent_death_signal(signal.SIGTERM)
    if os.getppid() != args.expected_parent_pid:
        _publish_failure(args.failure_file, UNKNOWN_EXIT_CODE)
        return UNKNOWN_EXIT_CODE

    try:
        log_stream = args.log.open("ab", buffering=0)
    except OSError as exc:
        print(f"matrix-ue-supervisor ERROR opening UE log: {exc}", file=sys.stderr)
        _publish_failure(args.failure_file, SPAWN_FAILURE_EXIT_CODE)
        return SPAWN_FAILURE_EXIT_CODE

    try:
        try:
            supervisor_pid = os.getpid()
            process = subprocess.Popen(
                args.command,
                stdin=subprocess.DEVNULL,
                stdout=log_stream,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                preexec_fn=partial(
                    _arm_ue_parent_death_signal, supervisor_pid
                ),
            )
        except (OSError, ValueError, subprocess.SubprocessError) as exc:
            print(f"matrix-ue-supervisor ERROR starting UE: {exc}", file=sys.stderr)
            _atomic_text(args.pid_file, "0\n")
            _publish_failure(args.failure_file, SPAWN_FAILURE_EXIT_CODE)
            while not stop_requested:
                event = _control_event(sys.stdin.buffer, 0.05)
                stop_requested = event in {"stop", "eof"}
            return SPAWN_FAILURE_EXIT_CODE

        _atomic_text(args.pid_file, f"{process.pid}\n")
        while True:
            returncode = _peek_returncode(process)
            if returncode is not None:
                _publish_failure(args.failure_file, returncode)
                final_code = _reap_process_group(
                    process,
                    returncode=returncode,
                    term_grace_seconds=0.2,
                )
                while not stop_requested:
                    event = _control_event(sys.stdin.buffer, 0.05)
                    stop_requested = event in {"stop", "eof"}
                return _normalized_returncode(final_code)

            control_event = _control_event(sys.stdin.buffer, 0.02)
            if (
                control_event == "eof"
                and not stop_requested
                and os.getppid() == args.expected_parent_pid
            ):
                # Losing the control writer while the launcher is still alive is
                # a supervisor failure, not an authorized clean shutdown.
                _publish_failure(args.failure_file, UNKNOWN_EXIT_CODE)
                _reap_process_group(
                    process,
                    returncode=None,
                    term_grace_seconds=args.term_grace_seconds,
                )
                return UNKNOWN_EXIT_CODE
            if stop_requested or control_event in {"stop", "eof"}:
                # Recheck after observing the stop request. A child that exited
                # before this boundary is an unexpected exit, even exit code 0.
                returncode = _peek_returncode(process)
                if returncode is not None:
                    _publish_failure(args.failure_file, returncode)
                    final_code = _reap_process_group(
                        process,
                        returncode=returncode,
                        term_grace_seconds=0.2,
                    )
                    return _normalized_returncode(final_code)
                _reap_process_group(
                    process,
                    returncode=None,
                    term_grace_seconds=args.term_grace_seconds,
                )
                return 0
    finally:
        log_stream.close()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pid-file", type=Path, required=True)
    parser.add_argument("--failure-file", type=Path, required=True)
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--expected-parent-pid", type=int, required=True)
    parser.add_argument("--term-grace-seconds", type=float, default=3.0)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if args.command[:1] == ["--"]:
        args.command = args.command[1:]
    if not args.command:
        parser.error("a UE command is required after --")
    if args.expected_parent_pid <= 1:
        parser.error("--expected-parent-pid must identify the launcher")
    if args.term_grace_seconds < 0.0:
        parser.error("--term-grace-seconds must be non-negative")
    return args


def main() -> int:
    return supervise(_parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
