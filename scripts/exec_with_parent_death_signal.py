#!/usr/bin/env python3
"""Run one command in this process group and tear it down if the parent dies."""

from __future__ import annotations

import argparse
import ctypes
import os
import signal
import subprocess
import sys
import time


PR_SET_PDEATHSIG = 1
TERM_GRACE_SECONDS = 2.0


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--expected-parent", type=int, required=True)
    parser.add_argument(
        "--exec-command",
        action="store_true",
        help=(
            "Replace the guardian with the command after arming PDEATHSIG. "
            "Use only for a leaf process whose exact PID is security-relevant."
        ),
    )
    parser.add_argument(
        "--argv0",
        default=None,
        help=(
            "Override argv[0] for an exec-mode leaf while still executing the "
            "exact command path."
        ),
    )
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if args.command[:1] == ["--"]:
        args.command = args.command[1:]
    if not args.command:
        parser.error("a command is required after --")
    if args.argv0 is not None:
        if not args.exec_command:
            parser.error("--argv0 requires --exec-command")
        if not args.argv0 or "\0" in args.argv0:
            parser.error("--argv0 must be a non-empty string without NUL")
    return args


def _arm_parent_death_signal(signum: int) -> None:
    if not sys.platform.startswith("linux"):
        raise RuntimeError("parent-death guardian requires Linux")
    libc = ctypes.CDLL(None, use_errno=True)
    prctl = libc.prctl
    prctl.argtypes = [
        ctypes.c_int,
        ctypes.c_ulong,
        ctypes.c_ulong,
        ctypes.c_ulong,
        ctypes.c_ulong,
    ]
    prctl.restype = ctypes.c_int
    if prctl(PR_SET_PDEATHSIG, signum, 0, 0, 0) != 0:
        error_number = ctypes.get_errno()
        error_detail = (
            os.strerror(error_number)
            if error_number
            else "prctl failed without errno"
        )
        raise OSError(error_number, error_detail)


def _terminate_process_group(_signum: int, _frame) -> None:
    # The guardian is the session/process-group leader. Ignore repeat delivery
    # to ourselves, forward TERM to every descendant in the exact group, then
    # enforce a bounded hard stop even when the supervisor no longer exists.
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, signal.SIG_IGN)
    try:
        os.killpg(os.getpgrp(), signal.SIGTERM)
    except ProcessLookupError:
        os._exit(0)
    time.sleep(TERM_GRACE_SECONDS)
    try:
        os.killpg(os.getpgrp(), signal.SIGKILL)
    except ProcessLookupError:
        os._exit(0)
    os._exit(128 + signal.SIGKILL)


def main() -> int:
    args = _parse_args()
    # A leaf exec has no guardian left to enforce a grace deadline. Use KILL
    # for unexpected parent death so a provider stuck in Xlib or cleanup cannot
    # survive while retaining the inherited host-lock fd. Normal supervised
    # shutdown still reaches the process group with TERM before escalation.
    _arm_parent_death_signal(
        signal.SIGKILL if args.exec_command else signal.SIGTERM
    )
    # PR_SET_PDEATHSIG has a fork-to-prctl race by definition. Checking the
    # expected parent closes it before any native command is spawned.
    if os.getppid() != args.expected_parent:
        return 125
    if args.exec_command:
        # exec preserves both the PID observed by the supervisor and the
        # parent-death signal armed above.  The game-input adapter is a leaf
        # process, so retaining a separate process-group reaper would only
        # obscure the SO_PEERCRED identity that the runtime must authenticate.
        exec_args = (
            [args.argv0, *args.command[1:]]
            if args.argv0 is not None
            else args.command
        )
        os.execvpe(args.command[0], exec_args, os.environ)
        raise AssertionError("os.execvp returned unexpectedly")
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, _terminate_process_group)

    child = subprocess.Popen(args.command)
    returncode = child.wait()
    return returncode if returncode >= 0 else 128 - returncode


if __name__ == "__main__":
    raise SystemExit(main())
