#!/usr/bin/env python3
"""Private, validated restart requests for the Matrix top-level launcher."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hmac
import json
import math
import os
from pathlib import Path
import secrets
import stat
import tempfile
import time

WATCH_REQUEST_VALID = 75


@dataclass(frozen=True)
class RestartRequest:
    launcher_pid: int
    provider_pid: int
    nonce: str

    def to_mapping(self) -> dict[str, object]:
        return {
            "version": 1,
            "action": "restart-whole-runtime",
            "launcher_pid": self.launcher_pid,
            "provider_pid": self.provider_pid,
            "nonce": self.nonce,
        }


def _valid_pid(value: object) -> bool:
    return type(value) is int and value > 1


def validate_nonce(value: str) -> None:
    if (
        not isinstance(value, str)
        or len(value) != 64
        or any(character not in "0123456789abcdef" for character in value)
    ):
        raise ValueError("restart nonce must be 64 lowercase hexadecimal characters")


def atomic_write_capability(path: Path) -> None:
    if not path.is_absolute():
        raise ValueError("restart capability path must be absolute")
    parent_stat = path.parent.stat()
    if (
        not stat.S_ISDIR(parent_stat.st_mode)
        or parent_stat.st_uid != os.getuid()
        or parent_stat.st_mode & 0o077
    ):
        raise PermissionError("restart capability directory must be private")
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    try:
        os.write(descriptor, f"{secrets.token_hex(32)}\n".encode("ascii"))
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def read_capability(path: Path) -> str:
    parent_stat = path.parent.stat()
    if (
        not stat.S_ISDIR(parent_stat.st_mode)
        or parent_stat.st_uid != os.getuid()
        or parent_stat.st_mode & 0o077
    ):
        raise PermissionError("restart capability directory must be private")
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    try:
        file_stat = os.fstat(descriptor)
        if (
            not stat.S_ISREG(file_stat.st_mode)
            or file_stat.st_uid != os.getuid()
            or file_stat.st_mode & 0o077
            or file_stat.st_size > 128
        ):
            raise PermissionError("restart capability is not a private regular file")
        value = os.read(descriptor, 128).decode("ascii").strip()
    finally:
        os.close(descriptor)
    validate_nonce(value)
    return value


def atomic_write_request(path: Path, request: RestartRequest) -> None:
    if not path.is_absolute():
        raise ValueError("restart request path must be absolute")
    if not _valid_pid(request.launcher_pid) or not _valid_pid(request.provider_pid):
        raise ValueError("restart PIDs must be greater than one")
    validate_nonce(request.nonce)
    parent = path.parent
    parent_stat = parent.stat()
    if (
        not stat.S_ISDIR(parent_stat.st_mode)
        or parent_stat.st_uid != os.getuid()
        or parent_stat.st_mode & 0o077
    ):
        raise PermissionError("restart request directory must be private and user-owned")
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=parent,
            prefix=f".{path.name}.",
            delete=False,
        ) as stream:
            temporary = Path(stream.name)
            os.fchmod(stream.fileno(), 0o600)
            json.dump(request.to_mapping(), stream, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        temporary = None
    finally:
        if temporary is not None:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass


def _read_private_request(path: Path) -> tuple[dict[str, object], os.stat_result]:
    parent_stat = path.parent.stat()
    if (
        not stat.S_ISDIR(parent_stat.st_mode)
        or parent_stat.st_uid != os.getuid()
        or parent_stat.st_mode & 0o077
    ):
        raise PermissionError("restart request directory is not private/user-owned")
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    try:
        file_stat = os.fstat(descriptor)
        if (
            not stat.S_ISREG(file_stat.st_mode)
            or file_stat.st_uid != os.getuid()
            or file_stat.st_mode & 0o077
            or file_stat.st_size > 4096
        ):
            raise PermissionError("restart request file is not a private regular file")
        with os.fdopen(descriptor, "r", encoding="utf-8", closefd=False) as stream:
            value = json.load(stream)
    finally:
        os.close(descriptor)
    if not isinstance(value, dict):
        raise ValueError("restart request must be an object")
    return (value, file_stat)


def _parent_pid(pid: int) -> int:
    # /proc/<pid>/stat's second field is parenthesized and may contain spaces.
    text = Path(f"/proc/{pid}/stat").read_text(encoding="ascii")
    closing = text.rfind(")")
    if closing < 0:
        raise ValueError(f"cannot parse parent PID for {pid}")
    fields = text[closing + 2 :].split()
    if len(fields) < 2:
        raise ValueError(f"cannot parse parent PID for {pid}")
    return int(fields[1])


def _is_descendant(pid: int, ancestor: int) -> bool:
    current = pid
    visited: set[int] = set()
    for _ in range(64):
        if current == ancestor:
            return True
        if current <= 1 or current in visited:
            return False
        visited.add(current)
        current = _parent_pid(current)
    return False


def _process_running(pid: int) -> bool:
    try:
        text = Path(f"/proc/{pid}/stat").read_text(encoding="ascii")
    except (FileNotFoundError, ProcessLookupError):
        return False
    closing = text.rfind(")")
    if closing < 0:
        return False
    fields = text[closing + 2 :].split()
    return bool(fields and fields[0] != "Z")


def _command_contains_script(pid: int, expected_script: Path) -> bool:
    raw = Path(f"/proc/{pid}/cmdline").read_bytes()
    expected = expected_script.resolve(strict=True)
    for field in raw.split(b"\0"):
        if not field:
            continue
        try:
            candidate = Path(os.fsdecode(field))
        except UnicodeError:
            continue
        if candidate.is_absolute() and candidate.resolve(strict=False) == expected:
            return True
    return False


def validate_request(
    path: Path,
    *,
    expected_launcher_pid: int,
    expected_run_sim_pid: int,
    expected_provider_script: Path,
    expected_nonce: str,
    maximum_age_s: float = 10.0,
    consume: bool = False,
    now: float | None = None,
) -> RestartRequest:
    validate_nonce(expected_nonce)
    if not _valid_pid(expected_launcher_pid) or not _valid_pid(expected_run_sim_pid):
        raise ValueError("expected restart PIDs must be greater than one")
    value, file_stat = _read_private_request(path)
    if set(value) != {
        "version",
        "action",
        "launcher_pid",
        "provider_pid",
        "nonce",
    }:
        raise ValueError("restart request schema mismatch")
    request = RestartRequest(
        launcher_pid=value.get("launcher_pid"),
        provider_pid=value.get("provider_pid"),
        nonce=value.get("nonce"),
    )
    if value.get("version") != 1 or value.get("action") != "restart-whole-runtime":
        raise ValueError("unsupported restart request")
    if not _valid_pid(request.launcher_pid) or not _valid_pid(request.provider_pid):
        raise ValueError("restart request contains an invalid PID")
    validate_nonce(request.nonce)
    if request.launcher_pid != expected_launcher_pid:
        raise ValueError("restart request targets another launcher")
    if not hmac.compare_digest(request.nonce, expected_nonce):
        raise ValueError("restart request nonce mismatch")
    current_time = time.time() if now is None else now
    age = current_time - file_stat.st_mtime
    if not math.isfinite(age) or age < -2.0 or age > maximum_age_s:
        raise ValueError("restart request is stale")
    provider_stat = Path(f"/proc/{request.provider_pid}").stat()
    if provider_stat.st_uid != os.getuid():
        raise PermissionError("restart provider belongs to another user")
    if not _is_descendant(request.provider_pid, expected_run_sim_pid):
        raise PermissionError("restart provider is outside the supervised run_sim tree")
    if not _command_contains_script(request.provider_pid, expected_provider_script):
        raise PermissionError("restart provider command does not match the pinned script")
    if consume:
        path.unlink()
    return request


def watch_for_request(
    path: Path,
    *,
    expected_launcher_pid: int,
    expected_run_sim_pid: int,
    expected_provider_script: Path,
    capability_file: Path,
    poll_seconds: float = 0.2,
) -> int:
    """Poll in one long-lived process; never interrupt the runtime on errors."""

    if not math.isfinite(poll_seconds) or not 0.05 <= poll_seconds <= 2.0:
        raise ValueError("restart poll interval must be in [0.05, 2.0]")
    expected_nonce = read_capability(capability_file)
    while os.getppid() == expected_launcher_pid and _process_running(
        expected_run_sim_pid
    ):
        try:
            path.lstat()
        except FileNotFoundError:
            time.sleep(poll_seconds)
            continue
        try:
            validate_request(
                path,
                expected_launcher_pid=expected_launcher_pid,
                expected_run_sim_pid=expected_run_sim_pid,
                expected_provider_script=expected_provider_script,
                expected_nonce=expected_nonce,
                consume=True,
            )
        except (OSError, UnicodeError, ValueError) as exc:
            print(f"[WARN] Rejected restart request: {exc}", file=os.sys.stderr)
            try:
                path.unlink()
            except OSError:
                pass
            time.sleep(poll_seconds)
            continue
        return WATCH_REQUEST_VALID
    return 0


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    capability = subparsers.add_parser("create-capability")
    capability.add_argument("--file", type=Path, required=True)
    validate = subparsers.add_parser("validate")
    validate.add_argument("--file", type=Path, required=True)
    validate.add_argument("--launcher-pid", type=int, required=True)
    validate.add_argument("--run-sim-pid", type=int, required=True)
    validate.add_argument("--provider-script", type=Path, required=True)
    validate.add_argument("--capability-file", type=Path, required=True)
    validate.add_argument("--consume", action="store_true")
    watch = subparsers.add_parser("watch")
    watch.add_argument("--file", type=Path, required=True)
    watch.add_argument("--launcher-pid", type=int, required=True)
    watch.add_argument("--run-sim-pid", type=int, required=True)
    watch.add_argument("--provider-script", type=Path, required=True)
    watch.add_argument("--capability-file", type=Path, required=True)
    watch.add_argument("--poll-seconds", type=float, default=0.2)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if args.command == "create-capability":
        atomic_write_capability(args.file)
        return 0
    if args.command == "validate":
        request = validate_request(
            args.file,
            expected_launcher_pid=args.launcher_pid,
            expected_run_sim_pid=args.run_sim_pid,
            expected_provider_script=args.provider_script,
            expected_nonce=read_capability(args.capability_file),
            consume=args.consume,
        )
        print(f"validated provider_pid={request.provider_pid}")
        return 0
    if args.command == "watch":
        return watch_for_request(
            args.file,
            expected_launcher_pid=args.launcher_pid,
            expected_run_sim_pid=args.run_sim_pid,
            expected_provider_script=args.provider_script,
            capability_file=args.capability_file,
            poll_seconds=args.poll_seconds,
        )
    raise AssertionError(f"unhandled command: {args.command}")


if __name__ == "__main__":
    raise SystemExit(main())
