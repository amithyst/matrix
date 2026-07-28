#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import asdict, dataclass
from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
import signal as signal_module
import tempfile
import time


NONCE_ENV = "MATRIX_BFM_CLEAN_RUN_NONCE"


@dataclass(frozen=True, slots=True)
class ProcessIdentity:
    pid: int
    pgid: int
    sid: int
    starttime: int
    uid: int


def process_identity(pid: int, proc_root: Path = Path("/proc")) -> ProcessIdentity | None:
    base = proc_root / str(int(pid))
    try:
        stat = (base / "stat").read_text()
        end = stat.rfind(")")
        if end < 0:
            return None
        fields = stat[end + 2 :].split()
        status = (base / "status").read_text()
    except (FileNotFoundError, PermissionError, ProcessLookupError, OSError):
        return None
    if len(fields) < 20:
        return None
    uid_line = next((line for line in status.splitlines() if line.startswith("Uid:")), "")
    if not uid_line:
        return None
    return ProcessIdentity(
        pid=int(pid),
        pgid=int(fields[2]),
        sid=int(fields[3]),
        starttime=int(fields[19]),
        uid=int(uid_line.split()[1]),
    )


def process_nonce(pid: int, proc_root: Path = Path("/proc")) -> str | None:
    try:
        values = (proc_root / str(int(pid)) / "environ").read_bytes().split(b"\0")
    except (FileNotFoundError, PermissionError, ProcessLookupError, OSError):
        return None
    prefix = (NONCE_ENV + "=").encode()
    for value in values:
        if value.startswith(prefix):
            return value[len(prefix) :].decode("utf-8", errors="replace")
    return None


def group_members(pgid: int, proc_root: Path = Path("/proc")) -> tuple[ProcessIdentity, ...]:
    try:
        entries = tuple(proc_root.iterdir())
    except OSError:
        return ()
    members = []
    for entry in entries:
        if not entry.name.isdigit():
            continue
        identity = process_identity(int(entry.name), proc_root)
        if identity is not None and identity.pgid == pgid:
            members.append(identity)
    return tuple(sorted(members, key=lambda item: item.pid))


def atomic_write(path: Path, payload: dict[str, object]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=path.name + ".", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        os.fchmod(descriptor, 0o600)
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        temporary.unlink(missing_ok=True)


@contextmanager
def ledger_lock(path: Path):
    path = Path(path)
    lock_path = path.with_suffix(path.suffix + ".lock")
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open("w") as handle:
        os.chmod(lock_path, 0o600)
        fcntl.flock(handle, fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(handle, fcntl.LOCK_UN)


def initialize(
    path: Path,
    nonce: str,
    launcher_pid: int,
    *,
    proc_root: Path = Path("/proc"),
) -> None:
    path = Path(path)
    launcher = process_identity(launcher_pid, proc_root)
    if launcher is None:
        raise RuntimeError(f"launcher PID is not alive: {launcher_pid}")
    with ledger_lock(path):
        if path.exists():
            remaining = owned_processes(path, proc_root=proc_root)
            if remaining:
                raise RuntimeError(
                    "refusing to replace non-empty instance ledger: "
                    + ",".join(str(item.pid) for item in remaining)
                )
        atomic_write(
            path,
            {
                "schema": "xvi.mbsc.instance_ledger",
                "version": 1,
                "nonce": nonce,
                "launcher": asdict(launcher),
                "groups": [],
            },
        )


def load(path: Path) -> dict[str, object]:
    path = Path(path)
    payload = json.loads(path.read_text())
    if payload.get("schema") != "xvi.mbsc.instance_ledger" or payload.get("version") != 1:
        raise ValueError("unsupported instance ledger")
    if not isinstance(payload.get("nonce"), str) or not payload["nonce"]:
        raise ValueError("instance ledger has no nonce")
    if not isinstance(payload.get("groups"), list):
        raise ValueError("instance ledger groups must be a list")
    return payload


def add_group(path: Path, nonce: str, leader_pid: int) -> ProcessIdentity:
    path = Path(path)
    deadline = time.monotonic() + 1.0
    identity = None
    while time.monotonic() < deadline:
        identity = process_identity(leader_pid)
        if identity is not None and identity.pid == identity.pgid == identity.sid:
            break
        time.sleep(0.02)
    if identity is None:
        raise RuntimeError(f"process group leader is not alive: {leader_pid}")
    if identity.pid != identity.pgid or identity.pid != identity.sid:
        raise RuntimeError(
            f"owned child did not become a setsid leader: pid={identity.pid} "
            f"pgid={identity.pgid} sid={identity.sid}"
        )
    with ledger_lock(path):
        payload = load(path)
        if payload["nonce"] != nonce:
            raise ValueError("instance ledger nonce mismatch")
        groups = payload["groups"]
        assert isinstance(groups, list)
        groups.append(asdict(identity))
        atomic_write(path, payload)
    return identity


def _identity_from_dict(payload: dict[str, object]) -> ProcessIdentity:
    return ProcessIdentity(**{key: int(payload[key]) for key in asdict(ProcessIdentity(0, 0, 0, 0, 0))})


def safe_group_targets(
    path: Path,
    *,
    expected_nonce: str | None = None,
    proc_root: Path = Path("/proc"),
) -> tuple[int, ...]:
    path = Path(path)
    payload = load(path)
    nonce = str(payload["nonce"])
    if expected_nonce is not None and nonce != expected_nonce:
        raise ValueError("instance ledger nonce mismatch")
    targets: list[int] = []
    groups = payload["groups"]
    assert isinstance(groups, list)
    for raw in groups:
        if not isinstance(raw, dict):
            continue
        recorded = _identity_from_dict(raw)
        current = process_identity(recorded.pid, proc_root)
        if current == recorded:
            targets.append(recorded.pgid)
            continue
        members = group_members(recorded.pgid, proc_root)
        if not members:
            continue
        if any(
            member.uid == recorded.uid
            and process_nonce(member.pid, proc_root) == nonce
            for member in members
        ):
            targets.append(recorded.pgid)
    return tuple(dict.fromkeys(targets))


def owned_processes(
    path: Path,
    *,
    expected_nonce: str | None = None,
    proc_root: Path = Path("/proc"),
) -> tuple[ProcessIdentity, ...]:
    path = Path(path)
    payload = load(path)
    nonce = str(payload["nonce"])
    if expected_nonce is not None and nonce != expected_nonce:
        raise ValueError("instance ledger nonce mismatch")
    owned: dict[int, ProcessIdentity] = {}
    groups = payload["groups"]
    assert isinstance(groups, list)
    for raw in groups:
        if not isinstance(raw, dict):
            continue
        recorded = _identity_from_dict(raw)
        current_leader = process_identity(recorded.pid, proc_root)
        if current_leader == recorded:
            # A matching leader identity is conclusive ownership evidence even
            # if /proc/<pid>/environ becomes unreadable.  Keep it visible to
            # verify-empty so a stale launcher cannot discard the only cleanup
            # record while an owned process is still alive.
            owned[recorded.pid] = recorded
        for member in group_members(recorded.pgid, proc_root):
            if member.uid != recorded.uid:
                continue
            member_nonce = process_nonce(member.pid, proc_root)
            if member_nonce != nonce:
                if member_nonce is None:
                    # Conservatively preserve the ledger for an uninspectable
                    # member. signal_groups still refuses to signal it without
                    # nonce proof.
                    owned[member.pid] = member
                continue
            owned[member.pid] = member
    return tuple(owned[pid] for pid in sorted(owned))


def owned_process_by_pid(
    path: Path,
    pid: int,
    *,
    expected_nonce: str,
    proc_root: Path = Path("/proc"),
) -> ProcessIdentity | None:
    for identity in owned_processes(
        path,
        expected_nonce=expected_nonce,
        proc_root=proc_root,
    ):
        if identity.pid == pid:
            return identity
    return None


def terminate_unregistered_group(
    leader_pid: int,
    nonce: str,
) -> tuple[int, ...]:
    """Fail-safe cleanup for a setsid child that could not enter the ledger."""

    recorded = process_identity(leader_pid)
    if recorded is None:
        return ()
    if recorded.pid != recorded.pgid or recorded.pid != recorded.sid:
        raise RuntimeError("unregistered child is not a setsid group leader")
    if process_nonce(recorded.pid) != nonce:
        raise RuntimeError("unregistered child nonce mismatch")
    pidfd = os.pidfd_open(recorded.pid)
    try:
        if process_identity(recorded.pid) != recorded:
            raise RuntimeError("unregistered child identity changed")
        members = group_members(recorded.pgid)
        if not members:
            return ()
        if any(
            member.uid != recorded.uid or process_nonce(member.pid) != nonce
            for member in members
        ):
            raise RuntimeError("unregistered process group contains an unowned member")
        # This branch exists only because ownership could not be recorded. A
        # single validated SIGKILL avoids leaving descendants behind after the
        # leader exits and removes the TERM-to-KILL process-group reuse window.
        os.killpg(recorded.pgid, signal_module.SIGKILL)
        return tuple(member.pid for member in members)
    finally:
        os.close(pidfd)


def signal_groups(
    path: Path,
    signal_name: str,
    *,
    expected_nonce: str | None = None,
) -> tuple[int, ...]:
    path = Path(path)
    try:
        signal_value = int(getattr(signal_module, "SIG" + signal_name.upper()))
    except (AttributeError, ValueError) as exc:
        raise ValueError(f"unsupported signal: {signal_name}") from exc
    with ledger_lock(path):
        targets = owned_processes(path, expected_nonce=expected_nonce)
    signaled = []
    for recorded in targets:
        pidfd = None
        try:
            pidfd = os.pidfd_open(recorded.pid)
            current = process_identity(recorded.pid)
            if current != recorded:
                continue
            payload = load(path)
            if process_nonce(recorded.pid) != payload["nonce"]:
                continue
            signal_module.pidfd_send_signal(pidfd, signal_value)
        except (ProcessLookupError, PermissionError):
            continue
        finally:
            if pidfd is not None:
                os.close(pidfd)
        signaled.append(recorded.pid)
    return tuple(signaled)


def signal_owned_pid(
    path: Path,
    pid: int,
    signal_name: str,
    *,
    expected_nonce: str,
) -> bool:
    try:
        signal_value = int(getattr(signal_module, "SIG" + signal_name.upper()))
    except (AttributeError, ValueError) as exc:
        raise ValueError(f"unsupported signal: {signal_name}") from exc
    with ledger_lock(path):
        recorded = owned_process_by_pid(
            path,
            pid,
            expected_nonce=expected_nonce,
        )
    if recorded is None:
        raise RuntimeError(f"PID {pid} is not owned by this instance ledger")
    pidfd = os.pidfd_open(recorded.pid)
    try:
        if process_identity(recorded.pid) != recorded:
            raise RuntimeError(f"PID {pid} identity changed before signal")
        payload = load(path)
        if payload["nonce"] != expected_nonce:
            raise RuntimeError("instance ledger nonce changed before signal")
        if process_nonce(recorded.pid) != expected_nonce:
            raise RuntimeError(f"PID {pid} nonce changed before signal")
        signal_module.pidfd_send_signal(pidfd, signal_value)
    finally:
        os.close(pidfd)
    return True

import argparse


def _main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Matrix BFM/Isaac instance owner ledger"
    )
    parser.add_argument("--path", type=Path, required=True)
    parser.add_argument("--nonce")
    subparsers = parser.add_subparsers(dest="action", required=True)
    init = subparsers.add_parser("init")
    init.add_argument("--launcher-pid", type=int, required=True)
    add = subparsers.add_parser("add")
    add.add_argument("--pid", type=int, required=True)
    send = subparsers.add_parser("signal")
    send.add_argument("--signal", choices=("TERM", "KILL"), required=True)
    send_pid = subparsers.add_parser("signal-pid")
    send_pid.add_argument("--pid", type=int, required=True)
    send_pid.add_argument("--signal", choices=("TERM", "KILL"), required=True)
    terminate = subparsers.add_parser("terminate-unregistered")
    terminate.add_argument("--pid", type=int, required=True)
    subparsers.add_parser("verify-empty")
    args = parser.parse_args(argv)

    if args.action in {
        "init",
        "add",
        "signal-pid",
        "terminate-unregistered",
    } and not args.nonce:
        parser.error(f"{args.action} requires --nonce")
    if args.action == "init":
        initialize(args.path, args.nonce, args.launcher_pid)
        return 0
    if args.action == "add":
        identity = add_group(args.path, args.nonce, args.pid)
        print(json.dumps({"ok": True, "identity": asdict(identity)}))
        return 0
    if args.action == "signal":
        signaled = signal_groups(
            args.path, args.signal, expected_nonce=args.nonce
        )
        print(json.dumps({"ok": True, "signal": args.signal, "pids": signaled}))
        return 0
    if args.action == "signal-pid":
        signal_owned_pid(
            args.path,
            args.pid,
            args.signal,
            expected_nonce=args.nonce,
        )
        print(
            json.dumps(
                {"ok": True, "signal": args.signal, "pid": args.pid}
            )
        )
        return 0
    if args.action == "terminate-unregistered":
        terminated = terminate_unregistered_group(args.pid, args.nonce)
        print(json.dumps({"ok": True, "pids": terminated}))
        return 0
    remaining = owned_processes(args.path, expected_nonce=args.nonce)
    print(
        json.dumps(
            {
                "ok": not remaining,
                "remaining": [asdict(item) for item in remaining],
            }
        )
    )
    return 0 if not remaining else 4


if __name__ == "__main__":
    raise SystemExit(_main())
