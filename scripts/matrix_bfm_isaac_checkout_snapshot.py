#!/usr/bin/env python3
"""Snapshot and atomically restore Matrix files mutated by the UE launcher."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import tempfile
from typing import Iterable


SNAPSHOT_SCHEMA = "matrix_bfm_isaac_checkout_snapshot.v1"
MANIFEST_NAME = "manifest.json"
SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")


class SnapshotError(RuntimeError):
    """Raised when a source or snapshot cannot be proven safe to restore."""


def _safe_relative_path(value: str) -> bool:
    if not value or "\\" in value:
        return False
    path = PurePosixPath(value)
    return (
        not path.is_absolute()
        and "." not in path.parts
        and ".." not in path.parts
        and path.as_posix() == value
    )


def _canonical_existing_directory(path: Path, *, label: str) -> Path:
    expanded = path.expanduser().absolute()
    if expanded.is_symlink():
        raise SnapshotError(f"{label} must not be a symlink: {expanded}")
    try:
        resolved = expanded.resolve(strict=True)
    except OSError as exc:
        raise SnapshotError(f"{label} is unavailable: {expanded}: {exc}") from exc
    if resolved != expanded or not resolved.is_dir():
        raise SnapshotError(f"{label} must be a real directory: {expanded}")
    return resolved


def _new_snapshot_path(path: Path) -> Path:
    expanded = path.expanduser().absolute()
    if expanded.exists() or expanded.is_symlink():
        raise SnapshotError(f"snapshot path already exists: {expanded}")
    parent = _canonical_existing_directory(expanded.parent, label="snapshot parent")
    candidate = parent / expanded.name
    if candidate != expanded:
        raise SnapshotError(f"snapshot path has a symlinked parent: {expanded}")
    return candidate


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _read_stable_regular_file(path: Path) -> tuple[bytes, int]:
    try:
        resolved = path.resolve(strict=True)
    except OSError as exc:
        raise SnapshotError(f"source file is unavailable: {path}: {exc}") from exc
    if resolved != path:
        raise SnapshotError(f"source file must not traverse a symlink: {path}")
    flags = os.O_RDONLY | os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        descriptor = os.open(path, flags)
    except OSError as exc:
        raise SnapshotError(f"cannot open source file: {path}: {exc}") from exc
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode):
            raise SnapshotError(f"source is not a regular file: {path}")
        chunks: list[bytes] = []
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            chunks.append(chunk)
        after = os.fstat(descriptor)
    finally:
        os.close(descriptor)
    identity_before = (
        before.st_dev,
        before.st_ino,
        before.st_size,
        before.st_mtime_ns,
        stat.S_IMODE(before.st_mode),
    )
    identity_after = (
        after.st_dev,
        after.st_ino,
        after.st_size,
        after.st_mtime_ns,
        stat.S_IMODE(after.st_mode),
    )
    data = b"".join(chunks)
    if identity_before != identity_after or len(data) != before.st_size:
        raise SnapshotError(f"source changed while it was being captured: {path}")
    return data, stat.S_IMODE(before.st_mode)


def _write_new_file(path: Path, data: bytes, *, mode: int) -> None:
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    descriptor = os.open(path, flags, mode)
    try:
        os.fchmod(descriptor, mode)
        view = memoryview(data)
        while view:
            written = os.write(descriptor, view)
            view = view[written:]
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _atomic_write_json(path: Path, payload: dict[str, object]) -> None:
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.tmp.", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        os.fchmod(descriptor, 0o600)
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        _fsync_directory(path.parent)
    finally:
        temporary.unlink(missing_ok=True)


def capture(root: Path, snapshot: Path, relative_paths: Iterable[str]) -> None:
    root = _canonical_existing_directory(root, label="Matrix root")
    snapshot = _new_snapshot_path(snapshot)
    paths = tuple(relative_paths)
    if not paths:
        raise SnapshotError("at least one source path is required")
    if len(paths) != len(set(paths)):
        raise SnapshotError("source paths must be unique")
    for relative in paths:
        if not _safe_relative_path(relative):
            raise SnapshotError(f"unsafe source path: {relative!r}")

    snapshot.mkdir(mode=0o700)
    try:
        entries: list[dict[str, object]] = []
        for index, relative in enumerate(paths):
            source = root / relative
            data, mode = _read_stable_regular_file(source)
            blob_name = f"{index:04d}.bin"
            _write_new_file(snapshot / blob_name, data, mode=0o600)
            entries.append(
                {
                    "path": relative,
                    "blob": blob_name,
                    "sha256": _sha256(data),
                    "size": len(data),
                    "mode": mode,
                }
            )
        _atomic_write_json(
            snapshot / MANIFEST_NAME,
            {
                "schema": SNAPSHOT_SCHEMA,
                "root": os.fspath(root),
                "entries": entries,
            },
        )
        _fsync_directory(snapshot)
    except BaseException:
        shutil.rmtree(snapshot, ignore_errors=True)
        raise


def _load_snapshot(
    root: Path, snapshot: Path
) -> tuple[Path, list[tuple[Path, bytes, int]]]:
    root = _canonical_existing_directory(root, label="Matrix root")
    snapshot = _canonical_existing_directory(snapshot, label="snapshot")
    manifest_path = snapshot / MANIFEST_NAME
    if manifest_path.is_symlink() or not manifest_path.is_file():
        raise SnapshotError(f"snapshot manifest is missing or a symlink: {manifest_path}")
    try:
        payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SnapshotError(f"cannot read snapshot manifest: {exc}") from exc
    if not isinstance(payload, dict) or set(payload) != {"schema", "root", "entries"}:
        raise SnapshotError("snapshot manifest has unexpected keys")
    if payload["schema"] != SNAPSHOT_SCHEMA or payload["root"] != os.fspath(root):
        raise SnapshotError("snapshot manifest does not belong to this Matrix root")
    raw_entries = payload["entries"]
    if not isinstance(raw_entries, list) or not raw_entries:
        raise SnapshotError("snapshot manifest has no entries")

    restored: list[tuple[Path, bytes, int]] = []
    seen: set[str] = set()
    for index, raw in enumerate(raw_entries):
        expected_keys = {"path", "blob", "sha256", "size", "mode"}
        if not isinstance(raw, dict) or set(raw) != expected_keys:
            raise SnapshotError(f"snapshot entry {index} has unexpected keys")
        relative = raw["path"]
        blob_name = raw["blob"]
        digest = raw["sha256"]
        size = raw["size"]
        mode = raw["mode"]
        if (
            not isinstance(relative, str)
            or not _safe_relative_path(relative)
            or relative in seen
        ):
            raise SnapshotError(f"snapshot entry {index} has an unsafe path")
        seen.add(relative)
        if blob_name != f"{index:04d}.bin":
            raise SnapshotError(f"snapshot entry {index} has an invalid blob name")
        if not isinstance(digest, str) or SHA256_PATTERN.fullmatch(digest) is None:
            raise SnapshotError(f"snapshot entry {index} has an invalid digest")
        if not isinstance(size, int) or isinstance(size, bool) or size < 0:
            raise SnapshotError(f"snapshot entry {index} has an invalid size")
        if (
            not isinstance(mode, int)
            or isinstance(mode, bool)
            or mode < 0
            or mode > 0o7777
        ):
            raise SnapshotError(f"snapshot entry {index} has an invalid mode")
        blob = snapshot / blob_name
        if blob.is_symlink() or not blob.is_file():
            raise SnapshotError(f"snapshot blob is missing or a symlink: {blob}")
        data = blob.read_bytes()
        if len(data) != size or _sha256(data) != digest:
            raise SnapshotError(f"snapshot blob failed integrity verification: {blob}")

        target = root / relative
        parent = target.parent
        try:
            resolved_parent = parent.resolve(strict=True)
        except OSError as exc:
            raise SnapshotError(f"restore parent is unavailable: {parent}: {exc}") from exc
        if resolved_parent != parent or not resolved_parent.is_dir():
            raise SnapshotError(f"restore parent must not traverse a symlink: {parent}")
        if target.exists() and target.is_dir() and not target.is_symlink():
            raise SnapshotError(f"restore target is a directory: {target}")
        restored.append((target, data, mode))
    return snapshot, restored


def _atomic_restore_file(path: Path, data: bytes, mode: int) -> None:
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.matrix-restore.", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        os.fchmod(descriptor, mode)
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        _fsync_directory(path.parent)
    finally:
        temporary.unlink(missing_ok=True)


def restore(
    root: Path,
    snapshot: Path,
    *,
    remove_snapshot: bool = False,
) -> None:
    snapshot, entries = _load_snapshot(root, snapshot)
    for target, data, mode in entries:
        _atomic_restore_file(target, data, mode)
    for target, expected_data, expected_mode in entries:
        if target.is_symlink() or not target.is_file():
            raise SnapshotError(f"restored target is missing or a symlink: {target}")
        actual_data = target.read_bytes()
        actual_mode = stat.S_IMODE(target.stat().st_mode)
        if actual_data != expected_data or actual_mode != expected_mode:
            raise SnapshotError(f"restored target failed byte/mode verification: {target}")
    if remove_snapshot:
        parent = snapshot.parent
        shutil.rmtree(snapshot)
        _fsync_directory(parent)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--snapshot", type=Path, required=True)
    subparsers = parser.add_subparsers(dest="action", required=True)
    capture_parser = subparsers.add_parser("capture")
    capture_parser.add_argument("--path", action="append", required=True)
    restore_parser = subparsers.add_parser("restore")
    restore_parser.add_argument("--remove-snapshot", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.action == "capture":
            capture(args.root, args.snapshot, args.path)
            print(f"[PASS] Captured {len(args.path)} Matrix source files: {args.snapshot}")
        else:
            restore(
                args.root,
                args.snapshot,
                remove_snapshot=args.remove_snapshot,
            )
            print(f"[PASS] Restored Matrix source files: {args.snapshot}")
    except (OSError, SnapshotError) as exc:
        print(f"[ERROR] Matrix source snapshot failed: {exc}", file=os.sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
