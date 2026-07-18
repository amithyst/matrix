#!/usr/bin/env python3
"""Verify and stage the narrowly-scoped Matrix cooked-camera overlay.

The external bundle is never trusted by name alone.  Every operation rejects
relative or symlinked paths, non-regular entries, unexpected names, size/hash
mismatches, and malformed contracts.  Installation and active removal use a
same-filesystem directory rename so UE never observes a partial active bundle.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from dataclasses import dataclass
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import secrets
import stat
import sys
from typing import Iterator, Sequence


SCHEMA_VERSION = 1
OVERLAY_VERSION = 3
OVERLAY_ID = "matrix-centered-camera-custom-v3"
STEM = "pakchunk99-MatrixCentered-Linux_P"
SUPPORTED_CLASS = "MujocoSim_Custom_C"
MODE = "centered"
SCOPE = ("MujocoSim_Custom", "Spectator")
RUNTIME_DIRECTORY = Path(
    "src/UeSim/Linux/zsibot_mujoco_ue/Saved/Paks/MatrixCenteredCameraActive"
)
ACTIVE_NAME = RUNTIME_DIRECTORY.name
LOCK_NAME = ".matrix-centered-camera-overlay.lock"
INSTALL_PREFIX = f".{ACTIVE_NAME}.install-"
REMOVE_PREFIX = f".{ACTIVE_NAME}.remove-"
PURGE_PREFIX = f".{ACTIVE_NAME}.purge-"
SUFFIXES = (".pak", ".utoc", ".ucas")
SHA256_RE = re.compile(r"[0-9a-f]{64}\Z")
CONTRACT_KEYS = {
    "schema_version",
    "overlay_version",
    "overlay_id",
    "stem",
    "runtime_directory",
    "mode",
    "scope",
    "supported_class",
    "files",
}
FILE_KEYS = {"name", "size", "sha256"}
PINNED_ARTIFACTS = {
    f"{STEM}.pak": (
        339,
        "b17dfaf284d60bef70d70dac05a32c74723afe689764628eb25cf8fdb9424487",
    ),
    f"{STEM}.utoc": (
        554,
        "6e95033e880fe2537e304317e2189c1ca5943f57acc3eb50ab439c26044afe9a",
    ),
    f"{STEM}.ucas": (
        34423,
        "f0fd22f538cb6d95c6e4e501c3aa5953247ba718a3e1cc4d218ce3f320c0c430",
    ),
}


class OverlayError(RuntimeError):
    """A fail-closed overlay verification or lifecycle error."""


@dataclass(frozen=True)
class Artifact:
    name: str
    size: int
    sha256: str


@dataclass(frozen=True)
class Contract:
    path: Path
    artifacts: tuple[Artifact, ...]

    @property
    def by_name(self) -> dict[str, Artifact]:
        return {artifact.name: artifact for artifact in self.artifacts}


def _reject_duplicate_keys(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise OverlayError(f"duplicate JSON key in overlay contract: {key}")
        result[key] = value
    return result


def _absolute_path(raw: str | os.PathLike[str], label: str) -> Path:
    path = Path(raw)
    if not path.is_absolute():
        raise OverlayError(f"{label} must be an absolute path: {path}")
    if ".." in path.parts:
        raise OverlayError(f"{label} must not contain '..': {path}")
    return path


def _reject_symlink_components(
    path: Path,
    label: str,
    *,
    require_leaf: bool,
) -> None:
    """Reject every existing symlink from the filesystem root through path."""

    current = Path(path.anchor)
    missing = False
    for part in path.parts[1:]:
        current /= part
        if missing:
            continue
        try:
            metadata = os.lstat(current)
        except FileNotFoundError:
            missing = True
            continue
        if stat.S_ISLNK(metadata.st_mode):
            raise OverlayError(f"{label} contains a symlink component: {current}")
    if require_leaf and missing:
        raise OverlayError(f"{label} does not exist: {path}")


def _require_regular_file(path: Path, label: str) -> None:
    _reject_symlink_components(path, label, require_leaf=True)
    metadata = os.lstat(path)
    if not stat.S_ISREG(metadata.st_mode):
        raise OverlayError(f"{label} is not a regular file: {path}")


def _require_directory(path: Path, label: str) -> None:
    _reject_symlink_components(path, label, require_leaf=True)
    metadata = os.lstat(path)
    if not stat.S_ISDIR(metadata.st_mode):
        raise OverlayError(f"{label} is not a directory: {path}")


def _load_json(path: Path) -> object:
    _require_regular_file(path, "overlay contract")
    if path.stat().st_size > 64 * 1024:
        raise OverlayError(f"overlay contract is unexpectedly large: {path}")
    try:
        with path.open("r", encoding="utf-8") as handle:
            return json.load(handle, object_pairs_hook=_reject_duplicate_keys)
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise OverlayError(f"cannot parse overlay contract {path}: {exc}") from exc


def load_contract(raw_path: str | os.PathLike[str]) -> Contract:
    path = _absolute_path(raw_path, "overlay contract")
    payload = _load_json(path)
    if not isinstance(payload, dict):
        raise OverlayError("overlay contract root must be an object")
    keys = set(payload)
    if keys != CONTRACT_KEYS:
        raise OverlayError(
            "overlay contract keys differ from the fixed schema: "
            f"missing={sorted(CONTRACT_KEYS - keys)} extra={sorted(keys - CONTRACT_KEYS)}"
        )
    fixed_values = (
        ("schema_version", SCHEMA_VERSION),
        ("overlay_version", OVERLAY_VERSION),
        ("overlay_id", OVERLAY_ID),
        ("stem", STEM),
        ("runtime_directory", RUNTIME_DIRECTORY.as_posix()),
        ("mode", MODE),
        ("scope", list(SCOPE)),
        ("supported_class", SUPPORTED_CLASS),
    )
    for field, expected in fixed_values:
        if payload[field] != expected:
            raise OverlayError(
                f"overlay contract {field} must be {expected!r}, got {payload[field]!r}"
            )

    raw_files = payload["files"]
    if not isinstance(raw_files, list) or len(raw_files) != len(SUFFIXES):
        raise OverlayError("overlay contract must list exactly pak, utoc, and ucas")
    artifacts: list[Artifact] = []
    for raw_file in raw_files:
        if not isinstance(raw_file, dict) or set(raw_file) != FILE_KEYS:
            raise OverlayError("each overlay file must contain only name, size, and sha256")
        name = raw_file["name"]
        size = raw_file["size"]
        digest = raw_file["sha256"]
        if not isinstance(name, str) or Path(name).name != name:
            raise OverlayError(f"overlay artifact name must be a basename: {name!r}")
        if isinstance(size, bool) or not isinstance(size, int) or size <= 0:
            raise OverlayError(f"overlay artifact size must be a positive integer: {name}")
        if not isinstance(digest, str) or not SHA256_RE.fullmatch(digest):
            raise OverlayError(f"overlay artifact sha256 is invalid: {name}")
        artifacts.append(Artifact(name=name, size=size, sha256=digest))

    expected_names = {f"{STEM}{suffix}" for suffix in SUFFIXES}
    actual_names = {artifact.name for artifact in artifacts}
    if actual_names != expected_names or len(actual_names) != len(artifacts):
        raise OverlayError(
            "overlay contract artifact names must be the fixed pak/utoc/ucas set"
        )
    actual_artifacts = {
        artifact.name: (artifact.size, artifact.sha256) for artifact in artifacts
    }
    if actual_artifacts != PINNED_ARTIFACTS:
        raise OverlayError(
            "overlay contract artifact sizes and sha256 values differ from pinned v3"
        )
    return Contract(path=path, artifacts=tuple(artifacts))


def _open_directory(path: Path) -> int:
    required_flags = ("O_DIRECTORY", "O_NOFOLLOW")
    if any(not hasattr(os, name) for name in required_flags):
        raise OverlayError("overlay lifecycle requires Linux O_DIRECTORY and O_NOFOLLOW")
    try:
        return os.open(
            path,
            os.O_RDONLY | os.O_CLOEXEC | os.O_DIRECTORY | os.O_NOFOLLOW,
        )
    except OSError as exc:
        raise OverlayError(
            f"cannot open overlay directory without following links: {path}"
        ) from exc


def _hash_open_file(file_fd: int) -> str:
    digest = hashlib.sha256()
    os.lseek(file_fd, 0, os.SEEK_SET)
    while True:
        block = os.read(file_fd, 1024 * 1024)
        if not block:
            break
        digest.update(block)
    return digest.hexdigest()


def _verify_directory(
    directory: Path,
    contract: Contract,
    *,
    allow_subset: bool = False,
) -> None:
    _require_directory(directory, "overlay directory")
    expected = contract.by_name
    directory_fd = _open_directory(directory)
    try:
        names = os.listdir(directory_fd)
        actual_names = set(names)
        if len(names) != len(actual_names):
            raise OverlayError(f"overlay directory contains duplicate names: {directory}")
        extras = actual_names - set(expected)
        missing = set(expected) - actual_names
        if extras or (missing and not allow_subset):
            raise OverlayError(
                f"overlay directory inventory mismatch at {directory}: "
                f"missing={sorted(missing)} extra={sorted(extras)}"
            )
        for name in sorted(actual_names):
            artifact = expected[name]
            metadata = os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
            if not stat.S_ISREG(metadata.st_mode):
                raise OverlayError(f"overlay artifact is not a regular file: {directory / name}")
            flags = os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW
            file_fd = os.open(name, flags, dir_fd=directory_fd)
            try:
                opened_metadata = os.fstat(file_fd)
                if (
                    opened_metadata.st_dev != metadata.st_dev
                    or opened_metadata.st_ino != metadata.st_ino
                ):
                    raise OverlayError(f"overlay artifact changed while opening: {name}")
                if opened_metadata.st_size != artifact.size:
                    raise OverlayError(
                        f"overlay artifact size mismatch for {name}: "
                        f"expected={artifact.size} actual={opened_metadata.st_size}"
                    )
                actual_digest = _hash_open_file(file_fd)
                if actual_digest != artifact.sha256:
                    raise OverlayError(
                        f"overlay artifact sha256 mismatch for {name}: "
                        f"expected={artifact.sha256} actual={actual_digest}"
                    )
            finally:
                os.close(file_fd)
        if set(os.listdir(directory_fd)) != actual_names:
            raise OverlayError(f"overlay directory changed during verification: {directory}")
    except OSError as exc:
        raise OverlayError(f"cannot verify overlay directory {directory}: {exc}") from exc
    finally:
        os.close(directory_fd)


def verify_bundle(bundle: str | os.PathLike[str], contract: Contract) -> Path:
    path = _absolute_path(bundle, "overlay bundle")
    _verify_directory(path, contract)
    return path


def _ensure_runtime_parent(project_root: Path) -> Path:
    _require_directory(project_root, "Matrix project root")
    current = project_root
    for part in RUNTIME_DIRECTORY.parent.parts:
        current /= part
        try:
            metadata = os.lstat(current)
        except FileNotFoundError:
            try:
                os.mkdir(current, 0o755)
            except OSError as exc:
                raise OverlayError(f"cannot create overlay runtime directory: {current}") from exc
            metadata = os.lstat(current)
        if stat.S_ISLNK(metadata.st_mode) or not stat.S_ISDIR(metadata.st_mode):
            raise OverlayError(f"overlay runtime path is not a real directory: {current}")
    return current


def _runtime_parent_if_present(project_root: Path) -> Path | None:
    _require_directory(project_root, "Matrix project root")
    current = project_root
    for part in RUNTIME_DIRECTORY.parent.parts:
        current /= part
        try:
            metadata = os.lstat(current)
        except FileNotFoundError:
            return None
        if stat.S_ISLNK(metadata.st_mode) or not stat.S_ISDIR(metadata.st_mode):
            raise OverlayError(f"overlay runtime path is not a real directory: {current}")
    return current


@contextmanager
def _operation_lock(parent: Path) -> Iterator[None]:
    lock_path = parent / LOCK_NAME
    flags = os.O_RDWR | os.O_CREAT | os.O_CLOEXEC | os.O_NOFOLLOW
    try:
        lock_fd = os.open(lock_path, flags, 0o600)
    except OSError as exc:
        raise OverlayError(f"cannot open overlay lifecycle lock: {lock_path}") from exc
    try:
        metadata = os.fstat(lock_fd)
        if not stat.S_ISREG(metadata.st_mode):
            raise OverlayError(f"overlay lifecycle lock is not a regular file: {lock_path}")
        fcntl.flock(lock_fd, fcntl.LOCK_EX)
        yield
    finally:
        os.close(lock_fd)


def _unique_sibling(parent: Path, prefix: str) -> Path:
    for _ in range(20):
        candidate = parent / f"{prefix}{os.getpid()}-{secrets.token_hex(8)}"
        if not os.path.lexists(candidate):
            return candidate
    raise OverlayError(f"cannot allocate a unique overlay lifecycle path in {parent}")


def _copy_artifact(source_dir: Path, destination_dir: Path, artifact: Artifact) -> None:
    source_directory_fd = _open_directory(source_dir)
    destination_directory_fd = _open_directory(destination_dir)
    source_fd = -1
    destination_fd = -1
    try:
        source_fd = os.open(
            artifact.name,
            os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW,
            dir_fd=source_directory_fd,
        )
        source_metadata = os.fstat(source_fd)
        if not stat.S_ISREG(source_metadata.st_mode):
            raise OverlayError(f"overlay source is not regular: {artifact.name}")
        destination_fd = os.open(
            artifact.name,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC | os.O_NOFOLLOW,
            0o444,
            dir_fd=destination_directory_fd,
        )
        digest = hashlib.sha256()
        total = 0
        while True:
            block = os.read(source_fd, 1024 * 1024)
            if not block:
                break
            digest.update(block)
            total += len(block)
            view = memoryview(block)
            while view:
                written = os.write(destination_fd, view)
                view = view[written:]
        if total != artifact.size or digest.hexdigest() != artifact.sha256:
            raise OverlayError(f"overlay source changed while copying: {artifact.name}")
        os.fsync(destination_fd)
    except OSError as exc:
        raise OverlayError(f"cannot copy overlay artifact {artifact.name}: {exc}") from exc
    finally:
        if destination_fd >= 0:
            os.close(destination_fd)
        if source_fd >= 0:
            os.close(source_fd)
        os.close(destination_directory_fd)
        os.close(source_directory_fd)


def _delete_known_directory(
    directory: Path,
    contract: Contract,
    *,
    allow_subset: bool,
) -> None:
    _verify_directory(directory, contract, allow_subset=allow_subset)
    directory_fd = _open_directory(directory)
    try:
        names = os.listdir(directory_fd)
        for name in names:
            os.unlink(name, dir_fd=directory_fd)
        os.fsync(directory_fd)
    except OSError as exc:
        raise OverlayError(f"cannot remove verified overlay files from {directory}") from exc
    finally:
        os.close(directory_fd)
    try:
        os.rmdir(directory)
    except OSError as exc:
        raise OverlayError(f"cannot remove verified overlay directory: {directory}") from exc


def _fsync_directory(directory: Path) -> None:
    directory_fd = _open_directory(directory)
    try:
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)


def install(
    project_root: str | os.PathLike[str],
    bundle: str | os.PathLike[str],
    contract: Contract,
) -> Path:
    root = _absolute_path(project_root, "Matrix project root")
    source = verify_bundle(bundle, contract)
    parent = _ensure_runtime_parent(root)
    active = root / RUNTIME_DIRECTORY
    with _operation_lock(parent):
        if os.path.lexists(active):
            raise OverlayError(
                f"active overlay path already exists; run purge-stale first: {active}"
            )
        staging = _unique_sibling(parent, INSTALL_PREFIX)
        try:
            os.mkdir(staging, 0o700)
            for artifact in contract.artifacts:
                _copy_artifact(source, staging, artifact)
            _verify_directory(staging, contract)
            verify_bundle(source, contract)
            _fsync_directory(staging)
            os.rename(staging, active)
            _fsync_directory(parent)
        except Exception:
            if os.path.lexists(staging):
                try:
                    _delete_known_directory(staging, contract, allow_subset=True)
                except OverlayError:
                    pass
            raise
    return active


def _atomic_remove_active(active: Path, parent: Path, contract: Contract) -> None:
    _verify_directory(active, contract)
    retired = _unique_sibling(parent, REMOVE_PREFIX)
    try:
        os.rename(active, retired)
        _fsync_directory(parent)
    except OSError as exc:
        raise OverlayError(f"cannot atomically retire active overlay: {active}") from exc
    _delete_known_directory(retired, contract, allow_subset=False)
    _fsync_directory(parent)


def remove(project_root: str | os.PathLike[str], contract: Contract) -> bool:
    root = _absolute_path(project_root, "Matrix project root")
    parent = _runtime_parent_if_present(root)
    if parent is None:
        return False
    active = root / RUNTIME_DIRECTORY
    with _operation_lock(parent):
        if not os.path.lexists(active):
            return False
        _atomic_remove_active(active, parent, contract)
    return True


def purge_stale(project_root: str | os.PathLike[str], contract: Contract) -> int:
    root = _absolute_path(project_root, "Matrix project root")
    parent = _runtime_parent_if_present(root)
    if parent is None:
        return 0
    purged = 0
    with _operation_lock(parent):
        names = os.listdir(parent)
        candidates = [
            name
            for name in names
            if name == ACTIVE_NAME
            or name.startswith(INSTALL_PREFIX)
            or name.startswith(REMOVE_PREFIX)
            or name.startswith(PURGE_PREFIX)
        ]
        for name in sorted(candidates):
            candidate = parent / name
            _verify_directory(candidate, contract, allow_subset=True)
            if name == ACTIVE_NAME:
                retired = _unique_sibling(parent, PURGE_PREFIX)
                try:
                    os.rename(candidate, retired)
                    _fsync_directory(parent)
                except OSError as exc:
                    raise OverlayError(
                        f"cannot atomically retire stale overlay: {candidate}"
                    ) from exc
                candidate = retired
            _delete_known_directory(candidate, contract, allow_subset=True)
            purged += 1
        _fsync_directory(parent)
    return purged


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    verify_parser = subparsers.add_parser("verify-bundle")
    verify_parser.add_argument("--contract", required=True)
    verify_parser.add_argument("--bundle", required=True)

    install_parser = subparsers.add_parser("install")
    install_parser.add_argument("--contract", required=True)
    install_parser.add_argument("--bundle", required=True)
    install_parser.add_argument("--project-root", required=True)

    remove_parser = subparsers.add_parser("remove")
    remove_parser.add_argument("--contract", required=True)
    remove_parser.add_argument("--project-root", required=True)

    purge_parser = subparsers.add_parser("purge-stale")
    purge_parser.add_argument("--contract", required=True)
    purge_parser.add_argument("--project-root", required=True)
    return parser


def _emit(action: str, **fields: object) -> None:
    print(json.dumps({"action": action, **fields}, sort_keys=True))


def main(argv: Sequence[str] | None = None) -> int:
    arguments = _parser().parse_args(argv)
    try:
        contract = load_contract(arguments.contract)
        if arguments.command == "verify-bundle":
            bundle = verify_bundle(arguments.bundle, contract)
            _emit("verify-bundle", bundle=os.fspath(bundle), version=OVERLAY_VERSION)
        elif arguments.command == "install":
            active = install(arguments.project_root, arguments.bundle, contract)
            _emit("install", active=os.fspath(active), version=OVERLAY_VERSION)
        elif arguments.command == "remove":
            removed = remove(arguments.project_root, contract)
            _emit("remove", removed=removed, version=OVERLAY_VERSION)
        elif arguments.command == "purge-stale":
            count = purge_stale(arguments.project_root, contract)
            _emit("purge-stale", purged=count, version=OVERLAY_VERSION)
        else:  # pragma: no cover - argparse owns the command set.
            raise OverlayError(f"unsupported overlay command: {arguments.command}")
    except (OverlayError, OSError) as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
