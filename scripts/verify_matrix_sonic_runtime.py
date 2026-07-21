#!/usr/bin/env python3
"""Verify the locked Matrix + SONIC runtime without modifying the host."""

from __future__ import annotations

import argparse
import base64
import configparser
import csv
import hashlib
import io
import json
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
import sys
from typing import Any
import zipfile


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LOCK = REPO_ROOT / "config/runtime/matrix-sonic.lock.json"
DEFAULT_RUNTIME = REPO_ROOT / "outputs/runtime/matrix-sonic-native-v2"
LARGE_FILE_THRESHOLD = 64 * 1024 * 1024
MAX_GLIBC_MINOR = 35
LEGACY_MANYLINUX_PLATFORMS = {
    "manylinux1_x86_64",
    "manylinux2010_x86_64",
    "manylinux2014_x86_64",
}
HOST_PROFILES = tuple(
    sorted(path.stem for path in (REPO_ROOT / "config/hosts").glob("*.env"))
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def sha256_tree(root: Path) -> tuple[str, int]:
    """Hash a directory's regular files with a deterministic manifest encoding.

    Each record is ``relative POSIX path + NUL + file SHA256 hex + newline``;
    records are ordered by relative POSIX path. Directories are not records.
    Symlinks and filesystem objects other than regular files/directories are
    rejected rather than followed or silently omitted.
    """

    try:
        root_mode = root.lstat().st_mode
    except OSError as exc:
        raise ValueError(f"cannot inspect tree root {root}: {exc}") from exc
    if stat.S_ISLNK(root_mode):
        raise ValueError(f"tree root is a symlink: {root}")
    if not stat.S_ISDIR(root_mode):
        raise ValueError(f"tree root is not a directory: {root}")

    files: list[tuple[str, Path]] = []
    pending = [root]
    while pending:
        directory = pending.pop()
        try:
            with os.scandir(directory) as scanner:
                entries = sorted(scanner, key=lambda entry: entry.name)
        except OSError as exc:
            raise ValueError(
                f"cannot enumerate runtime tree {directory}: {exc}"
            ) from exc
        for entry in entries:
            entry_path = Path(entry.path)
            relative = entry_path.relative_to(root).as_posix()
            try:
                mode = entry.stat(follow_symlinks=False).st_mode
            except OSError as exc:
                raise ValueError(
                    f"cannot inspect runtime tree entry {relative}: {exc}"
                ) from exc
            if stat.S_ISLNK(mode):
                raise ValueError(f"runtime tree contains symlink: {relative}")
            if stat.S_ISDIR(mode):
                pending.append(entry_path)
            elif stat.S_ISREG(mode):
                files.append((relative, entry_path))
            else:
                raise ValueError(f"runtime tree contains non-regular file: {relative}")

    digest = hashlib.sha256()
    for relative, path in sorted(files, key=lambda item: item[0]):
        try:
            encoded_relative = relative.encode("utf-8")
        except UnicodeEncodeError as exc:
            raise ValueError(
                f"runtime tree path is not valid UTF-8: {relative!r}"
            ) from exc
        try:
            file_sha256 = sha256_file(path)
        except OSError as exc:
            raise ValueError(
                f"cannot hash runtime tree file {relative}: {exc}"
            ) from exc
        digest.update(encoded_relative)
        digest.update(b"\0")
        digest.update(file_sha256.encode("ascii"))
        digest.update(b"\n")
    return digest.hexdigest(), len(files)


def load_lock(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        payload = json.load(stream)
    validate_schema(payload)
    return payload


def validate_policy_manifest_files(
    lock: dict[str, Any], matrix_root: Path
) -> None:
    """Bind policy-candidate declarations to the Matrix runtime lock."""

    root = matrix_root.resolve()
    for entry in lock["policy_slots"]["manifests"]:
        relative = str(entry["path"])
        path = (root / relative).resolve()
        try:
            path.relative_to(root)
        except ValueError as exc:  # pragma: no cover - schema rejects traversal.
            raise ValueError(
                f"policy slot manifest escapes matrix root: {relative}"
            ) from exc
        if not path.is_file():
            raise ValueError(f"policy slot manifest is missing: {relative}")
        try:
            actual = sha256_file(path)
        except OSError as exc:
            raise ValueError(
                f"cannot hash policy slot manifest {relative}: {exc}"
            ) from exc
        if actual != entry["sha256"]:
            raise ValueError(
                f"policy slot manifest SHA256 mismatch: {relative}"
            )


def is_sha256(value: object) -> bool:
    return isinstance(value, str) and re.fullmatch(r"[0-9a-f]{64}", value) is not None


def is_safe_relative_path(value: object) -> bool:
    if not isinstance(value, str) or not value or "\\" in value:
        return False
    path = Path(value)
    return not path.is_absolute() and ".." not in path.parts and path.as_posix() == value


def is_safe_root_name(value: object) -> bool:
    return (
        isinstance(value, str)
        and re.fullmatch(r"[a-z][a-z0-9_-]*", value) is not None
    )


def canonical_distribution_name(value: str) -> str:
    return re.sub(r"[-_.]+", "-", value).lower()


def parse_wheel_filename(
    filename: str,
) -> tuple[str, str, set[str], set[str], set[str]]:
    """Parse identity and compatibility tags from a normalized wheel filename.

    This verifier intentionally has no dependency on ``packaging`` because it
    is also used to bootstrap the environment that provides that package.
    Wheel distribution/version components cannot contain ``-`` after wheel
    escaping, so splitting the four right-most fields is unambiguous.
    """

    if Path(filename).name != filename or not filename.endswith(".whl"):
        raise ValueError("invalid wheel filename")
    wheel_parts = filename[:-4].rsplit("-", 3)
    if len(wheel_parts) != 4:
        raise ValueError("invalid wheel filename fields")
    name_and_version, python_tag, abi_tag, platform_tag = wheel_parts
    identity = name_and_version.split("-")
    if len(identity) not in (2, 3):
        raise ValueError("missing distribution/version or invalid build tag")
    distribution, version, *build = identity
    if re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.]*", distribution) is None:
        raise ValueError("invalid wheel distribution")
    if re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.+!]*", version) is None:
        raise ValueError("invalid wheel version")
    if build and re.fullmatch(r"[0-9][A-Za-z0-9_]*", build[0]) is None:
        raise ValueError("invalid wheel build tag")

    tags: list[set[str]] = []
    for label, compressed in (
        ("Python", python_tag),
        ("ABI", abi_tag),
        ("platform", platform_tag),
    ):
        values = compressed.split(".")
        if not values or any(
            re.fullmatch(r"[A-Za-z0-9_]+", value) is None for value in values
        ):
            raise ValueError(f"invalid {label} tag")
        tags.append(set(values))
    return distribution, version, tags[0], tags[1], tags[2]


def supported_manylinux_platform(tag: str) -> bool:
    if tag in LEGACY_MANYLINUX_PLATFORMS:
        return True
    match = re.fullmatch(r"manylinux_2_(0|[1-9][0-9]*)_x86_64", tag)
    return match is not None and int(match.group(1)) <= MAX_GLIBC_MINOR


def wheel_tags_compatible(
    python_tags: set[str], abi_tags: set[str], platform_tags: set[str]
) -> bool:
    def python_abi_compatible(python_tag: str, abi_tag: str) -> bool:
        if abi_tag == "none":
            return python_tag in {"py3", "py310", "cp310"}
        if abi_tag == "cp310":
            return python_tag == "cp310"
        if abi_tag != "abi3":
            return False
        match = re.fullmatch(r"cp3([0-9]+)", python_tag)
        return match is not None and 2 <= int(match.group(1)) <= 10

    python_abi_ok = any(
        python_abi_compatible(python_tag, abi_tag)
        for python_tag in python_tags
        for abi_tag in abi_tags
    )
    # Every compressed platform alternative must stay inside the target
    # contract. This rejects a wheel that advertises a future or malformed
    # manylinux tag even if another compressed tag happens to be compatible.
    platform_ok = bool(platform_tags) and all(
        tag in {"any", "linux_x86_64"} or supported_manylinux_platform(tag)
        for tag in platform_tags
    )
    return python_abi_ok and platform_ok


def parse_pinned_requirements(path: Path) -> dict[str, tuple[str, str]]:
    pins: dict[str, tuple[str, str]] = {}
    for line_number, raw_line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        match = re.fullmatch(r"([A-Za-z0-9][A-Za-z0-9_.-]*)==([^\s;]+)", line)
        if match is None:
            raise ValueError(
                f"requirements line {line_number} is not an exact distribution==version pin"
            )
        distribution, version = match.groups()
        canonical = canonical_distribution_name(distribution)
        if canonical in pins:
            raise ValueError(f"duplicate requirements distribution: {distribution}")
        pins[canonical] = (distribution, version)
    if not pins:
        raise ValueError("requirements file has no exact pins")
    return pins


def validate_runtime_entries(lock: dict[str, Any], key: str) -> None:
    entries = lock.get(key)
    if not isinstance(entries, list) or not entries:
        raise ValueError(f"{key} must be a non-empty list")
    identities: set[tuple[str, str]] = set()
    for entry in entries:
        if not isinstance(entry, dict):
            raise ValueError(f"invalid {key} entry: expected an object")
        for field in ("root", "path", "sha256"):
            if not isinstance(entry.get(field), str) or not entry[field]:
                raise ValueError(f"invalid {key} entry: missing {field}")
        identity = (entry["root"], entry["path"])
        if identity in identities:
            raise ValueError(f"duplicate {key} entry: {identity}")
        identities.add(identity)
        if not is_safe_root_name(entry["root"]):
            raise ValueError(f"unsafe {key} root: {identity}")
        if not is_safe_relative_path(entry["path"]):
            raise ValueError(f"unsafe {key} path: {identity}")
        if not is_sha256(entry["sha256"]):
            raise ValueError(f"invalid SHA256 for {key} entry {identity}")
        verification = entry.get("verification")
        if verification is not None and (
            not isinstance(verification, str)
            or "provisional" in verification.lower()
        ):
            raise ValueError(f"provisional {key} entry is forbidden: {identity}")
        lowered = f"{entry['root']}/{entry['path']}".lower()
        if entry["root"] in {"aue", "bridge"} or any(
            value in lowered
            for value in ("androidtwin", "aue-sim", "g1_sonic_sim_udp_dds_bridge")
        ):
            raise ValueError(f"legacy AndroidTwin {key} entry is forbidden: {identity}")


def validate_schema(lock: dict[str, Any]) -> None:
    required = {
        "schema_version",
        "runtime_id",
        "matrix_release",
        "python",
        "pico",
        "source_revisions",
        "inference",
        "runtime_files",
        "runtime_trees",
        "acceptance",
    }
    missing = sorted(required.difference(lock))
    if missing:
        raise ValueError(f"runtime lock is missing keys: {', '.join(missing)}")
    if lock["schema_version"] != 2:
        raise ValueError(f"unsupported runtime lock schema: {lock['schema_version']}")

    policy_slots = lock.get("policy_slots")
    if not isinstance(policy_slots, dict) or set(policy_slots) != {"manifests"}:
        raise ValueError("policy_slots must contain exactly manifests")
    policy_manifests = policy_slots["manifests"]
    if not isinstance(policy_manifests, list) or not policy_manifests:
        raise ValueError("policy_slots.manifests must be a non-empty list")
    policy_manifest_paths: set[str] = set()
    for entry in policy_manifests:
        if not isinstance(entry, dict) or set(entry) != {"path", "sha256"}:
            raise ValueError("policy slot manifest locks must contain path/sha256")
        path = entry.get("path")
        if (
            not isinstance(path, str)
            or not is_safe_relative_path(path)
            or not path.startswith("config/runtime/policy-slots/")
            or not path.endswith(".json")
            or path in policy_manifest_paths
        ):
            raise ValueError(f"invalid or duplicate policy slot manifest path: {path!r}")
        policy_manifest_paths.add(path)
        if not is_sha256(entry.get("sha256")):
            raise ValueError(f"invalid policy slot manifest SHA256: {path}")

    matrix_release = lock.get("matrix_release", {})
    installed_files = matrix_release.get("installed_files")
    if not isinstance(installed_files, list) or not installed_files:
        raise ValueError("matrix_release.installed_files must be a non-empty list")
    installed_paths: set[str] = set()
    for entry in installed_files:
        if not isinstance(entry, dict):
            raise ValueError("matrix_release.installed_files entries must be objects")
        path = entry.get("path")
        if (
            not isinstance(path, str)
            or not is_safe_relative_path(path)
            or path in installed_paths
        ):
            raise ValueError(f"invalid or duplicate installed file path: {path!r}")
        installed_paths.add(path)
        if not isinstance(entry.get("size"), int) or entry["size"] <= 0:
            raise ValueError(f"invalid installed file size: {path}")
        if not is_sha256(entry.get("sha256")):
            raise ValueError(f"invalid installed file SHA256: {path}")
    installed_trees = matrix_release.get("installed_trees")
    if not isinstance(installed_trees, list):
        raise ValueError("matrix_release.installed_trees must be a list")
    installed_tree_paths: set[str] = set()
    for entry in installed_trees:
        if not isinstance(entry, dict):
            raise ValueError("matrix_release.installed_trees entries must be objects")
        path = entry.get("path")
        if (
            not isinstance(path, str)
            or not is_safe_relative_path(path)
            or path in installed_tree_paths
        ):
            raise ValueError(f"invalid or duplicate installed tree path: {path!r}")
        installed_tree_paths.add(path)
        if not is_sha256(entry.get("sha256")):
            raise ValueError(f"invalid installed tree SHA256: {path}")

    python_lock = lock.get("python", {})
    for key in (
        "version",
        "soabi",
        "machine",
        "requirements",
        "requirements_sha256",
        "wheelhouse_manifest_sha256",
    ):
        if not isinstance(python_lock.get(key), str) or not python_lock[key]:
            raise ValueError(f"python.{key} must be a non-empty string")
    if re.fullmatch(r"[0-9]+\.[0-9]+", python_lock["version"]) is None:
        raise ValueError("python.version must contain only the locked major.minor")
    if not is_safe_relative_path(python_lock["requirements"]):
        raise ValueError("python.requirements must be a safe project-relative path")
    for key in ("requirements_sha256", "wheelhouse_manifest_sha256"):
        if not is_sha256(python_lock[key]):
            raise ValueError(f"python.{key} must be a lowercase SHA256")

    pico_lock = lock.get("pico", {})
    for key in (
        "delivery",
        "python_version",
        "python_soabi",
        "machine",
        "distribution",
        "version",
        "wheel_filename",
        "wheel_sha256",
        "runtime_overlay",
        "runtime_overlay_sha256",
    ):
        if not isinstance(pico_lock.get(key), str) or not pico_lock[key]:
            raise ValueError(f"pico.{key} must be a non-empty string")
    if pico_lock["delivery"] != "external-controlled-environment":
        raise ValueError(
            "pico.delivery must be external-controlled-environment; "
            "the runtime bundle does not carry the private PICO wheel"
        )
    for key in ("wheel_sha256", "runtime_overlay_sha256"):
        if not is_sha256(pico_lock[key]):
            raise ValueError(f"pico.{key} must be a lowercase SHA256")
    if (
        Path(pico_lock["wheel_filename"]).name != pico_lock["wheel_filename"]
        or not pico_lock["wheel_filename"].endswith(".whl")
    ):
        raise ValueError("pico.wheel_filename must be a safe wheel basename")
    try:
        (
            pico_wheel_distribution,
            pico_wheel_version,
            pico_python_tags,
            pico_abi_tags,
            pico_platform_tags,
        ) = parse_wheel_filename(pico_lock["wheel_filename"])
    except ValueError as exc:
        raise ValueError(f"invalid pico.wheel_filename: {exc}") from exc
    if (
        canonical_distribution_name(pico_wheel_distribution)
        != canonical_distribution_name(pico_lock["distribution"])
        or pico_wheel_version != pico_lock["version"]
    ):
        raise ValueError(
            "pico.wheel_filename distribution/version must match pico lock metadata"
        )
    if not wheel_tags_compatible(
        pico_python_tags, pico_abi_tags, pico_platform_tags
    ):
        raise ValueError("pico.wheel_filename is incompatible with CPython 3.10 x86_64")
    if not is_safe_relative_path(pico_lock["runtime_overlay"]):
        raise ValueError("pico.runtime_overlay must be a safe relative path")

    sonic = lock["source_revisions"].get("gr00t_whole_body_control", {})
    if not isinstance(sonic.get("commit"), str) or re.fullmatch(
        r"[0-9a-f]{40}", sonic["commit"]
    ) is None:
        raise ValueError("source_revisions.gr00t_whole_body_control.commit must be a SHA")
    critical_paths = sonic.get("critical_source_paths")
    if not isinstance(critical_paths, list) or not critical_paths:
        raise ValueError(
            "source_revisions.gr00t_whole_body_control.critical_source_paths "
            "must be a non-empty list"
        )
    if len(critical_paths) != len(set(critical_paths)) or not all(
        is_safe_relative_path(path) for path in critical_paths
    ):
        raise ValueError("critical SONIC source paths must be unique safe relative paths")

    validate_runtime_entries(lock, "runtime_files")
    validate_runtime_entries(lock, "runtime_trees")
    runtime_files = {
        (entry["root"], entry["path"]): entry["sha256"]
        for entry in lock["runtime_files"]
    }
    if runtime_files.get(("sonic", pico_lock["runtime_overlay"])) != pico_lock[
        "runtime_overlay_sha256"
    ]:
        raise ValueError("pico runtime overlay must match the SONIC runtime file lock")


def parse_sha256_manifest(path: Path) -> dict[str, str]:
    entries: dict[str, str] = {}
    for line_number, line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        match = re.fullmatch(r"([0-9a-f]{64}) ([ *])(.+)", line)
        if match is None:
            raise ValueError(f"invalid SHA256SUMS line {line_number}")
        digest, _, relative = match.groups()
        if not is_safe_relative_path(relative) or relative == path.name:
            raise ValueError(f"unsafe SHA256SUMS path on line {line_number}: {relative!r}")
        if relative in entries:
            raise ValueError(f"duplicate SHA256SUMS path: {relative}")
        entries[relative] = digest
    if not entries:
        raise ValueError("SHA256SUMS is empty")
    return entries


def verify_wheelhouse(
    wheelhouse: Path, expected_manifest_sha256: str
) -> list[tuple[str, bool, str]]:
    checks: list[tuple[str, bool, str]] = []
    manifest = wheelhouse / "SHA256SUMS"
    if not manifest.is_file() or manifest.is_symlink():
        return [
            ("Python wheelhouse manifest", False, f"missing regular file {manifest}")
        ]

    actual_manifest_sha256 = sha256_file(manifest)
    checks.append(
        (
            "Python wheelhouse manifest",
            actual_manifest_sha256 == expected_manifest_sha256,
            f"sha256={actual_manifest_sha256}",
        )
    )
    try:
        entries = parse_sha256_manifest(manifest)
    except (OSError, UnicodeError, ValueError) as exc:
        checks.append(("Python wheelhouse contents", False, str(exc)))
        return checks

    actual_files: set[str] = set()
    non_regular: list[str] = []
    for path in wheelhouse.rglob("*"):
        if path == manifest:
            continue
        relative = path.relative_to(wheelhouse).as_posix()
        if path.is_symlink():
            non_regular.append(relative)
        elif path.is_dir():
            continue
        elif not path.is_file():
            non_regular.append(relative)
        else:
            actual_files.add(relative)
    listed_files = set(entries)
    missing = sorted(listed_files - actual_files)
    extra = sorted(actual_files - listed_files)
    inventory_ok = not missing and not extra and not non_regular
    inventory_detail: list[str] = []
    if missing:
        inventory_detail.append(f"missing: {', '.join(missing)}")
    if extra:
        inventory_detail.append(f"unlisted: {', '.join(extra)}")
    if non_regular:
        inventory_detail.append(f"non-regular: {', '.join(sorted(non_regular))}")
    checks.append(
        (
            "Python wheelhouse inventory",
            inventory_ok,
            "; ".join(inventory_detail) if inventory_detail else "exact",
        )
    )

    incompatible: list[str] = []
    for relative in sorted(actual_files):
        filename = Path(relative).name
        if relative != filename:
            incompatible.append(f"{relative}:wheel-must-be-at-wheelhouse-root")
            continue
        try:
            _, _, python_tags, abi_tags, platform_tags = parse_wheel_filename(filename)
        except ValueError as exc:
            incompatible.append(f"{relative}:{exc}")
            continue
        if not wheel_tags_compatible(python_tags, abi_tags, platform_tags):
            incompatible.append(f"{relative}:incompatible wheel tags")
    checks.append(
        (
            "Python wheelhouse compatibility",
            not incompatible,
            "; ".join(incompatible) if incompatible else "CPython 3.10 x86_64 wheels only",
        )
    )

    mismatches: list[str] = []
    for relative in sorted(listed_files & actual_files):
        actual = sha256_file(wheelhouse / relative)
        if actual != entries[relative]:
            mismatches.append(relative)
    checks.append(
        (
            "Python wheelhouse contents",
            not mismatches and not missing,
            (
                f"SHA256 mismatch: {', '.join(mismatches)}"
                if mismatches
                else "all listed files verified"
            ),
        )
    )
    return checks


def _compact_failures(values: list[str], *, limit: int = 12) -> str:
    if len(values) <= limit:
        return "; ".join(values)
    return "; ".join(values[:limit]) + f"; ... ({len(values) - limit} more)"


def _wheel_record_site_path(
    record_path: str, wheel_stem: str, *, target_install: bool = False
) -> str | None:
    """Map a wheel RECORD path into its installed site-packages path."""

    parts = Path(record_path).parts
    data_root = f"{wheel_stem}.data"
    if parts[0] != data_root:
        return record_path
    if len(parts) < 3:
        raise ValueError(f"invalid wheel .data path: {record_path}")
    scheme = parts[1]
    if scheme in {"purelib", "platlib"}:
        relative = Path(*parts[2:]).as_posix()
        if not is_safe_relative_path(relative):
            raise ValueError(f"unsafe wheel site-packages path: {record_path}")
        return relative
    if target_install and scheme == "scripts":
        relative = (Path("bin") / Path(*parts[2:])).as_posix()
        if not is_safe_relative_path(relative):
            raise ValueError(f"unsafe wheel target script path: {record_path}")
        return relative
    if target_install and scheme == "data":
        relative = Path(*parts[2:]).as_posix()
        if not is_safe_relative_path(relative):
            raise ValueError(f"unsafe wheel target data path: {record_path}")
        if Path(relative).parts[0] != "share":
            raise ValueError(
                f"unsupported wheel target data path outside share/: {record_path}"
            )
        return relative
    if target_install and scheme == "headers":
        raise ValueError(f"unsupported wheel target headers path: {record_path}")
    if scheme in {"data", "headers", "scripts"}:
        return None
    raise ValueError(f"unknown wheel .data install scheme: {record_path}")


def _wheel_entry_point_script_paths(content: bytes) -> dict[str, str]:
    """Return safe script paths/specifications from locked entry-point metadata."""

    try:
        text = content.decode("utf-8")
    except UnicodeError as exc:
        raise ValueError(f"entry_points.txt is not UTF-8: {exc}") from exc

    parser = configparser.ConfigParser(
        interpolation=None,
        strict=True,
        delimiters=("=",),
    )
    parser.optionxform = str
    try:
        parser.read_string(text)
    except configparser.Error as exc:
        raise ValueError(f"invalid entry_points.txt: {exc}") from exc
    if parser.defaults():
        raise ValueError("entry_points.txt must not define DEFAULT entries")

    scripts: dict[str, str] = {}
    for section in ("console_scripts", "gui_scripts"):
        if not parser.has_section(section):
            continue
        for name, value in parser.items(section, raw=True):
            specification = value.strip()
            if (
                re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]*", name) is None
                or re.fullmatch(
                    r"[A-Za-z_][A-Za-z0-9_]*"
                    r"(?:\.[A-Za-z_][A-Za-z0-9_]*)*:"
                    r"[A-Za-z_][A-Za-z0-9_]*"
                    r"(?:\.[A-Za-z_][A-Za-z0-9_]*)*",
                    specification,
                )
                is None
            ):
                raise ValueError(f"unsafe {section} entry point: {name!r}")
            relative = (Path("bin") / name).as_posix()
            if (
                not is_safe_relative_path(relative)
                or _is_loadable_site_packages_file(relative)
            ):
                raise ValueError(f"unsafe {section} script path: {relative!r}")
            if relative in scripts:
                raise ValueError(f"duplicate generated entry-point path: {relative}")
            scripts[relative] = specification
    return scripts


def _entry_point_wrapper_bytes(
    specification: str, encoded_python: bytes
) -> bytes:
    """Reproduce pip/distlib's POSIX console-script wrapper exactly."""

    module, callable_path = specification.split(":", 1)
    import_name = callable_path.split(".", 1)[0]
    body = (
        "# -*- coding: utf-8 -*-\n"
        "import re\n"
        "import sys\n"
        f"from {module} import {import_name}\n"
        "if __name__ == '__main__':\n"
        "    sys.argv[0] = re.sub(r'(-script\\.pyw|\\.exe)?$', '', sys.argv[0])\n"
        f"    sys.exit({callable_path}())\n"
    ).encode("utf-8")
    return b"#!" + encoded_python + b"\n" + body


def _site_packages_inventory(
    root: Path,
) -> tuple[set[str], list[str], set[str]]:
    """Enumerate regular files without following links.

    PEP 3147 cache files are isolated by the qualified launcher's fresh
    ``PYTHONPYCACHEPREFIX``. They remain in the inventory so wheel-owned cache
    bytes can be attested and every unowned cache file is rejected.
    """

    try:
        root_mode = root.lstat().st_mode
    except OSError as exc:
        raise ValueError(f"cannot inspect site-packages root {root}: {exc}") from exc
    if stat.S_ISLNK(root_mode) or not stat.S_ISDIR(root_mode):
        raise ValueError(f"site-packages root is not a real directory: {root}")

    regular_files: set[str] = set()
    non_regular: list[str] = []
    cache_files: set[str] = set()
    pending: list[tuple[Path, bool]] = [(root, False)]
    while pending:
        directory, inside_cache = pending.pop()
        try:
            with os.scandir(directory) as scanner:
                entries = sorted(scanner, key=lambda entry: entry.name)
        except OSError as exc:
            relative = directory.relative_to(root).as_posix() or "."
            raise ValueError(
                f"cannot enumerate site-packages directory {relative}: {exc}"
            ) from exc
        for entry in entries:
            entry_path = Path(entry.path)
            relative = entry_path.relative_to(root).as_posix()
            try:
                mode = entry.stat(follow_symlinks=False).st_mode
            except OSError as exc:
                raise ValueError(
                    f"cannot inspect site-packages entry {relative}: {exc}"
                ) from exc
            if stat.S_ISLNK(mode):
                non_regular.append(f"symlink:{relative}")
            elif stat.S_ISDIR(mode):
                pending.append(
                    (entry_path, inside_cache or entry.name == "__pycache__")
                )
            elif stat.S_ISREG(mode):
                if inside_cache:
                    cache_files.add(relative)
                regular_files.add(relative)
            else:
                non_regular.append(f"non-regular:{relative}")
    return regular_files, non_regular, cache_files


def _is_loadable_site_packages_file(relative: str) -> bool:
    name = Path(relative).name.lower()
    return (
        name.endswith((".py", ".pyc", ".pyo", ".pth", ".pyd", ".egg", ".zip"))
        or name.endswith(".egg-link")
        or re.search(r"\.so(?:\.|$)", name) is not None
    )


def verify_python_wheel_records(
    wheelhouse: Path,
    site_packages: Path,
    pinned_requirements: dict[str, tuple[str, str]],
    python_executable: Path,
) -> list[tuple[str, bool, str]]:
    """Attest installed site-packages against RECORDs in locked wheels."""

    check_names = (
        "Python wheel RECORD metadata",
        "native runtime Python installed wheel files",
        "native runtime Python site-packages inventory",
    )
    metadata_errors: list[str] = []
    expected_files: dict[str, tuple[str | None, int | None, str]] = {}
    generated_entry_point_files: dict[str, tuple[str, bytes]] = {}
    wheel_identities: dict[str, tuple[str, str]] = {}
    installer_metadata_roots: set[str] = set()

    if not python_executable.is_absolute():
        detail = f"runtime Python path is not absolute: {python_executable}"
        return [(name, False, detail) for name in check_names]
    encoded_python = os.fsencode(str(python_executable))
    if (
        re.search(rb"\s", encoded_python) is not None
        or b"\x00" in encoded_python
        or len(encoded_python) + 3 > 127
    ):
        detail = f"runtime Python path is unsafe: {python_executable}"
        return [(name, False, detail) for name in check_names]

    try:
        manifest_entries = parse_sha256_manifest(wheelhouse / "SHA256SUMS")
    except (OSError, UnicodeError, ValueError) as exc:
        detail = f"cannot read locked wheel manifest: {exc}"
        return [(name, False, detail) for name in check_names]

    for relative in sorted(manifest_entries):
        wheel = wheelhouse / relative
        try:
            distribution, version, _, _, _ = parse_wheel_filename(relative)
        except ValueError as exc:
            metadata_errors.append(f"{relative}:{exc}")
            continue
        canonical = canonical_distribution_name(distribution)
        if canonical in wheel_identities:
            metadata_errors.append(
                f"duplicate wheel distribution:{canonical}"
            )
            continue
        wheel_identities[canonical] = (distribution, version)

        normalized_distribution = re.sub(r"[-_.]+", "_", distribution)
        wheel_stem = f"{normalized_distribution}-{version}"
        record_path = f"{wheel_stem}.dist-info/RECORD"
        entry_points_path = f"{wheel_stem}.dist-info/entry_points.txt"
        signature_paths = {
            f"{wheel_stem}.dist-info/RECORD.jws",
            f"{wheel_stem}.dist-info/RECORD.p7s",
        }
        try:
            with zipfile.ZipFile(wheel) as archive:
                archive_files: set[str] = set()
                archive_errors: list[str] = []
                for info in archive.infolist():
                    if info.is_dir():
                        continue
                    if not is_safe_relative_path(info.filename):
                        archive_errors.append(f"unsafe archive path:{info.filename!r}")
                        continue
                    if info.filename in archive_files:
                        archive_errors.append(f"duplicate archive path:{info.filename}")
                        continue
                    mode = info.external_attr >> 16
                    if mode and stat.S_ISLNK(mode):
                        archive_errors.append(f"archive symlink:{info.filename}")
                    archive_files.add(info.filename)
                if archive_errors:
                    metadata_errors.extend(
                        f"{relative}:{error}" for error in archive_errors
                    )
                    continue
                top_level_records = sorted(
                    name
                    for name in archive_files
                    if len(Path(name).parts) == 2
                    and name.endswith(".dist-info/RECORD")
                )
                if top_level_records != [record_path]:
                    metadata_errors.append(
                        f"{relative}:expected top-level {record_path}, "
                        f"found={top_level_records}"
                    )
                    continue
                installer_metadata_roots.add(f"{wheel_stem}.dist-info")
                record_bytes = archive.read(record_path)
                entry_points_bytes = (
                    archive.read(entry_points_path)
                    if entry_points_path in archive_files
                    else None
                )
                target_script_bytes = {
                    name: archive.read(name)
                    for name in archive_files
                    if len(Path(name).parts) >= 3
                    and Path(name).parts[0] == f"{wheel_stem}.data"
                    and Path(name).parts[1] == "scripts"
                }
        except (OSError, KeyError, zipfile.BadZipFile, RuntimeError) as exc:
            metadata_errors.append(f"{relative}:cannot read RECORD:{exc}")
            continue

        if entry_points_bytes is not None:
            try:
                generated_scripts = _wheel_entry_point_script_paths(
                    entry_points_bytes
                )
            except ValueError as exc:
                metadata_errors.append(f"{relative}:{exc}")
            else:
                for installed_path, specification in sorted(
                    generated_scripts.items()
                ):
                    previous = generated_entry_point_files.get(installed_path)
                    if previous is not None:
                        metadata_errors.append(
                            f"generated entry-point collision:{installed_path}:"
                            f"{previous[0]},{relative}"
                        )
                    else:
                        generated_entry_point_files[installed_path] = (
                            relative,
                            _entry_point_wrapper_bytes(
                                specification, encoded_python
                            ),
                        )

        try:
            record_text = record_bytes.decode("utf-8")
            rows = list(csv.reader(io.StringIO(record_text), strict=True))
        except (UnicodeError, csv.Error) as exc:
            metadata_errors.append(f"{relative}:invalid RECORD CSV:{exc}")
            continue

        record_archive_paths: set[str] = set()
        saw_record_self = False
        for line_number, row in enumerate(rows, start=1):
            if len(row) != 3:
                metadata_errors.append(
                    f"{relative}:RECORD line {line_number} has {len(row)} fields"
                )
                continue
            source_path, encoded_hash, encoded_size = row
            if not is_safe_relative_path(source_path):
                metadata_errors.append(
                    f"{relative}:unsafe RECORD path:{source_path!r}"
                )
                continue
            if source_path in record_archive_paths:
                metadata_errors.append(
                    f"{relative}:duplicate RECORD path:{source_path}"
                )
                continue
            record_archive_paths.add(source_path)
            if source_path == record_path:
                saw_record_self = True

            try:
                installed_path = _wheel_record_site_path(
                    source_path, wheel_stem, target_install=True
                )
            except ValueError as exc:
                metadata_errors.append(f"{relative}:{exc}")
                continue
            if installed_path is None:
                continue

            expected_hash: str | None
            expected_size: int | None
            if encoded_hash or encoded_size:
                match = re.fullmatch(r"sha256=([A-Za-z0-9_-]{43})", encoded_hash)
                if match is None or re.fullmatch(
                    r"0|[1-9][0-9]*", encoded_size
                ) is None:
                    metadata_errors.append(
                        f"{relative}:RECORD lacks sha256/size:{source_path}"
                    )
                    continue
                expected_hash = match.group(1)
                expected_size = int(encoded_size)
                script_source = target_script_bytes.get(source_path)
                if script_source is not None:
                    source_hash = base64.urlsafe_b64encode(
                        hashlib.sha256(script_source).digest()
                    ).rstrip(b"=").decode("ascii")
                    if (
                        len(script_source) != expected_size
                        or source_hash != expected_hash
                    ):
                        metadata_errors.append(
                            f"{relative}:RECORD does not attest archive script:"
                            f"{source_path}"
                        )
                        continue
                    first_line, separator, remainder = script_source.partition(b"\n")
                    if first_line.startswith(b"#!python"):
                        transformed = (
                            b"#!" + encoded_python + os.linesep.encode("ascii")
                            + (remainder if separator else b"")
                        )
                        expected_hash = base64.urlsafe_b64encode(
                            hashlib.sha256(transformed).digest()
                        ).rstrip(b"=").decode("ascii")
                        expected_size = len(transformed)
            else:
                if source_path != record_path and source_path not in signature_paths:
                    metadata_errors.append(
                        f"{relative}:unhashed RECORD entry:{source_path}"
                    )
                    continue
                expected_hash = None
                expected_size = None

            if installed_path in expected_files:
                metadata_errors.append(
                    f"installed path collision:{installed_path}"
                )
                continue
            expected_files[installed_path] = (
                expected_hash,
                expected_size,
                relative,
            )

        if not saw_record_self:
            metadata_errors.append(f"{relative}:RECORD does not list itself")
        archive_unlisted = sorted(
            archive_files - record_archive_paths - signature_paths
        )
        record_missing = sorted(record_archive_paths - archive_files)
        if archive_unlisted:
            metadata_errors.append(
                f"{relative}:archive files absent from RECORD:"
                + ",".join(archive_unlisted[:4])
            )
        if record_missing:
            metadata_errors.append(
                f"{relative}:RECORD files absent from archive:"
                + ",".join(record_missing[:4])
            )

    for installed_path in sorted(
        set(expected_files) & set(generated_entry_point_files)
    ):
        metadata_errors.append(
            f"generated entry point collides with RECORD file:{installed_path}"
        )
    for installed_path, (wheel_name, content) in sorted(
        generated_entry_point_files.items()
    ):
        if installed_path in expected_files:
            continue
        expected_files[installed_path] = (
            base64.urlsafe_b64encode(hashlib.sha256(content).digest())
            .rstrip(b"=")
            .decode("ascii"),
            len(content),
            f"{wheel_name}:generated-entry-point",
        )

    for canonical, (distribution, expected_version) in pinned_requirements.items():
        wheel_identity = wheel_identities.get(canonical)
        if wheel_identity is None:
            metadata_errors.append(f"missing pinned wheel:{distribution}")
        elif wheel_identity[1] != expected_version:
            metadata_errors.append(
                f"pinned wheel mismatch:{distribution}:"
                f"expected={expected_version},actual={wheel_identity[1]}"
            )

    metadata_ok = not metadata_errors
    metadata_detail = (
        f"wheels={len(wheel_identities)} files={len(expected_files)}"
        if metadata_ok
        else _compact_failures(metadata_errors)
    )
    if not metadata_ok:
        return [
            (check_names[0], False, metadata_detail),
            (check_names[1], False, "RECORD metadata is invalid"),
            (check_names[2], False, "RECORD metadata is invalid"),
        ]

    try:
        actual_files, non_regular, _cache_files = _site_packages_inventory(
            site_packages
        )
    except ValueError as exc:
        return [
            (check_names[0], True, metadata_detail),
            (check_names[1], False, str(exc)),
            (check_names[2], False, str(exc)),
        ]

    missing = sorted(set(expected_files) - actual_files)
    mismatches: list[str] = []
    verified_hashed_files = 0
    for relative in sorted(set(expected_files) & actual_files):
        expected_hash, expected_size, wheel_name = expected_files[relative]
        if expected_hash is None or expected_size is None:
            continue
        path = site_packages / relative
        try:
            actual_size = path.stat().st_size
            if actual_size != expected_size:
                mismatches.append(
                    f"{relative}:size={actual_size},expected={expected_size}"
                )
                continue
            actual_hash = base64.urlsafe_b64encode(
                bytes.fromhex(sha256_file(path))
            ).rstrip(b"=").decode("ascii")
        except OSError as exc:
            mismatches.append(f"{relative}:{wheel_name}:{exc}")
            continue
        if actual_hash != expected_hash:
            mismatches.append(f"{relative}:sha256 mismatch")
        else:
            verified_hashed_files += 1

    installed_errors = [f"missing:{value}" for value in missing]
    installed_errors.extend(mismatches)
    installed_ok = not installed_errors
    installed_detail = (
        f"verified={verified_hashed_files} present={len(expected_files)}"
        if installed_ok
        else _compact_failures(installed_errors)
    )

    generated_metadata_names = {"INSTALLER", "REQUESTED", "direct_url.json"}
    unexpected_files = actual_files - set(expected_files)
    allowed_generated_metadata = {
        relative
        for relative in unexpected_files
        if len(Path(relative).parts) == 2
        and Path(relative).parts[0] in installer_metadata_roots
        and Path(relative).name in generated_metadata_names
    }
    unowned_loadable = sorted(
        relative
        for relative in (
            unexpected_files
            - allowed_generated_metadata
        )
        if _is_loadable_site_packages_file(relative)
    )
    unowned_other = sorted(
        unexpected_files
        - allowed_generated_metadata
        - set(unowned_loadable)
    )
    inventory_errors = list(non_regular)
    inventory_errors.extend(
        f"unowned-loadable:{relative}" for relative in unowned_loadable
    )
    inventory_errors.extend(f"unowned-file:{relative}" for relative in unowned_other)
    inventory_ok = not inventory_errors
    inventory_detail = (
        f"exact content closure; installer-metadata={len(allowed_generated_metadata)}; "
        f"entry-point-wrappers={len(generated_entry_point_files)}; "
        "unowned-pycache-files=0"
        if inventory_ok
        else _compact_failures(inventory_errors)
    )
    return [
        (check_names[0], True, metadata_detail),
        (check_names[1], installed_ok, installed_detail),
        (check_names[2], inventory_ok, inventory_detail),
    ]


def verify_pico_wheel(
    wheel: Path, pico_lock: dict[str, Any]
) -> tuple[bool, str]:
    if not wheel.is_file() or wheel.is_symlink():
        return False, f"missing regular file {wheel}"
    details: list[str] = []
    if wheel.name != pico_lock["wheel_filename"]:
        details.append(
            f"filename expected={pico_lock['wheel_filename']} actual={wheel.name}"
        )
    try:
        actual_sha256 = sha256_file(wheel)
    except OSError as exc:
        return False, f"cannot hash PICO wheel {wheel}: {exc}"
    if actual_sha256 != pico_lock["wheel_sha256"]:
        details.append(
            f"sha256 expected={pico_lock['wheel_sha256']} actual={actual_sha256}"
        )
    try:
        _, _, python_tags, abi_tags, platform_tags = parse_wheel_filename(wheel.name)
        compatible = wheel_tags_compatible(
            python_tags, abi_tags, platform_tags
        )
    except ValueError as exc:
        compatible = False
        details.append(str(exc))
    if not compatible and not any("wheel" in detail for detail in details):
        details.append("incompatible with CPython 3.10 x86_64/glibc <=2.35")
    return not details, "; ".join(details) or f"sha256={actual_sha256}"


def verify_installed_pico_wheel(
    wheel: Path,
    site_packages: Path,
    pico_lock: dict[str, Any],
) -> tuple[bool, str]:
    """Bind the installed PICO SDK package bytes to its locked wheel RECORD.

    PICO remains an externally controlled environment, so this check does not
    claim ownership of unrelated dependency distributions. It does reject
    extra importable files in the SDK namespace and global startup hooks that
    could replace the locked extension module.
    """

    wheel_ok, wheel_detail = verify_pico_wheel(wheel, pico_lock)
    if not wheel_ok:
        return False, wheel_detail
    try:
        distribution, version, _, _, _ = parse_wheel_filename(wheel.name)
    except ValueError as exc:
        return False, str(exc)
    normalized_distribution = re.sub(r"[-_.]+", "_", distribution)
    wheel_stem = f"{normalized_distribution}-{version}"
    record_path = f"{wheel_stem}.dist-info/RECORD"
    top_level_path = f"{wheel_stem}.dist-info/top_level.txt"
    signature_paths = {
        f"{wheel_stem}.dist-info/RECORD.jws",
        f"{wheel_stem}.dist-info/RECORD.p7s",
    }
    errors: list[str] = []
    expected_files: dict[str, tuple[str | None, int | None]] = {}
    try:
        with zipfile.ZipFile(wheel) as archive:
            archive_files: set[str] = set()
            for info in archive.infolist():
                if info.is_dir():
                    continue
                if not is_safe_relative_path(info.filename):
                    errors.append(f"unsafe archive path:{info.filename!r}")
                    continue
                if info.filename in archive_files:
                    errors.append(f"duplicate archive path:{info.filename}")
                    continue
                mode = info.external_attr >> 16
                if mode and stat.S_ISLNK(mode):
                    errors.append(f"archive symlink:{info.filename}")
                archive_files.add(info.filename)
            top_records = sorted(
                name
                for name in archive_files
                if len(Path(name).parts) == 2
                and name.endswith(".dist-info/RECORD")
            )
            if top_records != [record_path]:
                errors.append(
                    f"expected top-level {record_path}, found={top_records}"
                )
                record_bytes = b""
            else:
                record_bytes = archive.read(record_path)
            if top_level_path not in archive_files:
                errors.append(f"wheel lacks {top_level_path}")
                top_level_bytes = b""
            else:
                top_level_bytes = archive.read(top_level_path)
    except (OSError, KeyError, zipfile.BadZipFile, RuntimeError) as exc:
        return False, f"cannot read locked PICO wheel RECORD: {exc}"
    if errors:
        return False, _compact_failures(errors)

    try:
        rows = list(
            csv.reader(
                io.StringIO(record_bytes.decode("utf-8")), strict=True
            )
        )
        top_level_names = {
            line.strip()
            for line in top_level_bytes.decode("utf-8").splitlines()
            if line.strip()
        }
    except (UnicodeError, csv.Error) as exc:
        return False, f"invalid PICO wheel metadata: {exc}"
    if not top_level_names or any(
        re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", value) is None
        for value in top_level_names
    ):
        return False, f"invalid PICO top_level.txt: {sorted(top_level_names)}"

    record_archive_paths: set[str] = set()
    saw_record_self = False
    for line_number, row in enumerate(rows, start=1):
        if len(row) != 3:
            errors.append(f"RECORD line {line_number} has {len(row)} fields")
            continue
        source_path, encoded_hash, encoded_size = row
        if not is_safe_relative_path(source_path):
            errors.append(f"unsafe RECORD path:{source_path!r}")
            continue
        if source_path in record_archive_paths:
            errors.append(f"duplicate RECORD path:{source_path}")
            continue
        record_archive_paths.add(source_path)
        saw_record_self = saw_record_self or source_path == record_path
        try:
            installed_path = _wheel_record_site_path(source_path, wheel_stem)
        except ValueError as exc:
            errors.append(str(exc))
            continue
        if installed_path is None:
            continue
        if encoded_hash or encoded_size:
            match = re.fullmatch(r"sha256=([A-Za-z0-9_-]{43})", encoded_hash)
            if match is None or re.fullmatch(r"0|[1-9][0-9]*", encoded_size) is None:
                errors.append(f"RECORD lacks sha256/size:{source_path}")
                continue
            expected_hash: str | None = match.group(1)
            expected_size: int | None = int(encoded_size)
        else:
            if source_path != record_path and source_path not in signature_paths:
                errors.append(f"unhashed RECORD entry:{source_path}")
                continue
            expected_hash = None
            expected_size = None
        if installed_path in expected_files:
            errors.append(f"installed path collision:{installed_path}")
            continue
        expected_files[installed_path] = (expected_hash, expected_size)
    if not saw_record_self:
        errors.append("RECORD does not list itself")
    archive_unlisted = sorted(archive_files - record_archive_paths - signature_paths)
    record_missing = sorted(record_archive_paths - archive_files)
    if archive_unlisted:
        errors.append("archive files absent from RECORD:" + ",".join(archive_unlisted))
    if record_missing:
        errors.append("RECORD files absent from archive:" + ",".join(record_missing))
    if errors:
        return False, _compact_failures(errors)

    try:
        actual_files, non_regular, _cache_files = _site_packages_inventory(
            site_packages
        )
    except ValueError as exc:
        return False, str(exc)
    missing = sorted(set(expected_files) - actual_files)
    for relative in missing:
        errors.append(f"missing:{relative}")
    verified = 0
    for relative in sorted(set(expected_files) & actual_files):
        expected_hash, expected_size = expected_files[relative]
        if expected_hash is None or expected_size is None:
            continue
        path = site_packages / relative
        try:
            actual_size = path.stat().st_size
        except OSError as exc:
            errors.append(f"{relative}:{exc}")
            continue
        if actual_size != expected_size:
            errors.append(
                f"{relative}:size={actual_size},expected={expected_size}"
            )
            continue
        try:
            actual_hash = base64.urlsafe_b64encode(
                bytes.fromhex(sha256_file(path))
            ).rstrip(b"=").decode("ascii")
        except OSError as exc:
            errors.append(f"{relative}:{exc}")
            continue
        if actual_hash != expected_hash:
            errors.append(f"{relative}:sha256 mismatch")
        else:
            verified += 1

    def sdk_related(relative: str) -> bool:
        parts = Path(relative).parts
        if not parts:
            return False
        if parts[0] in top_level_names:
            return True
        if len(parts) != 1:
            return False
        name = parts[0]
        return any(
            name == top_level
            or name.startswith(f"{top_level}.")
            for top_level in top_level_names
        ) and _is_loadable_site_packages_file(relative)

    generated_metadata_names = {"INSTALLER", "REQUESTED", "direct_url.json"}

    def global_startup_hook(relative: str) -> bool:
        parts = Path(relative).parts
        if len(parts) != 1:
            return False
        name = parts[0].lower()
        return name.endswith(".pth") or (
            (name == "sitecustomize.py" or name.startswith("sitecustomize."))
            or (name == "usercustomize.py" or name.startswith("usercustomize."))
        ) and _is_loadable_site_packages_file(relative)

    unexpected = actual_files - set(expected_files)
    for relative in sorted(unexpected):
        parts = Path(relative).parts
        generated_metadata = (
            len(parts) == 2
            and parts[0] == f"{wheel_stem}.dist-info"
            and parts[1] in generated_metadata_names
        )
        if not generated_metadata and (
            sdk_related(relative) or global_startup_hook(relative)
        ):
            errors.append(f"unowned PICO import file:{relative}")
    for value in non_regular:
        relative = value.split(":", 1)[-1]
        if sdk_related(relative) or global_startup_hook(relative):
            errors.append(value)
    return (
        not errors,
        (
            f"verified={verified} files={len(expected_files)} "
            f"top-level={','.join(sorted(top_level_names))} "
            "unowned-sdk-pycache-files=0"
            if not errors
            else _compact_failures(errors)
        ),
    )


def runtime_roots(runtime_root: Path, sonic_root: Path) -> dict[str, Path]:
    return {
        "sonic": sonic_root,
        "visual": runtime_root / "g1-visual",
        "inference": runtime_root / "inference",
        "native": runtime_root / "matrix-native-deps",
    }


def runtime_tree_attestation(
    base: Path, relative: str, expected_sha256: str
) -> tuple[bool, str]:
    if not is_safe_relative_path(relative):
        return False, f"unsafe relative path: {relative!r}"
    path = base / relative
    try:
        inside_root = path.resolve().is_relative_to(base.resolve())
    except (OSError, RuntimeError):
        inside_root = False
    if not inside_root:
        return False, f"tree escapes runtime root: {path}"

    current = base
    for part in Path(relative).parts:
        current /= part
        try:
            mode = current.lstat().st_mode
        except OSError:
            break
        if stat.S_ISLNK(mode):
            return False, f"tree path contains symlink: {current}"

    try:
        actual_sha256, file_count = sha256_tree(path)
    except ValueError as exc:
        return False, str(exc)
    return (
        actual_sha256 == expected_sha256,
        f"files={file_count} sha256={actual_sha256}",
    )


def library_paths(runtime_root: Path, matrix_root: Path, sonic_root: Path) -> list[Path]:
    native = Path(
        os.environ.get(
            "MATRIX_NATIVE_DEPS_ROOT", runtime_root / "matrix-native-deps"
        )
    )
    ros = Path(
        os.environ.get("MATRIX_ROS_PREFIX", runtime_root / "ros2-humble-prefix")
    )
    cuda = Path(os.environ.get("MATRIX_CUDA_ROOT", "/usr/local/cuda"))
    candidates = [
        runtime_root / "inference/TensorRT/lib",
        runtime_root / "inference/onnxruntime/lib",
        sonic_root / "gear_sonic_deploy/thirdparty/unitree_sdk2/thirdparty/lib/x86_64",
        sonic_root
        / "external_dependencies/XRoboToolkit-PC-Service-Pybind_X86_and_ARM64/lib",
        native / "usr/lib",
        native / "usr/lib/x86_64-linux-gnu",
        ros / "lib",
        matrix_root / "src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux",
        matrix_root / "src/UeSim/Linux/Engine/Binaries/Linux",
        cuda / "lib64",
        cuda / "lib",
    ]
    return [path for path in candidates if path.is_dir()]


def dynamic_environment(
    runtime_root: Path, matrix_root: Path, sonic_root: Path
) -> dict[str, str]:
    env = os.environ.copy()
    paths = [
        str(path) for path in library_paths(runtime_root, matrix_root, sonic_root)
    ]
    if env.get("LD_LIBRARY_PATH"):
        paths.append(env["LD_LIBRARY_PATH"])
    env["LD_LIBRARY_PATH"] = ":".join(paths)
    return env


def pip_check_environment(
    python_executable: str, base_env: dict[str, str]
) -> tuple[dict[str, str], str | None]:
    """Expose an attested host pip only to ``python -m pip check``.

    Bootstrap keeps pip in a dedicated module root next to the venv (moving an
    ensurepip seed there or linking a host fallback). Runtime imports remain
    isolated; only installer/dependency-consistency subprocesses receive this
    extra path.
    """

    env = base_env.copy()
    marker = Path(python_executable).expanduser().parent.parent / ".matrix-external-pip"
    if not marker.exists():
        return env, None
    if marker.is_symlink() or not marker.is_file():
        return env, f"invalid external pip marker: {marker}"
    try:
        lines = marker.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        return env, f"cannot read external pip marker: {exc}"
    if len(lines) != 1 or not lines[0]:
        return env, f"external pip marker must contain one path: {marker}"
    root = Path(lines[0])
    venv_root = marker.parent
    if root != venv_root / ".matrix-pip-runner" or not (
        root / "pip/__init__.py"
    ).is_file():
        return env, f"external pip module root is invalid: {root}"
    site_packages = sorted((venv_root / "lib").glob("python*/site-packages"))
    if len(site_packages) != 1 or not site_packages[0].is_dir():
        return env, f"external pip venv site-packages is invalid: {venv_root}"
    env["PYTHONPATH"] = os.pathsep.join((str(site_packages[0]), str(root))) + (
        os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else ""
    )
    return env, None


def python_identity_probe_code(marker: str) -> str:
    """Build the shared runtime/PICO interpreter identity probe."""

    if re.fullmatch(r"[A-Z0-9_]+=", marker) is None:
        raise ValueError(f"invalid Python identity marker: {marker!r}")
    return (
        "import json,platform,site,sys,sysconfig; "
        f"print({marker!r} + json.dumps({{'version': "
        "f'{sys.version_info.major}.{sys.version_info.minor}', "
        "'soabi': sysconfig.get_config_var('SOABI') or '', "
        "'machine': platform.machine(), 'prefix': sys.prefix, "
        "'base_prefix': sys.base_prefix, 'executable': sys.executable, "
        "'path': sys.path, "
        "'purelib': sysconfig.get_path('purelib'), "
        "'platlib': sysconfig.get_path('platlib'), "
        "'stdlib': sysconfig.get_path('stdlib'), "
        "'platstdlib': sysconfig.get_path('platstdlib'), "
        "'user_site_enabled': site.ENABLE_USER_SITE}))"
    )


def verify_python_isolation(
    venv_root: Path,
    site_packages: Path,
    matrix_root: Path,
    identity: dict[str, Any],
) -> tuple[bool, str]:
    """Prove the runtime venv cannot import unverified host/user packages."""

    failures: list[str] = []
    try:
        root_mode = venv_root.lstat().st_mode
    except OSError as exc:
        return False, f"cannot inspect venv root {venv_root}: {exc}"
    if stat.S_ISLNK(root_mode) or not stat.S_ISDIR(root_mode):
        failures.append("venv root is not a real directory")

    configuration = venv_root / "pyvenv.cfg"
    include_system_values: list[str] = []
    try:
        configuration_mode = configuration.lstat().st_mode
        if stat.S_ISLNK(configuration_mode) or not stat.S_ISREG(configuration_mode):
            raise ValueError("pyvenv.cfg is not a regular non-symlink file")
        for raw_line in configuration.read_text(encoding="utf-8").splitlines():
            if "=" not in raw_line:
                continue
            key, value = raw_line.split("=", 1)
            if key.strip().lower() == "include-system-site-packages":
                include_system_values.append(value.strip().lower())
    except (OSError, UnicodeError, ValueError) as exc:
        failures.append(str(exc))
    if include_system_values != ["false"]:
        failures.append(
            "pyvenv.cfg include-system-site-packages must occur once as false: "
            f"actual={include_system_values}"
        )

    expected_site = site_packages.absolute()
    expected_matrix = matrix_root.absolute()
    prefix = identity.get("prefix")
    base_prefix = identity.get("base_prefix")
    executable = identity.get("executable")
    purelib = identity.get("purelib")
    platlib = identity.get("platlib")
    stdlib = identity.get("stdlib")
    platstdlib = identity.get("platstdlib")
    paths = identity.get("path")
    if not isinstance(prefix, str) or Path(prefix).absolute() != venv_root.absolute():
        failures.append(f"runtime prefix escapes venv: {prefix!r}")
    if not isinstance(base_prefix, str) or not Path(base_prefix).is_absolute():
        failures.append(f"invalid base_prefix: {base_prefix!r}")
    elif Path(base_prefix).absolute() == venv_root.absolute():
        failures.append("runtime Python is not a venv")
    expected_executable = venv_root.absolute() / "bin/python"
    if (
        not isinstance(executable, str)
        or Path(executable).absolute() != expected_executable
    ):
        failures.append(
            f"runtime executable escapes venv: {executable!r}"
        )
    for label, value in (("purelib", purelib), ("platlib", platlib)):
        if not isinstance(value, str) or Path(value).absolute() != expected_site:
            failures.append(f"runtime {label} escapes locked site-packages: {value!r}")
    if identity.get("user_site_enabled") is not False:
        failures.append(
            f"user site is enabled: {identity.get('user_site_enabled')!r}"
        )
    if not isinstance(paths, list) or not all(isinstance(value, str) for value in paths):
        failures.append("runtime sys.path is not a string list")
        paths = []

    standard_roots: set[Path] = set()
    standard_zip_paths: set[Path] = set()
    for value in (stdlib, platstdlib):
        if not isinstance(value, str) or not Path(value).is_absolute():
            failures.append(f"invalid standard-library root: {value!r}")
            continue
        standard_root = Path(value).absolute()
        standard_roots.add(standard_root)
        version = identity.get("version")
        if isinstance(version, str):
            major_minor = version.replace(".", "")
            standard_zip_paths.add(
                standard_root.parent / f"python{major_minor}.zip"
            )

    saw_site_packages = False
    unauthorized_paths: list[str] = []
    for raw_path in paths:
        candidate = (
            expected_matrix if raw_path == "" else Path(raw_path).absolute()
        )
        if candidate == expected_site or candidate.is_relative_to(expected_site):
            saw_site_packages = saw_site_packages or candidate == expected_site
            continue
        if candidate == expected_matrix:
            continue
        lowered_parts = {part.lower() for part in candidate.parts}
        if "site-packages" in lowered_parts or "dist-packages" in lowered_parts:
            unauthorized_paths.append(str(candidate))
            continue
        if candidate in standard_zip_paths or any(
            candidate == root or candidate.is_relative_to(root)
            for root in standard_roots
        ):
            continue
        unauthorized_paths.append(str(candidate))
    if not saw_site_packages:
        failures.append(f"locked site-packages is absent from sys.path: {expected_site}")
    if unauthorized_paths:
        failures.append(
            "unauthorized sys.path entries: " + ",".join(unauthorized_paths)
        )
    return (
        not failures,
        "isolated venv and sys.path" if not failures else _compact_failures(failures),
    )


def git_checkout_root(path: Path) -> Path | None:
    try:
        result = subprocess.run(
            ["git", "-C", str(path), "rev-parse", "--show-toplevel"],
            text=True,
            capture_output=True,
            check=False,
        )
    except OSError:
        return None
    if result.returncode != 0:
        return None
    try:
        root = Path(result.stdout.strip()).resolve()
    except (OSError, RuntimeError):
        return None
    return root if root == path.resolve() else None


def archived_source_attestation(
    lock: dict[str, Any], sonic_root: Path
) -> tuple[bool, str]:
    sonic_lock = lock["source_revisions"]["gr00t_whole_body_control"]
    runtime_files = {
        (entry["root"], entry["path"]): entry["sha256"]
        for entry in lock["runtime_files"]
    }
    failures: list[str] = []
    for relative in sonic_lock["critical_source_paths"]:
        expected = runtime_files.get(("sonic", relative))
        if expected is None:
            failures.append(f"not locked: {relative}")
            continue
        path = sonic_root / relative
        try:
            inside_root = path.resolve().is_relative_to(sonic_root.resolve())
        except (OSError, RuntimeError):
            inside_root = False
        if not inside_root or not path.is_file() or path.is_symlink():
            failures.append(f"missing regular file: {relative}")
            continue
        actual = sha256_file(path)
        if actual != expected:
            failures.append(f"SHA256 mismatch: {relative}")
    return (
        not failures,
        "; ".join(failures) if failures else "critical source files match runtime lock",
    )


def matrix_source_overlay_attestation(
    matrix_root: Path, lock: dict[str, Any]
) -> tuple[bool, str]:
    """Reject uncommitted Matrix code that can shadow qualified runtime inputs."""

    inventories: list[tuple[str, list[str]]] = []
    for label, extra_arguments in (
        ("untracked", ["--exclude-standard"]),
        ("ignored", ["--ignored", "--exclude-standard"]),
    ):
        result = subprocess.run(
            [
                "git",
                "-C",
                str(matrix_root),
                "ls-files",
                "--others",
                *extra_arguments,
                "-z",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        if result.returncode != 0:
            detail = result.stderr.decode("utf-8", errors="replace").strip()
            return False, f"git {label} inventory failed: {detail or result.returncode}"
        inventories.append(
            (
                label,
                [
                    value.decode("utf-8", errors="surrogateescape")
                    for value in result.stdout.split(b"\0")
                    if value
                ],
            )
        )

    installed_files = {
        Path(entry["path"]) for entry in lock["matrix_release"]["installed_files"]
    }
    installed_trees = [
        Path(entry["path"]) for entry in lock["matrix_release"]["installed_trees"]
    ]
    excluded_roots = {".matrix", ".venv-audit", "outputs", "releases"}
    disabled_mc_build = Path("src/robot_mc/build")

    def locked_shared_object(candidate: Path) -> bool:
        return candidate in installed_files or any(
            candidate.is_relative_to(tree) for tree in installed_trees
        )

    failures: list[str] = []
    for label, paths in inventories:
        for value in paths:
            candidate = Path(value)
            if not is_safe_relative_path(value) or not candidate.parts:
                failures.append(f"{label}:unsafe-path:{value!r}")
                continue
            if (
                candidate.parts[0] in excluded_roots
                or candidate.parts[0].startswith(".chunk_downloads_")
                or candidate.is_relative_to(disabled_mc_build)
            ):
                continue
            suffix = candidate.suffix.lower()
            code_overlay = suffix in {".py", ".pyi"}
            legacy_bytecode = suffix == ".pyc" and "__pycache__" not in candidate.parts
            shared_object = re.search(r"\.so(?:\.|$)", candidate.name) is not None
            if code_overlay or legacy_bytecode or (
                shared_object and not locked_shared_object(candidate)
            ):
                failures.append(f"{label}:{value}")
    return (
        not failures,
        "none" if not failures else _compact_failures(sorted(set(failures))),
    )


def check_tensorrt_version(
    library: Path, expected: int, env: dict[str, str], python_executable: str
) -> tuple[bool, str]:
    code = (
        "import ctypes,sys; "
        "lib=ctypes.CDLL(sys.argv[1], mode=ctypes.RTLD_GLOBAL); "
        "fn=lib.getInferLibVersion; fn.restype=ctypes.c_int; print(fn())"
    )
    result = subprocess.run(
        [python_executable, "-c", code, str(library)],
        env=env,
        text=True,
        capture_output=True,
        check=False,
    )
    if result.returncode != 0:
        return False, result.stderr.strip() or "TensorRT library load failed"
    try:
        actual = int(result.stdout.strip())
    except ValueError:
        return False, f"invalid TensorRT version output: {result.stdout!r}"
    return actual == expected, f"expected={expected} actual={actual}"


def check_dlopen(
    path: Path, env: dict[str, str], python_executable: str
) -> tuple[bool, str]:
    code = "import ctypes,sys; ctypes.CDLL(sys.argv[1], mode=ctypes.RTLD_GLOBAL)"
    result = subprocess.run(
        [python_executable, "-c", code, str(path)],
        env=env,
        text=True,
        capture_output=True,
        check=False,
    )
    return result.returncode == 0, result.stderr.strip() or "dlopen succeeded"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lock", type=Path, default=DEFAULT_LOCK)
    parser.add_argument("--runtime-root", type=Path, default=DEFAULT_RUNTIME)
    parser.add_argument("--matrix-root", type=Path, default=REPO_ROOT)
    parser.add_argument("--sonic-root", type=Path)
    parser.add_argument(
        "--python",
        help=(
            "Actual locked interpreter used by the native runtime; required for "
            "all checks except --schema-only"
        ),
    )
    parser.add_argument(
        "--pico-python",
        help="Optional separate interpreter for validating the native PICO closure",
    )
    parser.add_argument(
        "--pico-wheel",
        type=Path,
        help="Locked xrobotoolkit_sdk wheel installed into --pico-python",
    )
    parser.add_argument("--release-cache", type=Path)
    parser.add_argument("--profile", choices=HOST_PROFILES)
    parser.add_argument(
        "--require-git-sonic",
        action="store_true",
        help="Reject an archived SONIC mirror; required by bounded qualification",
    )
    parser.add_argument("--schema-only", action="store_true")
    parser.add_argument("--skip-dynamic", action="store_true")
    parser.add_argument("--skip-installed-assets", action="store_true")
    parser.add_argument(
        "--fast",
        action="store_true",
        help="Hash small critical files and check large files by presence only",
    )
    parser.add_argument("--json-output", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        lock = load_lock(args.lock.resolve())
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        print(f"[FAIL] runtime lock: {exc}", file=sys.stderr)
        return 2

    try:
        validate_policy_manifest_files(lock, args.matrix_root)
    except (OSError, ValueError) as exc:
        print(f"[FAIL] policy slot manifest: {exc}", file=sys.stderr)
        return 2

    if args.schema_only:
        print(f"[PASS] runtime lock schema: {args.lock}")
        return 0

    runtime_root = args.runtime_root.resolve()
    matrix_root = args.matrix_root.resolve()
    sonic_root = (
        args.sonic_root.resolve()
        if args.sonic_root
        else runtime_root / "GR00T-WholeBodyControl"
    )
    roots = runtime_roots(runtime_root, sonic_root)
    checks: list[dict[str, Any]] = []

    def record(name: str, ok: bool, detail: str) -> None:
        checks.append({"name": name, "ok": ok, "detail": detail})
        print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")

    matrix_commit_result = subprocess.run(
        ["git", "-C", str(matrix_root), "rev-parse", "HEAD"],
        text=True,
        capture_output=True,
        check=False,
    )
    matrix_commit = matrix_commit_result.stdout.strip()
    record(
        "Matrix source commit",
        matrix_commit_result.returncode == 0
        and re.fullmatch(r"[0-9a-f]{40}", matrix_commit) is not None,
        matrix_commit or matrix_commit_result.stderr.strip() or "unavailable",
    )
    matrix_clean_result = subprocess.run(
        ["git", "-C", str(matrix_root), "diff", "--quiet", "HEAD", "--"],
        text=True,
        capture_output=True,
        check=False,
    )
    record(
        "Matrix tracked source clean",
        matrix_clean_result.returncode == 0,
        "clean" if matrix_clean_result.returncode == 0 else "tracked changes present",
    )
    matrix_overlays_ok, matrix_overlays_detail = matrix_source_overlay_attestation(
        matrix_root, lock
    )
    record(
        "Matrix ignored source overlays absent",
        matrix_overlays_ok,
        matrix_overlays_detail,
    )

    python_executable: str | None = None
    if args.python:
        python_executable = shutil.which(args.python)
        if python_executable is None:
            candidate = Path(args.python).expanduser()
            if candidate.is_file() and os.access(candidate, os.X_OK):
                python_executable = str(candidate.resolve())
    record(
        "native runtime Python",
        python_executable is not None,
        python_executable or "--python was not an executable explicit runtime path",
    )

    requirements_path = (matrix_root / lock["python"]["requirements"]).resolve()
    pinned_requirements: dict[str, tuple[str, str]] = {}
    try:
        requirements_in_project = requirements_path.is_relative_to(matrix_root)
    except (OSError, RuntimeError):
        requirements_in_project = False
    if not requirements_in_project or not requirements_path.is_file():
        record(
            "Python requirements lock",
            False,
            f"missing safe project file: {requirements_path}",
        )
    else:
        actual_requirements_sha256 = sha256_file(requirements_path)
        record(
            "Python requirements lock",
            actual_requirements_sha256 == lock["python"]["requirements_sha256"],
            f"sha256={actual_requirements_sha256}",
        )
        try:
            pinned_requirements = parse_pinned_requirements(requirements_path)
        except (OSError, UnicodeError, ValueError) as exc:
            record("Python requirements exact pins", False, str(exc))
        else:
            record(
                "Python requirements exact pins",
                True,
                f"distributions={len(pinned_requirements)}",
            )
            scalar_mismatches = []
            for distribution in ("mujoco",):
                expected = str(lock["python"].get(distribution, ""))
                pin = pinned_requirements.get(distribution)
                actual = pin[1] if pin is not None else ""
                if not expected or actual != expected:
                    scalar_mismatches.append(
                        f"{distribution}:lock={expected or '<missing>'},"
                        f"requirements={actual or '<missing>'}"
                    )
            record(
                "Python lock scalar consistency",
                not scalar_mismatches,
                "; ".join(scalar_mismatches) or "consistent",
            )

    identity_env = os.environ.copy()
    identity_env["PYTHONNOUSERSITE"] = "1"
    identity_env["PYTHONDONTWRITEBYTECODE"] = "1"
    python_identity: dict[str, Any] = {}
    identity_error = "runtime Python is unavailable"
    if python_executable is not None:
        identity_prefix = "MATRIX_VERIFY_PYTHON_JSON="
        identity_code = python_identity_probe_code(identity_prefix)
        identity_result = subprocess.run(
            [python_executable, "-c", identity_code],
            env=identity_env,
            cwd=matrix_root,
            text=True,
            capture_output=True,
            check=False,
        )
        identity_error = identity_result.stderr.strip() or "invalid identity output"
        if identity_result.returncode == 0:
            identity_line = next(
                (
                    line[len(identity_prefix) :]
                    for line in reversed(identity_result.stdout.splitlines())
                    if line.startswith(identity_prefix)
                ),
                "",
            )
            try:
                candidate_identity = json.loads(identity_line)
                if isinstance(candidate_identity, dict):
                    python_identity = candidate_identity
            except json.JSONDecodeError:
                identity_error = f"invalid JSON: {identity_result.stdout!r}"
    for key, label in (
        ("version", "native runtime Python version"),
        ("soabi", "native runtime Python SOABI"),
        ("machine", "native runtime machine"),
    ):
        candidate_identity = python_identity.get(key, "")
        actual_identity = candidate_identity if isinstance(candidate_identity, str) else ""
        expected_identity = lock["python"][key]
        record(
            label,
            actual_identity == expected_identity,
            (
                f"expected={expected_identity} actual={actual_identity}"
                if actual_identity
                else identity_error
            ),
        )
    expected_python_prefix = (
        Path(python_executable).absolute().parent.parent
        if python_executable is not None
        else Path()
    )
    python_site_packages = (
        expected_python_prefix
        / "lib"
        / f"python{lock['python']['version']}"
        / "site-packages"
        if python_executable is not None
        else None
    )
    identity_prefix_value = python_identity.get("prefix", "")
    actual_python_prefix = Path(
        identity_prefix_value if isinstance(identity_prefix_value, str) else ""
    ).absolute()
    record(
        "native runtime Python prefix",
        python_executable is not None
        and isinstance(identity_prefix_value, str)
        and identity_prefix_value != ""
        and actual_python_prefix == expected_python_prefix,
        f"expected={expected_python_prefix} actual={actual_python_prefix}",
    )
    if python_site_packages is None:
        isolation_ok, isolation_detail = False, "runtime Python is unavailable"
    else:
        isolation_ok, isolation_detail = verify_python_isolation(
            expected_python_prefix,
            python_site_packages,
            matrix_root,
            python_identity,
        )
    record("native runtime Python isolation", isolation_ok, isolation_detail)

    if python_executable is not None and pinned_requirements:
        metadata_prefix = "MATRIX_VERIFY_DISTRIBUTIONS_JSON="
        metadata_code = r"""
import importlib.metadata
import json
import sys

names = json.loads(sys.argv[1])
versions = {}
for name in names:
    try:
        versions[name] = importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError:
        versions[name] = None
print("MATRIX_VERIFY_DISTRIBUTIONS_JSON=" + json.dumps(versions, sort_keys=True))
"""
        result = subprocess.run(
            [
                python_executable,
                "-c",
                metadata_code,
                json.dumps(sorted(pinned_requirements)),
            ],
            env=identity_env,
            text=True,
            capture_output=True,
            check=False,
        )
        metadata_line = next(
            (
                line[len(metadata_prefix) :]
                for line in reversed(result.stdout.splitlines())
                if line.startswith(metadata_prefix)
            ),
            "",
        )
        try:
            installed_versions = json.loads(metadata_line) if metadata_line else {}
        except json.JSONDecodeError:
            installed_versions = {}
        version_mismatches = []
        for canonical, (distribution, expected_version) in pinned_requirements.items():
            actual_version = installed_versions.get(canonical)
            if actual_version != expected_version:
                version_mismatches.append(
                    f"{distribution}:expected={expected_version},"
                    f"actual={actual_version or '<missing>'}"
                )
        record(
            "native runtime Python distributions",
            result.returncode == 0 and not version_mismatches,
            (
                "; ".join(version_mismatches)
                if version_mismatches
                else f"matched={len(pinned_requirements)}"
            )
            if result.returncode == 0
            else result.stderr.strip() or "distribution query failed",
        )

        pip_env, pip_env_error = pip_check_environment(
            python_executable, identity_env
        )
        if pip_env_error is not None:
            record("native runtime pip check", False, pip_env_error)
        else:
            pip_check = subprocess.run(
                [python_executable, "-m", "pip", "check"],
                env=pip_env,
                text=True,
                capture_output=True,
                check=False,
            )
            record(
                "native runtime pip check",
                pip_check.returncode == 0,
                pip_check.stdout.strip()
                or pip_check.stderr.strip()
                or "pip check succeeded",
            )

    sonic_lock = lock["source_revisions"]["gr00t_whole_body_control"]
    expected_sonic_commit = sonic_lock["commit"]
    checkout_root = git_checkout_root(sonic_root)
    if checkout_root is not None:
        result = subprocess.run(
            ["git", "-C", str(sonic_root), "rev-parse", "HEAD"],
            text=True,
            capture_output=True,
            check=False,
        )
        actual_sonic_commit = result.stdout.strip() if result.returncode == 0 else ""
        dirty_result = subprocess.run(
            [
                "git",
                "-C",
                str(sonic_root),
                "status",
                "--porcelain=v1",
                "--untracked-files=all",
            ],
            text=True,
            capture_output=True,
            check=False,
        )
        dirty_lines = dirty_result.stdout.strip().splitlines()
        source_clean = dirty_result.returncode == 0 and not dirty_lines
        record(
            "native SONIC source clean",
            source_clean,
            (
                "clean"
                if source_clean
                else "; ".join(dirty_lines)
                or dirty_result.stderr.strip()
                or "git status failed"
            ),
        )
        ignored_result = subprocess.run(
            [
                "git",
                "-C",
                str(checkout_root),
                "ls-files",
                "--others",
                "--ignored",
                "--exclude-standard",
                "-z",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        ignored_paths = [
            value.decode("utf-8", errors="surrogateescape")
            for value in ignored_result.stdout.split(b"\0")
            if value
        ]
        locked_sonic_files = {
            Path(entry["path"])
            for entry in lock["runtime_files"]
            if entry["root"] == "sonic"
        }
        locked_sonic_trees = [
            Path(entry["path"])
            for entry in lock["runtime_trees"]
            if entry["root"] == "sonic"
        ]

        def ignored_shared_object_is_locked(path: str) -> bool:
            candidate = Path(path)
            return candidate in locked_sonic_files or any(
                candidate.is_relative_to(tree) for tree in locked_sonic_trees
            )

        ignored_overlays = [
            path
            for path in ignored_paths
            if Path(path).suffix.lower() in {".py", ".pyi"}
            or (
                re.search(r"\.so(?:\.|$)", Path(path).name) is not None
                and not ignored_shared_object_is_locked(path)
            )
            or (
                Path(path).suffix.lower() == ".pyc"
                and "__pycache__" not in Path(path).parts
            )
        ]
        ignored_clean = ignored_result.returncode == 0 and not ignored_overlays
        record(
            "native SONIC ignored source overlays absent",
            ignored_clean,
            (
                "none"
                if ignored_clean
                else "; ".join(ignored_overlays)
                or ignored_result.stderr.decode("utf-8", errors="replace").strip()
                or "git ignored-source inventory failed"
            ),
        )
        commit_trusted = source_clean and ignored_clean
        commit_detail = f"HEAD={actual_sonic_commit}" if actual_sonic_commit else "HEAD unavailable"
    else:
        marker = sonic_root / "SONIC_COMMIT"
        actual_sonic_commit = ""
        if marker.is_file() and not marker.is_symlink():
            try:
                marker_text = marker.read_text(encoding="utf-8")
            except (OSError, UnicodeError):
                marker_text = ""
            if re.fullmatch(r"[0-9a-f]{40}\n?", marker_text):
                actual_sonic_commit = marker_text.strip()
        commit_trusted, attestation_detail = archived_source_attestation(
            lock, sonic_root
        )
        record(
            "archived SONIC critical source attestation",
            commit_trusted,
            attestation_detail,
        )
        commit_detail = (
            f"SONIC_COMMIT={actual_sonic_commit}; source_attested={commit_trusted}"
            if actual_sonic_commit
            else f"missing regular marker {marker}"
        )
    if args.require_git_sonic:
        record(
            "native SONIC Git checkout required",
            checkout_root is not None,
            str(checkout_root) if checkout_root is not None else "archived mirror",
        )
    record(
        "native SONIC commit",
        actual_sonic_commit == expected_sonic_commit and commit_trusted,
        commit_detail,
    )

    for entry in lock["runtime_files"]:
        base = roots.get(entry["root"])
        if base is None:
            record("runtime lock", False, f"unknown root {entry['root']}")
            continue
        path = base / entry["path"]
        try:
            inside_root = path.resolve().is_relative_to(base.resolve())
        except (OSError, RuntimeError):
            inside_root = False
        if not inside_root or not path.is_file() or path.is_symlink():
            record(str(path), False, "missing regular file")
            continue
        if args.fast and path.stat().st_size > LARGE_FILE_THRESHOLD:
            record(str(path), True, f"present ({path.stat().st_size} bytes; fast mode)")
            continue
        actual = sha256_file(path)
        record(str(path), actual == entry["sha256"], f"sha256={actual}")

    # This runs for both an original Git checkout and the packaged source
    # archive selected above. Tree locks cover runtime directories whose
    # inventory cannot be represented safely by a single presence check.
    for entry in lock["runtime_trees"]:
        base = roots.get(entry["root"])
        if base is None:
            record("runtime tree lock", False, f"unknown root {entry['root']}")
            continue
        ok, detail = runtime_tree_attestation(
            base, entry["path"], entry["sha256"]
        )
        record(f"runtime tree {base / entry['path']}", ok, detail)

    wheelhouse = runtime_root / "python-wheelhouse"
    for name, ok, detail in verify_wheelhouse(
        wheelhouse, lock["python"]["wheelhouse_manifest_sha256"]
    ):
        record(name, ok, detail)
    if python_site_packages is None:
        for name in (
            "Python wheel RECORD metadata",
            "native runtime Python installed wheel files",
            "native runtime Python site-packages inventory",
        ):
            record(name, False, "runtime Python is unavailable")
    else:
        for name, ok, detail in verify_python_wheel_records(
            wheelhouse,
            python_site_packages,
            pinned_requirements,
            Path(python_executable).absolute(),
        ):
            record(name, ok, detail)

    if args.pico_wheel:
        pico_wheel_ok, pico_wheel_detail = verify_pico_wheel(
            args.pico_wheel.expanduser(), lock["pico"]
        )
        record("native PICO wheel artifact", pico_wheel_ok, pico_wheel_detail)
    elif args.pico_python:
        record(
            "native PICO wheel artifact",
            False,
            "--pico-wheel is required with --pico-python",
        )

    if args.release_cache:
        cache = args.release_cache.resolve()
        for package in lock["matrix_release"]["packages"]:
            path = cache / package["file"]
            if not path.is_file():
                record(f"release {package['name']}", False, f"missing {path}")
                continue
            size_ok = path.stat().st_size == package["size"]
            if not size_ok:
                record(
                    f"release {package['name']}",
                    False,
                    f"size={path.stat().st_size} expected={package['size']}",
                )
                continue
            actual = sha256_file(path)
            record(
                f"release {package['name']}",
                actual == package["sha256"],
                f"sha256={actual}",
            )

    if not args.skip_installed_assets:
        for entry in lock["matrix_release"]["installed_files"]:
            relative = entry["path"]
            path = matrix_root / relative
            try:
                inside_root = path.resolve().is_relative_to(matrix_root)
            except (OSError, RuntimeError):
                inside_root = False
            if not inside_root or not path.is_file() or path.is_symlink():
                record(f"installed {relative}", False, "missing regular file")
                continue
            actual_size = path.stat().st_size
            if actual_size != entry["size"]:
                record(
                    f"installed {relative}",
                    False,
                    f"size={actual_size} expected={entry['size']}",
                )
                continue
            if args.fast and actual_size > LARGE_FILE_THRESHOLD:
                record(
                    f"installed {relative}",
                    True,
                    f"present ({actual_size} bytes; fast mode)",
                )
                continue
            actual = sha256_file(path)
            record(
                f"installed {relative}",
                actual == entry["sha256"],
                f"sha256={actual}",
            )
        for entry in lock["matrix_release"]["installed_trees"]:
            ok, detail = runtime_tree_attestation(
                matrix_root, entry["path"], entry["sha256"]
            )
            record(f"installed tree {matrix_root / entry['path']}", ok, detail)

    native_deps_root = Path(
        os.environ.get(
            "MATRIX_NATIVE_DEPS_ROOT", runtime_root / "matrix-native-deps"
        )
    ).resolve()
    if not args.skip_dynamic:
        env = dynamic_environment(runtime_root, matrix_root, sonic_root)
        env["PYTHONNOUSERSITE"] = "1"
        env["PYTHONDONTWRITEBYTECODE"] = "1"
        if python_executable is None:
            record("native SONIC Python API", False, "runtime Python is unavailable")
            record("gear_sonic import origin", False, "runtime Python is unavailable")
            record("unitree_sdk2py Python package", False, "runtime Python is unavailable")
            record("cyclonedds Python package", False, "runtime Python is unavailable")
        else:
            import_code = r"""
import importlib.util
import json
import sys
import traceback

sys.path.insert(0, sys.argv[1])

def describe(name):
    spec = importlib.util.find_spec(name)
    if spec is None:
        return {"found": False, "origin": "", "locations": []}
    return {
        "found": True,
        "origin": spec.origin or "",
        "locations": list(spec.submodule_search_locations or []),
    }

payload = {
    "gear_sonic": describe("gear_sonic"),
    "unitree_sdk2py": describe("unitree_sdk2py"),
    "cyclonedds": describe("cyclonedds"),
    "api_ok": False,
    "api_error": "",
    "run_sim_loop_origin": "",
}
try:
    from gear_sonic.scripts import run_sim_loop
    from gear_sonic.scripts.run_sim_loop import create_simulator
    from gear_sonic.utils.mujoco_sim.configs import SimLoopConfig
    from gear_sonic.utils.teleop.zmq.zmq_planner_sender import (
        build_command_message,
        build_planner_message,
    )
    payload["api_ok"] = True
    payload["run_sim_loop_origin"] = run_sim_loop.__file__ or ""
except BaseException:
    payload["api_error"] = traceback.format_exc()
print("MATRIX_VERIFY_IMPORT_JSON=" + json.dumps(payload, sort_keys=True))
"""
            result = subprocess.run(
                [python_executable, "-c", import_code, str(sonic_root)],
                env=env,
                text=True,
                capture_output=True,
                check=False,
            )
            prefix = "MATRIX_VERIFY_IMPORT_JSON="
            import_line = next(
                (
                    line[len(prefix) :]
                    for line in reversed(result.stdout.splitlines())
                    if line.startswith(prefix)
                ),
                "",
            )
            try:
                import_payload = json.loads(import_line) if import_line else {}
            except json.JSONDecodeError:
                import_payload = {}
            api_ok = result.returncode == 0 and import_payload.get("api_ok") is True
            record(
                "native SONIC Python API",
                api_ok,
                (
                    str(import_payload.get("run_sim_loop_origin"))
                    if api_ok
                    else str(import_payload.get("api_error"))
                    or result.stderr.strip()
                    or "missing structured import output"
                ),
            )

            gear = import_payload.get("gear_sonic", {})
            gear_candidates = [
                value
                for value in [
                    gear.get("origin", "") if isinstance(gear, dict) else "",
                    *(
                        gear.get("locations", [])
                        if isinstance(gear, dict)
                        and isinstance(gear.get("locations", []), list)
                        else []
                    ),
                    str(import_payload.get("run_sim_loop_origin", "")),
                ]
                if value
            ]
            gear_inside = bool(gear_candidates)
            for candidate in gear_candidates:
                try:
                    if not Path(candidate).resolve().is_relative_to(sonic_root):
                        gear_inside = False
                except (OSError, RuntimeError):
                    gear_inside = False
            record(
                "gear_sonic import origin",
                gear_inside,
                ", ".join(str(value) for value in gear_candidates)
                or "gear_sonic was not found",
            )
            for package_name in ("unitree_sdk2py", "cyclonedds"):
                package = import_payload.get(package_name, {})
                found = isinstance(package, dict) and package.get("found") is True
                origins = []
                if isinstance(package, dict):
                    origins = [
                        value
                        for value in [
                            package.get("origin", ""),
                            *(package.get("locations", []) or []),
                        ]
                        if value
                    ]
                record(
                    f"{package_name} Python package",
                    found,
                    ", ".join(str(value) for value in origins) or "not found",
                )

        if args.pico_python:
            pico_python = shutil.which(args.pico_python)
            if pico_python is None:
                candidate = Path(args.pico_python).expanduser()
                if candidate.is_file() and os.access(candidate, os.X_OK):
                    pico_python = str(candidate.absolute())
            if pico_python is None:
                record(
                    "native PICO Python isolation",
                    False,
                    f"unavailable interpreter: {args.pico_python}",
                )
                record(
                    "native PICO SDK wheel installation",
                    False,
                    f"unavailable interpreter: {args.pico_python}",
                )
                record(
                    "native PICO Python API",
                    False,
                    f"unavailable interpreter: {args.pico_python}",
                )
                record(
                    "native PICO pip check",
                    False,
                    f"unavailable interpreter: {args.pico_python}",
                )
            else:
                pico_lock = lock["pico"]
                pico_lexical_prefix = Path(pico_python).absolute().parent.parent
                pico_site_packages = (
                    pico_lexical_prefix
                    / "lib"
                    / f"python{pico_lock['python_version']}"
                    / "site-packages"
                )
                pico_identity_marker = "MATRIX_VERIFY_PICO_IDENTITY_JSON="
                pico_identity_code = python_identity_probe_code(
                    pico_identity_marker
                )
                pico_identity_env = env.copy()
                pico_identity_env.pop("PYTHONPATH", None)
                pico_identity_env["PYTHONNOUSERSITE"] = "1"
                pico_identity_result = subprocess.run(
                    [pico_python, "-c", pico_identity_code],
                    env=pico_identity_env,
                    cwd=matrix_root,
                    text=True,
                    capture_output=True,
                    check=False,
                )
                pico_identity_line = next(
                    (
                        line[len(pico_identity_marker) :]
                        for line in reversed(pico_identity_result.stdout.splitlines())
                        if line.startswith(pico_identity_marker)
                    ),
                    "",
                )
                try:
                    pico_identity = (
                        json.loads(pico_identity_line) if pico_identity_line else {}
                    )
                except json.JSONDecodeError:
                    pico_identity = {}
                if not isinstance(pico_identity, dict):
                    pico_identity = {}
                pico_isolation_ok, pico_isolation_detail = verify_python_isolation(
                    pico_lexical_prefix,
                    pico_site_packages,
                    matrix_root,
                    pico_identity,
                )
                if pico_identity_result.returncode != 0:
                    pico_isolation_ok = False
                    pico_isolation_detail = (
                        pico_identity_result.stderr.strip()
                        or "PICO identity query failed"
                    )
                record(
                    "native PICO Python isolation",
                    pico_isolation_ok,
                    pico_isolation_detail,
                )
                pico_env = env.copy()
                pico_env["PYTHONPATH"] = str(sonic_root) + (
                    os.pathsep + pico_env["PYTHONPATH"]
                    if pico_env.get("PYTHONPATH")
                    else ""
                )
                pico_prefix = "MATRIX_VERIFY_PICO_JSON="
                pico_code = r"""
import importlib.metadata
import json
import platform
from pathlib import Path
import sys
import sysconfig

payload = {
    "version": f"{sys.version_info.major}.{sys.version_info.minor}",
    "soabi": sysconfig.get_config_var("SOABI") or "",
    "machine": platform.machine(),
    "prefix": str(Path(sys.prefix).resolve()),
    "purelib": str(Path(sysconfig.get_path("purelib")).resolve()),
    "platlib": str(Path(sysconfig.get_path("platlib")).resolve()),
    "distribution_version": importlib.metadata.version(sys.argv[1]),
}
import xrobotoolkit_sdk
payload["origin"] = str(Path(xrobotoolkit_sdk.__file__).resolve())
print("MATRIX_VERIFY_PICO_JSON=" + json.dumps(payload, sort_keys=True))
"""
                if args.pico_wheel is None:
                    pico_install_ok = False
                    pico_install_detail = "--pico-wheel is required"
                else:
                    pico_install_ok, pico_install_detail = verify_installed_pico_wheel(
                        args.pico_wheel.expanduser(),
                        pico_site_packages,
                        pico_lock,
                    )
                record(
                    "native PICO SDK wheel installation",
                    pico_install_ok,
                    pico_install_detail,
                )
                result = subprocess.run(
                    [
                        pico_python,
                        "-c",
                        pico_code,
                        pico_lock["distribution"],
                    ],
                    env=pico_env,
                    text=True,
                    capture_output=True,
                    check=False,
                )
                pico_line = next(
                    (
                        line[len(pico_prefix) :]
                        for line in reversed(result.stdout.splitlines())
                        if line.startswith(pico_prefix)
                    ),
                    "",
                )
                try:
                    pico_payload = json.loads(pico_line) if pico_line else {}
                except json.JSONDecodeError:
                    pico_payload = {}
                try:
                    origin_inside_prefix = Path(
                        str(pico_payload.get("origin", ""))
                    ).resolve().is_relative_to(pico_lexical_prefix.resolve())
                except (OSError, RuntimeError):
                    origin_inside_prefix = False
                pico_mismatches = []
                for actual_key, lock_key in (
                    ("version", "python_version"),
                    ("soabi", "python_soabi"),
                    ("machine", "machine"),
                    ("distribution_version", "version"),
                ):
                    if pico_payload.get(actual_key) != pico_lock[lock_key]:
                        pico_mismatches.append(
                            f"{actual_key}:expected={pico_lock[lock_key]},"
                            f"actual={pico_payload.get(actual_key)}"
                        )
                for actual_key, expected_path in (
                    ("prefix", pico_lexical_prefix),
                    ("purelib", pico_site_packages),
                    ("platlib", pico_site_packages),
                ):
                    try:
                        actual_path = Path(
                            str(pico_payload.get(actual_key, ""))
                        ).resolve()
                        path_matches = actual_path == expected_path.resolve()
                    except (OSError, RuntimeError):
                        path_matches = False
                    if not path_matches:
                        pico_mismatches.append(
                            f"{actual_key}:expected={expected_path},"
                            f"actual={pico_payload.get(actual_key)}"
                        )
                if not origin_inside_prefix:
                    pico_mismatches.append(
                        f"origin outside interpreter prefix: {pico_payload.get('origin', '')}"
                    )
                record(
                    "native PICO Python API",
                    result.returncode == 0 and not pico_mismatches,
                    "; ".join(pico_mismatches)
                    or str(pico_payload.get("origin", ""))
                    or result.stderr.strip()
                    or "import failed",
                )
                pico_pip_check = subprocess.run(
                    [pico_python, "-m", "pip", "check"],
                    env=pico_env,
                    text=True,
                    capture_output=True,
                    check=False,
                )
                record(
                    "native PICO pip check",
                    pico_pip_check.returncode == 0,
                    pico_pip_check.stdout.strip()
                    or pico_pip_check.stderr.strip()
                    or "pip check succeeded",
                )

        deploy = roots["sonic"] / "gear_sonic_deploy/target/release/g1_deploy_onnx_ref"
        if deploy.is_file():
            result = subprocess.run(
                ["ldd", str(deploy)],
                env=env,
                text=True,
                capture_output=True,
                check=False,
            )
            missing = [line.strip() for line in result.stdout.splitlines() if "not found" in line]
            record("SONIC deploy dependency closure", result.returncode == 0 and not missing, "; ".join(missing) or "complete")
            expected_inference = roots["inference"].resolve()
            for soname in (
                "libonnxruntime.so.1.16.3",
                "libnvinfer.so.10",
                "libnvonnxparser.so.10",
            ):
                line = next(
                    (
                        item.strip()
                        for item in result.stdout.splitlines()
                        if item.strip().startswith(f"{soname} =>")
                    ),
                    "",
                )
                resolved = line.split("=>", 1)[1].strip().split(" ", 1)[0] if line else ""
                try:
                    exact = Path(resolved).resolve().is_relative_to(expected_inference)
                except (OSError, RuntimeError):
                    exact = False
                record(
                    f"{soname} resolution",
                    exact,
                    resolved or "not present in ldd output",
                )

            soname = "libcudart.so.12"
            line = next(
                (
                    item.strip()
                    for item in result.stdout.splitlines()
                    if item.strip().startswith(f"{soname} =>")
                ),
                "",
            )
            resolved = (
                line.split("=>", 1)[1].strip().split(" ", 1)[0]
                if line
                else ""
            )
            expected_cuda = Path(
                os.environ.get("MATRIX_CUDA_ROOT", "/usr/local/cuda")
            ).resolve()
            try:
                exact = Path(resolved).resolve().is_relative_to(expected_cuda)
            except (OSError, RuntimeError):
                exact = False
            record(
                f"{soname} resolution",
                exact,
                resolved or "not present in ldd output",
            )

        ue_binary = (
            matrix_root
            / "src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux/zsibot_mujoco_ue"
        )
        if ue_binary.is_file():
            ue_ldd = subprocess.run(
                ["ldd", str(ue_binary)],
                env=env,
                text=True,
                capture_output=True,
                check=False,
            )
            missing = [
                line.strip()
                for line in ue_ldd.stdout.splitlines()
                if "not found" in line
            ]
            private_roots = [
                matrix_root
                / "src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux",
                matrix_root / "src/UeSim/Linux/Engine/Binaries/Linux",
                matrix_root
                / "src/UeSim/Linux/Engine/Plugins/Runtime/OpenCV/Binaries/ThirdParty/Linux",
                native_deps_root / "usr/lib",
            ]
            system_roots = [Path("/lib"), Path("/usr/lib")]
            unauthorized: list[str] = []
            for line in ue_ldd.stdout.splitlines():
                if "=>" not in line:
                    continue
                resolved = line.split("=>", 1)[1].strip().split(" ", 1)[0]
                if not resolved.startswith("/"):
                    continue
                resolved_path = Path(resolved).resolve()
                if not any(
                    resolved_path.is_relative_to(root.resolve())
                    for root in (*private_roots, *system_roots)
                    if root.exists()
                ):
                    unauthorized.append(str(resolved_path))
            record(
                "Matrix UE dependency closure",
                ue_ldd.returncode == 0 and not missing and not unauthorized,
                "; ".join(missing + unauthorized) or "complete and rooted",
            )

        trt = roots["inference"] / "TensorRT/lib/libnvinfer.so.10"
        if trt.is_file():
            if python_executable is None:
                ok, detail = False, "runtime Python is unavailable"
            else:
                ok, detail = check_tensorrt_version(
                    trt,
                    lock["inference"]["tensorrt_api_version"],
                    env,
                    python_executable,
                )
            record("TensorRT ABI", ok, detail)

        ros_prefix = Path(
            os.environ.get(
                "MATRIX_ROS_PREFIX", runtime_root / "ros2-humble-prefix"
            )
        )
        rmw = ros_prefix / "lib/librmw_fastrtps_cpp.so"
        if args.profile in {"heyuan", "trna"} or rmw.exists():
            if not rmw.exists():
                record("ROS2 RMW", False, f"missing {rmw}")
            elif python_executable is None:
                record("ROS2 RMW", False, "runtime Python is unavailable")
            else:
                ok, detail = check_dlopen(rmw, env, python_executable)
                record("ROS2 RMW", ok, detail)

    native_deps_root = Path(
        os.environ.get(
            "MATRIX_NATIVE_DEPS_ROOT", runtime_root / "matrix-native-deps"
        )
    ).resolve()
    ros_prefix = Path(
        os.environ.get("MATRIX_ROS_PREFIX", runtime_root / "ros2-humble-prefix")
    ).resolve()
    cuda_root = Path(os.environ.get("MATRIX_CUDA_ROOT", "/usr/local/cuda")).resolve()
    verification_flags = {
        "fast": bool(args.fast),
        "skip_dynamic": bool(args.skip_dynamic),
        "skip_installed_assets": bool(args.skip_installed_assets),
        "require_git_sonic": bool(args.require_git_sonic),
    }
    verification_inventory = {
        "runtime_files_expected": len(lock["runtime_files"]),
        "runtime_files_checked": len(lock["runtime_files"]),
        "runtime_trees_expected": len(lock["runtime_trees"]),
        "runtime_trees_checked": len(lock["runtime_trees"]),
        "installed_files_expected": len(lock["matrix_release"]["installed_files"]),
        "installed_files_checked": (
            0
            if args.skip_installed_assets
            else len(lock["matrix_release"]["installed_files"])
        ),
        "installed_trees_expected": len(lock["matrix_release"]["installed_trees"]),
        "installed_trees_checked": (
            0
            if args.skip_installed_assets
            else len(lock["matrix_release"]["installed_trees"])
        ),
        "dynamic_checks_performed": not args.skip_dynamic,
    }
    core_qualification_checks = {
        "Matrix source commit",
        "Matrix tracked source clean",
        "Matrix ignored source overlays absent",
        "native runtime Python",
        "Python requirements lock",
        "Python requirements exact pins",
        "Python lock scalar consistency",
        "native runtime Python version",
        "native runtime Python SOABI",
        "native runtime machine",
        "native runtime Python prefix",
        "native runtime Python isolation",
        "native runtime Python distributions",
        "native runtime pip check",
        "native SONIC source clean",
        "native SONIC ignored source overlays absent",
        "native SONIC Git checkout required",
        "native SONIC commit",
        "Python wheelhouse manifest",
        "Python wheelhouse inventory",
        "Python wheelhouse compatibility",
        "Python wheelhouse contents",
        "Python wheel RECORD metadata",
        "native runtime Python installed wheel files",
        "native runtime Python site-packages inventory",
        "native SONIC Python API",
        "gear_sonic import origin",
        "unitree_sdk2py Python package",
        "cyclonedds Python package",
        "SONIC deploy dependency closure",
        "Matrix UE dependency closure",
        "libonnxruntime.so.1.16.3 resolution",
        "libnvinfer.so.10 resolution",
        "libnvonnxparser.so.10 resolution",
        "libcudart.so.12 resolution",
        "TensorRT ABI",
    }
    if args.profile in {"heyuan", "trna"} or (
        ros_prefix / "lib/librmw_fastrtps_cpp.so"
    ).exists():
        core_qualification_checks.add("ROS2 RMW")
    if args.pico_python or args.pico_wheel:
        core_qualification_checks.update(
            {
                "native PICO wheel artifact",
                "native PICO Python isolation",
                "native PICO SDK wheel installation",
                "native PICO Python API",
                "native PICO pip check",
            }
        )
    check_names = {
        str(item.get("name")) for item in checks if isinstance(item, dict)
    }
    missing_qualification_checks = sorted(core_qualification_checks - check_names)
    full_verification = not (
        args.fast or args.skip_dynamic or args.skip_installed_assets
    )
    qualification_eligible = (
        full_verification
        and args.require_git_sonic
        and checkout_root is not None
        and not missing_qualification_checks
        and all(item["ok"] for item in checks)
    )
    payload = {
        "lock": str(args.lock.resolve()),
        "lock_sha256": sha256_file(args.lock.resolve()),
        "matrix_commit": matrix_commit,
        "matrix_root": str(matrix_root),
        "runtime_root": str(runtime_root),
        "sonic_root": str(sonic_root),
        "launch_roots": {
            "inference": str((runtime_root / "inference").resolve()),
            "visual_urdf": str((runtime_root / "g1-visual/g1_29dof.urdf").resolve()),
            "unitree_sdk2": str(
                (
                    sonic_root
                    / "gear_sonic_deploy/thirdparty/unitree_sdk2"
                ).resolve()
            ),
            "canonical_model": str(
                (
                    sonic_root
                    / "gear_sonic/data/robot_model/model_data/g1/g1_29dof_with_hand.xml"
                ).resolve()
            ),
            "canonical_meshes": str(
                (
                    sonic_root
                    / "gear_sonic/data/robot_model/model_data/g1/meshes"
                ).resolve()
            ),
            "native_deps": str(native_deps_root),
            "ros_prefix": str(ros_prefix),
            "cuda": str(cuda_root),
        },
        "launch_environment": {
            "ld_library_path": os.environ.get("LD_LIBRARY_PATH", ""),
            "pythonpath": os.environ.get("PYTHONPATH", ""),
            "tensorrt_root": os.environ.get("TensorRT_ROOT", ""),
            "python_pycache_prefix": os.environ.get("PYTHONPYCACHEPREFIX", ""),
            "python_dont_write_bytecode": os.environ.get(
                "PYTHONDONTWRITEBYTECODE", ""
            ),
        },
        "profile": args.profile,
        "python": str(Path(python_executable).absolute()) if python_executable else None,
        "python_prefix": (
            str(Path(identity_prefix_value).absolute())
            if isinstance(identity_prefix_value, str) and identity_prefix_value
            else None
        ),
        "pico_python": (
            str(Path(args.pico_python).expanduser().absolute())
            if args.pico_python
            else None
        ),
        "pico_wheel": (
            str(args.pico_wheel.expanduser().resolve()) if args.pico_wheel else None
        ),
        "full_hashes": full_verification,
        "sonic_git_checkout": checkout_root is not None,
        "verification_flags": verification_flags,
        "verification_inventory": verification_inventory,
        "qualification_required_checks": sorted(core_qualification_checks),
        "missing_qualification_checks": missing_qualification_checks,
        "qualification_eligible": qualification_eligible,
        "passed": all(item["ok"] for item in checks),
        "checks": checks,
    }
    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    return 0 if payload["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
