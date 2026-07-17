#!/usr/bin/env python3
"""Verify the locked Matrix + SONIC runtime without modifying the host."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
import sys
from typing import Any


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


def runtime_roots(runtime_root: Path, sonic_root: Path) -> dict[str, Path]:
    return {
        "sonic": sonic_root,
        "visual": runtime_root / "g1-visual",
        "inference": runtime_root / "inference",
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
    native = runtime_root / "matrix-native-deps"
    ros = runtime_root / "ros2-humble-prefix"
    cuda = Path(os.environ.get("MATRIX_CUDA_ROOT", "/usr/local/cuda"))
    candidates = [
        runtime_root / "inference/TensorRT/lib",
        runtime_root / "inference/onnxruntime/lib",
        sonic_root / "gear_sonic_deploy/thirdparty/unitree_sdk2/thirdparty/lib/x86_64",
        sonic_root
        / "external_dependencies/XRoboToolkit-PC-Service-Pybind_X86_and_ARM64/lib",
        native / "usr/lib",
        native / "usr/lib/x86_64-linux-gnu",
        native / "usr/local/lib",
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
    python_identity: dict[str, str] = {}
    identity_error = "runtime Python is unavailable"
    if python_executable is not None:
        identity_prefix = "MATRIX_VERIFY_PYTHON_JSON="
        identity_code = (
            "import json,platform,sys,sysconfig; "
            f"print('{identity_prefix}' + json.dumps({{'version': "
            "f'{sys.version_info.major}.{sys.version_info.minor}', "
            "'soabi': sysconfig.get_config_var('SOABI') or '', "
            "'machine': platform.machine()}))"
        )
        identity_result = subprocess.run(
            [python_executable, "-c", identity_code],
            env=identity_env,
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
                    python_identity = {
                        key: str(candidate_identity.get(key, ""))
                        for key in ("version", "soabi", "machine")
                    }
            except json.JSONDecodeError:
                identity_error = f"invalid JSON: {identity_result.stdout!r}"
    for key, label in (
        ("version", "native runtime Python version"),
        ("soabi", "native runtime Python SOABI"),
        ("machine", "native runtime machine"),
    ):
        actual_identity = python_identity.get(key, "")
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

        pip_check = subprocess.run(
            [python_executable, "-m", "pip", "check"],
            env=identity_env,
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
                "--",
                *sonic_lock["critical_source_paths"],
            ],
            text=True,
            capture_output=True,
            check=False,
        )
        dirty_lines = dirty_result.stdout.strip().splitlines()
        source_clean = dirty_result.returncode == 0 and not dirty_lines
        record(
            "native SONIC critical source clean",
            source_clean,
            (
                "clean"
                if source_clean
                else "; ".join(dirty_lines)
                or dirty_result.stderr.strip()
                or "git status failed"
            ),
        )
        commit_trusted = source_clean
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
        for relative in lock["matrix_release"]["installed_files"]:
            path = matrix_root / relative
            record(f"installed {relative}", path.is_file(), str(path))

    if not args.skip_dynamic:
        env = dynamic_environment(runtime_root, matrix_root, sonic_root)
        env["PYTHONNOUSERSITE"] = "1"
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
                    pico_python = str(candidate.resolve())
            if pico_python is None:
                record(
                    "native PICO Python API",
                    False,
                    f"unavailable interpreter: {args.pico_python}",
                )
            else:
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
    "distribution_version": importlib.metadata.version(sys.argv[1]),
}
import xrobotoolkit_sdk
payload["origin"] = str(Path(xrobotoolkit_sdk.__file__).resolve())
print("MATRIX_VERIFY_PICO_JSON=" + json.dumps(payload, sort_keys=True))
"""
                pico_lock = lock["pico"]
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
                    ).resolve().is_relative_to(
                        Path(str(pico_payload.get("prefix", ""))).resolve()
                    )
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

        rmw = runtime_root / "ros2-humble-prefix/lib/librmw_fastrtps_cpp.so"
        if args.profile == "heyuan" or rmw.exists():
            if not rmw.exists():
                record("ROS2 RMW", False, f"missing {rmw}")
            elif python_executable is None:
                record("ROS2 RMW", False, "runtime Python is unavailable")
            else:
                ok, detail = check_dlopen(rmw, env, python_executable)
                record("ROS2 RMW", ok, detail)

    payload = {
        "lock": str(args.lock.resolve()),
        "lock_sha256": sha256_file(args.lock.resolve()),
        "matrix_commit": matrix_commit,
        "matrix_root": str(matrix_root),
        "runtime_root": str(runtime_root),
        "sonic_root": str(sonic_root),
        "profile": args.profile,
        "passed": all(item["ok"] for item in checks),
        "checks": checks,
    }
    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    return 0 if payload["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
