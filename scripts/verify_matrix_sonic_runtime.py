#!/usr/bin/env python3
"""Verify the locked Matrix + SONIC runtime without modifying the host."""

from __future__ import annotations

import argparse
import ctypes
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LOCK = REPO_ROOT / "config/runtime/matrix-sonic.lock.json"
DEFAULT_RUNTIME = REPO_ROOT / "outputs/runtime/matrix-sonic-v1"
LARGE_FILE_THRESHOLD = 64 * 1024 * 1024


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_lock(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        payload = json.load(stream)
    validate_schema(payload)
    return payload


def validate_schema(lock: dict[str, Any]) -> None:
    required = {
        "schema_version",
        "runtime_id",
        "matrix_release",
        "inference",
        "runtime_files",
        "acceptance",
    }
    missing = sorted(required.difference(lock))
    if missing:
        raise ValueError(f"runtime lock is missing keys: {', '.join(missing)}")
    if lock["schema_version"] != 1:
        raise ValueError(f"unsupported runtime lock schema: {lock['schema_version']}")
    if not isinstance(lock["runtime_files"], list) or not lock["runtime_files"]:
        raise ValueError("runtime_files must be a non-empty list")
    identities: set[tuple[str, str]] = set()
    for entry in lock["runtime_files"]:
        for key in ("root", "path", "sha256"):
            if not isinstance(entry.get(key), str) or not entry[key]:
                raise ValueError(f"invalid runtime file entry: missing {key}")
        identity = (entry["root"], entry["path"])
        if identity in identities:
            raise ValueError(f"duplicate runtime file entry: {identity}")
        identities.add(identity)
        if len(entry["sha256"]) != 64:
            raise ValueError(f"invalid SHA256 for {identity}")


def runtime_roots(runtime_root: Path) -> dict[str, Path]:
    return {
        "aue": runtime_root / "aue-sim",
        "gear": runtime_root / "GR00T-WholeBodyControl",
        "visual": runtime_root / "g1-visual",
        "bridge": runtime_root / "bridge",
        "inference": runtime_root / "inference",
    }


def library_paths(runtime_root: Path, matrix_root: Path) -> list[Path]:
    gear = runtime_root / "GR00T-WholeBodyControl"
    native = runtime_root / "matrix-native-deps"
    ros = runtime_root / "ros2-humble-prefix"
    candidates = [
        runtime_root / "inference/TensorRT/lib",
        runtime_root / "inference/onnxruntime/lib",
        gear / "gear_sonic_deploy/thirdparty/unitree_sdk2/thirdparty/lib/x86_64",
        native / "usr/lib",
        native / "usr/lib/x86_64-linux-gnu",
        native / "usr/local/lib",
        ros / "lib",
        matrix_root / "src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux",
        matrix_root / "src/UeSim/Linux/Engine/Binaries/Linux",
        Path("/usr/local/cuda/lib64"),
    ]
    return [path for path in candidates if path.is_dir()]


def dynamic_environment(runtime_root: Path, matrix_root: Path) -> dict[str, str]:
    env = os.environ.copy()
    paths = [str(path) for path in library_paths(runtime_root, matrix_root)]
    if env.get("LD_LIBRARY_PATH"):
        paths.append(env["LD_LIBRARY_PATH"])
    env["LD_LIBRARY_PATH"] = ":".join(paths)
    return env


def check_tensorrt_version(
    library: Path, expected: int, env: dict[str, str]
) -> tuple[bool, str]:
    code = (
        "import ctypes,sys; "
        "lib=ctypes.CDLL(sys.argv[1], mode=ctypes.RTLD_GLOBAL); "
        "fn=lib.getInferLibVersion; fn.restype=ctypes.c_int; print(fn())"
    )
    result = subprocess.run(
        [sys.executable, "-c", code, str(library)],
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


def check_dlopen(path: Path, env: dict[str, str]) -> tuple[bool, str]:
    code = "import ctypes,sys; ctypes.CDLL(sys.argv[1], mode=ctypes.RTLD_GLOBAL)"
    result = subprocess.run(
        [sys.executable, "-c", code, str(path)],
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
    parser.add_argument("--release-cache", type=Path)
    parser.add_argument("--profile", choices=("heyuan", "trna"))
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
    roots = runtime_roots(runtime_root)
    checks: list[dict[str, Any]] = []

    def record(name: str, ok: bool, detail: str) -> None:
        checks.append({"name": name, "ok": ok, "detail": detail})
        print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")

    for entry in lock["runtime_files"]:
        base = roots.get(entry["root"])
        if base is None:
            record("runtime lock", False, f"unknown root {entry['root']}")
            continue
        path = base / entry["path"]
        if not path.is_file():
            record(str(path), False, "missing")
            continue
        if args.fast and path.stat().st_size > LARGE_FILE_THRESHOLD:
            record(str(path), True, f"present ({path.stat().st_size} bytes; fast mode)")
            continue
        actual = sha256_file(path)
        record(str(path), actual == entry["sha256"], f"sha256={actual}")

    wheelhouse_manifest = runtime_root / "python-wheelhouse/SHA256SUMS"
    if wheelhouse_manifest.exists():
        actual = sha256_file(wheelhouse_manifest)
        expected = lock["python"]["wheelhouse_manifest_sha256"]
        record("Python wheelhouse manifest", actual == expected, f"sha256={actual}")

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
        env = dynamic_environment(runtime_root, matrix_root)
        deploy = roots["gear"] / "gear_sonic_deploy/target/release/g1_deploy_onnx_ref"
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

        trt = roots["inference"] / "TensorRT/lib/libnvinfer.so.10"
        if trt.is_file():
            ok, detail = check_tensorrt_version(
                trt, lock["inference"]["tensorrt_api_version"], env
            )
            record("TensorRT ABI", ok, detail)

        rmw = runtime_root / "ros2-humble-prefix/lib/librmw_fastrtps_cpp.so"
        if args.profile == "heyuan" or rmw.exists():
            if not rmw.exists():
                record("ROS2 RMW", False, f"missing {rmw}")
            else:
                ok, detail = check_dlopen(rmw, env)
                record("ROS2 RMW", ok, detail)

    payload = {
        "lock": str(args.lock.resolve()),
        "runtime_root": str(runtime_root),
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
