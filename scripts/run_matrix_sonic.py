#!/usr/bin/env python3
"""Run native gear_sonic MuJoCo physics and mirror it into a Matrix UE map."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import tempfile
import time
from typing import Any, Callable, TypedDict
from urllib.parse import urlsplit
import uuid

_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

from matrix_game_control import (
    ControlConfig,
    GameControlCore,
    InputProtocolError,
    InputRejectedError,
    KEYBOARD_GAIT_TARGETS_MPS,
    PROTOCOL_NAME,
    RobotMotionCommand,
    SONIC_GAIT_NAMES,
    SONIC_GAIT_SPEED_RANGES_MPS,
    SONIC_IDLE_MODE,
    SONIC_RUN_MODE,
    SONIC_SLOW_WALK_MODE,
    SONIC_WALK_MODE,
    UnixInputConnection,
    UnixSeqpacketInputServer,
    wrap_angle_rad,
)
from matrix_mouse_settings import canonical_remote_speed_scale


def _remote_speed_scale_argument(value: str) -> float:
    try:
        numeric = float(value)
        return canonical_remote_speed_scale(numeric)
    except (TypeError, ValueError) as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--sonic-root", type=Path, required=True)
    parser.add_argument(
        "--control-source",
        choices=("planner", "game", "pico", "external"),
        default="planner",
    )
    parser.add_argument("--planner-bind", default="tcp://127.0.0.1:5556")
    parser.add_argument(
        "--game-input-socket",
        type=Path,
        default=Path(os.environ.get("XDG_RUNTIME_DIR", tempfile.gettempdir()))
        / f"matrix-game-control-{os.getuid()}-{os.getpid()}.sock",
        help="User-local Unix socket for camera-relative input snapshots",
    )
    parser.add_argument(
        "--game-max-speed",
        type=float,
        default=0.30,
        help="Analog SLOW_WALK cap (default 0.30, maximum 0.80); keyboard targets are fixed",
    )
    parser.add_argument("--game-max-acceleration", type=float, default=1.20)
    parser.add_argument("--game-max-deceleration", type=float, default=2.40)
    parser.add_argument("--game-max-turn-rate", type=float, default=2.50)
    parser.add_argument("--game-stick-deadzone", type=float, default=0.15)
    parser.add_argument("--game-input-timeout", type=float, default=0.15)
    parser.add_argument("--game-max-snapshot-age", type=float, default=0.15)
    parser.add_argument("--game-max-future-skew", type=float, default=0.05)
    parser.add_argument(
        "--game-input-provider",
        type=Path,
        default=_SCRIPT_DIR / "matrix_game_control_input.py",
    )
    parser.add_argument("--game-input-provider-python", default=sys.executable)
    parser.add_argument(
        "--game-input-source",
        choices=("auto", "keyboard", "gamepad"),
        default="auto",
    )
    parser.add_argument(
        "--game-camera-yaw-source",
        choices=(
            "x11-mirror",
            "x11-core-gated",
            "x11-absolute",
            "carla",
            "fixed",
        ),
        default="fixed",
    )
    parser.add_argument(
        "--game-look-button", choices=("left", "middle", "right"), default="left"
    )
    parser.add_argument(
        "--game-initial-camera-yaw-deg",
        type=float,
        default=0.0,
        help="Initial provider/UE yaw before provider-to-SONIC sign and offset",
    )
    parser.add_argument("--game-mouse-sensitivity-deg", type=float, default=0.12)
    parser.add_argument("--game-mouse-settings-file", type=Path)
    parser.add_argument(
        "--game-applied-mouse-profile",
        choices=("local", "remote"),
        default="local",
    )
    parser.add_argument(
        "--game-applied-mouse-speed-scale",
        type=_remote_speed_scale_argument,
        default=1.0,
    )
    parser.add_argument("--game-restart-request-file", type=Path)
    parser.add_argument("--game-restart-capability-file", type=Path)
    parser.add_argument("--game-restart-launcher-pid", type=int)
    parser.add_argument(
        "--game-camera-yaw-sign", type=int, choices=(-1, 1), default=-1
    )
    parser.add_argument("--game-camera-yaw-offset-deg", type=float, default=0.0)
    parser.add_argument("--game-carla-host", default="127.0.0.1")
    parser.add_argument("--game-carla-port", type=int, default=2000)
    parser.add_argument(
        "--gamepad-look-yaw-rate-deg-s", type=float, default=120.0
    )
    parser.add_argument(
        "--gamepad-look-pitch-rate-deg-s", type=float, default=90.0
    )
    parser.add_argument("--gamepad-look-deadzone", type=float, default=0.12)
    parser.add_argument("--gamepad-look-min-pitch-deg", type=float, default=-80.0)
    parser.add_argument("--gamepad-look-max-pitch-deg", type=float, default=60.0)
    parser.add_argument("--game-focus-title", default=r"(zsibot|matrix|unreal)")
    parser.add_argument("--game-input-status-file", type=Path)
    parser.add_argument("--no-game-input-provider", action="store_true")
    parser.add_argument(
        "--pico-python",
        default=None,
        help="Interpreter for SONIC's PICO manager (defaults to the simulator Python)",
    )
    parser.add_argument("--dds-interface", default="lo")
    parser.add_argument("--render-host", default="127.0.0.1")
    parser.add_argument("--render-port", type=int, default=9999)
    parser.add_argument(
        "--no-render-sync",
        action="store_true",
        help="Run physics/control without publishing state to a Matrix UE process",
    )
    parser.add_argument("--physics-hz", type=float, default=200.0)
    parser.add_argument("--control-hz", type=float, default=50.0)
    parser.add_argument("--max-seconds", type=float, default=0.0)
    parser.add_argument(
        "--fail-on-fall",
        action="store_true",
        help="Return non-zero when SONIC's authoritative fall flag is set",
    )
    parser.add_argument(
        "--min-active-seconds",
        type=float,
        default=0.0,
        help="Return non-zero unless native SONIC has one continuous fresh-lowcmd streak this long",
    )
    parser.add_argument(
        "--min-displacement-m",
        type=float,
        default=0.0,
        help="Minimum final world-XY root displacement required for acceptance",
    )
    parser.add_argument(
        "--min-final-x",
        type=float,
        default=None,
        help="Optional minimum final world-X root coordinate for directional acceptance",
    )
    parser.add_argument(
        "--min-forward-x-m",
        type=float,
        default=0.0,
        help="Minimum signed final-minus-initial world-X displacement",
    )
    parser.add_argument(
        "--low-cmd-fresh-timeout-seconds",
        type=float,
        default=0.1,
        help="Maximum native DDS lowcmd age that counts as fresh (default: 0.1)",
    )
    parser.add_argument("--min-physics-hz", type=float, default=0.0)
    parser.add_argument("--min-rtf", type=float, default=0.0)
    parser.add_argument("--max-resets", type=int, default=0)
    parser.add_argument("--walk-after", type=float, default=-1.0)
    parser.add_argument("--vx", type=float, default=0.30)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--yaw-rate", type=float, default=0.0)
    parser.add_argument("--status-file", type=Path)
    parser.add_argument("--qualified-runtime", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--qualification-profile", default=None, help=argparse.SUPPRESS)
    parser.add_argument("--runtime-lock-sha256", default=None, help=argparse.SUPPRESS)
    parser.add_argument("--scenario-layout-sha256", default=None, help=argparse.SUPPRESS)
    parser.add_argument("--matrix-commit", default=None, help=argparse.SUPPRESS)
    parser.add_argument(
        "--verification-receipt", type=Path, default=None, help=argparse.SUPPRESS
    )
    parser.add_argument(
        "--expected-parent-pid",
        type=int,
        default=None,
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--external-failure-file",
        type=Path,
        default=None,
        help=argparse.SUPPRESS,
    )
    parser.add_argument("--ue-pid", type=int, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--print-every", type=float, default=2.0)
    parser.add_argument(
        "--startup-band",
        action="store_true",
        help="Wait for native DDS lowcmd, hold the root, then fade the elastic band",
    )
    parser.add_argument("--startup-band-hold", type=float, default=4.0)
    parser.add_argument("--startup-band-fade", type=float, default=3.0)
    return parser.parse_args()


def _atomic_json(path: Path | None, payload: dict[str, object]) -> None:
    if path is None:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", dir=path.parent, prefix=f".{path.name}.", delete=False
    ) as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
        temporary_path = Path(stream.name)
    os.replace(temporary_path, path)


_UNKNOWN_EXTERNAL_EXIT_CODE = 255


def _read_external_failure(path: Path | None) -> tuple[str, int] | None:
    """Consume one atomic external-child failure record without inventing success."""
    if path is None or not path.exists():
        return None
    try:
        if path.is_symlink() or not path.is_file():
            raise ValueError("failure channel is not a regular file")
        payload = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(payload, dict) or payload.get("name") != "ue":
            raise ValueError("failure channel does not identify the UE child")
        exit_code = payload.get("exit_code")
        if type(exit_code) is not int or not 0 <= exit_code <= 255:
            raise ValueError("failure channel has an invalid exit code")
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as exc:
        print(
            "matrix-sonic-runtime ERROR invalid UE failure channel: "
            f"{path}: {exc}",
            flush=True,
        )
        return "ue", _UNKNOWN_EXTERNAL_EXIT_CODE
    return "ue", exit_code


def _record_external_child_failure(
    path: Path | None, failure: tuple[str, int]
) -> None:
    """Idempotently publish a late UE exit, even if no status was written yet."""
    if path is None:
        return
    payload: dict[str, object] = {}
    if path.exists():
        if not path.is_file():
            print(
                "matrix-sonic-runtime ERROR status path for UE failure is not a file: "
                f"{path}",
                flush=True,
            )
            return
        try:
            loaded = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError) as exc:
            print(
                f"matrix-sonic-runtime ERROR reading status for UE failure: {exc}",
                flush=True,
            )
            return
        if not isinstance(loaded, dict):
            return
        payload = loaded
    name, exit_code = failure
    failure_label = f"native_child_exit:{name}:{exit_code}"
    failures = payload.get("acceptance_failures")
    if not isinstance(failures, list):
        failures = []
    if failure_label not in failures:
        failures.append(failure_label)
    payload["acceptance_failures"] = failures
    payload["failed_child_name"] = name
    payload["failed_child_exit_code"] = exit_code
    payload["passed"] = False
    payload["completed"] = False
    if payload.get("termination_reason") != "child_exit":
        payload["pre_external_termination_reason"] = payload.get("termination_reason")
    payload["termination_reason"] = "child_exit"
    _atomic_json(path, payload)


def _arm_supervisor_parent_death(expected_parent_pid: int | None) -> None:
    if expected_parent_pid is None:
        return
    if expected_parent_pid <= 1:
        raise SystemExit("--expected-parent-pid must identify a live launcher")
    from exec_with_parent_death_signal import _arm_parent_death_signal

    _arm_parent_death_signal(signal.SIGTERM)
    if os.getppid() != expected_parent_pid:
        print(
            "matrix-sonic-runtime ERROR launcher exited before supervisor startup",
            file=sys.stderr,
        )
        raise SystemExit(125)


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _expected_receipt_roots(
    profile: str, runtime_root: Path, sonic_root: Path
) -> dict[str, str]:
    if profile == "trna":
        ros_prefix = Path("/opt/ros/humble")
        cuda_root = Path("/usr/local/cuda")
    elif profile == "heyuan":
        ros_prefix = runtime_root / "ros2-humble-prefix"
        cuda_root = Path("/usr/local/cuda")
    elif profile == "zza":
        ros_prefix = runtime_root / "ros2-humble-prefix"
        cuda_root = Path("/data/user_data/matrix-tools/cuda-runtime-12.1")
    else:
        raise SystemExit(f"unsupported qualification profile: {profile}")
    return {
        "inference": str((runtime_root / "inference").resolve()),
        "visual_urdf": str((runtime_root / "g1-visual/g1_29dof.urdf").resolve()),
        "unitree_sdk2": str(
            (
                sonic_root / "gear_sonic_deploy/thirdparty/unitree_sdk2"
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
        "native_deps": str((runtime_root / "matrix-native-deps").resolve()),
        "ros_prefix": str(ros_prefix.resolve()),
        "cuda": str(cuda_root.resolve()),
    }


def _validate_qualification_receipt(
    args: argparse.Namespace,
) -> dict[str, Any] | None:
    if not args.qualified_runtime:
        return None
    matrix_root = Path(__file__).resolve().parents[1]
    lock_path = matrix_root / "config/runtime/matrix-sonic.lock.json"
    if _sha256_file(lock_path) != args.runtime_lock_sha256:
        raise SystemExit("qualified runtime lock SHA does not match the active lock")
    commit = subprocess.run(
        ["git", "-C", str(matrix_root), "rev-parse", "HEAD"],
        text=True,
        capture_output=True,
        check=False,
    )
    if (
        commit.returncode != 0
        or commit.stdout.strip() != args.matrix_commit
    ):
        raise SystemExit("qualified runtime requires the recorded Matrix commit")

    receipt_path = args.verification_receipt
    if receipt_path is None or not receipt_path.is_file() or receipt_path.is_symlink():
        raise SystemExit("--qualified-runtime requires a regular verification receipt")
    try:
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise SystemExit(f"invalid runtime verification receipt: {exc}") from exc
    if not isinstance(receipt, dict):
        raise SystemExit("runtime verification receipt must be a JSON object")
    checks = receipt.get("checks")
    try:
        active_lock = json.loads(lock_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise SystemExit(f"invalid active runtime lock: {exc}") from exc
    if not isinstance(active_lock, dict):
        raise SystemExit("active runtime lock must be a JSON object")
    expected_flags = {
        "fast": False,
        "skip_dynamic": False,
        "skip_installed_assets": False,
        "require_git_sonic": True,
    }
    expected_inventory = {
        "runtime_files_expected": len(active_lock["runtime_files"]),
        "runtime_files_checked": len(active_lock["runtime_files"]),
        "runtime_trees_expected": len(active_lock["runtime_trees"]),
        "runtime_trees_checked": len(active_lock["runtime_trees"]),
        "installed_files_expected": len(
            active_lock["matrix_release"]["installed_files"]
        ),
        "installed_files_checked": len(
            active_lock["matrix_release"]["installed_files"]
        ),
        "installed_trees_expected": len(
            active_lock["matrix_release"]["installed_trees"]
        ),
        "installed_trees_checked": len(
            active_lock["matrix_release"]["installed_trees"]
        ),
        "dynamic_checks_performed": True,
    }
    core_required_checks = {
        "Matrix source commit",
        "Matrix tracked source clean",
        "Matrix ignored source overlays absent",
        "native runtime Python",
        "native runtime Python prefix",
        "native runtime Python isolation",
        "native SONIC source clean",
        "native SONIC ignored source overlays absent",
        "native SONIC Git checkout required",
        "native SONIC commit",
        "native SONIC Python API",
        "gear_sonic import origin",
        "SONIC deploy dependency closure",
        "Matrix UE dependency closure",
        "TensorRT ABI",
    }
    if args.control_source == "pico":
        core_required_checks.update(
            {
                "native PICO wheel artifact",
                "native PICO Python isolation",
                "native PICO SDK wheel installation",
                "native PICO Python API",
                "native PICO pip check",
            }
        )
    receipt_required_checks = receipt.get("qualification_required_checks")
    check_names = {
        str(item.get("name"))
        for item in checks or []
        if isinstance(item, dict)
    }
    receipt_inventory_complete = (
        isinstance(receipt_required_checks, list)
        and all(isinstance(name, str) for name in receipt_required_checks)
        and core_required_checks.issubset(receipt_required_checks)
        and set(receipt_required_checks).issubset(check_names)
    )
    runtime_root = Path(str(receipt.get("runtime_root", ""))).resolve()
    expected_roots = _expected_receipt_roots(
        args.qualification_profile, runtime_root, args.sonic_root.resolve()
    )
    expected_environment = {
        "ld_library_path": os.environ.get("LD_LIBRARY_PATH", ""),
        "pythonpath": os.environ.get("PYTHONPATH", ""),
        "tensorrt_root": os.environ.get("TensorRT_ROOT", ""),
        "python_pycache_prefix": os.environ.get("PYTHONPYCACHEPREFIX", ""),
        "python_dont_write_bytecode": os.environ.get(
            "PYTHONDONTWRITEBYTECODE", ""
        ),
    }
    pico_identity_matches = (
        receipt.get("pico_python") is None and receipt.get("pico_wheel") is None
        if args.control_source != "pico"
        else (
            Path(str(receipt.get("pico_python", ""))).absolute()
            == Path(args.pico_python or "").expanduser().absolute()
            and bool(receipt.get("pico_wheel"))
        )
    )
    receipt_ok = (
        receipt.get("passed") is True
        and isinstance(checks, list)
        and bool(checks)
        and all(isinstance(item, dict) and item.get("ok") is True for item in checks)
        and receipt.get("profile") == args.qualification_profile
        and receipt.get("lock_sha256") == args.runtime_lock_sha256
        and Path(str(receipt.get("lock", ""))).resolve() == lock_path.resolve()
        and Path(str(receipt.get("matrix_root", ""))).resolve() == matrix_root
        and receipt.get("matrix_commit") == args.matrix_commit
        and Path(str(receipt.get("sonic_root", ""))).resolve()
        == args.sonic_root.resolve()
        and receipt.get("full_hashes") is True
        and receipt.get("sonic_git_checkout") is True
        and receipt.get("qualification_eligible") is True
        and receipt.get("verification_flags") == expected_flags
        and receipt.get("verification_inventory") == expected_inventory
        and receipt.get("missing_qualification_checks") == []
        and receipt_inventory_complete
        and Path(str(receipt.get("python", ""))).absolute()
        == Path(sys.executable).absolute()
        == (matrix_root / ".venv-audit/bin/python").absolute()
        and Path(str(receipt.get("python_prefix", ""))).absolute()
        == Path(sys.prefix).absolute()
        == (matrix_root / ".venv-audit").absolute()
        and receipt.get("launch_roots") == expected_roots
        and receipt.get("launch_environment") == expected_environment
        and pico_identity_matches
    )
    if not receipt_ok:
        raise SystemExit("runtime verification receipt does not match this launch")
    args.verification_receipt = receipt_path.resolve()
    args.verification_receipt_sha256 = _sha256_file(args.verification_receipt)
    return receipt


def _sha256_tree(root: Path) -> str:
    if not root.is_dir() or root.is_symlink():
        raise SystemExit(f"qualified runtime requires a regular tree: {root}")
    digest = hashlib.sha256()
    paths = sorted(root.rglob("*"))
    if any(path.is_symlink() for path in paths):
        raise SystemExit(f"qualified runtime tree contains a symlink: {root}")
    for path in (item for item in paths if item.is_file()):
        digest.update(path.relative_to(root).as_posix().encode("utf-8"))
        digest.update(b"\0")
        digest.update(_sha256_file(path).encode("ascii"))
        digest.update(b"\n")
    return digest.hexdigest()


def _regular_json(path: Path, label: str) -> dict[str, Any]:
    if not path.is_file() or path.is_symlink():
        raise SystemExit(f"qualified runtime requires regular {label}: {path}")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise SystemExit(f"invalid {label} {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise SystemExit(f"invalid {label} object: {path}")
    return payload


def _validate_qualified_model(
    args: argparse.Namespace,
    model_path: Path,
    receipt: dict[str, Any] | None,
) -> dict[str, object]:
    model_sha256 = _sha256_file(model_path)
    manifest_path = model_path.parent / "manifest.json"
    basic = {
        "model_sha256": model_sha256,
        "model_manifest": str(manifest_path.resolve()) if manifest_path.is_file() else None,
        "model_manifest_sha256": (
            _sha256_file(manifest_path) if manifest_path.is_file() else None
        ),
        "model_reproduced_from_locked_inputs": False,
    }
    if not args.qualified_runtime:
        return basic
    if receipt is None:
        raise SystemExit("qualified runtime is missing its verifier receipt")

    matrix_root = Path(__file__).resolve().parents[1]
    lock = _regular_json(
        matrix_root / "config/runtime/matrix-sonic.lock.json", "runtime lock"
    )
    manifest = _regular_json(manifest_path, "physics model manifest")
    launch_roots = receipt["launch_roots"]
    canonical_model = Path(str(manifest.get("canonical_model", ""))).resolve()
    canonical_meshes = Path(str(manifest.get("canonical_meshes", ""))).resolve()
    if canonical_model != Path(launch_roots["canonical_model"]):
        raise SystemExit("qualified model canonical path does not match receipt")
    if canonical_meshes != Path(launch_roots["canonical_meshes"]):
        raise SystemExit("qualified model mesh path does not match receipt")

    runtime_files = {
        (entry["root"], entry["path"]): entry["sha256"]
        for entry in lock["runtime_files"]
    }
    runtime_trees = {
        (entry["root"], entry["path"]): entry["sha256"]
        for entry in lock["runtime_trees"]
    }
    canonical_model_relative = (
        "gear_sonic/data/robot_model/model_data/g1/g1_29dof_with_hand.xml"
    )
    canonical_meshes_relative = (
        "gear_sonic/data/robot_model/model_data/g1/meshes"
    )
    expected_model_sha = runtime_files[("sonic", canonical_model_relative)]
    expected_meshes_sha = runtime_trees[("sonic", canonical_meshes_relative)]
    if (
        manifest.get("canonical_model_sha256") != expected_model_sha
        or _sha256_file(canonical_model) != expected_model_sha
    ):
        raise SystemExit("qualified model canonical SHA does not match runtime lock")
    if (
        manifest.get("canonical_meshes_sha256") != expected_meshes_sha
        or _sha256_tree(canonical_meshes) != expected_meshes_sha
    ):
        raise SystemExit("qualified model mesh tree does not match runtime lock")

    native_scene = Path(str(manifest.get("native_scene", ""))).resolve()
    installed_files = {
        (matrix_root / entry["path"]).resolve(): entry
        for entry in lock["matrix_release"]["installed_files"]
    }
    installed_trees = {
        (matrix_root / entry["path"]).resolve(): entry
        for entry in lock["matrix_release"]["installed_trees"]
    }
    from prepare_sonic_physics_model import (
        G1_BODY_JOINT_NAMES,
        SonicPhysicsModelError,
        _bundle_sha256,
        _native_scene_asset_inventory,
        prepare_sonic_physics_model,
    )

    try:
        active_scene_assets = _native_scene_asset_inventory(native_scene)
    except SonicPhysicsModelError as exc:
        raise SystemExit(f"qualified native-scene asset inventory failed: {exc}") from exc
    if manifest.get("native_scene_assets") != active_scene_assets:
        raise SystemExit("qualified native-scene asset inventory is stale")
    if args.scenario_layout_sha256 is None:
        if manifest.get("spawn_xyz") is not None or manifest.get("spawn_yaw_rad") is not None:
            raise SystemExit("qualified Matrix native scene cannot override spawn")
        native_entry = installed_files.get(native_scene)
        if native_entry is None:
            raise SystemExit("qualified model uses an unlocked Matrix native scene")
        if (
            manifest.get("native_scene_sha256") != native_entry["sha256"]
            or _sha256_file(native_scene) != native_entry["sha256"]
        ):
            raise SystemExit("qualified Matrix native scene SHA does not match lock")
        native_assets = Path(str(manifest.get("native_assets", ""))).resolve()
        assets_entry = installed_trees.get(native_assets)
        if assets_entry is None or (
            manifest.get("native_assets_sha256") != assets_entry["sha256"]
            or _sha256_tree(native_assets) != assets_entry["sha256"]
        ):
            raise SystemExit("qualified Matrix native asset tree does not match lock")
        for asset in active_scene_assets:
            source = Path(str(asset["path"])).resolve()
            try:
                inside_assets = source.is_relative_to(native_assets)
            except (OSError, RuntimeError):
                inside_assets = False
            if not inside_assets:
                source_entry = installed_files.get(source)
                if source_entry is None or (
                    asset.get("sha256") != source_entry["sha256"]
                    or asset.get("size") != source_entry["size"]
                ):
                    raise SystemExit(
                        "qualified Matrix scene uses an unlocked sibling asset"
                    )
        reproduction_native_scene = native_scene
    else:
        layout_path = matrix_root / "research/overworld_v1/layout.json"
        if _sha256_file(layout_path) != args.scenario_layout_sha256:
            raise SystemExit("qualified Overworld layout does not match active source")
        layout = _regular_json(layout_path, "Overworld layout")
        expected_spawn = layout["acceptance"]["spawn_xyz"]
        expected_yaw = layout["acceptance"]["spawn_yaw_rad"]
        if (
            manifest.get("spawn_xyz") != expected_spawn
            or manifest.get("spawn_yaw_rad") != expected_yaw
        ):
            raise SystemExit("qualified Overworld spawn does not match layout")
        composed_manifest_path = native_scene.parent / "manifest.json"
        composed = _regular_json(composed_manifest_path, "Overworld manifest")
        if (
            composed.get("layout_sha256") != args.scenario_layout_sha256
            or Path(str(composed.get("output_scene", ""))).resolve() != native_scene
            or composed.get("output_scene_sha256") != _sha256_file(native_scene)
            or manifest.get("native_scene_sha256")
            != composed.get("output_scene_sha256")
        ):
            raise SystemExit("qualified Overworld scene manifest does not match output")
        composed_scenes = composed.get("scenes")
        if not isinstance(composed_scenes, list) or len(composed_scenes) != len(
            layout["scenes"]
        ):
            raise SystemExit("qualified Overworld source-scene inventory is incomplete")
        for scene in composed_scenes:
            if not isinstance(scene, dict):
                raise SystemExit("qualified Overworld source-scene entry is invalid")
            source = Path(str(scene.get("source_scene", ""))).resolve()
            entry = installed_files.get(source)
            if entry is None or (
                scene.get("source_sha256") != entry["sha256"]
                or _sha256_file(source) != entry["sha256"]
            ):
                raise SystemExit("qualified Overworld contains an unlocked source scene")
            source_assets = scene.get("source_assets")
            if not isinstance(source_assets, list):
                raise SystemExit(
                    "qualified Overworld source-asset inventory is incomplete"
                )
            locked_asset_root = (
                matrix_root / "src/robot_mujoco/zsibot_robots/xgb/assets"
            ).resolve()
            locked_asset_entry = installed_trees.get(locked_asset_root)
            if locked_asset_entry is None or (
                _sha256_tree(locked_asset_root) != locked_asset_entry["sha256"]
            ):
                raise SystemExit("qualified Matrix source asset tree is not locked")
            for asset in source_assets:
                if not isinstance(asset, dict):
                    raise SystemExit("qualified Overworld source asset is invalid")
                source_asset = Path(str(asset.get("path", ""))).resolve()
                if (
                    not source_asset.is_file()
                    or source_asset.is_symlink()
                    or asset.get("sha256") != _sha256_file(source_asset)
                    or asset.get("size") != source_asset.stat().st_size
                ):
                    raise SystemExit("qualified Overworld source asset is stale")
                if not source_asset.is_relative_to(locked_asset_root):
                    source_entry = installed_files.get(source_asset)
                    if source_entry is None or (
                        source_entry["sha256"] != asset.get("sha256")
                        or source_entry["size"] != asset.get("size")
                    ):
                        raise SystemExit(
                            "qualified Overworld contains an unlocked sibling asset"
                        )
        native_assets = native_scene.parent / "assets"
        if (
            Path(str(manifest.get("native_assets", ""))).resolve() != native_assets
            or
            composed.get("output_assets_sha256") != _sha256_tree(native_assets)
            or manifest.get("native_assets_sha256")
            != composed.get("output_assets_sha256")
        ):
            raise SystemExit("qualified Overworld asset tree does not match manifest")

        from compose_overworld_scene import compose_overworld_scene

        with tempfile.TemporaryDirectory(prefix="matrix-overworld-recheck.") as temporary:
            reproduction_native_scene = Path(temporary) / native_scene.name
            reproduced_composed = compose_overworld_scene(
                layout_path,
                matrix_root / "src/robot_mujoco/zsibot_robots/xgb",
                reproduction_native_scene,
            )
            if (
                _sha256_file(reproduction_native_scene) != _sha256_file(native_scene)
                or _sha256_tree(reproduction_native_scene.parent / "assets")
                != _sha256_tree(native_assets)
            ):
                raise SystemExit("qualified Overworld scene is not reproducible")
            if reproduced_composed.get("scenes") != composed_scenes:
                raise SystemExit(
                    "qualified Overworld source inventory is not reproducible"
                )
            reproduction_native_files = {
                path.relative_to(reproduction_native_scene.parent): path.read_bytes()
                for path in reproduction_native_scene.parent.rglob("*")
                if path.is_file() and path.name != "manifest.json"
            }
        # The temporary composition is removed above; reconstruct it below in
        # the model-reproduction temporary directory from the verified bytes.

    if manifest.get("body_joint_names") != list(G1_BODY_JOINT_NAMES):
        raise SystemExit("qualified model body-joint contract is not canonical")
    spawn_xyz_value = manifest.get("spawn_xyz")
    spawn_xyz = tuple(spawn_xyz_value) if spawn_xyz_value is not None else None
    spawn_yaw = manifest.get("spawn_yaw_rad")
    with tempfile.TemporaryDirectory(prefix="matrix-sonic-model-recheck.") as temporary:
        temporary_root = Path(temporary)
        if args.scenario_layout_sha256 is not None:
            reproduction_native_root = temporary_root / "native"
            for relative, content in reproduction_native_files.items():
                target = reproduction_native_root / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(content)
            reproduction_native_scene = reproduction_native_root / native_scene.name
        expected_output = temporary_root / "sonic"
        expected_scene = prepare_sonic_physics_model(
            canonical_model,
            canonical_meshes,
            reproduction_native_scene,
            expected_output,
            spawn_xyz=spawn_xyz,
            spawn_yaw=spawn_yaw,
        )
        actual_robot = model_path.parent / "robot.xml"
        actual_meshes = model_path.parent / "meshes"
        if not actual_robot.is_file() or actual_robot.is_symlink() or not actual_meshes.is_dir():
            raise SystemExit("qualified physics model bundle is incomplete")
        if (
            _sha256_file(expected_scene) != model_sha256
            or _sha256_file(expected_output / "robot.xml")
            != _sha256_file(actual_robot)
            or _sha256_tree(expected_output / "meshes")
            != _sha256_tree(actual_meshes)
            or _bundle_sha256(expected_output)
            != _bundle_sha256(model_path.parent)
        ):
            raise SystemExit("qualified physics model is not reproducible")
    if (
        manifest.get("derived_scene_sha256") != model_sha256
        or manifest.get("derived_robot_sha256") != _sha256_file(model_path.parent / "robot.xml")
        or manifest.get("derived_meshes_sha256") != _sha256_tree(model_path.parent / "meshes")
        or manifest.get("derived_bundle_sha256")
        != _bundle_sha256(model_path.parent)
    ):
        raise SystemExit("qualified physics model manifest has stale derived hashes")
    basic["model_reproduced_from_locked_inputs"] = True
    return basic


def _validate_qualified_acceptance(args: argparse.Namespace) -> None:
    """Reject any bounded gate that is weaker than the active runtime lock."""
    if not args.qualified_runtime:
        return
    lock_path = Path(__file__).resolve().parents[1] / "config/runtime/matrix-sonic.lock.json"
    try:
        acceptance = json.loads(lock_path.read_text(encoding="utf-8"))["acceptance"]
    except (OSError, UnicodeError, json.JSONDecodeError, KeyError, TypeError) as exc:
        raise SystemExit(f"cannot read qualified acceptance lock: {exc}") from exc

    weaker: list[str] = []
    minimums = (
        ("min_active_seconds", "active_lowcmd_seconds_min"),
        ("min_displacement_m", "root_displacement_xy_min_m"),
        ("min_physics_hz", "physics_hz_min"),
        ("min_rtf", "rtf_min"),
    )
    for argument, lock_key in minimums:
        actual = float(getattr(args, argument))
        expected = float(acceptance[lock_key])
        if not math.isfinite(actual) or actual + 1e-12 < expected:
            weaker.append(f"{argument}={actual!r} < lock {lock_key}={expected!r}")

    actual_freshness = float(args.low_cmd_fresh_timeout_seconds)
    locked_freshness = float(acceptance["low_cmd_fresh_timeout_seconds"])
    if (
        not math.isfinite(actual_freshness)
        or actual_freshness <= 0.0
        or actual_freshness - 1e-12 > locked_freshness
    ):
        weaker.append(
            "low_cmd_fresh_timeout_seconds="
            f"{actual_freshness!r} > lock={locked_freshness!r}"
        )

    locked_resets = int(acceptance["instability_resets_max"])
    if args.max_resets > locked_resets:
        weaker.append(f"max_resets={args.max_resets} > lock={locked_resets}")
    if acceptance["fall_detected"] is False and not args.fail_on_fall:
        weaker.append("fail_on_fall=false while lock requires no detected fall")
    if weaker:
        raise SystemExit(
            "qualified acceptance gates cannot weaken the runtime lock:\n  "
            + "\n  ".join(weaker)
        )


def _validate_qualified_game_control(args: argparse.Namespace) -> None:
    """Reject game-control qualification paths that bypass real camera input."""

    if not args.qualified_runtime or args.control_source != "game":
        return
    if args.no_game_input_provider:
        raise SystemExit(
            "qualified game control requires the supervised input provider"
        )
    if args.game_camera_yaw_source == "fixed":
        raise SystemExit(
            "qualified game control rejects an unobserved fixed camera yaw"
        )
    if args.game_camera_yaw_source in {"x11-core-gated", "x11-absolute"}:
        raise SystemExit(
            "qualified game control rejects experimental camera yaw sources"
        )
    if (
        args.game_camera_yaw_source
        in {"x11-mirror", "x11-core-gated", "x11-absolute"}
        and args.game_mouse_sensitivity_deg <= 0.0
    ):
        raise SystemExit(
            "qualified X11 camera control requires positive mouse sensitivity"
        )
    expected_provider = (_SCRIPT_DIR / "matrix_game_control_input.py").resolve()
    try:
        actual_provider = args.game_input_provider.resolve(strict=True)
    except OSError as exc:
        raise SystemExit(f"qualified game input provider is unavailable: {exc}") from exc
    if actual_provider != expected_provider:
        raise SystemExit(
            "qualified game control requires the bundled input provider: "
            f"expected={expected_provider} actual={actual_provider}"
        )
    expected_python = os.path.abspath(sys.executable)
    actual_python = os.path.abspath(os.fspath(args.game_input_provider_python))
    if actual_python != expected_python:
        raise SystemExit(
            "qualified game control requires the verified runtime Python for "
            f"its input provider: expected={expected_python} actual={actual_python}"
        )


def _root_up_z(qpos) -> float:
    """Diagnostic world-Z component of the floating base's local up axis."""
    _, x, y, _ = [float(value) for value in qpos[3:7]]
    return 1.0 - 2.0 * (x * x + y * y)


def _root_yaw_rad(qpos) -> float:
    """Return normalized floating-base yaw from MuJoCo's wxyz quaternion."""

    try:
        w, x, y, z = [float(value) for value in qpos[3:7]]
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("root quaternion must contain four finite numbers") from exc
    norm = math.sqrt((w * w) + (x * x) + (y * y) + (z * z))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("root quaternion has zero or non-finite norm")
    w, x, y, z = (value / norm for value in (w, x, y, z))
    sine = 2.0 * ((w * z) + (x * y))
    cosine = 1.0 - (2.0 * ((y * y) + (z * z)))
    return math.atan2(sine, cosine)


class _HeadingAnchorTelemetry:
    """Audit the startup yaw without changing the game-control frame.

    Native SONIC initializes its planner heading before Matrix can observe the
    first fresh LowCmd.  Matrix therefore keeps the initial MuJoCo snapshot as
    its command-frame anchor and records the first freshness edge only as
    evidence.  In particular, :meth:`observe` never calls the control core and
    never changes ``root_yaw_initial_rad``.
    """

    def __init__(self, initial_root_yaw_rad: float, initial_snapshot: Any) -> None:
        self.root_yaw_initial_rad = float(initial_root_yaw_rad)
        self.root_yaw_first_fresh_lowcmd_rad: float | None = None
        self.root_yaw_startup_delta_rad: float | None = None
        self.first_fresh_lowcmd_step_index: int | None = None
        self.first_fresh_lowcmd_sim_time_s: float | None = None
        self.first_fresh_lowcmd_wall_elapsed_s: float | None = None
        self._previous_low_cmd_fresh = bool(initial_snapshot.low_cmd_fresh)

        # A simulator reused in the same DDS domain may already report a
        # fresh command in the initial snapshot.  Record that state explicitly
        # instead of waiting for a false->true edge that may never occur.
        if self._previous_low_cmd_fresh:
            self._capture(
                initial_snapshot,
                root_yaw_rad=self.root_yaw_initial_rad,
                wall_elapsed_s=0.0,
            )

    def _capture(
        self,
        snapshot: Any,
        *,
        root_yaw_rad: float,
        wall_elapsed_s: float,
    ) -> None:
        if self.root_yaw_first_fresh_lowcmd_rad is not None:
            return
        yaw = float(root_yaw_rad)
        elapsed = float(wall_elapsed_s)
        if not math.isfinite(yaw):
            raise ValueError("first fresh LowCmd root yaw must be finite")
        if not math.isfinite(elapsed) or elapsed < 0.0:
            raise ValueError("first fresh LowCmd wall elapsed must be nonnegative")
        self.root_yaw_first_fresh_lowcmd_rad = yaw
        self.root_yaw_startup_delta_rad = wrap_angle_rad(
            yaw - self.root_yaw_initial_rad
        )
        self.first_fresh_lowcmd_step_index = int(snapshot.step_index)
        self.first_fresh_lowcmd_sim_time_s = float(snapshot.sim_time)
        self.first_fresh_lowcmd_wall_elapsed_s = elapsed

    def observe(self, snapshot: Any, *, wall_elapsed_s: float) -> bool:
        """Capture exactly the first observed false-to-true freshness edge."""

        fresh = bool(snapshot.low_cmd_fresh)
        captured = False
        if (
            self.root_yaw_first_fresh_lowcmd_rad is None
            and fresh
            and not self._previous_low_cmd_fresh
        ):
            self._capture(
                snapshot,
                root_yaw_rad=_root_yaw_rad(snapshot.qpos),
                wall_elapsed_s=wall_elapsed_s,
            )
            captured = True
        self._previous_low_cmd_fresh = fresh
        return captured

    def status_fields(self) -> dict[str, Any]:
        """Return stable JSON fields for periodic and final status payloads."""

        return {
            "heading_anchor_source": "initial_snapshot",
            "root_yaw_initial_rad": round(self.root_yaw_initial_rad, 6),
            "root_yaw_first_fresh_lowcmd_rad": (
                round(self.root_yaw_first_fresh_lowcmd_rad, 6)
                if self.root_yaw_first_fresh_lowcmd_rad is not None
                else None
            ),
            "root_yaw_startup_delta_rad": (
                round(self.root_yaw_startup_delta_rad, 6)
                if self.root_yaw_startup_delta_rad is not None
                else None
            ),
            "first_fresh_lowcmd_step_index": self.first_fresh_lowcmd_step_index,
            "first_fresh_lowcmd_sim_time_s": (
                round(self.first_fresh_lowcmd_sim_time_s, 6)
                if self.first_fresh_lowcmd_sim_time_s is not None
                else None
            ),
            "first_fresh_lowcmd_wall_elapsed_s": (
                round(self.first_fresh_lowcmd_wall_elapsed_s, 6)
                if self.first_fresh_lowcmd_wall_elapsed_s is not None
                else None
            ),
        }


class _GameSonicReadinessGate:
    """Keep interactive motion stopped until native SONIC is ready.

    The input core owns neutral re-arming, while this gate owns readiness of
    the native deploy path.  ``begin_frame`` deliberately runs before input is
    drained: invalidating first means a key that remains held across a LowCmd
    outage cannot clear the re-arm latch on the recovery frame.
    """

    ELASTIC_BAND_ZERO_ABS_TOL = 1e-6

    def __init__(self, initial_snapshot: Any) -> None:
        initial_fresh = getattr(initial_snapshot, "low_cmd_fresh", False)
        self._previous_low_cmd_fresh = (
            initial_fresh if type(initial_fresh) is bool else False
        )
        self._ready = False
        self._stop_facing = (1.0, 0.0, 0.0)

    @classmethod
    def snapshot_ready(cls, snapshot: Any) -> bool:
        if type(getattr(snapshot, "low_cmd_fresh", None)) is not bool:
            return False
        elastic_band_scale = getattr(snapshot, "elastic_band_scale", None)
        if type(elastic_band_scale) is not float:
            return False
        return snapshot.low_cmd_fresh and math.isfinite(
            elastic_band_scale
        ) and math.isclose(
            elastic_band_scale,
            0.0,
            rel_tol=0.0,
            abs_tol=cls.ELASTIC_BAND_ZERO_ABS_TOL,
        )

    def begin_frame(self, snapshot: Any, core: GameControlCore) -> bool:
        """Invalidate unsafe input before polling and return readiness."""

        fresh_value = getattr(snapshot, "low_cmd_fresh", False)
        fresh = fresh_value if type(fresh_value) is bool else False
        self._ready = self.snapshot_ready(snapshot)
        if not self._ready:
            reason = (
                "low_cmd_stale"
                if self._previous_low_cmd_fresh and not fresh
                else "sonic_not_ready"
            )
            # Repeat invalidation while SONIC is unavailable.  This prevents a
            # neutral packet observed during startup from arming a key that is
            # pressed before the elastic band has fully released.
            core.invalidate_input(reason)
            # Materialize the core's safety stop before a newly drained
            # neutral packet can clear the re-arm latch. Besides hard-zeroing
            # speed, this absorbs measured yaw so IDLE cannot finish an old
            # turn while LowCmd or the startup restraint is unavailable.
            stopped = core.command(now_s=0.0, dt_s=0.0)
            self._stop_facing = stopped.facing
        self._previous_low_cmd_fresh = fresh
        return self._ready

    def apply(
        self,
        command: RobotMotionCommand,
        core: GameControlCore,
    ) -> RobotMotionCommand:
        """Return ``command`` only when SONIC can accept interactive motion.

        The second invalidation is intentionally after the input drain. A
        neutral packet followed by a held-key packet can arrive in the same
        batch; neither may leave the neutral-rearm latch cleared while native
        readiness is false.
        """

        if not isinstance(command, RobotMotionCommand):
            raise TypeError("command must be a RobotMotionCommand")
        if not isinstance(core, GameControlCore):
            raise TypeError("core must be a GameControlCore")
        if self._ready:
            return command
        core.invalidate_input("sonic_not_ready")
        stopped = core.command(now_s=0.0, dt_s=0.0)
        self._stop_facing = stopped.facing
        return RobotMotionCommand(
            sequence=command.sequence,
            movement=(0.0, 0.0, 0.0),
            facing=stopped.facing,
            speed_mps=0.0,
            locomotion_mode=SONIC_IDLE_MODE,
            mode="deadman",
            safe_stop=True,
            reason="sonic_not_ready",
        )


def _pace_absolute_deadline(deadline_s: float, period_s: float) -> float:
    """Wait for one absolute tick and return the following tick deadline.

    A relative ``sleep(period - work)`` accumulates scheduler overshoot on
    every 5 ms SONIC step. Keeping the deadline on an absolute timeline lets
    the following sleep compensate for that overshoot while still resetting
    after a sustained overrun instead of emitting a catch-up burst.
    """
    now = time.perf_counter()
    remaining_s = deadline_s - now
    if remaining_s > 0.0:
        time.sleep(remaining_s)
        now = time.perf_counter()
    if now - deadline_s > 2.0 * period_s:
        return now + period_s
    return deadline_s + period_s


def _acceptance_failures(
    *,
    unstable: bool,
    fall_detected: bool,
    fail_on_fall: bool,
    active_lowcmd: bool,
    active_elapsed_s: float,
    min_active_seconds: float,
    physics_step_hz: float,
    min_physics_hz: float,
    rtf: float,
    min_rtf: float,
    failed_child: tuple[str, int] | None = None,
    root_displacement_xy_m: float = 0.0,
    min_displacement_m: float = 0.0,
    root_final_x: float = 0.0,
    min_final_x: float | None = None,
    root_displacement_x_m: float = 0.0,
    min_forward_x_m: float = 0.0,
    reset_count: int = 0,
    max_resets: int = 0,
) -> list[str]:
    failures = []
    if unstable:
        failures.append("numerical_instability")
    if failed_child is not None:
        name, code = failed_child
        failures.append(f"native_child_exit:{name}:{code}")
    if fail_on_fall and fall_detected:
        failures.append("fall_detected")
    if min_active_seconds > 0.0 and not active_lowcmd:
        failures.append("lowcmd_not_fresh_at_exit")
    if active_elapsed_s + 1e-6 < min_active_seconds:
        failures.append(
            f"active_lowcmd_too_short:{active_elapsed_s:.3f}<{min_active_seconds:.3f}"
        )
    if min_displacement_m > 0.0 and (
        not math.isfinite(root_displacement_xy_m)
        or root_displacement_xy_m + 1e-6 < min_displacement_m
    ):
        failures.append(
            "root_displacement_too_small:"
            f"{root_displacement_xy_m:.3f}<{min_displacement_m:.3f}"
        )
    if min_final_x is not None and (
        not math.isfinite(root_final_x) or root_final_x + 1e-6 < min_final_x
    ):
        failures.append(f"final_x_too_small:{root_final_x:.3f}<{min_final_x:.3f}")
    if min_forward_x_m > 0.0 and (
        not math.isfinite(root_displacement_x_m)
        or root_displacement_x_m + 1e-6 < min_forward_x_m
    ):
        failures.append(
            "forward_x_too_small:"
            f"{root_displacement_x_m:.3f}<{min_forward_x_m:.3f}"
        )
    if reset_count > max_resets:
        failures.append(f"reset_count_exceeded:{reset_count}>{max_resets}")
    if min_physics_hz > 0.0 and (
        not math.isfinite(physics_step_hz)
        or physics_step_hz + 1e-6 < min_physics_hz
    ):
        failures.append(
            f"physics_hz_too_low:{physics_step_hz:.3f}<{min_physics_hz:.3f}"
        )
    if min_rtf > 0.0 and (not math.isfinite(rtf) or rtf + 1e-6 < min_rtf):
        failures.append(f"rtf_too_low:{rtf:.4f}<{min_rtf:.4f}")
    return failures


def _game_input_acceptance_failures(
    *,
    accepted_connections: int,
    packets_applied: int,
    moving_command_frames: int,
    protocol_errors: int,
    rejected_packets: int,
    peer_pid_mismatches: int,
    connected_at_boundary: bool,
    input_age_s: float | None,
    maximum_boundary_age_s: float,
    safe_stop_at_boundary: bool,
) -> list[str]:
    """Qualified game runs require a clean, exercised operator input path."""

    failures = []
    if accepted_connections < 1:
        failures.append("game_input_no_connection")
    if packets_applied < 1:
        failures.append("game_input_no_applied_packets")
    if moving_command_frames < 1:
        failures.append("game_input_no_moving_command_frames")
    if protocol_errors:
        failures.append(f"game_input_protocol_errors:{protocol_errors}")
    if rejected_packets:
        failures.append(f"game_input_rejected_packets:{rejected_packets}")
    if peer_pid_mismatches:
        failures.append(f"game_input_peer_pid_mismatches:{peer_pid_mismatches}")
    if not connected_at_boundary:
        failures.append("game_input_disconnected_at_boundary")
    if input_age_s is None or input_age_s > maximum_boundary_age_s:
        failures.append("game_input_stale_at_boundary")
    if safe_stop_at_boundary:
        failures.append("game_input_safe_stop_at_boundary")
    return failures


def _game_control_status_fields(args: argparse.Namespace) -> dict[str, object]:
    """Return the immutable input/camera claim carried by every status frame."""

    source = args.game_camera_yaw_source
    if source == "fixed":
        yaw_observation = "constant_unobserved"
        yaw_truth_scope = "configured_constant_not_final_view"
        button_gate_truth_scope = "no_button_gate"
    elif source == "x11-mirror":
        yaw_observation = "xinput2_raw_motion_mirror"
        yaw_truth_scope = "xi2_raw_input_mirror_not_final_view"
        button_gate_truth_scope = "xi2_raw_button_edges_same_slave_source"
    elif source == "x11-core-gated":
        yaw_observation = "xinput2_raw_motion_core_button_level_gate"
        yaw_truth_scope = "xi2_raw_motion_core_button_gate_not_final_view"
        button_gate_truth_scope = (
            "xquerypointer_core_button_level_sampled_not_event_ordered"
        )
    elif source == "x11-absolute":
        yaw_observation = "xquerypointer_root_absolute_delta"
        yaw_truth_scope = "x11_absolute_pointer_delta_mirror_not_final_view"
        button_gate_truth_scope = "xquerypointer_core_level_sampled_at_50hz"
    else:
        yaw_observation = "carla_spectator_rpc_write_readback"
        yaw_truth_scope = "carla_spectator_not_verified_final_view"
        button_gate_truth_scope = "not_applicable_carla_rpc"
    effective_input_source = args.game_input_source
    if source != "carla" and effective_input_source == "auto":
        effective_input_source = "keyboard"
    applied_mouse_scale = args.game_applied_mouse_speed_scale
    effective_mouse_sensitivity = (
        args.game_mouse_sensitivity_deg * applied_mouse_scale
    )
    if source in {"x11-mirror", "x11-core-gated"}:
        sensitivity_units = "degrees_per_xi2_raw_unit"
    elif source == "x11-absolute":
        sensitivity_units = "degrees_per_x11_root_pixel"
    else:
        sensitivity_units = "degrees_per_unobserved_input_unit"
    return {
        "input_protocol": PROTOCOL_NAME,
        "input_source_requested": args.game_input_source,
        "input_source_effective": effective_input_source,
        "native_gait": "IDLE/SLOW_WALK/WALK/RUN selected by movement tier",
        "native_gait_modes": {
            SONIC_GAIT_NAMES[mode]: mode for mode in sorted(SONIC_GAIT_NAMES)
        },
        "keyboard_slow_speed_mps": KEYBOARD_GAIT_TARGETS_MPS[
            SONIC_SLOW_WALK_MODE
        ],
        "keyboard_walk_speed_mps": KEYBOARD_GAIT_TARGETS_MPS[SONIC_WALK_MODE],
        "keyboard_run_speed_mps": KEYBOARD_GAIT_TARGETS_MPS[SONIC_RUN_MODE],
        # Preserve the historical status contract: maximum_speed_mps is the
        # configurable analog SLOW_WALK ceiling.  Keyboard tiers now have a
        # separate native RUN target and must not silently change that field.
        "maximum_speed_mps": args.game_max_speed,
        "analog_maximum_speed_mps": args.game_max_speed,
        "keyboard_maximum_target_speed_mps": KEYBOARD_GAIT_TARGETS_MPS[
            SONIC_RUN_MODE
        ],
        "maximum_acceleration_mps2": args.game_max_acceleration,
        "maximum_deceleration_mps2": args.game_max_deceleration,
        "maximum_turn_rate_rad_s": args.game_max_turn_rate,
        "stick_deadzone": args.game_stick_deadzone,
        "input_timeout_s": args.game_input_timeout,
        "maximum_snapshot_age_s": args.game_max_snapshot_age,
        "maximum_future_skew_s": args.game_max_future_skew,
        "camera_yaw_source": source,
        "camera_look_button": args.game_look_button,
        "focus_title_pattern": args.game_focus_title,
        "expected_ue_pid": args.ue_pid,
        "camera_yaw_observation": yaw_observation,
        "camera_yaw_truth_scope": yaw_truth_scope,
        "button_gate_truth_scope": button_gate_truth_scope,
        "legacy": source == "x11-absolute",
        "experimental": source in {"x11-core-gated", "x11-absolute"},
        "camera_yaw_sign": args.game_camera_yaw_sign,
        "camera_yaw_offset_deg": args.game_camera_yaw_offset_deg,
        "initial_camera_yaw_deg": args.game_initial_camera_yaw_deg,
        "visible_mouse_backend": "sdl-relative-speed-scale",
        "applied_mouse_profile": args.game_applied_mouse_profile,
        "applied_mouse_speed_scale": applied_mouse_scale,
        # Historical field retained as the unscaled/base calibration value.
        "mouse_sensitivity_deg_per_px": args.game_mouse_sensitivity_deg,
        "mouse_sensitivity_base_deg_per_px": args.game_mouse_sensitivity_deg,
        "mouse_sensitivity_effective_deg_per_px": effective_mouse_sensitivity,
        "mouse_sensitivity_units": sensitivity_units,
        "mouse_sensitivity_base_deg_per_unit": args.game_mouse_sensitivity_deg,
        "mouse_sensitivity_effective_deg_per_unit": (
            effective_mouse_sensitivity
        ),
        "mouse_sensitivity_base_deg_per_raw_unit": (
            args.game_mouse_sensitivity_deg
        ),
        "mouse_sensitivity_effective_deg_per_raw_unit": (
            effective_mouse_sensitivity
        ),
        "carla_host": args.game_carla_host,
        "carla_port": args.game_carla_port,
        "gamepad_look_yaw_rate_deg_s": args.gamepad_look_yaw_rate_deg_s,
        "gamepad_look_pitch_rate_deg_s": args.gamepad_look_pitch_rate_deg_s,
        "gamepad_look_deadzone": args.gamepad_look_deadzone,
        "gamepad_look_min_pitch_deg": args.gamepad_look_min_pitch_deg,
        "gamepad_look_max_pitch_deg": args.gamepad_look_max_pitch_deg,
        "carla_write_readback_tolerance_deg": 0.5,
        # Neither pointer integration nor a CARLA spectator transform proves
        # that the cooked Matrix follow camera rendered the same direction.
        # Qualified status therefore names its scope instead of over-claiming
        # a visual camera-relative acceptance result.
        "qualification_scope": "runtime_input_and_motion_path_only",
        "visible_follow_camera_verified": False,
        "external_visual_evidence_required": True,
    }


_EXPECTED_SNAPSHOT_DIMS = {
    "qpos": 36,
    "qvel": 35,
    "ctrl": 29,
    "applied_torque": 29,
}


def _snapshot_validation_error(snapshot, previous_snapshot=None) -> str | None:
    """Return a precise native snapshot invariant violation, if any."""
    for field, expected in _EXPECTED_SNAPSHOT_DIMS.items():
        values = getattr(snapshot, field, None)
        if values is None:
            return f"snapshot_missing_field:{field}"
        try:
            actual = len(values)
        except TypeError:
            return f"snapshot_field_not_sized:{field}"
        if actual != expected:
            return f"snapshot_dimension:{field}={actual},expected={expected}"

    try:
        step_index = int(snapshot.step_index)
    except (TypeError, ValueError, OverflowError):
        return f"snapshot_invalid_step_index:{snapshot.step_index!r}"
    if step_index != snapshot.step_index:
        return f"snapshot_invalid_step_index:{snapshot.step_index!r}"

    try:
        sim_time = float(snapshot.sim_time)
    except (TypeError, ValueError, OverflowError):
        return f"snapshot_invalid_sim_time:{snapshot.sim_time!r}"
    if not math.isfinite(sim_time):
        return f"snapshot_non_finite:sim_time={sim_time!r}"

    if type(getattr(snapshot, "fall_detected", None)) is not bool:
        return f"snapshot_invalid_fall_detected:{getattr(snapshot, 'fall_detected', None)!r}"
    if type(getattr(snapshot, "low_cmd_fresh", None)) is not bool:
        return (
            "snapshot_invalid_low_cmd_fresh:"
            f"{getattr(snapshot, 'low_cmd_fresh', None)!r}"
        )
    if type(getattr(snapshot, "low_cmd_received", None)) is not bool:
        return (
            "snapshot_invalid_low_cmd_received:"
            f"{getattr(snapshot, 'low_cmd_received', None)!r}"
        )
    low_cmd_age_s = getattr(snapshot, "low_cmd_age_s", None)
    if low_cmd_age_s is not None:
        try:
            low_cmd_age = float(low_cmd_age_s)
        except (TypeError, ValueError, OverflowError):
            low_cmd_age = math.nan
        if (
            isinstance(low_cmd_age_s, bool)
            or not math.isfinite(low_cmd_age)
            or low_cmd_age < 0.0
        ):
            return f"snapshot_invalid_low_cmd_age_s:{low_cmd_age_s!r}"
    elastic_band_scale = getattr(snapshot, "elastic_band_scale", None)
    if type(elastic_band_scale) is not float or not math.isfinite(
        elastic_band_scale
    ):
        return f"snapshot_invalid_elastic_band_scale:{elastic_band_scale!r}"
    try:
        reset_count = int(snapshot.reset_count)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return f"snapshot_invalid_reset_count:{getattr(snapshot, 'reset_count', None)!r}"
    if reset_count != snapshot.reset_count or reset_count < 0:
        return f"snapshot_invalid_reset_count:{snapshot.reset_count!r}"
    last_reset_reason = getattr(snapshot, "last_reset_reason", None)
    if last_reset_reason is not None and not isinstance(last_reset_reason, str):
        return f"snapshot_invalid_last_reset_reason:{last_reset_reason!r}"

    for field in _EXPECTED_SNAPSHOT_DIMS:
        for index, value in enumerate(getattr(snapshot, field)):
            try:
                finite = math.isfinite(float(value))
            except (TypeError, ValueError, OverflowError):
                finite = False
            if not finite:
                return f"snapshot_non_finite:{field}[{index}]={value!r}"

    if previous_snapshot is not None:
        expected_step_index = int(previous_snapshot.step_index) + 1
        if step_index != expected_step_index:
            return (
                "snapshot_step_index_not_sequential:"
                f"{step_index},expected={expected_step_index}"
            )
        previous_sim_time = float(previous_snapshot.sim_time)
        if sim_time <= previous_sim_time:
            return (
                "snapshot_sim_time_not_increasing:"
                f"{sim_time!r},previous={previous_sim_time!r}"
            )
        if reset_count < int(previous_snapshot.reset_count):
            return (
                "snapshot_reset_count_decreased:"
                f"{reset_count},previous={int(previous_snapshot.reset_count)}"
            )
    return None


class _QualificationState(TypedDict):
    acceptance_failures: list[str]
    qualification_attempted: bool
    completed: bool
    interrupted: bool
    passed: bool


def _qualification_state(
    *,
    max_seconds: float,
    termination_reason: str,
    failures: list[str],
    runtime_verified: bool,
) -> _QualificationState:
    """Classify a finalized run without treating operator stops as passes."""
    acceptance_failures = list(failures)
    attempted = max_seconds > 0.0
    if attempted and not runtime_verified:
        acceptance_failures.append("runtime_not_verified_for_qualification")
    if attempted and termination_reason == "signal":
        acceptance_failures.append("run_interrupted")
    if termination_reason == "unknown":
        acceptance_failures.append("unknown_termination")
    completed = termination_reason == "max_seconds"
    return {
        "acceptance_failures": acceptance_failures,
        "qualification_attempted": attempted,
        "completed": completed,
        "interrupted": termination_reason == "signal",
        "passed": completed and not acceptance_failures,
    }


def _configure_native_runtime(args: argparse.Namespace) -> Path:
    sonic_root = args.sonic_root.resolve()
    required = (
        sonic_root / "gear_sonic/scripts/run_sim_loop.py",
        sonic_root / "gear_sonic/utils/mujoco_sim/configs.py",
        sonic_root / "gear_sonic/utils/teleop/zmq/zmq_planner_sender.py",
        sonic_root / "gear_sonic_deploy/target/release/g1_deploy_onnx_ref",
        sonic_root / "gear_sonic_deploy/policy/release/observation_config.yaml",
        sonic_root / "gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/config/fastrtps_profile.xml",
    )
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise SystemExit(
            "Native SONIC checkout/runtime is incomplete:\n  " + "\n  ".join(missing)
        )
    motion_data = sonic_root / "gear_sonic_deploy/reference/example"
    if not motion_data.is_dir():
        raise SystemExit(f"Native SONIC motion data is missing: {motion_data}")
    sys.path.insert(0, str(sonic_root))
    return sonic_root


def _sonic_commit(sonic_root: Path) -> str:
    marker = sonic_root / "SONIC_COMMIT"
    if marker.is_file():
        return marker.read_text(encoding="utf-8").strip().splitlines()[0]
    result = subprocess.run(
        ["git", "-C", str(sonic_root), "rev-parse", "HEAD"],
        text=True,
        capture_output=True,
        check=False,
    )
    return result.stdout.strip() if result.returncode == 0 else "unknown"


def _native_config_kwargs(args: argparse.Namespace, model_path: Path) -> dict[str, Any]:
    release_enabled = True
    return {
        "interface": args.dds_interface,
        "sim_frequency": int(round(args.physics_hz)),
        "enable_onscreen": False,
        "enable_offscreen": False,
        "robot_scene": str(model_path),
        "elastic_band_release_enabled": release_enabled,
        "elastic_band_hold_seconds": args.startup_band_hold if args.startup_band else 0.0,
        "elastic_band_fade_seconds": args.startup_band_fade if args.startup_band else 0.0,
        "elastic_band_wait_for_lowcmd": bool(args.startup_band),
        "low_cmd_fresh_timeout_seconds": args.low_cmd_fresh_timeout_seconds,
        "with_hands": False,
        "reset_on_fall": False,
    }


def _loopback_zmq_port(endpoint: str) -> int:
    try:
        parsed = urlsplit(endpoint)
        port = parsed.port
    except ValueError as exc:
        raise ValueError(f"invalid planner endpoint: {endpoint}") from exc
    if parsed.scheme != "tcp" or parsed.path or parsed.query or parsed.fragment:
        raise ValueError(f"planner endpoint must be tcp://HOST:PORT: {endpoint}")
    if parsed.hostname not in {"127.0.0.1", "localhost", "::1"}:
        raise ValueError(f"planner endpoint must bind loopback: {endpoint}")
    if port is None or not 1 <= port <= 65535:
        raise ValueError(f"planner endpoint has invalid port: {endpoint}")
    return port


class NativePlannerClient:
    """Thin socket lifecycle around SONIC's canonical ZMQ wire builders."""

    def __init__(
        self,
        endpoint: str,
        *,
        zmq_module,
        build_command_message: Callable[..., bytes],
        build_planner_message: Callable[..., bytes],
    ) -> None:
        self._build_command_message = build_command_message
        self._build_planner_message = build_planner_message
        self._context = zmq_module.Context.instance()
        self._socket = self._context.socket(zmq_module.PUB)
        try:
            self._socket.setsockopt(zmq_module.LINGER, 0)
            self._socket.bind(endpoint)
        except Exception:
            self._socket.close(linger=0)
            raise
        self._heading: float | None = None

    @staticmethod
    def _wrap_angle(value: float) -> float:
        return math.atan2(math.sin(value), math.cos(value))

    def send_velocity(
        self,
        vx: float,
        vy: float,
        yaw_rate: float,
        *,
        dt: float,
        start: bool = True,
    ) -> None:
        if self._heading is None:
            # SONIC normalizes the planner context to a zero-yaw frame. Keep
            # this command frame independent from a Matrix XML world spawn.
            self._heading = 0.0
        self._heading = self._wrap_angle(self._heading + yaw_rate * dt)
        speed = math.hypot(vx, vy)
        moving = speed > 1e-6
        if moving:
            local_x = vx / speed
            local_y = vy / speed
            cosine = math.cos(self._heading)
            sine = math.sin(self._heading)
            movement = [
                cosine * local_x - sine * local_y,
                sine * local_x + cosine * local_y,
                0.0,
            ]
        else:
            movement = [0.0, 0.0, 0.0]
        facing = [math.cos(self._heading), math.sin(self._heading), 0.0]
        self.send_direction(
            movement=movement,
            facing=facing,
            speed=speed if moving else 0.0,
            start=start,
        )

    def send_direction(
        self,
        *,
        movement,
        facing,
        speed: float,
        locomotion_mode: int = 2,
        start: bool = True,
    ) -> None:
        """Send an absolute planner direction in SONIC's normalized XY frame."""

        movement_values = [float(value) for value in movement]
        facing_values = [float(value) for value in facing]
        if len(movement_values) != 3 or len(facing_values) != 3:
            raise ValueError("movement and facing must both have length 3")
        if not all(math.isfinite(value) for value in (*movement_values, *facing_values)):
            raise ValueError("movement and facing must be finite")
        speed_value = float(speed)
        if not math.isfinite(speed_value) or speed_value < 0.0:
            raise ValueError("speed must be non-negative and finite")
        moving = speed_value > 1e-6 and math.hypot(
            movement_values[0], movement_values[1]
        ) > 1e-6
        if type(locomotion_mode) is not int or not 0 <= locomotion_mode <= 26:
            raise ValueError("locomotion_mode must be a native SONIC motion in [0, 26]")
        if moving and locomotion_mode == SONIC_IDLE_MODE:
            raise ValueError("moving planner command cannot use native IDLE")
        self._socket.send(
            self._build_command_message(
                start=start,
                stop=False,
                planner=True,
                delta_heading=None,
            )
        )
        self._socket.send(
            self._build_planner_message(
                mode=locomotion_mode if moving else 0,
                movement=movement_values if moving else [0.0, 0.0, 0.0],
                facing=facing_values,
                speed=speed_value if moving else -1.0,
                height=-1.0,
            )
        )

    def send_game_command(self, command: RobotMotionCommand) -> None:
        if not isinstance(command, RobotMotionCommand):
            raise TypeError("command must be a RobotMotionCommand")
        if command.locomotion_mode not in {
            SONIC_IDLE_MODE,
            SONIC_SLOW_WALK_MODE,
            SONIC_WALK_MODE,
            SONIC_RUN_MODE,
        }:
            raise ValueError("game command must use native IDLE/SLOW_WALK/WALK/RUN")
        has_speed = command.speed_mps > 1e-6
        has_direction = math.hypot(
            command.movement[0], command.movement[1]
        ) > 1e-6
        if has_speed != has_direction:
            raise ValueError("game command speed and movement must become active together")
        moving = has_speed and has_direction
        if not moving:
            if command.locomotion_mode != SONIC_IDLE_MODE:
                raise ValueError("stationary game command must use native IDLE")
        else:
            if command.locomotion_mode == SONIC_IDLE_MODE:
                raise ValueError("moving game command cannot use native IDLE")
            minimum, maximum = SONIC_GAIT_SPEED_RANGES_MPS[
                command.locomotion_mode
            ]
            if not minimum <= command.speed_mps <= maximum:
                gait_name = SONIC_GAIT_NAMES[command.locomotion_mode]
                raise ValueError(
                    f"game command speed is outside native {gait_name} "
                    f"range {minimum:.1f}-{maximum:.1f} m/s"
                )
        self.send_direction(
            movement=command.movement,
            facing=command.facing,
            speed=command.speed_mps,
            locomotion_mode=command.locomotion_mode,
        )

    def close(self) -> None:
        stop_error = None
        try:
            stop_message = self._build_command_message(
                start=False,
                stop=True,
                planner=True,
                delta_heading=None,
            )
            # The native deploy binary exits through its ZMQ stop state and
            # does not install SIGTERM handlers. Repeat the native stop frame
            # briefly before the supervisor escalates to process-group signals.
            for _ in range(3):
                self._socket.send(stop_message)
                time.sleep(0.02)
        except Exception as exc:
            stop_error = exc
        finally:
            self._socket.close(linger=0)
        if stop_error is not None:
            raise RuntimeError(f"failed to send native planner stop: {stop_error}")


class GameInputRuntime:
    """Non-blocking bridge from one authenticated UI peer to the control core."""

    def __init__(
        self,
        path: Path,
        core: GameControlCore,
        *,
        expected_peer_pid: int | None = None,
    ) -> None:
        self.path = path
        self.core = core
        self.server = UnixSeqpacketInputServer(path)
        self.connection: UnixInputConnection | None = None
        self.accepted_connections = 0
        self.disconnects = 0
        self.packets_received = 0
        self.packets_applied = 0
        self.protocol_errors = 0
        self.rejected_packets = 0
        self.peer_pid_mismatches = 0
        self.moving_command_frames = 0
        self.peer_pid: int | None = None
        self.expected_peer_pid: int | None = None
        self.last_packet_at_s: float | None = None
        self.last_error: str | None = None
        self.last_command: RobotMotionCommand | None = None
        if expected_peer_pid is not None:
            self.bind_expected_peer_pid(expected_peer_pid)

    def open(self) -> None:
        self.server.open()

    def bind_expected_peer_pid(self, pid: int) -> None:
        """Pin the only peer allowed to drive this runtime."""

        if isinstance(pid, bool) or not isinstance(pid, int) or pid <= 1:
            raise ValueError("expected game-input peer PID must be greater than 1")
        if self.connection is not None:
            raise RuntimeError("cannot bind an expected peer after accepting a connection")
        if self.expected_peer_pid is not None and self.expected_peer_pid != pid:
            raise RuntimeError("game-input peer PID is already bound")
        self.expected_peer_pid = pid

    def _drop_connection(self, reason: str) -> None:
        connection = self.connection
        self.connection = None
        if connection is not None:
            try:
                connection.close()
            except OSError as exc:
                reason = f"{reason}; close_error: {exc}"
            self.disconnects += 1
        self.peer_pid = None
        self.last_error = reason
        # Invalidating the core is intentionally last and unconditional: even
        # a broken peer fd that fails close must zero the command this frame.
        self.core.invalidate_input(reason)

    def _accept_if_ready(self) -> None:
        if self.connection is not None:
            return
        try:
            connection = self.server.accept(timeout_s=0.0)
        except (socket.timeout, BlockingIOError):
            return
        if (
            self.expected_peer_pid is not None
            and connection.credentials.pid != self.expected_peer_pid
        ):
            actual_pid = connection.credentials.pid
            try:
                connection.close()
            finally:
                self.peer_pid_mismatches += 1
                self.last_error = (
                    "peer_pid_mismatch: "
                    f"expected={self.expected_peer_pid} actual={actual_pid}"
                )
                self.core.invalidate_input("peer_pid_mismatch")
            return
        self.connection = connection
        self.peer_pid = connection.credentials.pid
        self.accepted_connections += 1
        self.last_error = None

    def _drain(self, now_s: float) -> None:
        self._accept_if_ready()
        if self.connection is None:
            return
        batch_fault_reason: str | None = None
        for _ in range(64):
            try:
                snapshot = self.connection.receive(timeout_s=0.0)
            except (socket.timeout, BlockingIOError):
                break
            except EOFError:
                self._drop_connection("peer_closed")
                break
            except (OSError, InputProtocolError) as exc:
                self.protocol_errors += 1
                self.last_error = f"protocol_error: {exc}"
                if isinstance(exc, OSError):
                    self._drop_connection(f"peer_error: {exc}")
                    break
                self.core.invalidate_input("protocol_error")
                if batch_fault_reason is None:
                    batch_fault_reason = "protocol_error"
                continue
            self.packets_received += 1
            try:
                self.core.accept_snapshot(snapshot, received_at_s=now_s)
            except InputRejectedError as exc:
                self.rejected_packets += 1
                self.last_error = f"rejected: {exc}"
                self.core.invalidate_input("input_rejected")
                if batch_fault_reason is None:
                    batch_fault_reason = "input_rejected"
                continue
            self.packets_applied += 1
            self.last_packet_at_s = now_s
            self.last_error = None
        if batch_fault_reason is not None:
            # A later neutral packet in the same 64-message drain must not
            # erase a malformed/rejected packet's hard-stop frame, nor may a
            # following held key pre-arm the next frame. Preserve the first
            # fault as the authoritative batch outcome after the queue is
            # drained.
            self.core.invalidate_input(batch_fault_reason)
            self.last_error = batch_fault_reason

    def poll(self, *, now_s: float, dt_s: float) -> RobotMotionCommand:
        self._drain(now_s)
        self.last_command = self.core.command(now_s=now_s, dt_s=dt_s)
        return self.last_command

    def record_published_command(self, command: RobotMotionCommand) -> None:
        """Record commands that actually crossed the native planner boundary."""

        if not isinstance(command, RobotMotionCommand):
            raise TypeError("published command must be a RobotMotionCommand")
        movement_norm = math.sqrt(
            sum(float(component) ** 2 for component in command.movement)
        )
        if (
            command.mode == "move"
            and command.speed_mps > 0.0
            and movement_norm > 0.0
            and not command.safe_stop
        ):
            self.moving_command_frames += 1

    def telemetry(self, *, now_s: float) -> dict[str, object]:
        command = self.last_command
        return {
            "accepted_connections": self.accepted_connections,
            "connected": self.connection is not None,
            "disconnects": self.disconnects,
            "free_camera": self.core.free_camera,
            "free_camera_authoritative": False,
            "free_camera_inferred": self.core.free_camera,
            "heading_rad": round(self.core.heading_rad, 6),
            "measured_heading_rad": (
                round(self.core.measured_heading_rad, 6)
                if self.core.measured_heading_rad is not None
                else None
            ),
            "input_age_s": (
                round(max(0.0, now_s - self.last_packet_at_s), 6)
                if self.last_packet_at_s is not None
                else None
            ),
            "last_error": self.last_error,
            "mode": command.mode if command is not None else "deadman",
            "locomotion_mode": (
                command.locomotion_mode if command is not None else SONIC_IDLE_MODE
            ),
            "locomotion_mode_name": (
                SONIC_GAIT_NAMES.get(command.locomotion_mode, "UNKNOWN")
                if command is not None
                else SONIC_GAIT_NAMES[SONIC_IDLE_MODE]
            ),
            "moving_command_frames": self.moving_command_frames,
            "packets_applied": self.packets_applied,
            "packets_received": self.packets_received,
            "peer_pid": self.peer_pid,
            "expected_peer_pid": self.expected_peer_pid,
            "peer_pid_mismatches": self.peer_pid_mismatches,
            "protocol_errors": self.protocol_errors,
            "rejected_packets": self.rejected_packets,
            "safe_stop": command.safe_stop if command is not None else True,
            "sequence": command.sequence if command is not None else None,
            "socket": str(self.path),
            "speed_mps": round(command.speed_mps, 6) if command is not None else 0.0,
            "stop_reason": command.reason if command is not None else "no_input",
        }

    def emergency_stop(self, *, now_s: float, reason: str) -> RobotMotionCommand:
        """Hard-zero without draining any packets that remain queued by a peer."""

        self.core.invalidate_input(reason)
        self.last_command = self.core.command(now_s=now_s, dt_s=0.0)
        return self.last_command

    def close(self) -> None:
        if self.connection is not None:
            self.connection.close()
            self.connection = None
        self.server.close()


def _peek_child_returncode(process: subprocess.Popen[bytes]) -> int | None:
    """Observe a direct child without releasing its PID/process-group identity."""

    flags = os.WEXITED | os.WNOHANG | os.WNOWAIT
    while True:
        try:
            result = os.waitid(os.P_PID, process.pid, flags)
            break
        except InterruptedError:
            continue
        except ChildProcessError as exc:
            cached = process.returncode
            if type(cached) is int:
                return cached
            raise RuntimeError(
                f"native child {process.pid} is no longer waitable"
            ) from exc
    if result is None:
        return None
    if result.si_code == os.CLD_EXITED:
        return int(result.si_status)
    return -int(result.si_status)


class NativeProcessGroup:
    """Own native SONIC deploy/PICO children without touching unrelated processes."""

    def __init__(self, sonic_root: Path, env: dict[str, str]) -> None:
        self.sonic_root = sonic_root
        self.env = env
        self.guardian = Path(__file__).with_name(
            "exec_with_parent_death_signal.py"
        ).resolve()
        if not self.guardian.is_file():
            raise RuntimeError(f"native parent-death guardian is missing: {self.guardian}")
        profile = (
            sonic_root
            / "gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/config/fastrtps_profile.xml"
        )
        self.env["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(profile)
        self.env["ROS_LOCALHOST_ONLY"] = "1"
        self.env["PYTHONNOUSERSITE"] = "1"
        existing_pythonpath = self.env.get("PYTHONPATH")
        self.env["PYTHONPATH"] = str(sonic_root) + (
            os.pathsep + existing_pythonpath if existing_pythonpath else ""
        )
        self.pass_fds: tuple[int, ...] = ()
        lock_fd_value = self.env.get("MATRIX_SONIC_HOST_LOCK_FD")
        if lock_fd_value:
            try:
                lock_fd = int(lock_fd_value)
                os.fstat(lock_fd)
            except (OSError, ValueError) as exc:
                raise RuntimeError(
                    f"invalid MATRIX_SONIC_HOST_LOCK_FD={lock_fd_value!r}: {exc}"
                ) from exc
            self.pass_fds = (lock_fd,)
        self.children: list[tuple[str, subprocess.Popen[bytes]]] = []
        self._stopping = False
        self._boundary_failure: tuple[str, int] | None = None
        self._closed = False

    def _start(
        self,
        name: str,
        command: list[str],
        cwd: Path,
        *,
        exec_command: bool = False,
    ) -> int:
        guarded_command = [
            sys.executable,
            str(self.guardian),
            "--expected-parent",
            str(os.getpid()),
        ]
        if exec_command:
            guarded_command.append("--exec-command")
        guarded_command.extend(("--", *command))
        process = subprocess.Popen(
            guarded_command,
            cwd=cwd,
            env=self.env,
            pass_fds=self.pass_fds,
            start_new_session=True,
        )
        self.children.append((name, process))
        return process.pid

    def start_pico(self, python: str, *, port: int) -> None:
        self._start(
            "pico-manager",
            [
                python,
                "-u",
                str(self.sonic_root / "gear_sonic/scripts/pico_manager_thread_server.py"),
                "--manager",
                "--port",
                str(port),
            ],
            self.sonic_root,
        )

    def start_game_input(
        self,
        python: str,
        script: Path,
        *,
        input_socket: Path,
        input_source: str,
        camera_yaw_source: str,
        look_button: str,
        initial_camera_yaw_deg: float,
        mouse_sensitivity_deg: float,
        camera_yaw_sign: int,
        camera_yaw_offset_deg: float,
        carla_host: str,
        carla_port: int,
        gamepad_look_yaw_rate_deg_s: float,
        gamepad_look_pitch_rate_deg_s: float,
        gamepad_look_deadzone: float,
        gamepad_look_min_pitch_deg: float,
        gamepad_look_max_pitch_deg: float,
        focus_title: str,
        expected_ue_pid: int,
        status_file: Path | None,
        mouse_settings_file: Path | None = None,
        applied_mouse_profile: str = "local",
        applied_mouse_speed_scale: float = 1.0,
        restart_request_file: Path | None = None,
        restart_capability_file: Path | None = None,
        restart_launcher_pid: int | None = None,
    ) -> int:
        command = [
            python,
            "-u",
            str(script),
            "--socket",
            str(input_socket),
            "--input-source",
            input_source,
            "--camera-yaw-source",
            camera_yaw_source,
            "--look-button",
            look_button,
            "--initial-camera-yaw-deg",
            str(initial_camera_yaw_deg),
            "--mouse-sensitivity-deg",
            str(mouse_sensitivity_deg),
            "--applied-mouse-profile",
            applied_mouse_profile,
            "--applied-mouse-speed-scale",
            str(applied_mouse_speed_scale),
            "--camera-yaw-sign",
            str(camera_yaw_sign),
            "--camera-yaw-offset-deg",
            str(camera_yaw_offset_deg),
            "--carla-host",
            carla_host,
            "--carla-port",
            str(carla_port),
            "--gamepad-look-yaw-rate-deg-s",
            str(gamepad_look_yaw_rate_deg_s),
            "--gamepad-look-pitch-rate-deg-s",
            str(gamepad_look_pitch_rate_deg_s),
            "--gamepad-look-deadzone",
            str(gamepad_look_deadzone),
            "--gamepad-look-min-pitch-deg",
            str(gamepad_look_min_pitch_deg),
            "--gamepad-look-max-pitch-deg",
            str(gamepad_look_max_pitch_deg),
            "--focus-title",
            focus_title,
            "--expected-ue-pid",
            str(expected_ue_pid),
        ]
        if mouse_settings_file is not None:
            command.extend(("--mouse-settings-file", str(mouse_settings_file)))
        restart_values = (
            restart_request_file,
            restart_capability_file,
            restart_launcher_pid,
        )
        if all(value is not None for value in restart_values):
            command.extend(
                (
                    "--restart-request-file",
                    str(restart_request_file),
                    "--restart-capability-file",
                    str(restart_capability_file),
                    "--restart-launcher-pid",
                    str(restart_launcher_pid),
                )
            )
        if status_file is not None:
            command.extend(("--status-file", str(status_file)))
        return self._start(
            "game-input",
            command,
            script.parent.parent,
            exec_command=True,
        )

    def start_deploy(self, *, interface: str, zmq_port: int) -> None:
        deploy_root = self.sonic_root / "gear_sonic_deploy"
        self._start(
            "deploy",
            [
                str(deploy_root / "target/release/g1_deploy_onnx_ref"),
                interface,
                "policy/release/model_decoder.onnx",
                "reference/example",
                "--obs-config",
                "policy/release/observation_config.yaml",
                "--encoder-file",
                "policy/release/model_encoder.onnx",
                "--planner-file",
                "planner/target_vel/V2/planner_sonic.onnx",
                "--input-type",
                "zmq_manager",
                "--output-type",
                "all",
                "--zmq-host",
                "localhost",
                "--zmq-port",
                str(zmq_port),
                "--disable-crc-check",
            ],
            deploy_root,
        )

    def failed_child(self) -> tuple[str, int] | None:
        if self._stopping:
            return None
        for name, process in self.children:
            code = _peek_child_returncode(process)
            if code is not None:
                return name, code
        return None

    def begin_expected_stop(self) -> tuple[str, int] | None:
        """Set the authoritative stop boundary after one final non-reaping peek."""

        if self._stopping:
            return self._boundary_failure
        self._boundary_failure = self.failed_child()
        self._stopping = True
        return self._boundary_failure

    def wait_for_child(self, name: str, *, timeout: float) -> bool:
        """Give a native child time to finish its own stop/cleanup path."""
        matching = [process for child_name, process in self.children if child_name == name]
        if not matching:
            return True
        deadline = time.monotonic() + max(timeout, 0.0)
        while any(_peek_child_returncode(process) is None for process in matching):
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            time.sleep(min(0.05, remaining))
        return True

    def close(self) -> None:
        if self._closed:
            return
        cleanup_errors: list[str] = []
        try:
            self.begin_expected_stop()
        except Exception as exc:
            # A supervision error must not prevent teardown of retained PGIDs.
            cleanup_errors.append(f"stop boundary: {exc}")
        self._closed = True
        process_groups = {process.pid for _, process in self.children}
        for process_group in process_groups:
            try:
                # The session leader may already have exited while one of its
                # descendants remains. Always address the whole process group.
                os.killpg(process_group, signal.SIGTERM)
            except ProcessLookupError:
                pass
            except PermissionError as exc:
                cleanup_errors.append(f"SIGTERM pgid={process_group}: {exc}")

        deadline = time.monotonic() + 5.0
        while self.children and time.monotonic() < deadline:
            try:
                if all(
                    _peek_child_returncode(process) is not None
                    for _, process in self.children
                ):
                    break
            except RuntimeError as exc:
                cleanup_errors.append(str(exc))
                break
            time.sleep(min(0.05, max(deadline - time.monotonic(), 0.0)))

        # Kill every retained group before reaping any leader. Even when a
        # leader exited during the grace period, an ignoring descendant can
        # still occupy that group; the unreaped leader prevents PGID reuse.
        for process_group in process_groups:
            try:
                os.killpg(process_group, signal.SIGKILL)
            except ProcessLookupError:
                pass
            except PermissionError as exc:
                cleanup_errors.append(f"SIGKILL pgid={process_group}: {exc}")

        for name, process in reversed(self.children):
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                cleanup_errors.append(f"{name} did not exit after SIGKILL")
        if cleanup_errors:
            raise RuntimeError("; ".join(cleanup_errors))


def _close_runtime_resource(name: str, resource) -> str | None:
    if resource is None:
        return None
    try:
        resource.close()
    except Exception as exc:
        print(f"matrix-sonic-runtime ERROR closing {name}: {exc}", flush=True)
        return f"{name}: {exc}"
    return None


def _record_cleanup_failure(path: Path | None, errors: list[str]) -> None:
    if path is None or not path.is_file():
        return
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        print(
            f"matrix-sonic-runtime ERROR reading status for cleanup failure: {exc}",
            flush=True,
        )
        return
    if not isinstance(payload, dict):
        return
    failures = payload.get("acceptance_failures")
    if not isinstance(failures, list):
        failures = []
    if "cleanup_failure" not in failures:
        failures.append("cleanup_failure")
    payload["acceptance_failures"] = failures
    payload["cleanup_errors"] = errors
    payload["passed"] = False
    payload["pre_cleanup_termination_reason"] = payload.get("termination_reason")
    payload["termination_reason"] = "cleanup_failure"
    _atomic_json(path, payload)


def main() -> int:
    args = _parse_args()
    args.verification_receipt_sha256 = None
    _arm_supervisor_parent_death(args.expected_parent_pid)
    run_id = uuid.uuid4().hex
    model_path = args.model.resolve()
    if not model_path.is_file():
        raise SystemExit(f"composed Matrix model is missing: {model_path}")
    if args.physics_hz <= 0.0 or args.control_hz <= 0.0:
        raise SystemExit("--physics-hz and --control-hz must be positive")
    if not math.isfinite(args.max_seconds) or args.max_seconds < 0.0:
        raise SystemExit("--max-seconds must be non-negative and finite")
    if args.min_active_seconds < 0.0:
        raise SystemExit("--min-active-seconds must be non-negative")
    if not math.isfinite(args.min_displacement_m) or args.min_displacement_m < 0.0:
        raise SystemExit("--min-displacement-m must be non-negative and finite")
    if args.min_final_x is not None and not math.isfinite(args.min_final_x):
        raise SystemExit("--min-final-x must be finite")
    if not math.isfinite(args.min_forward_x_m) or args.min_forward_x_m < 0.0:
        raise SystemExit("--min-forward-x-m must be non-negative and finite")
    if args.min_physics_hz < 0.0 or args.min_rtf < 0.0:
        raise SystemExit("--min-physics-hz and --min-rtf must be non-negative")
    if args.max_resets < 0:
        raise SystemExit("--max-resets must be non-negative")
    if args.ue_pid is not None and args.ue_pid <= 1:
        raise SystemExit("--ue-pid must identify a live UE process")
    game_config = None
    if args.control_source == "game":
        if args.game_max_speed > 0.8:
            raise SystemExit("--game-max-speed cannot exceed SLOW_WALK maximum 0.8")
        try:
            game_config = ControlConfig(
                max_speed_mps=args.game_max_speed,
                max_acceleration_mps2=args.game_max_acceleration,
                max_deceleration_mps2=args.game_max_deceleration,
                max_turn_rate_rad_s=args.game_max_turn_rate,
                stick_deadzone=args.game_stick_deadzone,
                input_timeout_s=args.game_input_timeout,
                max_snapshot_age_s=args.game_max_snapshot_age,
                max_future_skew_s=args.game_max_future_skew,
            )
        except (InputProtocolError, ValueError) as exc:
            raise SystemExit(f"invalid game control configuration: {exc}") from exc
        if not args.game_input_socket.is_absolute():
            raise SystemExit("--game-input-socket must be an absolute path")
        if not args.game_input_socket.parent.is_dir():
            raise SystemExit(
                "--game-input-socket parent does not exist: "
                f"{args.game_input_socket.parent}"
            )
        for name in (
            "game_initial_camera_yaw_deg",
            "game_mouse_sensitivity_deg",
            "game_camera_yaw_offset_deg",
            "gamepad_look_yaw_rate_deg_s",
            "gamepad_look_pitch_rate_deg_s",
            "gamepad_look_deadzone",
            "gamepad_look_min_pitch_deg",
            "gamepad_look_max_pitch_deg",
        ):
            if not math.isfinite(getattr(args, name)):
                raise SystemExit(f"--{name.replace('_', '-')} must be finite")
        if not 1 <= args.game_carla_port <= 65535:
            raise SystemExit("--game-carla-port must be in [1, 65535]")
        if (
            args.gamepad_look_yaw_rate_deg_s <= 0.0
            or args.gamepad_look_pitch_rate_deg_s <= 0.0
        ):
            raise SystemExit("gamepad look rates must be positive")
        if not 0.0 <= args.gamepad_look_deadzone < 1.0:
            raise SystemExit("--gamepad-look-deadzone must be in [0, 1)")
        if args.gamepad_look_min_pitch_deg >= args.gamepad_look_max_pitch_deg:
            raise SystemExit("gamepad camera pitch limits must be ordered")
        try:
            args.game_applied_mouse_speed_scale = canonical_remote_speed_scale(
                args.game_applied_mouse_speed_scale
            )
        except ValueError as exc:
            raise SystemExit(
                f"--game-applied-mouse-speed-scale is invalid: {exc}"
            ) from exc
        if (
            args.game_applied_mouse_profile == "local"
            and args.game_applied_mouse_speed_scale != 1.0
        ):
            raise SystemExit("Local applied mouse profile must use 1.0x")
        if (
            args.game_mouse_settings_file is not None
            and not args.game_mouse_settings_file.is_absolute()
        ):
            raise SystemExit("--game-mouse-settings-file must be absolute")
        restart_values = (
            args.game_restart_request_file,
            args.game_restart_capability_file,
            args.game_restart_launcher_pid,
        )
        if any(value is not None for value in restart_values) and not all(
            value is not None for value in restart_values
        ):
            raise SystemExit("game restart request arguments are all-or-none")
        for name in (
            "game_restart_request_file",
            "game_restart_capability_file",
        ):
            path = getattr(args, name)
            if path is not None and not path.is_absolute():
                raise SystemExit(f"--{name.replace('_', '-')} must be absolute")
        if (
            args.game_restart_launcher_pid is not None
            and args.game_restart_launcher_pid <= 1
        ):
            raise SystemExit("--game-restart-launcher-pid must be greater than one")
        if not args.no_game_input_provider:
            if args.ue_pid is None:
                raise SystemExit(
                    "game input provider requires --ue-pid for exact X11 focus binding"
                )
            if not args.game_input_provider.is_file():
                raise SystemExit(
                    f"game input provider is missing: {args.game_input_provider}"
                )
            if not Path(args.game_input_provider_python).is_file():
                raise SystemExit(
                    "game input provider Python is missing: "
                    f"{args.game_input_provider_python}"
                )
    sha256_pattern = r"[0-9a-f]{64}"
    if args.qualified_runtime:
        if not args.qualification_profile:
            raise SystemExit("--qualified-runtime requires --qualification-profile")
        if re.fullmatch(sha256_pattern, args.runtime_lock_sha256 or "") is None:
            raise SystemExit("--qualified-runtime requires --runtime-lock-sha256")
        if re.fullmatch(r"[0-9a-f]{40}", args.matrix_commit or "") is None:
            raise SystemExit("--qualified-runtime requires --matrix-commit")
        if args.verification_receipt is None:
            raise SystemExit("--qualified-runtime requires --verification-receipt")
        if (
            args.scenario_layout_sha256 is not None
            and re.fullmatch(sha256_pattern, args.scenario_layout_sha256) is None
        ):
            raise SystemExit("--scenario-layout-sha256 must be a lowercase SHA256")
    elif any(
        value is not None
        for value in (
            args.qualification_profile,
            args.runtime_lock_sha256,
            args.scenario_layout_sha256,
            args.matrix_commit,
            args.verification_receipt,
        )
    ):
        raise SystemExit("qualification metadata requires --qualified-runtime")
    qualification_receipt = _validate_qualification_receipt(args)
    _validate_qualified_acceptance(args)
    _validate_qualified_game_control(args)
    model_attestation = _validate_qualified_model(
        args, model_path, qualification_receipt
    )
    if (
        not math.isfinite(args.low_cmd_fresh_timeout_seconds)
        or args.low_cmd_fresh_timeout_seconds <= 0.0
    ):
        raise SystemExit("--low-cmd-fresh-timeout-seconds must be positive and finite")
    if args.dds_interface != "lo":
        raise SystemExit("native Matrix SONIC requires --dds-interface lo")
    try:
        planner_port = _loopback_zmq_port(args.planner_bind)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    if args.startup_band_hold < 0.0 or args.startup_band_fade < 0.0:
        raise SystemExit("startup band hold/fade durations must be non-negative")
    sonic_root = _configure_native_runtime(args)
    sonic_commit = _sonic_commit(sonic_root)
    try:
        import numpy as np
        import zmq
        from gear_sonic.scripts.run_sim_loop import create_simulator
        from gear_sonic.utils.mujoco_sim.configs import SimLoopConfig
        from gear_sonic.utils.teleop.zmq.zmq_planner_sender import (
            build_command_message,
            build_planner_message,
        )
        from matrix_render_protocol import MatrixRenderPublisher, packet_size
    except ImportError as exc:
        raise SystemExit(
            f"Native SONIC runtime dependency is missing: {exc}. "
            "Use the SONIC commit pinned by config/runtime/matrix-sonic.lock.json."
        ) from exc

    config = SimLoopConfig(**_native_config_kwargs(args, model_path))
    simulator = create_simulator(config)
    renderer = None
    planner = None
    game_input = None
    game_readiness = None
    game_command = None
    processes = None
    previous_signal_handlers: dict[int, Any] = {}
    running = True
    termination_reason: str | None = None
    termination_signal: int | None = None
    child_failure: tuple[str, int] | None = None
    numerical_error: str | None = None

    def request_stop(signum, _frame) -> None:
        nonlocal running, termination_reason, termination_signal
        running = False
        if termination_reason is None:
            termination_reason = "signal"
            termination_signal = int(signum)

    try:
        # Install handlers before any child is started. Everything after the
        # simulator construction is inside this cleanup boundary.
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handler = signal.getsignal(signum)
            signal.signal(signum, request_stop)
            previous_signal_handlers[int(signum)] = previous_handler

        snapshot = simulator.get_state_snapshot()
        initial_snapshot_error = _snapshot_validation_error(snapshot)
        if initial_snapshot_error is not None:
            raise SystemExit(
                f"invalid native SONIC initial snapshot: {initial_snapshot_error}"
            )
        qpos = snapshot.qpos

        physics_hz = float(args.physics_hz)
        substeps_float = physics_hz / args.control_hz
        substeps = int(round(substeps_float))
        if substeps <= 0 or not math.isclose(
            substeps_float, substeps, rel_tol=0.0, abs_tol=1e-6
        ):
            raise SystemExit(
                f"control_hz={args.control_hz} must divide "
                f"physics_hz={physics_hz} exactly"
            )

        initial_root_xy = np.asarray(qpos[:2], dtype=np.float64).copy()
        try:
            initial_root_yaw_rad = (
                _root_yaw_rad(qpos) if args.control_source == "game" else None
            )
        except ValueError as exc:
            raise SystemExit(f"invalid native SONIC initial root heading: {exc}") from exc
        renderer = (
            None
            if args.no_render_sync
            else MatrixRenderPublisher(args.render_host, args.render_port)
        )
        processes = NativeProcessGroup(sonic_root, os.environ.copy())

        def register_child_failure(failure: tuple[str, int] | None) -> bool:
            nonlocal child_failure, running, termination_reason
            if failure is None:
                return False
            if child_failure is None:
                child_failure = failure
                name, code = failure
                print(
                    "matrix-sonic-runtime ERROR child exited: "
                    f"{name}={code}",
                    flush=True,
                )
            running = False
            if termination_reason != "numerical_instability":
                termination_reason = "child_exit"
            return True

        def poll_failed_child() -> bool:
            failure = _read_external_failure(args.external_failure_file)
            if failure is None:
                failure = processes.failed_child()
            return register_child_failure(failure)

        # The parent shell supervises UE from the instant it is spawned. A UE
        # failure during the historical seven-second startup window is already
        # present here and must prevent deploy/PICO from starting.
        poll_failed_child()
        if running and args.control_source in {"planner", "game"}:
            planner = NativePlannerClient(
                args.planner_bind,
                zmq_module=zmq,
                build_command_message=build_command_message,
                build_planner_message=build_planner_message,
            )
            if args.control_source == "game":
                assert game_config is not None
                game_input = GameInputRuntime(
                    args.game_input_socket,
                    GameControlCore(game_config),
                )
                game_readiness = _GameSonicReadinessGate(snapshot)
                try:
                    game_input.open()
                except OSError as exc:
                    raise SystemExit(
                        f"failed to open game input socket {args.game_input_socket}: {exc}"
                    ) from exc
                if not args.no_game_input_provider:
                    provider_pid = processes.start_game_input(
                        args.game_input_provider_python,
                        args.game_input_provider,
                        input_socket=args.game_input_socket,
                        input_source=args.game_input_source,
                        camera_yaw_source=args.game_camera_yaw_source,
                        look_button=args.game_look_button,
                        initial_camera_yaw_deg=args.game_initial_camera_yaw_deg,
                        mouse_sensitivity_deg=args.game_mouse_sensitivity_deg,
                        mouse_settings_file=args.game_mouse_settings_file,
                        applied_mouse_profile=args.game_applied_mouse_profile,
                        applied_mouse_speed_scale=(
                            args.game_applied_mouse_speed_scale
                        ),
                        restart_request_file=args.game_restart_request_file,
                        restart_capability_file=(
                            args.game_restart_capability_file
                        ),
                        restart_launcher_pid=args.game_restart_launcher_pid,
                        camera_yaw_sign=args.game_camera_yaw_sign,
                        camera_yaw_offset_deg=args.game_camera_yaw_offset_deg,
                        carla_host=args.game_carla_host,
                        carla_port=args.game_carla_port,
                        gamepad_look_yaw_rate_deg_s=(
                            args.gamepad_look_yaw_rate_deg_s
                        ),
                        gamepad_look_pitch_rate_deg_s=(
                            args.gamepad_look_pitch_rate_deg_s
                        ),
                        gamepad_look_deadzone=args.gamepad_look_deadzone,
                        gamepad_look_min_pitch_deg=(
                            args.gamepad_look_min_pitch_deg
                        ),
                        gamepad_look_max_pitch_deg=(
                            args.gamepad_look_max_pitch_deg
                        ),
                        focus_title=args.game_focus_title,
                        expected_ue_pid=args.ue_pid,
                        status_file=args.game_input_status_file,
                    )
                    game_input.bind_expected_peer_pid(provider_pid)
        elif running and args.control_source == "pico":
            processes.start_pico(
                args.pico_python or sys.executable, port=planner_port
            )
        if running:
            processes.start_deploy(
                interface=args.dds_interface, zmq_port=planner_port
            )

        expected_packet_size = packet_size(
            nq=len(snapshot.qpos), nv=len(snapshot.qvel), nu=len(snapshot.ctrl)
        )
        render_target = (
            "disabled"
            if renderer is None
            else f"{args.render_host}:{args.render_port}"
        )
        print(
            "matrix-sonic-runtime "
            f"backend=gear_sonic_native sonic_commit={sonic_commit} "
            f"model={model_path} nq={len(snapshot.qpos)} nv={len(snapshot.qvel)} "
            f"nu={len(snapshot.ctrl)} physics_hz={physics_hz:.1f} "
            f"control_hz={args.control_hz:.1f} substeps={substeps} "
            f"render={render_target} packet_bytes={expected_packet_size} "
            f"control_source={args.control_source}",
            flush=True,
        )

        started_wall = time.perf_counter()
        heading_anchor_telemetry = (
            _HeadingAnchorTelemetry(initial_root_yaw_rad, snapshot)
            if initial_root_yaw_rad is not None
            else None
        )
        physics_period_s = 1.0 / physics_hz
        next_physics_wall = started_wall + physics_period_s
        next_print = started_wall
        last_print_wall = started_wall
        last_render_count = 0
        last_physics_steps = 0
        control_frames = 0
        active_frames = 0
        physics_steps = 0
        instability_resets = int(snapshot.reset_count)
        unstable = False
        fall_detected = bool(snapshot.fall_detected)
        min_root_z = float(snapshot.qpos[2])
        active_started_wall = None
        longest_active_elapsed_s = 0.0
        walking = False
        while running:
            # Poll on both sides of the duration gate so a child exit at the
            # acceptance boundary cannot be mistaken for normal completion.
            if poll_failed_child():
                break
            frame_wall = time.perf_counter()
            elapsed_wall = frame_wall - started_wall
            if args.max_seconds > 0.0 and elapsed_wall >= args.max_seconds:
                termination_reason = "max_seconds"
                poll_failed_child()
                break

            active_elapsed = (
                frame_wall - active_started_wall
                if active_started_wall is not None
                else 0.0
            )
            walking = (
                active_started_wall is not None
                and args.walk_after >= 0.0
                and active_elapsed >= args.walk_after
            )
            if planner is not None:
                if game_input is not None:
                    assert initial_root_yaw_rad is not None
                    assert game_readiness is not None
                    try:
                        measured_heading = wrap_angle_rad(
                            _root_yaw_rad(snapshot.qpos) - initial_root_yaw_rad
                        )
                    except (InputProtocolError, ValueError) as exc:
                        unstable = True
                        running = False
                        termination_reason = "numerical_instability"
                        numerical_error = f"root_heading:{exc}"
                        print(
                            "matrix-sonic-runtime ERROR invalid root heading: "
                            f"{exc}",
                            flush=True,
                        )
                        break
                    game_input.core.synchronize_heading(measured_heading)
                    game_readiness.begin_frame(snapshot, game_input.core)
                    candidate_game_command = game_input.poll(
                        now_s=frame_wall,
                        dt_s=1.0 / args.control_hz,
                    )
                    game_command = game_readiness.apply(
                        candidate_game_command,
                        game_input.core,
                    )
                    # Telemetry must describe the command actually published,
                    # not the pre-readiness candidate returned by the core.
                    game_input.last_command = game_command
                    planner.send_game_command(game_command)
                    game_input.record_published_command(game_command)
                    walking = game_command.mode == "move"
                else:
                    planner.send_velocity(
                        args.vx if walking else 0.0,
                        args.vy if walking else 0.0,
                        args.yaw_rate if walking else 0.0,
                        dt=1.0 / args.control_hz,
                    )

            for _ in range(substeps):
                if not running:
                    break
                # Keep native DDS lowstate and MuJoCo cadence at 200 Hz instead
                # of emitting four back-to-back steps once per 50 Hz frame.
                # Matrix owns the absolute deadline because SONIC's relative
                # per-step sleep otherwise accumulates scheduler overshoot.
                next_snapshot = simulator.step_once(rate_limit=False)
                physics_steps += 1
                step_error = _snapshot_validation_error(next_snapshot, snapshot)
                if step_error is not None:
                    unstable = True
                    running = False
                    termination_reason = "numerical_instability"
                    numerical_error = step_error
                    print(
                        "matrix-sonic-runtime ERROR invalid native SONIC step: "
                        f"{step_error}",
                        flush=True,
                    )
                    break
                snapshot = next_snapshot
                if heading_anchor_telemetry is not None:
                    try:
                        heading_anchor_telemetry.observe(
                            snapshot,
                            wall_elapsed_s=max(
                                time.perf_counter() - started_wall, 0.0
                            ),
                        )
                    except ValueError as exc:
                        unstable = True
                        running = False
                        termination_reason = "numerical_instability"
                        numerical_error = f"root_heading_telemetry:{exc}"
                        print(
                            "matrix-sonic-runtime ERROR invalid root heading "
                            f"telemetry: {exc}",
                            flush=True,
                        )
                        break
                instability_resets = int(snapshot.reset_count)
                if args.fail_on_fall and bool(snapshot.fall_detected):
                    fall_detected = True
                    running = False
                    termination_reason = "fall_detected"
                    break
                if instability_resets > args.max_resets:
                    running = False
                    termination_reason = "reset_detected"
                    break
                next_physics_wall = _pace_absolute_deadline(
                    next_physics_wall, physics_period_s
                )

            if not running:
                break
            if poll_failed_child():
                break

            active_lowcmd = bool(snapshot.low_cmd_fresh)
            low_cmd_age_s = snapshot.low_cmd_age_s
            freshness_sample_wall = time.perf_counter()
            if active_lowcmd:
                if active_started_wall is None:
                    active_started_wall = freshness_sample_wall
                active_elapsed = freshness_sample_wall - active_started_wall
                longest_active_elapsed_s = max(
                    longest_active_elapsed_s, active_elapsed
                )
                active_frames += 1
            else:
                active_started_wall = None
                active_elapsed = 0.0

            root_z = float(snapshot.qpos[2])
            root_up_z = _root_up_z(snapshot.qpos)
            min_root_z = min(min_root_z, root_z)
            # SONIC is the sole fall authority. Height and orientation remain
            # diagnostics only, so Matrix cannot disagree with its snapshot.
            fall_detected = fall_detected or bool(snapshot.fall_detected)

            if renderer is not None:
                renderer.send(
                    snapshot.sim_time,
                    snapshot.qpos,
                    snapshot.qvel,
                    snapshot.ctrl,
                )
            control_frames += 1

            now = time.perf_counter()
            if now >= next_print:
                window_wall = max(now - last_print_wall, 1e-9)
                render_count = renderer.packet_count if renderer is not None else 0
                window_render = render_count - last_render_count
                window_physics_steps = physics_steps - last_physics_steps
                status = {
                    "active_elapsed_s": round(
                        now - active_started_wall, 3
                    ) if active_started_wall is not None else 0.0,
                    "active_lowcmd_longest_s": round(longest_active_elapsed_s, 3),
                    "active_lowcmd": active_lowcmd,
                    "backend": "gear_sonic_native",
                    "control_frames": control_frames,
                    "control_hz": args.control_hz,
                    "control_source": args.control_source,
                    "elapsed_wall_s": round(now - started_wall, 3),
                    "model": str(model_path),
                    **model_attestation,
                    "nu": len(snapshot.ctrl),
                    "low_cmd_age_s": (
                        round(float(low_cmd_age_s), 6)
                        if low_cmd_age_s is not None
                        else None
                    ),
                    "low_cmd_fresh_timeout_s": args.low_cmd_fresh_timeout_seconds,
                    "low_cmd_received": bool(snapshot.low_cmd_received),
                    "fall_detected": fall_detected,
                    "min_displacement_m": args.min_displacement_m,
                    "min_final_x": args.min_final_x,
                    "min_forward_x_m": args.min_forward_x_m,
                    "min_root_z": round(min_root_z, 5),
                    "physics_hz_target": physics_hz,
                    "physics_step_hz": round(window_physics_steps / window_wall, 3),
                    "render_hz": round(window_render / window_wall, 3),
                    "render_packet_bytes": expected_packet_size,
                    "render_sync_enabled": renderer is not None,
                    "ue_state_sync_hz": round(window_render / window_wall, 3),
                    "ue_pid": args.ue_pid,
                    "root_xyz": [round(float(value), 5) for value in snapshot.qpos[:3]],
                    "root_displacement_xy_m": round(
                        float(
                            np.linalg.norm(
                                np.asarray(snapshot.qpos[:2]) - initial_root_xy
                            )
                        ),
                        5,
                    ),
                    "root_up_z": round(root_up_z, 5),
                    "run_id": run_id,
                    "runtime_lock_sha256": args.runtime_lock_sha256,
                    "runtime_verified": bool(args.qualified_runtime),
                    "qualification_profile": args.qualification_profile,
                    "scenario_layout_sha256": args.scenario_layout_sha256,
                    "matrix_commit": args.matrix_commit,
                    "verification_receipt": (
                        str(args.verification_receipt)
                        if args.verification_receipt is not None
                        else None
                    ),
                    "verification_receipt_sha256": args.verification_receipt_sha256,
                    "rtf": round(
                        (window_physics_steps / physics_hz) / window_wall,
                        4,
                    ),
                    "sim_time_s": round(float(snapshot.sim_time), 4),
                    "sonic_commit": sonic_commit,
                    "sonic_step_index": int(snapshot.step_index),
                    "instability_resets": instability_resets,
                    "last_reset_reason": snapshot.last_reset_reason,
                    "max_resets": args.max_resets,
                    "startup_band_enabled": bool(args.startup_band),
                    "startup_band_hold_s": args.startup_band_hold,
                    "startup_band_fade_s": args.startup_band_fade,
                    "startup_band_scale": round(float(snapshot.elastic_band_scale), 5),
                    "walking_commanded": walking,
                }
                if game_input is not None:
                    status["game_input"] = game_input.telemetry(now_s=now)
                    status["game_control_configuration"] = (
                        _game_control_status_fields(args)
                    )
                    current_root_yaw = _root_yaw_rad(snapshot.qpos)
                    assert initial_root_yaw_rad is not None
                    assert heading_anchor_telemetry is not None
                    status.update(heading_anchor_telemetry.status_fields())
                    status["root_yaw_world_rad"] = round(current_root_yaw, 6)
                    status["root_yaw_relative_rad"] = round(
                        wrap_angle_rad(current_root_yaw - initial_root_yaw_rad), 6
                    )
                print(
                    f"matrix-sonic-runtime status={json.dumps(status, sort_keys=True)}",
                    flush=True,
                )
                _atomic_json(args.status_file, status)
                last_print_wall = now
                last_render_count = render_count
                last_physics_steps = physics_steps
                next_print = now + max(args.print_every, 0.1)

        # Drain packets already queued at the exact acceptance boundary.  The
        # duration gate can otherwise break before observing a final focus-loss,
        # EOF, or malformed packet.  dt=0 updates only input state; this command
        # is never published, and emergency_stop() immediately follows.
        game_input_boundary = None
        if game_input is not None:
            assert game_readiness is not None
            assert initial_root_yaw_rad is not None
            try:
                boundary_measured_heading = wrap_angle_rad(
                    _root_yaw_rad(snapshot.qpos) - initial_root_yaw_rad
                )
            except (InputProtocolError, ValueError) as exc:
                unstable = True
                termination_reason = "numerical_instability"
                numerical_error = f"root_heading_at_boundary:{exc}"
                game_input.core.invalidate_input("invalid_boundary_heading")
            else:
                # The duration gate runs before the next regular control frame.
                # Refresh measured yaw from the latest physics snapshot so the
                # boundary/emergency stop cannot retain a one-frame-old facing.
                game_input.core.synchronize_heading(boundary_measured_heading)
            boundary_now = time.perf_counter()
            game_readiness.begin_frame(snapshot, game_input.core)
            boundary_candidate = game_input.poll(now_s=boundary_now, dt_s=0.0)
            game_input.last_command = game_readiness.apply(
                boundary_candidate,
                game_input.core,
            )
            # emergency_stop() intentionally overwrites the live command below,
            # so sampling later would make every clean qualified run appear to
            # have ended in safe-stop mode.
            game_input_boundary = game_input.telemetry(now_s=boundary_now)

        # Zero interactive motion before status aggregation or child teardown.
        # In particular, a provider crash must not leave the last moving frame
        # active while the runtime prepares its final report.
        if game_input is not None and planner is not None:
            if child_failure is not None:
                child_name, child_code = child_failure
                game_stop_reason = f"child_exit:{child_name}:{child_code}"
            else:
                game_stop_reason = f"runtime_stop:{termination_reason or 'stopping'}"
            game_command = game_input.emergency_stop(
                now_s=time.perf_counter(),
                reason=game_stop_reason,
            )
            planner.send_game_command(game_command)
            walking = False

        # One final poll followed by a non-reaping stop boundary closes the race
        # between the last loop poll and final status publication. Any native
        # exit observed before this boundary is a failure, including exit 0.
        poll_failed_child()
        register_child_failure(processes.begin_expected_stop())
        # The UE supervisor has its own exact stop boundary. Re-read its atomic
        # channel after committing the native boundary so neither child class
        # can hide in the handoff between loop completion and acceptance.
        poll_failed_child()
        if termination_reason is None:
            termination_reason = "signal" if not running else "unknown"

        finished_wall = time.perf_counter()
        elapsed_wall_s = finished_wall - started_wall
        active_lowcmd = bool(snapshot.low_cmd_fresh)
        active_elapsed_s = 0.0
        if active_lowcmd and active_started_wall is not None:
            active_elapsed_s = finished_wall - active_started_wall
        longest_active_elapsed_s = max(longest_active_elapsed_s, active_elapsed_s)
        physics_step_hz_aggregate = physics_steps / max(elapsed_wall_s, 1e-9)
        rtf_aggregate = (physics_steps / physics_hz) / max(elapsed_wall_s, 1e-9)
        render_count = renderer.packet_count if renderer is not None else 0
        render_hz_aggregate = render_count / max(elapsed_wall_s, 1e-9)
        root_z = float(snapshot.qpos[2])
        root_up_z = _root_up_z(snapshot.qpos)
        min_root_z = min(min_root_z, root_z)
        fall_detected = fall_detected or bool(snapshot.fall_detected)
        instability_resets = int(snapshot.reset_count)
        root_displacement_xy_m = float(
            np.linalg.norm(np.asarray(snapshot.qpos[:2]) - initial_root_xy)
        )
        acceptance_failures = _acceptance_failures(
            unstable=unstable,
            fall_detected=fall_detected,
            fail_on_fall=args.fail_on_fall,
            active_lowcmd=active_lowcmd,
            active_elapsed_s=longest_active_elapsed_s,
            min_active_seconds=args.min_active_seconds,
            physics_step_hz=physics_step_hz_aggregate,
            min_physics_hz=args.min_physics_hz,
            rtf=rtf_aggregate,
            min_rtf=args.min_rtf,
            failed_child=child_failure,
            root_displacement_xy_m=root_displacement_xy_m,
            min_displacement_m=args.min_displacement_m,
            root_final_x=float(snapshot.qpos[0]),
            min_final_x=args.min_final_x,
            root_displacement_x_m=float(snapshot.qpos[0]) - float(initial_root_xy[0]),
            min_forward_x_m=args.min_forward_x_m,
            reset_count=instability_resets,
            max_resets=args.max_resets,
        )
        if args.qualified_runtime and args.control_source == "game":
            assert game_input is not None
            assert game_input_boundary is not None
            acceptance_failures.extend(
                _game_input_acceptance_failures(
                    accepted_connections=game_input.accepted_connections,
                    packets_applied=game_input.packets_applied,
                    moving_command_frames=game_input.moving_command_frames,
                    protocol_errors=game_input.protocol_errors,
                    rejected_packets=game_input.rejected_packets,
                    peer_pid_mismatches=game_input.peer_pid_mismatches,
                    connected_at_boundary=bool(game_input_boundary["connected"]),
                    input_age_s=game_input_boundary["input_age_s"],
                    maximum_boundary_age_s=(
                        args.game_input_timeout + (1.0 / args.control_hz)
                    ),
                    safe_stop_at_boundary=bool(game_input_boundary["safe_stop"]),
                )
            )
        qualification = _qualification_state(
            max_seconds=args.max_seconds,
            termination_reason=termination_reason,
            failures=acceptance_failures,
            runtime_verified=bool(args.qualified_runtime),
        )
        acceptance_failures = qualification["acceptance_failures"]
        qualification_attempted = bool(qualification["qualification_attempted"])
        completed = bool(qualification["completed"])
        passed = bool(qualification["passed"])
        interrupted = bool(qualification["interrupted"])

        failed_child_name = child_failure[0] if child_failure is not None else None
        failed_child_code = child_failure[1] if child_failure is not None else None
        final_status = {
            "acceptance_failures": acceptance_failures,
            "active_elapsed_s": round(active_elapsed_s, 3),
            "active_frames": active_frames,
            "active_lowcmd": active_lowcmd,
            "active_lowcmd_longest_s": round(longest_active_elapsed_s, 3),
            "backend": "gear_sonic_native",
            "completed": completed,
            "control_frames": control_frames,
            "control_hz": args.control_hz,
            "control_source": args.control_source,
            "elapsed_wall_s": round(elapsed_wall_s, 3),
            "failed_child_exit_code": failed_child_code,
            "failed_child_name": failed_child_name,
            "fall_detected": fall_detected,
            "instability_resets": instability_resets,
            "interrupted": interrupted,
            "last_reset_reason": snapshot.last_reset_reason,
            "low_cmd_age_s": (
                round(float(snapshot.low_cmd_age_s), 6)
                if snapshot.low_cmd_age_s is not None
                else None
            ),
            "low_cmd_fresh_timeout_s": args.low_cmd_fresh_timeout_seconds,
            "low_cmd_received": bool(snapshot.low_cmd_received),
            "min_physics_hz": args.min_physics_hz,
            "min_displacement_m": args.min_displacement_m,
            "min_final_x": args.min_final_x,
            "min_forward_x_m": args.min_forward_x_m,
            "min_root_z": round(min_root_z, 5),
            "min_rtf": args.min_rtf,
            "max_resets": args.max_resets,
            "matrix_commit": args.matrix_commit,
            "verification_receipt": (
                str(args.verification_receipt)
                if args.verification_receipt is not None
                else None
            ),
            "verification_receipt_sha256": args.verification_receipt_sha256,
            "model": str(model_path),
            **model_attestation,
            "nq": len(snapshot.qpos),
            "nu": len(snapshot.ctrl),
            "numerical_error": numerical_error,
            "nv": len(snapshot.qvel),
            "passed": passed,
            "physics_hz_target": physics_hz,
            "physics_step_hz": round(physics_step_hz_aggregate, 3),
            "physics_step_hz_aggregate": round(physics_step_hz_aggregate, 3),
            "physics_steps": physics_steps,
            "qualification_attempted": qualification_attempted,
            "qualification_profile": args.qualification_profile,
            "render_hz": round(render_hz_aggregate, 3),
            "render_packet_bytes": expected_packet_size,
            "render_sync_enabled": renderer is not None,
            "root_displacement_xy_m": round(root_displacement_xy_m, 5),
            "root_final_x": round(float(snapshot.qpos[0]), 5),
            "root_displacement_x_m": round(
                float(snapshot.qpos[0]) - float(initial_root_xy[0]), 5
            ),
            "root_up_z": round(root_up_z, 5),
            "root_xyz": [round(float(value), 5) for value in snapshot.qpos[:3]],
            "rtf": round(rtf_aggregate, 4),
            "rtf_aggregate": round(rtf_aggregate, 4),
            "run_id": run_id,
            "runtime_lock_sha256": args.runtime_lock_sha256,
            "runtime_verified": bool(args.qualified_runtime),
            "scenario_layout_sha256": args.scenario_layout_sha256,
            "sim_time_s": round(float(snapshot.sim_time), 4),
            "sonic_commit": sonic_commit,
            "sonic_step_index": int(snapshot.step_index),
            "startup_band_enabled": bool(args.startup_band),
            "startup_band_fade_s": args.startup_band_fade,
            "startup_band_hold_s": args.startup_band_hold,
            "startup_band_scale": round(float(snapshot.elastic_band_scale), 5),
            "termination_reason": termination_reason,
            "termination_signal": termination_signal,
            "ue_state_sync_hz": round(render_hz_aggregate, 3),
            "ue_pid": args.ue_pid,
            "walking_commanded": walking,
        }
        if game_input is not None:
            final_status["game_input"] = game_input.telemetry(now_s=finished_wall)
            final_status["game_input_at_boundary"] = game_input_boundary
            final_status["game_control_configuration"] = (
                _game_control_status_fields(args)
            )
            assert initial_root_yaw_rad is not None
            assert heading_anchor_telemetry is not None
            final_status.update(heading_anchor_telemetry.status_fields())
            try:
                final_root_yaw = _root_yaw_rad(snapshot.qpos)
            except ValueError:
                final_status["root_yaw_world_rad"] = None
                final_status["root_yaw_relative_rad"] = None
            else:
                final_status["root_yaw_world_rad"] = round(final_root_yaw, 6)
                final_status["root_yaw_relative_rad"] = round(
                    wrap_angle_rad(final_root_yaw - initial_root_yaw_rad), 6
                )
        _atomic_json(args.status_file, final_status)
        print(
            "matrix-sonic-runtime stopped "
            f"wall_s={elapsed_wall_s:.2f} sim_s={snapshot.sim_time:.2f} "
            f"frames={control_frames} active_frames={active_frames} "
            f"reason={termination_reason} passed={passed} "
            f"failures={acceptance_failures}",
            flush=True,
        )
        if passed or (not qualification_attempted and interrupted and not acceptance_failures):
            return 0
        return 2
    finally:
        active_exception = sys.exc_info()[0] is not None
        cleanup_errors = []
        error = _close_runtime_resource("planner", planner)
        if error is not None:
            cleanup_errors.append(error)
        if planner is not None and processes is not None:
            try:
                processes.wait_for_child("deploy", timeout=2.0)
            except Exception as exc:
                print(
                    "matrix-sonic-runtime ERROR waiting for native deploy stop: "
                    f"{exc}",
                    flush=True,
                )
                cleanup_errors.append(f"native deploy stop wait: {exc}")
        error = _close_runtime_resource("game input", game_input)
        if error is not None:
            cleanup_errors.append(error)
        for name, resource in (
            ("native processes", processes),
            ("renderer", renderer),
            ("simulator", simulator),
        ):
            error = _close_runtime_resource(name, resource)
            if error is not None:
                cleanup_errors.append(error)
        for signum, previous_handler in previous_signal_handlers.items():
            try:
                signal.signal(signum, previous_handler)
            except (OSError, ValueError) as exc:
                print(
                    "matrix-sonic-runtime ERROR restoring signal handler "
                    f"{signum}: {exc}",
                    flush=True,
                )
                cleanup_errors.append(
                    f"signal handler {signum}: {exc}"
                )
        if cleanup_errors:
            _record_cleanup_failure(args.status_file, cleanup_errors)
            if not active_exception:
                raise RuntimeError(
                    "native cleanup failed: " + "; ".join(cleanup_errors)
                )


if __name__ == "__main__":
    raise SystemExit(main())
