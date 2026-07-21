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
import struct
import subprocess
import sys
import tempfile
import threading
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
from matrix_mc_commands import (
    CommandExecutionError,
    CommandProtocolError,
    GameCommandRequest,
    GameCommandResponse,
    MAX_COMMAND_PACKET_BYTES,
    PolicySlotAssignment,
    decode_command_request,
    encode_command_response,
    execute_command,
)
from matrix_mouse_settings import canonical_remote_speed_scale
from matrix_policy_slots import (
    BFM_TEACHER50K_POLICY_ID,
    PolicyCandidateState,
    evaluate_policy_candidate,
)
from matrix_world_state import (
    WorldPose,
    WorldStateError,
    WorldStateStore,
)


_GAME_INTERNAL_RESTART_EXIT_CODE = 75
_GAME_INTERNAL_RESTART_REASONS = frozenset(
    {"game_fall_respawn", "game_teleport"}
)
_WORLD_SAFE_MIN_ROOT_Z = 0.55
_WORLD_SAFE_MIN_ROOT_UP_Z = 0.85
_WORLD_SAFE_MAX_VERTICAL_SPEED_M_S = 0.35
_WORLD_SAFE_MAX_TILT_RATE_RAD_S = 0.75
from matrix_mujoco_contacts import (
    has_external_foot_support,
    has_external_ground_support,
)
from matrix_sonic_recovery import (
    RecoveryConfig,
    RecoveryInput,
    RecoveryOutput,
    RecoveryState,
    ResidentPolicyRecoveryFSM,
    ResidentRecoveryInput,
    ResidentRecoveryOutput,
    ResidentRecoveryState,
    SingleWriterRecoveryFSM,
)


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
        "--game-fall-recovery",
        choices=("off", "sonic", "physical"),
        default="off",
        help=(
            "Interactive fall behavior: 'sonic' keeps the runtime alive and "
            "holds native IDLE; 'physical' hands LowCmd to a get-up policy, "
            "then returns authority to SONIC"
        ),
    )
    parser.add_argument(
        "--game-fall-recovery-timeout",
        type=float,
        default=15.0,
        help="Seconds before an active SONIC recovery is marked timed out",
    )
    parser.add_argument(
        "--physical-recovery-worker",
        type=Path,
        default=_SCRIPT_DIR / "matrix_sonic_host_worker.py",
        help="Writer-gated physical get-up worker",
    )
    parser.add_argument(
        "--physical-recovery-initial-controller",
        choices=("host", "amp", "kungfu"),
        default=os.environ.get(
            "MATRIX_PHYSICAL_RECOVERY_INITIAL_CONTROLLER", "host"
        ),
        help="Physical policy that receives the first post-fall LowCmd lease",
    )
    parser.add_argument(
        "--physical-recovery-handoff",
        choices=("amp", "sonic"),
        default=os.environ.get("MATRIX_PHYSICAL_RECOVERY_HANDOFF", "amp"),
        help="Stabilize through AMP or hand a stable get-up pose directly to SONIC",
    )
    parser.add_argument(
        "--physical-recovery-python",
        default=sys.executable,
        help="Python interpreter containing NumPy and ONNX Runtime",
    )
    parser.add_argument(
        "--physical-recovery-resident-policies",
        action="store_true",
        default=os.environ.get(
            "MATRIX_PHYSICAL_RECOVERY_RESIDENT_POLICIES", "0"
        ).strip().lower()
        in {"1", "true", "yes", "on"},
        help="Keep SONIC and every recovery policy loaded; switch writer authority only",
    )
    parser.add_argument(
        "--locomotion-policy-manifest",
        type=Path,
        default=Path(
            os.environ.get(
                "MATRIX_BFM_SONIC_MANIFEST",
                _SCRIPT_DIR.parent
                / "config/runtime/policy-slots/bfm-sonic-teacher50k.json",
            )
        ),
        help=(
            "Locked optional locomotion-policy declaration; incomplete or "
            "unverified candidates remain visible but unavailable"
        ),
    )
    parser.add_argument(
        "--physical-recovery-execution-provider",
        choices=("cuda", "cpu"),
        default=os.environ.get(
            "MATRIX_PHYSICAL_RECOVERY_EXECUTION_PROVIDER", "cpu"
        ),
        help="ONNX execution provider used by all resident recovery policies",
    )
    parser.add_argument(
        "--physical-recovery-model",
        type=Path,
        help="Primary physical get-up ONNX model (required for physical mode)",
    )
    parser.add_argument(
        "--physical-recovery-fallback-model",
        action="append",
        default=[],
        type=Path,
        help="Optional physically continuous fallback ONNX (repeatable)",
    )
    parser.add_argument(
        "--physical-recovery-kungfu-model",
        type=Path,
        default=(
            Path(os.environ["MATRIX_KUNGFU_RECOVERY_MODEL"])
            if os.environ.get("MATRIX_KUNGFU_RECOVERY_MODEL")
            else None
        ),
        help="KungFuAthleteBot 154 -> 29 recovery ONNX",
    )
    parser.add_argument(
        "--physical-recovery-kungfu-motion",
        type=Path,
        default=(
            Path(os.environ["MATRIX_KUNGFU_RECOVERY_MOTION"])
            if os.environ.get("MATRIX_KUNGFU_RECOVERY_MOTION")
            else None
        ),
        help="KungFuAthleteBot 1307 reference NPZ",
    )
    parser.add_argument(
        "--physical-recovery-kungfu-model-sha256",
        default=os.environ.get("MATRIX_KUNGFU_RECOVERY_MODEL_SHA256"),
    )
    parser.add_argument(
        "--physical-recovery-kungfu-model-data-sha256",
        default=os.environ.get("MATRIX_KUNGFU_RECOVERY_MODEL_DATA_SHA256"),
    )
    parser.add_argument(
        "--physical-recovery-kungfu-motion-sha256",
        default=os.environ.get("MATRIX_KUNGFU_RECOVERY_MOTION_SHA256"),
    )
    parser.add_argument(
        "--physical-recovery-kungfu-reference-frame",
        type=int,
        default=int(
            os.environ.get("MATRIX_KUNGFU_RECOVERY_REFERENCE_FRAME", "0")
        ),
    )
    parser.add_argument(
        "--physical-recovery-kungfu-gain-scale",
        type=float,
        default=float(
            os.environ.get("MATRIX_KUNGFU_RECOVERY_GAIN_SCALE", "1.0")
        ),
    )
    parser.add_argument(
        "--physical-recovery-amp-config",
        type=Path,
        help="AMP zero-command dynamic-hold JSON (required for physical mode)",
    )
    parser.add_argument(
        "--physical-recovery-amp-model",
        type=Path,
        help="AMP zero-command dynamic-hold ONNX (required for physical mode)",
    )
    parser.add_argument("--physical-recovery-amp-config-sha256")
    parser.add_argument("--physical-recovery-amp-model-sha256")
    parser.add_argument(
        "--physical-recovery-fallback-after-seconds", type=float, default=10.0
    )
    parser.add_argument(
        "--physical-recovery-stable-hold-seconds", type=float, default=1.5
    )
    parser.add_argument(
        "--physical-recovery-policy-exit-hold-seconds",
        type=float,
        default=float(
            os.environ.get("MATRIX_PHYSICAL_RECOVERY_POLICY_EXIT_HOLD_SECONDS", "0")
        ),
        help=(
            "Optional terminal dwell before a stable resident recovery policy "
            "releases writer authority"
        ),
    )
    parser.add_argument(
        "--physical-recovery-timeout-seconds",
        type=float,
        default=90.0,
        help="Fail-closed deadline after the physical policy receives GO",
    )
    parser.add_argument(
        "--physical-recovery-sonic-prewarm-timeout-seconds",
        type=float,
        default=45.0,
        help="Deadline for replacement SONIC writer-free shadow readiness",
    )
    parser.add_argument(
        "--physical-recovery-sonic-full-control-timeout-seconds",
        type=float,
        default=10.0,
        help="Deadline from replacement first LowCmd to full SONIC policy control",
    )
    parser.add_argument(
        "--physical-recovery-control-socket",
        type=Path,
        default=Path(os.environ.get("XDG_RUNTIME_DIR", tempfile.gettempdir()))
        / f"matrix-sonic-recovery-{os.getuid()}-{os.getpid()}.sock",
    )
    parser.add_argument(
        "--physical-recovery-sonic-control-socket",
        type=Path,
        default=Path(os.environ.get("XDG_RUNTIME_DIR", tempfile.gettempdir()))
        / f"matrix-sonic-recovery-sonic-{os.getuid()}-{os.getpid()}.sock",
    )
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
            "ue-final-pov",
            "carla",
            "fixed",
        ),
        default="fixed",
    )
    parser.add_argument(
        "--game-ue-camera-state-file",
        type=Path,
        help="Supervised fresh PlayerCameraManager final-POV state",
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
    parser.add_argument("--game-world-id")
    parser.add_argument("--game-world-revision")
    parser.add_argument("--game-world-state-file", type=Path)
    parser.add_argument(
        "--game-world-checkpoint-seconds",
        type=float,
        default=0.75,
        help="Durable last-exit checkpoint interval for interactive game runs",
    )
    parser.add_argument(
        "--game-auto-respawn",
        action="store_true",
        help="On fall, save an upright resume pose and request a cold full-runtime reload",
    )
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
    scene_transform = manifest.get("scene_transform")
    if not isinstance(scene_transform, str):
        raise SystemExit("qualified physics model has no scene transform contract")
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
            scene_transform=scene_transform,
        )
        expected_manifest = _regular_json(
            expected_output / "manifest.json", "reproduced physics model manifest"
        )
        for field in (
            "pipeline_version",
            "scene_transform",
            "removed_environment_geoms",
        ):
            if manifest.get(field) != expected_manifest.get(field):
                raise SystemExit(
                    f"qualified physics model {field} contract is stale"
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


def _validate_game_fall_recovery(args: argparse.Namespace) -> None:
    """Keep both recovery implementations isolated to interactive game runs."""

    mode = getattr(args, "game_fall_recovery", "off")
    if mode == "off":
        return
    if mode not in {"sonic", "physical"}:
        raise SystemExit(f"unsupported game fall recovery mode: {mode}")
    if args.control_source != "game":
        raise SystemExit("SONIC fall recovery requires --control-source game")
    if bool(args.fail_on_fall):
        raise SystemExit("SONIC fall recovery conflicts with --fail-on-fall")
    if bool(args.qualified_runtime):
        raise SystemExit("qualified runtime requires fail-fast fall handling")
    timeout = float(getattr(args, "game_fall_recovery_timeout", 15.0))
    if not math.isfinite(timeout) or timeout <= 0.0:
        raise SystemExit(
            "--game-fall-recovery-timeout must be positive and finite"
        )
    if mode != "physical":
        return

    initial_controller = str(
        getattr(args, "physical_recovery_initial_controller", "host")
    )
    if initial_controller not in {"host", "amp", "kungfu"}:
        raise SystemExit(
            "--physical-recovery-initial-controller must be host, amp, or kungfu"
        )
    handoff = str(getattr(args, "physical_recovery_handoff", "amp"))
    if handoff not in {"amp", "sonic"}:
        raise SystemExit("--physical-recovery-handoff must be amp or sonic")
    resident_policies = bool(
        getattr(args, "physical_recovery_resident_policies", False)
    )
    execution_provider = str(
        getattr(args, "physical_recovery_execution_provider", "cpu")
    )
    if resident_policies:
        if handoff != "sonic":
            raise SystemExit(
                "resident physical recovery requires a direct policy-to-SONIC handoff"
            )
        if execution_provider != "cuda":
            raise SystemExit(
                "resident physical recovery requires CUDAExecutionProvider"
            )

    worker = getattr(args, "physical_recovery_worker", None)
    python = getattr(args, "physical_recovery_python", None)
    model = getattr(args, "physical_recovery_model", None)
    fallbacks = tuple(getattr(args, "physical_recovery_fallback_model", ()))
    amp_config = getattr(args, "physical_recovery_amp_config", None)
    amp_model = getattr(args, "physical_recovery_amp_model", None)
    if not isinstance(worker, Path) or not worker.is_file():
        raise SystemExit(f"physical recovery worker is missing: {worker}")
    if not python or not Path(python).is_file():
        raise SystemExit(f"physical recovery Python is missing: {python}")
    if not isinstance(model, Path) or not model.is_file():
        raise SystemExit(f"physical recovery model is missing: {model}")
    for fallback in fallbacks:
        if not isinstance(fallback, Path) or not fallback.is_file():
            raise SystemExit(
                f"physical recovery fallback model is missing: {fallback}"
            )
    for label, artifact in (
        ("AMP hold config", amp_config),
        ("AMP hold model", amp_model),
    ):
        if not isinstance(artifact, Path) or not artifact.is_file():
            raise SystemExit(f"physical recovery {label} is missing: {artifact}")
    for name in (
        "physical_recovery_amp_config_sha256",
        "physical_recovery_amp_model_sha256",
    ):
        digest = str(getattr(args, name, "") or "")
        if len(digest) != 64 or any(char not in "0123456789abcdef" for char in digest):
            raise SystemExit(f"--{name.replace('_', '-')} must be 64 lowercase hex")
    if initial_controller == "kungfu":
        kungfu_model = getattr(args, "physical_recovery_kungfu_model", None)
        kungfu_motion = getattr(args, "physical_recovery_kungfu_motion", None)
        for label, artifact in (
            ("KungFu recovery model", kungfu_model),
            ("KungFu recovery motion", kungfu_motion),
        ):
            if not isinstance(artifact, Path) or not artifact.is_file():
                raise SystemExit(f"physical recovery {label} is missing: {artifact}")
        assert isinstance(kungfu_model, Path)
        kungfu_model_data = kungfu_model.with_name(f"{kungfu_model.name}.data")
        if not kungfu_model_data.is_file():
            raise SystemExit(
                "physical recovery KungFu ONNX external data is missing: "
                f"{kungfu_model_data}"
            )
        for name in (
            "physical_recovery_kungfu_model_sha256",
            "physical_recovery_kungfu_model_data_sha256",
            "physical_recovery_kungfu_motion_sha256",
        ):
            digest = str(getattr(args, name, "") or "")
            if len(digest) != 64 or any(
                char not in "0123456789abcdef" for char in digest
            ):
                raise SystemExit(
                    f"--{name.replace('_', '-')} must be 64 lowercase hex"
                )
        reference_frame = int(args.physical_recovery_kungfu_reference_frame)
        if reference_frame < 0:
            raise SystemExit(
                "--physical-recovery-kungfu-reference-frame must be non-negative"
            )
        gain_scale = float(args.physical_recovery_kungfu_gain_scale)
        if not math.isfinite(gain_scale) or gain_scale <= 0.0:
            raise SystemExit(
                "--physical-recovery-kungfu-gain-scale must be positive and finite"
            )
    control_sockets = (
        getattr(args, "physical_recovery_control_socket", None),
        getattr(args, "physical_recovery_sonic_control_socket", None),
    )
    for control_socket in control_sockets:
        if not isinstance(control_socket, Path) or not control_socket.is_absolute():
            raise SystemExit("physical recovery control sockets must be absolute")
        if len(os.fsencode(control_socket)) >= 108:
            raise SystemExit(f"physical recovery control socket is too long: {control_socket}")
    if control_sockets[0] == control_sockets[1]:
        raise SystemExit("physical recovery control sockets must be distinct")
    for name in (
        "physical_recovery_fallback_after_seconds",
        "physical_recovery_stable_hold_seconds",
        "physical_recovery_timeout_seconds",
        "physical_recovery_sonic_prewarm_timeout_seconds",
    ):
        value = float(getattr(args, name, 0.0))
        if not math.isfinite(value) or value <= 0.0:
            raise SystemExit(f"--{name.replace('_', '-')} must be positive and finite")
    policy_exit_hold_s = float(
        getattr(args, "physical_recovery_policy_exit_hold_seconds", 0.0)
    )
    if not math.isfinite(policy_exit_hold_s) or policy_exit_hold_s < 0.0:
        raise SystemExit(
            "--physical-recovery-policy-exit-hold-seconds must be finite and "
            "non-negative"
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
    if args.game_camera_yaw_source in {
        "x11-core-gated",
        "x11-absolute",
        "ue-final-pov",
    }:
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


def _physical_foot_contact(simulator: Any) -> bool:
    """Read the same G1 foot-contact condition used by policy validation."""

    model = simulator.sim_env.mj_model
    data = simulator.sim_env.mj_data
    foot_body_ids: set[int] = set()
    for name in (
        "left_ankle_roll_link",
        "right_ankle_roll_link",
        "left_foot",
        "right_foot",
    ):
        try:
            body_id = int(model.body(name).id)
        except (KeyError, ValueError):
            continue
        if body_id > 0:
            foot_body_ids.add(body_id)
    if not foot_body_ids:
        return False
    robot_root_body_id = None
    for name in ("pelvis", "base"):
        try:
            candidate = int(model.body(name).id)
        except (KeyError, ValueError):
            continue
        if candidate > 0:
            robot_root_body_id = candidate
            break
    if robot_root_body_id is None:
        return False
    return has_external_foot_support(
        model,
        data,
        foot_body_ids=foot_body_ids,
        robot_root_body_id=robot_root_body_id,
    )


def _physical_ground_contact(simulator: Any) -> bool:
    """Return whether any robot link rests on an external support surface."""

    model = simulator.sim_env.mj_model
    data = simulator.sim_env.mj_data
    robot_root_body_id = None
    for name in ("pelvis", "base"):
        try:
            candidate = int(model.body(name).id)
        except (KeyError, ValueError):
            continue
        if candidate > 0:
            robot_root_body_id = candidate
            break
    if robot_root_body_id is None:
        return False
    return has_external_ground_support(
        model,
        data,
        robot_root_body_id=robot_root_body_id,
    )


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


def _snapshot_world_pose(snapshot: Any) -> WorldPose:
    try:
        qpos = snapshot.qpos
        return WorldPose(
            float(qpos[0]),
            float(qpos[1]),
            float(qpos[2]),
            _root_yaw_rad(qpos),
        )
    except (AttributeError, IndexError, TypeError, ValueError, WorldStateError) as exc:
        raise WorldStateError(f"snapshot does not contain a valid root pose: {exc}") from exc


def _snapshot_world_upright(snapshot: Any) -> bool:
    try:
        qvel = snapshot.qvel
        vertical_speed = float(qvel[2])
        roll_rate = float(qvel[3])
        pitch_rate = float(qvel[4])
        return bool(
            not bool(snapshot.fall_detected)
            and float(snapshot.qpos[2]) >= _WORLD_SAFE_MIN_ROOT_Z
            and _root_up_z(snapshot.qpos) >= _WORLD_SAFE_MIN_ROOT_UP_Z
            and math.isfinite(vertical_speed)
            and abs(vertical_speed) <= _WORLD_SAFE_MAX_VERTICAL_SPEED_M_S
            and math.isfinite(roll_rate)
            and abs(roll_rate) <= _WORLD_SAFE_MAX_TILT_RATE_RAD_S
            and math.isfinite(pitch_rate)
            and abs(pitch_rate) <= _WORLD_SAFE_MAX_TILT_RATE_RAD_S
        )
    except (AttributeError, IndexError, TypeError, ValueError):
        return False


class _GameWorldStateRuntime:
    """Checkpoint semantic root poses without serializing dynamic MuJoCo state."""

    def __init__(
        self,
        *,
        path: Path,
        world_id: str,
        world_revision: str,
        checkpoint_seconds: float,
    ) -> None:
        interval = float(checkpoint_seconds)
        if not math.isfinite(interval) or not 0.1 <= interval <= 60.0:
            raise WorldStateError(
                "game world checkpoint interval must be finite and in [0.1, 60]"
            )
        self.store = WorldStateStore(
            path,
            world_id=world_id,
            world_revision=world_revision,
        )
        self.state = self.store.load()
        self.checkpoint_seconds = interval
        self.next_checkpoint_s = 0.0
        self.checkpoint_count = 0
        self.last_error: str | None = self.store.load_error
        self.last_checkpoint_monotonic_s: float | None = None

    def checkpoint(
        self,
        snapshot: Any,
        *,
        now_s: float,
        force: bool = False,
        required: bool = False,
    ) -> bool:
        now = float(now_s)
        if not math.isfinite(now) or now < 0.0:
            raise WorldStateError("checkpoint monotonic time is invalid")
        if not force and now < self.next_checkpoint_s:
            return False
        try:
            pose = _snapshot_world_pose(snapshot)
            state = self.state.checkpoint(
                pose,
                upright=_snapshot_world_upright(snapshot),
            )
            self.store.save(state)
        except WorldStateError as exc:
            self.last_error = str(exc)
            self.next_checkpoint_s = now + self.checkpoint_seconds
            if required:
                raise
            return False
        self.state = state
        self.checkpoint_count += 1
        self.last_error = None
        self.last_checkpoint_monotonic_s = now
        self.next_checkpoint_s = now + self.checkpoint_seconds
        return True

    def telemetry(self) -> dict[str, object]:
        return {
            "enabled": True,
            "path": str(self.store.path),
            "world_id": self.store.world_id,
            "world_revision": self.store.world_revision,
            "load_status": self.store.load_status,
            "load_error": self.store.load_error,
            "checkpoint_count": self.checkpoint_count,
            "checkpoint_seconds": self.checkpoint_seconds,
            "last_checkpoint_monotonic_s": self.last_checkpoint_monotonic_s,
            "last_error": self.last_error,
            "resume_source": self.state.resume_source,
            "has_last_exit": self.state.last_exit is not None,
            "has_home": self.state.home is not None,
            "teleport_point_count": len(self.state.teleport_points),
            "frame": "matrix_mj_world",
            "units": "m",
        }


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


class _GameFallRecoveryGate:
    """Keep interactive control neutral while SONIC recovers its own policy.

    SONIC's public fall flag is session-sticky, so it cannot identify the end
    of one recovery or the beginning of a later fall.  The current fall level
    therefore mirrors SONIC's own exact root-height condition (``z < 0.2``).
    Interactive game runs additionally debounce a low, strongly tilted base
    pose because a stable side fall can remain above SONIC's height threshold
    and never set the sticky flag.  This pose trigger is local to this gate;
    recovery never clears or rewrites SONIC's historical fall flag, and the
    qualification/fail-fast path continues to use that flag unchanged.
    """

    FALL_HEIGHT_M = 0.2
    POSE_TRIGGER_HEIGHT_M = 0.45
    POSE_TRIGGER_UP_Z = 0.5
    POSE_TRIGGER_HOLD_S = 0.35
    UPRIGHT_HEIGHT_M = 0.65
    UPRIGHT_UP_Z = 0.85
    STABLE_HOLD_S = 1.0
    KNEEL_TWO_LEGS_MODE = 5
    KNEEL_HEIGHT_M = 0.4
    KNEEL_STAGE_S = 2.0
    RETRY_PERIOD_S = 6.0

    def __init__(self, *, timeout_s: float = 15.0) -> None:
        timeout = float(timeout_s)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError("fall recovery timeout must be positive and finite")
        self.timeout_s = timeout
        self.recovering = False
        self.current_fallen = False
        self.episodes = 0
        self.recoveries = 0
        self.started_at_s: float | None = None
        self.pose_candidate_since_s: float | None = None
        self.stable_since_s: float | None = None
        self.last_duration_s: float | None = None
        self.timed_out = False
        self.pose_candidate = False
        self.last_entry_source: str | None = None
        self.native_mode = SONIC_IDLE_MODE
        self.target_height = -1.0

    def observe(self, snapshot: Any, *, now_s: float) -> str | None:
        """Observe one control-frame snapshot and return a transition name."""

        now = float(now_s)
        if not math.isfinite(now) or now < 0.0:
            raise ValueError("fall recovery time must be non-negative and finite")
        try:
            root_z = float(snapshot.qpos[2])
            root_up_z = _root_up_z(snapshot.qpos)
        except (AttributeError, IndexError, TypeError, ValueError) as exc:
            raise ValueError("fall recovery requires a valid root pose") from exc
        if not math.isfinite(root_z) or not math.isfinite(root_up_z):
            raise ValueError("fall recovery root pose must be finite")

        self.current_fallen = root_z < self.FALL_HEIGHT_M
        self.pose_candidate = (
            root_z < self.POSE_TRIGGER_HEIGHT_M
            and root_up_z < self.POSE_TRIGGER_UP_Z
        )
        if self.pose_candidate:
            if self.pose_candidate_since_s is None:
                self.pose_candidate_since_s = now
            elif now < self.pose_candidate_since_s:
                raise ValueError("fall recovery pose-candidate time regressed")
        else:
            self.pose_candidate_since_s = None

        native_trigger = self.current_fallen and bool(
            getattr(snapshot, "fall_detected", False)
        )
        pose_trigger = (
            self.pose_candidate_since_s is not None
            and now - self.pose_candidate_since_s >= self.POSE_TRIGGER_HOLD_S
        )
        transition = None
        if (
            not self.recovering
            and (native_trigger or pose_trigger)
        ):
            self.recovering = True
            self.episodes += 1
            self.started_at_s = now
            self.pose_candidate_since_s = None
            self.stable_since_s = None
            self.timed_out = False
            self.last_entry_source = (
                "sonic_fall_detected" if native_trigger else "pose_debounce"
            )
            transition = "entered"

        if not self.recovering:
            return transition

        assert self.started_at_s is not None
        if now < self.started_at_s:
            raise ValueError("fall recovery time regressed")
        episode_elapsed_s = now - self.started_at_s
        retry_phase_s = episode_elapsed_s % self.RETRY_PERIOD_S
        if retry_phase_s < self.KNEEL_STAGE_S:
            self.native_mode = self.KNEEL_TWO_LEGS_MODE
            self.target_height = self.KNEEL_HEIGHT_M
        else:
            self.native_mode = SONIC_IDLE_MODE
            self.target_height = -1.0
        ready = _GameSonicReadinessGate.snapshot_ready(snapshot)
        upright = (
            not self.current_fallen
            and root_z >= self.UPRIGHT_HEIGHT_M
            and root_up_z >= self.UPRIGHT_UP_Z
            and ready
            and self.native_mode == SONIC_IDLE_MODE
        )
        if upright:
            if self.stable_since_s is None:
                self.stable_since_s = now
            elif now < self.stable_since_s:
                raise ValueError("fall recovery stable time regressed")
            if now - self.stable_since_s >= self.STABLE_HOLD_S:
                self.last_duration_s = now - self.started_at_s
                self.recovering = False
                self.recoveries += 1
                self.started_at_s = None
                self.stable_since_s = None
                self.timed_out = False
                self.native_mode = SONIC_IDLE_MODE
                self.target_height = -1.0
                return "recovered"
        else:
            self.stable_since_s = None

        if now - self.started_at_s >= self.timeout_s:
            self.timed_out = True
        return transition

    def status(self, *, now_s: float) -> dict[str, object]:
        now = float(now_s)
        active_elapsed_s = (
            max(0.0, now - self.started_at_s)
            if self.recovering and self.started_at_s is not None
            else 0.0
        )
        stable_elapsed_s = (
            max(0.0, now - self.stable_since_s)
            if self.recovering and self.stable_since_s is not None
            else 0.0
        )
        pose_candidate_elapsed_s = (
            max(0.0, now - self.pose_candidate_since_s)
            if self.pose_candidate_since_s is not None
            else 0.0
        )
        state = (
            "recovering_timeout"
            if self.recovering and self.timed_out
            else "recovering"
            if self.recovering
            else "monitoring"
        )
        return {
            "mode": "sonic",
            "state": state,
            "policy_command": "KNEEL_TWO_LEGS_TO_IDLE",
            "native_mode": self.native_mode,
            "target_height": self.target_height,
            "current_fall_detected": self.current_fallen,
            "pose_recovery_candidate": self.pose_candidate,
            "pose_candidate_elapsed_s": round(pose_candidate_elapsed_s, 3),
            "last_entry_source": self.last_entry_source,
            "episodes": self.episodes,
            "recoveries": self.recoveries,
            "active_elapsed_s": round(active_elapsed_s, 3),
            "stable_elapsed_s": round(stable_elapsed_s, 3),
            "last_duration_s": (
                round(self.last_duration_s, 3)
                if self.last_duration_s is not None
                else None
            ),
            "timeout_s": self.timeout_s,
            "timed_out": self.timed_out,
            "fall_height_m": self.FALL_HEIGHT_M,
            "pose_trigger_height_m": self.POSE_TRIGGER_HEIGHT_M,
            "pose_trigger_up_z": self.POSE_TRIGGER_UP_Z,
            "pose_trigger_hold_s": self.POSE_TRIGGER_HOLD_S,
            "upright_height_m": self.UPRIGHT_HEIGHT_M,
            "upright_up_z": self.UPRIGHT_UP_Z,
            "stable_hold_s": self.STABLE_HOLD_S,
            "kneel_stage_s": self.KNEEL_STAGE_S,
            "retry_period_s": self.RETRY_PERIOD_S,
            "recovered_requires_neutral": True,
        }


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


def _effective_game_camera_yaw_offset_deg(
    *,
    source: str,
    configured_offset_deg: float,
    initial_root_yaw_rad: float | None,
) -> float:
    """Map absolute provider yaw into SONIC's initial-root-relative frame."""

    configured = float(configured_offset_deg)
    if not math.isfinite(configured):
        raise ValueError("configured camera yaw offset must be finite")
    if source != "ue-final-pov":
        return configured
    if initial_root_yaw_rad is None or not math.isfinite(initial_root_yaw_rad):
        raise ValueError("UE final-POV yaw requires a finite initial root yaw")
    return configured - math.degrees(initial_root_yaw_rad)


def _game_control_status_fields(
    args: argparse.Namespace,
    *,
    applied_camera_yaw_offset_deg: float | None = None,
    initial_root_yaw_rad: float | None = None,
) -> dict[str, object]:
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
    elif source == "ue-final-pov":
        yaw_observation = "ue_player_camera_manager_final_pov_state"
        yaw_truth_scope = "player_camera_manager_final_pov"
        button_gate_truth_scope = (
            "xquerypointer_core_level_or_xi2_raw_button_edges"
        )
    else:
        yaw_observation = "carla_spectator_rpc_write_readback"
        yaw_truth_scope = "carla_spectator_not_verified_final_view"
        button_gate_truth_scope = "not_applicable_carla_rpc"
    effective_input_source = args.game_input_source
    if (
        source not in {"carla", "ue-final-pov"}
        and effective_input_source == "auto"
    ):
        effective_input_source = "keyboard"
    applied_mouse_scale = args.game_applied_mouse_speed_scale
    effective_mouse_sensitivity = (
        args.game_mouse_sensitivity_deg * applied_mouse_scale
    )
    if source in {"x11-mirror", "x11-core-gated"}:
        sensitivity_units = "degrees_per_xi2_raw_unit"
    elif source == "x11-absolute":
        sensitivity_units = "degrees_per_x11_root_pixel"
    elif source == "ue-final-pov":
        sensitivity_units = "absolute_degrees_from_player_camera_manager_final_pov"
    else:
        sensitivity_units = "degrees_per_unobserved_input_unit"
    configured_camera_yaw_offset_deg = args.game_camera_yaw_offset_deg
    effective_camera_yaw_offset_deg = (
        configured_camera_yaw_offset_deg
        if applied_camera_yaw_offset_deg is None
        else applied_camera_yaw_offset_deg
    )
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
        "fall_recovery_mode": getattr(args, "game_fall_recovery", "off"),
        "fall_recovery_timeout_s": getattr(
            args, "game_fall_recovery_timeout", 15.0
        ),
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
        "experimental": source
        in {"x11-core-gated", "x11-absolute", "ue-final-pov"},
        "ue_camera_state_file": os.fspath(args.game_ue_camera_state_file)
        if getattr(args, "game_ue_camera_state_file", None) is not None
        else None,
        "camera_yaw_sign": args.game_camera_yaw_sign,
        "camera_yaw_offset_deg": effective_camera_yaw_offset_deg,
        "camera_yaw_offset_configured_deg": configured_camera_yaw_offset_deg,
        "camera_yaw_initial_root_compensation_deg": (
            effective_camera_yaw_offset_deg - configured_camera_yaw_offset_deg
        ),
        "initial_root_yaw_rad_for_camera": initial_root_yaw_rad,
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
        # Pointer integration and CARLA do not prove the cooked final view.
        # ue-final-pov names PlayerCameraManager's final POV, but remains
        # experimental until live visual/cardinal acceptance succeeds.
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
    scenario_completed = termination_reason == "scenario_complete"
    # A bounded max-seconds exit is a formal runtime qualification.  An
    # in-process scenario completion is instead a harness-owned acceptance
    # boundary (recovery explicitly forbids --qualified-runtime), so it must not
    # inherit the formal runtime-verification requirement.
    attempted = max_seconds > 0.0 and not scenario_completed
    if attempted and not runtime_verified:
        acceptance_failures.append("runtime_not_verified_for_qualification")
    if attempted and termination_reason == "signal":
        acceptance_failures.append("run_interrupted")
    if termination_reason == "unknown":
        acceptance_failures.append("unknown_termination")
    completed = termination_reason in {"max_seconds", "scenario_complete"}
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
        turning = (
            not moving
            and locomotion_mode == SONIC_SLOW_WALK_MODE
            and speed_value <= 1e-6
            and math.hypot(movement_values[0], movement_values[1]) <= 1e-6
        )
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
                mode=locomotion_mode if moving or turning else 0,
                movement=movement_values if moving else [0.0, 0.0, 0.0],
                facing=facing_values,
                speed=(speed_value if moving else 0.0 if turning else -1.0),
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
        turning = command.mode == "turn"
        if turning:
            if has_speed or has_direction:
                raise ValueError("turn-only game command must not translate")
            if command.locomotion_mode != SONIC_SLOW_WALK_MODE:
                raise ValueError("turn-only game command must use native SLOW_WALK")
            if command.safe_stop:
                raise ValueError("turn-only game command cannot be a safe stop")
        elif not moving:
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

    def send_recovery_posture(
        self,
        *,
        locomotion_mode: int,
        height: float,
        facing: tuple[float, float, float],
    ) -> None:
        """Send the native kneel/stand sequence used by SONIC's gamepad path."""

        if locomotion_mode not in {0, 5}:
            raise ValueError("fall recovery only permits native IDLE or KNEEL_TWO_LEGS")
        facing_values = [float(value) for value in facing]
        if len(facing_values) != 3 or not all(
            math.isfinite(value) for value in facing_values
        ):
            raise ValueError("fall recovery facing must contain three finite values")
        height_value = float(height)
        if locomotion_mode == 0:
            if height_value != -1.0:
                raise ValueError("native IDLE recovery must use default height -1")
        elif not 0.2 <= height_value <= 0.8:
            raise ValueError("native KNEEL_TWO_LEGS height must be in [0.2, 0.8]")
        self._socket.send(
            self._build_command_message(
                start=True,
                stop=False,
                planner=True,
                delta_heading=None,
            )
        )
        self._socket.send(
            self._build_planner_message(
                mode=locomotion_mode,
                movement=[0.0, 0.0, 0.0],
                facing=facing_values,
                speed=-1.0,
                height=height_value,
            )
        )

    def request_deploy_stop(self) -> None:
        """Stop only the current deploy while retaining this ZMQ client.

        The physical-recovery handoff starts a new deploy on the same endpoint;
        game input and UE therefore keep their original lifetimes and this
        socket must remain available for the new receiver generation.
        """

        stop_message = self._build_command_message(
            start=False,
            stop=True,
            planner=True,
            delta_heading=None,
        )
        # The native deploy binary exits through its ZMQ stop state and does
        # not install SIGTERM handlers. Repeat the native stop frame briefly.
        for _ in range(3):
            self._socket.send(stop_message)
            time.sleep(0.02)

    def close(self) -> None:
        stop_error = None
        try:
            self.request_deploy_stop()
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


class GameCommandRuntime:
    """Execute typed ESC-panel commands over one inherited private socketpair."""

    def __init__(
        self,
        connection: socket.socket,
        world: _GameWorldStateRuntime | None,
        *,
        policy_slots: Any | None = None,
    ) -> None:
        self.connection = connection
        self.connection.setblocking(False)
        self.world = world
        self.session: str | None = None
        self.last_sequence = 0
        self.request_ids: set[str] = set()
        self.requests_received = 0
        self.commands_executed = 0
        self.protocol_errors = 0
        self.rejected_commands = 0
        self.response_errors = 0
        self.restart_requested = False
        self.last_response: dict[str, object] | None = None
        self.policy_slots = policy_slots
        self.pending_policy_request: GameCommandRequest | None = None
        self.policy_changes_executed = 0

    def _send(self, response: GameCommandResponse) -> None:
        payload = encode_command_response(response)
        try:
            sent = self.connection.send(payload)
        except (BlockingIOError, OSError) as exc:
            self.response_errors += 1
            raise RuntimeError(f"cannot send game command response: {exc}") from exc
        if sent != len(payload):
            self.response_errors += 1
            raise RuntimeError(
                f"partial game command response: sent {sent}/{len(payload)}"
            )
        self.last_response = response.to_mapping()

    @staticmethod
    def _response(
        request: GameCommandRequest,
        *,
        ok: bool,
        code: str,
        message: str,
        restart_required: bool = False,
        data: dict[str, object] | None = None,
    ) -> GameCommandResponse:
        return GameCommandResponse(
            session=request.session,
            sequence=request.sequence,
            request_id=request.request_id,
            ok=ok,
            code=code,
            message=message,
            restart_required=restart_required,
            data=data,
        )

    def _validate_identity(self, request: GameCommandRequest) -> str | None:
        if self.session is None:
            self.session = request.session
        elif request.session != self.session:
            return "command session changed"
        if request.sequence <= self.last_sequence:
            return "command sequence did not increase"
        self.last_sequence = request.sequence
        if request.request_id in self.request_ids:
            return "command request_id was already used"
        self.request_ids.add(request.request_id)
        if len(self.request_ids) > 256:
            # Sequences already prevent replay; keep only a bounded diagnostic set.
            self.request_ids = {request.request_id}
        return None

    def poll(self, *, current_pose: WorldPose, command_allowed: bool) -> bool:
        if self.restart_requested:
            return True
        if self.pending_policy_request is not None:
            if self.policy_slots is None:
                raise RuntimeError("pending policy selection lost its coordinator")
            request = self.pending_policy_request
            result = self.policy_slots.poll_policy_slot_assignment(
                request.request_id
            )
            if result is None:
                return False
            ok, code, message, loadout = result
            self.pending_policy_request = None
            if ok:
                self.commands_executed += 1
                self.policy_changes_executed += 1
            else:
                self.rejected_commands += 1
            self._send(
                self._response(
                    request,
                    ok=ok,
                    code=code,
                    message=message,
                    data={"strategy_loadout": loadout},
                )
            )
            return False
        for _ in range(16):
            try:
                payload = self.connection.recv(MAX_COMMAND_PACKET_BYTES + 1)
            except BlockingIOError:
                break
            except OSError as exc:
                raise RuntimeError(f"game command channel failed: {exc}") from exc
            if not payload:
                raise EOFError("game command provider closed its channel")
            self.requests_received += 1
            try:
                request = decode_command_request(payload)
            except CommandProtocolError as exc:
                self.protocol_errors += 1
                # A packet without a valid request identity cannot receive a
                # trustworthy correlated response.  Keeping the channel alive
                # would leave the provider's single in-flight request pending
                # forever and would also permit traffic after protocol drift.
                # Escalate through the supervised runtime boundary instead; its
                # cleanup closes the socket and lets the provider report the
                # already-sent command as outcome-unknown without retrying it.
                raise RuntimeError(
                    f"invalid game command request: {exc}"
                ) from exc
            identity_error = self._validate_identity(request)
            if identity_error is not None:
                self.protocol_errors += 1
                self._send(
                    self._response(
                        request,
                        ok=False,
                        code="E_PROTOCOL_IDENTITY",
                        message=identity_error,
                    )
                )
                continue
            if not command_allowed:
                self.rejected_commands += 1
                self._send(
                    self._response(
                        request,
                        ok=False,
                        code="E_NOT_PAUSED",
                        message="Open ESC and wait for a neutral frame before commands",
                    )
                )
                continue
            if isinstance(request.command, PolicySlotAssignment):
                if self.policy_slots is None:
                    self.rejected_commands += 1
                    self._send(
                        self._response(
                            request,
                            ok=False,
                            code="E_POLICY_UNAVAILABLE",
                            message="Resident policy slots are unavailable for this run",
                        )
                    )
                    continue
                try:
                    loadout = self.policy_slots.request_policy_slot_assignment(
                        request.command,
                        transition_id=request.request_id,
                    )
                except CommandExecutionError as exc:
                    self.rejected_commands += 1
                    self._send(
                        self._response(
                            request,
                            ok=False,
                            code=exc.code,
                            message=exc.message,
                            data={
                                "strategy_loadout": (
                                    self.policy_slots.strategy_loadout_mapping()
                                )
                            },
                        )
                    )
                    continue
                if loadout is None:
                    self.pending_policy_request = request
                    return False
                self.commands_executed += 1
                self.policy_changes_executed += 1
                self._send(
                    self._response(
                        request,
                        ok=True,
                        code="OK_POLICY_SLOT_ASSIGNED",
                        message=(
                            f"Assigned {request.command.policy_id} to "
                            f"{request.command.slot}"
                        ),
                        data={"strategy_loadout": loadout},
                    )
                )
                continue
            if self.world is None:
                self.rejected_commands += 1
                self._send(
                    self._response(
                        request,
                        ok=False,
                        code="E_WORLD_UNAVAILABLE",
                        message="Persistent world commands are unavailable for this run",
                    )
                )
                continue
            try:
                effect = execute_command(
                    request.command,
                    state=self.world.state,
                    current_pose=current_pose,
                    now_unix_ns=time.time_ns(),
                )
                self.world.store.save(effect.state)
            except CommandExecutionError as exc:
                self.rejected_commands += 1
                self._send(
                    self._response(
                        request,
                        ok=False,
                        code=exc.code,
                        message=exc.message,
                    )
                )
                continue
            except WorldStateError as exc:
                self.rejected_commands += 1
                self.world.last_error = str(exc)
                self._send(
                    self._response(
                        request,
                        ok=False,
                        code="E_STATE_PERSIST",
                        message="Could not persist the command result",
                    )
                )
                continue
            self.world.state = effect.state
            self.world.last_error = None
            self.commands_executed += 1
            self._send(
                self._response(
                    request,
                    ok=True,
                    code=effect.code,
                    message=effect.message,
                    restart_required=effect.restart_required,
                    data=dict(effect.data),
                )
            )
            if effect.restart_required:
                self.restart_requested = True
                return True
        return False

    def telemetry(self) -> dict[str, object]:
        return {
            "enabled": True,
            "session_bound": self.session is not None,
            "last_sequence": self.last_sequence,
            "requests_received": self.requests_received,
            "commands_executed": self.commands_executed,
            "policy_changes_executed": self.policy_changes_executed,
            "policy_change_pending": self.pending_policy_request is not None,
            "protocol_errors": self.protocol_errors,
            "rejected_commands": self.rejected_commands,
            "response_errors": self.response_errors,
            "restart_requested": self.restart_requested,
            "last_response": self.last_response,
        }

    def close(self) -> None:
        self.connection.close()


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
        self._expected_exit_pids: set[int] = set()
        self._active_deploy: subprocess.Popen[bytes] | None = None
        self._active_recovery_policy: subprocess.Popen[bytes] | None = None
        self._deploy_generation = 0
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
        extra_pass_fds: tuple[int, ...] = (),
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
        pass_fds = tuple(dict.fromkeys((*self.pass_fds, *extra_pass_fds)))
        for descriptor in pass_fds:
            os.fstat(descriptor)
        process = subprocess.Popen(
            guarded_command,
            cwd=cwd,
            env=self.env,
            pass_fds=pass_fds,
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
        ue_camera_state_file: Path | None = None,
        mouse_settings_file: Path | None = None,
        applied_mouse_profile: str = "local",
        applied_mouse_speed_scale: float = 1.0,
        restart_request_file: Path | None = None,
        restart_capability_file: Path | None = None,
        restart_launcher_pid: int | None = None,
        command_fd: int | None = None,
        strategy_loadout_json: str | None = None,
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
        if ue_camera_state_file is not None:
            command.extend(("--ue-camera-state-file", str(ue_camera_state_file)))
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
        if strategy_loadout_json is not None:
            command.extend(("--strategy-loadout-json", strategy_loadout_json))
        extra_pass_fds: tuple[int, ...] = ()
        if command_fd is not None:
            if isinstance(command_fd, bool) or not isinstance(command_fd, int):
                raise ValueError("game command fd must be an integer")
            os.fstat(command_fd)
            command.extend(("--game-command-fd", str(command_fd)))
            extra_pass_fds = (command_fd,)
        return self._start(
            "game-input",
            command,
            script.parent.parent,
            exec_command=True,
            extra_pass_fds=extra_pass_fds,
        )

    @property
    def deploy_generation(self) -> int:
        return self._deploy_generation

    @staticmethod
    def _alive(process: subprocess.Popen[bytes] | None) -> bool:
        return process is not None and _peek_child_returncode(process) is None

    def deploy_alive(self) -> bool:
        return self._alive(self._active_deploy)

    def deploy_pid(self) -> int | None:
        return self._active_deploy.pid if self._active_deploy is not None else None

    def recovery_policy_alive(self) -> bool:
        return self._alive(self._active_recovery_policy)

    def begin_deploy_stop(self) -> None:
        """Authorize only the active deploy's expected exit; send no signal."""

        if self._active_deploy is not None:
            self._expected_exit_pids.add(self._active_deploy.pid)

    def begin_recovery_policy_stop(self) -> None:
        """Authorize only the temporary policy worker's expected exit."""

        if self._active_recovery_policy is not None:
            self._expected_exit_pids.add(self._active_recovery_policy.pid)

    def start_deploy(
        self,
        *,
        interface: str,
        zmq_port: int,
        writer_control_socket: Path | None = None,
        physical_reentry: bool = False,
    ) -> int:
        if self.deploy_alive():
            raise RuntimeError("cannot start a second live SONIC deploy")
        deploy_root = self.sonic_root / "gear_sonic_deploy"
        command = [
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
            # TensorRT 10.16 produces planner roots that diverge from the
            # source ONNX graph, including upside-down IDLE and locomotion
            # generations. Keep policy/encoder on TensorRT, but use the
            # parity-verified native ONNX planner for every deploy generation.
            "--planner-backend",
            "onnx",
            "--input-type",
            "zmq_manager",
            "--output-type",
            "all",
            "--zmq-host",
            "localhost",
            "--zmq-port",
            str(zmq_port),
            "--disable-crc-check",
        ]
        if writer_control_socket is not None:
            command.extend(("--writer-control-socket", str(writer_control_socket)))
        if physical_reentry:
            if writer_control_socket is None:
                raise RuntimeError("physical SONIC re-entry requires a writer gate")
            command.extend(
                (
                    "--writer-reentry",
                    "--writer-reentry-hold-seconds",
                    "0.5",
                    "--writer-reentry-align-seconds",
                    "5.0",
                    # Keep the predecessor's exact physical command during
                    # warmup. Shadow admission and the later 6 s target blend
                    # own the SONIC transition.  The longer window keeps the
                    # joint-4 policy burst inside the existing 1.25 rad
                    # controller envelope across the observed spread of
                    # upright KungFu terminal poses, without relaxing that
                    # safety gate; even a 1% stand bridge has destabilized
                    # marginal but upright poses.
                    "--writer-reentry-align-fraction",
                    "0.0",
                    "--writer-reentry-settle-seconds",
                    "1.0",
                    "--writer-reentry-blend-seconds",
                    "6.0",
                )
            )
        self._start(
            "deploy",
            command,
            deploy_root,
            exec_command=True,
        )
        self._active_deploy = self.children[-1][1]
        self._deploy_generation += 1
        return self._deploy_generation

    def start_recovery_policy(
        self,
        python: str,
        worker: Path,
        *,
        interface: str,
        control_socket: Path,
        model: Path,
        fallback_models: tuple[Path, ...],
        fallback_after_s: float,
        amp_config: Path,
        amp_model: Path,
        amp_config_sha256: str,
        amp_model_sha256: str,
        kungfu_model: Path | None = None,
        kungfu_motion: Path | None = None,
        kungfu_model_sha256: str | None = None,
        kungfu_model_data_sha256: str | None = None,
        kungfu_motion_sha256: str | None = None,
        kungfu_reference_frame: int = 0,
        kungfu_gain_scale: float = 1.0,
        initial_controller: str = "host",
        execution_provider: str = "cpu",
    ) -> int:
        if self.recovery_policy_alive():
            raise RuntimeError("physical recovery policy is already alive")
        command = [
            python,
            "-u",
            str(worker),
            "--model",
            str(model),
            "--interface",
            interface,
            "--control-socket",
            str(control_socket),
            "--initial-controller",
            initial_controller,
            "--fallback-after-seconds",
            str(fallback_after_s),
            "--execution-provider",
            execution_provider,
            "--amp-hold-config",
            str(amp_config),
            "--amp-hold-model",
            str(amp_model),
            "--amp-hold-config-sha256",
            amp_config_sha256,
            "--amp-hold-model-sha256",
            amp_model_sha256,
        ]
        for fallback in fallback_models:
            command.extend(("--fallback-model", str(fallback)))
        kungfu_arguments = (
            kungfu_model,
            kungfu_motion,
            kungfu_model_sha256,
            kungfu_model_data_sha256,
            kungfu_motion_sha256,
        )
        if any(value is not None for value in kungfu_arguments):
            if not all(value is not None for value in kungfu_arguments):
                raise RuntimeError("incomplete KungFu recovery worker arguments")
            command.extend(
                (
                    "--kungfu-model",
                    str(kungfu_model),
                    "--kungfu-motion",
                    str(kungfu_motion),
                    "--kungfu-model-sha256",
                    str(kungfu_model_sha256),
                    "--kungfu-model-data-sha256",
                    str(kungfu_model_data_sha256),
                    "--kungfu-motion-sha256",
                    str(kungfu_motion_sha256),
                    "--kungfu-reference-frame",
                    str(kungfu_reference_frame),
                    "--kungfu-gain-scale",
                    str(kungfu_gain_scale),
                )
            )
        pid = self._start(
            "recovery-policy",
            command,
            worker.parent.parent,
            exec_command=True,
        )
        self._active_recovery_policy = self.children[-1][1]
        return pid

    def failed_child(self) -> tuple[str, int] | None:
        if self._stopping:
            return None
        for name, process in self.children:
            if process.pid in self._expected_exit_pids:
                continue
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


class _RecoveryWorkerControl:
    """Own the local writer-gate socket; the worker owns only policy/DDS I/O."""

    SCHEMAS = {
        "matrix.sonic_host_worker.control.v1",
        "matrix.sonic_amp_worker.control.v1",
    }

    def __init__(
        self,
        path: Path,
        *,
        require_resident_attestation: bool = False,
    ) -> None:
        self.path = path
        self.require_resident_attestation = bool(require_resident_attestation)
        self.listener: socket.socket | None = None
        self.connection: socket.socket | None = None
        self.ready = False
        self.first_write = False
        self.amp_hold_first_write = False
        self.joint_hold_first_write = False
        self.stopped = False
        self.paused = False
        self.pause_sent = False
        self.execution_provider: str | None = None
        self.resident_policies: list[dict[str, Any]] = []
        self.registered_policy_ids: list[str] = []
        self.initial_policy_id: str | None = None
        self.selected_policy_id: str | None = None
        self.policy_selection_transition_id: str | None = None
        self.policy_selection_requested_id: str | None = None
        self.last_policy_selection: dict[str, Any] | None = None
        self.last_policy_selection_rejection: dict[str, Any] | None = None
        self.models_loaded_once = False
        self.models_warmed = False
        self.last_event: str | None = None
        self.last_status: dict[str, Any] | None = None
        self.fallback_due = False
        self.last_fallback_due: dict[str, Any] | None = None
        self.last_policy_switch: dict[str, Any] | None = None
        self.policy_switch_first_writes: list[dict[str, Any]] = []
        self.error: str | None = None
        self.events_received = 0
        self.expected_peer_pid: int | None = None
        self.peer_pid: int | None = None
        self.peer_pid_mismatches = 0
        self.last_packet_monotonic: float | None = None
        self.completed_episodes: list[dict[str, Any]] = []
        self.episode_counter = 0
        self.episode_id: int | None = None
        self.episode_started_monotonic: float | None = None
        self.go_sent = False
        self.stop_sent = False
        self.amp_hold_sent = False
        self.joint_hold_sent = False
        self.hold_transition_counter = 0
        self.hold_transition_id: str | None = None
        self.hold_kind: str | None = None
        self.superseded_fallback_due_events: list[dict[str, Any]] = []
        self.advance_transition_counter = 0
        self.advance_transition_id: str | None = None
        self.policy_switch_accepted = False
        self.command_history: list[dict[str, Any]] = []

    def _episode_snapshot(self) -> dict[str, Any]:
        return {
            "ready_no_writer": self.ready,
            "first_write": self.first_write,
            "amp_hold_first_write": self.amp_hold_first_write,
            "joint_hold_first_write": self.joint_hold_first_write,
            "stopped": self.stopped,
            "paused": self.paused,
            "pause_sent": self.pause_sent,
            "execution_provider": self.execution_provider,
            "resident_policies": list(self.resident_policies),
            "registered_policy_ids": list(self.registered_policy_ids),
            "initial_policy_id": self.initial_policy_id,
            "selected_policy_id": self.selected_policy_id,
            "last_policy_selection": self.last_policy_selection,
            "last_policy_selection_rejection": self.last_policy_selection_rejection,
            "models_loaded_once": self.models_loaded_once,
            "models_warmed": self.models_warmed,
            "last_event": self.last_event,
            "events_received": self.events_received,
            "expected_peer_pid": self.expected_peer_pid,
            "peer_pid": self.peer_pid,
            "status": self.last_status,
            "fallback_due": self.fallback_due,
            "last_fallback_due": self.last_fallback_due,
            "last_policy_switch": self.last_policy_switch,
            "policy_switch_first_writes": list(self.policy_switch_first_writes),
            "episode_id": self.episode_id,
            "episode_started_monotonic": self.episode_started_monotonic,
            "episode_finished_monotonic": time.monotonic(),
            "go_sent": self.go_sent,
            "stop_sent": self.stop_sent,
            "amp_hold_sent": self.amp_hold_sent,
            "joint_hold_sent": self.joint_hold_sent,
            "hold_transition_id": self.hold_transition_id,
            "hold_kind": self.hold_kind,
            "superseded_fallback_due_events": list(
                self.superseded_fallback_due_events
            ),
            "command_history": list(self.command_history),
            "error": self.error,
        }

    def open(self) -> None:
        if self.listener is not None:
            return
        socket_type = getattr(socket, "SOCK_SEQPACKET", None)
        if socket_type is None:
            raise RuntimeError("physical recovery requires AF_UNIX/SOCK_SEQPACKET")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if self.path.exists() or self.path.is_symlink():
            if self.path.is_symlink() or not self.path.is_socket():
                raise RuntimeError(
                    f"refusing to replace non-socket recovery endpoint: {self.path}"
                )
            self.path.unlink()
        listener = socket.socket(socket.AF_UNIX, socket_type)
        try:
            listener.bind(str(self.path))
            os.chmod(self.path, 0o600)
            listener.listen(1)
            listener.setblocking(False)
        except Exception:
            listener.close()
            try:
                self.path.unlink()
            except FileNotFoundError:
                pass
            raise
        self.listener = listener

    def reset_for_start(self) -> None:
        if self.episode_id is not None and self.events_received > 0:
            self.completed_episodes.append(self._episode_snapshot())
            self.completed_episodes[:] = self.completed_episodes[-8:]
        if self.connection is not None:
            self.connection.close()
            self.connection = None
        self.ready = False
        self.first_write = False
        self.amp_hold_first_write = False
        self.joint_hold_first_write = False
        self.stopped = False
        self.paused = False
        self.pause_sent = False
        self.execution_provider = None
        self.resident_policies = []
        self.registered_policy_ids = []
        self.initial_policy_id = None
        self.selected_policy_id = None
        self.policy_selection_transition_id = None
        self.policy_selection_requested_id = None
        self.last_policy_selection = None
        self.last_policy_selection_rejection = None
        self.models_loaded_once = False
        self.models_warmed = False
        self.last_event = None
        self.last_status = None
        self.fallback_due = False
        self.last_fallback_due = None
        self.last_policy_switch = None
        self.policy_switch_first_writes = []
        self.error = None
        self.events_received = 0
        self.expected_peer_pid = None
        self.peer_pid = None
        self.last_packet_monotonic = None
        self.episode_counter += 1
        self.episode_id = self.episode_counter
        self.episode_started_monotonic = time.monotonic()
        self.go_sent = False
        self.stop_sent = False
        self.amp_hold_sent = False
        self.joint_hold_sent = False
        self.hold_transition_counter = 0
        self.hold_transition_id = None
        self.hold_kind = None
        self.superseded_fallback_due_events = []
        self.advance_transition_counter = 0
        self.advance_transition_id = None
        self.policy_switch_accepted = False
        self.command_history = []

    def begin_resident_episode(self) -> int:
        """Start a new writer lease while preserving the loaded worker process."""

        if not self.ready or self.connection is None or not self.paused:
            raise RuntimeError("resident recovery worker is not ready and paused")
        if self.policy_selection_transition_id is not None:
            raise RuntimeError("cannot start recovery during a policy selection")
        if self.episode_id is not None and self.go_sent:
            self.completed_episodes.append(self._episode_snapshot())
            self.completed_episodes[:] = self.completed_episodes[-8:]
            self.episode_counter += 1
            self.episode_id = self.episode_counter
            self.episode_started_monotonic = time.monotonic()
        elif self.episode_id is None:
            self.episode_counter += 1
            self.episode_id = self.episode_counter
            self.episode_started_monotonic = time.monotonic()
        self.first_write = False
        self.amp_hold_first_write = False
        self.joint_hold_first_write = False
        self.stopped = False
        self.fallback_due = False
        self.last_fallback_due = None
        self.last_policy_switch = None
        self.policy_switch_first_writes = []
        self.error = None
        self.go_sent = False
        self.stop_sent = False
        self.pause_sent = False
        self.amp_hold_sent = False
        self.joint_hold_sent = False
        self.hold_transition_counter = 0
        self.hold_transition_id = None
        self.hold_kind = None
        self.superseded_fallback_due_events = []
        self.advance_transition_counter = 0
        self.advance_transition_id = None
        self.policy_switch_accepted = False
        return self.episode_id

    def bind_expected_peer_pid(self, pid: int) -> None:
        if type(pid) is not int or pid <= 1:
            raise ValueError("recovery worker PID must be greater than one")
        if self.connection is not None:
            raise RuntimeError("cannot change recovery worker PID after accept")
        if self.episode_id is None:
            self.episode_counter += 1
            self.episode_id = self.episode_counter
            self.episode_started_monotonic = time.monotonic()
        self.expected_peer_pid = pid

    def _handle_packet(self, packet: bytes) -> None:
        try:
            payload = json.loads(packet.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"invalid recovery worker packet: {exc}") from exc
        if not isinstance(payload, dict) or payload.get("schema") not in self.SCHEMAS:
            raise RuntimeError("recovery worker packet has an unsupported schema")
        self.last_packet_monotonic = time.monotonic()
        event = payload.get("event")
        if not isinstance(event, str):
            raise RuntimeError("recovery worker packet has no event")
        self.events_received += 1
        self.last_event = event
        episode_bound_events = {
            "FIRST_WRITE",
            "AMP_HOLD_FIRST_WRITE",
            "JOINT_HOLD_FIRST_WRITE",
            "STOPPED",
            "PAUSED_NO_WRITER",
            "PAUSED_RESIDENT_WRITER",
            "POLICY_FALLBACK_DUE",
            "POLICY_SWITCH",
            "POLICY_SWITCH_FIRST_WRITE",
        }
        if event in episode_bound_events and payload.get("episode_id") != self.episode_id:
            raise RuntimeError(
                "recovery worker event episode mismatch: "
                f"event={event} expected={self.episode_id} "
                f"actual={payload.get('episode_id')}"
            )
        if event == "READY_NO_WRITER":
            if payload.get("writer_created") is not False:
                raise RuntimeError("recovery worker READY already owns a writer")
            provider = payload.get("execution_provider")
            resident_policies = payload.get("resident_policies")
            registered_policy_ids = payload.get("registered_policy_ids")
            initial_policy_id = payload.get("initial_policy_id")
            selected_policy_id = payload.get("selected_policy_id", initial_policy_id)
            has_resident_attestation = any(
                key in payload
                for key in (
                    "execution_provider",
                    "resident_policies",
                    "resident_policy_count",
                    "models_loaded_once",
                    "models_warmed",
                )
            )
            if self.require_resident_attestation or has_resident_attestation:
                if not isinstance(provider, str) or not provider:
                    raise RuntimeError(
                        "recovery worker READY has no provider attestation"
                    )
                if not isinstance(resident_policies, list) or not resident_policies:
                    raise RuntimeError(
                        "recovery worker READY has no resident policies"
                    )
                if payload.get("resident_policy_count") != len(resident_policies):
                    raise RuntimeError(
                        "recovery worker resident policy count mismatch"
                    )
                if payload.get("models_loaded_once") is not True:
                    raise RuntimeError("recovery worker models were not loaded once")
                if payload.get("models_warmed") is not True:
                    raise RuntimeError("recovery worker models were not warmed")
                if any(
                    not isinstance(item, dict)
                    or item.get("execution_provider") != provider
                    or item.get("warmed") is not True
                    for item in resident_policies
                ):
                    raise RuntimeError(
                        "recovery worker resident policy attestation is invalid"
                    )
                if self.require_resident_attestation:
                    if (
                        not isinstance(registered_policy_ids, list)
                        or not registered_policy_ids
                        or any(
                            not isinstance(policy_id, str) or not policy_id
                            for policy_id in registered_policy_ids
                        )
                    ):
                        raise RuntimeError(
                            "recovery worker READY has no registered policy IDs"
                        )
                    if initial_policy_id not in registered_policy_ids:
                        raise RuntimeError(
                            "recovery worker initial policy is not registered"
                        )
                    if selected_policy_id not in registered_policy_ids:
                        raise RuntimeError(
                            "recovery worker selected policy is not registered"
                        )
                self.execution_provider = provider
                self.resident_policies = [dict(item) for item in resident_policies]
                self.registered_policy_ids = list(registered_policy_ids or [])
                self.initial_policy_id = (
                    str(initial_policy_id) if initial_policy_id is not None else None
                )
                self.selected_policy_id = (
                    str(selected_policy_id)
                    if selected_policy_id is not None
                    else self.initial_policy_id
                )
                self.models_loaded_once = True
                self.models_warmed = True
            self.ready = True
            self.paused = True
        elif event == "FIRST_WRITE":
            if not self.ready:
                raise RuntimeError("recovery worker wrote before READY")
            if not self.go_sent:
                raise RuntimeError("recovery worker wrote before supervisor GO")
            self.first_write = True
            self.paused = False
        elif event == "AMP_HOLD_FIRST_WRITE":
            if not self.first_write:
                raise RuntimeError("AMP hold wrote before the HoST first write")
            if not self.amp_hold_sent:
                raise RuntimeError("AMP hold wrote before supervisor request")
            if payload.get("transition_id") != self.hold_transition_id:
                raise RuntimeError("AMP hold transition_id mismatch")
            self.amp_hold_first_write = True
        elif event == "JOINT_HOLD_FIRST_WRITE":
            if not self.first_write:
                raise RuntimeError("joint hold wrote before the HoST first write")
            if not self.joint_hold_sent:
                raise RuntimeError("joint hold wrote before supervisor request")
            if payload.get("transition_id") != self.hold_transition_id:
                raise RuntimeError("joint hold transition_id mismatch")
            if payload.get("writer_reused") is not True:
                raise RuntimeError("joint hold created an unexpected writer")
            if payload.get("measured_joint_target") is not True:
                raise RuntimeError("joint hold did not use measured joint targets")
            if payload.get("measured_joint_count") != 29:
                raise RuntimeError("joint hold did not capture all 29 joints")
            if payload.get("capture_once") is not True:
                raise RuntimeError("joint hold target was not captured once")
            if payload.get("target_velocity_zero") is not True:
                raise RuntimeError("joint hold retained a velocity target")
            if payload.get("feedforward_torque_zero") is not True:
                raise RuntimeError("joint hold retained feedforward torque")
            capture_age = payload.get("lowstate_capture_age_s")
            if (
                not isinstance(capture_age, (int, float))
                or not math.isfinite(float(capture_age))
                or float(capture_age) < 0.0
                or float(capture_age) > 0.05
            ):
                raise RuntimeError("joint hold captured a stale LowState")
            self.joint_hold_first_write = True
        elif event == "STOPPED":
            if not self.stop_sent:
                raise RuntimeError("recovery worker stopped before supervisor STOP")
            self.stopped = True
            self.paused = False
        elif event == "PAUSED_NO_WRITER":
            if not self.pause_sent:
                raise RuntimeError("recovery worker paused before supervisor PAUSE")
            if payload.get("writer_created") is not False:
                raise RuntimeError("recovery worker PAUSE retained a writer")
            self.paused = True
        elif event == "PAUSED_RESIDENT_WRITER":
            if not self.pause_sent:
                raise RuntimeError("recovery worker paused before supervisor PAUSE")
            if payload.get("writer_created") is not True:
                raise RuntimeError("resident recovery PAUSE lost its writer")
            if payload.get("write_authorized") is not False:
                raise RuntimeError("resident recovery PAUSE retained write authority")
            if payload.get("writer_reused") is not True:
                raise RuntimeError("resident recovery PAUSE cannot reuse its writer")
            self.paused = True
        elif event == "STATUS":
            controller = payload.get("controller")
            if controller == "WRITER_FREE_STANDBY":
                if payload.get("writer_created") is not False:
                    raise RuntimeError("recovery standby STATUS reported a writer")
            elif controller == "PAUSED_RESIDENT_WRITER":
                if payload.get("writer_created") is not True:
                    raise RuntimeError("resident standby STATUS lost its writer")
                if payload.get("write_authorized") is not False:
                    raise RuntimeError(
                        "resident standby STATUS retained write authority"
                    )
            self.last_status = dict(payload)
            selected_policy_id = payload.get("selected_policy_id")
            if selected_policy_id is not None:
                if selected_policy_id not in self.registered_policy_ids:
                    raise RuntimeError("worker STATUS selected an unknown policy")
                self.selected_policy_id = str(selected_policy_id)
        elif event == "POLICY_SELECTED":
            if payload.get("transition_id") != self.policy_selection_transition_id:
                raise RuntimeError("policy selection transition_id mismatch")
            if payload.get("slot") != "recovery":
                raise RuntimeError("policy selection targeted an unknown slot")
            if payload.get("policy_id") != self.policy_selection_requested_id:
                raise RuntimeError("policy selection acknowledged the wrong policy")
            if payload.get("writer_active") is not False:
                raise RuntimeError("policy selection changed an active writer")
            if payload.get("models_reused") is not True:
                raise RuntimeError("policy selection did not reuse resident models")
            self.selected_policy_id = str(payload["policy_id"])
            self.last_policy_selection = dict(payload)
            self.last_policy_selection_rejection = None
            self.policy_selection_transition_id = None
            self.policy_selection_requested_id = None
        elif event == "POLICY_SELECTION_REJECTED":
            if payload.get("transition_id") != self.policy_selection_transition_id:
                raise RuntimeError("policy selection rejection transition_id mismatch")
            self.last_policy_selection_rejection = dict(payload)
            self.policy_selection_transition_id = None
            self.policy_selection_requested_id = None
        elif event == "ERROR":
            self.error = str(payload.get("message", "worker error"))
        elif event == "POLICY_FALLBACK_DUE":
            if not self.first_write:
                raise RuntimeError("policy fallback became due before first write")
            if payload.get("requires_supervisor_authorization") is not True:
                raise RuntimeError("policy fallback bypassed supervisor authority")
            if self.amp_hold_sent or self.joint_hold_sent or self.stop_sent:
                # The worker may have queued this packet immediately before it
                # consumed the hold/STOP command in the opposite socket
                # direction.  Once that command is sent the old HoST epoch can
                # never be advanced, so retain the packet as evidence and
                # explicitly supersede it instead of failing the recovery.
                superseded = dict(payload)
                superseded["superseded_by"] = (
                    "STOP" if self.stop_sent else self.hold_kind
                )
                self.superseded_fallback_due_events.append(superseded)
                return
            if (
                not self.go_sent
            ):
                raise RuntimeError("policy fallback became due outside HoST ownership")
            if self.advance_transition_id is not None:
                raise RuntimeError("new policy fallback arrived before prior first write")
            self.fallback_due = True
            self.last_fallback_due = dict(payload)
        elif event == "POLICY_SWITCH":
            if not self.fallback_due or self.advance_transition_id is None:
                raise RuntimeError("policy switched without an authorized fallback")
            if payload.get("transition_id") != self.advance_transition_id:
                raise RuntimeError("policy switch transition_id mismatch")
            if payload.get("physical_continuation") is not True:
                raise RuntimeError("policy switch was not a physical continuation")
            due = self.last_fallback_due or {}
            if (
                payload.get("from_policy_index") != due.get("policy_index")
                or payload.get("to_policy_index") != due.get("next_policy_index")
            ):
                raise RuntimeError("policy switch did not match the due fallback")
            self.policy_switch_accepted = True
            self.last_policy_switch = dict(payload)
        elif event == "POLICY_SWITCH_FIRST_WRITE":
            if not self.policy_switch_accepted or self.advance_transition_id is None:
                raise RuntimeError("policy switch wrote before accepted authorization")
            if payload.get("transition_id") != self.advance_transition_id:
                raise RuntimeError("policy switch first-write transition_id mismatch")
            if payload.get("writer_reused") is not True:
                raise RuntimeError("policy switch created an unexpected writer")
            if payload.get("physical_continuation") is not True:
                raise RuntimeError("policy switch first write was not physical")
            self.policy_switch_first_writes.append(dict(payload))
            self.fallback_due = False
            self.advance_transition_id = None
            self.policy_switch_accepted = False
        else:
            raise RuntimeError(f"unsupported recovery worker event: {event}")

    def poll(self) -> None:
        if self.listener is None:
            raise RuntimeError("recovery worker control endpoint is not open")
        if self.connection is None:
            if self.expected_peer_pid is None:
                return
            try:
                connection, _address = self.listener.accept()
            except BlockingIOError:
                return
            if not hasattr(socket, "SO_PEERCRED"):
                connection.close()
                raise RuntimeError("recovery worker peer credentials are unavailable")
            credentials = connection.getsockopt(
                socket.SOL_SOCKET,
                socket.SO_PEERCRED,
                struct.calcsize("3i"),
            )
            peer_pid, _peer_uid, _peer_gid = struct.unpack("3i", credentials)
            if peer_pid != self.expected_peer_pid:
                self.peer_pid_mismatches += 1
                connection.close()
                raise RuntimeError(
                    "recovery worker peer PID mismatch: "
                    f"expected={self.expected_peer_pid} actual={peer_pid}"
                )
            connection.setblocking(False)
            self.connection = connection
            self.peer_pid = peer_pid
        assert self.connection is not None
        while True:
            try:
                packet = self.connection.recv(65536)
            except BlockingIOError:
                break
            if not packet:
                self.connection.close()
                self.connection = None
                self.peer_pid = None
                if not self.stopped:
                    self.error = "recovery worker control disconnected"
                    raise RuntimeError(self.error)
                break
            self._handle_packet(packet)

    def send(self, command: str) -> None:
        command = command.upper()
        if command not in {
            "GO",
            "PAUSE",
            "STOP",
            "ENTER_AMP_HOLD",
            "ENTER_JOINT_HOLD",
            "ADVANCE_POLICY",
        }:
            raise ValueError(f"unsupported recovery worker command: {command}")
        if self.connection is None:
            raise RuntimeError("recovery worker is not connected")
        if self.episode_id is None:
            raise RuntimeError("recovery worker command has no active episode")
        payload: dict[str, Any] = {
            "schema": "matrix.sonic_host_worker.control.v1",
            "command": command,
            "episode_id": self.episode_id,
        }
        if command == "GO":
            if not self.ready:
                raise RuntimeError("cannot authorize recovery worker before READY")
            if self.go_sent:
                raise RuntimeError("recovery worker GO was already sent")
            if self.stop_sent:
                raise RuntimeError("cannot authorize a stopped recovery worker")
            if not self.paused:
                raise RuntimeError("cannot authorize an unpaused recovery worker")
        elif command == "PAUSE":
            if not self.go_sent or not self.first_write or self.paused:
                raise RuntimeError("cannot pause recovery worker before ownership")
            if self.pause_sent or self.stop_sent:
                raise RuntimeError("recovery worker PAUSE is not valid in current state")
        elif command in {"ENTER_AMP_HOLD", "ENTER_JOINT_HOLD"}:
            if not self.go_sent or not self.first_write:
                raise RuntimeError("cannot request policy hold before HoST ownership")
            if self.stop_sent or self.amp_hold_sent or self.joint_hold_sent:
                raise RuntimeError("policy hold request is not valid in current state")
            if self.advance_transition_id is not None:
                raise RuntimeError("cannot enter policy hold during a policy switch")
            self.hold_transition_counter += 1
            hold_kind = (
                "amp" if command == "ENTER_AMP_HOLD" else "joint_pose"
            )
            hold_transition_id = (
                f"recovery-{self.episode_id}-hold-"
                f"{self.hold_transition_counter}"
            )
            payload.update(
                {
                    "transition_id": hold_transition_id,
                    "hold_kind": hold_kind,
                }
            )
        elif command == "ADVANCE_POLICY":
            if not self.go_sent or not self.first_write:
                raise RuntimeError("cannot advance policy before HoST ownership")
            if not self.fallback_due or self.last_fallback_due is None:
                raise RuntimeError("cannot advance policy before fallback is due")
            if self.stop_sent or self.amp_hold_sent or self.joint_hold_sent:
                raise RuntimeError("cannot advance policy outside HoST ownership")
            if self.advance_transition_id is not None:
                raise RuntimeError("policy advance is already pending")
            self.advance_transition_counter += 1
            transition_id = (
                f"recovery-{self.episode_id}-switch-"
                f"{self.advance_transition_counter}"
            )
            payload.update(
                {
                    "transition_id": transition_id,
                    "expected_from_policy_index": self.last_fallback_due.get(
                        "policy_index"
                    ),
                    "expected_to_policy_index": self.last_fallback_due.get(
                        "next_policy_index"
                    ),
                }
            )
        packet = json.dumps(
            payload,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8")
        written = self.connection.send(packet)
        if written != len(packet):
            raise RuntimeError("short recovery worker control packet")
        self.command_history.append(
            {
                "command": command,
                "episode_id": self.episode_id,
                "transition_id": payload.get("transition_id"),
                "sent_monotonic": time.monotonic(),
            }
        )
        if command == "GO":
            self.go_sent = True
            self.paused = False
        elif command == "PAUSE":
            self.pause_sent = True
        elif command == "STOP":
            self.stop_sent = True
        elif command == "ENTER_AMP_HOLD":
            self.amp_hold_sent = True
            self.hold_transition_id = str(payload["transition_id"])
            self.hold_kind = "amp"
            self.fallback_due = False
        elif command == "ENTER_JOINT_HOLD":
            self.joint_hold_sent = True
            self.hold_transition_id = str(payload["transition_id"])
            self.hold_kind = "joint_pose"
            self.fallback_due = False
        elif command == "ADVANCE_POLICY":
            self.advance_transition_id = str(payload["transition_id"])
            self.policy_switch_accepted = False

    def select_policy(self, policy_id: str, *, transition_id: str) -> None:
        """Assign the next recovery policy while the resident writer is paused."""

        selected = str(policy_id).strip().lower()
        if selected not in self.registered_policy_ids:
            raise ValueError(f"recovery policy is not registered: {selected!r}")
        if not transition_id or len(transition_id) > 128:
            raise ValueError("policy selection transition_id is invalid")
        if self.connection is None or not self.ready or not self.paused:
            raise RuntimeError("recovery worker is not ready and paused")
        if self.go_sent and not self.pause_sent:
            raise RuntimeError("recovery worker still owns the active episode")
        if self.policy_selection_transition_id is not None:
            raise RuntimeError("a recovery policy selection is already pending")
        payload = {
            "schema": "matrix.sonic_host_worker.control.v1",
            "command": "SELECT_POLICY",
            "slot": "recovery",
            "policy_id": selected,
            "transition_id": transition_id,
        }
        packet = json.dumps(
            payload,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8")
        written = self.connection.send(packet)
        if written != len(packet):
            raise RuntimeError("short recovery policy selection packet")
        self.policy_selection_transition_id = transition_id
        self.policy_selection_requested_id = selected
        self.last_policy_selection_rejection = None
        self.command_history.append(
            {
                "command": "SELECT_POLICY",
                "episode_id": self.episode_id,
                "transition_id": transition_id,
                "policy_id": selected,
                "sent_monotonic": time.monotonic(),
            }
        )

    def ready_recent(self, *, max_age_s: float) -> bool:
        if not math.isfinite(max_age_s) or max_age_s <= 0.0:
            raise ValueError("max_age_s must be finite and positive")
        if (
            not self.ready
            or self.connection is None
            or self.last_packet_monotonic is None
        ):
            return False
        age = time.monotonic() - self.last_packet_monotonic
        return 0.0 <= age <= max_age_s

    def telemetry(self) -> dict[str, object]:
        resident_writer_created = bool(
            isinstance(self.last_status, dict)
            and self.last_status.get("writer_created") is True
        )
        return {
            "connected": self.connection is not None,
            "ready_no_writer": self.ready,
            "first_write": self.first_write,
            "amp_hold_first_write": self.amp_hold_first_write,
            "joint_hold_first_write": self.joint_hold_first_write,
            "stopped": self.stopped,
            "paused_no_writer": self.paused and not resident_writer_created,
            "resident_paused": self.paused,
            "resident_writer_created": resident_writer_created,
            "last_event": self.last_event,
            "events_received": self.events_received,
            "expected_peer_pid": self.expected_peer_pid,
            "peer_pid": self.peer_pid,
            "peer_pid_mismatches": self.peer_pid_mismatches,
            "last_packet_age_s": (
                round(max(0.0, time.monotonic() - self.last_packet_monotonic), 6)
                if self.last_packet_monotonic is not None
                else None
            ),
            "fallback_due": self.fallback_due,
            "last_fallback_due": self.last_fallback_due,
            "last_policy_switch": self.last_policy_switch,
            "policy_switch_first_writes": list(self.policy_switch_first_writes),
            "completed_episodes": list(self.completed_episodes),
            "episode_id": self.episode_id,
            "go_sent": self.go_sent,
            "stop_sent": self.stop_sent,
            "pause_sent": self.pause_sent,
            "amp_hold_sent": self.amp_hold_sent,
            "joint_hold_sent": self.joint_hold_sent,
            "hold_transition_id": self.hold_transition_id,
            "hold_kind": self.hold_kind,
            "superseded_fallback_due_events": list(
                self.superseded_fallback_due_events
            ),
            "advance_transition_id": self.advance_transition_id,
            "command_history": list(self.command_history),
            "error": self.error,
            "status": self.last_status,
            "execution_provider": self.execution_provider,
            "resident_policies": list(self.resident_policies),
            "registered_policy_ids": list(self.registered_policy_ids),
            "initial_policy_id": self.initial_policy_id,
            "selected_policy_id": self.selected_policy_id,
            "policy_selection_pending": self.policy_selection_transition_id is not None,
            "last_policy_selection": self.last_policy_selection,
            "last_policy_selection_rejection": self.last_policy_selection_rejection,
            "models_loaded_once": self.models_loaded_once,
            "models_warmed": self.models_warmed,
            "resident_attestation_required": self.require_resident_attestation,
            "socket": str(self.path),
        }

    def close(self) -> None:
        if self.connection is not None:
            self.connection.close()
            self.connection = None
        if self.listener is not None:
            self.listener.close()
            self.listener = None
        try:
            self.path.unlink()
        except FileNotFoundError:
            pass


class _SonicWriterControl(_RecoveryWorkerControl):
    """Authenticate and drive one native SONIC rt/lowcmd startup gate."""

    SCHEMA = "matrix.sonic_deploy.control.v1"

    def __init__(
        self,
        path: Path,
        *,
        require_authority_epoch: bool = False,
    ) -> None:
        super().__init__(path)
        self.require_authority_epoch = bool(require_authority_epoch)
        self.writer_created = False
        self.writer_revoked = False
        self.writer_failed_closed = False
        self.paused = False
        self.pause_pending = False
        self.resume_pending = False
        self.authority_epoch = 0
        self.epoch_first_write = False
        self.resume_count = 0
        self.shadow_ready = False
        self.reentry_alignment_complete = False
        self.reentry_safe_idle_hold_active = False
        self.reentry_policy_full_control = False

    def reset_for_start(self) -> None:
        super().reset_for_start()
        self.writer_created = False
        self.writer_revoked = False
        self.writer_failed_closed = False
        self.paused = False
        self.pause_pending = False
        self.resume_pending = False
        self.authority_epoch = 0
        self.epoch_first_write = False
        self.resume_count = 0
        self.shadow_ready = False
        self.reentry_alignment_complete = False
        self.reentry_safe_idle_hold_active = False
        self.reentry_policy_full_control = False

    @property
    def current_first_write(self) -> bool:
        """Whether this generation has written and still owns authority."""

        return (
            self.epoch_first_write
            and self.writer_created
            and not self.paused
            and not self.writer_revoked
        )

    def _handle_packet(self, packet: bytes) -> None:
        try:
            payload = json.loads(packet.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"invalid SONIC writer-gate packet: {exc}") from exc
        if not isinstance(payload, dict) or payload.get("schema") != self.SCHEMA:
            raise RuntimeError("SONIC writer-gate packet has an unsupported schema")
        if payload.get("writer_scope") != "rt/lowcmd":
            raise RuntimeError("SONIC writer-gate packet has an invalid writer scope")
        event = payload.get("event")
        if not isinstance(event, str):
            raise RuntimeError("SONIC writer-gate packet has no event")
        self.last_packet_monotonic = time.monotonic()
        self.events_received += 1
        self.last_event = event
        writer_created = payload.get("lowcmd_writer_created")
        write_authorized = payload.get("write_authorized")
        authority_epoch = payload.get("authority_epoch")
        if authority_epoch is None and not self.require_authority_epoch:
            if event == "WRITER_CREATED" and self.authority_epoch == 0:
                authority_epoch = 1
            elif event == "WRITER_RESUMED":
                authority_epoch = self.authority_epoch + 1
            else:
                authority_epoch = self.authority_epoch
        if type(authority_epoch) is not int or authority_epoch < 0:
            raise RuntimeError("SONIC writer event has an invalid authority epoch")
        if event == "READY_NO_LOWCMD_WRITER":
            if writer_created is not False:
                raise RuntimeError("SONIC READY already owns an rt/lowcmd writer")
            if write_authorized not in (None, False):
                raise RuntimeError("SONIC READY reported write authorization")
            if self.first_write:
                raise RuntimeError("SONIC READY arrived after FIRST_WRITE")
            self.ready = True
        elif event == "SHADOW_READY_NO_LOWCMD_WRITER":
            if not self.ready:
                raise RuntimeError(
                    "SONIC shadow READY arrived before process READY"
                )
            if writer_created is not False:
                raise RuntimeError("SONIC shadow READY already owns rt/lowcmd")
            if write_authorized not in (None, False):
                raise RuntimeError("SONIC shadow READY reported write authority")
            if self.first_write or self.go_sent:
                raise RuntimeError("SONIC shadow READY arrived after activation")
            if self.shadow_ready:
                raise RuntimeError("duplicate SONIC shadow readiness attestation")
            self.shadow_ready = True
        elif event == "WRITER_CREATED":
            if not self.ready:
                raise RuntimeError("SONIC created writer before READY")
            if not self.go_sent:
                raise RuntimeError("SONIC created writer before supervisor GO")
            if writer_created is not True:
                raise RuntimeError("SONIC WRITER_CREATED event has no writer")
            if write_authorized not in (None, True):
                raise RuntimeError("SONIC writer was created without authorization")
            if self.writer_revoked:
                raise RuntimeError("SONIC recreated a writer after hard revocation")
            self.writer_created = True
            self.authority_epoch = authority_epoch
        elif event == "FIRST_WRITE":
            if not self.ready:
                raise RuntimeError("SONIC wrote before READY")
            if not self.go_sent:
                raise RuntimeError("SONIC wrote before supervisor GO")
            if writer_created is not True:
                raise RuntimeError("SONIC FIRST_WRITE did not own rt/lowcmd")
            if write_authorized not in (None, True):
                raise RuntimeError("SONIC FIRST_WRITE lacked authorization")
            if self.writer_revoked:
                raise RuntimeError("SONIC wrote after hard revocation")
            if self.paused or self.pause_pending:
                raise RuntimeError("SONIC wrote after resident PAUSE")
            if authority_epoch != self.authority_epoch:
                raise RuntimeError("SONIC FIRST_WRITE authority epoch mismatch")
            self.writer_created = True
            self.first_write = True
            self.epoch_first_write = True
        elif event == "WRITER_PAUSED":
            if not self.pause_pending:
                raise RuntimeError("SONIC paused before supervisor PAUSE")
            if writer_created is not True or write_authorized is not False:
                raise RuntimeError("SONIC PAUSE did not retain a fenced writer")
            if authority_epoch != self.authority_epoch:
                raise RuntimeError("SONIC PAUSE authority epoch mismatch")
            self.paused = True
            self.pause_pending = False
            self.epoch_first_write = False
        elif event == "WRITER_RESUMED":
            if not self.resume_pending or not self.paused:
                raise RuntimeError("SONIC resumed before supervisor RESUME")
            if writer_created is not True or write_authorized is not True:
                raise RuntimeError("SONIC RESUME did not restore writer authority")
            if authority_epoch != self.authority_epoch + 1:
                raise RuntimeError("SONIC RESUME did not advance authority epoch")
            self.authority_epoch = authority_epoch
            self.paused = False
            self.resume_pending = False
            self.epoch_first_write = False
            self.resume_count += 1
        elif event == "REENTRY_ALIGNMENT_COMPLETE":
            if not self.shadow_ready:
                raise RuntimeError(
                    "SONIC alignment arrived without shadow readiness"
                )
            if not self.first_write or not self.writer_created:
                raise RuntimeError("SONIC alignment completed before first write")
            if writer_created is not True:
                raise RuntimeError("SONIC alignment event reported no writer")
            if write_authorized is not True or self.writer_revoked:
                raise RuntimeError("SONIC alignment lacked writer authority")
            if self.reentry_alignment_complete:
                raise RuntimeError("duplicate SONIC alignment attestation")
            self.reentry_alignment_complete = True
        elif event == "REENTRY_SAFE_IDLE_HOLD_ACTIVE":
            if not self.shadow_ready:
                raise RuntimeError(
                    "SONIC safe-idle hold lacked shadow readiness"
                )
            if not self.reentry_alignment_complete:
                raise RuntimeError("SONIC safe-idle hold started before alignment")
            if not self.first_write or not self.writer_created:
                raise RuntimeError("SONIC safe-idle hold started without a writer")
            if writer_created is not True:
                raise RuntimeError("SONIC safe-idle hold event reported no writer")
            if write_authorized is not True or self.writer_revoked:
                raise RuntimeError("SONIC safe-idle hold lacked authority")
            if self.reentry_policy_full_control:
                raise RuntimeError(
                    "SONIC safe-idle hold regressed from policy full control"
                )
            if self.reentry_safe_idle_hold_active:
                raise RuntimeError("duplicate SONIC safe-idle hold attestation")
            self.reentry_safe_idle_hold_active = True
        elif event == "REENTRY_POLICY_FULL_CONTROL":
            if not self.shadow_ready:
                raise RuntimeError(
                    "SONIC policy activation lacked shadow readiness"
                )
            if not self.reentry_alignment_complete:
                raise RuntimeError("SONIC policy activated before alignment")
            if not self.first_write or not self.writer_created:
                raise RuntimeError("SONIC policy activated without a writer")
            if writer_created is not True:
                raise RuntimeError("SONIC policy event reported no writer")
            if write_authorized is not True or self.writer_revoked:
                raise RuntimeError("SONIC policy activation lacked authority")
            if self.reentry_policy_full_control:
                raise RuntimeError("duplicate SONIC policy activation attestation")
            self.reentry_policy_full_control = True
        elif event == "WRITER_REVOKED":
            if not self.stop_sent:
                raise RuntimeError("SONIC revoked writer before supervisor STOP")
            if write_authorized is not False:
                raise RuntimeError("SONIC WRITER_REVOKED retained authorization")
            if not self.ready:
                raise RuntimeError("SONIC revoked before READY")
            self.writer_created = bool(writer_created)
            self.writer_revoked = True
            self.paused = False
        elif event == "WRITER_FAILED_CLOSED":
            if not self.go_sent or not self.first_write:
                raise RuntimeError("SONIC failed closed before first write")
            if write_authorized is not False:
                raise RuntimeError("SONIC failed-closed event retained authority")
            self.writer_created = bool(writer_created)
            self.writer_revoked = True
            self.paused = False
            self.writer_failed_closed = True
            self.error = "native SONIC LowCmd producer failed closed"
        elif event == "STOPPED":
            if not self.stop_sent and not self.writer_failed_closed:
                raise RuntimeError("SONIC stopped before supervisor STOP")
            if writer_created is not False:
                raise RuntimeError("SONIC STOPPED retained its rt/lowcmd writer")
            if write_authorized not in (None, False):
                raise RuntimeError("SONIC STOPPED retained write authorization")
            self.writer_created = False
            self.writer_revoked = True
            self.paused = False
            self.stopped = True
        else:
            raise RuntimeError(f"unsupported SONIC writer-gate event: {event}")

    def send(self, command: str) -> None:
        command = command.upper()
        if command not in {"GO", "PAUSE", "RESUME", "STOP"}:
            raise ValueError(f"unsupported SONIC writer-gate command: {command}")
        if self.connection is None:
            raise RuntimeError("SONIC writer gate is not connected")
        if self.episode_id is None:
            raise RuntimeError("SONIC writer gate has no active episode")
        if command == "GO":
            if not self.ready:
                raise RuntimeError("cannot authorize SONIC before READY")
            if self.go_sent or self.stop_sent:
                raise RuntimeError("SONIC GO is not valid in current state")
        elif command == "PAUSE":
            if (
                not self.current_first_write
                or self.pause_pending
                or self.paused
                or self.resume_pending
                or self.stop_sent
            ):
                raise RuntimeError("SONIC PAUSE is not valid in current state")
        elif command == "RESUME":
            if (
                not self.paused
                or self.pause_pending
                or self.resume_pending
                or self.stop_sent
            ):
                raise RuntimeError("SONIC RESUME is not valid in current state")
        elif self.stop_sent:
            raise RuntimeError("SONIC STOP was already sent")
        packet = command.encode("ascii")
        written = self.connection.send(packet)
        if written != len(packet):
            raise RuntimeError("short SONIC writer-gate command")
        self.command_history.append(
            {
                "command": command,
                "episode_id": self.episode_id,
                "sent_monotonic": time.monotonic(),
            }
        )
        if command == "GO":
            self.go_sent = True
        elif command == "PAUSE":
            self.pause_pending = True
        elif command == "RESUME":
            self.resume_pending = True
        else:
            self.stop_sent = True

    def telemetry(self) -> dict[str, object]:
        result = super().telemetry()
        result.update(
            {
                "schema": self.SCHEMA,
                "ready_no_lowcmd_writer": self.ready,
                "shadow_ready_no_lowcmd_writer": self.shadow_ready,
                "lowcmd_writer_created": self.writer_created,
                "write_authorized": (
                    self.writer_created
                    and not self.paused
                    and not self.writer_revoked
                ),
                "resident_paused": self.paused,
                "pause_pending": self.pause_pending,
                "resume_pending": self.resume_pending,
                "authority_epoch": self.authority_epoch,
                "authority_epoch_first_write": self.epoch_first_write,
                "authority_epoch_required": self.require_authority_epoch,
                "resident_resume_count": self.resume_count,
                "writer_revoked": self.writer_revoked,
                "writer_failed_closed": self.writer_failed_closed,
                "first_write": self.first_write,
                "reentry_alignment_complete": (
                    self.reentry_alignment_complete
                ),
                "reentry_safe_idle_hold_active": (
                    self.reentry_safe_idle_hold_active
                ),
                "reentry_policy_full_control": (
                    self.reentry_policy_full_control
                ),
                "writer_scope": "rt/lowcmd",
            }
        )
        return result


class _PhysicalRecoveryCoordinator:
    """Translate snapshots/FSM actions without stopping input or rendering."""

    # Some focused tests construct this class with __new__ to isolate one
    # legacy method.  Keep that path on the non-resident state machine unless
    # the normal constructor explicitly enables residency.
    resident_policies = False
    execution_provider = "cpu"

    POSE_TRIGGER_HEIGHT_M = 0.45
    POSE_TRIGGER_UP_Z = 0.5
    POSE_TRIGGER_HOLD_S = 0.35
    STANDBY_HEARTBEAT_TIMEOUT_S = 2.0
    FALLBACK_NEAR_UPRIGHT_HEIGHT_M = 0.62
    FALLBACK_NEAR_UPRIGHT_UP_Z = 0.85
    FALLBACK_NEAR_UPRIGHT_GRACE_S = 2.0
    FALLBACK_QUIET_HOLD_S = 0.25
    FALLBACK_MAX_LINEAR_SPEED_M_S = 0.5
    FALLBACK_MAX_ANGULAR_SPEED_RAD_S = 1.5
    FALLBACK_MAX_JOINT_VELOCITY_RMS_RAD_S = 1.5
    # Conservative first qualification after a replacement deploy.  At the
    # 50 Hz game command rate this is 1 rad/s and prevents a frame transform
    # from bypassing the core's own yaw slew limiter.
    WIRE_MAX_TURN_RATE_RAD_S = 1.0
    WIRE_MAX_HEADING_STEP_RAD = 0.02
    # GameControlCore deliberately keeps its planner-facing target close to
    # measured yaw.  That short lead is below SONIC's effective turning
    # threshold after a deploy-frame re-anchor, so the recovery adapter may use
    # more of the *true* camera-relative remaining turn carried as command
    # metadata.  It never extrapolates past that true target, and every output
    # frame remains subject to the 0.02 rad wire slew limit above.
    WIRE_TURN_LEAD_WINDOW_RAD = 0.4
    _PREWARM_POLICY_STATES = frozenset(
        {
            RecoveryState.POLICY_RECOVERING,
            RecoveryState.POLICY_GETUP_STABLE,
            RecoveryState.POLICY_AMP_HOLD_REQUESTED,
            RecoveryState.POLICY_AMP_HOLDING,
        }
    )

    def __init__(
        self,
        args: argparse.Namespace,
        *,
        initial_root_yaw_rad: float,
    ) -> None:
        timeout = float(args.physical_recovery_timeout_seconds)
        self.resident_policies = bool(
            getattr(args, "physical_recovery_resident_policies", False)
        )
        self.execution_provider = str(
            getattr(args, "physical_recovery_execution_provider", "cpu")
        )
        recovery_config = RecoveryConfig(
            stable_hold_s=float(args.physical_recovery_stable_hold_seconds),
            policy_exit_hold_s=float(
                getattr(args, "physical_recovery_policy_exit_hold_seconds", 0.0)
            ),
            stable_root_z_m=self.FALLBACK_NEAR_UPRIGHT_HEIGHT_M,
            max_lowcmd_age_s=float(args.low_cmd_fresh_timeout_seconds),
            policy_recovery_timeout_s=timeout,
            use_amp_hold=(
                not self.resident_policies
                and str(getattr(args, "physical_recovery_handoff", "amp"))
                == "amp"
            ),
            sonic_prewarm_timeout_s=float(
                args.physical_recovery_sonic_prewarm_timeout_seconds
            ),
            sonic_full_control_timeout_s=float(
                args.physical_recovery_sonic_full_control_timeout_seconds
            ),
            episode_timeout_s=max(120.0, timeout + 60.0),
        )
        self.fsm = (
            ResidentPolicyRecoveryFSM(
                recovery_config,
                recovery_policy_id=str(
                    getattr(args, "physical_recovery_initial_controller", "host")
                ),
            )
            if self.resident_policies
            else SingleWriterRecoveryFSM(recovery_config)
        )
        self.worker = _RecoveryWorkerControl(
            args.physical_recovery_control_socket,
            require_resident_attestation=self.resident_policies,
        )
        self.sonic_writer = _SonicWriterControl(
            args.physical_recovery_sonic_control_socket,
            require_authority_epoch=self.resident_policies,
        )
        self.worker_python = str(args.physical_recovery_python)
        self.worker_script = args.physical_recovery_worker.resolve()
        self.initial_controller = str(
            getattr(args, "physical_recovery_initial_controller", "host")
        ).strip().lower()
        self.handoff_mode = str(
            getattr(args, "physical_recovery_handoff", "amp")
        )
        assert args.physical_recovery_model is not None
        self.model = args.physical_recovery_model.resolve()
        self.fallback_models = tuple(
            path.resolve() for path in args.physical_recovery_fallback_model
        )
        self.fallback_after_s = float(
            args.physical_recovery_fallback_after_seconds
        )
        assert args.physical_recovery_amp_config is not None
        assert args.physical_recovery_amp_model is not None
        self.amp_config = args.physical_recovery_amp_config.resolve()
        self.amp_model = args.physical_recovery_amp_model.resolve()
        self.amp_config_sha256 = str(args.physical_recovery_amp_config_sha256)
        self.amp_model_sha256 = str(args.physical_recovery_amp_model_sha256)
        kungfu_model_arg = getattr(args, "physical_recovery_kungfu_model", None)
        kungfu_motion_arg = getattr(args, "physical_recovery_kungfu_motion", None)
        self.kungfu_model = (
            kungfu_model_arg.resolve()
            if isinstance(kungfu_model_arg, Path)
            else None
        )
        self.kungfu_motion = (
            kungfu_motion_arg.resolve()
            if isinstance(kungfu_motion_arg, Path)
            else None
        )
        kungfu_model_sha256_arg = getattr(
            args, "physical_recovery_kungfu_model_sha256", None
        )
        kungfu_model_data_sha256_arg = getattr(
            args, "physical_recovery_kungfu_model_data_sha256", None
        )
        kungfu_motion_sha256_arg = getattr(
            args, "physical_recovery_kungfu_motion_sha256", None
        )
        self.kungfu_model_sha256 = (
            str(kungfu_model_sha256_arg)
            if kungfu_model_sha256_arg
            else None
        )
        self.kungfu_model_data_sha256 = (
            str(kungfu_model_data_sha256_arg)
            if kungfu_model_data_sha256_arg
            else None
        )
        self.kungfu_motion_sha256 = (
            str(kungfu_motion_sha256_arg)
            if kungfu_motion_sha256_arg
            else None
        )
        self.kungfu_reference_frame = int(
            getattr(args, "physical_recovery_kungfu_reference_frame", 0)
        )
        self.kungfu_gain_scale = float(
            getattr(args, "physical_recovery_kungfu_gain_scale", 1.0)
        )
        self.interface = args.dds_interface
        self.zmq_port: int | None = None
        self.pose_candidate_since_s: float | None = None
        self.current_fall_detected = False
        self.episodes = 0
        self.recoveries = 0
        self.last_output: RecoveryOutput | ResidentRecoveryOutput | None = None
        self.last_transition_s: float | None = None
        self.initial_root_yaw_rad = float(initial_root_yaw_rad)
        self.restarted_root_yaw_rad: float | None = None
        self.command_frame_rotation_rad = 0.0
        self.command_frame_epoch = 0
        self.last_wire_facing_heading_rad: float | None = None
        self.reframe_limited_frames = 0
        self.last_reframe_limited = False
        self.last_reframe_heading_error_rad = 0.0
        self.replacement_sonic_started_s: float | None = None
        self.replacement_sonic_ready_s: float | None = None
        self.replacement_sonic_first_fresh_s: float | None = None
        self.initial_sonic_gate_pending = False
        self.previous_sonic_writer_revoked = False
        self.previous_sonic_stopped = False
        self.policy_fallback_last_near_upright_s: float | None = None
        self.policy_fallback_quiet_since_s: float | None = None
        self.policy_advance_requested = False
        self.policy_advances = 0
        self.current_recovery_worker_episode_id: int | None = None
        self.latest_completed_recovery_worker_episode_id: int | None = None
        self._policy_selection_pending: dict[str, object] | None = None
        self._policy_selection_results: dict[
            str, tuple[bool, str, str, dict[str, object]]
        ] = {}
        manifest_path = Path(
            getattr(
                args,
                "locomotion_policy_manifest",
                _SCRIPT_DIR.parent
                / "config/runtime/policy-slots/bfm-sonic-teacher50k.json",
            )
        )
        self.locomotion_policy_candidates: tuple[PolicyCandidateState, ...] = (
            evaluate_policy_candidate(
                manifest_path,
                _SCRIPT_DIR.parent / "config/runtime/matrix-sonic.lock.json",
                project_root=_SCRIPT_DIR.parent,
            ),
        )

    def _configured_recovery_policy_ids(self) -> tuple[str, ...]:
        configured = ["kungfu", "host", "amp"]
        if (
            getattr(self, "kungfu_model", None) is None
            or getattr(self, "kungfu_motion", None) is None
        ):
            configured.remove("kungfu")
        initial_controller = str(getattr(self, "initial_controller", "host"))
        if initial_controller not in configured:
            configured.append(initial_controller)
        worker = getattr(self, "worker", None)
        if worker is not None and worker.registered_policy_ids:
            registered = set(worker.registered_policy_ids)
            configured = [policy for policy in configured if policy in registered]
        return tuple(dict.fromkeys(configured))

    def strategy_loadout_mapping(self) -> dict[str, object]:
        """Return the game-facing two-slot view over resident policy sessions."""

        state = self.fsm.state
        game_state = state in {
            RecoveryState.GAME_SONIC,
            ResidentRecoveryState.GAME_SONIC,
        }
        recovery_ids = self._configured_recovery_policy_ids()
        pending_value = getattr(self, "_policy_selection_pending", None)
        pending = (
            dict(pending_value)
            if pending_value is not None
            else None
        )
        worker = self.worker
        resident_ready = bool(
            self.resident_policies
            and worker.ready
            and worker.models_loaded_once
            and worker.models_warmed
        )
        selected_recovery = (
            worker.selected_policy_id
            if getattr(worker, "selected_policy_id", None) in recovery_ids
            else str(getattr(self, "initial_controller", "host"))
        )
        locomotion_candidates = [
            {
                "policy_id": "sonic",
                "name": "SONIC",
                "resident": True,
                "available": True,
                "provenance_verified": True,
                "unavailable_reason": None,
            },
            *[
                candidate.to_mapping()
                for candidate in getattr(self, "locomotion_policy_candidates", ())
            ],
        ]
        return {
            "version": 1,
            "available": bool(self.resident_policies),
            "status": (
                "unavailable"
                if not self.resident_policies
                else (
                    "switching"
                    if pending is not None
                    else ("ready" if resident_ready else "loading")
                )
            ),
            "active_slot": "locomotion" if game_state else "recovery",
            "pending": pending,
            "slots": [
                {
                    "slot": "locomotion",
                    "selected_policy_id": "sonic",
                    "locked": not any(
                        candidate.get("available") is True
                        and candidate.get("policy_id") != "sonic"
                        for candidate in locomotion_candidates
                    ),
                    "candidates": locomotion_candidates,
                },
                {
                    "slot": "recovery",
                    "selected_policy_id": selected_recovery,
                    "locked": not self.resident_policies,
                    "candidates": [
                        {
                            "policy_id": policy_id,
                            "resident": bool(self.resident_policies),
                            "available": bool(self.resident_policies),
                        }
                        for policy_id in recovery_ids
                    ],
                },
            ],
            "resident_models": [
                {"policy_id": "sonic", "name": "sonic", "resident": True},
                *[
                    {
                        "policy_id": candidate.policy_id,
                        "name": candidate.display_name,
                        "resident": candidate.resident,
                        "available": candidate.available,
                        "unavailable_reason": candidate.unavailable_reason,
                    }
                    for candidate in getattr(
                        self, "locomotion_policy_candidates", ()
                    )
                ],
                *[
                    {
                        "policy_id": policy_id,
                        "name": policy_id,
                        "resident": bool(self.resident_policies),
                    }
                    for policy_id in recovery_ids
                ],
            ],
        }

    def request_policy_slot_assignment(
        self,
        command: PolicySlotAssignment,
        *,
        transition_id: str,
    ) -> dict[str, object] | None:
        """Begin one writer-free slot assignment, or return an idempotent result."""

        if command.slot == "locomotion":
            if command.policy_id == "sonic":
                return self.strategy_loadout_mapping()
            candidate = next(
                (
                    item
                    for item in getattr(self, "locomotion_policy_candidates", ())
                    if item.policy_id == command.policy_id
                ),
                None,
            )
            if candidate is None:
                raise CommandExecutionError(
                    "E_POLICY_NOT_REGISTERED",
                    f"Locomotion policy is not registered: {command.policy_id}",
                )
            if not candidate.available or not candidate.resident:
                reason = candidate.unavailable_reason or "runtime_adapter_not_registered"
                raise CommandExecutionError(
                    "E_POLICY_UNAVAILABLE",
                    f"Locomotion policy is unavailable: {candidate.policy_id}: {reason}",
                )
            # No BFM writer implementation reaches this branch today.  Keep a
            # fail-closed guard so a manifest edit alone cannot grant LowCmd.
            if command.policy_id == BFM_TEACHER50K_POLICY_ID:
                raise CommandExecutionError(
                    "E_POLICY_UNAVAILABLE",
                    "BFM Teacher50k has no registered Matrix writer adapter",
                )
            raise CommandExecutionError(
                "E_POLICY_NOT_REGISTERED",
                f"Locomotion policy is not switchable: {command.policy_id}",
            )
        if not self.resident_policies:
            raise CommandExecutionError(
                "E_POLICY_UNAVAILABLE",
                "Resident recovery policy slots are disabled",
            )
        if self._policy_selection_pending is not None:
            raise CommandExecutionError(
                "E_POLICY_SWITCH_BUSY",
                "A recovery policy selection is already pending",
            )
        if self.fsm.state is not ResidentRecoveryState.GAME_SONIC:
            raise CommandExecutionError(
                "E_POLICY_SLOT_ACTIVE",
                "Recovery policy can change only while SONIC owns control",
            )
        candidates = self._configured_recovery_policy_ids()
        if command.policy_id not in candidates:
            raise CommandExecutionError(
                "E_POLICY_NOT_REGISTERED",
                f"Recovery policy is not resident: {command.policy_id}",
            )
        if (
            command.policy_id == self.initial_controller
            and self.worker.selected_policy_id == command.policy_id
        ):
            return self.strategy_loadout_mapping()
        if not self.worker.ready or not self.worker.paused:
            raise CommandExecutionError(
                "E_POLICY_WORKER_NOT_READY",
                "Resident recovery policies are not ready and writer-free",
            )
        try:
            self.worker.select_policy(
                command.policy_id,
                transition_id=transition_id,
            )
        except (RuntimeError, ValueError) as exc:
            raise CommandExecutionError(
                "E_POLICY_SWITCH_REJECTED",
                str(exc),
            ) from exc
        self._policy_selection_pending = {
            "slot": command.slot,
            "policy_id": command.policy_id,
            "transition_id": transition_id,
            "requested_monotonic_s": time.monotonic(),
        }
        return None

    def _reconcile_policy_slot_assignment(self) -> None:
        pending = getattr(self, "_policy_selection_pending", None)
        if pending is None:
            return
        transition_id = str(pending["transition_id"])
        rejection = self.worker.last_policy_selection_rejection
        if (
            rejection is not None
            and rejection.get("transition_id") == transition_id
        ):
            self._policy_selection_pending = None
            self._policy_selection_results[transition_id] = (
                False,
                "E_POLICY_SWITCH_REJECTED",
                str(rejection.get("message", "Recovery policy selection failed")),
                self.strategy_loadout_mapping(),
            )
            return
        selection = self.worker.last_policy_selection
        if selection is None or selection.get("transition_id") != transition_id:
            return
        selected = str(selection["policy_id"])
        self.fsm.select_recovery_policy(selected)
        self.initial_controller = selected
        self._policy_selection_pending = None
        self._policy_selection_results[transition_id] = (
            True,
            "OK_POLICY_SLOT_ASSIGNED",
            f"Assigned {selected} to recovery",
            self.strategy_loadout_mapping(),
        )

    def poll_policy_slot_assignment(
        self,
        transition_id: str,
    ) -> tuple[bool, str, str, dict[str, object]] | None:
        self._reconcile_policy_slot_assignment()
        return getattr(self, "_policy_selection_results", {}).pop(
            transition_id, None
        )

    def _capture_restart_anchor(self, qpos: Any) -> None:
        self.restarted_root_yaw_rad = _root_yaw_rad(qpos)
        anchor_delta = wrap_angle_rad(
            self.restarted_root_yaw_rad - self.initial_root_yaw_rad
        )
        self.command_frame_rotation_rad = -anchor_delta
        self.command_frame_epoch += 1
        # The replacement deploy normalizes its physical startup yaw to wire
        # heading zero.  Seed the cross-frame limiter at that exact boundary so
        # the first active command, and every command after it, can move by at
        # most one wire slew step even if measured yaw changes abruptly.
        self.last_wire_facing_heading_rad = 0.0
        self.last_reframe_limited = False
        self.last_reframe_heading_error_rad = 0.0

    def open(self, *, zmq_port: int) -> None:
        self.zmq_port = int(zmq_port)
        self.worker.open()
        self.sonic_writer.open()

    def prepare_initial_sonic_gate(self) -> Path:
        self.sonic_writer.reset_for_start()
        self.initial_sonic_gate_pending = True
        return self.sonic_writer.path

    def bind_initial_sonic_gate(self, *, processes: NativeProcessGroup) -> None:
        deploy_pid = processes.deploy_pid()
        if deploy_pid is None:
            raise RuntimeError("initial gated SONIC has no process PID")
        self.sonic_writer.bind_expected_peer_pid(deploy_pid)

    def _fall_level(self, snapshot: Any, now_s: float) -> bool:
        root_z = float(snapshot.qpos[2])
        root_up_z = _root_up_z(snapshot.qpos)
        pose_candidate = (
            root_z < self.POSE_TRIGGER_HEIGHT_M
            and root_up_z < self.POSE_TRIGGER_UP_Z
        )
        if pose_candidate:
            if self.pose_candidate_since_s is None:
                self.pose_candidate_since_s = now_s
        else:
            self.pose_candidate_since_s = None
        pose_trigger = (
            self.pose_candidate_since_s is not None
            and now_s - self.pose_candidate_since_s >= self.POSE_TRIGGER_HOLD_S
        )
        # SONIC's fall flag is sticky, so gate it with its live height test.
        native_trigger = root_z < 0.2 and bool(snapshot.fall_detected)
        self.current_fall_detected = native_trigger or pose_trigger
        return self.current_fall_detected

    def _maybe_authorize_policy_advance(
        self,
        *,
        now_s: float,
        root_z_m: float,
        root_up_z: float,
        root_linear_speed_m_s: float,
        root_angular_speed_rad_s: float,
        joint_velocity_rms_rad_s: float,
        grounded_contact: bool,
        recovery_state: RecoveryState,
        policy_alive: bool,
        worker_controller: str | None,
    ) -> None:
        """Use full simulator state to authorize a physical HoST fallback.

        The worker deliberately cannot see root height or contacts.  A timeout
        therefore only raises ``fallback_due``; this supervisor waits through
        near-upright hysteresis and high dynamics, then sends ADVANCE_POLICY
        after a continuous low-energy grounded window.
        """

        near_upright = (
            root_z_m >= self.FALLBACK_NEAR_UPRIGHT_HEIGHT_M
            and root_up_z >= self.FALLBACK_NEAR_UPRIGHT_UP_Z
        )
        if near_upright:
            self.policy_fallback_last_near_upright_s = now_s

        if not self.worker.fallback_due:
            self.policy_fallback_quiet_since_s = None
            if self.policy_advance_requested:
                # POLICY_SWITCH_FIRST_WRITE was acknowledged; do not leak the
                # old policy's grace window into the new controller.
                self.policy_fallback_last_near_upright_s = None
            self.policy_advance_requested = False
            return

        eligible_owner = (
            recovery_state is RecoveryState.POLICY_RECOVERING
            and policy_alive
            and self.worker.connection is not None
            and self.worker.go_sent
            and not self.worker.stop_sent
            and not self.worker.amp_hold_sent
            and not self.worker.joint_hold_sent
            and worker_controller == "HOST_GETUP"
        )
        if not eligible_owner:
            self.policy_fallback_quiet_since_s = None
            return
        if self.policy_advance_requested:
            return
        if near_upright:
            self.policy_fallback_quiet_since_s = None
            return

        if (
            self.policy_fallback_last_near_upright_s is not None
            and now_s - self.policy_fallback_last_near_upright_s
            < self.FALLBACK_NEAR_UPRIGHT_GRACE_S
        ):
            self.policy_fallback_quiet_since_s = None
            return

        # Once the near-upright grace has expired, every non-upright grounded
        # pose is eligible.  This deliberately avoids a height/orientation
        # dead zone around the standing threshold.
        low_posture = not near_upright
        low_dynamics = (
            root_linear_speed_m_s <= self.FALLBACK_MAX_LINEAR_SPEED_M_S
            and root_angular_speed_rad_s
            <= self.FALLBACK_MAX_ANGULAR_SPEED_RAD_S
            and joint_velocity_rms_rad_s
            <= self.FALLBACK_MAX_JOINT_VELOCITY_RMS_RAD_S
        )
        if not (low_posture and low_dynamics and grounded_contact):
            self.policy_fallback_quiet_since_s = None
            return
        if self.policy_fallback_quiet_since_s is None:
            self.policy_fallback_quiet_since_s = now_s
            return
        if (
            now_s - self.policy_fallback_quiet_since_s
            < self.FALLBACK_QUIET_HOLD_S
        ):
            return

        self.worker.send("ADVANCE_POLICY")
        self.policy_advance_requested = True
        self.policy_advances += 1
        self.policy_fallback_quiet_since_s = None
        self.policy_fallback_last_near_upright_s = None

    def _resident_worker_attested(self, *, policy_alive: bool) -> bool:
        expected_provider = (
            "CUDAExecutionProvider"
            if self.execution_provider == "cuda"
            else "CPUExecutionProvider"
        )
        names = {
            str(item.get("name"))
            for item in self.worker.resident_policies
            if isinstance(item, dict)
        }
        required_loaded = (
            "kungfu:1307_recovery" in names
            and "amp:walk_run_getup" in names
            and any(name.startswith("host:") for name in names)
        )
        return (
            policy_alive
            and self.worker.ready_recent(
                max_age_s=self.STANDBY_HEARTBEAT_TIMEOUT_S
            )
            and self.worker.execution_provider == expected_provider
            and self.worker.models_loaded_once
            and self.worker.models_warmed
            and required_loaded
        )

    def observe(
        self,
        snapshot: Any,
        *,
        now_s: float,
        neutral_confirmed: bool,
        foot_contact: bool,
        grounded_contact: bool,
        processes: NativeProcessGroup,
    ) -> RecoveryOutput | ResidentRecoveryOutput:
        self.worker.poll()
        self.sonic_writer.poll()
        self._reconcile_policy_slot_assignment()
        if self.fsm.state not in {
            RecoveryState.GAME_SONIC,
            ResidentRecoveryState.GAME_SONIC,
        }:
            self.previous_sonic_writer_revoked |= bool(
                self.sonic_writer.writer_revoked
            )
            self.previous_sonic_stopped |= bool(self.sonic_writer.stopped)
        if self.worker.error is not None:
            raise RuntimeError(f"physical recovery worker: {self.worker.error}")
        if self.sonic_writer.error is not None:
            raise RuntimeError(
                f"replacement SONIC writer gate: {self.sonic_writer.error}"
            )
        if self.sonic_writer.ready and self.replacement_sonic_ready_s is None:
            self.replacement_sonic_ready_s = now_s
        qvel = tuple(float(value) for value in snapshot.qvel)
        root_linear_speed = math.sqrt(sum(value * value for value in qvel[:3]))
        root_angular_speed = math.sqrt(sum(value * value for value in qvel[3:6]))
        joint_values = qvel[6:]
        joint_rms = math.sqrt(
            sum(value * value for value in joint_values) / len(joint_values)
        )
        root_z = float(snapshot.qpos[2])
        root_up_z = _root_up_z(snapshot.qpos)
        policy_alive = processes.recovery_policy_alive()
        worker_controller = (
            str(self.worker.last_status.get("controller"))
            if isinstance(self.worker.last_status, dict)
            and self.worker.last_status.get("controller") is not None
            else None
        )
        if self.resident_policies:
            policy_ready = self._resident_worker_attested(
                policy_alive=policy_alive
            )
            resident_observation = ResidentRecoveryInput(
                now_s=now_s,
                fall_detected=self._fall_level(snapshot, now_s),
                root_z_m=root_z,
                root_up_z=root_up_z,
                root_linear_speed_m_s=root_linear_speed,
                root_angular_speed_rad_s=root_angular_speed,
                joint_velocity_rms_rad_s=joint_rms,
                lowcmd_fresh=bool(snapshot.low_cmd_fresh),
                lowcmd_age_s=(
                    float(snapshot.low_cmd_age_s)
                    if snapshot.low_cmd_age_s is not None
                    else None
                ),
                sonic_alive=processes.deploy_alive(),
                sonic_generation=processes.deploy_generation,
                sonic_resident_ready=(
                    processes.deploy_alive()
                    and self.sonic_writer.ready
                    and not self.sonic_writer.writer_failed_closed
                ),
                sonic_writer_active=self.sonic_writer.current_first_write,
                sonic_writer_paused=self.sonic_writer.paused,
                sonic_resume_first_write=(
                    self.sonic_writer.resume_count > 0
                    and self.sonic_writer.epoch_first_write
                ),
                policy_alive=policy_alive,
                policy_resident_ready=policy_ready,
                policy_writer_active=(
                    policy_alive
                    and self.worker.go_sent
                    and self.worker.first_write
                    and not self.worker.paused
                ),
                policy_writer_paused=self.worker.paused,
                policy_first_write=self.worker.first_write,
                reset_count=int(snapshot.reset_count),
                foot_contact=bool(foot_contact),
                grounded_contact=bool(grounded_contact),
                neutral_confirmed=bool(neutral_confirmed),
            )
            previous_resident = self.fsm.state
            output_resident = self.fsm.step(resident_observation)
            self.last_output = output_resident
            if output_resident.state is not previous_resident:
                self.last_transition_s = now_s
                if (
                    output_resident.state
                    is ResidentRecoveryState.SONIC_PAUSE_REQUESTED
                ):
                    self.episodes += 1
                    self.current_recovery_worker_episode_id = None
                    self.policy_fallback_last_near_upright_s = None
                    self.policy_fallback_quiet_since_s = None
                    self.policy_advance_requested = False
                elif output_resident.resume_game:
                    self.recoveries += 1
                    self.latest_completed_recovery_worker_episode_id = (
                        self.current_recovery_worker_episode_id
                    )
            return output_resident
        self._maybe_authorize_policy_advance(
            now_s=now_s,
            root_z_m=root_z,
            root_up_z=root_up_z,
            root_linear_speed_m_s=root_linear_speed,
            root_angular_speed_rad_s=root_angular_speed,
            joint_velocity_rms_rad_s=joint_rms,
            grounded_contact=bool(grounded_contact),
            recovery_state=self.fsm.state,
            policy_alive=policy_alive,
            worker_controller=worker_controller,
        )
        deploy_alive = processes.deploy_alive()
        gated_deploy = self.sonic_writer.expected_peer_pid is not None
        observation = RecoveryInput(
            now_s=now_s,
            fall_detected=self._fall_level(snapshot, now_s),
            root_z_m=root_z,
            root_up_z=root_up_z,
            root_linear_speed_m_s=root_linear_speed,
            root_angular_speed_rad_s=root_angular_speed,
            joint_velocity_rms_rad_s=joint_rms,
            lowcmd_fresh=bool(snapshot.low_cmd_fresh),
            lowcmd_age_s=(
                float(snapshot.low_cmd_age_s)
                if snapshot.low_cmd_age_s is not None
                else None
            ),
            deploy_alive=deploy_alive,
            deploy_generation=processes.deploy_generation,
            deploy_process_ready=(
                gated_deploy and deploy_alive and self.sonic_writer.ready
            ),
            deploy_writer_ready=(
                gated_deploy
                and deploy_alive
                and self.sonic_writer.shadow_ready
            ),
            deploy_writer_created=(
                deploy_alive
                and (
                    self.sonic_writer.writer_created
                    if gated_deploy
                    else True
                )
            ),
            deploy_writer_revoked=(
                gated_deploy and self.sonic_writer.writer_revoked
            ),
            deploy_first_write=(
                gated_deploy
                and deploy_alive
                and self.sonic_writer.current_first_write
            ),
            deploy_policy_full_control=(
                gated_deploy
                and deploy_alive
                and self.sonic_writer.reentry_policy_full_control
                and self.sonic_writer.current_first_write
            ),
            deploy_safe_idle_hold=(
                gated_deploy
                and deploy_alive
                and self.sonic_writer.reentry_safe_idle_hold_active
                and self.sonic_writer.current_first_write
            ),
            policy_alive=policy_alive,
            policy_ready=(
                policy_alive
                and self.worker.ready_recent(
                    max_age_s=self.STANDBY_HEARTBEAT_TIMEOUT_S
                )
            ),
            policy_first_write=self.worker.first_write and policy_alive,
            policy_hold_first_write=(
                (
                    self.worker.amp_hold_first_write
                    if self.handoff_mode == "amp"
                    else self.worker.joint_hold_first_write
                )
                and policy_alive
            ),
            reset_count=int(snapshot.reset_count),
            foot_contact=bool(foot_contact),
            grounded_contact=bool(grounded_contact),
            neutral_confirmed=bool(neutral_confirmed),
        )
        previous = self.fsm.state
        if (
            previous
            in {RecoveryState.SONIC_RESTARTING, RecoveryState.SONIC_STABILIZING}
            and bool(snapshot.low_cmd_fresh)
            and self.replacement_sonic_first_fresh_s is None
        ):
            self.replacement_sonic_first_fresh_s = now_s
        output = self.fsm.step(observation)
        self.last_output = output
        if output.state is not previous:
            self.last_transition_s = now_s
            if output.state is RecoveryState.SONIC_STOP_REQUESTED:
                self.episodes += 1
                self.current_recovery_worker_episode_id = None
                self.previous_sonic_writer_revoked = False
                self.previous_sonic_stopped = False
                self.policy_fallback_last_near_upright_s = None
                self.policy_fallback_quiet_since_s = None
                self.policy_advance_requested = False
            elif output.resume_game:
                self.recoveries += 1
                self.latest_completed_recovery_worker_episode_id = (
                    self.current_recovery_worker_episode_id
                )
        if (
            previous is RecoveryState.SONIC_STABILIZING
            and output.state is RecoveryState.WAIT_NEUTRAL
        ):
            # Each native deploy defines yaw zero from the physical pose it sees
            # at startup. Keep camera/game math in the original world frame and
            # rotate only its planner wire command into the new deploy frame.
            self._capture_restart_anchor(snapshot.qpos)
        return output

    def reframe_game_command(
        self,
        command: RobotMotionCommand,
        *,
        measured_heading_rad: float | None = None,
        dt_s: float | None = None,
    ) -> RobotMotionCommand:
        """Rotate a world command and bound it against physical deploy yaw.

        The transform is downstream of :class:`GameControlCore`, so it is a
        safety boundary of its own.  When physical yaw is supplied, the wire
        target slews by at most ``WIRE_MAX_HEADING_STEP_RAD`` and
        ``WIRE_MAX_TURN_RATE_RAD_S * dt_s`` per published frame.  Do not clamp
        a turn back to the core's small single-tick planner lead: the command
        also carries its true camera-relative final heading, and this boundary
        uses up to a finite physical lead window without ever targeting beyond
        that final heading.  Once the core requests movement, the boundary
        slews back to the exact core target before allowing translation.
        A clipped moving command is converted to native turn-only form;
        translation never gets published in a direction this boundary did not
        approve.  Neutral/safety commands keep both the core's one-shot
        stopped-heading latch and this active-wire latch unchanged.
        """

        if not isinstance(command, RobotMotionCommand):
            raise TypeError("command must be a RobotMotionCommand")
        angle = self.command_frame_rotation_rad
        cosine = math.cos(angle)
        sine = math.sin(angle)

        def rotate(vector: tuple[float, float, float]) -> tuple[float, float, float]:
            x, y, z = vector
            return (
                cosine * x - sine * y,
                sine * x + cosine * y,
                z,
            )

        rotated_movement = rotate(command.movement)
        rotated_facing = rotate(command.facing)
        rotated_desired_facing = (
            rotate(command.desired_facing)
            if command.desired_facing is not None
            else None
        )
        finite_values = [
            *rotated_movement,
            *rotated_facing,
            command.speed_mps,
        ]
        if rotated_desired_facing is not None:
            finite_values.extend(rotated_desired_facing)
        if not all(
            math.isfinite(value) for value in finite_values
        ):
            raise ValueError("reframed game command must be finite")
        locomotion_mode = command.locomotion_mode
        mode = command.mode
        speed_mps = command.speed_mps
        reason = command.reason
        limited = False
        heading_error = 0.0
        movement_norm = math.hypot(
            rotated_movement[0], rotated_movement[1]
        )
        active_turn_intent = (
            mode in {"move", "turn"}
            or abs(speed_mps) > 1e-12
            or movement_norm > 1e-12
        )
        step_seconds = 1.0 / 50.0 if dt_s is None else float(dt_s)
        if not math.isfinite(step_seconds) or step_seconds < 0.0:
            raise ValueError("reframe dt_s must be finite and nonnegative")
        maximum_step = min(
            self.WIRE_MAX_HEADING_STEP_RAD,
            self.WIRE_MAX_TURN_RATE_RAD_S * step_seconds,
        )
        if measured_heading_rad is not None and active_turn_intent:
            measured_wire_heading = wrap_angle_rad(
                measured_heading_rad + angle
            )
            facing_norm = math.hypot(rotated_facing[0], rotated_facing[1])
            if facing_norm <= 1e-12:
                raise ValueError(
                    "active reframed game command has zero horizontal facing"
                )
            else:
                requested_wire_heading = math.atan2(
                    rotated_facing[1], rotated_facing[0]
                )
                heading_error = wrap_angle_rad(
                    requested_wire_heading - measured_wire_heading
                )
                wire_target_heading = requested_wire_heading
                if mode == "turn" and rotated_desired_facing is not None:
                    desired_norm = math.hypot(
                        rotated_desired_facing[0],
                        rotated_desired_facing[1],
                    )
                    if desired_norm <= 1e-12:
                        raise ValueError(
                            "active reframed game command has zero desired facing"
                        )
                    desired_wire_heading = math.atan2(
                        rotated_desired_facing[1],
                        rotated_desired_facing[0],
                    )
                    desired_error = wrap_angle_rad(
                        desired_wire_heading - measured_wire_heading
                    )
                    bounded_desired_error = max(
                        -self.WIRE_TURN_LEAD_WINDOW_RAD,
                        min(self.WIRE_TURN_LEAD_WINDOW_RAD, desired_error),
                    )
                    wire_target_heading = wrap_angle_rad(
                        measured_wire_heading
                        + bounded_desired_error
                    )
                    heading_error = desired_error
                previous_wire_heading = self.last_wire_facing_heading_rad
                if previous_wire_heading is None:
                    previous_wire_heading = measured_wire_heading
                wire_slew = wrap_angle_rad(
                    wire_target_heading - previous_wire_heading
                )
                bounded_wire_slew = max(
                    -maximum_step,
                    min(maximum_step, wire_slew),
                )
                wire_limited = not math.isclose(
                    bounded_wire_slew,
                    wire_slew,
                    rel_tol=0.0,
                    abs_tol=1e-12,
                )
                bounded_heading = wrap_angle_rad(
                    previous_wire_heading + bounded_wire_slew
                )
                rotated_facing = (
                    math.cos(bounded_heading),
                    math.sin(bounded_heading),
                    0.0,
                )
                self.last_wire_facing_heading_rad = bounded_heading
                limited = wire_limited
                if mode == "turn" or limited:
                    rotated_movement = (0.0, 0.0, 0.0)
                    speed_mps = 0.0
                    locomotion_mode = SONIC_SLOW_WALK_MODE
                    mode = "turn"
                    if limited:
                        reason = "recovery_heading_slew_limited"
                if limited:
                    self.reframe_limited_frames += 1
            self.last_reframe_limited = limited
            self.last_reframe_heading_error_rad = heading_error
        else:
            if not active_turn_intent:
                # Key release, focus loss, and deadman already carry the one-
                # shot physical heading latched by GameControlCore.  Unwind any
                # larger recovery turn lead toward that fixed safe target
                # without a discontinuity.  Translation is zero throughout;
                # importantly, this does not keep chasing noisy measured yaw.
                target_norm = math.hypot(
                    rotated_facing[0], rotated_facing[1]
                )
                if target_norm <= 1e-12:
                    raise ValueError(
                        "stopped reframed game command has zero horizontal facing"
                    )
                stopped_target_heading = math.atan2(
                    rotated_facing[1], rotated_facing[0]
                )
                previous_wire_heading = self.last_wire_facing_heading_rad
                if previous_wire_heading is None:
                    previous_wire_heading = stopped_target_heading
                wire_slew = wrap_angle_rad(
                    stopped_target_heading - previous_wire_heading
                )
                bounded_wire_slew = max(
                    -maximum_step,
                    min(maximum_step, wire_slew),
                )
                limited = not math.isclose(
                    bounded_wire_slew,
                    wire_slew,
                    rel_tol=0.0,
                    abs_tol=1e-12,
                )
                bounded_heading = wrap_angle_rad(
                    previous_wire_heading + bounded_wire_slew
                )
                self.last_wire_facing_heading_rad = bounded_heading
                rotated_facing = (
                    math.cos(bounded_heading),
                    math.sin(bounded_heading),
                    0.0,
                )
                rotated_movement = (0.0, 0.0, 0.0)
                speed_mps = 0.0
                locomotion_mode = SONIC_IDLE_MODE
                if limited:
                    self.reframe_limited_frames += 1
            self.last_reframe_limited = limited
            self.last_reframe_heading_error_rad = 0.0

        return RobotMotionCommand(
            sequence=command.sequence,
            movement=rotated_movement,
            facing=rotated_facing,
            speed_mps=speed_mps,
            locomotion_mode=locomotion_mode,
            mode=mode,
            safe_stop=command.safe_stop,
            reason=reason,
            desired_facing=rotated_desired_facing,
        )

    @staticmethod
    def sonic_bootstrap_command(command: RobotMotionCommand) -> RobotMotionCommand:
        """Return a neutral start frame expressed in a new deploy's yaw frame."""

        return RobotMotionCommand(
            sequence=command.sequence,
            movement=(0.0, 0.0, 0.0),
            facing=(1.0, 0.0, 0.0),
            speed_mps=0.0,
            locomotion_mode=SONIC_IDLE_MODE,
            mode="deadman",
            safe_stop=True,
            reason="physical_recovery_sonic_bootstrap",
            desired_facing=(1.0, 0.0, 0.0),
        )

    def resident_sonic_hold_command(
        self,
        command: RobotMotionCommand,
        *,
        measured_heading_rad: float,
    ) -> RobotMotionCommand:
        """Keep the resident SONIC planner neutral at the live body heading."""

        wire_heading = wrap_angle_rad(
            float(measured_heading_rad) + self.command_frame_rotation_rad
        )
        self.last_wire_facing_heading_rad = wire_heading
        facing = (
            math.cos(wire_heading),
            math.sin(wire_heading),
            0.0,
        )
        return RobotMotionCommand(
            sequence=command.sequence,
            movement=(0.0, 0.0, 0.0),
            facing=facing,
            speed_mps=0.0,
            locomotion_mode=SONIC_IDLE_MODE,
            mode="deadman",
            safe_stop=True,
            reason="physical_recovery_resident_sonic_hold",
            desired_facing=facing,
        )

    def recovery_wire_command(
        self,
        command: RobotMotionCommand,
        output: RecoveryOutput | ResidentRecoveryOutput | None,
        *,
        measured_heading_rad: float,
        dt_s: float,
    ) -> RobotMotionCommand:
        """Apply recovery lifecycle gating at the final planner boundary."""

        if (
            output is not None
            and isinstance(output.state, ResidentRecoveryState)
        ):
            if output.state is ResidentRecoveryState.GAME_SONIC:
                # Resident recovery returns authority to the same SONIC
                # process and therefore never creates a new deploy yaw frame.
                # Applying the replacement-deploy 1 rad/s wire limiter here
                # double-limits the core's already feedback-bounded 2.5 rad/s
                # turn, forcing ordinary camera-relative running into long
                # turn-only (zero-translation) intervals.
                if not math.isclose(
                    self.command_frame_rotation_rad,
                    0.0,
                    rel_tol=0.0,
                    abs_tol=1e-12,
                ):
                    raise RuntimeError(
                        "resident SONIC command frame unexpectedly rotated"
                    )
                facing_norm = math.hypot(command.facing[0], command.facing[1])
                if facing_norm <= 1e-12:
                    raise ValueError(
                        "resident SONIC command has zero horizontal facing"
                    )
                self.last_wire_facing_heading_rad = math.atan2(
                    command.facing[1], command.facing[0]
                )
                self.last_reframe_limited = False
                self.last_reframe_heading_error_rad = 0.0
                return command
            # The resident deploy keeps consuming planner frames while KungFu
            # owns LowCmd. Track the live body yaw so RESUME cannot inherit a
            # stale camera-facing turn request during the fragile handoff.
            return self.resident_sonic_hold_command(
                command,
                measured_heading_rad=measured_heading_rad,
            )
        if (
            output is not None
            and (
                output.state
                in {
                    RecoveryState.SONIC_RESTARTING,
                    RecoveryState.SONIC_STABILIZING,
                }
                or (
                    output.state is RecoveryState.WAIT_NEUTRAL
                    and not self.sonic_writer.reentry_policy_full_control
                )
            )
        ):
            # SONIC owns its writer-ACKed IDLE policy takeover.  Keep sending the
            # exact deploy-frame bootstrap while that takeover is in progress;
            # a non-IDLE planner frame would interrupt the protected transition.
            return self.sonic_bootstrap_command(command)
        return self.reframe_game_command(
            command,
            measured_heading_rad=measured_heading_rad,
            dt_s=dt_s,
        )

    def execute(
        self,
        output: RecoveryOutput | ResidentRecoveryOutput,
        *,
        processes: NativeProcessGroup,
        planner: NativePlannerClient,
    ) -> None:
        game_state = output.state in {
            RecoveryState.GAME_SONIC,
            ResidentRecoveryState.GAME_SONIC,
        }
        resident_worker_ready = (
            not self.resident_policies
            or self._resident_worker_attested(
                policy_alive=processes.recovery_policy_alive()
            )
        )
        if (
            self.initial_sonic_gate_pending
            and game_state
            and self.sonic_writer.ready
            and resident_worker_ready
        ):
            if self.sonic_writer.writer_created or self.sonic_writer.first_write:
                raise RuntimeError("initial SONIC wrote before writer-gate GO")
            self.sonic_writer.send("GO")
            self.initial_sonic_gate_pending = False
        if output.start_policy_process:
            self.start_writer_free_policy(processes=processes)
        if self.resident_policies:
            if output.request_sonic_pause:
                if not (
                    self.sonic_writer.paused
                    or self.sonic_writer.pause_pending
                    or self.sonic_writer.stop_sent
                ):
                    self.sonic_writer.send("PAUSE")
            if output.authorize_policy_writer:
                episode_id = self.worker.begin_resident_episode()
                self.current_recovery_worker_episode_id = episode_id
                self.worker.send("GO")
            if output.request_policy_pause:
                if not self.worker.paused and not self.worker.pause_sent:
                    self.worker.send("PAUSE")
            if output.resume_sonic_writer:
                if self.sonic_writer.paused and not self.sonic_writer.resume_pending:
                    self.sonic_writer.send("RESUME")
            return
        if output.request_sonic_stop:
            processes.begin_deploy_stop()
            # Recovery outputs may keep requesting a stop after the original
            # request has timed out.  Preserve strict duplicate detection in
            # the writer protocol itself, but make this action dispatcher
            # idempotent so a fail-closed retry cannot mask the real timeout.
            if not self.sonic_writer.stop_sent:
                if (
                    self.sonic_writer.expected_peer_pid is not None
                    and self.sonic_writer.connection is not None
                    and not self.sonic_writer.stopped
                ):
                    self.sonic_writer.send("STOP")
                else:
                    planner.request_deploy_stop()
        if output.authorize_policy_writer:
            if self.worker.episode_id is None:
                raise RuntimeError("physical recovery worker has no episode id")
            self.current_recovery_worker_episode_id = self.worker.episode_id
            self.worker.send("GO")
        if output.request_policy_hold:
            self.worker.send("ENTER_AMP_HOLD")
        if output.request_policy_stop:
            # Mark this PID as an expected protocol-driven exit before STOP is
            # sent.  The worker can revoke its publisher, emit STOPPED, and
            # exit(0) within one supervisor frame; without this boundary the
            # generic child monitor can misclassify that successful exit as a
            # crash before the FSM consumes STOPPED.  This method sends no
            # signal, so the reliable control protocol remains the first and
            # only normal writer-revocation path.
            processes.begin_recovery_policy_stop()
            if self.worker.connection is not None:
                # STOP is the writer-revocation fence.  Sending SIGTERM to the
                # process group first lets the worker observe EOF/signal and
                # emit STOPPED before this control object has recorded that a
                # supervisor STOP was authorized.  Let the reliable seqpacket
                # command close the publisher and exit the worker normally;
                # the FSM timeout/outer cleanup remains the hard-kill backup.
                self.worker.send("STOP")
        if output.start_sonic:
            if self.zmq_port is None:
                raise RuntimeError("physical recovery coordinator is not open")
            self.sonic_writer.reset_for_start()
            processes.start_deploy(
                interface=self.interface,
                zmq_port=self.zmq_port,
                writer_control_socket=self.sonic_writer.path,
                # Recovery must bypass SONIC's three-second default-angle
                # InitControl ramp: live E2E shows that ramp pulls an already
                # stable robot back to the floor before planner control begins.
                # Writer re-entry captures the measured 29-DoF handoff pose,
                # holds it for 0.5 s, then blends into SONIC over 5 s.
                physical_reentry=True,
            )
            deploy_pid = processes.deploy_pid()
            if deploy_pid is None:
                raise RuntimeError("replacement SONIC has no process PID")
            self.sonic_writer.bind_expected_peer_pid(deploy_pid)
            self.replacement_sonic_started_s = time.perf_counter()
            self.replacement_sonic_ready_s = None
            self.replacement_sonic_first_fresh_s = None
        if output.authorize_sonic_writer:
            self.sonic_writer.send("GO")

    def start_writer_free_policy(self, *, processes: NativeProcessGroup) -> int:
        """Start a fully loaded HoST/AMP standby without a LowCmd writer."""

        self.worker.reset_for_start()
        worker_pid = processes.start_recovery_policy(
            self.worker_python,
            self.worker_script,
            interface=self.interface,
            control_socket=self.worker.path,
            model=self.model,
            fallback_models=self.fallback_models,
            fallback_after_s=self.fallback_after_s,
            amp_config=self.amp_config,
            amp_model=self.amp_model,
            amp_config_sha256=self.amp_config_sha256,
            amp_model_sha256=self.amp_model_sha256,
            kungfu_model=self.kungfu_model,
            kungfu_motion=self.kungfu_motion,
            kungfu_model_sha256=self.kungfu_model_sha256,
            kungfu_model_data_sha256=self.kungfu_model_data_sha256,
            kungfu_motion_sha256=self.kungfu_motion_sha256,
            kungfu_reference_frame=self.kungfu_reference_frame,
            kungfu_gain_scale=self.kungfu_gain_scale,
            initial_controller=self.initial_controller,
            execution_provider=self.execution_provider,
        )
        self.worker.bind_expected_peer_pid(worker_pid)
        return worker_pid

    def verify_writer_free_prewarm_start(
        self,
        output: RecoveryOutput,
        *,
        processes: NativeProcessGroup,
        previous_generation: int,
    ) -> None:
        """Fail closed unless a new SONIC is alive and still writer-free.

        Replacement SONIC now prewarms while HoST/AMP still owns LowCmd, so a
        successful spawn must remain in a policy-owned state.  Entering
        ``SONIC_STABILIZING`` here was the old, sequential hand-off semantic.
        """

        if processes.deploy_generation != previous_generation + 1:
            raise RuntimeError(
                "writer-free SONIC prewarm did not advance exactly one generation"
            )
        if not processes.deploy_alive():
            raise RuntimeError("writer-free SONIC prewarm process is not alive")
        if output.fail_closed or output.state not in self._PREWARM_POLICY_STATES:
            raise RuntimeError(
                "writer-free SONIC prewarm left the policy-owned recovery phase"
            )
        if self.sonic_writer.writer_created or self.sonic_writer.first_write:
            raise RuntimeError("writer-free SONIC prewarm acquired LowCmd before GO")

    def telemetry(
        self, *, now_s: float, processes: NativeProcessGroup
    ) -> dict[str, object]:
        output = self.last_output
        return {
            "mode": "physical",
            "policy_lifecycle": (
                "resident_authority_switch"
                if self.resident_policies
                else "replacement_process"
            ),
            "resident_policies_enabled": self.resident_policies,
            "execution_provider": self.execution_provider,
            "state": self.fsm.state.value,
            "authority_policy_id": (
                output.authority_policy_id if output is not None else "sonic"
            ),
            "recovery_policy_id": (
                output.recovery_policy_id
                if output is not None
                else self.initial_controller
            ),
            "failure_reason": self.fsm.failure_reason,
            "fail_closed": self.fsm.failed,
            "current_fall_detected": self.current_fall_detected,
            "episodes": self.episodes,
            "recoveries": self.recoveries,
            "deploy_alive": processes.deploy_alive(),
            "deploy_generation": processes.deploy_generation,
            "sonic_pid": self.sonic_writer.expected_peer_pid,
            "recovery_policy_pid": self.worker.expected_peer_pid,
            "resident_process_identity_stable": (
                self.resident_policies
                and processes.deploy_alive()
                and processes.recovery_policy_alive()
                and not self.sonic_writer.stopped
                and not self.worker.stopped
            ),
            "initial_root_yaw_rad": round(self.initial_root_yaw_rad, 6),
            "restarted_root_yaw_rad": (
                round(self.restarted_root_yaw_rad, 6)
                if self.restarted_root_yaw_rad is not None
                else None
            ),
            "command_frame_rotation_rad": round(
                self.command_frame_rotation_rad, 6
            ),
            "command_frame_epoch": self.command_frame_epoch,
            "last_wire_facing_heading_rad": (
                round(self.last_wire_facing_heading_rad, 6)
                if self.last_wire_facing_heading_rad is not None
                else None
            ),
            "reframe_limited_frames": self.reframe_limited_frames,
            "last_reframe_limited": self.last_reframe_limited,
            "last_reframe_heading_error_rad": round(
                self.last_reframe_heading_error_rad, 6
            ),
            "policy_alive": processes.recovery_policy_alive(),
            "inhibit_game_input": (
                output.inhibit_game_input if output is not None else False
            ),
            "last_transition_age_s": (
                round(max(0.0, now_s - self.last_transition_s), 6)
                if self.last_transition_s is not None
                else None
            ),
            "worker": self.worker.telemetry(),
            "sonic_writer_gate": self.sonic_writer.telemetry(),
            "replacement_sonic_writer_gate": self.sonic_writer.telemetry(),
            "initial_sonic_gate_pending": self.initial_sonic_gate_pending,
            "previous_sonic_writer_revoked": (
                self.previous_sonic_writer_revoked
            ),
            "previous_sonic_stopped": self.previous_sonic_stopped,
            "physical_only": True,
            "simulator_state_mutation": False,
            "single_writer_scope": (
                "authority_epoch_fence"
                if self.resident_policies
                else "managed_processes_and_host_lock"
            ),
            "external_dds_writer_identity_observable": False,
            "takeover_settle_s": self.fsm.config.takeover_settle_s,
            "stable_hold_s": self.fsm.config.stable_hold_s,
            "policy_exit_hold_s": self.fsm.config.policy_exit_hold_s,
            "policy_timeout_s": self.fsm.config.policy_recovery_timeout_s,
            "sonic_prewarm_timeout_s": self.fsm.config.sonic_prewarm_timeout_s,
            "sonic_stabilize_timeout_s": (
                self.fsm.config.sonic_stabilize_timeout_s
            ),
            "episode_timeout_s": self.fsm.config.episode_timeout_s,
            "fallback_after_s": self.fallback_after_s,
            "fallback_authority": "matrix_full_physical_state",
            "policy_advance_requested": self.policy_advance_requested,
            "policy_advances": self.policy_advances,
            "current_recovery_worker_episode_id": (
                self.current_recovery_worker_episode_id
            ),
            "latest_completed_recovery_worker_episode_id": (
                self.latest_completed_recovery_worker_episode_id
            ),
            "policy_fallback_quiet_since_s": self.policy_fallback_quiet_since_s,
            "policy_fallback_last_near_upright_s": (
                self.policy_fallback_last_near_upright_s
            ),
            "initial_controller": self.initial_controller,
            "strategy_loadout": self.strategy_loadout_mapping(),
            "kungfu_reference_frame": (
                self.kungfu_reference_frame
                if self.initial_controller == "kungfu"
                else None
            ),
            "kungfu_gain_scale": (
                self.kungfu_gain_scale
                if self.initial_controller == "kungfu"
                else None
            ),
            "handoff_mode": self.handoff_mode,
            "replacement_sonic_started_s": self.replacement_sonic_started_s,
            "replacement_sonic_ready_s": self.replacement_sonic_ready_s,
            "replacement_sonic_ready_latency_s": (
                round(
                    self.replacement_sonic_ready_s
                    - self.replacement_sonic_started_s,
                    6,
                )
                if self.replacement_sonic_started_s is not None
                and self.replacement_sonic_ready_s is not None
                else None
            ),
            "replacement_sonic_first_fresh_s": (
                self.replacement_sonic_first_fresh_s
            ),
            "replacement_sonic_first_fresh_latency_s": (
                round(
                    self.replacement_sonic_first_fresh_s
                    - self.replacement_sonic_started_s,
                    6,
                )
                if self.replacement_sonic_started_s is not None
                and self.replacement_sonic_first_fresh_s is not None
                else None
            ),
        }

    def close(self) -> None:
        self.worker.close()
        self.sonic_writer.close()


def _reanchor_game_heading_after_recovery_transition(
    output: RecoveryOutput,
    core: GameControlCore,
    *,
    measured_heading_rad: float,
) -> bool:
    """Reseed the game heading exactly on the replacement-deploy handoff."""

    if not (
        output.previous_state is RecoveryState.SONIC_STABILIZING
        and output.state is RecoveryState.WAIT_NEUTRAL
    ):
        return False
    assert output.inhibit_game_input
    core.reanchor_heading(measured_heading_rad)
    return True


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


def main(*, completion_event: threading.Event | None = None) -> int:
    args = _parse_args()
    # Several focused tests and downstream embedders construct the historical
    # argparse namespace directly. New optional gameplay fields remain
    # absent-safe for those callers.
    for name, default in (
        ("game_world_id", None),
        ("game_world_revision", None),
        ("game_world_state_file", None),
        ("game_world_checkpoint_seconds", 0.75),
        ("game_auto_respawn", False),
    ):
        if not hasattr(args, name):
            setattr(args, name, default)
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
        if args.game_camera_yaw_source == "ue-final-pov":
            if args.game_ue_camera_state_file is None:
                raise SystemExit(
                    "--game-ue-camera-state-file is required for ue-final-pov"
                )
            if not args.game_ue_camera_state_file.is_absolute():
                raise SystemExit("--game-ue-camera-state-file must be absolute")
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
    world_values = (
        args.game_world_id,
        args.game_world_revision,
        args.game_world_state_file,
    )
    if any(value is not None for value in world_values) and not all(
        value is not None for value in world_values
    ):
        raise SystemExit("game world-state arguments are all-or-none")
    if all(value is not None for value in world_values):
        if args.control_source != "game":
            raise SystemExit("game world-state persistence requires game control")
        assert args.game_world_state_file is not None
        if not args.game_world_state_file.is_absolute():
            raise SystemExit("--game-world-state-file must be absolute")
        try:
            WorldStateStore(
                args.game_world_state_file,
                world_id=args.game_world_id,
                world_revision=args.game_world_revision,
            )
        except WorldStateError as exc:
            raise SystemExit(f"invalid game world-state configuration: {exc}") from exc
        if (
            not math.isfinite(args.game_world_checkpoint_seconds)
            or not 0.1 <= args.game_world_checkpoint_seconds <= 60.0
        ):
            raise SystemExit(
                "--game-world-checkpoint-seconds must be in [0.1, 60]"
            )
        if args.qualified_runtime or args.max_seconds > 0.0:
            raise SystemExit(
                "bounded qualification rejects persistent game world state"
            )
    if args.game_auto_respawn:
        if not all(value is not None for value in world_values):
            raise SystemExit("--game-auto-respawn requires game world-state persistence")
        if args.fail_on_fall:
            raise SystemExit("--game-auto-respawn conflicts with --fail-on-fall")
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
    _validate_game_fall_recovery(args)
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
    game_world = None
    game_commands = None
    game_command_child_socket = None
    game_fall_recovery = None
    physical_recovery = None
    game_command = None
    processes = None
    previous_signal_handlers: dict[int, Any] = {}
    running = True
    termination_reason: str | None = None
    termination_signal: int | None = None
    child_failure: tuple[str, int] | None = None
    numerical_error: str | None = None
    world_checkpoint_failed = False
    proposed_exit_code = 2
    final_status: dict[str, Any] | None = None
    termination_boundary_previous_mask: set[signal.Signals] | None = None
    termination_boundary_safe = False

    def request_stop(signum, _frame) -> None:
        nonlocal running, termination_reason, termination_signal
        running = False
        termination_signal = int(signum)
        if termination_reason is None:
            termination_reason = "signal"

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
        if args.game_world_state_file is not None:
            try:
                game_world = _GameWorldStateRuntime(
                    path=args.game_world_state_file,
                    world_id=args.game_world_id,
                    world_revision=args.game_world_revision,
                    checkpoint_seconds=args.game_world_checkpoint_seconds,
                )
                if _snapshot_world_upright(snapshot):
                    game_world.checkpoint(
                        snapshot,
                        now_s=time.perf_counter(),
                        force=True,
                        required=bool(args.game_auto_respawn),
                    )
            except WorldStateError as exc:
                raise SystemExit(f"cannot initialize game world state: {exc}") from exc
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
        applied_game_camera_yaw_offset_deg = float(
            getattr(args, "game_camera_yaw_offset_deg", 0.0)
        )
        try:
            if args.control_source == "game":
                applied_game_camera_yaw_offset_deg = (
                    _effective_game_camera_yaw_offset_deg(
                        source=args.game_camera_yaw_source,
                        configured_offset_deg=args.game_camera_yaw_offset_deg,
                        initial_root_yaw_rad=initial_root_yaw_rad,
                    )
                )
        except ValueError as exc:
            raise SystemExit(f"invalid game camera yaw frame: {exc}") from exc
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
                if getattr(args, "game_fall_recovery", "off") == "sonic":
                    game_fall_recovery = _GameFallRecoveryGate(
                        timeout_s=args.game_fall_recovery_timeout
                    )
                elif getattr(args, "game_fall_recovery", "off") == "physical":
                    physical_recovery = _PhysicalRecoveryCoordinator(
                        args,
                        initial_root_yaw_rad=initial_root_yaw_rad,
                    )
                    physical_recovery.open(zmq_port=planner_port)
                if not args.no_game_input_provider and (
                    game_world is not None or physical_recovery is not None
                ):
                    command_parent, game_command_child_socket = socket.socketpair(
                        socket.AF_UNIX,
                        socket.SOCK_SEQPACKET,
                    )
                    game_commands = GameCommandRuntime(
                        command_parent,
                        game_world,
                        policy_slots=physical_recovery,
                    )
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
                        camera_yaw_offset_deg=(
                            applied_game_camera_yaw_offset_deg
                        ),
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
                        ue_camera_state_file=args.game_ue_camera_state_file,
                        command_fd=(
                            game_command_child_socket.fileno()
                            if game_command_child_socket is not None
                            else None
                        ),
                        strategy_loadout_json=(
                            json.dumps(
                                physical_recovery.strategy_loadout_mapping(),
                                separators=(",", ":"),
                                sort_keys=True,
                            )
                            if physical_recovery is not None
                            else None
                        ),
                    )
                    if game_command_child_socket is not None:
                        game_command_child_socket.close()
                        game_command_child_socket = None
                    game_input.bind_expected_peer_pid(provider_pid)
        elif running and args.control_source == "pico":
            processes.start_pico(
                args.pico_python or sys.executable, port=planner_port
            )
        if running:
            initial_writer_control_socket = (
                physical_recovery.prepare_initial_sonic_gate()
                if physical_recovery is not None
                else None
            )
            processes.start_deploy(
                interface=args.dds_interface,
                zmq_port=planner_port,
                writer_control_socket=initial_writer_control_socket,
            )
            if physical_recovery is not None:
                physical_recovery.bind_initial_sonic_gate(processes=processes)

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
                if termination_reason is None:
                    termination_reason = "max_seconds"
                poll_failed_child()
                break
            if completion_event is not None and completion_event.is_set():
                if termination_reason is None:
                    termination_reason = "scenario_complete"
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
                    recovery_transition = None
                    if game_fall_recovery is not None:
                        try:
                            recovery_transition = game_fall_recovery.observe(
                                snapshot,
                                now_s=frame_wall,
                            )
                        except ValueError as exc:
                            unstable = True
                            running = False
                            termination_reason = "numerical_instability"
                            numerical_error = f"fall_recovery:{exc}"
                            print(
                                "matrix-sonic-runtime ERROR invalid fall "
                                f"recovery state: {exc}",
                                flush=True,
                            )
                            break
                        if recovery_transition == "entered":
                            print(
                                "matrix-sonic-runtime fall recovery entered "
                                f"episode={game_fall_recovery.episodes} "
                                "source="
                                f"{game_fall_recovery.last_entry_source}",
                                flush=True,
                            )
                        elif recovery_transition == "recovered":
                            game_input.core.invalidate_input(
                                "fall_recovered_awaiting_neutral"
                            )
                            print(
                                "matrix-sonic-runtime fall recovery completed "
                                f"episode={game_fall_recovery.episodes} "
                                f"duration_s={game_fall_recovery.last_duration_s:.3f}",
                                flush=True,
                            )
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
                    ready_game_command = game_readiness.apply(
                        candidate_game_command,
                        game_input.core,
                    )
                    physical_output = None
                    if physical_recovery is not None:
                        foot_contact = _physical_foot_contact(simulator)
                        grounded_contact = _physical_ground_contact(simulator)
                        neutral_confirmed = (
                            candidate_game_command.mode == "idle"
                            and not candidate_game_command.safe_stop
                        )
                        try:
                            physical_output = physical_recovery.observe(
                                snapshot,
                                now_s=frame_wall,
                                neutral_confirmed=neutral_confirmed,
                                foot_contact=foot_contact,
                                grounded_contact=grounded_contact,
                                processes=processes,
                            )
                        except (OSError, RuntimeError, ValueError) as exc:
                            unstable = True
                            running = False
                            termination_reason = "physical_recovery_failed"
                            numerical_error = f"physical_recovery:{exc}"
                            print(
                                "matrix-sonic-runtime ERROR physical recovery: "
                                f"{exc}",
                                flush=True,
                            )
                            break
                        if _reanchor_game_heading_after_recovery_transition(
                            physical_output,
                            game_input.core,
                            measured_heading_rad=measured_heading,
                        ):
                            # observe() has just crossed
                            # SONIC_STABILIZING -> WAIT_NEUTRAL and captured the
                            # replacement deploy's physical yaw zero.  The core
                            # command was polled before that lifecycle event, so
                            # reseed it once.  This transition inhibits game
                            # input, and emergency_stop() below materializes the
                            # reanchored neutral command exactly once.
                            print(
                                "matrix-sonic-runtime recovery heading "
                                "reanchored "
                                f"epoch={physical_recovery.command_frame_epoch} "
                                f"world_heading_rad={measured_heading:.6f}",
                                flush=True,
                            )
                    if (
                        physical_output is not None
                        and physical_output.inhibit_game_input
                    ) or (
                        game_fall_recovery is not None
                        and game_fall_recovery.recovering
                    ):
                        game_command = game_input.emergency_stop(
                            now_s=frame_wall,
                            reason=(
                                "physical_fall_recovery"
                                if physical_output is not None
                                else "fall_recovery"
                            ),
                        )
                    else:
                        game_command = ready_game_command
                    if physical_recovery is not None:
                        game_command = physical_recovery.recovery_wire_command(
                            game_command,
                            physical_output,
                            measured_heading_rad=measured_heading,
                            dt_s=1.0 / args.control_hz,
                        )
                    # Telemetry must describe the command actually published,
                    # not the pre-readiness candidate returned by the core.
                    game_input.last_command = game_command
                    command_published = False
                    if physical_output is not None:
                        # On the trigger frame publish one final neutral command
                        # before the deploy-only stop frame. Once the replacement
                        # process exists, repeat neutral start frames to cross the
                        # ZMQ subscriber handshake without forwarding user input.
                        if (
                            physical_recovery is not None
                            and physical_recovery.resident_policies
                        ):
                            # The same SONIC process keeps consuming neutral
                            # planner frames while KungFu owns LowCmd.
                            planner.send_game_command(game_command)
                            command_published = True
                        elif physical_output.request_sonic_stop:
                            planner.send_game_command(game_command)
                            command_published = True
                        elif (
                            physical_output.state
                            in {
                                RecoveryState.SONIC_RESTARTING,
                                RecoveryState.SONIC_STABILIZING,
                                RecoveryState.WAIT_NEUTRAL,
                            }
                            and processes.deploy_alive()
                        ):
                            planner.send_game_command(game_command)
                            command_published = True
                        elif not physical_output.inhibit_game_input:
                            planner.send_game_command(game_command)
                            command_published = True
                        if not running:
                            break
                        try:
                            deploy_generation_before = processes.deploy_generation
                            physical_recovery.execute(
                                physical_output,
                                processes=processes,
                                planner=planner,
                            )
                            if physical_output.start_sonic:
                                # Generation is incremented synchronously with
                                # Popen. Re-observe the still-stale snapshot
                                # before any physics tick so a very fast deploy
                                # cannot hide the required new-generation stale
                                # baseline before its first LowCmd.
                                physical_output = physical_recovery.observe(
                                    snapshot,
                                    now_s=frame_wall,
                                    neutral_confirmed=False,
                                    foot_contact=foot_contact,
                                    grounded_contact=grounded_contact,
                                    processes=processes,
                                )
                                physical_recovery.verify_writer_free_prewarm_start(
                                    physical_output,
                                    processes=processes,
                                    previous_generation=deploy_generation_before,
                                )
                                if processes.deploy_alive():
                                    planner.send_game_command(game_command)
                                    command_published = True
                        except (OSError, RuntimeError, ValueError) as exc:
                            unstable = True
                            running = False
                            termination_reason = "physical_recovery_failed"
                            numerical_error = f"physical_recovery_action:{exc}"
                            print(
                                "matrix-sonic-runtime ERROR physical recovery "
                                f"action: {exc}",
                                flush=True,
                            )
                            break
                        if physical_output.fail_closed:
                            unstable = True
                            running = False
                            termination_reason = "physical_recovery_failed"
                            numerical_error = (
                                "physical_recovery_fsm:"
                                f"{physical_output.failure_reason}"
                            )
                            break
                    elif (
                        game_fall_recovery is not None
                        and game_fall_recovery.recovering
                    ):
                        planner.send_recovery_posture(
                            locomotion_mode=game_fall_recovery.native_mode,
                            height=game_fall_recovery.target_height,
                            facing=game_command.facing,
                        )
                        command_published = True
                    else:
                        planner.send_game_command(game_command)
                        command_published = True
                    if command_published:
                        game_input.record_published_command(game_command)
                    walking = game_command.mode == "move"
                    if game_commands is not None:
                        command_allowed = bool(
                            game_command.safe_stop
                            and game_command.speed_mps == 0.0
                            and game_command.mode != "move"
                        )
                        try:
                            command_restart = game_commands.poll(
                                current_pose=_snapshot_world_pose(snapshot),
                                command_allowed=command_allowed,
                            )
                        except (EOFError, RuntimeError, WorldStateError) as exc:
                            game_input.core.invalidate_input(
                                "game_command_channel_error"
                            )
                            running = False
                            termination_reason = "game_command_channel_error"
                            numerical_error = str(exc)
                            print(
                                "matrix-sonic-runtime ERROR game command "
                                f"channel failed: {exc}",
                                flush=True,
                            )
                        else:
                            if command_restart:
                                game_command = game_input.emergency_stop(
                                    now_s=time.perf_counter(),
                                    reason="teleport_reload",
                                )
                                planner.send_game_command(game_command)
                                walking = False
                                running = False
                                termination_reason = "game_teleport"
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
                if bool(snapshot.fall_detected):
                    fall_detected = True
                    if args.game_auto_respawn:
                        assert game_world is not None
                        assert game_input is not None
                        assert planner is not None
                        game_command = game_input.emergency_stop(
                            now_s=time.perf_counter(),
                            reason="fall_respawn_reload",
                        )
                        planner.send_game_command(game_command)
                        walking = False
                        try:
                            game_world.checkpoint(
                                snapshot,
                                now_s=time.perf_counter(),
                                force=True,
                                required=True,
                            )
                        except WorldStateError as exc:
                            running = False
                            termination_reason = "world_state_error"
                            numerical_error = f"fall_checkpoint:{exc}"
                            print(
                                "matrix-sonic-runtime ERROR cannot save fall "
                                f"respawn checkpoint: {exc}",
                                flush=True,
                            )
                            break
                        running = False
                        termination_reason = "game_fall_respawn"
                        print(
                            "matrix-sonic-runtime fall detected; saved an "
                            "upright cold-respawn checkpoint",
                            flush=True,
                        )
                        break
                    if args.fail_on_fall:
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
            if game_world is not None:
                game_world.checkpoint(snapshot, now_s=freshness_sample_wall)
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
                        _game_control_status_fields(
                            args,
                            applied_camera_yaw_offset_deg=(
                                applied_game_camera_yaw_offset_deg
                            ),
                            initial_root_yaw_rad=initial_root_yaw_rad,
                        )
                    )
                    current_root_yaw = _root_yaw_rad(snapshot.qpos)
                    assert initial_root_yaw_rad is not None
                    assert heading_anchor_telemetry is not None
                    status.update(heading_anchor_telemetry.status_fields())
                    status["root_yaw_world_rad"] = round(current_root_yaw, 6)
                    status["root_yaw_relative_rad"] = round(
                        wrap_angle_rad(current_root_yaw - initial_root_yaw_rad), 6
                    )
                if game_world is not None:
                    status["game_world_state"] = game_world.telemetry()
                    status["game_auto_respawn"] = bool(args.game_auto_respawn)
                if game_commands is not None:
                    status["game_commands"] = game_commands.telemetry()
                    if game_fall_recovery is not None:
                        recovery_status = game_fall_recovery.status(now_s=now)
                        status["game_fall_recovery"] = recovery_status
                        status["current_fall_detected"] = recovery_status[
                            "current_fall_detected"
                        ]
                    elif physical_recovery is not None:
                        recovery_status = physical_recovery.telemetry(
                            now_s=now,
                            processes=processes,
                        )
                        status["game_fall_recovery"] = recovery_status
                        status["current_fall_detected"] = recovery_status[
                            "current_fall_detected"
                        ]
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
            boundary_measured_heading = None
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
            if physical_recovery is not None:
                game_command = physical_recovery.reframe_game_command(
                    game_command,
                    measured_heading_rad=boundary_measured_heading,
                    dt_s=0.0,
                )
            planner.send_game_command(game_command)
            walking = False

        if (
            game_world is not None
            and termination_reason not in _GAME_INTERNAL_RESTART_REASONS
        ):
            try:
                game_world.checkpoint(
                    snapshot,
                    now_s=time.perf_counter(),
                    force=True,
                    required=True,
                )
            except WorldStateError as exc:
                world_checkpoint_failed = True
                numerical_error = f"final_checkpoint:{exc}"
                print(
                    "matrix-sonic-runtime ERROR cannot save final game-world "
                    f"checkpoint: {exc}",
                    flush=True,
                )

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
        if world_checkpoint_failed:
            acceptance_failures.append("world_state_checkpoint_failed")
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
                _game_control_status_fields(
                    args,
                    applied_camera_yaw_offset_deg=(
                        applied_game_camera_yaw_offset_deg
                    ),
                    initial_root_yaw_rad=initial_root_yaw_rad,
                )
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
        internal_restart_requested = bool(
            termination_reason in _GAME_INTERNAL_RESTART_REASONS
            and termination_signal is None
            and child_failure is None
            and not unstable
            and not world_checkpoint_failed
        )
        final_status["internal_restart"] = {
            "requested": internal_restart_requested,
            "reason": termination_reason if internal_restart_requested else None,
        }
        final_status["game_auto_respawn"] = bool(args.game_auto_respawn)
        if game_world is not None:
            final_status["game_world_state"] = game_world.telemetry()
        if game_commands is not None:
            final_status["game_commands"] = game_commands.telemetry()
            if game_fall_recovery is not None:
                recovery_status = game_fall_recovery.status(now_s=finished_wall)
                final_status["game_fall_recovery"] = recovery_status
                final_status["current_fall_detected"] = recovery_status[
                    "current_fall_detected"
                ]
            elif physical_recovery is not None:
                recovery_status = physical_recovery.telemetry(
                    now_s=finished_wall,
                    processes=processes,
                )
                final_status["game_fall_recovery"] = recovery_status
                final_status["current_fall_detected"] = recovery_status[
                    "current_fall_detected"
                ]
        _atomic_json(args.status_file, final_status)
        print(
            "matrix-sonic-runtime stopped "
            f"wall_s={elapsed_wall_s:.2f} sim_s={snapshot.sim_time:.2f} "
            f"frames={control_frames} active_frames={active_frames} "
            f"reason={termination_reason} passed={passed} "
            f"failures={acceptance_failures}",
            flush=True,
        )
        if internal_restart_requested:
            proposed_exit_code = _GAME_INTERNAL_RESTART_EXIT_CODE
        elif passed or (
            not qualification_attempted
            and interrupted
            and not acceptance_failures
        ):
            proposed_exit_code = 0
        else:
            proposed_exit_code = 2
        return 0 if passed else 2
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
        error = _close_runtime_resource("game commands", game_commands)
        if error is not None:
            cleanup_errors.append(error)
        error = _close_runtime_resource(
            "game command child socket", game_command_child_socket
        )
        if error is not None:
            cleanup_errors.append(error)
        for name, resource in (
            ("physical recovery", physical_recovery),
            ("native processes", processes),
            ("renderer", renderer),
            ("simulator", simulator),
        ):
            error = _close_runtime_resource(name, resource)
            if error is not None:
                cleanup_errors.append(error)
        if (
            not active_exception
            and not cleanup_errors
            and proposed_exit_code == _GAME_INTERNAL_RESTART_EXIT_CODE
        ):
            try:
                termination_boundary_previous_mask = signal.pthread_sigmask(
                    signal.SIG_BLOCK,
                    {signal.SIGINT, signal.SIGTERM},
                )
                termination_boundary_safe = True
            except (AttributeError, OSError, ValueError) as exc:
                print(
                    "matrix-sonic-runtime ERROR cannot close the internal "
                    f"restart signal boundary: {exc}",
                    flush=True,
                )
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
            if termination_boundary_previous_mask is not None:
                signal.pthread_sigmask(
                    signal.SIG_SETMASK,
                    termination_boundary_previous_mask,
                )
            _record_cleanup_failure(args.status_file, cleanup_errors)
            if not active_exception:
                raise RuntimeError(
                    "native cleanup failed: " + "; ".join(cleanup_errors)
                )

    try:
        if termination_boundary_previous_mask is not None:
            try:
                pending_termination_signals = signal.sigpending().intersection(
                    {signal.SIGINT, signal.SIGTERM}
                )
            except (AttributeError, OSError, ValueError) as exc:
                termination_boundary_safe = False
                print(
                    "matrix-sonic-runtime ERROR cannot inspect pending "
                    f"termination signals: {exc}",
                    flush=True,
                )
            else:
                if pending_termination_signals:
                    termination_signal = int(
                        min(pending_termination_signals, key=int)
                    )

        status_changed = False
        if (
            final_status is not None
            and final_status.get("termination_signal") != termination_signal
        ):
            final_status["termination_signal"] = termination_signal
            status_changed = True
        if proposed_exit_code == _GAME_INTERNAL_RESTART_EXIT_CODE and (
            termination_signal is not None or not termination_boundary_safe
        ):
            proposed_exit_code = 2
            if final_status is not None:
                final_status["internal_restart"] = {
                    "requested": False,
                    "reason": None,
                }
                status_changed = True
        if status_changed:
            _atomic_json(args.status_file, final_status)
    finally:
        if termination_boundary_previous_mask is not None:
            signal.pthread_sigmask(
                signal.SIG_SETMASK,
                termination_boundary_previous_mask,
            )
    return proposed_exit_code


if __name__ == "__main__":
    raise SystemExit(main())
