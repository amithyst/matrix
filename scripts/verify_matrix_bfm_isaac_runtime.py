#!/usr/bin/env python3
"""Verify the frozen Leo BFM/Isaac runtime and its 200/50 Hz evidence."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
from typing import Any, Iterable
import xml.etree.ElementTree as ET

try:
    import tomllib
except ModuleNotFoundError:  # Python 3.10 host bootstrap path
    tomllib = None  # type: ignore[assignment]


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LOCK = REPO_ROOT / "config/runtime/matrix-bfm-isaac.lock.json"
RELAY_STATUS_SCHEMA = "matrix_bfm_isaac_relay_status.v1"
ACCEPTANCE_SCHEMA = "matrix_bfm_isaac_acceptance.v2"
RUNTIME_VERIFICATION_SCHEMA = "matrix_bfm_isaac_runtime_verification.v1"
VIDEO_SETTINGS_SCHEMA = "matrix_bfm_isaac_video_settings.v1"
VISUAL_VENV_MARKER_SCHEMA = "matrix_bfm_isaac_visual_venv.v1"
SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")
COMMIT_PATTERN = re.compile(r"[0-9a-f]{40}")
BUILD_ID_PATTERN = re.compile(r"[0-9a-f]{16,64}")
REQUIREMENT_PATTERN = re.compile(r"([A-Za-z0-9_.-]+)==([A-Za-z0-9_.+]+)")

EXPECTED_MATRIX_JOINT_ORDER = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)
EXPECTED_ISAACLAB_JOINT_ORDER = (
    "left_hip_pitch_joint",
    "right_hip_pitch_joint",
    "waist_yaw_joint",
    "left_hip_roll_joint",
    "right_hip_roll_joint",
    "waist_roll_joint",
    "left_hip_yaw_joint",
    "right_hip_yaw_joint",
    "waist_pitch_joint",
    "left_knee_joint",
    "right_knee_joint",
    "left_shoulder_pitch_joint",
    "right_shoulder_pitch_joint",
    "left_ankle_pitch_joint",
    "right_ankle_pitch_joint",
    "left_shoulder_roll_joint",
    "right_shoulder_roll_joint",
    "left_ankle_roll_joint",
    "right_ankle_roll_joint",
    "left_shoulder_yaw_joint",
    "right_shoulder_yaw_joint",
    "left_elbow_joint",
    "right_elbow_joint",
    "left_wrist_roll_joint",
    "right_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "right_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_wrist_yaw_joint",
)
EXPECTED_ISAACLAB_TO_MATRIX_SOURCE_INDICES = (
    0,
    3,
    6,
    9,
    13,
    17,
    1,
    4,
    7,
    10,
    14,
    18,
    2,
    5,
    8,
    11,
    15,
    19,
    21,
    23,
    25,
    27,
    12,
    16,
    20,
    22,
    24,
    26,
    28,
)

EXPECTED_EXECUTION_CONTRACT = {
    "physics_device": "cpu",
    "reference_device": "cuda:0",
    "physics_command_write_mode": "implicit_once_per_control_interval",
    "physics_command_writes_per_articulation_per_control_step": 1,
    "articulation_updates_per_articulation_per_control_step": 1,
    "articulation_update_dt_s": 0.02,
    "teacher_onnx_session": {
        "providers": ["CPUExecutionProvider"],
        "execution_mode": "ORT_SEQUENTIAL",
        "intra_op_num_threads": 1,
        "inter_op_num_threads": 1,
        "intra_op_allow_spinning": False,
        "inter_op_allow_spinning": False,
    },
}


@dataclass(frozen=True)
class Check:
    name: str
    ok: bool
    detail: str
    gate: str = "runtime"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _is_number(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _finite_number(value: object) -> bool:
    return _is_number(value) and math.isfinite(float(value))


def _positive_number(value: object) -> bool:
    return _finite_number(value) and float(value) > 0.0


def _nonnegative_integer(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def _safe_relative_path(value: object) -> bool:
    if not isinstance(value, str) or not value or "\\" in value:
        return False
    path = Path(value)
    return not path.is_absolute() and ".." not in path.parts and path.as_posix() == value


def _require_exact_keys(payload: dict[str, Any], expected: set[str], label: str) -> None:
    actual = set(payload)
    if actual != expected:
        missing = sorted(expected - actual)
        extra = sorted(actual - expected)
        raise ValueError(f"{label} keys mismatch: missing={missing} extra={extra}")


def _require_sha(value: object, label: str) -> str:
    if not isinstance(value, str) or SHA256_PATTERN.fullmatch(value) is None:
        raise ValueError(f"{label} must be a lowercase SHA256")
    return value


def _require_commit(value: object, label: str) -> str:
    if not isinstance(value, str) or COMMIT_PATTERN.fullmatch(value) is None:
        raise ValueError(f"{label} must be a full lowercase Git commit")
    return value


def validate_schema(lock: dict[str, Any]) -> None:
    _require_exact_keys(
        lock,
        {
            "schema_version",
            "runtime_id",
            "matrix_port",
            "ue_material_bridge",
            "leo_release",
            "bfm_runtime",
            "isaac_runtime",
            "physics_assets",
            "visual_import",
            "matrix_visual",
            "policy",
            "clock_contract",
            "execution_contract",
            "scene_assets",
            "scene_collision_contract",
            "wire_contract",
            "acceptance",
        },
        "lock",
    )
    if lock["schema_version"] != 5:
        raise ValueError("unsupported matrix-bfm-isaac lock schema")
    if lock["runtime_id"] != "matrix-bfm-isaac-sync-world16-v1":
        raise ValueError("unexpected runtime_id")

    matrix_port = lock["matrix_port"]
    if not isinstance(matrix_port, dict):
        raise ValueError("matrix_port must be an object")
    _require_exact_keys(
        matrix_port,
        {"repository", "base_commit", "critical_files"},
        "matrix_port",
    )
    _require_commit(matrix_port["base_commit"], "matrix_port.base_commit")
    matrix_critical_files = matrix_port["critical_files"]
    if not isinstance(matrix_critical_files, list) or not matrix_critical_files:
        raise ValueError("matrix_port.critical_files must be non-empty")
    matrix_critical_paths: set[str] = set()
    for index, entry in enumerate(matrix_critical_files):
        if not isinstance(entry, dict):
            raise ValueError(f"matrix_port.critical_files[{index}] must be an object")
        _require_exact_keys(
            entry,
            {"path", "sha256"},
            f"matrix_port.critical_files[{index}]",
        )
        path = entry["path"]
        if not _safe_relative_path(path) or path in matrix_critical_paths:
            raise ValueError(f"unsafe or duplicate Matrix critical path: {path!r}")
        matrix_critical_paths.add(path)
        _require_sha(
            entry["sha256"],
            f"matrix_port.critical_files[{index}].sha256",
        )
    required_matrix_critical = {
        "config/hosts/heyuan.env",
        "config/hosts/trna.env",
        "config/hosts/zza.env",
        "config/runtime/matrix-bfm-isaac-bootstrap-state.json",
        "config/runtime/matrix-bfm-isaac-video-settings.json",
        "scripts/bootstrap_matrix_bfm_isaac.sh",
        "scripts/build_matrix_ue_material_fix.sh",
        "scripts/launch_matrix_bfm_isaac_desktop.sh",
        "scripts/launch_matrix_sonic_desktop.sh",
        "scripts/matrix_bfm_isaac_checkout_snapshot.py",
        "scripts/matrix_bfm_isaac_video_settings.py",
        "scripts/matrix_local_env.sh",
        "scripts/run_matrix_sonic.sh",
        "scripts/run_matrix_sonic_moon_v1.sh",
        "scripts/run_custom_urdf.sh",
        "scripts/run_sim.sh",
        "scripts/run_matrix_bfm_isaac.sh",
        "scripts/run_matrix_bfm_isaac_guarded.sh",
        "scripts/matrix_external_state_relay.py",
        "scripts/validate_matrix_bfm_isaac_desktop_shortcut.sh",
        "scripts/verify_matrix_bfm_isaac_runtime.py",
        "src/ue_shims/matrix_ue_material_fix.c",
    }
    if not required_matrix_critical.issubset(matrix_critical_paths):
        raise ValueError("matrix_port critical closure is incomplete")

    material_bridge = lock["ue_material_bridge"]
    if not isinstance(material_bridge, dict):
        raise ValueError("ue_material_bridge must be an object")
    _require_exact_keys(
        material_bridge,
        {
            "relative_path",
            "sha256",
            "ue_binary_relative_path",
            "ue_binary_build_id",
        },
        "ue_material_bridge",
    )
    if (
        material_bridge["relative_path"]
        != "outputs/runtime/matrix-ue-material-fix/libmatrix_ue_material_fix.so"
        or not _safe_relative_path(material_bridge["relative_path"])
    ):
        raise ValueError("unexpected UE material bridge path")
    if (
        material_bridge["ue_binary_relative_path"]
        != "src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux/"
        "zsibot_mujoco_ue"
        or not _safe_relative_path(material_bridge["ue_binary_relative_path"])
    ):
        raise ValueError("unexpected Matrix UE binary path")
    _require_sha(material_bridge["sha256"], "ue_material_bridge.sha256")
    build_id = material_bridge["ue_binary_build_id"]
    if not isinstance(build_id, str) or BUILD_ID_PATTERN.fullmatch(build_id) is None:
        raise ValueError("ue_material_bridge.ue_binary_build_id is invalid")
    if build_id != "056e17b8675b1006":
        raise ValueError("unexpected Matrix UE Build ID")

    release = lock["leo_release"]
    if not isinstance(release, dict):
        raise ValueError("leo_release must be an object")
    _require_exact_keys(
        release,
        {
            "integration_repository",
            "integration_commit",
            "documentation_commit",
            "tag",
        },
        "leo_release",
    )
    _require_commit(release["integration_commit"], "leo_release.integration_commit")
    _require_commit(release["documentation_commit"], "leo_release.documentation_commit")
    if release["tag"] != "v0.1.0-world16-step079000":
        raise ValueError("unexpected Leo release tag")

    runtime = lock["bfm_runtime"]
    if not isinstance(runtime, dict):
        raise ValueError("bfm_runtime must be an object")
    _require_exact_keys(runtime, {"repository", "commit", "critical_files"}, "bfm_runtime")
    _require_commit(runtime["commit"], "bfm_runtime.commit")
    critical_files = runtime["critical_files"]
    if not isinstance(critical_files, list) or not critical_files:
        raise ValueError("bfm_runtime.critical_files must be non-empty")
    critical_paths: set[str] = set()
    for index, entry in enumerate(critical_files):
        if not isinstance(entry, dict):
            raise ValueError(f"critical_files[{index}] must be an object")
        _require_exact_keys(entry, {"path", "sha256"}, f"critical_files[{index}]")
        path = entry["path"]
        if not _safe_relative_path(path) or path in critical_paths:
            raise ValueError(f"unsafe or duplicate critical path: {path!r}")
        critical_paths.add(path)
        _require_sha(entry["sha256"], f"critical_files[{index}].sha256")
    required_critical = {
        "scripts/run_g1_teacher_closed_loop.py",
        "src/bfm_sonic_realscan_play/implicit_control_step.py",
        "src/bfm_sonic_realscan_play/matrix_state_sink.py",
        "configs/base.toml",
        "configs/alienware/moon-matrix.toml",
    }
    if not required_critical.issubset(critical_paths):
        raise ValueError("bfm_runtime critical closure is incomplete")

    isaac_runtime = lock["isaac_runtime"]
    if not isinstance(isaac_runtime, dict):
        raise ValueError("isaac_runtime must be an object")
    _require_exact_keys(
        isaac_runtime,
        {
            "python_implementation",
            "python_version",
            "platform_machine",
            "include_system_site_packages",
            "distributions",
            "isaaclab",
        },
        "isaac_runtime",
    )
    if (
        isaac_runtime["python_implementation"] != "CPython"
        or isaac_runtime["python_version"] != "3.11.15"
        or isaac_runtime["platform_machine"] != "x86_64"
        or isaac_runtime["include_system_site_packages"] is not True
    ):
        raise ValueError("unexpected Isaac runtime Python identity")
    distributions = isaac_runtime["distributions"]
    expected_distributions = {
        "isaaclab": "0.54.2",
        "isaacsim": "5.1.0.0",
        "numpy": "1.26.4",
        "onnxruntime": "1.22.1",
        "torch": "2.7.0+cu128",
    }
    if distributions != expected_distributions:
        raise ValueError("unexpected Isaac runtime distributions")
    isaaclab = isaac_runtime["isaaclab"]
    if not isinstance(isaaclab, dict):
        raise ValueError("isaac_runtime.isaaclab must be an object")
    _require_exact_keys(
        isaaclab,
        {
            "repository",
            "commit",
            "module_path",
            "critical_files",
            "allowed_status",
            "unused_paths",
        },
        "isaac_runtime.isaaclab",
    )
    _require_commit(isaaclab["commit"], "isaac_runtime.isaaclab.commit")
    module_path = isaaclab["module_path"]
    if module_path != "source/isaaclab/isaaclab/__init__.py":
        raise ValueError("unexpected IsaacLab module path")
    overlay_paths: set[str] = set()
    for index, entry in enumerate(isaaclab["critical_files"]):
        if not isinstance(entry, dict):
            raise ValueError(f"isaaclab.critical_files[{index}] must be an object")
        _require_exact_keys(
            entry,
            {"path", "sha256"},
            f"isaaclab.critical_files[{index}]",
        )
        path = entry["path"]
        if not _safe_relative_path(path) or path in overlay_paths:
            raise ValueError(f"unsafe or duplicate IsaacLab path: {path!r}")
        overlay_paths.add(path)
        _require_sha(entry["sha256"], f"isaaclab.critical_files[{index}].sha256")
    expected_overlay_paths = {
        "source/isaaclab/isaaclab/sim/converters/urdf_converter.py",
        "source/isaaclab/isaaclab/sim/utils/prims.py",
    }
    if overlay_paths != expected_overlay_paths:
        raise ValueError("IsaacLab compatibility overlay closure is incomplete")
    allowed_status = isaaclab["allowed_status"]
    if not isinstance(allowed_status, list) or allowed_status != [
        f" M {path}" for path in sorted(expected_overlay_paths)
    ] + ["?? apps/isaaclab.python.sonic.kit"]:
        raise ValueError("unexpected IsaacLab checkout status allowlist")
    unused_paths = isaaclab["unused_paths"]
    if unused_paths != ["apps/isaaclab.python.sonic.kit"]:
        raise ValueError("unexpected IsaacLab unused-path allowlist")

    physics_assets = lock["physics_assets"]
    if not isinstance(physics_assets, dict):
        raise ValueError("physics_assets must be an object")
    _require_exact_keys(
        physics_assets,
        {"manifest", "manifest_sha256", "file_count", "main_usd", "main_usd_sha256"},
        "physics_assets",
    )
    if (
        physics_assets["manifest"]
        != "config/runtime/matrix-bfm-isaac-g1-usd.SHA256SUMS"
        or physics_assets["file_count"] != 7
        or physics_assets["main_usd"] != "main_nodex.usd"
    ):
        raise ValueError("unexpected frozen PhysX asset closure")
    _require_sha(physics_assets["manifest_sha256"], "physics_assets.manifest_sha256")
    _require_sha(physics_assets["main_usd_sha256"], "physics_assets.main_usd_sha256")

    visual_import = lock["visual_import"]
    if not isinstance(visual_import, dict):
        raise ValueError("visual_import must be an object")
    _require_exact_keys(
        visual_import,
        {
            "python_implementation",
            "python_version",
            "platform_machine",
            "requirements",
            "requirements_sha256",
            "wheelhouse_manifest",
            "wheelhouse_manifest_sha256",
            "wheel_count",
            "wheel_bytes",
            "required_imports",
        },
        "visual_import",
    )
    if (
        visual_import["python_implementation"] != "CPython"
        or visual_import["python_version"] != "3.10"
        or visual_import["platform_machine"] != "x86_64"
        or visual_import["requirements"]
        != "config/runtime/matrix-bfm-isaac-visual-requirements.txt"
        or visual_import["wheelhouse_manifest"]
        != "config/runtime/matrix-bfm-isaac-visual-wheelhouse.SHA256SUMS"
        or visual_import["wheel_count"] != 11
        or visual_import["wheel_bytes"] != 27402051
        or visual_import["required_imports"]
        != ["mujoco", "numpy", "urdf2mjcf.convert"]
    ):
        raise ValueError("unexpected visual-import closure")
    _require_sha(
        visual_import["requirements_sha256"],
        "visual_import.requirements_sha256",
    )
    _require_sha(
        visual_import["wheelhouse_manifest_sha256"],
        "visual_import.wheelhouse_manifest_sha256",
    )

    matrix_visual = lock["matrix_visual"]
    if not isinstance(matrix_visual, dict):
        raise ValueError("matrix_visual must be an object")
    _require_exact_keys(
        matrix_visual,
        {
            "manifest",
            "manifest_sha256",
            "file_count",
            "urdf",
            "urdf_sha256",
        },
        "matrix_visual",
    )
    if (
        matrix_visual["manifest"]
        != "config/runtime/matrix-bfm-isaac-g1-visual.SHA256SUMS"
        or matrix_visual["file_count"] != 37
        or matrix_visual["urdf"] != "g1_29dof.urdf"
    ):
        raise ValueError("unexpected Matrix G1 visual closure")
    _require_sha(
        matrix_visual["manifest_sha256"],
        "matrix_visual.manifest_sha256",
    )
    _require_sha(matrix_visual["urdf_sha256"], "matrix_visual.urdf_sha256")

    policy = lock["policy"]
    if not isinstance(policy, dict):
        raise ValueError("policy must be an object")
    _require_exact_keys(
        policy,
        {
            "profile_id",
            "training_task",
            "training_source_commit",
            "artifacts",
            "onnx_contract",
        },
        "policy",
    )
    if policy["profile_id"] != "bfm-sonic-world16-step079000":
        raise ValueError("unexpected policy profile_id")
    _require_commit(policy["training_source_commit"], "policy.training_source_commit")
    artifacts = policy["artifacts"]
    if not isinstance(artifacts, list) or len(artifacts) != 3:
        raise ValueError("policy.artifacts must contain checkpoint/config/ONNX")
    artifact_names: set[str] = set()
    for index, entry in enumerate(artifacts):
        if not isinstance(entry, dict):
            raise ValueError(f"policy.artifacts[{index}] must be an object")
        _require_exact_keys(entry, {"name", "sha256"}, f"policy.artifacts[{index}]")
        name = entry["name"]
        if not isinstance(name, str) or Path(name).name != name or name in artifact_names:
            raise ValueError(f"invalid or duplicate policy artifact name: {name!r}")
        artifact_names.add(name)
        _require_sha(entry["sha256"], f"policy.artifacts[{index}].sha256")
    if artifact_names != {
        "model_step_079000.pt",
        "config.yaml",
        "model_step_079000_g1.onnx",
    }:
        raise ValueError("policy artifact names do not match world16 step079000")
    onnx = policy["onnx_contract"]
    if not isinstance(onnx, dict):
        raise ValueError("policy.onnx_contract must be an object")
    _require_exact_keys(
        onnx,
        {"input_name", "input_width", "output_name", "output_width"},
        "policy.onnx_contract",
    )
    if onnx != {
        "input_name": "obs_dict",
        "input_width": 1790,
        "output_name": "action",
        "output_width": 29,
    }:
        raise ValueError("unexpected world16 ONNX contract")

    clock = lock["clock_contract"]
    if not isinstance(clock, dict):
        raise ValueError("clock_contract must be an object")
    _require_exact_keys(
        clock,
        {
            "physics_dt_s",
            "physics_hz_sim",
            "control_hz_sim",
            "decimation",
            "action_hold_substeps",
        },
        "clock_contract",
    )
    if (
        clock["physics_dt_s"] != 0.005
        or clock["physics_hz_sim"] != 200.0
        or clock["control_hz_sim"] != 50.0
        or clock["decimation"] != 4
        or clock["action_hold_substeps"] != 4
    ):
        raise ValueError("clock contract must be exactly 200 Hz / 50 Hz / decimation 4")
    if not math.isclose(
        float(clock["physics_dt_s"]) * float(clock["physics_hz_sim"]),
        1.0,
        abs_tol=1.0e-12,
    ):
        raise ValueError("physics_dt_s and physics_hz_sim disagree")
    if not math.isclose(
        float(clock["physics_hz_sim"]) / float(clock["control_hz_sim"]),
        float(clock["decimation"]),
        abs_tol=1.0e-12,
    ):
        raise ValueError("physics/control rates and decimation disagree")

    execution = lock["execution_contract"]
    if not isinstance(execution, dict):
        raise ValueError("execution_contract must be an object")
    if execution != EXPECTED_EXECUTION_CONTRACT:
        raise ValueError(
            "execution contract must pin CPU PhysX, CUDA Robo-PFNN, "
            "implicit once-write, and the single-thread Teacher ORT session"
        )

    scene_assets = lock["scene_assets"]
    if not isinstance(scene_assets, dict):
        raise ValueError("scene_assets must be an object")
    _require_exact_keys(
        scene_assets,
        {
            "visual_bin",
            "visual_bin_sha256",
            "visual_bin_bytes",
            "collision_manifest",
            "collision_manifest_sha256",
            "collision_usd",
            "collision_usd_sha256",
        },
        "scene_assets",
    )
    if (
        scene_assets["visual_bin"] != "dynamicmaps/moonworld.bin"
        or scene_assets["visual_bin_bytes"] != 144000000
        or scene_assets["collision_manifest"] != "manifest.json"
        or scene_assets["collision_usd"] != "collision.usda"
    ):
        raise ValueError("unexpected Moon visual/collision asset closure")
    for field in (
        "visual_bin_sha256",
        "collision_manifest_sha256",
        "collision_usd_sha256",
    ):
        _require_sha(scene_assets[field], f"scene_assets.{field}")

    scene = lock["scene_collision_contract"]
    if not isinstance(scene, dict):
        raise ValueError("scene_collision_contract must be an object")
    _require_exact_keys(
        scene,
        {
            "scene_id",
            "runtime_config_suffix",
            "root_position_indices",
            "x_min_m",
            "x_max_m",
            "y_min_m",
            "y_max_m",
            "warning_margin_m",
            "stop_margin_m",
            "boundary_guard_required",
        },
        "scene_collision_contract",
    )
    if (
        scene["scene_id"] != 15
        or scene["runtime_config_suffix"]
        != "configs/alienware/moon-matrix.toml"
        or scene["root_position_indices"] != [0, 1]
        or scene["boundary_guard_required"] is not True
    ):
        raise ValueError("unexpected Moon scene collision identity")
    bounds = (
        scene["x_min_m"],
        scene["x_max_m"],
        scene["y_min_m"],
        scene["y_max_m"],
        scene["warning_margin_m"],
        scene["stop_margin_m"],
    )
    if not all(_finite_number(value) for value in bounds):
        raise ValueError("scene collision bounds/margins must be finite")
    x_min, x_max, y_min, y_max, warning_margin, stop_margin = map(float, bounds)
    half_short_side = 0.5 * min(x_max - x_min, y_max - y_min)
    if (
        x_min >= x_max
        or y_min >= y_max
        or not 0.0 < stop_margin < warning_margin < half_short_side
    ):
        raise ValueError("invalid scene collision bounds or safety margins")

    wire = lock["wire_contract"]
    if not isinstance(wire, dict):
        raise ValueError("wire_contract must be an object")
    _require_exact_keys(
        wire,
        {
            "endianness",
            "root_frame",
            "root_quaternion_order",
            "input",
            "matrix_output",
            "joint_order",
            "isaaclab_joint_order",
            "isaaclab_to_matrix_source_indices",
        },
        "wire_contract",
    )
    if (
        wire["endianness"] != "little"
        or wire["root_frame"] != "RH_Z_UP_METRES"
        or wire["root_quaternion_order"] != "wxyz"
    ):
        raise ValueError("unexpected Matrix coordinate/wire convention")
    expected_dimensions = {
        "input": {"nq": 36, "nv": 35, "nu": 0, "packet_bytes": 588},
        "matrix_output": {
            "nq": 36,
            "nv": 35,
            "nu": 29,
            "packet_bytes": 820,
            "ctrl_fill": 0.0,
        },
    }
    for name, expected in expected_dimensions.items():
        if wire[name] != expected:
            raise ValueError(f"wire_contract.{name} does not match frozen ABI")
    if (
        not isinstance(wire["joint_order"], list)
        or tuple(wire["joint_order"]) != EXPECTED_MATRIX_JOINT_ORDER
    ):
        raise ValueError("wire_contract.joint_order is not the frozen Matrix order")
    if (
        not isinstance(wire["isaaclab_joint_order"], list)
        or tuple(wire["isaaclab_joint_order"]) != EXPECTED_ISAACLAB_JOINT_ORDER
    ):
        raise ValueError("wire_contract.isaaclab_joint_order is not frozen")
    raw_source_indices = wire["isaaclab_to_matrix_source_indices"]
    if not isinstance(raw_source_indices, list):
        raise ValueError("wire_contract joint mapping must be a list")
    source_indices = tuple(raw_source_indices)
    if source_indices != EXPECTED_ISAACLAB_TO_MATRIX_SOURCE_INDICES:
        raise ValueError("wire_contract IsaacLab-to-Matrix mapping is not frozen")
    derived_indices = tuple(
        EXPECTED_ISAACLAB_JOINT_ORDER.index(name)
        for name in EXPECTED_MATRIX_JOINT_ORDER
    )
    if source_indices != derived_indices:
        raise ValueError("wire_contract joint mapping disagrees with joint names")

    acceptance = lock["acceptance"]
    if not isinstance(acceptance, dict):
        raise ValueError("acceptance must be an object")
    _require_exact_keys(acceptance, {"correctness", "realtime"}, "acceptance")
    correctness = acceptance["correctness"]
    realtime = acceptance["realtime"]
    if not isinstance(correctness, dict) or not isinstance(realtime, dict):
        raise ValueError("acceptance gates must be objects")
    expected_correctness_keys = {
        "fall_count_max",
        "recovery_count_max",
        "matrix_state_frames_dropped_max",
        "relay_invalid_frames_max",
        "relay_sequence_gaps_max",
        "relay_duplicate_frames_max",
        "relay_out_of_order_frames_max",
        "boundary_stop_events_max",
        "boundary_command_errors_max",
        "boundary_hard_violations_max",
        "required_schedule_modes",
        "required_observed_gaits",
        "height_raycast_hits_min",
        "height_query_path",
        "root_clearance_min_m",
        "root_clearance_max_m",
        "reference_source",
        "reference_source_hz",
        "reference_output_hz",
        "reference_buffer_swap_count_min",
        "reference_pending_elapsed_steps_max",
        "reference_root_xy_error_p95_m_max",
        "reference_root_yaw_error_p95_rad_max",
        "reference_root_tilt_error_p95_rad_max",
        "reference_joint_tracking_rmse_rad_max",
    }
    _require_exact_keys(correctness, expected_correctness_keys, "acceptance.correctness")
    integer_keys = {
        "fall_count_max",
        "recovery_count_max",
        "matrix_state_frames_dropped_max",
        "relay_invalid_frames_max",
        "relay_sequence_gaps_max",
        "relay_duplicate_frames_max",
        "relay_out_of_order_frames_max",
        "boundary_stop_events_max",
        "boundary_command_errors_max",
        "boundary_hard_violations_max",
        "height_raycast_hits_min",
        "reference_buffer_swap_count_min",
        "reference_pending_elapsed_steps_max",
    }
    if any(not _nonnegative_integer(correctness[key]) for key in integer_keys):
        raise ValueError("acceptance.correctness integer gates must be non-negative")
    if correctness["required_schedule_modes"] != [
        "stand",
        "walk",
        "jog",
        "turn_left",
        "turn_right",
        "rotate_left",
        "rotate_right",
    ]:
        raise ValueError("unexpected required schedule coverage")
    if correctness["required_observed_gaits"] != ["stand", "walk", "jog"]:
        raise ValueError("unexpected required gait coverage")
    if (
        correctness["height_raycast_hits_min"] != 121
        or correctness["height_query_path"] != "/World/Collision/terrain"
        or correctness["reference_source"] != "robo_pfnn_formal7168"
        or correctness["reference_source_hz"] != 60.0
        or correctness["reference_output_hz"] != 50.0
        or correctness["reference_pending_elapsed_steps_max"] != 4
    ):
        raise ValueError("unexpected terrain/reference correctness contract")
    numeric_correctness_keys = {
        "root_clearance_min_m",
        "root_clearance_max_m",
        "reference_root_xy_error_p95_m_max",
        "reference_root_yaw_error_p95_rad_max",
        "reference_root_tilt_error_p95_rad_max",
        "reference_joint_tracking_rmse_rad_max",
    }
    if any(
        not _positive_number(correctness[key]) for key in numeric_correctness_keys
    ):
        raise ValueError("acceptance.correctness numeric gates must be positive")
    if correctness["root_clearance_min_m"] >= correctness["root_clearance_max_m"]:
        raise ValueError("root clearance correctness range is invalid")
    _require_exact_keys(
        realtime,
        {
            "physics_hz_wall_min",
            "control_hz_wall_min",
            "simulation_realtime_factor_min",
            "control_step_wall_ms_p95_max",
        },
        "acceptance.realtime",
    )
    if any(not _positive_number(value) for value in realtime.values()):
        raise ValueError("acceptance.realtime values must be positive finite numbers")


def load_lock(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("runtime lock must be a JSON object")
    validate_schema(payload)
    return payload


def _run_git(root: Path, *args: str) -> str:
    result = subprocess.run(
        ("git", "-C", str(root), *args),
        check=True,
        capture_output=True,
        text=True,
        timeout=15,
    )
    # Porcelain status uses its two leading columns as data.  Remove only line
    # endings so the first entry cannot lose its index/worktree status byte.
    return result.stdout.rstrip("\r\n")


def verify_runtime_checkout(lock: dict[str, Any], root: Path) -> list[Check]:
    checks: list[Check] = []
    expected_commit = lock["bfm_runtime"]["commit"]
    try:
        actual_commit = _run_git(root, "rev-parse", "HEAD")
    except (OSError, subprocess.SubprocessError) as exc:
        return [Check("runtime_git_checkout", False, str(exc))]
    checks.append(
        Check(
            "runtime_commit",
            actual_commit == expected_commit,
            f"expected={expected_commit} actual={actual_commit}",
        )
    )
    try:
        checkout_status = _run_git(
            root,
            "status",
            "--porcelain",
            "--untracked-files=all",
        )
    except (OSError, subprocess.SubprocessError) as exc:
        checks.append(Check("runtime_checkout_clean", False, str(exc)))
    else:
        checks.append(
            Check(
                "runtime_checkout_clean",
                checkout_status == "",
                "clean" if checkout_status == "" else checkout_status,
            )
        )
    for entry in lock["bfm_runtime"]["critical_files"]:
        path = root / entry["path"]
        if not path.is_file() or path.is_symlink():
            checks.append(Check(f"runtime_file:{entry['path']}", False, "missing or symlink"))
            continue
        actual = sha256_file(path)
        checks.append(
            Check(
                f"runtime_file:{entry['path']}",
                actual == entry["sha256"],
                f"expected={entry['sha256']} actual={actual}",
            )
        )
    return checks


def verify_matrix_port(
    lock: dict[str, Any], root: Path
) -> tuple[list[Check], str | None]:
    """Verify the clean Matrix checkout and the port's locked critical files."""

    checks: list[Check] = []
    try:
        actual_commit = _run_git(root, "rev-parse", "HEAD")
        checkout_status = _run_git(
            root,
            "status",
            "--porcelain=v1",
            "--untracked-files=all",
        )
        ancestry = subprocess.run(
            (
                "git",
                "-C",
                str(root),
                "merge-base",
                "--is-ancestor",
                lock["matrix_port"]["base_commit"],
                "HEAD",
            ),
            check=False,
            capture_output=True,
            text=True,
            timeout=15,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return [Check("matrix_port_git_checkout", False, str(exc))], None
    checks.extend(
        [
            Check(
                "matrix_port_base_ancestor",
                ancestry.returncode == 0,
                f"base={lock['matrix_port']['base_commit']} head={actual_commit}",
            ),
            Check(
                "matrix_port_checkout_clean",
                checkout_status == "",
                "clean" if checkout_status == "" else checkout_status,
            ),
        ]
    )
    for entry in lock["matrix_port"]["critical_files"]:
        path = root / entry["path"]
        if path.is_symlink() or not path.is_file():
            checks.append(
                Check(
                    f"matrix_port_file:{entry['path']}",
                    False,
                    "missing or symlink",
                )
            )
            continue
        actual_hash = sha256_file(path)
        checks.append(
            Check(
                f"matrix_port_file:{entry['path']}",
                actual_hash == entry["sha256"],
                f"expected={entry['sha256']} actual={actual_hash}",
            )
        )
    return checks, actual_commit


def verify_ue_material_bridge(
    lock: dict[str, Any], matrix_root: Path, candidate: Path
) -> tuple[list[Check], dict[str, object]]:
    """Verify the exact bridge bytes and Matrix UE executable identity."""

    bridge = lock["ue_material_bridge"]
    expected_path = Path(
        os.path.abspath(matrix_root / bridge["relative_path"])
    )
    candidate_path = Path(os.path.abspath(candidate))
    checks = [
        Check(
            "ue_material_bridge_path",
            candidate_path == expected_path,
            f"expected={expected_path} actual={candidate_path}",
        )
    ]
    actual_sha256: str | None = None
    regular = candidate_path.is_file() and not candidate_path.is_symlink()
    checks.append(
        Check(
            "ue_material_bridge_regular_file",
            regular,
            str(candidate_path),
        )
    )
    checks.append(
        Check(
            "ue_material_bridge_executable",
            regular and os.access(candidate_path, os.X_OK),
            str(candidate_path),
        )
    )
    if regular:
        actual_sha256 = sha256_file(candidate_path)
    checks.append(
        Check(
            "ue_material_bridge_sha256",
            actual_sha256 == bridge["sha256"],
            f"expected={bridge['sha256']} actual={actual_sha256}",
        )
    )

    ue_binary = matrix_root / bridge["ue_binary_relative_path"]
    actual_build_id: str | None = None
    ue_binary_regular = ue_binary.is_file() and not ue_binary.is_symlink()
    checks.append(
        Check(
            "matrix_ue_binary_regular_file",
            ue_binary_regular,
            str(ue_binary),
        )
    )
    if ue_binary_regular:
        try:
            readelf = subprocess.run(
                ("readelf", "-n", str(ue_binary)),
                check=True,
                capture_output=True,
                text=True,
                timeout=15,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            checks.append(Check("matrix_ue_binary_build_id", False, str(exc)))
        else:
            match = re.search(r"Build ID:\s*([0-9a-f]+)", readelf.stdout)
            actual_build_id = match.group(1) if match is not None else None
            checks.append(
                Check(
                    "matrix_ue_binary_build_id",
                    actual_build_id == bridge["ue_binary_build_id"],
                    f"expected={bridge['ue_binary_build_id']} "
                    f"actual={actual_build_id}",
                )
            )
    else:
        checks.append(
            Check(
                "matrix_ue_binary_build_id",
                False,
                "Matrix UE executable missing or symlink",
            )
        )
    evidence = {
        "relative_path": bridge["relative_path"],
        "sha256": actual_sha256,
        "expected_sha256": bridge["sha256"],
        "ue_binary_relative_path": bridge["ue_binary_relative_path"],
        "ue_binary_build_id": actual_build_id,
        "expected_ue_binary_build_id": bridge["ue_binary_build_id"],
    }
    return checks, evidence


def verify_isaac_runtime(
    lock: dict[str, Any], runtime_python: Path
) -> list[Check]:
    """Lock the host Python identity and its editable IsaacLab checkout."""

    checks: list[Check] = []
    if not runtime_python.exists() or not os.access(runtime_python, os.X_OK):
        return [
            Check(
                "isaac_runtime_python",
                False,
                f"missing executable: {runtime_python}",
            )
        ]
    venv_root = runtime_python.parent.parent.resolve()
    pyvenv = venv_root / "pyvenv.cfg"
    include_system_site_packages: bool | None = None
    if pyvenv.is_file() and not pyvenv.is_symlink():
        for raw_line in pyvenv.read_text(encoding="utf-8").splitlines():
            if "=" not in raw_line:
                continue
            key, value = raw_line.split("=", 1)
            if key.strip().lower() == "include-system-site-packages":
                normalized = value.strip().lower()
                if normalized in {"true", "false"}:
                    include_system_site_packages = normalized == "true"
                break
    expected_runtime = lock["isaac_runtime"]
    checks.append(
        Check(
            "isaac_runtime_pyvenv",
            include_system_site_packages
            is expected_runtime["include_system_site_packages"],
            f"root={venv_root} include_system_site_packages="
            f"{include_system_site_packages}",
        )
    )

    probe = r'''
import importlib.metadata
import inspect
import json
import platform
from pathlib import Path
import sys

import isaaclab

module_file = Path(inspect.getfile(isaaclab)).resolve()
checkout_root = None
for candidate in module_file.parents:
    if (candidate / ".git").exists():
        checkout_root = candidate
        break
if checkout_root is None:
    raise SystemExit("IsaacLab import is not inside a Git checkout")
names = json.loads(sys.argv[1])
distributions = {}
for name in names:
    distributions[name] = importlib.metadata.version(name)
print(json.dumps({
    "implementation": platform.python_implementation(),
    "version": platform.python_version(),
    "machine": platform.machine(),
    "prefix": str(Path(sys.prefix).resolve()),
    "base_prefix": str(Path(sys.base_prefix).resolve()),
    "module_file": str(module_file),
    "checkout_root": str(checkout_root.resolve()),
    "distributions": distributions,
}, sort_keys=True))
'''
    environment = os.environ.copy()
    environment.pop("PYTHONHOME", None)
    environment.pop("PYTHONPATH", None)
    environment["PYTHONNOUSERSITE"] = "1"
    environment["PYTHONDONTWRITEBYTECODE"] = "1"
    try:
        result = subprocess.run(
            (
                os.fspath(runtime_python),
                "-I",
                "-c",
                probe,
                json.dumps(sorted(expected_runtime["distributions"])),
            ),
            check=True,
            capture_output=True,
            text=True,
            timeout=30,
            env=environment,
        )
        payload = json.loads(result.stdout)
        checkout_root = Path(payload["checkout_root"])
    except (
        OSError,
        KeyError,
        TypeError,
        subprocess.SubprocessError,
        json.JSONDecodeError,
    ) as exc:
        checks.append(Check("isaac_runtime_probe", False, str(exc)))
        return checks

    identity_ok = (
        payload["implementation"] == expected_runtime["python_implementation"]
        and payload["version"] == expected_runtime["python_version"]
        and payload["machine"] == expected_runtime["platform_machine"]
        and payload["prefix"] == os.fspath(venv_root)
        and payload["base_prefix"] != payload["prefix"]
    )
    checks.extend(
        [
            Check(
                "isaac_runtime_identity",
                identity_ok,
                f"implementation={payload['implementation']} "
                f"version={payload['version']} machine={payload['machine']} "
                f"prefix={payload['prefix']}",
            ),
            Check(
                "isaac_runtime_distributions",
                payload["distributions"] == expected_runtime["distributions"],
                f"actual={payload['distributions']}",
            ),
        ]
    )

    isaaclab = expected_runtime["isaaclab"]
    module_path = checkout_root / isaaclab["module_path"]
    checks.append(
        Check(
            "isaaclab_import_origin",
            payload["module_file"] == os.fspath(module_path),
            f"checkout={checkout_root} module={payload['module_file']}",
        )
    )
    try:
        actual_commit = _run_git(checkout_root, "rev-parse", "HEAD")
        actual_status = _run_git(
            checkout_root,
            "status",
            "--porcelain=v1",
            "--untracked-files=all",
        ).splitlines()
    except (OSError, subprocess.SubprocessError) as exc:
        checks.append(Check("isaaclab_git_checkout", False, str(exc)))
        return checks
    checks.extend(
        [
            Check(
                "isaaclab_commit",
                actual_commit == isaaclab["commit"],
                f"expected={isaaclab['commit']} actual={actual_commit}",
            ),
            Check(
                "isaaclab_checkout_status",
                actual_status == isaaclab["allowed_status"],
                f"actual={actual_status}",
            ),
        ]
    )
    for entry in isaaclab["critical_files"]:
        path = checkout_root / entry["path"]
        if not path.is_file() or path.is_symlink():
            checks.append(
                Check(f"isaaclab_file:{entry['path']}", False, "missing or symlink")
            )
            continue
        actual_hash = sha256_file(path)
        checks.append(
            Check(
                f"isaaclab_file:{entry['path']}",
                actual_hash == entry["sha256"],
                f"expected={entry['sha256']} actual={actual_hash}",
            )
        )
    return checks


def _parse_sha256_manifest(path: Path) -> list[tuple[str, str]]:
    entries: list[tuple[str, str]] = []
    seen: set[str] = set()
    for line_number, raw_line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        match = re.fullmatch(r"([0-9a-f]{64})  ([^\s]+)", raw_line)
        if match is None:
            raise ValueError(f"invalid SHA256 manifest line {line_number}")
        digest, relative = match.groups()
        if not _safe_relative_path(relative) or relative in seen:
            raise ValueError(f"unsafe or duplicate manifest path: {relative!r}")
        seen.add(relative)
        entries.append((relative, digest))
    return entries


def verify_matrix_visual(
    lock: dict[str, Any], matrix_root: Path, visual_root: Path
) -> list[Check]:
    """Verify the exact URDF and every mesh referenced by the Matrix G1."""

    visual = lock["matrix_visual"]
    checks: list[Check] = []
    manifest = matrix_root / visual["manifest"]
    if manifest.is_symlink() or not manifest.is_file():
        return [
            Check(
                "matrix_visual_manifest",
                False,
                f"missing regular file: {manifest}",
            )
        ]
    actual_manifest_hash = sha256_file(manifest)
    checks.append(
        Check(
            "matrix_visual_manifest",
            actual_manifest_hash == visual["manifest_sha256"],
            f"expected={visual['manifest_sha256']} actual={actual_manifest_hash}",
        )
    )
    try:
        entries = dict(_parse_sha256_manifest(manifest))
    except (OSError, UnicodeError, ValueError) as exc:
        checks.append(Check("matrix_visual_manifest_contract", False, str(exc)))
        return checks
    expected_names = set(entries)
    checks.append(
        Check(
            "matrix_visual_manifest_contract",
            len(entries) == visual["file_count"]
            and entries.get(visual["urdf"]) == visual["urdf_sha256"],
            f"files={len(entries)} urdf={entries.get(visual['urdf'])}",
        )
    )
    if visual_root.is_symlink() or not visual_root.is_dir():
        checks.append(
            Check(
                "matrix_visual_root",
                False,
                f"missing regular directory: {visual_root}",
            )
        )
        return checks

    mismatches: list[str] = []
    for relative, expected_hash in entries.items():
        path = visual_root / relative
        if path.is_symlink() or not path.is_file():
            mismatches.append(f"{relative}:missing-or-symlink")
            continue
        actual_hash = sha256_file(path)
        if actual_hash != expected_hash:
            mismatches.append(f"{relative}:hash={actual_hash}")
    checks.append(
        Check(
            "matrix_visual_files",
            not mismatches,
            "verified" if not mismatches else f"mismatches={mismatches}",
        )
    )

    urdf = visual_root / visual["urdf"]
    try:
        document = ET.parse(urdf)
        referenced = {
            filename
            for mesh in document.getroot().findall(".//mesh")
            if (filename := mesh.get("filename")) is not None
        }
    except (OSError, ET.ParseError) as exc:
        checks.append(Check("matrix_visual_urdf_closure", False, str(exc)))
    else:
        safe_references = all(_safe_relative_path(path) for path in referenced)
        expected_references = expected_names - {visual["urdf"]}
        checks.append(
            Check(
                "matrix_visual_urdf_closure",
                safe_references and referenced == expected_references,
                f"referenced={len(referenced)} expected={len(expected_references)}",
            )
        )
    return checks


def _canonical_distribution_name(value: str) -> str:
    return re.sub(r"[-_.]+", "-", value).lower()


def _parse_visual_requirements(path: Path) -> dict[str, str]:
    requirements: dict[str, str] = {}
    for line_number, raw_line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        match = REQUIREMENT_PATTERN.fullmatch(line)
        if match is None:
            raise ValueError(f"invalid locked requirement at line {line_number}")
        raw_name, version = match.groups()
        name = _canonical_distribution_name(raw_name)
        if name in requirements:
            raise ValueError(f"duplicate locked requirement: {name}")
        requirements[name] = version
    if not requirements:
        raise ValueError("visual requirements are empty")
    return requirements


def visual_venv_marker_payload(lock: dict[str, Any]) -> dict[str, object]:
    visual = lock["visual_import"]
    return {
        "schema": VISUAL_VENV_MARKER_SCHEMA,
        "python_implementation": visual["python_implementation"],
        "python_version": visual["python_version"],
        "platform_machine": visual["platform_machine"],
        "requirements_sha256": visual["requirements_sha256"],
        "wheelhouse_manifest_sha256": visual["wheelhouse_manifest_sha256"],
    }


def verify_visual_lock_files(
    lock: dict[str, Any], matrix_root: Path
) -> list[Check]:
    visual = lock["visual_import"]
    checks: list[Check] = []
    paths = {
        "visual_requirements": (
            matrix_root / visual["requirements"],
            visual["requirements_sha256"],
        ),
        "visual_wheelhouse_manifest": (
            matrix_root / visual["wheelhouse_manifest"],
            visual["wheelhouse_manifest_sha256"],
        ),
    }
    for label, (path, expected_hash) in paths.items():
        if not path.is_file() or path.is_symlink():
            checks.append(Check(label, False, f"missing regular file: {path}"))
            continue
        actual_hash = sha256_file(path)
        checks.append(
            Check(
                label,
                actual_hash == expected_hash,
                f"expected={expected_hash} actual={actual_hash}",
            )
        )
    requirements_path = paths["visual_requirements"][0]
    manifest_path = paths["visual_wheelhouse_manifest"][0]
    try:
        requirements = _parse_visual_requirements(requirements_path)
    except (OSError, UnicodeError, ValueError) as exc:
        checks.append(Check("visual_requirements_contract", False, str(exc)))
    else:
        checks.append(
            Check(
                "visual_requirements_contract",
                len(requirements) == visual["wheel_count"],
                f"requirements={len(requirements)} expected={visual['wheel_count']}",
            )
        )
    try:
        manifest_entries = _parse_sha256_manifest(manifest_path)
    except (OSError, UnicodeError, ValueError) as exc:
        checks.append(Check("visual_manifest_contract", False, str(exc)))
    else:
        checks.append(
            Check(
                "visual_manifest_contract",
                len(manifest_entries) == visual["wheel_count"]
                and all(relative.endswith(".whl") for relative, _ in manifest_entries),
                f"wheels={len(manifest_entries)} expected={visual['wheel_count']}",
            )
        )
    return checks


def _wheel_identity(filename: str) -> tuple[str, str, str, str, str]:
    if not filename.endswith(".whl"):
        raise ValueError("not a wheel")
    stem = filename[:-4]
    try:
        prefix, python_tag, abi_tag, platform_tag = stem.rsplit("-", 3)
        distribution, version = prefix.split("-", 1)
    except ValueError as exc:
        raise ValueError("invalid wheel filename") from exc
    return (
        _canonical_distribution_name(distribution),
        version,
        python_tag,
        abi_tag,
        platform_tag,
    )


def _wheel_tags_support_locked_host(
    python_tag: str, abi_tag: str, platform_tag: str
) -> bool:
    python_tags = set(python_tag.split("."))
    python_ok = bool(python_tags & {"cp310", "py310", "py3"})
    abi_ok = abi_tag == "none" or "cp310" in abi_tag.split(".")
    platform_tags = platform_tag.split(".")
    platform_ok = platform_tags == ["any"] or all(
        tag.endswith("_x86_64") for tag in platform_tags
    )
    return python_ok and abi_ok and platform_ok


def verify_visual_wheelhouse(
    lock: dict[str, Any], matrix_root: Path, wheelhouse: Path
) -> list[Check]:
    checks = verify_visual_lock_files(lock, matrix_root)
    visual = lock["visual_import"]
    locked_manifest = matrix_root / visual["wheelhouse_manifest"]
    manifest = wheelhouse / "SHA256SUMS"
    if wheelhouse.is_symlink() or not wheelhouse.is_dir():
        checks.append(
            Check("visual_wheelhouse_root", False, f"missing directory: {wheelhouse}")
        )
        return checks
    if manifest.is_symlink() or not manifest.is_file():
        checks.append(
            Check("visual_wheelhouse_manifest", False, f"missing regular file: {manifest}")
        )
        return checks
    actual_manifest_hash = sha256_file(manifest)
    checks.append(
        Check(
            "visual_wheelhouse_manifest",
            actual_manifest_hash == visual["wheelhouse_manifest_sha256"],
            "expected="
            f"{visual['wheelhouse_manifest_sha256']} actual={actual_manifest_hash}",
        )
    )
    try:
        if manifest.read_bytes() != locked_manifest.read_bytes():
            raise ValueError("wheelhouse manifest differs from the repository lock")
        manifest_entries = dict(_parse_sha256_manifest(manifest))
        requirements = _parse_visual_requirements(
            matrix_root / visual["requirements"]
        )
    except (OSError, UnicodeError, ValueError) as exc:
        checks.append(Check("visual_wheelhouse_contract", False, str(exc)))
        return checks

    actual_files: dict[str, Path] = {}
    non_regular: list[str] = []
    for path in wheelhouse.rglob("*"):
        relative = path.relative_to(wheelhouse).as_posix()
        if path == manifest:
            continue
        if path.is_symlink() or (not path.is_file() and not path.is_dir()):
            non_regular.append(relative)
        elif path.is_file():
            actual_files[relative] = path
    expected_names = set(manifest_entries)
    actual_names = set(actual_files)
    missing = sorted(expected_names - actual_names)
    extra = sorted(actual_names - expected_names)
    exact_inventory = not missing and not extra and not non_regular
    checks.append(
        Check(
            "visual_wheelhouse_inventory",
            exact_inventory,
            (
                "exact"
                if exact_inventory
                else f"missing={missing} extra={extra} non_regular={sorted(non_regular)}"
            ),
        )
    )

    mismatches: list[str] = []
    total_bytes = 0
    wheel_identities: dict[str, str] = {}
    incompatible: list[str] = []
    for relative in sorted(expected_names & actual_names):
        path = actual_files[relative]
        total_bytes += path.stat().st_size
        if sha256_file(path) != manifest_entries[relative]:
            mismatches.append(relative)
        if Path(relative).name != relative:
            incompatible.append(f"{relative}:not-at-wheelhouse-root")
            continue
        try:
            name, version, python_tag, abi_tag, platform_tag = _wheel_identity(
                relative
            )
        except ValueError as exc:
            incompatible.append(f"{relative}:{exc}")
            continue
        if name in wheel_identities:
            incompatible.append(f"{relative}:duplicate-distribution")
        wheel_identities[name] = version
        if not _wheel_tags_support_locked_host(python_tag, abi_tag, platform_tag):
            incompatible.append(f"{relative}:incompatible-tags")
    checks.extend(
        [
            Check(
                "visual_wheelhouse_hashes",
                not mismatches and not missing,
                "verified" if not mismatches else f"mismatches={mismatches}",
            ),
            Check(
                "visual_wheelhouse_size",
                len(actual_files) == visual["wheel_count"]
                and total_bytes == visual["wheel_bytes"],
                f"wheels={len(actual_files)} bytes={total_bytes}",
            ),
            Check(
                "visual_wheelhouse_tags",
                not incompatible and wheel_identities == requirements,
                (
                    "CPython 3.10 x86_64 closure"
                    if not incompatible and wheel_identities == requirements
                    else f"incompatible={incompatible} identities={wheel_identities}"
                ),
            ),
        ]
    )
    return checks


def verify_visual_venv(
    lock: dict[str, Any], matrix_root: Path, venv_root: Path
) -> list[Check]:
    checks = verify_visual_lock_files(lock, matrix_root)
    visual = lock["visual_import"]
    original_root = venv_root
    if original_root.is_symlink() or not original_root.is_dir():
        checks.append(
            Check("visual_venv_root", False, f"missing regular directory: {original_root}")
        )
        return checks
    venv_root = original_root.resolve()
    marker = venv_root / ".matrix-bfm-visual-lock.json"
    pyvenv = venv_root / "pyvenv.cfg"
    python = venv_root / "bin/python"
    if marker.is_symlink() or not marker.is_file():
        checks.append(Check("visual_venv_marker", False, f"missing: {marker}"))
    else:
        try:
            marker_payload = json.loads(marker.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError) as exc:
            checks.append(Check("visual_venv_marker", False, str(exc)))
        else:
            expected_marker = visual_venv_marker_payload(lock)
            checks.append(
                Check(
                    "visual_venv_marker",
                    marker_payload == expected_marker,
                    "locked" if marker_payload == expected_marker else "marker mismatch",
                )
            )
    if pyvenv.is_symlink() or not pyvenv.is_file():
        checks.append(Check("visual_venv_isolation", False, f"missing: {pyvenv}"))
    else:
        values = []
        for raw_line in pyvenv.read_text(encoding="utf-8").splitlines():
            if "=" not in raw_line:
                continue
            key, value = raw_line.split("=", 1)
            if key.strip().lower() == "include-system-site-packages":
                values.append(value.strip().lower())
        checks.append(
            Check(
                "visual_venv_isolation",
                values == ["false"],
                f"include-system-site-packages={values}",
            )
        )
    if not python.exists() or not os.access(python, os.X_OK):
        checks.append(Check("visual_venv_python", False, f"missing: {python}"))
        return checks

    probe = r'''
import importlib
import importlib.metadata
import json
import platform
from pathlib import Path
import sys

modules = {}
for name in json.loads(sys.argv[1]):
    module = importlib.import_module(name)
    modules[name] = str(Path(module.__file__).resolve())
distributions = {}
for distribution in importlib.metadata.distributions():
    raw_name = distribution.metadata.get("Name")
    if raw_name:
        distributions[raw_name] = distribution.version
print(json.dumps({
    "implementation": platform.python_implementation(),
    "version": f"{sys.version_info.major}.{sys.version_info.minor}",
    "machine": platform.machine(),
    "prefix": str(Path(sys.prefix).resolve()),
    "base_prefix": str(Path(sys.base_prefix).resolve()),
    "distributions": distributions,
    "modules": modules,
}, sort_keys=True))
'''
    environment = os.environ.copy()
    environment.pop("PYTHONHOME", None)
    environment.pop("PYTHONPATH", None)
    environment["PYTHONNOUSERSITE"] = "1"
    environment["PYTHONDONTWRITEBYTECODE"] = "1"
    try:
        result = subprocess.run(
            (
                os.fspath(python),
                "-I",
                "-c",
                probe,
                json.dumps(visual["required_imports"]),
            ),
            check=True,
            capture_output=True,
            text=True,
            timeout=30,
            env=environment,
        )
        payload = json.loads(result.stdout)
    except (OSError, subprocess.SubprocessError, json.JSONDecodeError) as exc:
        checks.append(Check("visual_venv_probe", False, str(exc)))
        return checks

    expected_requirements = _parse_visual_requirements(
        matrix_root / visual["requirements"]
    )
    actual_distributions = {
        _canonical_distribution_name(name): version
        for name, version in payload["distributions"].items()
    }
    allowed_base = {"pip", "setuptools"}
    unexpected = sorted(set(actual_distributions) - set(expected_requirements) - allowed_base)
    missing_or_wrong = sorted(
        name
        for name, version in expected_requirements.items()
        if actual_distributions.get(name) != version
    )
    identity_ok = (
        payload["implementation"] == visual["python_implementation"]
        and payload["version"] == visual["python_version"]
        and payload["machine"] == visual["platform_machine"]
        and payload["prefix"] == os.fspath(venv_root)
        and payload["base_prefix"] != payload["prefix"]
    )
    module_paths_ok = all(
        Path(module_path).is_relative_to(venv_root)
        for module_path in payload["modules"].values()
    )
    checks.extend(
        [
            Check(
                "visual_venv_identity",
                identity_ok,
                f"implementation={payload['implementation']} version={payload['version']} "
                f"machine={payload['machine']} prefix={payload['prefix']}",
            ),
            Check(
                "visual_venv_distributions",
                not unexpected and not missing_or_wrong,
                f"unexpected={unexpected} missing_or_wrong={missing_or_wrong}",
            ),
            Check(
                "visual_venv_imports",
                module_paths_ok
                and set(payload["modules"]) == set(visual["required_imports"]),
                f"modules={payload['modules']}",
            ),
        ]
    )
    return checks


def verify_physics_assets(
    lock: dict[str, Any], matrix_root: Path, asset_root: Path
) -> list[Check]:
    checks: list[Check] = []
    metadata = lock["physics_assets"]
    manifest = matrix_root / metadata["manifest"]
    if not manifest.is_file() or manifest.is_symlink():
        return [Check("physics_asset_manifest", False, f"missing or symlink: {manifest}")]
    actual_manifest_hash = sha256_file(manifest)
    checks.append(
        Check(
            "physics_asset_manifest",
            actual_manifest_hash == metadata["manifest_sha256"],
            f"expected={metadata['manifest_sha256']} actual={actual_manifest_hash}",
        )
    )
    try:
        entries = _parse_sha256_manifest(manifest)
    except (OSError, ValueError) as exc:
        checks.append(Check("physics_asset_manifest_entries", False, str(exc)))
        return checks
    checks.append(
        Check(
            "physics_asset_manifest_entries",
            len(entries) == metadata["file_count"],
            f"expected={metadata['file_count']} actual={len(entries)}",
        )
    )
    for relative, expected in entries:
        path = asset_root / relative
        if not path.is_file() or path.is_symlink():
            checks.append(Check(f"physics_asset:{relative}", False, "missing or symlink"))
            continue
        actual = sha256_file(path)
        checks.append(
            Check(
                f"physics_asset:{relative}",
                actual == expected,
                f"expected={expected} actual={actual}",
            )
        )
    return checks


def verify_scene_assets(
    lock: dict[str, Any], matrix_root: Path, collision_root: Path
) -> list[Check]:
    checks: list[Check] = []
    assets = lock["scene_assets"]
    scene = lock["scene_collision_contract"]
    visual_bin = matrix_root / assets["visual_bin"]
    if visual_bin.is_symlink() or not visual_bin.is_file():
        checks.append(
            Check("moon_visual_bin", False, f"missing regular file: {visual_bin}")
        )
    else:
        actual_hash = sha256_file(visual_bin)
        actual_bytes = visual_bin.stat().st_size
        checks.append(
            Check(
                "moon_visual_bin",
                actual_hash == assets["visual_bin_sha256"]
                and actual_bytes == assets["visual_bin_bytes"],
                f"sha256={actual_hash} bytes={actual_bytes}",
            )
        )

    if collision_root.is_symlink() or not collision_root.is_dir():
        checks.append(
            Check(
                "moon_collision_root",
                False,
                f"missing regular directory: {collision_root}",
            )
        )
        return checks
    manifest_path = collision_root / assets["collision_manifest"]
    collision_path = collision_root / assets["collision_usd"]
    payload: dict[str, Any] | None = None
    for label, path, expected_hash in (
        (
            "moon_collision_manifest",
            manifest_path,
            assets["collision_manifest_sha256"],
        ),
        ("moon_collision_usd", collision_path, assets["collision_usd_sha256"]),
    ):
        if path.is_symlink() or not path.is_file():
            checks.append(Check(label, False, f"missing regular file: {path}"))
            continue
        actual_hash = sha256_file(path)
        checks.append(
            Check(
                label,
                actual_hash == expected_hash,
                f"expected={expected_hash} actual={actual_hash}",
            )
        )
        if path == manifest_path:
            try:
                loaded = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, UnicodeError, json.JSONDecodeError) as exc:
                checks.append(Check("moon_collision_manifest_json", False, str(exc)))
            else:
                if isinstance(loaded, dict):
                    payload = loaded
                else:
                    checks.append(
                        Check(
                            "moon_collision_manifest_json",
                            False,
                            "manifest is not an object",
                        )
                    )
    if payload is None:
        return checks

    expected_manifest = {
        "schema_version": 2,
        "source_sha256": assets["visual_bin_sha256"],
        "source_size": 6000,
        "source_resolution_m": 0.1,
        "sample_stride": 4,
        "collision_resolution_m": 0.4,
        "patch_size_m": 240.0,
        "center_x_m": 23.0,
        "center_y_m": 13.0,
        "patch_side": 601,
        "vertex_count": 361201,
        "quad_count": 360000,
        "x_min_m": scene["x_min_m"],
        "x_max_m": scene["x_max_m"],
        "y_min_m": scene["y_min_m"],
        "y_max_m": scene["y_max_m"],
        "ground_z_m": -2.0390634536743164,
        "collision_sha256": assets["collision_usd_sha256"],
    }
    mismatches = {
        key: {"expected": expected, "actual": payload.get(key)}
        for key, expected in expected_manifest.items()
        if payload.get(key) != expected
    }
    source_name = Path(str(payload.get("source", ""))).name
    collision_name = Path(str(payload.get("collision", ""))).name
    if source_name != Path(assets["visual_bin"]).name:
        mismatches["source"] = {
            "expected": Path(assets["visual_bin"]).name,
            "actual": source_name,
        }
    if collision_name != assets["collision_usd"]:
        mismatches["collision"] = {
            "expected": assets["collision_usd"],
            "actual": collision_name,
        }
    checks.append(
        Check(
            "moon_collision_manifest_contract",
            not mismatches,
            "locked" if not mismatches else json.dumps(mismatches, sort_keys=True),
        )
    )
    return checks


def verify_teacher_profile(lock: dict[str, Any], path: Path) -> list[Check]:
    checks: list[Check] = []
    try:
        text = path.read_text(encoding="utf-8")
        raw = _load_profile_toml(text)
    except (OSError, ValueError) as exc:
        return [Check("teacher_profile", False, str(exc))]
    expected_profile = lock["policy"]["profile_id"]
    actual_profile = raw.get("profile_id")
    checks.append(
        Check(
            "teacher_profile_id",
            actual_profile == expected_profile,
            f"expected={expected_profile} actual={actual_profile}",
        )
    )
    paths = raw.get("paths")
    hashes = raw.get("hashes")
    if not isinstance(paths, dict) or not isinstance(hashes, dict):
        checks.append(Check("teacher_profile_contract", False, "paths/hashes tables missing"))
        return checks
    expected_by_name = {
        entry["name"]: entry["sha256"] for entry in lock["policy"]["artifacts"]
    }
    field_names = {
        "teacher_checkpoint": "model_step_079000.pt",
        "teacher_config": "config.yaml",
        "teacher_onnx": "model_step_079000_g1.onnx",
    }
    for field, artifact_name in field_names.items():
        raw_path = paths.get(field)
        expected_hash = expected_by_name[artifact_name]
        profile_hash = hashes.get(f"{field}_sha256")
        if not isinstance(raw_path, str) or not Path(raw_path).is_absolute():
            checks.append(Check(f"teacher_artifact:{field}", False, "path is not absolute"))
            continue
        artifact = Path(raw_path)
        checks.append(
            Check(
                f"teacher_profile_hash:{field}",
                profile_hash == expected_hash,
                f"expected={expected_hash} profile={profile_hash}",
            )
        )
        if not artifact.is_file() or artifact.is_symlink():
            checks.append(Check(f"teacher_artifact:{field}", False, f"missing: {artifact}"))
            continue
        actual_hash = sha256_file(artifact)
        checks.append(
            Check(
                f"teacher_artifact:{field}",
                actual_hash == expected_hash,
                f"expected={expected_hash} actual={actual_hash}",
            )
        )
    return checks


def _load_profile_toml(text: str) -> dict[str, Any]:
    """Load the frozen scalar-only profile on Python 3.10 or newer."""

    if tomllib is not None:
        try:
            return tomllib.loads(text)
        except tomllib.TOMLDecodeError as exc:
            raise ValueError(str(exc)) from exc

    root: dict[str, Any] = {}
    table = root
    for line_number, raw_line in enumerate(text.splitlines(), start=1):
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        if line.startswith("[") and line.endswith("]"):
            name = line[1:-1].strip()
            if not name or "." in name or name in root:
                raise ValueError(f"unsupported TOML table at line {line_number}")
            table = {}
            root[name] = table
            continue
        if "=" not in line:
            raise ValueError(f"invalid TOML assignment at line {line_number}")
        key, encoded = (part.strip() for part in line.split("=", 1))
        if not key or key in table:
            raise ValueError(f"invalid or duplicate TOML key at line {line_number}")
        if encoded.startswith('"'):
            try:
                value = json.loads(encoded)
            except json.JSONDecodeError as exc:
                raise ValueError(f"invalid TOML string at line {line_number}") from exc
            if not isinstance(value, str):
                raise ValueError(f"unsupported TOML value at line {line_number}")
        else:
            try:
                value = int(encoded)
            except ValueError as exc:
                raise ValueError(
                    f"unsupported TOML value at line {line_number}"
                ) from exc
        table[key] = value
    return root


def _eq_number(actual: object, expected: float, *, tolerance: float = 1.0e-9) -> bool:
    return _finite_number(actual) and math.isclose(
        float(actual), expected, rel_tol=0.0, abs_tol=tolerance
    )


def _check_max(name: str, actual: object, maximum: float, gate: str) -> Check:
    ok = _finite_number(actual) and float(actual) <= maximum
    return Check(name, ok, f"actual={actual} max={maximum}", gate)


def _check_min(name: str, actual: object, minimum: float, gate: str) -> Check:
    ok = _finite_number(actual) and float(actual) >= minimum
    return Check(name, ok, f"actual={actual} min={minimum}", gate)


def verify_evidence(
    lock: dict[str, Any], report: dict[str, Any], relay: dict[str, Any]
) -> tuple[list[Check], dict[str, float | int | None]]:
    clock = lock["clock_contract"]
    execution = lock["execution_contract"]
    correctness = lock["acceptance"]["correctness"]
    realtime = lock["acceptance"]["realtime"]
    checks: list[Check] = []
    checks.extend(
        [
            Check("runtime_report_ok", report.get("ok") is True, f"value={report.get('ok')}", "correctness"),
            Check("runtime_failure_empty", report.get("failure") is None, f"value={report.get('failure')}", "correctness"),
            Check(
                "physics_dt",
                _eq_number(report.get("physics_dt"), float(clock["physics_dt_s"])),
                f"actual={report.get('physics_dt')} expected={clock['physics_dt_s']}",
                "correctness",
            ),
            Check(
                "control_hz_sim",
                _eq_number(report.get("control_hz"), float(clock["control_hz_sim"])),
                f"actual={report.get('control_hz')} expected={clock['control_hz_sim']}",
                "correctness",
            ),
            Check(
                "physics_device",
                report.get("physics_device") == execution["physics_device"],
                f"actual={report.get('physics_device')} "
                f"expected={execution['physics_device']}",
                "correctness",
            ),
            Check(
                "reference_device",
                report.get("reference_device") == execution["reference_device"],
                f"actual={report.get('reference_device')} "
                f"expected={execution['reference_device']}",
                "correctness",
            ),
            Check(
                "physics_command_write_mode",
                report.get("physics_command_write_mode")
                == execution["physics_command_write_mode"],
                f"actual={report.get('physics_command_write_mode')} "
                f"expected={execution['physics_command_write_mode']}",
                "correctness",
            ),
            Check(
                "physics_command_writes",
                report.get(
                    "physics_command_writes_per_articulation_per_control_step"
                )
                == execution[
                    "physics_command_writes_per_articulation_per_control_step"
                ],
                "actual="
                f"{report.get('physics_command_writes_per_articulation_per_control_step')} "
                "expected="
                f"{execution['physics_command_writes_per_articulation_per_control_step']}",
                "correctness",
            ),
            Check(
                "articulation_updates",
                report.get(
                    "articulation_updates_per_articulation_per_control_step"
                )
                == execution[
                    "articulation_updates_per_articulation_per_control_step"
                ],
                "actual="
                f"{report.get('articulation_updates_per_articulation_per_control_step')} "
                "expected="
                f"{execution['articulation_updates_per_articulation_per_control_step']}",
                "correctness",
            ),
            Check(
                "articulation_update_dt",
                _eq_number(
                    report.get("articulation_update_dt_s"),
                    float(execution["articulation_update_dt_s"]),
                ),
                f"actual={report.get('articulation_update_dt_s')} "
                f"expected={execution['articulation_update_dt_s']}",
                "correctness",
            ),
            Check(
                "teacher_onnx_session",
                report.get("teacher_onnx_session")
                == execution["teacher_onnx_session"],
                f"actual={report.get('teacher_onnx_session')}",
                "correctness",
            ),
        ]
    )
    completed = report.get("completed_control_steps")
    requested = report.get("requested_control_steps")
    completed_valid = isinstance(completed, int) and not isinstance(completed, bool) and completed > 0
    checks.append(Check("completed_control_steps", completed_valid, f"value={completed}", "correctness"))
    checks.append(
        Check(
            "bounded_run_completed",
            completed_valid and requested == completed,
            f"requested={requested} completed={completed}",
            "correctness",
        )
    )
    checks.extend(
        [
            _check_max("fall_count", report.get("fall_count"), correctness["fall_count_max"], "correctness"),
            _check_max("recovery_count", report.get("recovery_count"), correctness["recovery_count_max"], "correctness"),
            Check(
                "not_awaiting_recovery",
                report.get("awaiting_recovery_final") is False,
                f"value={report.get('awaiting_recovery_final')}",
                "correctness",
            ),
            _check_max(
                "matrix_state_frames_dropped",
                report.get("matrix_state_frames_dropped"),
                correctness["matrix_state_frames_dropped_max"],
                "correctness",
            ),
        ]
    )
    frames_sent = report.get("matrix_state_frames_sent")
    checks.append(
        Check(
            "one_state_per_control_tick",
            completed_valid and frames_sent == completed,
            f"sent={frames_sent} completed={completed}",
            "correctness",
        )
    )
    raw_schedule = report.get("schedule")
    schedule_modes: set[str] = set()
    if isinstance(raw_schedule, list):
        for item in raw_schedule:
            if (
                isinstance(item, list)
                and len(item) == 2
                and isinstance(item[0], str)
                and _positive_number(item[1])
            ):
                schedule_modes.add(item[0])
    required_schedule_modes = set(correctness["required_schedule_modes"])
    schedule_mode = report.get("mode")
    schedule_gate = "correctness" if schedule_mode == "schedule" else "manual"
    schedule_coverage_ok = (
        schedule_mode == "schedule"
        and required_schedule_modes.issubset(schedule_modes)
    )
    checks.append(
        Check(
            "command_schedule_coverage",
            schedule_coverage_ok,
            f"mode={schedule_mode} observed={sorted(schedule_modes)} "
            f"required={sorted(required_schedule_modes)}",
            schedule_gate,
        )
    )
    observed_gaits = report.get("observed_gaits")
    observed_gait_set = (
        set(observed_gaits)
        if isinstance(observed_gaits, list)
        and all(isinstance(item, str) for item in observed_gaits)
        else set()
    )
    required_gaits = set(correctness["required_observed_gaits"])
    checks.append(
        Check(
            "observed_gait_coverage",
            required_gaits.issubset(observed_gait_set),
            f"observed={sorted(observed_gait_set)} required={sorted(required_gaits)}",
            "correctness",
        )
    )
    checks.extend(
        [
            _check_min(
                "height_raycast_hits_min",
                report.get("height_raycast_hits_min"),
                correctness["height_raycast_hits_min"],
                "correctness",
            ),
            Check(
                "height_query_path",
                isinstance(report.get("height_query_paths_last"), list)
                and correctness["height_query_path"]
                in report["height_query_paths_last"],
                f"value={report.get('height_query_paths_last')}",
                "correctness",
            ),
            _check_min(
                "root_clearance_min",
                report.get("root_clearance_min"),
                correctness["root_clearance_min_m"],
                "correctness",
            ),
            _check_max(
                "root_clearance_max",
                report.get("root_clearance_max"),
                correctness["root_clearance_max_m"],
                "correctness",
            ),
            Check(
                "reference_source",
                report.get("reference_source") == correctness["reference_source"],
                f"value={report.get('reference_source')}",
                "correctness",
            ),
            Check(
                "reference_source_hz",
                _eq_number(
                    report.get("reference_source_hz"),
                    correctness["reference_source_hz"],
                ),
                f"value={report.get('reference_source_hz')}",
                "correctness",
            ),
            Check(
                "reference_output_hz",
                _eq_number(
                    report.get("reference_output_hz"),
                    correctness["reference_output_hz"],
                ),
                f"value={report.get('reference_output_hz')}",
                "correctness",
            ),
            _check_min(
                "reference_buffer_swap_count",
                report.get("reference_buffer_swap_count"),
                correctness["reference_buffer_swap_count_min"],
                "correctness",
            ),
            _check_max(
                "reference_pending_elapsed_steps",
                report.get("reference_pending_elapsed_steps_max"),
                correctness["reference_pending_elapsed_steps_max"],
                "correctness",
            ),
            _check_max(
                "reference_root_xy_error_p95",
                report.get("reference_root_xy_error_p95_m"),
                correctness["reference_root_xy_error_p95_m_max"],
                "correctness",
            ),
            _check_max(
                "reference_root_yaw_error_p95",
                report.get("reference_root_yaw_error_p95_rad"),
                correctness["reference_root_yaw_error_p95_rad_max"],
                "correctness",
            ),
            _check_max(
                "reference_root_tilt_error_p95",
                report.get("reference_root_tilt_error_p95_rad"),
                correctness["reference_root_tilt_error_p95_rad_max"],
                "correctness",
            ),
            _check_max(
                "reference_joint_tracking_rmse",
                report.get("reference_joint_tracking_rmse_rad"),
                correctness["reference_joint_tracking_rmse_rad_max"],
                "correctness",
            ),
        ]
    )
    expected_physics_steps = (
        int(completed) * int(clock["decimation"]) if completed_valid else None
    )
    checks.append(
        Check(
            "locked_four_substeps_per_action",
            clock["decimation"] == clock["action_hold_substeps"] == 4,
            f"derived_physics_steps={expected_physics_steps} "
            f"control_steps={completed} decimation=4 "
            "evidence=locked_runtime_source_and_clock_contract",
            "correctness",
        )
    )

    checks.append(
        Check(
            "relay_schema",
            relay.get("schema") == RELAY_STATUS_SCHEMA,
            f"value={relay.get('schema')}",
            "correctness",
        )
    )
    checks.append(
        Check(
            "relay_status_ok",
            relay.get("ok") is True,
            f"value={relay.get('ok')}",
            "correctness",
        )
    )
    input_contract = relay.get("input_contract")
    output_contract = relay.get("output_contract")
    checks.append(
        Check(
            "relay_input_contract",
            input_contract == lock["wire_contract"]["input"],
            f"actual={input_contract}",
            "correctness",
        )
    )
    checks.append(
        Check(
            "relay_output_contract",
            output_contract == lock["wire_contract"]["matrix_output"],
            f"actual={output_contract}",
            "correctness",
        )
    )
    stats = relay.get("stats")
    if not isinstance(stats, dict):
        stats = {}
        checks.append(Check("relay_stats", False, "missing stats object", "correctness"))
    checks.extend(
        [
            Check(
                "relay_received_all_states",
                completed_valid and stats.get("received") == frames_sent == completed,
                f"received={stats.get('received')} sent={frames_sent} completed={completed}",
                "correctness",
            ),
            _check_max("relay_invalid", stats.get("invalid"), correctness["relay_invalid_frames_max"], "correctness"),
            _check_max("relay_sequence_gaps", stats.get("sequence_gaps"), correctness["relay_sequence_gaps_max"], "correctness"),
            _check_max("relay_duplicates", stats.get("duplicates"), correctness["relay_duplicate_frames_max"], "correctness"),
            _check_max("relay_out_of_order", stats.get("out_of_order"), correctness["relay_out_of_order_frames_max"], "correctness"),
            _check_max("relay_non_grid_time", stats.get("non_grid_time"), 0, "correctness"),
            Check(
                "relay_first_sequence_zero",
                stats.get("first_sequence") == 0,
                f"value={stats.get('first_sequence')}",
                "correctness",
            ),
            Check(
                "relay_last_sequence",
                completed_valid and stats.get("last_sequence") == int(completed) - 1,
                f"actual={stats.get('last_sequence')} expected={int(completed) - 1 if completed_valid else None}",
                "correctness",
            ),
        ]
    )
    boundary = relay.get("boundary_guard")
    if not isinstance(boundary, dict):
        boundary = {}
        checks.append(
            Check("boundary_guard", False, "missing boundary guard", "correctness")
        )
    checks.extend(
        [
            Check(
                "boundary_guard_armed",
                boundary.get("armed") is True,
                f"value={boundary.get('armed')}",
                "correctness",
            ),
            _check_max(
                "boundary_stop_events",
                boundary.get("stop_events"),
                correctness["boundary_stop_events_max"],
                "correctness",
            ),
            _check_max(
                "boundary_command_errors",
                boundary.get("command_errors"),
                correctness["boundary_command_errors_max"],
                "correctness",
            ),
            _check_max(
                "boundary_hard_violations",
                boundary.get("hard_violations"),
                correctness["boundary_hard_violations_max"],
                "correctness",
            ),
        ]
    )

    wall_s = report.get("control_loop_wall_s")
    if completed_valid and _positive_number(wall_s):
        control_hz_wall = float(completed) / float(wall_s)
        physics_hz_wall = float(expected_physics_steps) / float(wall_s)
        simulation_realtime_factor = (
            float(completed) / float(clock["control_hz_sim"]) / float(wall_s)
        )
    else:
        control_hz_wall = None
        physics_hz_wall = None
        simulation_realtime_factor = None
    metrics: dict[str, float | int | None] = {
        "completed_control_steps": int(completed) if completed_valid else None,
        "derived_physics_steps": expected_physics_steps,
        "control_loop_wall_s": float(wall_s) if _positive_number(wall_s) else None,
        "control_hz_wall": control_hz_wall,
        "physics_hz_wall": physics_hz_wall,
        "simulation_realtime_factor": simulation_realtime_factor,
        "control_step_wall_ms_p95": (
            float(report["control_step_wall_ms_p95"])
            if _finite_number(report.get("control_step_wall_ms_p95"))
            else None
        ),
        "boundary_warning_events": (
            int(boundary["warning_events"])
            if _nonnegative_integer(boundary.get("warning_events"))
            else None
        ),
        "boundary_stop_events": (
            int(boundary["stop_events"])
            if _nonnegative_integer(boundary.get("stop_events"))
            else None
        ),
    }
    checks.extend(
        [
            _check_min("physics_hz_wall", physics_hz_wall, realtime["physics_hz_wall_min"], "realtime"),
            _check_min("control_hz_wall", control_hz_wall, realtime["control_hz_wall_min"], "realtime"),
            _check_min(
                "simulation_realtime_factor",
                simulation_realtime_factor,
                realtime["simulation_realtime_factor_min"],
                "realtime",
            ),
            _check_max(
                "control_step_wall_ms_p95",
                report.get("control_step_wall_ms_p95"),
                realtime["control_step_wall_ms_p95_max"],
                "realtime",
            ),
        ]
    )
    reported_rtf = report.get("simulation_realtime_factor")
    checks.append(
        Check(
            "reported_rtf_matches_derived",
            _finite_number(reported_rtf)
            and simulation_realtime_factor is not None
            and math.isclose(
                float(reported_rtf), simulation_realtime_factor, rel_tol=1.0e-9, abs_tol=1.0e-9
            ),
            f"reported={reported_rtf} derived={simulation_realtime_factor}",
            "realtime",
        )
    )
    return checks, metrics


def _load_json_object(path: Path, label: str) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"{label} must be a JSON object")
    return payload


def load_resolved_video_settings(path: Path) -> dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"resolved video settings must be a regular file: {path}")
    if path.stat().st_size > 16 * 1024:
        raise ValueError("resolved video settings exceed the size limit")

    def strict_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
        value: dict[str, object] = {}
        for key, item in pairs:
            if key in value:
                raise ValueError(f"duplicate resolved video field: {key}")
            value[key] = item
        return value

    payload = json.loads(
        path.read_text(encoding="utf-8"), object_pairs_hook=strict_object
    )
    if not isinstance(payload, dict):
        raise ValueError("resolved video settings must be a JSON object")
    expected_keys = {
        "schema",
        "resolution",
        "resolution_width",
        "resolution_height",
        "window_mode",
        "fps_limit",
        "quality",
        "camera_smoothing",
        "screen_percentage",
    }
    _require_exact_keys(payload, expected_keys, "resolved_video_settings")
    if payload["schema"] != VIDEO_SETTINGS_SCHEMA:
        raise ValueError("unexpected resolved video settings schema")
    resolutions = {
        "1280x720": (1280, 720),
        "1600x900": (1600, 900),
        "1920x1080": (1920, 1080),
        "2560x1440": (2560, 1440),
    }
    resolution = payload["resolution"]
    if not isinstance(resolution, str) or resolution not in resolutions:
        raise ValueError("resolved video resolution is not allowed")
    width = payload["resolution_width"]
    height = payload["resolution_height"]
    if (
        not isinstance(width, int)
        or isinstance(width, bool)
        or not isinstance(height, int)
        or isinstance(height, bool)
        or (width, height) != resolutions[resolution]
    ):
        raise ValueError("resolved video dimensions disagree with resolution")
    fps_limit = payload["fps_limit"]
    screen_percentage = payload["screen_percentage"]
    if (
        not isinstance(fps_limit, int)
        or isinstance(fps_limit, bool)
        or fps_limit not in {30, 60, 90, 120}
    ):
        raise ValueError("resolved video FPS is not allowed")
    if (
        not isinstance(screen_percentage, int)
        or isinstance(screen_percentage, bool)
        or not 25 <= screen_percentage <= 200
    ):
        raise ValueError("resolved video screen percentage is not allowed")
    if payload["window_mode"] not in {"windowed", "borderless", "fullscreen"}:
        raise ValueError("resolved video window mode is not allowed")
    if payload["quality"] not in {"low", "medium", "high", "epic"}:
        raise ValueError("resolved video quality is not allowed")
    if payload["camera_smoothing"] not in {"off", "low", "medium", "high"}:
        raise ValueError("resolved video camera smoothing is not allowed")
    return payload


def _print_checks(checks: Iterable[Check]) -> None:
    for check in checks:
        print(
            f"[{'PASS' if check.ok else 'FAIL'}] [{check.gate}] "
            f"{check.name}: {check.detail}"
        )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lock", type=Path, default=DEFAULT_LOCK)
    parser.add_argument("--schema-only", action="store_true")
    parser.add_argument("--matrix-root", type=Path)
    parser.add_argument("--runtime-root", type=Path)
    parser.add_argument("--runtime-python", type=Path)
    parser.add_argument("--physics-asset-root", type=Path)
    parser.add_argument("--collision-root", type=Path)
    parser.add_argument("--teacher-profile", type=Path)
    parser.add_argument("--visual-wheelhouse", type=Path)
    parser.add_argument("--visual-venv", type=Path)
    parser.add_argument("--matrix-visual-root", type=Path)
    parser.add_argument("--material-bridge", type=Path)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--relay-status", type=Path)
    parser.add_argument("--video-settings", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--correctness-only",
        action="store_true",
        help="diagnostic mode: report but do not fail on the separate real-time gate",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    evidence_paths = (args.report, args.relay_status, args.video_settings)
    evidence_requested = any(path is not None for path in evidence_paths)
    if evidence_requested and not all(
        path is not None for path in evidence_paths
    ):
        parser.error(
            "--report, --relay-status, and --video-settings must be supplied together"
        )
    if evidence_requested:
        required_runtime_closure = {
            "--matrix-root": args.matrix_root,
            "--runtime-root": args.runtime_root,
            "--runtime-python": args.runtime_python,
            "--physics-asset-root": args.physics_asset_root,
            "--collision-root": args.collision_root,
            "--teacher-profile": args.teacher_profile,
            "--visual-venv": args.visual_venv,
            "--matrix-visual-root": args.matrix_visual_root,
            "--material-bridge": args.material_bridge,
        }
        missing = [
            option
            for option, value in required_runtime_closure.items()
            if value is None
        ]
        if missing:
            parser.error(
                "acceptance evidence requires the complete runtime closure; "
                f"missing: {', '.join(missing)}"
            )
    elif args.output is not None or args.correctness_only:
        parser.error(
            "--output and --correctness-only require complete acceptance evidence"
        )
    if args.material_bridge is not None and args.matrix_root is None:
        parser.error("--material-bridge requires --matrix-root")
    try:
        lock = load_lock(args.lock)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"[FAIL] [schema] runtime_lock: {exc}", file=sys.stderr)
        return 2
    print(f"[PASS] [schema] runtime_lock: {args.lock}")
    if args.schema_only:
        if any(
            value is not None
            for value in (
                args.runtime_root,
                args.matrix_root,
                args.runtime_python,
                args.physics_asset_root,
                args.collision_root,
                args.teacher_profile,
                args.visual_wheelhouse,
                args.visual_venv,
                args.matrix_visual_root,
                args.material_bridge,
                args.report,
                args.relay_status,
                args.video_settings,
                args.output,
            )
        ) or args.correctness_only:
            parser.error("--schema-only cannot be combined with runtime/evidence options")
        return 0

    checks: list[Check] = []
    matrix_commit: str | None = None
    material_bridge_evidence: dict[str, object] | None = None
    resolved_video_settings: dict[str, Any] | None = None
    if args.matrix_root is not None:
        matrix_checks, matrix_commit = verify_matrix_port(
            lock, args.matrix_root.resolve()
        )
        checks.extend(matrix_checks)
    if args.runtime_root is not None:
        checks.extend(verify_runtime_checkout(lock, args.runtime_root.resolve()))
    if args.runtime_python is not None:
        checks.extend(verify_isaac_runtime(lock, args.runtime_python))
    if args.physics_asset_root is not None:
        checks.extend(
            verify_physics_assets(lock, REPO_ROOT, args.physics_asset_root.resolve())
        )
    if args.collision_root is not None:
        checks.extend(
            verify_scene_assets(
                lock,
                REPO_ROOT,
                args.collision_root,
            )
        )
    if args.teacher_profile is not None:
        checks.extend(verify_teacher_profile(lock, args.teacher_profile.resolve()))
    if args.visual_wheelhouse is not None:
        checks.extend(
            verify_visual_wheelhouse(
                lock,
                REPO_ROOT,
                args.visual_wheelhouse,
            )
        )
    if args.visual_venv is not None:
        checks.extend(
            verify_visual_venv(
                lock,
                REPO_ROOT,
                args.visual_venv,
            )
        )
    if args.matrix_visual_root is not None:
        checks.extend(
            verify_matrix_visual(
                lock,
                REPO_ROOT,
                args.matrix_visual_root.resolve(),
            )
        )
    if args.material_bridge is not None:
        bridge_checks, material_bridge_evidence = verify_ue_material_bridge(
            lock,
            args.matrix_root.resolve(),
            args.material_bridge,
        )
        checks.extend(bridge_checks)

    metrics: dict[str, float | int | None] = {}
    if (
        args.report is not None
        and args.relay_status is not None
        and args.video_settings is not None
    ):
        try:
            report = _load_json_object(args.report, "runtime report")
            relay = _load_json_object(args.relay_status, "relay status")
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            checks.append(Check("evidence_files", False, str(exc), "correctness"))
        else:
            evidence_checks, metrics = verify_evidence(lock, report, relay)
            checks.extend(evidence_checks)
        try:
            resolved_video_settings = load_resolved_video_settings(
                args.video_settings
            )
        except (OSError, UnicodeError, ValueError, json.JSONDecodeError) as exc:
            checks.append(
                Check(
                    "resolved_video_settings",
                    False,
                    str(exc),
                    "correctness",
                )
            )
        else:
            checks.append(
                Check(
                    "resolved_video_settings",
                    True,
                    json.dumps(resolved_video_settings, sort_keys=True),
                    "correctness",
                )
            )

    if not checks:
        print("[INFO] No runtime checkout or evidence was requested")
        return 0
    _print_checks(checks)
    runtime_ok = all(check.ok for check in checks if check.gate == "runtime")
    correctness_ok = all(check.ok for check in checks if check.gate == "correctness")
    realtime_ok = all(check.ok for check in checks if check.gate == "realtime")
    manual_checks = [check for check in checks if check.gate == "manual"]
    manual_ok = all(check.ok for check in manual_checks)
    overall_ok = (
        runtime_ok
        and correctness_ok
        and (realtime_ok or args.correctness_only)
        and manual_ok
    )
    result: dict[str, object] = {
        "schema": (
            ACCEPTANCE_SCHEMA
            if evidence_requested
            else RUNTIME_VERIFICATION_SCHEMA
        ),
        "runtime_id": lock["runtime_id"],
        "matrix_commit": matrix_commit,
        "runtime_lock_sha256": sha256_file(args.lock),
        "ue_material_bridge": material_bridge_evidence,
        "resolved_video_settings": resolved_video_settings,
        "runtime_ok": runtime_ok,
        "correctness_ok": correctness_ok,
        "realtime_ok": realtime_ok,
        "manual_ok": manual_ok,
        "manual_review_required": bool(manual_checks),
        "correctness_only": args.correctness_only,
        "overall_ok": overall_ok,
        "metrics": metrics,
        "checks": [asdict(check) for check in checks],
    }
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        temporary = args.output.with_name(f".{args.output.name}.tmp.{os.getpid()}")
        temporary.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        os.replace(temporary, args.output)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if overall_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
