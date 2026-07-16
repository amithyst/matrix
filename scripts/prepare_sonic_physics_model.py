#!/usr/bin/env python3
"""Prepare SONIC's canonical 29-DOF G1 physics model for a Matrix map."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
from compose_custom_scene import compose_custom_scene  # noqa: E402


PIPELINE_VERSION = 1
G1_BODY_JOINT_NAMES = (
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


class SonicPhysicsModelError(RuntimeError):
    """Raised when the canonical SONIC model contract is not satisfied."""


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _tree_sha256(root: Path) -> str:
    digest = hashlib.sha256()
    for path in sorted(item for item in root.rglob("*") if item.is_file()):
        digest.update(path.relative_to(root).as_posix().encode("utf-8"))
        digest.update(b"\0")
        digest.update(_file_sha256(path).encode("ascii"))
        digest.update(b"\n")
    return digest.hexdigest()


def _source_contract(
    canonical_model: Path,
    canonical_meshes: Path,
    native_scene: Path,
    *,
    body_joint_names: tuple[str, ...],
) -> dict[str, object]:
    return {
        "pipeline_version": PIPELINE_VERSION,
        "canonical_model": str(canonical_model.resolve()),
        "canonical_model_sha256": _file_sha256(canonical_model),
        "canonical_meshes": str(canonical_meshes.resolve()),
        "canonical_meshes_sha256": _tree_sha256(canonical_meshes),
        "native_scene": str(native_scene.resolve()),
        "native_scene_sha256": _file_sha256(native_scene),
        "body_joint_names": list(body_joint_names),
    }


def _strip_non_body_joints(
    canonical_model: Path,
    output_model: Path,
    *,
    body_joint_names: tuple[str, ...],
) -> tuple[str, ...]:
    try:
        tree = ET.parse(canonical_model)
    except ET.ParseError as exc:
        raise SonicPhysicsModelError(
            f"invalid canonical SONIC model {canonical_model}: {exc}"
        ) from exc
    root = tree.getroot()
    actuator = root.find("actuator")
    if actuator is None:
        raise SonicPhysicsModelError("canonical SONIC model has no actuator section")
    motors = list(actuator)
    body_actuator_count = len(body_joint_names)
    if len(set(body_joint_names)) != body_actuator_count:
        raise SonicPhysicsModelError("SONIC body joint contract contains duplicates")
    body_joint_set = set(body_joint_names)
    motor_by_joint = {motor.get("joint"): motor for motor in motors}
    missing_actuators = [
        joint_name for joint_name in body_joint_names if joint_name not in motor_by_joint
    ]
    if missing_actuators:
        raise SonicPhysicsModelError(
            f"canonical SONIC model is missing body actuators: {missing_actuators}"
        )

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise SonicPhysicsModelError("canonical SONIC model has no worldbody")
    for parent in worldbody.iter():
        for child in list(parent):
            if child.tag != "joint":
                continue
            if child.get("type") == "free":
                continue
            if child.get("name") not in body_joint_set:
                parent.remove(child)

    for motor in list(actuator):
        actuator.remove(motor)
    for joint_name in body_joint_names:
        actuator.append(motor_by_joint[joint_name])

    sensor = root.find("sensor")
    if sensor is not None:
        for item in list(sensor):
            joint_name = item.get("joint")
            actuator_name = item.get("actuator")
            if joint_name is not None and joint_name not in body_joint_set:
                sensor.remove(item)
            elif actuator_name is not None and actuator_name not in {
                motor.get("name") for motor in actuator
            }:
                sensor.remove(item)

    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("meshdir", "meshes")
    option = root.find("option")
    if option is None:
        option = ET.Element("option")
        root.insert(1, option)
    option.set("timestep", "0.005")
    root.set("model", "matrix_sonic_g1_29dof")
    root.insert(
        0,
        ET.Comment(
            f" derived from {canonical_model.name}; canonical {body_actuator_count}-joint SONIC body "
        ),
    )

    remaining_actuators = list(actuator)
    remaining_hinges = [
        joint
        for joint in worldbody.iter("joint")
        if joint.get("type") != "free"
    ]
    if len(remaining_actuators) != body_actuator_count:
        raise SonicPhysicsModelError(
            f"derived model has {len(remaining_actuators)} actuators, "
            f"expected {body_actuator_count}"
        )
    if len(remaining_hinges) != body_actuator_count:
        raise SonicPhysicsModelError(
            f"derived model has {len(remaining_hinges)} body joints, "
            f"expected {body_actuator_count}"
        )

    ET.indent(tree, space="  ")
    tree.write(output_model, encoding="utf-8", xml_declaration=False)
    with output_model.open("ab") as stream:
        stream.write(b"\n")
    return body_joint_names


def prepare_sonic_physics_model(
    canonical_model: Path,
    canonical_meshes: Path,
    native_scene: Path,
    output_dir: Path,
    *,
    body_joint_names: tuple[str, ...] = G1_BODY_JOINT_NAMES,
) -> Path:
    canonical_model = canonical_model.resolve()
    canonical_meshes = canonical_meshes.resolve()
    native_scene = native_scene.resolve()
    output_dir = output_dir.resolve()
    if not canonical_model.is_file():
        raise SonicPhysicsModelError(f"canonical SONIC model is missing: {canonical_model}")
    if not canonical_meshes.is_dir():
        raise SonicPhysicsModelError(f"canonical SONIC meshes are missing: {canonical_meshes}")
    if not native_scene.is_file():
        raise SonicPhysicsModelError(f"Matrix native scene is missing: {native_scene}")
    if not body_joint_names:
        raise SonicPhysicsModelError("body joint contract must not be empty")

    contract = _source_contract(
        canonical_model,
        canonical_meshes,
        native_scene,
        body_joint_names=body_joint_names,
    )
    manifest_path = output_dir / "manifest.json"
    scene_path = output_dir / native_scene.name
    if manifest_path.is_file() and scene_path.is_file():
        try:
            existing = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (json.JSONDecodeError, OSError):
            existing = None
        existing_contract = (
            {key: existing.get(key) for key in contract}
            if isinstance(existing, dict)
            else None
        )
        if existing_contract == contract:
            return scene_path

    output_dir.parent.mkdir(parents=True, exist_ok=True)
    temporary_dir = Path(
        tempfile.mkdtemp(prefix=f".{output_dir.name}.", dir=output_dir.parent)
    )
    try:
        shutil.copytree(canonical_meshes, temporary_dir / "meshes")
        body_joint_names = _strip_non_body_joints(
            canonical_model,
            temporary_dir / "robot.xml",
            body_joint_names=body_joint_names,
        )
        compose_custom_scene(
            native_scene,
            temporary_dir / native_scene.name,
            robot_include="robot.xml",
            source_asset_root=native_scene.parent / "assets",
            target_asset_root=temporary_dir / "meshes",
        )
        contract["body_joint_names"] = list(body_joint_names)
        (temporary_dir / "manifest.json").write_text(
            json.dumps(contract, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        if output_dir.exists():
            shutil.rmtree(output_dir)
        os.replace(temporary_dir, output_dir)
    except Exception:
        shutil.rmtree(temporary_dir, ignore_errors=True)
        raise
    return output_dir / native_scene.name


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--canonical-model", type=Path, required=True)
    parser.add_argument("--canonical-meshes", type=Path, required=True)
    parser.add_argument("--native-scene", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    try:
        scene = prepare_sonic_physics_model(
            args.canonical_model,
            args.canonical_meshes,
            args.native_scene,
            args.output_dir,
        )
    except SonicPhysicsModelError as exc:
        raise SystemExit(f"[ERROR] {exc}") from exc
    print(f"[INFO] Matrix SONIC physics model ready: {scene}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
