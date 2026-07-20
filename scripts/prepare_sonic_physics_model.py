#!/usr/bin/env python3
"""Prepare SONIC's canonical 29-DOF G1 physics model for a Matrix map."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
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


PIPELINE_VERSION = 3
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
    paths = sorted(root.rglob("*"))
    if any(path.is_symlink() for path in paths):
        raise SonicPhysicsModelError(f"source tree contains a symlink: {root}")
    for path in (item for item in paths if item.is_file()):
        digest.update(path.relative_to(root).as_posix().encode("utf-8"))
        digest.update(b"\0")
        digest.update(_file_sha256(path).encode("ascii"))
        digest.update(b"\n")
    return digest.hexdigest()


def _bundle_sha256(root: Path) -> str:
    """Hash every derived file except the self-describing manifest."""
    digest = hashlib.sha256()
    paths = sorted(root.rglob("*"))
    if any(path.is_symlink() for path in paths):
        raise SonicPhysicsModelError(f"derived bundle contains a symlink: {root}")
    for path in (item for item in paths if item.is_file()):
        relative = path.relative_to(root).as_posix()
        if relative == "manifest.json":
            continue
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(_file_sha256(path).encode("ascii"))
        digest.update(b"\n")
    return digest.hexdigest()


def _native_scene_asset_inventory(native_scene: Path) -> list[dict[str, object]]:
    """Resolve every native scene file input, including assets/../ siblings."""
    try:
        root = ET.parse(native_scene).getroot()
    except ET.ParseError as exc:
        raise SonicPhysicsModelError(
            f"invalid Matrix native scene {native_scene}: {exc}"
        ) from exc
    scene_root = native_scene.parent.resolve()
    asset_root = scene_root / "assets"
    assets = root.find("asset")
    if assets is None:
        return []
    sources: dict[Path, dict[str, object]] = {}
    for element in assets.iter():
        file_name = element.get("file")
        if not file_name:
            continue
        relative = Path(file_name)
        if relative.is_absolute():
            raise SonicPhysicsModelError(
                f"native scene asset must be relative: {file_name}"
            )
        source = (asset_root / relative).resolve()
        try:
            source_relative = source.relative_to(scene_root)
        except ValueError as exc:
            raise SonicPhysicsModelError(
                f"native scene asset escapes its robot root: {file_name}"
            ) from exc
        if not source.is_file() or source.is_symlink():
            raise SonicPhysicsModelError(
                f"native scene asset is not a regular file: {source}"
            )
        sources[source] = {
            "path": str(source),
            "relative_path": source_relative.as_posix(),
            "size": source.stat().st_size,
            "sha256": _file_sha256(source),
        }
    return [sources[path] for path in sorted(sources)]


def _source_contract(
    canonical_model: Path,
    canonical_meshes: Path,
    native_scene: Path,
    *,
    body_joint_names: tuple[str, ...],
    spawn_xyz: tuple[float, float, float] | None,
    spawn_yaw: float | None,
) -> dict[str, object]:
    native_assets = native_scene.parent / "assets"
    return {
        "pipeline_version": PIPELINE_VERSION,
        "canonical_model": str(canonical_model.resolve()),
        "canonical_model_sha256": _file_sha256(canonical_model),
        "canonical_meshes": str(canonical_meshes.resolve()),
        "canonical_meshes_sha256": _tree_sha256(canonical_meshes),
        "native_scene": str(native_scene.resolve()),
        "native_scene_sha256": _file_sha256(native_scene),
        "native_assets": str(native_assets.resolve()) if native_assets.is_dir() else None,
        "native_assets_sha256": (
            _tree_sha256(native_assets) if native_assets.is_dir() else None
        ),
        "native_scene_assets": _native_scene_asset_inventory(native_scene),
        "body_joint_names": list(body_joint_names),
        "spawn_xyz": list(spawn_xyz) if spawn_xyz is not None else None,
        "spawn_yaw_rad": spawn_yaw,
    }


def physics_revision_payload(
    canonical_model: Path,
    canonical_meshes: Path,
    native_scene: Path,
    *,
    body_joint_names: tuple[str, ...] = G1_BODY_JOINT_NAMES,
) -> dict[str, object]:
    """Return the location-independent source contract for save isolation.

    The preparation manifest intentionally records absolute provenance paths and
    the selected spawn override.  Neither belongs in a persistent-world
    revision: identical physics assets copied to another host must select the
    same save slot, while changing a resume pose must not invalidate that slot.
    Keep the content-bearing fields sourced from :func:`_source_contract` so
    preparation and persistence cannot silently drift apart.
    """

    contract = _source_contract(
        canonical_model,
        canonical_meshes,
        native_scene,
        body_joint_names=body_joint_names,
        spawn_xyz=None,
        spawn_yaw=None,
    )
    native_scene_assets = []
    for asset in contract["native_scene_assets"]:
        if not isinstance(asset, dict):
            raise SonicPhysicsModelError("native scene asset contract is invalid")
        native_scene_assets.append(
            {
                "relative_path": asset["relative_path"],
                "size": asset["size"],
                "sha256": asset["sha256"],
            }
        )
    return {
        "schema": "matrix-sonic-physics-source/v1",
        "pipeline_version": contract["pipeline_version"],
        "canonical_model_sha256": contract["canonical_model_sha256"],
        "canonical_meshes_sha256": contract["canonical_meshes_sha256"],
        "native_scene_sha256": contract["native_scene_sha256"],
        "native_assets_sha256": contract["native_assets_sha256"],
        "native_scene_assets": native_scene_assets,
        "body_joint_names": contract["body_joint_names"],
    }


def _strip_non_body_joints(
    canonical_model: Path,
    output_model: Path,
    *,
    body_joint_names: tuple[str, ...],
    spawn_xyz: tuple[float, float, float] | None,
    spawn_yaw: float | None,
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
    if spawn_xyz is not None or spawn_yaw is not None:
        root_body = next(
            (
                body
                for body in worldbody.iter("body")
                if any(
                    child.tag == "freejoint"
                    or (child.tag == "joint" and child.get("type") == "free")
                    for child in list(body)
                )
            ),
            None,
        )
        if root_body is None:
            raise SonicPhysicsModelError(
                "canonical SONIC model has no body with a free root joint"
            )
        if spawn_xyz is not None:
            root_body.set("pos", " ".join(f"{value:.12g}" for value in spawn_xyz))
        if spawn_yaw is not None:
            root_body.set(
                "quat",
                f"{math.cos(spawn_yaw / 2.0):.12g} 0 0 "
                f"{math.sin(spawn_yaw / 2.0):.12g}",
            )
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
    spawn_xyz: tuple[float, float, float] | None = None,
    spawn_yaw: float | None = None,
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
    if spawn_xyz is not None and (
        len(spawn_xyz) != 3
        or not all(math.isfinite(float(value)) for value in spawn_xyz)
    ):
        raise SonicPhysicsModelError("spawn_xyz must contain three finite values")
    if spawn_yaw is not None and not math.isfinite(float(spawn_yaw)):
        raise SonicPhysicsModelError("spawn_yaw must be finite")
    normalized_spawn_xyz = (
        tuple(float(value) for value in spawn_xyz)
        if spawn_xyz is not None
        else None
    )
    normalized_spawn_yaw = float(spawn_yaw) if spawn_yaw is not None else None

    contract = _source_contract(
        canonical_model,
        canonical_meshes,
        native_scene,
        body_joint_names=body_joint_names,
        spawn_xyz=normalized_spawn_xyz,
        spawn_yaw=normalized_spawn_yaw,
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
            derived_outputs = {
                "derived_robot_sha256": output_dir / "robot.xml",
                "derived_scene_sha256": scene_path,
                "derived_meshes_sha256": output_dir / "meshes",
                "derived_bundle_sha256": output_dir,
            }
            derived_match = True
            for key, path in derived_outputs.items():
                if key == "derived_meshes_sha256":
                    actual = _tree_sha256(path) if path.is_dir() else None
                elif key == "derived_bundle_sha256":
                    actual = _bundle_sha256(path) if path.is_dir() else None
                else:
                    actual = _file_sha256(path) if path.is_file() else None
                if existing.get(key) != actual:
                    derived_match = False
                    break
            if derived_match:
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
            spawn_xyz=normalized_spawn_xyz,
            spawn_yaw=normalized_spawn_yaw,
        )
        compose_custom_scene(
            native_scene,
            temporary_dir / native_scene.name,
            robot_include="robot.xml",
            source_asset_root=native_scene.parent / "assets",
            target_asset_root=temporary_dir / "meshes",
        )
        contract["body_joint_names"] = list(body_joint_names)
        contract["derived_robot_sha256"] = _file_sha256(temporary_dir / "robot.xml")
        contract["derived_scene_sha256"] = _file_sha256(
            temporary_dir / native_scene.name
        )
        contract["derived_meshes_sha256"] = _tree_sha256(temporary_dir / "meshes")
        contract["derived_bundle_sha256"] = _bundle_sha256(temporary_dir)
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
    parser.add_argument("--spawn-x", type=float)
    parser.add_argument("--spawn-y", type=float)
    parser.add_argument("--spawn-z", type=float)
    parser.add_argument("--spawn-yaw", type=float)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    spawn_components = (args.spawn_x, args.spawn_y, args.spawn_z)
    if any(value is not None for value in spawn_components) and not all(
        value is not None for value in spawn_components
    ):
        raise SystemExit("[ERROR] --spawn-x, --spawn-y, and --spawn-z must be set together")
    spawn_xyz = (
        tuple(float(value) for value in spawn_components)
        if all(value is not None for value in spawn_components)
        else None
    )
    try:
        scene = prepare_sonic_physics_model(
            args.canonical_model,
            args.canonical_meshes,
            args.native_scene,
            args.output_dir,
            spawn_xyz=spawn_xyz,
            spawn_yaw=args.spawn_yaw,
        )
    except SonicPhysicsModelError as exc:
        raise SystemExit(f"[ERROR] {exc}") from exc
    print(f"[INFO] Matrix SONIC physics model ready: {scene}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
