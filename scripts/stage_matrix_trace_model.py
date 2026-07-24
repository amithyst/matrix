#!/usr/bin/env python3
"""Stage and restore the full-hand model used by a Matrix trace replay."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import stat
import tempfile
from typing import Any, Sequence
import xml.etree.ElementTree as ET

from replay_matrix_physics_trace import validate_trace


STATE_SCHEMA = "matrix.physics_trace_model_stage.v1"
TARGET_RELATIVE_ROOTS = {
    "mujoco": Path("src/robot_mujoco/zsibot_robots/custom"),
    "ue": Path("src/UeSim/Linux/zsibot_mujoco_ue/Content/model/custom"),
}
RUNTIME_MUTATION_RELATIVE_PATHS = {
    "config_json": Path("config/config.json"),
    "sim_config": Path("src/robot_mujoco/simulate/config.yaml"),
    "mc_launcher": Path("src/robot_mc/run_mc.sh"),
    "mc_xg_parameters": Path(
        "src/robot_mc/build/export/config/xg-user-parameters.yaml"
    ),
    "mc_xgw_parameters": Path(
        "src/robot_mc/build/export/config/xg_wheel-user-parameters.yaml"
    ),
    "mc_zgws_parameters": Path(
        "src/robot_mc/build/export/config/zg_wheels-user-parameters.yaml"
    ),
    "mc_xxg_parameters": Path(
        "src/robot_mc/build/export/config/xxg-user-parameters.yaml"
    ),
    "mujoco_custom_scene": Path(
        "src/robot_mujoco/zsibot_robots/custom/scene_terrain_house.xml"
    ),
    "ue_custom_scene": Path(
        "src/UeSim/Linux/zsibot_mujoco_ue/Content/model/custom/scene_terrain_custom.xml"
    ),
    "ue_runtime_config": Path(
        "src/UeSim/Linux/zsibot_mujoco_ue/Content/model/config/config.json"
    ),
    "ue_scene_loader": Path(
        "src/UeSim/Linux/zsibot_mujoco_ue/Content/model/SceneLoder/scene.json"
    ),
}
CANONICAL_ACTUATOR_JOINTS = (
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
    "left_hand_thumb_0_joint",
    "left_hand_thumb_1_joint",
    "left_hand_thumb_2_joint",
    "left_hand_middle_0_joint",
    "left_hand_middle_1_joint",
    "left_hand_index_0_joint",
    "left_hand_index_1_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
    "right_hand_thumb_0_joint",
    "right_hand_thumb_1_joint",
    "right_hand_thumb_2_joint",
    "right_hand_middle_0_joint",
    "right_hand_middle_1_joint",
    "right_hand_index_0_joint",
    "right_hand_index_1_joint",
)


class ModelStageError(RuntimeError):
    """Raised when staging cannot preserve the active Matrix model safely."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _atomic_bytes(path: Path, payload: bytes, *, mode: int = 0o664) -> None:
    if path.is_symlink() or path.is_dir():
        raise ModelStageError(f"atomic output must not be a symlink or directory: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="wb", dir=path.parent, prefix=f".{path.name}.", delete=False
    ) as stream:
        stream.write(payload)
        temporary = Path(stream.name)
    os.chmod(temporary, mode)
    os.replace(temporary, path)


def _atomic_copy(source: Path, target: Path, *, mode: int) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        dir=target.parent, prefix=f".{target.name}."
    )
    os.close(descriptor)
    temporary = Path(temporary_name)
    try:
        shutil.copy2(source, temporary)
        os.chmod(temporary, mode)
        os.replace(temporary, target)
    finally:
        if temporary.exists():
            temporary.unlink()


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    _atomic_bytes(
        path,
        (json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n").encode(
            "utf-8"
        ),
        mode=0o600,
    )


def _regular_file(path: Path, *, label: str) -> Path:
    if path.is_symlink() or not path.is_file():
        raise ModelStageError(f"{label} must be a regular non-symlink file: {path}")
    return path.resolve()


def _safe_relative(value: str, *, label: str) -> Path:
    relative = Path(value)
    if relative.is_absolute() or not relative.parts or ".." in relative.parts:
        raise ModelStageError(f"{label} must be a confined relative path: {value}")
    if any(part in {"", "."} for part in relative.parts):
        raise ModelStageError(f"{label} contains an empty path component: {value}")
    return relative


def _load_robot_model(scene_model: Path) -> tuple[Path, ET.ElementTree]:
    scene_model = _regular_file(scene_model, label="trace scene model")
    try:
        scene_tree = ET.parse(scene_model)
    except ET.ParseError as exc:
        raise ModelStageError(f"invalid trace scene XML: {exc}") from exc
    scene_root = scene_tree.getroot()
    if scene_root.tag != "mujoco":
        raise ModelStageError("trace scene root must be <mujoco>")
    includes = scene_root.findall("include")
    if len(includes) == 1:
        include_name = includes[0].get("file")
        if not include_name:
            raise ModelStageError("trace scene robot include has no file")
        relative = _safe_relative(include_name, label="trace scene robot include")
        robot_candidate = scene_model.parent / relative
        if robot_candidate.is_symlink():
            raise ModelStageError("trace scene robot include must not be a symlink")
        robot_model = robot_candidate.resolve()
        try:
            robot_model.relative_to(scene_model.parent)
        except ValueError as exc:
            raise ModelStageError("trace scene robot include escapes its model root") from exc
        robot_model = _regular_file(robot_model, label="included full-hand robot model")
        try:
            robot_tree = ET.parse(robot_model)
        except ET.ParseError as exc:
            raise ModelStageError(f"invalid included robot XML: {exc}") from exc
        return robot_model, robot_tree
    if not includes:
        return scene_model, scene_tree
    raise ModelStageError(
        f"trace scene must have zero or one top-level include, got {len(includes)}"
    )


def _validate_robot(root: ET.Element) -> None:
    if root.tag != "mujoco":
        raise ModelStageError("included robot root must be <mujoco>")
    if root.findall("include"):
        raise ModelStageError("included robot must be flattened and contain no includes")
    actuator = root.find("actuator")
    if actuator is None:
        raise ModelStageError("included robot has no actuator section")
    actuator_joints = [item.get("joint") for item in list(actuator)]
    if tuple(actuator_joints) != CANONICAL_ACTUATOR_JOINTS:
        raise ModelStageError(
            "included robot actuator order must match canonical G1+Dex3 43-DOF order"
        )
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ModelStageError("included robot has no worldbody")
    cubes = [body for body in worldbody.iter("body") if body.get("name") == "pick_cube"]
    if len(cubes) != 1:
        raise ModelStageError("included robot must contain exactly one pick_cube body")
    cube_freejoints = [
        child
        for child in list(cubes[0])
        if child.tag == "freejoint"
        or (child.tag == "joint" and child.get("type") == "free")
    ]
    if len(cube_freejoints) != 1:
        raise ModelStageError("pick_cube must contain exactly one free joint")


def _mesh_closure(
    robot_model: Path, tree: ET.ElementTree
) -> tuple[Path, list[tuple[Path, Path, str]]]:
    root = tree.getroot()
    compiler = root.find("compiler")
    if compiler is None or not compiler.get("meshdir"):
        raise ModelStageError("included robot compiler.meshdir is required")
    meshdir = Path(str(compiler.get("meshdir"))).expanduser()
    mesh_candidate = (
        meshdir
        if meshdir.is_absolute()
        else robot_model.parent / meshdir
    )
    if mesh_candidate.is_symlink():
        raise ModelStageError(
            f"included robot meshdir must not be a symlink: {mesh_candidate}"
        )
    mesh_root = mesh_candidate.resolve()
    if not mesh_root.is_dir():
        raise ModelStageError(
            f"included robot meshdir must be a non-symlink directory: {mesh_root}"
        )
    assets = root.find("asset")
    mesh_elements = list(assets.iter("mesh")) if assets is not None else []
    if not mesh_elements:
        raise ModelStageError("included robot has no mesh assets")
    unsupported_files = (
        [
            element.tag
            for element in assets.iter()
            if element.tag != "mesh" and element.get("file")
        ]
        if assets is not None
        else []
    )
    if unsupported_files:
        raise ModelStageError(
            "included robot has unsupported non-mesh file assets: "
            + ", ".join(sorted(set(unsupported_files)))
        )
    closure: list[tuple[Path, Path, str]] = []
    seen: set[Path] = set()
    for index, mesh in enumerate(mesh_elements):
        file_name = mesh.get("file")
        if not file_name:
            raise ModelStageError(f"mesh asset {index} has no file")
        relative = _safe_relative(file_name, label=f"mesh asset {index}")
        if relative in seen:
            continue
        seen.add(relative)
        source_candidate = mesh_root / relative
        if source_candidate.is_symlink():
            raise ModelStageError(f"mesh asset must not be a symlink: {file_name}")
        source = source_candidate.resolve()
        try:
            source.relative_to(mesh_root)
        except ValueError as exc:
            raise ModelStageError(f"mesh asset escapes meshdir: {file_name}") from exc
        source = _regular_file(source, label=f"mesh asset {index}")
        closure.append((relative, source, _sha256(source)))
    closure.sort(key=lambda item: item[0].as_posix())
    return mesh_root, closure


def _closure_sha256(closure: list[tuple[Path, Path, str]]) -> str:
    digest = hashlib.sha256()
    for relative, _source, file_hash in closure:
        digest.update(relative.as_posix().encode("utf-8"))
        digest.update(b"\0")
        digest.update(file_hash.encode("ascii"))
        digest.update(b"\n")
    return digest.hexdigest()


def _normalized_robot_xml(
    tree: ET.ElementTree, *, relative_meshdir: Path
) -> bytes:
    root = tree.getroot()
    compiler = root.find("compiler")
    assert compiler is not None
    compiler.set("meshdir", relative_meshdir.as_posix())
    compiler.attrib.pop("texturedir", None)
    ET.indent(tree, space="  ")
    with tempfile.SpooledTemporaryFile(mode="w+b") as stream:
        tree.write(stream, encoding="utf-8", xml_declaration=False)
        stream.write(b"\n")
        stream.seek(0)
        return stream.read()


def _prepare_state_dir(state_dir: Path) -> Path:
    state_dir = state_dir.expanduser().resolve()
    state_dir.mkdir(parents=True, exist_ok=True)
    if (state_dir / "state.json").exists():
        raise ModelStageError(f"stage state already exists: {state_dir / 'state.json'}")
    if any(state_dir.iterdir()):
        raise ModelStageError(f"stage state directory must be empty: {state_dir}")
    return state_dir


def _target_roots(matrix_root: Path) -> dict[str, Path]:
    matrix_root = matrix_root.expanduser().resolve()
    if not matrix_root.is_dir():
        raise ModelStageError(f"Matrix root does not exist: {matrix_root}")
    roots: dict[str, Path] = {}
    for name, relative in TARGET_RELATIVE_ROOTS.items():
        parent = matrix_root / relative.parent
        if parent.is_symlink() or not parent.is_dir():
            raise ModelStageError(f"Matrix {name} model root is missing: {parent}")
        root = matrix_root / relative
        if root.is_symlink():
            raise ModelStageError(f"Matrix {name} custom model root is a symlink: {root}")
        root.mkdir(parents=True, exist_ok=True)
        roots[name] = root.resolve()
    return roots


def _install_bundle(
    target_root: Path,
    *,
    bundle_relative: Path,
    closure: list[tuple[Path, Path, str]],
    closure_hash: str,
) -> Path:
    bundle = target_root / bundle_relative
    bundle.parent.mkdir(parents=True, exist_ok=True)
    mesh_target = bundle / "meshes"
    manifest_path = bundle / "mesh-manifest.json"
    expected_manifest = {
        "schema_id": "matrix.physics_trace_mesh_closure.v1",
        "closure_sha256": closure_hash,
        "files": [
            {
                "path": relative.as_posix(),
                "sha256": file_hash,
                "size_bytes": source.stat().st_size,
            }
            for relative, source, file_hash in closure
        ],
    }
    if bundle.is_symlink():
        raise ModelStageError(f"existing staged mesh bundle is unsafe: {bundle}")
    if bundle.exists():
        if not bundle.is_dir():
            raise ModelStageError(f"existing staged mesh bundle is unsafe: {bundle}")
        try:
            existing = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise ModelStageError(f"existing staged mesh bundle is invalid: {bundle}") from exc
        if existing != expected_manifest:
            raise ModelStageError(f"existing staged mesh bundle conflicts: {bundle}")
        for item in expected_manifest["files"]:
            target = mesh_target / item["path"]
            if target.is_symlink() or not target.is_file() or _sha256(target) != item["sha256"]:
                raise ModelStageError(f"existing staged mesh file drifted: {target}")
        return bundle

    temporary = Path(
        tempfile.mkdtemp(prefix=f".{bundle.name}.", dir=bundle.parent)
    )
    try:
        temporary_meshes = temporary / "meshes"
        for relative, source, _file_hash in closure:
            destination = temporary_meshes / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, destination)
        _atomic_json(temporary / "mesh-manifest.json", expected_manifest)
        os.replace(temporary, bundle)
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return bundle


def _snapshot_file(path: Path, backup: Path, *, label: str) -> dict[str, Any]:
    existed = path.exists() or path.is_symlink()
    if not existed:
        return {
            "path": str(path),
            "original_existed": False,
            "original_sha256": None,
            "original_mode": None,
            "backup": None,
        }
    _regular_file(path, label=label)
    mode = stat.S_IMODE(path.stat().st_mode)
    backup.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(path, backup)
    return {
        "path": str(path),
        "original_existed": True,
        "original_sha256": _sha256(path),
        "original_mode": mode,
        "backup": str(backup),
    }


def _validate_mode(value: Any, *, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 0o7777:
        raise ModelStageError(f"{label} has invalid original mode")
    return value


def _validate_backup(
    item: dict[str, Any], *, state_dir: Path, label: str
) -> Path | None:
    if item.get("original_existed") is not True:
        if any(
            item.get(field) is not None
            for field in ("original_sha256", "original_mode", "backup")
        ):
            raise ModelStageError(f"{label} absent-file journal is invalid")
        return None
    backup = _regular_file(Path(str(item.get("backup"))), label=f"{label} backup")
    try:
        backup.relative_to(state_dir / "backups")
    except ValueError as exc:
        raise ModelStageError(f"{label} backup escapes the state directory") from exc
    if _sha256(backup) != item.get("original_sha256"):
        raise ModelStageError(f"{label} backup hash drifted")
    _validate_mode(item.get("original_mode"), label=label)
    return backup


def stage(
    *,
    matrix_root: Path,
    trace_path: Path,
    model_override: Path | None,
    state_dir: Path,
) -> dict[str, Any]:
    state_dir = _prepare_state_dir(state_dir)
    matrix_root = matrix_root.expanduser().resolve()
    validated = validate_trace(trace_path, model_override=model_override)
    robot_model, robot_tree = _load_robot_model(validated.model_path)
    if robot_model != validated.render_model_path:
        raise ModelStageError(
            "trace render_robot_model_path does not match the scene robot include"
        )
    if _sha256(robot_model) != validated.render_model_sha256:
        raise ModelStageError("trace render robot model hash drifted after validation")
    _validate_robot(robot_tree.getroot())
    _mesh_root, closure = _mesh_closure(robot_model, robot_tree)
    closure_hash = _closure_sha256(closure)
    robot_hash = _sha256(robot_model)
    bundle_key = hashlib.sha256(
        f"{robot_hash}:{closure_hash}".encode("ascii")
    ).hexdigest()[:20]
    bundle_relative = Path("_twinbot_trace_replay") / bundle_key
    active_meshdir = bundle_relative / "meshes"
    staged_xml = _normalized_robot_xml(robot_tree, relative_meshdir=active_meshdir)
    staged_hash = hashlib.sha256(staged_xml).hexdigest()
    roots = _target_roots(matrix_root)
    backups = state_dir / "backups"
    backups.mkdir()

    targets: dict[str, dict[str, Any]] = {}
    for name, root in roots.items():
        _install_bundle(
            root,
            bundle_relative=bundle_relative,
            closure=closure,
            closure_hash=closure_hash,
        )
        current = root / "current.xml"
        item = _snapshot_file(
            current,
            backups / "current" / f"{name}.xml",
            label=f"Matrix {name} active current.xml",
        )
        item.update(
            {
                "root": str(root),
                "current_xml": str(current),
                "staged_sha256": staged_hash,
                "staged_mode": item["original_mode"] or 0o664,
                "bundle": str(root / bundle_relative),
            }
        )
        targets[name] = item

    runtime_files = {
        name: _snapshot_file(
            matrix_root / relative,
            backups / "runtime" / name,
            label=f"Matrix runtime mutation target {name}",
        )
        for name, relative in RUNTIME_MUTATION_RELATIVE_PATHS.items()
    }
    state = {
        "schema_id": STATE_SCHEMA,
        "active": True,
        "phase": "prepared",
        "matrix_root": str(matrix_root),
        "physics_execution": "offline_mujoco_persistent_world",
        "render_mode": "matrix_ue_trace_replay",
        "trace": validated.inspection()["trace"],
        "scene_model": validated.inspection()["model"],
        "robot_model": {
            "path": str(robot_model),
            "sha256": robot_hash,
            "staged_sha256": staged_hash,
        },
        "dimensions": {"nq": 57, "nv": 55, "nu": 43},
        "mesh_closure": {
            "file_count": len(closure),
            "sha256": closure_hash,
            "bundle_key": bundle_key,
        },
        "targets": targets,
        "runtime_files": runtime_files,
        "installed_targets": [],
    }
    state_path = state_dir / "state.json"
    # This journal is durable before either active current.xml is replaced.
    _atomic_json(state_path, state)
    try:
        for name, item in targets.items():
            _atomic_bytes(
                Path(item["current_xml"]),
                staged_xml,
                mode=int(item["staged_mode"]),
            )
            state["installed_targets"].append(name)
            _atomic_json(state_path, state)
        state["phase"] = "active"
        _atomic_json(state_path, state)
    except BaseException:
        try:
            restore(matrix_root=matrix_root, state_dir=state_dir)
        except BaseException as restore_exc:
            raise ModelStageError(
                f"model stage failed and journal restore also failed: {restore_exc}"
            ) from restore_exc
        raise
    return state


def restore(
    *,
    matrix_root: Path,
    state_dir: Path,
    receipt_path: Path | None = None,
) -> dict[str, Any]:
    state_dir = state_dir.expanduser().resolve()
    state_path = state_dir / "state.json"
    try:
        state = json.loads(state_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ModelStageError(f"invalid stage state: {state_path}") from exc
    if state.get("schema_id") != STATE_SCHEMA:
        raise ModelStageError("unexpected stage state schema")
    expected_root = matrix_root.expanduser().resolve()
    if Path(str(state.get("matrix_root"))).resolve() != expected_root:
        raise ModelStageError("stage state belongs to a different Matrix root")
    if state.get("active") is False:
        if receipt_path is not None:
            _atomic_json(receipt_path.expanduser().resolve(), state)
        return state

    expected_targets = {
        name: (expected_root / relative / "current.xml").resolve()
        for name, relative in TARGET_RELATIVE_ROOTS.items()
    }
    targets = state.get("targets")
    if not isinstance(targets, dict) or set(targets) != set(expected_targets):
        raise ModelStageError("stage state target inventory is invalid")
    expected_runtime = {
        name: (expected_root / relative).resolve()
        for name, relative in RUNTIME_MUTATION_RELATIVE_PATHS.items()
    }
    runtime_files = state.get("runtime_files")
    if not isinstance(runtime_files, dict) or set(runtime_files) != set(expected_runtime):
        raise ModelStageError("stage state runtime-file inventory is invalid")

    # Validate the full journal and every active target before restoring any
    # bytes, so a drift failure cannot leave a half-restored Matrix checkout.
    prepared_targets: list[tuple[str, Path, dict[str, Any], Path | None]] = []
    for name, expected_current in expected_targets.items():
        item = targets[name]
        if not isinstance(item, dict):
            raise ModelStageError(f"stage state target is invalid: {name}")
        current = Path(str(item.get("current_xml"))).resolve()
        if current != expected_current or Path(str(item.get("path"))).resolve() != current:
            raise ModelStageError(f"stage target path drifted: {name}")
        backup = _validate_backup(
            item, state_dir=state_dir, label=f"Matrix {name} current.xml"
        )
        if current.exists() or current.is_symlink():
            _regular_file(current, label=f"Matrix {name} active current.xml")
            current_hash = _sha256(current)
            allowed_hashes = {str(item.get("staged_sha256"))}
            if item.get("original_existed") is True:
                allowed_hashes.add(str(item.get("original_sha256")))
            if current_hash not in allowed_hashes:
                raise ModelStageError(
                    f"Matrix {name} current.xml changed after staging; refusing to overwrite"
                )
        prepared_targets.append((name, current, item, backup))

    prepared_runtime: list[tuple[str, Path, dict[str, Any], Path | None]] = []
    for name, expected_path in expected_runtime.items():
        item = runtime_files[name]
        if not isinstance(item, dict) or Path(str(item.get("path"))).resolve() != expected_path:
            raise ModelStageError(f"runtime mutation target path drifted: {name}")
        backup = _validate_backup(
            item, state_dir=state_dir, label=f"Matrix runtime target {name}"
        )
        if expected_path.exists() or expected_path.is_symlink():
            _regular_file(expected_path, label=f"Matrix runtime target {name}")
        prepared_runtime.append((name, expected_path, item, backup))

    for _name, current, item, backup in prepared_targets:
        if backup is not None:
            mode = _validate_mode(item.get("original_mode"), label=str(current))
            _atomic_copy(backup, current, mode=mode)
        elif current.exists():
            current.unlink()
    for _name, path, item, backup in prepared_runtime:
        if backup is not None:
            mode = _validate_mode(item.get("original_mode"), label=str(path))
            _atomic_copy(backup, path, mode=mode)
        elif path.exists():
            path.unlink()

    state["active"] = False
    state["phase"] = "restored"
    state["restored_targets"] = [name for name, *_rest in prepared_targets]
    state["restored_runtime_files"] = [name for name, *_rest in prepared_runtime]
    _atomic_json(state_path, state)
    if receipt_path is not None:
        _atomic_json(receipt_path.expanduser().resolve(), state)
    return state


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    stage_parser = subparsers.add_parser("stage")
    stage_parser.add_argument("--matrix-root", type=Path, required=True)
    stage_parser.add_argument("--trace", type=Path, required=True)
    stage_parser.add_argument("--model", type=Path)
    stage_parser.add_argument("--state-dir", type=Path, required=True)
    restore_parser = subparsers.add_parser("restore")
    restore_parser.add_argument("--matrix-root", type=Path, required=True)
    restore_parser.add_argument("--state-dir", type=Path, required=True)
    restore_parser.add_argument("--receipt", type=Path)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    try:
        if args.command == "stage":
            result = stage(
                matrix_root=args.matrix_root,
                trace_path=args.trace,
                model_override=args.model,
                state_dir=args.state_dir,
            )
        else:
            result = restore(
                matrix_root=args.matrix_root,
                state_dir=args.state_dir,
                receipt_path=args.receipt,
            )
    except (OSError, ValueError, ModelStageError) as exc:
        print(f"[matrix-trace-model] ERROR: {exc}", file=os.sys.stderr)
        return 2
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
