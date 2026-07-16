#!/usr/bin/env python3
"""Compose translated Matrix native physics proxies into one adjacent world."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import tempfile
from typing import Any
import xml.etree.ElementTree as ET


PIPELINE_VERSION = 1
REFERENCE_TAGS = {
    "mesh": "mesh",
    "material": "material",
    "hfield": "hfield",
    "texture": "texture",
}


class OverworldCompositionError(RuntimeError):
    """Raised when adjacent native scenes cannot be composed safely."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _is_relative_to(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
    except ValueError:
        return False
    return True


def _load_layout(path: Path) -> dict[str, Any]:
    try:
        layout = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise OverworldCompositionError(f"invalid Overworld layout {path}: {exc}") from exc
    if layout.get("schema_version") != 1:
        raise OverworldCompositionError(
            f"unsupported layout schema_version={layout.get('schema_version')!r}"
        )
    if not layout.get("scenes"):
        raise OverworldCompositionError("layout must contain at least one scene")
    return layout


def _world_bounds(scene: dict[str, Any]) -> tuple[float, float, float, float]:
    bounds = scene.get("source_bounds_xy")
    translation = scene.get("translation")
    if not isinstance(bounds, list) or len(bounds) != 4:
        raise OverworldCompositionError(
            f"scene {scene.get('key')} must define source_bounds_xy=[min_x,min_y,max_x,max_y]"
        )
    if not isinstance(translation, list) or len(translation) != 3:
        raise OverworldCompositionError(
            f"scene {scene.get('key')} must define translation=[x,y,z]"
        )
    min_x, min_y, max_x, max_y = [float(value) for value in bounds]
    if min_x >= max_x or min_y >= max_y:
        raise OverworldCompositionError(f"scene {scene.get('key')} has invalid bounds")
    yaw = math.radians(float(scene.get("yaw_deg", 0.0)))
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    points = []
    for x, y in ((min_x, min_y), (min_x, max_y), (max_x, min_y), (max_x, max_y)):
        points.append(
            (
                cosine * x - sine * y + float(translation[0]),
                sine * x + cosine * y + float(translation[1]),
            )
        )
    return (
        min(point[0] for point in points),
        min(point[1] for point in points),
        max(point[0] for point in points),
        max(point[1] for point in points),
    )


def _validate_layout(layout: dict[str, Any]) -> dict[str, tuple[float, float, float, float]]:
    scenes = layout["scenes"]
    keys = [scene.get("key") for scene in scenes]
    if any(not isinstance(key, str) or not key for key in keys):
        raise OverworldCompositionError("every scene must have a non-empty key")
    if len(keys) != len(set(keys)):
        raise OverworldCompositionError("scene keys must be unique")

    bounds = {scene["key"]: _world_bounds(scene) for scene in scenes}
    for index, left in enumerate(scenes):
        left_bounds = bounds[left["key"]]
        for right in scenes[index + 1 :]:
            right_bounds = bounds[right["key"]]
            overlap_x = min(left_bounds[2], right_bounds[2]) - max(
                left_bounds[0], right_bounds[0]
            )
            overlap_y = min(left_bounds[3], right_bounds[3]) - max(
                left_bounds[1], right_bounds[1]
            )
            if overlap_x > 1e-6 and overlap_y > 1e-6:
                raise OverworldCompositionError(
                    f"scene bounds overlap: {left['key']} and {right['key']} "
                    f"by ({overlap_x:.3f}, {overlap_y:.3f}) m"
                )

    connectors = layout.get("connectors", [])
    connector_keys: set[str] = set()
    for connector in connectors:
        key = connector.get("key")
        if not isinstance(key, str) or not key or key in connector_keys:
            raise OverworldCompositionError("connector keys must be unique and non-empty")
        connector_keys.add(key)
        for endpoint in ("from", "to"):
            if connector.get(endpoint) not in bounds:
                raise OverworldCompositionError(
                    f"connector {key} references unknown {endpoint} scene"
                )
        center = connector.get("center_xy")
        half_extents = connector.get("half_extents_xy")
        if not isinstance(center, list) or len(center) != 2:
            raise OverworldCompositionError(f"connector {key} must define center_xy")
        if (
            not isinstance(half_extents, list)
            or len(half_extents) != 2
            or min(float(value) for value in half_extents) <= 0.0
        ):
            raise OverworldCompositionError(
                f"connector {key} must define positive half_extents_xy"
            )
    return bounds


def _prune_world_element(
    element: ET.Element, *, remove_geoms: set[str]
) -> tuple[ET.Element | None, int, int]:
    if not isinstance(element.tag, str) or element.tag == "light":
        return None, 0, 0
    if element.tag == "geom":
        if element.get("type", "sphere") == "plane":
            return None, 0, 0
        if element.get("name") in remove_geoms:
            return None, 0, 1

    cloned = copy.deepcopy(element)
    geom_count = 1 if cloned.tag == "geom" else 0
    removed_count = 0
    for child in list(cloned):
        replacement, child_geoms, child_removed = _prune_world_element(
            child, remove_geoms=remove_geoms
        )
        cloned.remove(child)
        if replacement is not None:
            cloned.append(replacement)
        geom_count += child_geoms
        removed_count += child_removed
    return cloned, geom_count, removed_count


def _collect_asset_references(elements: list[ET.Element]) -> set[tuple[str, str]]:
    references: set[tuple[str, str]] = set()
    for root in elements:
        for element in root.iter():
            for attribute, tag in REFERENCE_TAGS.items():
                value = element.get(attribute)
                if value:
                    references.add((tag, value))
    return references


def _copy_file_asset(
    element: ET.Element,
    *,
    scene_key: str,
    source_scene_root: Path,
    target_asset_root: Path,
) -> None:
    file_name = element.get("file")
    if not file_name:
        return
    relative = Path(file_name)
    if relative.is_absolute():
        raise OverworldCompositionError(
            f"scene {scene_key} asset must use a relative path: {file_name}"
        )
    source_asset_root = source_scene_root / "assets"
    source = (source_asset_root / relative).resolve()
    if not _is_relative_to(source, source_scene_root):
        raise OverworldCompositionError(
            f"scene {scene_key} asset escapes its source root: {file_name}"
        )
    if not source.is_file():
        raise OverworldCompositionError(
            f"scene {scene_key} asset does not exist: {source}"
        )
    if _is_relative_to(source, source_asset_root.resolve()):
        target_relative = source.relative_to(source_asset_root.resolve())
    else:
        target_relative = Path("_root") / source.relative_to(source_scene_root)
    target_relative = Path(scene_key) / target_relative
    target = (target_asset_root / target_relative).resolve()
    if not _is_relative_to(target, target_asset_root.resolve()):
        raise OverworldCompositionError(
            f"scene {scene_key} asset escapes the output root: {file_name}"
        )
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists() and _sha256(target) != _sha256(source):
        raise OverworldCompositionError(f"output asset collision: {target}")
    if not target.exists():
        shutil.copy2(source, target)
    element.set("file", target_relative.as_posix())


def _copy_referenced_assets(
    source_root: ET.Element,
    world_elements: list[ET.Element],
    *,
    scene_key: str,
    source_scene_root: Path,
    target_asset_root: Path,
) -> tuple[list[ET.Element], dict[tuple[str, str], str]]:
    source_asset = source_root.find("asset")
    if source_asset is None:
        if _collect_asset_references(world_elements):
            raise OverworldCompositionError(
                f"scene {scene_key} references assets but has no asset section"
            )
        return [], {}

    by_key: dict[tuple[str, str], ET.Element] = {}
    for element in source_asset:
        if isinstance(element.tag, str) and element.get("name"):
            by_key[(element.tag, element.get("name"))] = element

    required = _collect_asset_references(world_elements)
    selected: set[tuple[str, str]] = set()
    queue = list(required)
    while queue:
        reference = queue.pop()
        if reference in selected:
            continue
        source = by_key.get(reference)
        if source is None:
            raise OverworldCompositionError(
                f"scene {scene_key} references missing {reference[0]} {reference[1]!r}"
            )
        selected.add(reference)
        for attribute, tag in REFERENCE_TAGS.items():
            value = source.get(attribute)
            dependency = (tag, value) if value else None
            if dependency is not None and dependency not in selected:
                queue.append(dependency)

    names = {
        reference: f"{scene_key}__{reference[1]}" for reference in sorted(selected)
    }
    copied: list[ET.Element] = []
    for reference in sorted(selected):
        element = copy.deepcopy(by_key[reference])
        element.set("name", names[reference])
        for attribute, tag in REFERENCE_TAGS.items():
            value = element.get(attribute)
            if value and (tag, value) in names:
                element.set(attribute, names[(tag, value)])
        _copy_file_asset(
            element,
            scene_key=scene_key,
            source_scene_root=source_scene_root,
            target_asset_root=target_asset_root,
        )
        copied.append(element)
    return copied, names


def _namespace_world_elements(
    elements: list[ET.Element],
    *,
    scene_key: str,
    asset_names: dict[tuple[str, str], str],
) -> None:
    for root in elements:
        for element in root.iter():
            name = element.get("name")
            if name:
                element.set("name", f"{scene_key}__{name}")
            for attribute, tag in REFERENCE_TAGS.items():
                value = element.get(attribute)
                if value and (tag, value) in asset_names:
                    element.set(attribute, asset_names[(tag, value)])


def _atomic_write_xml(tree: ET.ElementTree, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    with tempfile.NamedTemporaryFile(
        mode="wb", dir=path.parent, prefix=f".{path.name}.", delete=False
    ) as stream:
        temporary = Path(stream.name)
        tree.write(stream, encoding="utf-8", xml_declaration=False)
        stream.write(b"\n")
    os.replace(temporary, path)


def _atomic_write_json(payload: dict[str, Any], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", dir=path.parent, prefix=f".{path.name}.", delete=False
    ) as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
        temporary = Path(stream.name)
    os.replace(temporary, path)


def _build_overworld_bundle(
    layout_path: Path,
    native_scene_root: Path,
    output_scene: Path,
    *,
    target_asset_root: Path,
    robot_include: str = "xgb.xml",
) -> tuple[ET.ElementTree, dict[str, Any]]:
    layout_path = layout_path.resolve()
    native_scene_root = native_scene_root.resolve()
    output_scene = output_scene.resolve()
    if not layout_path.is_file():
        raise OverworldCompositionError(f"layout does not exist: {layout_path}")
    if not native_scene_root.is_dir():
        raise OverworldCompositionError(
            f"native scene root does not exist: {native_scene_root}"
        )
    if Path(robot_include).is_absolute():
        raise OverworldCompositionError("robot include must be relative")

    layout = _load_layout(layout_path)
    world_bounds = _validate_layout(layout)
    target_asset_root = target_asset_root.resolve()
    target_asset_root.mkdir(parents=True, exist_ok=False)

    root = ET.Element("mujoco", attrib={"model": layout["world_name"]})
    root.append(
        ET.Comment(
            " generated adjacent physics world; UE visual composition remains gated by visual_contract "
        )
    )
    ET.SubElement(root, "include", attrib={"file": robot_include})
    asset = ET.SubElement(root, "asset")
    worldbody = ET.SubElement(root, "worldbody")
    ET.SubElement(
        worldbody,
        "light",
        attrib={"pos": "0 0 20", "dir": "0 0 -1", "directional": "true"},
    )
    ground = layout.get("ground", {})
    ground_z = float(ground.get("z", 0.0))
    friction = ground.get("friction", [1.0, 0.01, 0.01])
    ET.SubElement(
        worldbody,
        "geom",
        attrib={
            "name": "overworld_ground",
            "type": "plane",
            "pos": f"0 0 {ground_z:.9g}",
            "size": "0 0 0.05",
            "friction": " ".join(f"{float(value):.9g}" for value in friction),
        },
    )

    scene_manifest: list[dict[str, Any]] = []
    environment_geoms = 1
    for scene in layout["scenes"]:
        scene_key = scene["key"]
        source_scene = native_scene_root / scene["source_scene"]
        if not source_scene.is_file():
            raise OverworldCompositionError(
                f"native scene {scene_key} does not exist: {source_scene}"
            )
        try:
            source_tree = ET.parse(source_scene)
        except ET.ParseError as exc:
            raise OverworldCompositionError(
                f"invalid native scene XML {source_scene}: {exc}"
            ) from exc
        source_root = source_tree.getroot()
        source_worldbody = source_root.find("worldbody")
        if source_worldbody is None:
            raise OverworldCompositionError(f"scene {scene_key} has no worldbody")
        if any(
            element.tag in {"joint", "freejoint"}
            for element in source_worldbody.iter()
            if isinstance(element.tag, str)
        ):
            raise OverworldCompositionError(
                f"scene {scene_key} contains dynamic joints; adjacent V1 accepts static proxies only"
            )

        remove_geoms = set(scene.get("remove_geoms", []))
        copied_world: list[ET.Element] = []
        copied_geoms = 0
        removed_geoms = 0
        for child in source_worldbody:
            cloned, child_geoms, child_removed = _prune_world_element(
                child, remove_geoms=remove_geoms
            )
            if cloned is not None:
                copied_world.append(cloned)
            copied_geoms += child_geoms
            removed_geoms += child_removed
        expected = int(scene["expected_copied_geoms"])
        if copied_geoms != expected:
            raise OverworldCompositionError(
                f"scene {scene_key} copied {copied_geoms} geoms, expected {expected}"
            )
        if removed_geoms != len(remove_geoms):
            raise OverworldCompositionError(
                f"scene {scene_key} removed {removed_geoms}/{len(remove_geoms)} requested geoms"
            )

        copied_assets, asset_names = _copy_referenced_assets(
            source_root,
            copied_world,
            scene_key=scene_key,
            source_scene_root=native_scene_root,
            target_asset_root=target_asset_root,
        )
        for copied_asset in copied_assets:
            asset.append(copied_asset)
        _namespace_world_elements(
            copied_world, scene_key=scene_key, asset_names=asset_names
        )

        translation = [float(value) for value in scene["translation"]]
        yaw = math.radians(float(scene.get("yaw_deg", 0.0)))
        scene_body = ET.SubElement(
            worldbody,
            "body",
            attrib={
                "name": f"overworld_scene__{scene_key}",
                "pos": " ".join(f"{value:.9g}" for value in translation),
                "quat": f"{math.cos(yaw / 2.0):.9g} 0 0 {math.sin(yaw / 2.0):.9g}",
            },
        )
        for element in copied_world:
            scene_body.append(element)
        environment_geoms += copied_geoms
        scene_manifest.append(
            {
                "key": scene_key,
                "source_scene": str(source_scene),
                "source_sha256": _sha256(source_scene),
                "translation": translation,
                "yaw_deg": float(scene.get("yaw_deg", 0.0)),
                "world_bounds_xy": [round(value, 6) for value in world_bounds[scene_key]],
                "copied_geoms": copied_geoms,
                "removed_geoms": sorted(remove_geoms),
                "copied_assets": len(copied_assets),
            }
        )

    for connector in layout.get("connectors", []):
        center = [float(value) for value in connector["center_xy"]]
        half_extents = [float(value) for value in connector["half_extents_xy"]]
        ET.SubElement(
            worldbody,
            "site",
            attrib={
                "name": f"overworld_connector__{connector['key']}",
                "type": "box",
                "pos": f"{center[0]:.9g} {center[1]:.9g} {ground_z + 0.01:.9g}",
                "size": f"{half_extents[0]:.9g} {half_extents[1]:.9g} 0.01",
                "rgba": "0.1 0.8 0.2 0.35",
            },
        )

    expected_environment_geoms = int(layout["expected_environment_geoms"])
    if environment_geoms != expected_environment_geoms:
        raise OverworldCompositionError(
            f"composed environment has {environment_geoms} geoms, "
            f"expected {expected_environment_geoms}"
        )

    tree = ET.ElementTree(root)
    manifest = {
        "pipeline_version": PIPELINE_VERSION,
        "layout": str(layout_path),
        "layout_sha256": _sha256(layout_path),
        "world_name": layout["world_name"],
        "mode": layout["mode"],
        "visual_contract": layout["visual_contract"],
        "environment_geoms": environment_geoms,
        "connectors": layout.get("connectors", []),
        "scenes": scene_manifest,
        "output_scene": str(output_scene),
    }
    return tree, manifest


def _remove_path(path: Path) -> None:
    if path.is_symlink() or path.is_file():
        path.unlink()
    elif path.exists():
        shutil.rmtree(path)


def _publish_staged_bundle(staging_root: Path, output_scene: Path) -> None:
    if output_scene.name in {"assets", "manifest.json"}:
        raise OverworldCompositionError(
            "output scene name must not conflict with assets or manifest.json"
        )

    output_root = output_scene.parent
    staged_items = [
        (staging_root / "assets", output_root / "assets"),
        (staging_root / output_scene.name, output_scene),
        (staging_root / "manifest.json", output_root / "manifest.json"),
    ]
    for source, _target in staged_items:
        if not source.exists():
            raise OverworldCompositionError(f"staged bundle item is missing: {source}")

    backup_root = staging_root / "previous"
    backup_root.mkdir()
    backed_up: list[tuple[Path, Path]] = []
    published: list[Path] = []
    try:
        for index, (_source, target) in enumerate(staged_items):
            if target.exists() or target.is_symlink():
                backup = backup_root / f"{index}-{target.name}"
                os.replace(target, backup)
                backed_up.append((backup, target))
        for source, target in staged_items:
            os.replace(source, target)
            published.append(target)
    except BaseException:
        for target in reversed(published):
            _remove_path(target)
        for backup, target in reversed(backed_up):
            if backup.exists() or backup.is_symlink():
                os.replace(backup, target)
        raise


def compose_overworld_scene(
    layout_path: Path,
    native_scene_root: Path,
    output_scene: Path,
    *,
    robot_include: str = "xgb.xml",
) -> dict[str, Any]:
    output_scene = output_scene.resolve()
    output_scene.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(
        prefix=f".{output_scene.stem}.staging.", dir=output_scene.parent
    ) as temporary_dir:
        staging_root = Path(temporary_dir)
        tree, manifest = _build_overworld_bundle(
            layout_path,
            native_scene_root,
            output_scene,
            target_asset_root=staging_root / "assets",
            robot_include=robot_include,
        )
        _atomic_write_xml(tree, staging_root / output_scene.name)
        _atomic_write_json(manifest, staging_root / "manifest.json")
        _publish_staged_bundle(staging_root, output_scene)
    return manifest


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--layout", type=Path, required=True)
    parser.add_argument("--native-scene-root", type=Path, required=True)
    parser.add_argument("--output-scene", type=Path, required=True)
    parser.add_argument("--robot-include", default="xgb.xml")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    try:
        manifest = compose_overworld_scene(
            args.layout,
            args.native_scene_root,
            args.output_scene,
            robot_include=args.robot_include,
        )
    except OverworldCompositionError as exc:
        raise SystemExit(f"[ERROR] {exc}") from exc
    print(
        "[INFO] Matrix Overworld adjacent physics ready: "
        f"scene={args.output_scene} scenes={len(manifest['scenes'])} "
        f"environment_geoms={manifest['environment_geoms']} "
        f"visual={manifest['visual_contract']['status']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
