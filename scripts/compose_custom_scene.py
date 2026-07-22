#!/usr/bin/env python3
"""Compose a custom robot with one of Matrix's native MuJoCo scenes."""

from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET


class SceneCompositionError(RuntimeError):
    """Raised when a native scene cannot be composed reproducibly."""


def _is_relative_to(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
    except ValueError:
        return False
    return True


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _copy_scene_assets(
    root: ET.Element,
    *,
    source_scene_root: Path,
    source_asset_root: Path,
    target_scene_root: Path,
    target_asset_root: Path,
) -> list[Path]:
    asset = root.find("asset")
    if asset is None:
        return []

    copied: list[Path] = []
    for element in asset.iter():
        file_name = element.get("file")
        if not file_name:
            continue
        relative_path = Path(file_name)
        if relative_path.is_absolute():
            raise SceneCompositionError(
                f"native scene asset must use a relative path: {file_name}"
            )

        source = (source_asset_root / relative_path).resolve()
        target = (target_asset_root / relative_path).resolve()
        if not _is_relative_to(source, source_scene_root):
            raise SceneCompositionError(
                f"native scene asset escapes its robot root: {file_name}"
            )
        if not _is_relative_to(target, target_scene_root):
            raise SceneCompositionError(
                f"composed scene asset escapes its custom root: {file_name}"
            )
        if not source.is_file():
            raise SceneCompositionError(
                f"native scene asset does not exist: {source}"
            )

        target.parent.mkdir(parents=True, exist_ok=True)
        if target.exists() and _sha256(target) != _sha256(source):
            raise SceneCompositionError(
                "native scene asset conflicts with an existing custom robot asset: "
                f"{target}"
            )
        if not target.exists():
            shutil.copy2(source, target)
        copied.append(target)
    return copied


def _remove_named_geoms(
    root: ET.Element, *, remove_geoms: tuple[str, ...]
) -> tuple[str, ...]:
    if not remove_geoms:
        return ()
    if any(not name for name in remove_geoms):
        raise SceneCompositionError("removed geom names must be non-empty")
    if len(set(remove_geoms)) != len(remove_geoms):
        raise SceneCompositionError("removed geom names must not contain duplicates")

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise SceneCompositionError("native scene has no worldbody")
    requested = set(remove_geoms)
    removed: set[str] = set()
    for parent in worldbody.iter():
        for child in list(parent):
            if child.tag != "geom" or child.get("name") not in requested:
                continue
            name = child.get("name")
            assert name is not None
            if name in removed:
                raise SceneCompositionError(
                    f"native scene contains duplicate requested geom: {name}"
                )
            parent.remove(child)
            removed.add(name)

    missing = [name for name in remove_geoms if name not in removed]
    if missing:
        raise SceneCompositionError(
            f"native scene is missing requested geoms: {', '.join(missing)}"
        )
    return tuple(name for name in remove_geoms if name in removed)


def freejoint_body_names(root: ET.Element) -> tuple[str, ...]:
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise SceneCompositionError("native scene has no worldbody")

    names: list[str] = []
    seen: set[str] = set()
    for body in worldbody.iter("body"):
        has_freejoint = any(
            child.tag == "freejoint"
            or (child.tag == "joint" and child.get("type") == "free")
            for child in list(body)
        )
        if not has_freejoint:
            continue
        name = body.get("name")
        if not name:
            raise SceneCompositionError("freejoint body must have a name")
        if name in seen:
            raise SceneCompositionError(f"duplicate freejoint body name: {name}")
        seen.add(name)
        names.append(name)
    return tuple(names)


def _staticize_freejoint_bodies(root: ET.Element) -> tuple[str, ...]:
    names = freejoint_body_names(root)
    if not names:
        return ()

    for body in root.find("worldbody").iter("body"):  # type: ignore[union-attr]
        for child in list(body):
            if child.tag == "freejoint" or (
                child.tag == "joint" and child.get("type") == "free"
            ):
                body.remove(child)
    return names


def compose_custom_scene(
    source_scene: Path,
    output_scene: Path,
    *,
    robot_include: str = "current.xml",
    source_asset_root: Path | None = None,
    target_asset_root: Path | None = None,
    remove_geoms: tuple[str, ...] = (),
    staticize_freejoint_bodies: bool = False,
) -> list[Path]:
    source_scene = source_scene.resolve()
    output_scene = output_scene.resolve()
    if not source_scene.is_file():
        raise SceneCompositionError(f"native scene does not exist: {source_scene}")
    if Path(robot_include).is_absolute():
        raise SceneCompositionError("robot include must be relative to the custom scene")

    try:
        tree = ET.parse(source_scene)
    except ET.ParseError as exc:
        raise SceneCompositionError(
            f"invalid native scene XML {source_scene}: {exc}"
        ) from exc
    root = tree.getroot()
    if root.tag != "mujoco":
        raise SceneCompositionError(
            f"native scene root must be <mujoco>, got <{root.tag}>"
        )

    includes = root.findall("include")
    if len(includes) != 1:
        raise SceneCompositionError(
            f"native scene must have exactly one top-level robot include, got {len(includes)}"
        )
    includes[0].set("file", robot_include)
    staticized = (
        _staticize_freejoint_bodies(root) if staticize_freejoint_bodies else ()
    )
    removed = _remove_named_geoms(root, remove_geoms=remove_geoms)
    if removed:
        root.insert(
            0,
            ET.Comment(f" removed environment geoms: {','.join(removed)} "),
        )
    if staticized:
        root.insert(
            0,
            ET.Comment(
                " staticized freejoint bodies: "
                f"{len(staticized)} ({','.join(staticized[:8])}"
                f"{'...' if len(staticized) > 8 else ''}) "
            ),
        )
    root.set("model", f"custom::{source_scene.stem}")
    root.insert(
        0,
        ET.Comment(
            f" generated from {source_scene.name}; robot include={robot_include} "
        ),
    )

    source_scene_root = source_scene.parent.resolve()
    target_scene_root = output_scene.parent.resolve()
    source_asset_root = (
        source_asset_root.resolve()
        if source_asset_root is not None
        else source_scene_root / "assets"
    )
    target_asset_root = (
        target_asset_root.resolve()
        if target_asset_root is not None
        else target_scene_root / "assets"
    )
    copied = _copy_scene_assets(
        root,
        source_scene_root=source_scene_root,
        source_asset_root=source_asset_root,
        target_scene_root=target_scene_root,
        target_asset_root=target_asset_root,
    )

    output_scene.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    with tempfile.NamedTemporaryFile(
        mode="wb", dir=output_scene.parent, prefix=f".{output_scene.name}.", delete=False
    ) as stream:
        temporary_path = Path(stream.name)
        tree.write(stream, encoding="utf-8", xml_declaration=False)
        stream.write(b"\n")
    os.replace(temporary_path, output_scene)
    return copied


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source_scene", type=Path)
    parser.add_argument("output_scene", type=Path)
    parser.add_argument("--robot-include", default="current.xml")
    parser.add_argument("--source-asset-root", type=Path)
    parser.add_argument("--target-asset-root", type=Path)
    parser.add_argument("--remove-geom", action="append", default=[])
    parser.add_argument("--staticize-freejoint-bodies", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    try:
        copied = compose_custom_scene(
            args.source_scene,
            args.output_scene,
            robot_include=args.robot_include,
            source_asset_root=args.source_asset_root,
            target_asset_root=args.target_asset_root,
            remove_geoms=tuple(args.remove_geom),
            staticize_freejoint_bodies=args.staticize_freejoint_bodies,
        )
    except SceneCompositionError as exc:
        raise SystemExit(f"[ERROR] {exc}") from exc
    print(
        "[INFO] composed native scene "
        f"source={args.source_scene} output={args.output_scene} "
        f"assets={len(copied)}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
