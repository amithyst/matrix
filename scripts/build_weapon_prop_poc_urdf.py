#!/usr/bin/env python3
"""Build a G1 URDF with a visual-only training blaster fixed to one hand."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import struct
import xml.etree.ElementTree as ET


WEAPON_LINK = "training_blaster_link"
WEAPON_JOINT = "training_blaster_fixed_joint"
WEAPON_MESH = "training_blaster.stl"
WEAPON_RGBA = "0.95 0.19 0.035 1"
WEAPON_SCALE = "0.7 0.7 0.7"
KENNEY_COLORMAP_SHA256 = "4d0867c3c3c8c539106f91fbc0987d0e1b7d2811362d1daed593169c7d3b0fdc"
KENNEY_PALETTE = (
    (0.21875, "dark", "0.235294118 0.235294118 0.262745098 1"),
    (0.34375, "graphite", "0.42745098 0.447058824 0.541176471 1"),
    (0.59375, "silver_blue", "0.623529412 0.650980392 0.780392157 1"),
    (0.71875, "orange", "0.917647059 0.384313725 0.274509804 1"),
)


def _resolve_mesh(source_urdf: Path, filename: str) -> Path:
    if filename.startswith("package://"):
        raise ValueError(f"package URI is not supported by this PoC: {filename}")
    path = Path(filename)
    if not path.is_absolute():
        path = source_urdf.parent / path
    path = path.resolve()
    if not path.is_file():
        raise FileNotFoundError(f"URDF mesh does not exist: {path}")
    return path


def _parse_obj(path: Path) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
    vertices: list[tuple[float, float, float]] = []
    triangles: list[tuple[int, int, int]] = []
    for line_number, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        fields = raw_line.strip().split()
        if not fields or fields[0].startswith("#"):
            continue
        if fields[0] == "v" and len(fields) >= 4:
            vertices.append(tuple(float(value) for value in fields[1:4]))
        elif fields[0] == "f" and len(fields) >= 4:
            indices: list[int] = []
            for field in fields[1:]:
                vertex_index = int(field.split("/", 1)[0])
                if vertex_index == 0:
                    raise ValueError(f"OBJ index 0 at {path}:{line_number}")
                resolved = vertex_index - 1 if vertex_index > 0 else len(vertices) + vertex_index
                if resolved < 0 or resolved >= len(vertices):
                    raise ValueError(f"OBJ vertex index out of range at {path}:{line_number}")
                indices.append(resolved)
            for index in range(1, len(indices) - 1):
                triangles.append((indices[0], indices[index], indices[index + 1]))
    if not vertices or not triangles:
        raise ValueError(f"OBJ has no usable triangle mesh: {path}")
    return vertices, triangles


def _normal(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    c: tuple[float, float, float],
) -> tuple[float, float, float]:
    ab = tuple(b[index] - a[index] for index in range(3))
    ac = tuple(c[index] - a[index] for index in range(3))
    cross = (
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    )
    length = math.sqrt(sum(value * value for value in cross))
    if length == 0:
        return (0.0, 0.0, 0.0)
    return tuple(value / length for value in cross)


def _write_binary_stl(
    vertices: list[tuple[float, float, float]],
    triangles: list[tuple[int, int, int]],
    output: Path,
) -> int:
    header = b"Matrix weapon prop PoC".ljust(80, b"\0")
    with output.open("wb") as stream:
        stream.write(header)
        stream.write(struct.pack("<I", len(triangles)))
        for triangle in triangles:
            points = [vertices[index] for index in triangle]
            stream.write(struct.pack("<3f", *_normal(*points)))
            for point in points:
                stream.write(struct.pack("<3f", *point))
            stream.write(struct.pack("<H", 0))
    return len(triangles)


def convert_obj_to_binary_stl(source: Path, output: Path) -> int:
    vertices, triangles = _parse_obj(source)
    return _write_binary_stl(vertices, triangles, output)


def _triangle_palette_groups(path: Path) -> list[str]:
    texture_coordinates: list[tuple[float, float]] = []
    groups: list[str] = []
    for line_number, raw_line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), 1
    ):
        fields = raw_line.strip().split()
        if not fields or fields[0].startswith("#"):
            continue
        if fields[0] == "vt" and len(fields) >= 3:
            texture_coordinates.append(tuple(float(value) for value in fields[1:3]))
        elif fields[0] == "f" and len(fields) >= 4:
            uv_indices: list[int] = []
            for field in fields[1:]:
                components = field.split("/")
                if len(components) < 2 or not components[1]:
                    raise ValueError(f"OBJ face has no UV at {path}:{line_number}")
                uv_index = int(components[1])
                resolved = (
                    uv_index - 1
                    if uv_index > 0
                    else len(texture_coordinates) + uv_index
                )
                if resolved < 0 or resolved >= len(texture_coordinates):
                    raise ValueError(f"OBJ UV index out of range at {path}:{line_number}")
                uv_indices.append(resolved)
            for index in range(1, len(uv_indices) - 1):
                triangle = (uv_indices[0], uv_indices[index], uv_indices[index + 1])
                mean_u = sum(texture_coordinates[item][0] for item in triangle) / 3
                _, group, _ = min(
                    KENNEY_PALETTE,
                    key=lambda entry: abs(entry[0] - mean_u),
                )
                groups.append(group)
    return groups


def convert_obj_to_palette_stls(source: Path, output_dir: Path) -> dict[str, int]:
    vertices, triangles = _parse_obj(source)
    groups = _triangle_palette_groups(source)
    if len(groups) != len(triangles):
        raise ValueError("OBJ geometry and UV triangle counts differ")
    grouped_triangles: dict[str, list[tuple[int, int, int]]] = {
        group: [] for _, group, _ in KENNEY_PALETTE
    }
    for triangle, group in zip(triangles, groups, strict=True):
        grouped_triangles[group].append(triangle)
    counts = {}
    for _, group, _ in KENNEY_PALETTE:
        group_triangles = grouped_triangles[group]
        if not group_triangles:
            continue
        counts[group] = _write_binary_stl(
            vertices,
            group_triangles,
            output_dir / f"training_blaster_{group}.stl",
        )
    return counts


def _add_weapon_part(
    root: ET.Element,
    *,
    group: str,
    rgba: str,
    mesh_name: str,
    parent_link: str,
    joint_xyz: str,
    joint_rpy: str,
    visual_xyz: str,
    visual_rpy: str,
    weapon_scale: str,
) -> tuple[str, str]:
    suffix = "" if group == "single" else f"_{group}"
    link_name = f"training_blaster{suffix}_link"
    joint_name = f"training_blaster{suffix}_fixed_joint"
    weapon_link = ET.SubElement(root, "link", {"name": link_name})
    visual = ET.SubElement(weapon_link, "visual")
    ET.SubElement(visual, "origin", {"xyz": visual_xyz, "rpy": visual_rpy})
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(
        geometry,
        "mesh",
        {"filename": f"assets/{mesh_name}", "scale": weapon_scale},
    )
    material_group = "orange" if group == "single" else group
    material = ET.SubElement(
        visual,
        "material",
        {"name": f"matrix_source_training_{material_group}"},
    )
    ET.SubElement(material, "color", {"rgba": rgba})
    joint = ET.SubElement(root, "joint", {"name": joint_name, "type": "fixed"})
    ET.SubElement(joint, "origin", {"xyz": joint_xyz, "rpy": joint_rpy})
    ET.SubElement(joint, "parent", {"link": parent_link})
    ET.SubElement(joint, "child", {"link": link_name})
    return link_name, joint_name


def build_weapon_urdf(
    source_urdf: Path,
    weapon_obj: Path,
    output_dir: Path,
    *,
    parent_link: str = "right_rubber_hand",
    joint_xyz: str = "0 0 0",
    joint_rpy: str = "0 0 0",
    visual_xyz: str = "0.12 0 0",
    visual_rpy: str = "0 1.57079632679 0",
    weapon_scale: str = WEAPON_SCALE,
    weapon_colormap: Path | None = None,
    license_path: Path | None = None,
    source_metadata_path: Path | None = None,
) -> dict[str, object]:
    source_urdf = source_urdf.resolve()
    weapon_obj = weapon_obj.resolve()
    output_dir = output_dir.resolve()
    if not source_urdf.is_file():
        raise FileNotFoundError(f"source URDF does not exist: {source_urdf}")
    if not weapon_obj.is_file():
        raise FileNotFoundError(f"weapon OBJ does not exist: {weapon_obj}")

    tree = ET.parse(source_urdf)
    root = tree.getroot()
    links = {link.get("name") for link in root.findall("link")}
    joints = {joint.get("name") for joint in root.findall("joint")}
    if parent_link not in links:
        raise ValueError(f"parent link is absent from source URDF: {parent_link}")
    if any(name and name.startswith("training_blaster") for name in links | joints):
        raise ValueError("source URDF already contains the training blaster")

    assets_dir = output_dir / "assets"
    if output_dir.exists():
        shutil.rmtree(output_dir)
    assets_dir.mkdir(parents=True)

    copied_meshes: dict[str, Path] = {}
    for mesh in root.iter("mesh"):
        filename = mesh.get("filename")
        if not filename:
            continue
        source_mesh = _resolve_mesh(source_urdf, filename)
        destination = assets_dir / source_mesh.name
        previous = copied_meshes.get(source_mesh.name)
        if previous is not None and previous.read_bytes() != source_mesh.read_bytes():
            raise ValueError(f"different source meshes share basename {source_mesh.name}")
        if previous is None:
            shutil.copy2(source_mesh, destination)
            copied_meshes[source_mesh.name] = source_mesh
        mesh.set("filename", f"assets/{source_mesh.name}")

    weapon_parts: list[tuple[str, str, str]] = []
    if weapon_colormap is not None:
        weapon_colormap = weapon_colormap.resolve()
        if not weapon_colormap.is_file():
            raise FileNotFoundError(f"weapon colormap does not exist: {weapon_colormap}")
        colormap_sha256 = hashlib.sha256(weapon_colormap.read_bytes()).hexdigest()
        if colormap_sha256 != KENNEY_COLORMAP_SHA256:
            raise ValueError(
                "weapon colormap does not match Kenney Blaster Kit 2.1: "
                f"{colormap_sha256}"
            )
        shutil.copy2(weapon_colormap, assets_dir / "kenney_colormap.png")
        triangle_counts = convert_obj_to_palette_stls(weapon_obj, assets_dir)
        palette_by_group = {group: rgba for _, group, rgba in KENNEY_PALETTE}
        for _, group, _ in KENNEY_PALETTE:
            if group in triangle_counts:
                weapon_parts.append(
                    (
                        group,
                        f"training_blaster_{group}.stl",
                        palette_by_group[group],
                    )
                )
    else:
        triangle_counts = {
            "single": convert_obj_to_binary_stl(
                weapon_obj,
                assets_dir / WEAPON_MESH,
            )
        }
        weapon_parts.append(("single", WEAPON_MESH, WEAPON_RGBA))
    for provenance in (license_path, source_metadata_path):
        if provenance is not None:
            provenance = provenance.resolve()
            if not provenance.is_file():
                raise FileNotFoundError(f"provenance file does not exist: {provenance}")
            shutil.copy2(provenance, assets_dir / provenance.name)

    weapon_links = []
    weapon_joints = []
    for group, mesh_name, rgba in weapon_parts:
        link_name, joint_name = _add_weapon_part(
            root,
            group=group,
            rgba=rgba,
            mesh_name=mesh_name,
            parent_link=parent_link,
            joint_xyz=joint_xyz,
            joint_rpy=joint_rpy,
            visual_xyz=visual_xyz,
            visual_rpy=visual_rpy,
            weapon_scale=weapon_scale,
        )
        weapon_links.append(link_name)
        weapon_joints.append(joint_name)

    output_urdf = output_dir / "g1_blaster_poc.urdf"
    ET.indent(tree, space="  ")
    tree.write(output_urdf, encoding="utf-8", xml_declaration=True)
    return {
        "output_urdf": str(output_urdf),
        "assets_dir": str(assets_dir),
        "parent_link": parent_link,
        "source_meshes": len(copied_meshes),
        "weapon_triangles": sum(triangle_counts.values()),
        "weapon_triangle_groups": triangle_counts,
        "weapon_links": weapon_links,
        "weapon_joints": weapon_joints,
        "weapon_scale": weapon_scale,
        "visual_xyz": visual_xyz,
        "visual_rpy": visual_rpy,
        "visual_only": True,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-urdf", type=Path, required=True)
    parser.add_argument("--weapon-obj", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--parent-link", default="right_rubber_hand")
    parser.add_argument("--joint-xyz", default="0 0 0")
    parser.add_argument("--joint-rpy", default="0 0 0")
    parser.add_argument("--visual-xyz", default="0.12 0 0")
    parser.add_argument("--visual-rpy", default="0 1.57079632679 0")
    parser.add_argument("--weapon-scale", default=WEAPON_SCALE)
    parser.add_argument("--weapon-colormap", type=Path)
    parser.add_argument("--license", type=Path)
    parser.add_argument("--source-metadata", type=Path)
    args = parser.parse_args()
    result = build_weapon_urdf(
        args.source_urdf,
        args.weapon_obj,
        args.output_dir,
        parent_link=args.parent_link,
        joint_xyz=args.joint_xyz,
        joint_rpy=args.joint_rpy,
        visual_xyz=args.visual_xyz,
        visual_rpy=args.visual_rpy,
        weapon_scale=args.weapon_scale,
        weapon_colormap=args.weapon_colormap,
        license_path=args.license,
        source_metadata_path=args.source_metadata,
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
