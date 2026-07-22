#!/usr/bin/env python3
"""Build a G1 URDF with a visual-only training blaster fixed to one hand."""

from __future__ import annotations

import argparse
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


def convert_obj_to_binary_stl(source: Path, output: Path) -> int:
    vertices, triangles = _parse_obj(source)
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
    if WEAPON_LINK in links or WEAPON_JOINT in joints:
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

    triangle_count = convert_obj_to_binary_stl(weapon_obj, assets_dir / WEAPON_MESH)
    for provenance in (license_path, source_metadata_path):
        if provenance is not None:
            provenance = provenance.resolve()
            if not provenance.is_file():
                raise FileNotFoundError(f"provenance file does not exist: {provenance}")
            shutil.copy2(provenance, assets_dir / provenance.name)

    weapon_link = ET.SubElement(root, "link", {"name": WEAPON_LINK})
    visual = ET.SubElement(weapon_link, "visual")
    ET.SubElement(visual, "origin", {"xyz": visual_xyz, "rpy": visual_rpy})
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(
        geometry,
        "mesh",
        {"filename": f"assets/{WEAPON_MESH}", "scale": weapon_scale},
    )
    material = ET.SubElement(
        visual,
        "material",
        {"name": "matrix_source_training_orange"},
    )
    ET.SubElement(material, "color", {"rgba": WEAPON_RGBA})

    joint = ET.SubElement(root, "joint", {"name": WEAPON_JOINT, "type": "fixed"})
    ET.SubElement(joint, "origin", {"xyz": joint_xyz, "rpy": joint_rpy})
    ET.SubElement(joint, "parent", {"link": parent_link})
    ET.SubElement(joint, "child", {"link": WEAPON_LINK})

    output_urdf = output_dir / "g1_blaster_poc.urdf"
    ET.indent(tree, space="  ")
    tree.write(output_urdf, encoding="utf-8", xml_declaration=True)
    return {
        "output_urdf": str(output_urdf),
        "assets_dir": str(assets_dir),
        "parent_link": parent_link,
        "source_meshes": len(copied_meshes),
        "weapon_triangles": triangle_count,
        "weapon_link": WEAPON_LINK,
        "weapon_joint": WEAPON_JOINT,
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
        license_path=args.license,
        source_metadata_path=args.source_metadata,
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
