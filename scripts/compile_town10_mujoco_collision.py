#!/usr/bin/env python3
"""Compile Town10 visual/source geometry into deterministic MuJoCo collision.

The input contract is intentionally small and explicit: callers provide
already-extracted visual or source geometry in Matrix's canonical frame
(right-handed, Z-up, meters after ``meters_per_unit`` conversion).  The
compiler emits a self-contained MJCF collision scene, simplified collision OBJ
meshes, and a manifest with source and artifact checksums.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import tempfile
import xml.etree.ElementTree as ET
from typing import Any, Iterable


SOURCE_SCHEMA = "matrix-town10-visual-collision-source/v1"
BUNDLE_SCHEMA = "matrix-town10-mujoco-collision-bundle/v1"
PIPELINE_VERSION = 1
CANONICAL_FRAME = {
    "id": "frame.matrix_world",
    "meters_per_unit": 1.0,
    "up_axis": "Z",
    "handedness": "right",
}
DEFAULT_FRICTION = "1 0.005 0.0001"
DEFAULT_SOLREF = "0.02 1"
DEFAULT_SOLIMP = "0.9 0.95 0.001 0.5 2"
ID_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_.:-]{0,127}$")
SAFE_NAME_RE = re.compile(r"[^A-Za-z0-9_.:-]+")
MAX_SOURCE_BYTES = 64 * 1024 * 1024
MAX_OBJECTS = 65536
MAX_VERTICES = 2_000_000
MAX_FACES = 4_000_000


class Town10CollisionCompileError(ValueError):
    """Fail-closed compiler error."""


@dataclass(frozen=True)
class Transform:
    translation_m: tuple[float, float, float]
    rotation_xyzw: tuple[float, float, float, float]
    scale: tuple[float, float, float]

    @property
    def rotation_wxyz(self) -> tuple[float, float, float, float]:
        x, y, z, w = self.rotation_xyzw
        return (w, x, y, z)


@dataclass(frozen=True)
class CollisionObject:
    object_id: str
    kind: str
    transform: Transform
    geometry: dict[str, Any]
    collision_enabled: bool = True


@dataclass(frozen=True)
class CompileResult:
    xml_path: Path
    manifest_path: Path
    output_dir: Path


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _tree_sha256(root: Path, *, exclude: frozenset[str] = frozenset()) -> str:
    digest = hashlib.sha256()
    paths = sorted(root.rglob("*"))
    if any(path.is_symlink() for path in paths):
        raise Town10CollisionCompileError(f"output tree contains a symlink: {root}")
    for path in (item for item in paths if item.is_file()):
        relative = path.relative_to(root).as_posix()
        if relative in exclude:
            continue
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(_file_sha256(path).encode("ascii"))
        digest.update(b"\n")
    return digest.hexdigest()


def _loads_json_strict(text: str, *, source: str) -> Any:
    def no_duplicates(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise Town10CollisionCompileError(f"{source}: duplicate key {key!r}")
            result[key] = value
        return result

    def no_constants(value: str) -> None:
        raise Town10CollisionCompileError(f"{source}: non-finite number {value}")

    try:
        return json.loads(
            text,
            object_pairs_hook=no_duplicates,
            parse_constant=no_constants,
        )
    except Town10CollisionCompileError:
        raise
    except (json.JSONDecodeError, ValueError) as exc:
        raise Town10CollisionCompileError(f"{source}: invalid JSON: {exc}") from exc


def _load_source(path: Path) -> dict[str, Any]:
    if not path.is_file() or path.is_symlink():
        raise Town10CollisionCompileError(f"source is not a regular file: {path}")
    if path.stat().st_size > MAX_SOURCE_BYTES:
        raise Town10CollisionCompileError(f"source is too large: {path}")
    try:
        text = path.read_text(encoding="utf-8")
    except UnicodeDecodeError as exc:
        raise Town10CollisionCompileError(f"source is not UTF-8 JSON: {path}") from exc
    document = _loads_json_strict(text, source=str(path))
    return _validate_source_document(document)


def _validate_source_document(document: Any) -> dict[str, Any]:
    _require_object(document, "document")
    _require_equal(document.get("schema"), SOURCE_SCHEMA, "schema")
    frame = _validate_frame(document.get("coordinate_frame"))
    objects = _require_list(document.get("objects"), "objects")
    if len(objects) > MAX_OBJECTS:
        raise Town10CollisionCompileError(f"objects exceeds {MAX_OBJECTS}")
    normalized_objects = [
        _normalize_object(item, f"objects[{index}]", frame["meters_per_unit"])
        for index, item in enumerate(objects)
    ]
    return {
        "schema": SOURCE_SCHEMA,
        "coordinate_frame": frame,
        "source": _optional_object(document.get("source"), "source"),
        "objects": normalized_objects,
    }


def _validate_frame(value: Any) -> dict[str, Any]:
    frame = _require_object(value, "coordinate_frame")
    up_axis = frame.get("up_axis")
    handedness = frame.get("handedness")
    meters_per_unit = _finite_float(
        frame.get("meters_per_unit", 1.0), "coordinate_frame.meters_per_unit"
    )
    if meters_per_unit <= 0.0:
        raise Town10CollisionCompileError("coordinate_frame.meters_per_unit must be > 0")
    if up_axis not in {"Z", "+Z"}:
        raise Town10CollisionCompileError(
            "only canonical Matrix Z-up input is supported at this boundary"
        )
    if handedness != "right":
        raise Town10CollisionCompileError(
            "only right-handed input is supported at this boundary"
        )
    return {
        "id": str(frame.get("id", CANONICAL_FRAME["id"])),
        "meters_per_unit": meters_per_unit,
        "up_axis": "Z",
        "handedness": "right",
    }


def _normalize_object(
    value: Any,
    label: str,
    meters_per_unit: float,
) -> CollisionObject:
    item = _require_object(value, label)
    object_id = _require_id(item.get("id"), f"{label}.id")
    kind = str(item.get("kind", "scene"))
    collision_enabled = _optional_bool(
        item.get("collision_enabled", True),
        f"{label}.collision_enabled",
    )
    transform = _normalize_transform(
        _optional_object(item.get("transform"), f"{label}.transform"),
        f"{label}.transform",
        meters_per_unit,
    )
    geometry = _normalize_geometry(
        _require_object(item.get("geometry"), f"{label}.geometry"),
        f"{label}.geometry",
        meters_per_unit,
    )
    return CollisionObject(
        object_id=object_id,
        kind=kind,
        transform=transform,
        geometry=geometry,
        collision_enabled=collision_enabled,
    )


def _normalize_transform(
    value: dict[str, Any],
    label: str,
    meters_per_unit: float,
) -> Transform:
    translation = _vector(
        value.get("translation", (0.0, 0.0, 0.0)),
        f"{label}.translation",
        length=3,
    )
    rotation_xyzw = _normalize_quaternion_xyzw(
        _vector(
            value.get("rotation_xyzw", (0.0, 0.0, 0.0, 1.0)),
            f"{label}.rotation_xyzw",
            length=4,
        ),
        f"{label}.rotation_xyzw",
    )
    scale = _vector(value.get("scale", (1.0, 1.0, 1.0)), f"{label}.scale", length=3)
    if any(component <= 0.0 for component in scale):
        raise Town10CollisionCompileError(f"{label}.scale components must be > 0")
    return Transform(
        translation_m=tuple(component * meters_per_unit for component in translation),
        rotation_xyzw=rotation_xyzw,
        scale=scale,
    )


def _normalize_geometry(
    value: dict[str, Any],
    label: str,
    meters_per_unit: float,
) -> dict[str, Any]:
    geometry_type = value.get("type")
    if geometry_type == "box":
        if "half_extents" in value:
            half_extents = _vector(
                value["half_extents"],
                f"{label}.half_extents",
                length=3,
            )
        elif "size" in value:
            size = _vector(value["size"], f"{label}.size", length=3)
            half_extents = tuple(component / 2.0 for component in size)
        else:
            raise Town10CollisionCompileError(
                f"{label}: box requires half_extents or size"
            )
        if any(component <= 0.0 for component in half_extents):
            raise Town10CollisionCompileError(f"{label}: box extents must be > 0")
        center = _vector(
            value.get("center", (0.0, 0.0, 0.0)),
            f"{label}.center",
            length=3,
        )
        return {
            "type": "box",
            "half_extents_m": tuple(
                component * meters_per_unit for component in half_extents
            ),
            "center_m": tuple(component * meters_per_unit for component in center),
    }
    if geometry_type == "mesh":
        vertices = _vertices(
            value.get("vertices"),
            f"{label}.vertices",
            meters_per_unit,
        )
        faces = _faces(value.get("faces"), f"{label}.faces", vertex_count=len(vertices))
        return {
            "type": "mesh",
            "vertices_m": vertices,
            "faces": faces,
        }
    raise Town10CollisionCompileError(f"{label}.type must be 'box' or 'mesh'")


def _vertices(
    value: Any,
    label: str,
    meters_per_unit: float,
) -> tuple[tuple[float, float, float], ...]:
    rows = _require_list(value, label)
    if not rows:
        raise Town10CollisionCompileError(f"{label} must not be empty")
    if len(rows) > MAX_VERTICES:
        raise Town10CollisionCompileError(f"{label} exceeds {MAX_VERTICES}")
    return tuple(
        tuple(
            component * meters_per_unit
            for component in _vector(row, f"{label}[{index}]", length=3)
        )
        for index, row in enumerate(rows)
    )


def _faces(
    value: Any,
    label: str,
    *,
    vertex_count: int,
) -> tuple[tuple[int, int, int], ...]:
    rows = _require_list(value, label)
    if not rows:
        raise Town10CollisionCompileError(f"{label} must not be empty")
    if len(rows) > MAX_FACES:
        raise Town10CollisionCompileError(f"{label} exceeds {MAX_FACES}")
    faces: list[tuple[int, int, int]] = []
    for index, row in enumerate(rows):
        if not isinstance(row, list) or len(row) != 3:
            raise Town10CollisionCompileError(f"{label}[{index}] must be a triangle")
        face = tuple(_int(item, f"{label}[{index}]") for item in row)
        if len(set(face)) != 3 or any(
            item < 0 or item >= vertex_count for item in face
        ):
            raise Town10CollisionCompileError(
                f"{label}[{index}] references invalid vertices"
            )
        faces.append(face)
    return tuple(faces)


def compile_town10_mujoco_collision(
    source_path: Path,
    output_dir: Path,
) -> CompileResult:
    """Compile one source JSON document into a deterministic collision bundle."""

    source_path = source_path.resolve()
    output_dir = output_dir.resolve()
    document = _load_source(source_path)
    objects: list[CollisionObject] = list(document["objects"])
    output_dir.parent.mkdir(parents=True, exist_ok=True)
    temporary_dir = Path(
        tempfile.mkdtemp(prefix=f".{output_dir.name}.", dir=output_dir.parent)
    )
    try:
        mesh_dir = temporary_dir / "meshes"
        mesh_dir.mkdir()
        xml_root = ET.Element("mujoco", {"model": "matrix_town10_collision"})
        ET.SubElement(
            xml_root,
            "compiler",
            {
                "angle": "radian",
                "coordinate": "local",
                "meshdir": "meshes",
            },
        )
        ET.SubElement(xml_root, "option", {"timestep": "0.005"})
        asset = ET.SubElement(xml_root, "asset")
        worldbody = ET.SubElement(xml_root, "worldbody")
        manifest_objects: list[dict[str, Any]] = []
        used_names: set[str] = set()
        for item in objects:
            if not item.collision_enabled:
                manifest_objects.append(_manifest_skipped_object(item))
                continue
            safe_name = _unique_name(_safe_name(item.object_id), used_names)
            manifest_objects.append(
                _emit_object(
                    item,
                    safe_name=safe_name,
                    asset=asset,
                    worldbody=worldbody,
                    mesh_dir=mesh_dir,
                )
            )
        if len(asset) == 0:
            xml_root.remove(asset)
        xml_path = temporary_dir / "collision.xml"
        ET.indent(ET.ElementTree(xml_root), space="  ")
        ET.ElementTree(xml_root).write(
            xml_path,
            encoding="utf-8",
            xml_declaration=False,
        )
        with xml_path.open("ab") as stream:
            stream.write(b"\n")
        manifest = {
            "schema": BUNDLE_SCHEMA,
            "pipeline_version": PIPELINE_VERSION,
            "canonical_frame": CANONICAL_FRAME,
            "source": {
                "path": str(source_path),
                "size_bytes": source_path.stat().st_size,
                "sha256": _file_sha256(source_path),
                "schema": SOURCE_SCHEMA,
                "coordinate_frame": document["coordinate_frame"],
                "metadata": document["source"],
            },
            "collision": {
                "unit": "meter",
                "frame": "right-handed Z-up Matrix world",
                "mesh_simplification": "mesh-aabb-box-v1",
                "friction": DEFAULT_FRICTION,
                "solref": DEFAULT_SOLREF,
                "solimp": DEFAULT_SOLIMP,
                "objects": manifest_objects,
            },
            "artifacts": {
                "xml": {
                    "path": "collision.xml",
                    "size_bytes": xml_path.stat().st_size,
                    "sha256": _file_sha256(xml_path),
                },
                "meshes": _mesh_manifest(mesh_dir),
            },
        }
        manifest["artifacts"]["bundle_sha256"] = _tree_sha256(
            temporary_dir, exclude=frozenset({"manifest.json"})
        )
        (temporary_dir / "manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        if output_dir.exists():
            shutil.rmtree(output_dir)
        os.replace(temporary_dir, output_dir)
    except Exception:
        shutil.rmtree(temporary_dir, ignore_errors=True)
        raise
    return CompileResult(
        xml_path=output_dir / "collision.xml",
        manifest_path=output_dir / "manifest.json",
        output_dir=output_dir,
    )


def _emit_object(
    item: CollisionObject,
    *,
    safe_name: str,
    asset: ET.Element,
    worldbody: ET.Element,
    mesh_dir: Path,
) -> dict[str, Any]:
    body = ET.SubElement(
        worldbody,
        "body",
        {
            "name": f"body_{safe_name}",
            "pos": _mjcf_vector(item.transform.translation_m),
            "quat": _mjcf_vector(item.transform.rotation_wxyz),
        },
    )
    geometry = item.geometry
    if geometry["type"] == "box":
        half_extents = _scale_vector(geometry["half_extents_m"], item.transform.scale)
        center = _scale_vector(geometry["center_m"], item.transform.scale)
        ET.SubElement(
            body,
            "geom",
            _collision_geom_attributes(
                name=f"collision_{safe_name}",
                geometry_type="box",
                extra={
                    "size": _mjcf_vector(half_extents),
                    "pos": _mjcf_vector(center),
                },
            ),
        )
        return {
            "id": item.object_id,
            "kind": item.kind,
            "enabled": True,
            "strategy": "analytic-box-v1",
            "mjcf_body": f"body_{safe_name}",
            "mjcf_geom": f"collision_{safe_name}",
            "transform": _manifest_transform(item.transform),
            "box": {
                "center_m": list(center),
                "half_extents_m": list(half_extents),
            },
        }
    scaled_vertices = [
        _scale_vector(vertex, item.transform.scale)
        for vertex in geometry["vertices_m"]
    ]
    bbox = _axis_aligned_bbox(scaled_vertices)
    mesh_vertices, mesh_faces = _bbox_mesh(bbox)
    mesh_name = f"mesh_{safe_name}"
    mesh_file = PurePosixPath(f"{safe_name}__aabb.obj")
    _write_obj(mesh_dir / mesh_file.name, mesh_vertices, mesh_faces)
    ET.SubElement(
        asset,
        "mesh",
        {
            "name": mesh_name,
            "file": mesh_file.as_posix(),
        },
    )
    ET.SubElement(
        body,
        "geom",
        _collision_geom_attributes(
            name=f"collision_{safe_name}",
            geometry_type="mesh",
            extra={"mesh": mesh_name},
        ),
    )
    return {
        "id": item.object_id,
        "kind": item.kind,
        "enabled": True,
        "strategy": "mesh-aabb-box-v1",
        "mjcf_body": f"body_{safe_name}",
        "mjcf_geom": f"collision_{safe_name}",
        "mjcf_mesh": mesh_name,
        "transform": _manifest_transform(item.transform),
        "source_mesh": {
            "vertices": len(geometry["vertices_m"]),
            "faces": len(geometry["faces"]),
        },
        "simplified_mesh": {
            "path": f"meshes/{mesh_file.name}",
            "vertices": len(mesh_vertices),
            "faces": len(mesh_faces),
            "aabb_min_m": list(bbox[0]),
            "aabb_max_m": list(bbox[1]),
        },
    }


def _collision_geom_attributes(
    *,
    name: str,
    geometry_type: str,
    extra: dict[str, str],
) -> dict[str, str]:
    return {
        "name": name,
        "type": geometry_type,
        "contype": "1",
        "conaffinity": "1",
        "friction": DEFAULT_FRICTION,
        "solref": DEFAULT_SOLREF,
        "solimp": DEFAULT_SOLIMP,
        "rgba": "0 0 0 0",
        **extra,
    }


def _manifest_skipped_object(item: CollisionObject) -> dict[str, Any]:
    return {
        "id": item.object_id,
        "kind": item.kind,
        "enabled": False,
        "strategy": "skipped-collision-disabled",
        "transform": _manifest_transform(item.transform),
    }


def _manifest_transform(transform: Transform) -> dict[str, Any]:
    return {
        "translation_m": list(transform.translation_m),
        "rotation_xyzw": list(transform.rotation_xyzw),
        "rotation_wxyz": list(transform.rotation_wxyz),
        "scale": list(transform.scale),
    }


def _mesh_manifest(mesh_dir: Path) -> list[dict[str, Any]]:
    return [
        {
            "path": f"meshes/{path.name}",
            "size_bytes": path.stat().st_size,
            "sha256": _file_sha256(path),
        }
        for path in sorted(mesh_dir.glob("*"))
        if path.is_file()
    ]


def smoke_mujoco_collision(
    xml_path: Path,
    *,
    require_mujoco: bool = False,
) -> dict[str, Any]:
    """Structurally validate the MJCF and run MuJoCo load/step when available."""

    xml_path = xml_path.resolve()
    try:
        root = ET.parse(xml_path).getroot()
    except ET.ParseError as exc:
        raise Town10CollisionCompileError(f"invalid MJCF XML {xml_path}: {exc}") from exc
    if root.tag != "mujoco":
        raise Town10CollisionCompileError(f"not a MuJoCo XML document: {xml_path}")
    geoms = list(root.iter("geom"))
    if not geoms:
        raise Town10CollisionCompileError("compiled collision MJCF has no geoms")
    colliding = [
        geom
        for geom in geoms
        if geom.get("contype", "1") != "0" and geom.get("conaffinity", "1") != "0"
    ]
    if not colliding:
        raise Town10CollisionCompileError(
            "compiled collision MJCF has no active collision geoms"
        )

    if importlib.util.find_spec("mujoco") is None:
        if require_mujoco:
            raise Town10CollisionCompileError("mujoco Python package is not installed")
        return {
            "schema": "matrix-town10-mujoco-collision-smoke/v1",
            "xml": str(xml_path),
            "structural": "passed",
            "mujoco": "skipped-not-installed",
            "geom_count": len(geoms),
            "active_collision_geom_count": len(colliding),
        }

    import mujoco  # type: ignore[import-not-found]

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    mujoco.mj_step(model, data)
    return {
        "schema": "matrix-town10-mujoco-collision-smoke/v1",
        "xml": str(xml_path),
        "structural": "passed",
        "mujoco": "passed",
        "geom_count": int(model.ngeom),
        "active_collision_geom_count": len(colliding),
    }


def _axis_aligned_bbox(
    vertices: Iterable[tuple[float, float, float]]
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    rows = list(vertices)
    if not rows:
        raise Town10CollisionCompileError("mesh has no vertices")
    minimum = tuple(min(row[axis] for row in rows) for axis in range(3))
    maximum = tuple(max(row[axis] for row in rows) for axis in range(3))
    if any(
        math.isclose(minimum[axis], maximum[axis], abs_tol=1e-12)
        for axis in range(3)
    ):
        raise Town10CollisionCompileError("mesh AABB is degenerate")
    return minimum, maximum


def _bbox_mesh(
    bbox: tuple[tuple[float, float, float], tuple[float, float, float]]
) -> tuple[tuple[tuple[float, float, float], ...], tuple[tuple[int, int, int], ...]]:
    (x0, y0, z0), (x1, y1, z1) = bbox
    vertices = (
        (x0, y0, z0),
        (x1, y0, z0),
        (x1, y1, z0),
        (x0, y1, z0),
        (x0, y0, z1),
        (x1, y0, z1),
        (x1, y1, z1),
        (x0, y1, z1),
    )
    faces = (
        (1, 2, 3),
        (1, 3, 4),
        (5, 8, 7),
        (5, 7, 6),
        (1, 5, 6),
        (1, 6, 2),
        (2, 6, 7),
        (2, 7, 3),
        (3, 7, 8),
        (3, 8, 4),
        (4, 8, 5),
        (4, 5, 1),
    )
    return vertices, faces


def _write_obj(
    path: Path,
    vertices: tuple[tuple[float, float, float], ...],
    faces: tuple[tuple[int, int, int], ...],
) -> None:
    lines = ["# matrix-town10 mesh-aabb-box-v1\n"]
    for vertex in vertices:
        lines.append("v " + _mjcf_vector(vertex) + "\n")
    for face in faces:
        lines.append("f " + " ".join(str(index) for index in face) + "\n")
    path.write_text("".join(lines), encoding="utf-8")


def _scale_vector(
    values: tuple[float, float, float],
    scale: tuple[float, float, float],
) -> tuple[float, float, float]:
    return tuple(values[index] * scale[index] for index in range(3))


def _safe_name(value: str) -> str:
    safe = SAFE_NAME_RE.sub("_", value).strip("_")
    if not safe:
        raise Town10CollisionCompileError(f"cannot derive safe name from {value!r}")
    if safe[0].isdigit():
        safe = f"n_{safe}"
    return safe[:96]


def _unique_name(value: str, used_names: set[str]) -> str:
    candidate = value
    index = 2
    while candidate in used_names:
        suffix = f"_{index}"
        candidate = f"{value[:96 - len(suffix)]}{suffix}"
        index += 1
    used_names.add(candidate)
    return candidate


def _mjcf_vector(values: Iterable[float]) -> str:
    return " ".join(f"{float(value):.12g}" for value in values)


def _normalize_quaternion_xyzw(
    values: tuple[float, float, float, float],
    label: str,
) -> tuple[float, float, float, float]:
    length = math.sqrt(sum(component * component for component in values))
    if length <= 0.0 or not math.isfinite(length):
        raise Town10CollisionCompileError(
            f"{label} must be a finite non-zero quaternion"
        )
    return tuple(component / length for component in values)


def _vector(value: Any, label: str, *, length: int) -> tuple[float, ...]:
    if not isinstance(value, list) and not isinstance(value, tuple):
        raise Town10CollisionCompileError(f"{label} must be a vector")
    if len(value) != length:
        raise Town10CollisionCompileError(f"{label} must have length {length}")
    return tuple(_finite_float(component, f"{label}[]") for component in value)


def _finite_float(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise Town10CollisionCompileError(f"{label} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise Town10CollisionCompileError(f"{label} must be a finite number") from exc
    if not math.isfinite(result):
        raise Town10CollisionCompileError(f"{label} must be finite")
    return result


def _optional_bool(value: Any, label: str) -> bool:
    if not isinstance(value, bool):
        raise Town10CollisionCompileError(f"{label} must be a boolean")
    return value


def _int(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise Town10CollisionCompileError(f"{label} must be an integer")
    return value


def _require_object(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise Town10CollisionCompileError(f"{label} must be an object")
    return value


def _optional_object(value: Any, label: str) -> dict[str, Any]:
    if value is None:
        return {}
    return _require_object(value, label)


def _require_list(value: Any, label: str) -> list[Any]:
    if not isinstance(value, list):
        raise Town10CollisionCompileError(f"{label} must be a list")
    return value


def _require_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise Town10CollisionCompileError(f"{label} must be {expected!r}")


def _require_id(value: Any, label: str) -> str:
    if not isinstance(value, str) or ID_RE.fullmatch(value) is None:
        raise Town10CollisionCompileError(f"{label} is not a valid Matrix id")
    return value


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--smoke", action="store_true")
    parser.add_argument("--require-mujoco", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    try:
        result = compile_town10_mujoco_collision(args.source, args.output_dir)
        print(f"[INFO] Town10 MuJoCo collision bundle ready: {result.xml_path}")
        if args.smoke:
            smoke = smoke_mujoco_collision(
                result.xml_path,
                require_mujoco=args.require_mujoco,
            )
            print(json.dumps(smoke, indent=2, sort_keys=True))
    except Town10CollisionCompileError as exc:
        raise SystemExit(f"[ERROR] {exc}") from exc
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
