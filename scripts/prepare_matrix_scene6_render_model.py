#!/usr/bin/env python3
"""Derive a self-contained Scene6 model with one unambiguous cube visual.

TwinBot's source model and trace remain the physics authority.  A live
shim-only replay proved that the packaged Matrix UE still did not show the
source ``pick_cube_visual`` primitive.  This tool therefore copies the pinned
robot mesh closure, makes only the source box's render alpha zero, and adds one
deterministic, massless, non-colliding 6 cm mesh visual to the same cube body.
The source box's size and mass contribution, collision box, free joint, poses,
inertials, joints, and actuators are unchanged.  Hiding the source render
surface prevents coincident visible surfaces and z-fighting.
"""

from __future__ import annotations

import argparse
import copy
import ctypes
from dataclasses import dataclass
import errno
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import stat
import struct
import sys
import tempfile
from typing import Any
import xml.etree.ElementTree as ET


RECEIPT_SCHEMA = "matrix.scene6_render_model_derivation.receipt.v1"
OPERATION = "add_zero_density_pick_cube_mesh_visual"
SCENE_BASENAME = "scene6_house_task.xml"
ROBOT_BASENAME = "g1_29dof_dex3.scene6.xml"
RECEIPT_BASENAME = "render-model-receipt.json"
OUTPUT_MESHDIR = "meshes"
TARGET_BODY = "pick_cube"
SOURCE_VISUAL = "pick_cube_visual"
SOURCE_COLLISION = "pick_cube_collision"
UE_VISUAL = "pick_cube_ue_visual"
UE_MESH_NAME = "pick_cube_ue_visual_mesh"
UE_MESH_FILE = "pick_cube_ue_visual.stl"
EXPECTED_SIZE = (0.03, 0.03, 0.03)
EXPECTED_RGBA = (0.95, 0.18, 0.05, 1.0)
UE_SCOPE_RGBA = (0.95, 0.18, 0.05, 0.99609375)
MAX_XML_BYTES = 64 * 1024 * 1024
MAX_MESH_BYTES = 256 * 1024 * 1024
AT_FDCWD = -100
RENAME_NOREPLACE = 1


class RenderModelError(ValueError):
    """Raised before a derived render model can be safely published."""


@dataclass(frozen=True)
class Artifact:
    path: Path
    sha256: str
    size_bytes: int
    device: int
    inode: int
    mtime_ns: int
    contents: bytes | None = None

    def binding(self) -> dict[str, Any]:
        return {
            "path": os.fspath(self.path),
            "sha256": self.sha256,
            "size_bytes": self.size_bytes,
        }


@dataclass(frozen=True)
class MeshSource:
    relative: Path
    artifact: Artifact


def _sha256(contents: bytes) -> str:
    return hashlib.sha256(contents).hexdigest()


def _open_regular(path: Path, *, label: str) -> tuple[int, os.stat_result, Path]:
    supplied = path.expanduser()
    if supplied.is_symlink():
        raise RenderModelError(f"{label} must not be a symlink: {supplied}")
    try:
        resolved = supplied.resolve(strict=True)
    except OSError as exc:
        raise RenderModelError(f"cannot resolve {label}: {supplied}: {exc}") from exc
    flags = os.O_RDONLY | os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        descriptor = os.open(resolved, flags)
    except OSError as exc:
        raise RenderModelError(f"cannot open {label}: {resolved}: {exc}") from exc
    metadata = os.fstat(descriptor)
    if not stat.S_ISREG(metadata.st_mode):
        os.close(descriptor)
        raise RenderModelError(f"{label} must be a regular file: {resolved}")
    return descriptor, metadata, resolved


def _read_regular(
    path: Path, *, label: str, max_bytes: int = MAX_XML_BYTES
) -> Artifact:
    descriptor, before, resolved = _open_regular(path, label=label)
    try:
        if before.st_size <= 0 or before.st_size > max_bytes:
            raise RenderModelError(
                f"{label} size must be in 1..{max_bytes}, got {before.st_size}"
            )
        blocks: list[bytes] = []
        remaining = before.st_size
        while remaining:
            block = os.read(descriptor, min(1024 * 1024, remaining))
            if not block:
                raise RenderModelError(f"{label} was truncated while being read")
            blocks.append(block)
            remaining -= len(block)
        if os.read(descriptor, 1):
            raise RenderModelError(f"{label} grew while being read")
        after = os.fstat(descriptor)
    finally:
        os.close(descriptor)
    _same_metadata(before, after, label=label)
    _same_path_metadata(resolved, after, label=label)
    contents = b"".join(blocks)
    return Artifact(
        path=resolved,
        contents=contents,
        sha256=_sha256(contents),
        size_bytes=len(contents),
        device=after.st_dev,
        inode=after.st_ino,
        mtime_ns=after.st_mtime_ns,
    )


def _hash_regular(path: Path, *, label: str) -> Artifact:
    descriptor, before, resolved = _open_regular(path, label=label)
    digest = hashlib.sha256()
    total = 0
    try:
        if before.st_size <= 0 or before.st_size > MAX_MESH_BYTES:
            raise RenderModelError(
                f"{label} size must be in 1..{MAX_MESH_BYTES}, got {before.st_size}"
            )
        while True:
            block = os.read(descriptor, 1024 * 1024)
            if not block:
                break
            total += len(block)
            if total > before.st_size:
                raise RenderModelError(f"{label} grew while being hashed")
            digest.update(block)
        after = os.fstat(descriptor)
    finally:
        os.close(descriptor)
    _same_metadata(before, after, label=label)
    _same_path_metadata(resolved, after, label=label)
    if total != after.st_size:
        raise RenderModelError(f"{label} was truncated while being hashed")
    return Artifact(
        path=resolved,
        contents=None,
        sha256=digest.hexdigest(),
        size_bytes=total,
        device=after.st_dev,
        inode=after.st_ino,
        mtime_ns=after.st_mtime_ns,
    )


def _same_metadata(
    before: os.stat_result, after: os.stat_result, *, label: str
) -> None:
    if (
        before.st_dev,
        before.st_ino,
        before.st_size,
        before.st_mtime_ns,
    ) != (
        after.st_dev,
        after.st_ino,
        after.st_size,
        after.st_mtime_ns,
    ):
        raise RenderModelError(f"{label} changed while being read")


def _same_path_metadata(path: Path, metadata: os.stat_result, *, label: str) -> None:
    current = os.stat(path, follow_symlinks=False)
    if (
        current.st_dev,
        current.st_ino,
        current.st_size,
        current.st_mtime_ns,
    ) != (
        metadata.st_dev,
        metadata.st_ino,
        metadata.st_size,
        metadata.st_mtime_ns,
    ):
        raise RenderModelError(f"{label} path changed while being read")


def _parse_xml(artifact: Artifact, *, label: str) -> ET.Element:
    assert artifact.contents is not None
    try:
        root = ET.fromstring(artifact.contents)
    except ET.ParseError as exc:
        raise RenderModelError(f"invalid {label} XML: {exc}") from exc
    if root.tag != "mujoco":
        raise RenderModelError(f"{label} root must be <mujoco>")
    return root


def _float_vector(value: str | None, *, field: str, length: int) -> tuple[float, ...]:
    if value is None:
        raise RenderModelError(f"{field} is required")
    try:
        parsed = tuple(float(item) for item in value.split())
    except ValueError as exc:
        raise RenderModelError(f"{field} must be numeric") from exc
    if len(parsed) != length:
        raise RenderModelError(f"{field} must contain {length} values")
    return parsed


def _safe_relative(value: str, *, label: str) -> Path:
    relative = Path(value)
    if relative.is_absolute() or not relative.parts or ".." in relative.parts:
        raise RenderModelError(f"{label} must be a confined relative path: {value}")
    if any(part in ("", ".") for part in relative.parts):
        raise RenderModelError(f"{label} has an empty path component: {value}")
    return relative


def _validate_scene(scene: Artifact) -> None:
    if scene.path.name != SCENE_BASENAME:
        raise RenderModelError(f"scene basename must be {SCENE_BASENAME}")
    root = _parse_xml(scene, label="source scene")
    includes = root.findall("include")
    if len(includes) != 1 or includes[0].get("file") != ROBOT_BASENAME:
        raise RenderModelError(
            f"source scene must include {ROBOT_BASENAME} exactly once"
        )
    if root.find('.//geom[@name="worktop"]') is None:
        raise RenderModelError("source scene is missing the Scene6 worktop")


def _validate_robot(robot: Artifact) -> tuple[ET.Element, ET.Element, ET.Element]:
    if robot.path.name != ROBOT_BASENAME:
        raise RenderModelError(f"robot basename must be {ROBOT_BASENAME}")
    root = _parse_xml(robot, label="source robot")
    if root.findall("include"):
        raise RenderModelError("source robot must be flattened")
    compiler = root.find("compiler")
    assets = root.find("asset")
    if compiler is None or not compiler.get("meshdir"):
        raise RenderModelError("source robot compiler.meshdir is required")
    if assets is None:
        raise RenderModelError("source robot asset section is required")
    unsupported_files = [
        element.tag
        for element in assets.iter()
        if element.tag != "mesh" and element.get("file")
    ]
    if unsupported_files:
        raise RenderModelError(
            "source robot has unsupported non-mesh file assets: "
            + ", ".join(sorted(set(unsupported_files)))
        )
    if assets.find(f'mesh[@name="{UE_MESH_NAME}"]') is not None:
        raise RenderModelError(f"source robot already contains {UE_MESH_NAME}")

    bodies = root.findall(f'.//body[@name="{TARGET_BODY}"]')
    if len(bodies) != 1:
        raise RenderModelError(f"source robot must contain one {TARGET_BODY} body")
    body = bodies[0]
    freejoints = [
        child
        for child in list(body)
        if child.tag == "freejoint"
        or (child.tag == "joint" and child.get("type") == "free")
    ]
    if len(freejoints) != 1:
        raise RenderModelError(f"{TARGET_BODY} must contain exactly one free joint")
    if body.find(f'geom[@name="{UE_VISUAL}"]') is not None:
        raise RenderModelError(f"source robot already contains {UE_VISUAL}")
    visuals = body.findall(f'geom[@name="{SOURCE_VISUAL}"]')
    collisions = body.findall(f'geom[@name="{SOURCE_COLLISION}"]')
    if len(visuals) != 1 or len(collisions) != 1:
        raise RenderModelError(
            "pick_cube must contain exactly one named source visual and collision"
        )
    visual, collision = visuals[0], collisions[0]
    if visual.get("type") != "box" or collision.get("type") != "box":
        raise RenderModelError("source pick_cube geoms must remain boxes")
    if _float_vector(
        visual.get("size"), field="pick_cube_visual.size", length=3
    ) != EXPECTED_SIZE:
        raise RenderModelError("pick_cube_visual size differs from validated v2")
    if _float_vector(
        collision.get("size"), field="pick_cube_collision.size", length=3
    ) != EXPECTED_SIZE:
        raise RenderModelError("pick_cube collision size differs from visual")
    if _float_vector(
        visual.get("rgba"), field="pick_cube_visual.rgba", length=4
    ) != EXPECTED_RGBA:
        raise RenderModelError("pick_cube_visual RGBA differs from validated red")
    if visual.get("contype") != "0" or visual.get("conaffinity") != "0":
        raise RenderModelError("source pick_cube visual must remain non-colliding")
    collision_rgba = _float_vector(
        collision.get("rgba"), field="pick_cube_collision.rgba", length=4
    )
    if collision_rgba[3] != 0.0:
        raise RenderModelError("source pick_cube collision must remain transparent")
    return root, compiler, body


def _mesh_root(robot: Artifact, compiler: ET.Element) -> Path:
    configured = Path(str(compiler.get("meshdir"))).expanduser()
    candidate = (
        configured if configured.is_absolute() else robot.path.parent / configured
    )
    if candidate.is_symlink():
        raise RenderModelError(f"source meshdir must not be a symlink: {candidate}")
    try:
        resolved = candidate.resolve(strict=True)
    except OSError as exc:
        raise RenderModelError(f"cannot resolve source meshdir: {candidate}") from exc
    if not resolved.is_dir():
        raise RenderModelError(f"source meshdir is not a directory: {resolved}")
    return resolved


def _mesh_inventory(
    robot: Artifact, root: ET.Element, compiler: ET.Element
) -> tuple[Path, list[MeshSource]]:
    mesh_root = _mesh_root(robot, compiler)
    assets = root.find("asset")
    assert assets is not None
    mesh_elements = list(assets.iter("mesh"))
    if not mesh_elements:
        raise RenderModelError("source robot has no mesh assets")
    names: set[str] = set()
    relatives: set[Path] = set()
    inventory: list[MeshSource] = []
    for index, element in enumerate(mesh_elements):
        name = element.get("name")
        file_name = element.get("file")
        if not name or name in names:
            raise RenderModelError(f"mesh asset {index} has a missing/duplicate name")
        if not file_name:
            raise RenderModelError(f"mesh asset {index} has no file")
        names.add(name)
        relative = _safe_relative(file_name, label=f"mesh asset {index}")
        if relative in relatives:
            continue
        relatives.add(relative)
        source = mesh_root / relative
        if source.is_symlink():
            raise RenderModelError(f"mesh asset must not be a symlink: {source}")
        resolved = source.resolve()
        try:
            resolved.relative_to(mesh_root)
        except ValueError as exc:
            raise RenderModelError(f"mesh asset escapes meshdir: {file_name}") from exc
        inventory.append(
            MeshSource(
                relative=relative,
                artifact=_hash_regular(resolved, label=f"mesh asset {index}"),
            )
        )
    generated_relative = Path(UE_MESH_FILE)
    if generated_relative in relatives:
        raise RenderModelError(f"generated mesh path already exists: {UE_MESH_FILE}")
    inventory.sort(key=lambda item: item.relative.as_posix())
    return mesh_root, inventory


def _cube_stl() -> bytes:
    half = 0.03
    triangles = (
        ((1.0, 0.0, 0.0), ((half, -half, -half), (half, half, -half), (half, half, half))),
        ((1.0, 0.0, 0.0), ((half, -half, -half), (half, half, half), (half, -half, half))),
        ((-1.0, 0.0, 0.0), ((-half, -half, -half), (-half, -half, half), (-half, half, half))),
        ((-1.0, 0.0, 0.0), ((-half, -half, -half), (-half, half, half), (-half, half, -half))),
        ((0.0, 1.0, 0.0), ((-half, half, -half), (-half, half, half), (half, half, half))),
        ((0.0, 1.0, 0.0), ((-half, half, -half), (half, half, half), (half, half, -half))),
        ((0.0, -1.0, 0.0), ((-half, -half, -half), (half, -half, -half), (half, -half, half))),
        ((0.0, -1.0, 0.0), ((-half, -half, -half), (half, -half, half), (-half, -half, half))),
        ((0.0, 0.0, 1.0), ((-half, -half, half), (half, -half, half), (half, half, half))),
        ((0.0, 0.0, 1.0), ((-half, -half, half), (half, half, half), (-half, half, half))),
        ((0.0, 0.0, -1.0), ((-half, -half, -half), (-half, half, -half), (half, half, -half))),
        ((0.0, 0.0, -1.0), ((-half, -half, -half), (half, half, -half), (half, -half, -half))),
    )
    header = b"Matrix Scene6 pick cube mesh v1".ljust(80, b"\0")
    payload = bytearray(header)
    payload.extend(struct.pack("<I", len(triangles)))
    for normal, vertices in triangles:
        values = (*normal, *vertices[0], *vertices[1], *vertices[2])
        payload.extend(struct.pack("<12fH", *values, 0))
    result = bytes(payload)
    if len(result) != 684:
        raise RenderModelError("internal cube STL size drifted")
    return result


def _derive_robot(source_root: ET.Element) -> bytes:
    root = copy.deepcopy(source_root)
    derived_compiler = root.find("compiler")
    assets = root.find("asset")
    derived_bodies = root.findall(f'.//body[@name="{TARGET_BODY}"]')
    if derived_compiler is None or assets is None or len(derived_bodies) != 1:
        raise RenderModelError("internal robot derivation lost required elements")
    derived_compiler.set("meshdir", OUTPUT_MESHDIR)
    ET.SubElement(
        assets,
        "mesh",
        {"name": UE_MESH_NAME, "file": UE_MESH_FILE},
    )
    derived_body = derived_bodies[0]
    source_visual = derived_body.find(f'geom[@name="{SOURCE_VISUAL}"]')
    if source_visual is None:
        raise RenderModelError("internal robot derivation lost source visual")
    source_visual.set("rgba", "0.95 0.18 0.05 0")
    insertion_index = list(derived_body).index(source_visual) + 1
    derived_body.insert(
        insertion_index,
        ET.Element(
            "geom",
            {
                "name": UE_VISUAL,
                "type": "mesh",
                "mesh": UE_MESH_NAME,
                "rgba": "0.95 0.18 0.05 0.99609375",
                "contype": "0",
                "conaffinity": "0",
                "mass": "0",
                "group": "1",
            },
        ),
    )
    ET.indent(root, space="  ")
    result = ET.tostring(root, encoding="utf-8", xml_declaration=False) + b"\n"

    try:
        check = ET.fromstring(result)
    except ET.ParseError as exc:
        raise RenderModelError(f"derived robot XML is invalid: {exc}") from exc
    added_meshes = check.findall(f'.//asset/mesh[@name="{UE_MESH_NAME}"]')
    added_geoms = check.findall(
        f'.//body[@name="{TARGET_BODY}"]/geom[@name="{UE_VISUAL}"]'
    )
    if len(added_meshes) != 1 or len(added_geoms) != 1:
        raise RenderModelError("derived robot lacks the unique UE cube visual")
    added = added_geoms[0]
    required = {
        "type": "mesh",
        "mesh": UE_MESH_NAME,
        "rgba": "0.95 0.18 0.05 0.99609375",
        "contype": "0",
        "conaffinity": "0",
        "mass": "0",
        "group": "1",
    }
    if any(added.get(key) != value for key, value in required.items()):
        raise RenderModelError("derived UE cube visual attributes drifted")
    hidden_source = check.find(
        f'.//body[@name="{TARGET_BODY}"]/geom[@name="{SOURCE_VISUAL}"]'
    )
    if hidden_source is None or hidden_source.get("rgba") != "0.95 0.18 0.05 0":
        raise RenderModelError("derived source cube visual was not render-hidden")
    if (
        hidden_source.get("type") != "box"
        or hidden_source.get("size") != "0.03 0.03 0.03"
        or hidden_source.get("contype") != "0"
        or hidden_source.get("conaffinity") != "0"
    ):
        raise RenderModelError("derived source cube physics fields drifted")
    derived_compiler_check = check.find("compiler")
    if (
        derived_compiler_check is None
        or derived_compiler_check.get("meshdir") != OUTPUT_MESHDIR
    ):
        raise RenderModelError("derived compiler.meshdir did not become local")
    return result


def _validate_expected_hash(
    artifact: Artifact, expected: str | None, *, label: str
) -> None:
    if expected is None:
        return
    if not re.fullmatch(r"[0-9a-f]{64}", expected):
        raise RenderModelError(f"expected {label} SHA256 is invalid")
    if artifact.sha256 != expected:
        raise RenderModelError(
            f"{label} SHA256 mismatch: expected {expected}, got {artifact.sha256}"
        )


def _write_file(path: Path, contents: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor = os.open(
        path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC, 0o644
    )
    try:
        view = memoryview(contents)
        while view:
            written = os.write(descriptor, view)
            if written <= 0:
                raise OSError("short write")
            view = view[written:]
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _copy_mesh(source: MeshSource, destination: Path) -> None:
    artifact = source.artifact
    destination.parent.mkdir(parents=True, exist_ok=True)
    source_fd, before, resolved = _open_regular(
        artifact.path, label=f"mesh {source.relative.as_posix()}"
    )
    destination_fd = os.open(
        destination,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC,
        0o644,
    )
    digest = hashlib.sha256()
    total = 0
    try:
        while True:
            block = os.read(source_fd, 1024 * 1024)
            if not block:
                break
            digest.update(block)
            total += len(block)
            view = memoryview(block)
            while view:
                written = os.write(destination_fd, view)
                if written <= 0:
                    raise OSError("short mesh write")
                view = view[written:]
        after = os.fstat(source_fd)
        os.fsync(destination_fd)
    finally:
        os.close(destination_fd)
        os.close(source_fd)
    _same_metadata(before, after, label=f"mesh {source.relative.as_posix()}")
    _same_path_metadata(
        resolved, after, label=f"mesh {source.relative.as_posix()}"
    )
    if (
        before.st_dev != artifact.device
        or before.st_ino != artifact.inode
        or before.st_mtime_ns != artifact.mtime_ns
        or total != artifact.size_bytes
        or digest.hexdigest() != artifact.sha256
    ):
        raise RenderModelError(
            f"mesh changed before publication: {source.relative.as_posix()}"
        )


def _closure_digest(entries: list[dict[str, Any]]) -> str:
    digest = hashlib.sha256()
    for entry in sorted(entries, key=lambda item: str(item["relative_path"])):
        digest.update(str(entry["relative_path"]).encode("utf-8"))
        digest.update(b"\0")
        digest.update(str(entry["sha256"]).encode("ascii"))
        digest.update(b"\n")
    return digest.hexdigest()


def _json_bytes(payload: dict[str, Any]) -> bytes:
    return (
        json.dumps(
            payload,
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            indent=2,
        )
        + "\n"
    ).encode("utf-8")


def _fsync_directories(root: Path) -> None:
    directories = [root, *(path for path in root.rglob("*") if path.is_dir())]
    for directory in sorted(
        directories, key=lambda path: len(path.parts), reverse=True
    ):
        flags = os.O_RDONLY | os.O_CLOEXEC
        if hasattr(os, "O_DIRECTORY"):
            flags |= os.O_DIRECTORY
        descriptor = os.open(directory, flags)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)


def _rename_noreplace(source: Path, destination: Path) -> None:
    libc = ctypes.CDLL(None, use_errno=True)
    renameat2 = getattr(libc, "renameat2", None)
    if renameat2 is None:
        raise RenderModelError("renameat2(RENAME_NOREPLACE) is unavailable")
    renameat2.argtypes = [
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_uint,
    ]
    renameat2.restype = ctypes.c_int
    result = renameat2(
        AT_FDCWD,
        os.fsencode(source),
        AT_FDCWD,
        os.fsencode(destination),
        RENAME_NOREPLACE,
    )
    if result == 0:
        return
    error = ctypes.get_errno()
    if error in (errno.EEXIST, errno.ENOTEMPTY):
        raise RenderModelError(
            f"output directory appeared before publication: {destination}"
        )
    raise RenderModelError(
        f"atomic no-replace publication failed: {os.strerror(error)}"
    )


def _assert_xml_unchanged(artifact: Artifact, *, label: str) -> None:
    current = _read_regular(artifact.path, label=label)
    if (
        current.device,
        current.inode,
        current.mtime_ns,
        current.size_bytes,
        current.sha256,
    ) != (
        artifact.device,
        artifact.inode,
        artifact.mtime_ns,
        artifact.size_bytes,
        artifact.sha256,
    ):
        raise RenderModelError(f"{label} changed before publication")


def derive_render_model(
    *,
    source_scene_path: Path,
    source_robot_path: Path,
    output_dir: Path,
    expected_scene_sha256: str | None = None,
    expected_robot_sha256: str | None = None,
    expected_mesh_closure_sha256: str | None = None,
) -> dict[str, Any]:
    """Validate inputs and atomically publish an isolated render-model bundle."""

    scene = _read_regular(source_scene_path, label="source scene")
    robot = _read_regular(source_robot_path, label="source robot")
    if scene.path == robot.path:
        raise RenderModelError("source scene and robot must be distinct files")
    _validate_expected_hash(scene, expected_scene_sha256, label="source scene")
    _validate_expected_hash(robot, expected_robot_sha256, label="source robot")
    _validate_scene(scene)
    source_root, compiler, _body = _validate_robot(robot)
    mesh_root, mesh_sources = _mesh_inventory(robot, source_root, compiler)
    generated_mesh = _cube_stl()
    derived_robot = _derive_robot(source_root)

    supplied_output = output_dir.expanduser()
    if supplied_output.exists() or supplied_output.is_symlink():
        raise RenderModelError(f"output directory already exists: {supplied_output}")
    if not supplied_output.name or supplied_output.name in (".", ".."):
        raise RenderModelError("output directory must have a basename")
    try:
        parent = supplied_output.parent.resolve(strict=True)
    except OSError as exc:
        raise RenderModelError(
            f"output parent must already exist: {supplied_output.parent}"
        ) from exc
    if not parent.is_dir():
        raise RenderModelError(f"output parent is not a directory: {parent}")
    published_dir = parent / supplied_output.name
    if published_dir.exists() or published_dir.is_symlink():
        raise RenderModelError(f"output directory already exists: {published_dir}")

    output_scene = published_dir / SCENE_BASENAME
    output_robot = published_dir / ROBOT_BASENAME
    output_receipt = published_dir / RECEIPT_BASENAME
    output_meshdir = published_dir / OUTPUT_MESHDIR
    mesh_entries = [
        {
            "relative_path": item.relative.as_posix(),
            "sha256": item.artifact.sha256,
            "size_bytes": item.artifact.size_bytes,
            "source_path": os.fspath(item.artifact.path),
            "generated": False,
        }
        for item in mesh_sources
    ]
    mesh_entries.append(
        {
            "relative_path": UE_MESH_FILE,
            "sha256": _sha256(generated_mesh),
            "size_bytes": len(generated_mesh),
            "source_path": None,
            "generated": True,
        }
    )
    mesh_entries.sort(key=lambda item: str(item["relative_path"]))
    source_mesh_entries = [item for item in mesh_entries if not item["generated"]]
    source_mesh_closure_sha256 = _closure_digest(source_mesh_entries)
    if expected_mesh_closure_sha256 is not None:
        if not re.fullmatch(r"[0-9a-f]{64}", expected_mesh_closure_sha256):
            raise RenderModelError("expected source mesh closure SHA256 is invalid")
        if source_mesh_closure_sha256 != expected_mesh_closure_sha256:
            raise RenderModelError(
                "source mesh closure SHA256 mismatch: expected "
                f"{expected_mesh_closure_sha256}, got {source_mesh_closure_sha256}"
            )
    receipt = {
        "schema_id": RECEIPT_SCHEMA,
        "passed": True,
        "operation": OPERATION,
        "physics_authority": "unchanged_twinbot_source_trace_and_physics_fields",
        "render_purpose": "make_pick_cube_visible_in_packaged_matrix_ue",
        "inputs": {
            "scene_model": scene.binding(),
            "render_robot_model": robot.binding(),
            "mesh_root": os.fspath(mesh_root),
            "mesh_closure": {
                "file_count": len(source_mesh_entries),
                "size_bytes": sum(
                    int(item["size_bytes"]) for item in source_mesh_entries
                ),
                "sha256": source_mesh_closure_sha256,
            },
        },
        "outputs": {
            "scene_model": {
                "path": os.fspath(output_scene),
                "sha256": scene.sha256,
                "size_bytes": scene.size_bytes,
            },
            "render_robot_model": {
                "path": os.fspath(output_robot),
                "sha256": _sha256(derived_robot),
                "size_bytes": len(derived_robot),
            },
            "mesh_closure": {
                "path": os.fspath(output_meshdir),
                "file_count": len(mesh_entries),
                "size_bytes": sum(int(item["size_bytes"]) for item in mesh_entries),
                "sha256": _closure_digest(mesh_entries),
                "files": mesh_entries,
            },
        },
        "render_addition": {
            "body": TARGET_BODY,
            "geom": UE_VISUAL,
            "mesh": UE_MESH_NAME,
            "mesh_file": UE_MESH_FILE,
            "dimensions_m": [0.06, 0.06, 0.06],
            "triangle_count": 12,
            "rgba": list(UE_SCOPE_RGBA),
            "group": 1,
            "mass": 0,
            "contype": 0,
            "conaffinity": 0,
        },
        "render_suppression": {
            "body": TARGET_BODY,
            "geom": SOURCE_VISUAL,
            "attribute": "rgba_alpha",
            "before": 1,
            "after": 0,
            "physics_effect": "none",
        },
        "invariants": {
            "scene_bytes_preserved": True,
            "source_visual_box_physics_fields_preserved": True,
            "source_visual_render_alpha_zeroed": True,
            "source_collision_box_preserved": True,
            "source_free_joint_preserved": True,
            "source_pose_inertial_joint_actuator_fields_preserved": True,
            "only_compiler_path_rebound_to_identical_mesh_copies": True,
            "added_geom_is_massless_and_noncolliding": True,
            "trace_frames_changed": False,
        },
    }
    receipt_bytes = _json_bytes(receipt)

    temporary = Path(tempfile.mkdtemp(prefix=f".{published_dir.name}.", dir=parent))
    published = False
    try:
        _write_file(temporary / SCENE_BASENAME, scene.contents or b"")
        _write_file(temporary / ROBOT_BASENAME, derived_robot)
        for source in mesh_sources:
            _copy_mesh(source, temporary / OUTPUT_MESHDIR / source.relative)
        _write_file(temporary / OUTPUT_MESHDIR / UE_MESH_FILE, generated_mesh)
        _write_file(temporary / RECEIPT_BASENAME, receipt_bytes)

        staged_scene = _read_regular(temporary / SCENE_BASENAME, label="staged scene")
        staged_robot = _read_regular(temporary / ROBOT_BASENAME, label="staged robot")
        staged_receipt = _read_regular(
            temporary / RECEIPT_BASENAME, label="staged receipt"
        )
        if staged_scene.sha256 != scene.sha256:
            raise RenderModelError("staged scene bytes drifted")
        if staged_robot.sha256 != _sha256(derived_robot):
            raise RenderModelError("staged render robot bytes drifted")
        if staged_receipt.contents != receipt_bytes:
            raise RenderModelError("staged receipt bytes drifted")
        for entry in mesh_entries:
            staged_mesh = _hash_regular(
                temporary / OUTPUT_MESHDIR / str(entry["relative_path"]),
                label=f"staged mesh {entry['relative_path']}",
            )
            if (
                staged_mesh.sha256 != entry["sha256"]
                or staged_mesh.size_bytes != entry["size_bytes"]
            ):
                raise RenderModelError(
                    f"staged mesh drifted: {entry['relative_path']}"
                )
        _assert_xml_unchanged(scene, label="source scene")
        _assert_xml_unchanged(robot, label="source robot")
        if published_dir.exists() or published_dir.is_symlink():
            raise RenderModelError("output directory appeared before publication")
        _fsync_directories(temporary)
        _rename_noreplace(temporary, published_dir)
        published = True
        parent_fd = os.open(parent, os.O_RDONLY | os.O_CLOEXEC)
        try:
            os.fsync(parent_fd)
        finally:
            os.close(parent_fd)
    finally:
        if not published:
            shutil.rmtree(temporary, ignore_errors=True)

    return receipt


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-scene", type=Path, required=True)
    parser.add_argument("--source-robot-model", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--expected-source-scene-sha256")
    parser.add_argument("--expected-source-robot-model-sha256")
    parser.add_argument("--expected-source-mesh-closure-sha256", required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        receipt = derive_render_model(
            source_scene_path=args.source_scene,
            source_robot_path=args.source_robot_model,
            output_dir=args.output_dir,
            expected_scene_sha256=args.expected_source_scene_sha256,
            expected_robot_sha256=args.expected_source_robot_model_sha256,
            expected_mesh_closure_sha256=args.expected_source_mesh_closure_sha256,
        )
    except (OSError, RenderModelError) as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 2

    receipt_path = (
        Path(receipt["outputs"]["scene_model"]["path"]).parent
        / RECEIPT_BASENAME
    )
    print(
        json.dumps(
            {
                "passed": True,
                "operation": OPERATION,
                "scene_model": receipt["outputs"]["scene_model"],
                "render_robot_model": receipt["outputs"]["render_robot_model"],
                "mesh_closure": {
                    key: receipt["outputs"]["mesh_closure"][key]
                    for key in ("path", "file_count", "size_bytes", "sha256")
                },
                "receipt": {
                    "path": os.fspath(receipt_path),
                    "sha256": _sha256(receipt_path.read_bytes()),
                },
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
