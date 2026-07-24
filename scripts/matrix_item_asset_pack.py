#!/usr/bin/env python3
"""Strict, content-addressed Matrix item asset-pack manifests.

This module deliberately performs no asset conversion.  It verifies immutable
pack files and returns frozen DTOs that a renderer or the existing MuJoCo
creative-inventory injector can adapt explicitly.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import stat
import sys
from typing import Any, Iterable
from urllib.parse import urlparse


ASSET_PACK_SCHEMA = "matrix-item-asset-pack/v1"
INVENTORY_SCHEMA = "matrix-item-inventory/v1"
CANONICALIZATION = "matrix_item_asset_pack_v1_canonical_json_sha256"
REGISTRY_MANIFEST_NAME = "matrix-item-asset-pack.json"
MAX_MANIFEST_BYTES = 2 * 1024 * 1024
MAX_ASSET_BYTES = 512 * 1024 * 1024
MAX_FILES = 512
MAX_ITEMS = 256
MAX_VISUAL_PARTS = 64
MAX_INVENTORY_ENTRIES = 512
MAX_POOL_SIZE = 32

ID_RE = re.compile(r"^[a-z0-9][a-z0-9_.-]{0,127}$")
REVISION_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._+-]{0,127}$")
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
DIGEST_REF_RE = re.compile(r"^sha256:([0-9a-f]{64})$")
SPDX_ID_RE = re.compile(
    r"^(?:LicenseRef-[A-Za-z0-9][A-Za-z0-9.-]{0,127}|"
    r"(?!LicenseRef-)[A-Za-z0-9][A-Za-z0-9.+-]{0,127})$"
)
AXES = frozenset({"+X", "-X", "+Y", "-Y", "+Z", "-Z"})
HANDEDNESSES = frozenset({"left", "right"})
VISUAL_FORMAT_SUFFIXES = {
    "stl": (".stl",),
    "obj": (".obj",),
    "glb": (".glb",),
    "gltf": (".gltf",),
    "fbx": (".fbx",),
    "usd": (".usd",),
    "usda": (".usda",),
    "usdc": (".usdc",),
}
SUPPORTED_VISUAL_FORMATS = frozenset(VISUAL_FORMAT_SUFFIXES)
FILE_FORMAT_SUFFIXES = {
    **VISUAL_FORMAT_SUFFIXES,
    "text": (".txt",),
    "markdown": (".md",),
    "mtl": (".mtl",),
    "png": (".png",),
    "jpeg": (".jpg", ".jpeg"),
    "json": (".json",),
    "binary": (".bin",),
}
SUPPORTED_FILE_FORMATS = frozenset(FILE_FORMAT_SUFFIXES)
FILE_ROLES = frozenset(
    {
        "visual_mesh",
        "license",
        "provenance",
        "material",
        "texture",
        "auxiliary",
    }
)
MEDIA_TYPE_RE = re.compile(
    r"^[a-z0-9][a-z0-9!#$&^_.+-]{0,63}/"
    r"[a-z0-9][a-z0-9!#$&^_.+-]{0,127}$"
)


class ItemAssetPackError(ValueError):
    """Base class for fail-closed item asset-pack errors."""


class ItemAssetValidationError(ItemAssetPackError):
    pass


class ItemAssetIOError(ItemAssetPackError):
    pass


class ItemAssetVerificationError(ItemAssetPackError):
    pass


class ItemInventoryResolutionError(ItemAssetPackError):
    pass


@dataclass(frozen=True)
class LicenseMetadata:
    spdx_id: str
    attribution: str


@dataclass(frozen=True)
class Provenance:
    source_name: str
    source_uri: str
    source_revision: str
    source_item_ids: tuple[str, ...]


@dataclass(frozen=True)
class CoordinateFrame:
    up_axis: str
    forward_axis: str
    handedness: str
    meters_per_unit: float


@dataclass(frozen=True)
class VerifiedAssetFile:
    file_id: str
    relative_path: PurePosixPath
    path: Path
    size_bytes: int
    sha256: str
    role: str
    media_type: str
    format: str


@dataclass(frozen=True)
class BoxCollision:
    half_extents_m: tuple[float, float, float]


@dataclass(frozen=True)
class PhysicsDefinition:
    mass_kg: float
    collision: BoxCollision


@dataclass(frozen=True)
class VisualPart:
    part_id: str
    asset_file: VerifiedAssetFile
    rgba: tuple[float, float, float, float]
    scale: tuple[float, float, float]
    translation_m: tuple[float, float, float]
    rotation_wxyz: tuple[float, float, float, float]


@dataclass(frozen=True)
class ItemDefinition:
    item_id: str
    label: str
    physics: PhysicsDefinition
    visual_parts: tuple[VisualPart, ...]


@dataclass(frozen=True)
class AssetPack:
    manifest_path: Path
    digest: str
    pack_id: str
    revision: str
    license: LicenseMetadata
    provenance: Provenance
    coordinate_frame: CoordinateFrame
    files: tuple[VerifiedAssetFile, ...]
    items: tuple[ItemDefinition, ...]

    @property
    def digest_ref(self) -> str:
        return f"sha256:{self.digest}"

    def item(self, item_id: str) -> ItemDefinition:
        for item in self.items:
            if item.item_id == item_id:
                return item
        raise ItemInventoryResolutionError(
            f"pack {self.digest_ref} has no item {item_id!r}"
        )


@dataclass(frozen=True)
class SpawnDefinition:
    distance_m: float
    height_m: float
    quaternion_wxyz: tuple[float, float, float, float]


@dataclass(frozen=True)
class InventoryEntry:
    slot_id: str
    pack_digest: str
    item_id: str
    pool_size: int
    spawn: SpawnDefinition


@dataclass(frozen=True)
class ItemInventory:
    manifest_path: Path
    inventory_id: str
    entries: tuple[InventoryEntry, ...]


@dataclass(frozen=True)
class ResolvedInventoryItem:
    slot_id: str
    pack: AssetPack
    item: ItemDefinition
    pool_size: int
    spawn: SpawnDefinition


@dataclass(frozen=True)
class ResolvedInventory:
    inventory: ItemInventory
    items: tuple[ResolvedInventoryItem, ...]

    def legacy_injector_specs(self) -> tuple["LegacyInjectorItemSpec", ...]:
        """Return explicit, lossless specs for the current STL-only injector.

        Non-canonical coordinate frames, transformed visual parts, and formats
        other than STL fail closed.  An adapter must handle those semantics
        instead of silently discarding them or transcoding assets.
        """

        specs: list[LegacyInjectorItemSpec] = []
        for resolved in self.items:
            frame = resolved.pack.coordinate_frame
            if frame != CoordinateFrame(
                up_axis="+Z",
                forward_axis="+X",
                handedness="right",
                meters_per_unit=1.0,
            ):
                raise ItemInventoryResolutionError(
                    f"slot {resolved.slot_id!r} needs a coordinate-frame adapter"
                )
            visuals: list[LegacyInjectorVisualSpec] = []
            for part in resolved.item.visual_parts:
                if part.asset_file.format != "stl":
                    raise ItemInventoryResolutionError(
                        f"slot {resolved.slot_id!r} visual {part.part_id!r} "
                        f"uses unsupported legacy injector format "
                        f"{part.asset_file.format!r}"
                    )
                if part.translation_m != (0.0, 0.0, 0.0) or part.rotation_wxyz != (
                    1.0,
                    0.0,
                    0.0,
                    0.0,
                ):
                    raise ItemInventoryResolutionError(
                        f"slot {resolved.slot_id!r} visual {part.part_id!r} "
                        "needs a transform-aware adapter"
                    )
                visuals.append(
                    LegacyInjectorVisualSpec(
                        mesh=part.asset_file.path,
                        rgba=part.rgba,
                        scale=part.scale,
                    )
                )
            specs.append(
                LegacyInjectorItemSpec(
                    item_id=resolved.slot_id,
                    label=resolved.item.label,
                    pool_size=resolved.pool_size,
                    mass_kg=resolved.item.physics.mass_kg,
                    collision_half_size=(
                        resolved.item.physics.collision.half_extents_m
                    ),
                    spawn_distance_m=resolved.spawn.distance_m,
                    spawn_height_m=resolved.spawn.height_m,
                    spawn_quat=resolved.spawn.quaternion_wxyz,
                    visuals=tuple(visuals),
                )
            )
        return tuple(specs)


@dataclass(frozen=True)
class LegacyInjectorVisualSpec:
    mesh: Path
    rgba: tuple[float, float, float, float]
    scale: tuple[float, float, float]


@dataclass(frozen=True)
class LegacyInjectorItemSpec:
    item_id: str
    label: str
    pool_size: int
    mass_kg: float
    collision_half_size: tuple[float, float, float]
    spawn_distance_m: float
    spawn_height_m: float
    spawn_quat: tuple[float, float, float, float]
    visuals: tuple[LegacyInjectorVisualSpec, ...]


def loads_json_strict(text: str, *, source: str = "<json>") -> Any:
    def reject_duplicates(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise ItemAssetValidationError(
                    f"{source}: duplicate key {key!r}"
                )
            result[key] = value
        return result

    def reject_constant(token: str) -> None:
        raise ItemAssetValidationError(
            f"{source}: non-finite JSON number {token}"
        )

    try:
        return json.loads(
            text,
            object_pairs_hook=reject_duplicates,
            parse_constant=reject_constant,
        )
    except ItemAssetValidationError:
        raise
    except (json.JSONDecodeError, ValueError) as exc:
        detail = exc.msg if isinstance(exc, json.JSONDecodeError) else str(exc)
        raise ItemAssetValidationError(
            f"{source}: invalid JSON: {detail}"
        ) from exc


def canonical_asset_pack_bytes(document: Any) -> bytes:
    validated = _validate_asset_pack_document(document)
    return json.dumps(
        validated,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")


def asset_pack_digest(document: Any) -> str:
    return hashlib.sha256(canonical_asset_pack_bytes(document)).hexdigest()


def load_asset_pack(manifest_path: Path) -> AssetPack:
    manifest_path = _absolute_path(manifest_path)
    _require_plain_directory(manifest_path.parent, "pack directory")
    raw_bytes = _read_regular_file(
        manifest_path,
        maximum_bytes=MAX_MANIFEST_BYTES,
        description="asset-pack manifest",
    )
    try:
        text = raw_bytes.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ItemAssetValidationError(
            f"{manifest_path}: manifest is not UTF-8"
        ) from exc
    document = loads_json_strict(text, source=str(manifest_path))
    validated = _validate_asset_pack_document(document)
    digest = asset_pack_digest(validated)
    pack = validated["pack"]

    files_by_id: dict[str, VerifiedAssetFile] = {}
    verified_files: list[VerifiedAssetFile] = []
    for file_document in pack["files"]:
        relative = _safe_relative_path(
            file_document["path"],
            f"file {file_document['file_id']!r} path",
        )
        path = _asset_path_without_symlinks(manifest_path.parent, relative)
        actual_size, actual_sha256 = _verify_regular_file(
            path,
            maximum_bytes=MAX_ASSET_BYTES,
            description=f"asset file {file_document['file_id']!r}",
            expected_size=file_document["size_bytes"],
            expected_sha256=file_document["sha256"],
        )
        verified = VerifiedAssetFile(
            file_id=file_document["file_id"],
            relative_path=relative,
            path=path,
            size_bytes=actual_size,
            sha256=actual_sha256,
            role=file_document["role"],
            media_type=file_document["media_type"],
            format=file_document["format"],
        )
        files_by_id[verified.file_id] = verified
        verified_files.append(verified)

    items: list[ItemDefinition] = []
    for item_document in pack["items"]:
        physics_document = item_document["physics"]
        collision_document = physics_document["collision"]
        parts: list[VisualPart] = []
        for part_document in item_document["visual_parts"]:
            asset_file = files_by_id[part_document["file_id"]]
            parts.append(
                VisualPart(
                    part_id=part_document["part_id"],
                    asset_file=asset_file,
                    rgba=tuple(part_document["rgba"]),
                    scale=tuple(part_document["scale"]),
                    translation_m=tuple(part_document["translation_m"]),
                    rotation_wxyz=tuple(part_document["rotation_wxyz"]),
                )
            )
        items.append(
            ItemDefinition(
                item_id=item_document["item_id"],
                label=item_document["label"],
                physics=PhysicsDefinition(
                    mass_kg=physics_document["mass_kg"],
                    collision=BoxCollision(
                        half_extents_m=tuple(
                            collision_document["half_extents_m"]
                        )
                    ),
                ),
                visual_parts=tuple(parts),
            )
        )

    license_document = pack["license"]
    provenance_document = pack["provenance"]
    frame_document = pack["coordinate_frame"]
    return AssetPack(
        manifest_path=manifest_path,
        digest=digest,
        pack_id=pack["pack_id"],
        revision=pack["revision"],
        license=LicenseMetadata(
            spdx_id=license_document["spdx_id"],
            attribution=license_document["attribution"],
        ),
        provenance=Provenance(
            source_name=provenance_document["source_name"],
            source_uri=provenance_document["source_uri"],
            source_revision=provenance_document["source_revision"],
            source_item_ids=tuple(provenance_document["source_item_ids"]),
        ),
        coordinate_frame=CoordinateFrame(
            up_axis=frame_document["up_axis"],
            forward_axis=frame_document["forward_axis"],
            handedness=frame_document["handedness"],
            meters_per_unit=frame_document["meters_per_unit"],
        ),
        files=tuple(verified_files),
        items=tuple(items),
    )


def registry_manifest_relative_path(digest_ref: str) -> PurePosixPath:
    digest = _digest_hex(digest_ref, "pack digest")
    return PurePosixPath(
        "sha256", digest[:2], digest, REGISTRY_MANIFEST_NAME
    )


def resolve_registry_pack(registry_root: Path, digest_ref: str) -> AssetPack:
    registry_root = _absolute_path(registry_root)
    _require_plain_directory(registry_root, "registry root")
    relative = registry_manifest_relative_path(digest_ref)
    manifest_path = _asset_path_without_symlinks(registry_root, relative)
    pack = load_asset_pack(manifest_path)
    expected = _digest_hex(digest_ref, "pack digest")
    if pack.digest != expected:
        raise ItemAssetVerificationError(
            f"registry pack digest mismatch: expected sha256:{expected}, "
            f"got {pack.digest_ref}"
        )
    return pack


def load_inventory(manifest_path: Path) -> ItemInventory:
    manifest_path = _absolute_path(manifest_path)
    _require_plain_directory(manifest_path.parent, "inventory directory")
    raw_bytes = _read_regular_file(
        manifest_path,
        maximum_bytes=MAX_MANIFEST_BYTES,
        description="item inventory manifest",
    )
    try:
        text = raw_bytes.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ItemAssetValidationError(
            f"{manifest_path}: inventory is not UTF-8"
        ) from exc
    document = loads_json_strict(text, source=str(manifest_path))
    validated = _validate_inventory_document(document)
    inventory_document = validated["inventory"]
    entries: list[InventoryEntry] = []
    for entry_document in inventory_document["entries"]:
        spawn_document = entry_document["spawn"]
        entries.append(
            InventoryEntry(
                slot_id=entry_document["slot_id"],
                pack_digest=entry_document["pack_digest"],
                item_id=entry_document["item_id"],
                pool_size=entry_document["pool_size"],
                spawn=SpawnDefinition(
                    distance_m=spawn_document["distance_m"],
                    height_m=spawn_document["height_m"],
                    quaternion_wxyz=tuple(
                        spawn_document["quaternion_wxyz"]
                    ),
                ),
            )
        )
    return ItemInventory(
        manifest_path=manifest_path,
        inventory_id=inventory_document["inventory_id"],
        entries=tuple(entries),
    )


def resolve_inventory(
    inventory_path: Path,
    registry_root: Path,
) -> ResolvedInventory:
    inventory = load_inventory(inventory_path)
    packs: dict[str, AssetPack] = {}
    items: list[ResolvedInventoryItem] = []
    for entry in inventory.entries:
        pack = packs.get(entry.pack_digest)
        if pack is None:
            pack = resolve_registry_pack(registry_root, entry.pack_digest)
            packs[entry.pack_digest] = pack
        items.append(
            ResolvedInventoryItem(
                slot_id=entry.slot_id,
                pack=pack,
                item=pack.item(entry.item_id),
                pool_size=entry.pool_size,
                spawn=entry.spawn,
            )
        )
    return ResolvedInventory(inventory=inventory, items=tuple(items))


def _validate_asset_pack_document(document: Any) -> dict[str, Any]:
    root = _object(document, "document")
    _exact_keys(root, {"schema", "pack"}, "document")
    _equal(root["schema"], ASSET_PACK_SCHEMA, "schema")
    pack = _object(root["pack"], "pack")
    _exact_keys(
        pack,
        {
            "pack_id",
            "revision",
            "license",
            "provenance",
            "coordinate_frame",
            "files",
            "items",
        },
        "pack",
    )
    pack_id = _identifier(pack["pack_id"], "pack.pack_id")
    revision = _bounded_string(
        pack["revision"], "pack.revision", maximum=128
    )
    if REVISION_RE.fullmatch(revision) is None:
        raise ItemAssetValidationError("pack.revision is invalid")
    license_document = _validate_license(pack["license"])
    provenance_document = _validate_provenance(pack["provenance"])
    frame_document = _validate_coordinate_frame(pack["coordinate_frame"])
    files = _validate_files(pack["files"])
    items = _validate_items(
        pack["items"],
        {
            entry["file_id"]: (entry["role"], entry["format"])
            for entry in files
        },
    )
    return {
        "schema": ASSET_PACK_SCHEMA,
        "pack": {
            "pack_id": pack_id,
            "revision": revision,
            "license": license_document,
            "provenance": provenance_document,
            "coordinate_frame": frame_document,
            "files": files,
            "items": items,
        },
    }


def _validate_license(value: Any) -> dict[str, str]:
    document = _object(value, "pack.license")
    _exact_keys(document, {"spdx_id", "attribution"}, "pack.license")
    spdx_id = _bounded_string(
        document["spdx_id"], "pack.license.spdx_id", maximum=160
    )
    if SPDX_ID_RE.fullmatch(spdx_id) is None:
        raise ItemAssetValidationError(
            "pack.license.spdx_id must be one SPDX identifier or LicenseRef"
        )
    attribution = _bounded_string(
        document["attribution"],
        "pack.license.attribution",
        maximum=4096,
        allow_empty=True,
    )
    return {"spdx_id": spdx_id, "attribution": attribution}


def _validate_provenance(value: Any) -> dict[str, Any]:
    document = _object(value, "pack.provenance")
    _exact_keys(
        document,
        {"source_name", "source_uri", "source_revision", "source_item_ids"},
        "pack.provenance",
    )
    source_name = _bounded_string(
        document["source_name"],
        "pack.provenance.source_name",
        maximum=256,
    )
    source_uri = _bounded_string(
        document["source_uri"],
        "pack.provenance.source_uri",
        maximum=2048,
    )
    parsed_uri = urlparse(source_uri)
    if (
        parsed_uri.scheme not in {"https", "http", "ssh", "git", "urn"}
        or (parsed_uri.scheme != "urn" and not parsed_uri.netloc)
        or parsed_uri.password is not None
    ):
        raise ItemAssetValidationError(
            "pack.provenance.source_uri is not an allowed public source URI"
        )
    source_revision = _bounded_string(
        document["source_revision"],
        "pack.provenance.source_revision",
        maximum=256,
    )
    source_ids_raw = _array(
        document["source_item_ids"],
        "pack.provenance.source_item_ids",
        minimum=1,
        maximum=256,
    )
    source_item_ids = [
        _bounded_string(
            item,
            f"pack.provenance.source_item_ids[{index}]",
            maximum=256,
        )
        for index, item in enumerate(source_ids_raw)
    ]
    if len(source_item_ids) != len(set(source_item_ids)):
        raise ItemAssetValidationError(
            "pack.provenance.source_item_ids contains duplicates"
        )
    return {
        "source_name": source_name,
        "source_uri": source_uri,
        "source_revision": source_revision,
        "source_item_ids": source_item_ids,
    }


def _validate_coordinate_frame(value: Any) -> dict[str, Any]:
    document = _object(value, "pack.coordinate_frame")
    _exact_keys(
        document,
        {"up_axis", "forward_axis", "handedness", "meters_per_unit"},
        "pack.coordinate_frame",
    )
    up_axis = document["up_axis"]
    forward_axis = document["forward_axis"]
    handedness = document["handedness"]
    if up_axis not in AXES:
        raise ItemAssetValidationError("pack.coordinate_frame.up_axis is invalid")
    if forward_axis not in AXES:
        raise ItemAssetValidationError(
            "pack.coordinate_frame.forward_axis is invalid"
        )
    if up_axis[-1] == forward_axis[-1]:
        raise ItemAssetValidationError(
            "pack.coordinate_frame up and forward axes must be orthogonal"
        )
    if handedness not in HANDEDNESSES:
        raise ItemAssetValidationError(
            "pack.coordinate_frame.handedness is invalid"
        )
    meters_per_unit = _finite_float(
        document["meters_per_unit"],
        "pack.coordinate_frame.meters_per_unit",
        minimum=1e-9,
        maximum=1e9,
    )
    return {
        "up_axis": up_axis,
        "forward_axis": forward_axis,
        "handedness": handedness,
        "meters_per_unit": meters_per_unit,
    }


def _validate_files(value: Any) -> list[dict[str, Any]]:
    entries = _array(value, "pack.files", minimum=1, maximum=MAX_FILES)
    result: list[dict[str, Any]] = []
    seen_ids: set[str] = set()
    seen_paths: set[str] = set()
    for index, raw in enumerate(entries):
        name = f"pack.files[{index}]"
        document = _object(raw, name)
        _exact_keys(
            document,
            {
                "file_id",
                "path",
                "size_bytes",
                "sha256",
                "role",
                "media_type",
                "format",
            },
            name,
        )
        file_id = _identifier(document["file_id"], f"{name}.file_id")
        if file_id in seen_ids:
            raise ItemAssetValidationError(f"duplicate file_id {file_id!r}")
        seen_ids.add(file_id)
        relative = _safe_relative_path(document["path"], f"{name}.path")
        path_text = relative.as_posix()
        if path_text in seen_paths:
            raise ItemAssetValidationError(f"duplicate asset path {path_text!r}")
        seen_paths.add(path_text)
        size_bytes = _integer(
            document["size_bytes"],
            f"{name}.size_bytes",
            minimum=1,
            maximum=MAX_ASSET_BYTES,
        )
        sha256 = document["sha256"]
        if not isinstance(sha256, str) or SHA256_RE.fullmatch(sha256) is None:
            raise ItemAssetValidationError(f"{name}.sha256 is invalid")
        role = document["role"]
        if role not in FILE_ROLES:
            raise ItemAssetValidationError(f"{name}.role {role!r} is unsupported")
        media_type = document["media_type"]
        if (
            not isinstance(media_type, str)
            or MEDIA_TYPE_RE.fullmatch(media_type) is None
        ):
            raise ItemAssetValidationError(f"{name}.media_type is invalid")
        format_name = document["format"]
        if format_name not in SUPPORTED_FILE_FORMATS:
            raise ItemAssetValidationError(
                f"{name}.format {format_name!r} is unsupported"
            )
        if relative.suffix.lower() not in FILE_FORMAT_SUFFIXES[format_name]:
            raise ItemAssetValidationError(
                f"{name}.path suffix does not match format {format_name!r}"
            )
        if role == "visual_mesh" and format_name not in SUPPORTED_VISUAL_FORMATS:
            raise ItemAssetValidationError(
                f"{name} visual_mesh role requires a supported mesh format"
            )
        result.append(
            {
                "file_id": file_id,
                "path": path_text,
                "size_bytes": size_bytes,
                "sha256": sha256,
                "role": role,
                "media_type": media_type,
                "format": format_name,
            }
        )
    return result


def _validate_items(
    value: Any,
    files: dict[str, tuple[str, str]],
) -> list[dict[str, Any]]:
    entries = _array(value, "pack.items", minimum=1, maximum=MAX_ITEMS)
    result: list[dict[str, Any]] = []
    seen_ids: set[str] = set()
    for index, raw in enumerate(entries):
        name = f"pack.items[{index}]"
        document = _object(raw, name)
        _exact_keys(
            document,
            {"item_id", "label", "physics", "visual_parts"},
            name,
        )
        item_id = _identifier(document["item_id"], f"{name}.item_id")
        if item_id in seen_ids:
            raise ItemAssetValidationError(f"duplicate item_id {item_id!r}")
        seen_ids.add(item_id)
        label = _bounded_string(
            document["label"], f"{name}.label", maximum=128
        )
        physics = _validate_physics(document["physics"], f"{name}.physics")
        parts_raw = _array(
            document["visual_parts"],
            f"{name}.visual_parts",
            minimum=1,
            maximum=MAX_VISUAL_PARTS,
        )
        parts: list[dict[str, Any]] = []
        seen_parts: set[str] = set()
        for part_index, part_raw in enumerate(parts_raw):
            part_name = f"{name}.visual_parts[{part_index}]"
            part = _object(part_raw, part_name)
            _exact_keys(
                part,
                {
                    "part_id",
                    "file_id",
                    "rgba",
                    "scale",
                    "translation_m",
                    "rotation_wxyz",
                },
                part_name,
            )
            part_id = _identifier(part["part_id"], f"{part_name}.part_id")
            if part_id in seen_parts:
                raise ItemAssetValidationError(
                    f"{name} has duplicate visual part {part_id!r}"
                )
            seen_parts.add(part_id)
            file_id = _identifier(part["file_id"], f"{part_name}.file_id")
            if file_id not in files:
                raise ItemAssetValidationError(
                    f"{part_name}.file_id references unknown file {file_id!r}"
                )
            role, format_name = files[file_id]
            if role != "visual_mesh" or format_name not in SUPPORTED_VISUAL_FORMATS:
                raise ItemAssetValidationError(
                    f"{part_name}.file_id must reference a visual_mesh file"
                )
            rgba = _finite_vector(
                part["rgba"],
                f"{part_name}.rgba",
                length=4,
                minimum=0.0,
                maximum=1.0,
            )
            scale = _finite_vector(
                part["scale"],
                f"{part_name}.scale",
                length=3,
                minimum=1e-6,
                maximum=1e6,
            )
            translation = _finite_vector(
                part["translation_m"],
                f"{part_name}.translation_m",
                length=3,
                minimum=-1e6,
                maximum=1e6,
            )
            rotation = _unit_quaternion(
                part["rotation_wxyz"], f"{part_name}.rotation_wxyz"
            )
            parts.append(
                {
                    "part_id": part_id,
                    "file_id": file_id,
                    "rgba": list(rgba),
                    "scale": list(scale),
                    "translation_m": list(translation),
                    "rotation_wxyz": list(rotation),
                }
            )
        result.append(
            {
                "item_id": item_id,
                "label": label,
                "physics": physics,
                "visual_parts": parts,
            }
        )
    return result


def _validate_physics(value: Any, name: str) -> dict[str, Any]:
    document = _object(value, name)
    _exact_keys(document, {"mass_kg", "collision"}, name)
    mass_kg = _finite_float(
        document["mass_kg"],
        f"{name}.mass_kg",
        minimum=0.001,
        maximum=10000.0,
    )
    collision = _object(document["collision"], f"{name}.collision")
    _exact_keys(collision, {"shape", "half_extents_m"}, f"{name}.collision")
    _equal(collision["shape"], "box", f"{name}.collision.shape")
    half_extents = _finite_vector(
        collision["half_extents_m"],
        f"{name}.collision.half_extents_m",
        length=3,
        minimum=0.0001,
        maximum=1000.0,
    )
    return {
        "mass_kg": mass_kg,
        "collision": {
            "shape": "box",
            "half_extents_m": list(half_extents),
        },
    }


def _validate_inventory_document(document: Any) -> dict[str, Any]:
    root = _object(document, "document")
    _exact_keys(root, {"schema", "inventory"}, "document")
    _equal(root["schema"], INVENTORY_SCHEMA, "schema")
    inventory = _object(root["inventory"], "inventory")
    _exact_keys(inventory, {"inventory_id", "entries"}, "inventory")
    inventory_id = _identifier(
        inventory["inventory_id"], "inventory.inventory_id"
    )
    entries_raw = _array(
        inventory["entries"],
        "inventory.entries",
        minimum=1,
        maximum=MAX_INVENTORY_ENTRIES,
    )
    entries: list[dict[str, Any]] = []
    seen_slots: set[str] = set()
    for index, raw in enumerate(entries_raw):
        name = f"inventory.entries[{index}]"
        entry = _object(raw, name)
        _exact_keys(
            entry,
            {"slot_id", "pack_digest", "item_id", "pool_size", "spawn"},
            name,
        )
        slot_id = _identifier(entry["slot_id"], f"{name}.slot_id")
        if slot_id in seen_slots:
            raise ItemAssetValidationError(f"duplicate slot_id {slot_id!r}")
        seen_slots.add(slot_id)
        digest = _digest_hex(entry["pack_digest"], f"{name}.pack_digest")
        item_id = _identifier(entry["item_id"], f"{name}.item_id")
        pool_size = _integer(
            entry["pool_size"],
            f"{name}.pool_size",
            minimum=1,
            maximum=MAX_POOL_SIZE,
        )
        spawn = _object(entry["spawn"], f"{name}.spawn")
        _exact_keys(
            spawn,
            {"distance_m", "height_m", "quaternion_wxyz"},
            f"{name}.spawn",
        )
        distance = _finite_float(
            spawn["distance_m"],
            f"{name}.spawn.distance_m",
            minimum=0.0,
            maximum=1000.0,
        )
        height = _finite_float(
            spawn["height_m"],
            f"{name}.spawn.height_m",
            minimum=-1000.0,
            maximum=1000.0,
        )
        quaternion = _unit_quaternion(
            spawn["quaternion_wxyz"],
            f"{name}.spawn.quaternion_wxyz",
        )
        entries.append(
            {
                "slot_id": slot_id,
                "pack_digest": f"sha256:{digest}",
                "item_id": item_id,
                "pool_size": pool_size,
                "spawn": {
                    "distance_m": distance,
                    "height_m": height,
                    "quaternion_wxyz": list(quaternion),
                },
            }
        )
    return {
        "schema": INVENTORY_SCHEMA,
        "inventory": {
            "inventory_id": inventory_id,
            "entries": entries,
        },
    }


def _object(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ItemAssetValidationError(f"{name} must be an object")
    return value


def _array(
    value: Any,
    name: str,
    *,
    minimum: int,
    maximum: int,
) -> list[Any]:
    if not isinstance(value, list) or not minimum <= len(value) <= maximum:
        raise ItemAssetValidationError(
            f"{name} must contain {minimum}..{maximum} entries"
        )
    return value


def _exact_keys(
    value: dict[str, Any],
    expected: set[str],
    name: str,
) -> None:
    actual = set(value)
    if actual != expected:
        missing = sorted(expected - actual)
        unknown = sorted(actual - expected)
        details: list[str] = []
        if missing:
            details.append(f"missing={missing}")
        if unknown:
            details.append(f"unknown={unknown}")
        raise ItemAssetValidationError(
            f"{name} has invalid fields ({', '.join(details)})"
        )


def _equal(value: Any, expected: Any, name: str) -> None:
    if value != expected:
        raise ItemAssetValidationError(
            f"{name} must equal {expected!r}"
        )


def _identifier(value: Any, name: str) -> str:
    if not isinstance(value, str) or ID_RE.fullmatch(value) is None:
        raise ItemAssetValidationError(f"{name} is not a valid identifier")
    return value


def _bounded_string(
    value: Any,
    name: str,
    *,
    maximum: int,
    allow_empty: bool = False,
) -> str:
    if (
        not isinstance(value, str)
        or len(value) > maximum
        or (not allow_empty and not value)
        or "\x00" in value
    ):
        raise ItemAssetValidationError(f"{name} is invalid")
    return value


def _integer(
    value: Any,
    name: str,
    *,
    minimum: int,
    maximum: int,
) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not minimum <= value <= maximum
    ):
        raise ItemAssetValidationError(
            f"{name} must be an integer in [{minimum}, {maximum}]"
        )
    return value


def _finite_float(
    value: Any,
    name: str,
    *,
    minimum: float,
    maximum: float,
) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ItemAssetValidationError(f"{name} must be a number")
    result = float(value)
    if not math.isfinite(result) or not minimum <= result <= maximum:
        raise ItemAssetValidationError(
            f"{name} must be finite and in [{minimum}, {maximum}]"
        )
    return result


def _finite_vector(
    value: Any,
    name: str,
    *,
    length: int,
    minimum: float,
    maximum: float,
) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != length:
        raise ItemAssetValidationError(
            f"{name} must contain exactly {length} numbers"
        )
    return tuple(
        _finite_float(
            item,
            f"{name}[{index}]",
            minimum=minimum,
            maximum=maximum,
        )
        for index, item in enumerate(value)
    )


def _unit_quaternion(value: Any, name: str) -> tuple[float, float, float, float]:
    vector = _finite_vector(
        value,
        name,
        length=4,
        minimum=-1.0e12,
        maximum=1.0e12,
    )
    norm = math.sqrt(sum(component * component for component in vector))
    if norm < 1e-12:
        raise ItemAssetValidationError(f"{name} must not be zero")
    return tuple(component / norm for component in vector)  # type: ignore[return-value]


def _digest_hex(value: Any, name: str) -> str:
    if not isinstance(value, str):
        raise ItemAssetValidationError(f"{name} is invalid")
    match = DIGEST_REF_RE.fullmatch(value)
    if match is None:
        raise ItemAssetValidationError(
            f"{name} must use sha256:<64 lowercase hex>"
        )
    return match.group(1)


def _safe_relative_path(value: Any, name: str) -> PurePosixPath:
    if (
        not isinstance(value, str)
        or not value
        or len(value) > 1024
        or "\x00" in value
        or "\\" in value
    ):
        raise ItemAssetValidationError(f"{name} is not a safe relative path")
    path = PurePosixPath(value)
    if (
        path.is_absolute()
        or path.as_posix() != value
        or any(part in {"", ".", ".."} for part in path.parts)
    ):
        raise ItemAssetValidationError(f"{name} is not a safe relative path")
    return path


def _absolute_path(path: Path) -> Path:
    return Path(os.path.abspath(os.fspath(path)))


def _require_plain_directory(path: Path, description: str) -> None:
    _reject_symlink_ancestors(path, description)
    try:
        metadata = path.lstat()
    except OSError as exc:
        raise ItemAssetIOError(f"cannot inspect {description} {path}: {exc}") from exc
    if stat.S_ISLNK(metadata.st_mode):
        raise ItemAssetVerificationError(f"{description} must not be a symlink: {path}")
    if not stat.S_ISDIR(metadata.st_mode):
        raise ItemAssetVerificationError(f"{description} is not a directory: {path}")


def _reject_symlink_ancestors(path: Path, description: str) -> None:
    for ancestor in reversed(path.parents):
        try:
            metadata = ancestor.lstat()
        except OSError as exc:
            raise ItemAssetIOError(
                f"cannot inspect {description} ancestor {ancestor}: {exc}"
            ) from exc
        if stat.S_ISLNK(metadata.st_mode):
            raise ItemAssetVerificationError(
                f"{description} ancestor must not be a symlink: {ancestor}"
            )


def _asset_path_without_symlinks(
    root: Path,
    relative: PurePosixPath,
) -> Path:
    root = _absolute_path(root)
    _require_plain_directory(root, "asset root")
    current = root
    for component in relative.parts[:-1]:
        current = current / component
        _require_plain_directory(current, "asset parent")
    target = current / relative.parts[-1]
    try:
        metadata = target.lstat()
    except OSError as exc:
        raise ItemAssetIOError(f"cannot inspect asset path {target}: {exc}") from exc
    if stat.S_ISLNK(metadata.st_mode):
        raise ItemAssetVerificationError(f"asset path must not be a symlink: {target}")
    if not stat.S_ISREG(metadata.st_mode):
        raise ItemAssetVerificationError(f"asset path is not a regular file: {target}")
    return target


def _read_regular_file(
    path: Path,
    *,
    maximum_bytes: int,
    description: str,
) -> bytes:
    descriptor, opened = _open_regular_file(
        path,
        maximum_bytes=maximum_bytes,
        description=description,
    )
    try:
        chunks: list[bytes] = []
        remaining = maximum_bytes + 1
        while remaining:
            chunk = os.read(descriptor, min(1024 * 1024, remaining))
            if not chunk:
                break
            chunks.append(chunk)
            remaining -= len(chunk)
        payload = b"".join(chunks)
        _verify_open_file_stable(
            descriptor,
            opened,
            bytes_read=len(payload),
            maximum_bytes=maximum_bytes,
            description=description,
            path=path,
        )
        return payload
    finally:
        os.close(descriptor)


def _verify_regular_file(
    path: Path,
    *,
    maximum_bytes: int,
    description: str,
    expected_size: int,
    expected_sha256: str,
) -> tuple[int, str]:
    descriptor, opened = _open_regular_file(
        path,
        maximum_bytes=maximum_bytes,
        description=description,
    )
    try:
        if opened.st_size != expected_size:
            raise ItemAssetVerificationError(
                f"{description} size mismatch: expected {expected_size}, "
                f"got {opened.st_size}"
            )
        digest = hashlib.sha256()
        bytes_read = 0
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            bytes_read += len(chunk)
            if bytes_read > maximum_bytes:
                raise ItemAssetVerificationError(
                    f"{description} exceeded its size limit: {path}"
                )
            digest.update(chunk)
        _verify_open_file_stable(
            descriptor,
            opened,
            bytes_read=bytes_read,
            maximum_bytes=maximum_bytes,
            description=description,
            path=path,
        )
        actual_sha256 = digest.hexdigest()
        if actual_sha256 != expected_sha256:
            raise ItemAssetVerificationError(f"{description} SHA256 mismatch")
        return bytes_read, actual_sha256
    finally:
        os.close(descriptor)


def _open_regular_file(
    path: Path,
    *,
    maximum_bytes: int,
    description: str,
) -> tuple[int, os.stat_result]:
    _reject_symlink_ancestors(path, description)
    try:
        before = path.lstat()
    except OSError as exc:
        raise ItemAssetIOError(f"cannot inspect {description} {path}: {exc}") from exc
    if stat.S_ISLNK(before.st_mode) or not stat.S_ISREG(before.st_mode):
        raise ItemAssetVerificationError(
            f"{description} must be a non-symlink regular file: {path}"
        )
    if before.st_size < 1 or before.st_size > maximum_bytes:
        raise ItemAssetVerificationError(
            f"{description} size must be in [1, {maximum_bytes}]: {path}"
        )
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(path, flags)
    except OSError as exc:
        raise ItemAssetIOError(f"cannot open {description} {path}: {exc}") from exc
    opened = os.fstat(descriptor)
    if (
        not stat.S_ISREG(opened.st_mode)
        or (opened.st_dev, opened.st_ino) != (before.st_dev, before.st_ino)
    ):
        os.close(descriptor)
        raise ItemAssetVerificationError(
            f"{description} changed while opening: {path}"
        )
    return descriptor, opened


def _verify_open_file_stable(
    descriptor: int,
    opened: os.stat_result,
    *,
    bytes_read: int,
    maximum_bytes: int,
    description: str,
    path: Path,
) -> None:
    after = os.fstat(descriptor)
    if (
        bytes_read > maximum_bytes
        or bytes_read != opened.st_size
        or opened.st_size != after.st_size
        or (opened.st_dev, opened.st_ino) != (after.st_dev, after.st_ino)
    ):
        raise ItemAssetVerificationError(
            f"{description} changed or exceeded its size limit: {path}"
        )


def _summary(pack: AssetPack) -> dict[str, Any]:
    return {
        "schema": ASSET_PACK_SCHEMA,
        "pack_id": pack.pack_id,
        "revision": pack.revision,
        "digest": pack.digest_ref,
        "files": len(pack.files),
        "items": [item.item_id for item in pack.items],
    }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    verify = subparsers.add_parser("verify-pack")
    verify.add_argument("manifest", type=Path)
    resolve = subparsers.add_parser("resolve-pack")
    resolve.add_argument("registry", type=Path)
    resolve.add_argument("digest")
    inventory = subparsers.add_parser("resolve-inventory")
    inventory.add_argument("manifest", type=Path)
    inventory.add_argument("registry", type=Path)
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    arguments = _build_parser().parse_args(list(argv) if argv is not None else None)
    try:
        if arguments.command == "verify-pack":
            output = _summary(load_asset_pack(arguments.manifest))
        elif arguments.command == "resolve-pack":
            output = _summary(
                resolve_registry_pack(arguments.registry, arguments.digest)
            )
        elif arguments.command == "resolve-inventory":
            resolved = resolve_inventory(arguments.manifest, arguments.registry)
            output = {
                "schema": INVENTORY_SCHEMA,
                "inventory_id": resolved.inventory.inventory_id,
                "slots": [
                    {
                        "slot_id": item.slot_id,
                        "pack_digest": item.pack.digest_ref,
                        "pack_id": item.pack.pack_id,
                        "item_id": item.item.item_id,
                    }
                    for item in resolved.items
                ],
            }
        else:  # pragma: no cover - argparse enforces the subcommand.
            raise AssertionError(arguments.command)
    except ItemAssetPackError as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 2
    print(json.dumps(output, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
