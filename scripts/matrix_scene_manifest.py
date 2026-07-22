"""Versioned, atomic scene manifests for live RealScan scene authoring.

Manifests contain only small immutable asset references, transforms, and
placed-entity declarations.  NuRec/3DGS and collision payloads remain external
files and are verified by size and SHA256 before a runtime restores them.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import stat
import sys
import tempfile
from typing import Any, Iterable, Iterator
from urllib.parse import unquote, urlparse
from uuid import uuid4


SCHEMA_ID = "matrix-scene-manifest/v1"
CANONICALIZATION = "matrix_scene_manifest_v1_canonical_json_sha256"
STORE_DIGEST_CANONICALIZATION = "matrix_scene_manifest_v1_store_metadata_sha256"
DIGEST_ALGORITHM = "sha256"
VISUAL_ROLE = "visual"
COLLISION_ROLE = "collision"
ASSET_ROLES = frozenset({VISUAL_ROLE, COLLISION_ROLE})
BACKENDS_BY_ROLE = {
    VISUAL_ROLE: frozenset({"3dgs", "nurec", "ue_cooked"}),
    COLLISION_ROLE: frozenset({"mujoco_mesh", "physx_usd"}),
}
MEDIA_TYPES_BY_BACKEND = {
    "3dgs": frozenset({"3dgs_ply", "splat", "spz"}),
    "nurec": frozenset({"nurec_usdz"}),
    "ue_cooked": frozenset({"ue_asset"}),
    "mujoco_mesh": frozenset({"msh", "obj", "stl"}),
    "physx_usd": frozenset({"usd", "usda", "usdc"}),
}
LOCATOR_SCHEMES_BY_BACKEND = {
    "3dgs": frozenset({"file"}),
    "nurec": frozenset({"file"}),
    "ue_cooked": frozenset({"ue_package"}),
    "mujoco_mesh": frozenset({"file"}),
    "physx_usd": frozenset({"file"}),
}
DERIVATION_SOURCES_BY_BACKEND = {
    "ue_cooked": frozenset({"3dgs", "nurec"}),
    "mujoco_mesh": frozenset({"physx_usd"}),
}
ENTITY_KINDS = frozenset({"scene", "robot", "prop", "anchor"})
PHYSICS_MODES = frozenset({"none", "static", "kinematic", "dynamic"})
UP_AXES = frozenset({"X", "Y", "Z"})
HANDEDNESSES = frozenset({"left", "right"})
ID_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_.:-]{0,127}$")
DIGEST_RE = re.compile(r"^[0-9a-f]{64}$")
REVISION_RE = re.compile(r"^rev-[0-9a-f]{32}$")
UE_PACKAGE_RE = re.compile(r"^/Game/[A-Za-z0-9_./-]{1,240}$")
MAX_STORE_BYTES = 4 * 1024 * 1024
MAX_ASSETS = 8192
MAX_TRANSFORMS = 8192
MAX_ENTITIES = 8192
MAX_TAGS = 64


class SceneManifestError(ValueError):
    """Base class for fail-closed scene manifest errors."""


class ManifestValidationError(SceneManifestError):
    pass


class ManifestIOError(SceneManifestError):
    pass


class ManifestConflictError(SceneManifestError):
    pass


class AssetVerificationError(SceneManifestError):
    pass


@dataclass(frozen=True)
class StoredSceneManifest:
    path: Path
    document: dict[str, Any]
    generation: int
    scene_digest: str
    store_digest: str
    revision_id: str
    recovered_from_backup: bool = False


@dataclass(frozen=True)
class VerifiedAssetHandle:
    asset_ids: tuple[str, ...]
    path: Path
    fd: int
    size_bytes: int
    sha256: str

    @property
    def proc_path(self) -> Path:
        return Path(f"/proc/self/fd/{self.fd}")


def loads_json_strict(text: str, *, source: str = "<json>") -> Any:
    def no_duplicates(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise ManifestValidationError(f"{source}: duplicate key {key!r}")
            result[key] = value
        return result

    def no_constants(value: str) -> None:
        raise ManifestValidationError(f"{source}: non-finite number {value}")

    try:
        return json.loads(
            text,
            object_pairs_hook=no_duplicates,
            parse_constant=no_constants,
        )
    except ManifestValidationError:
        raise
    except (json.JSONDecodeError, ValueError) as exc:
        detail = exc.msg if isinstance(exc, json.JSONDecodeError) else str(exc)
        raise ManifestValidationError(f"{source}: invalid JSON: {detail}") from exc


def load_json_strict(path: Path) -> Any:
    path = Path(path)
    try:
        text = _read_bytes_secure(path).decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ManifestValidationError(f"{path} is not UTF-8 JSON") from exc
    return loads_json_strict(text, source=str(path))


def validate_scene_document(document: Any) -> dict[str, Any]:
    _require_object(document, "document")
    _require_exact_keys(document, {"schema", "scene"}, "document")
    _require_equal(document["schema"], SCHEMA_ID, "schema")
    _validate_scene(document["scene"], "scene")
    return {
        "schema": SCHEMA_ID,
        "scene": _canonical_scene(document["scene"]),
    }


def validate_store_document(document: Any) -> dict[str, Any]:
    _require_object(document, "document")
    _require_exact_keys(document, {"schema", "scene", "storage"}, "document")
    _require_equal(document["schema"], SCHEMA_ID, "schema")
    scene = validate_scene_document(
        {"schema": document["schema"], "scene": document["scene"]}
    )
    storage = _require_object(document["storage"], "storage")
    _require_exact_keys(
        storage,
        {
            "generation",
            "scene_digest",
            "store_digest",
            "revision_id",
            "canonicalization",
            "created_at",
            "updated_at",
        },
        "storage",
    )
    generation = _require_int(storage["generation"], "storage.generation")
    if generation < 1:
        raise ManifestValidationError("storage.generation must be >= 1")
    _require_equal(
        storage["canonicalization"],
        CANONICALIZATION,
        "storage.canonicalization",
    )
    revision_id = _require_revision_id(
        storage["revision_id"], "storage.revision_id"
    )
    created_at = _require_timestamp(storage["created_at"], "storage.created_at")
    updated_at = _require_timestamp(storage["updated_at"], "storage.updated_at")
    if updated_at < created_at:
        raise ManifestValidationError("storage.updated_at predates created_at")
    digest_meta = _validate_digest_meta(
        storage["scene_digest"], "storage.scene_digest"
    )
    actual = scene_digest(scene)
    if digest_meta["digest"] != actual:
        raise ManifestValidationError(
            "storage.scene_digest does not match canonical scene digest"
        )
    store_digest_meta = _validate_digest_meta(
        storage["store_digest"], "storage.store_digest"
    )
    expected_store_digest = _store_metadata_digest(
        generation=generation,
        revision_id=revision_id,
        scene_digest_value=digest_meta["digest"],
        created_at=storage["created_at"],
        updated_at=storage["updated_at"],
    )
    if store_digest_meta["digest"] != expected_store_digest:
        raise ManifestValidationError(
            "storage.store_digest does not match generation and scene metadata"
        )
    return {
        "schema": SCHEMA_ID,
        "scene": scene["scene"],
        "storage": {
            "generation": generation,
            "scene_digest": digest_meta,
            "store_digest": store_digest_meta,
            "revision_id": revision_id,
            "canonicalization": CANONICALIZATION,
            "created_at": storage["created_at"],
            "updated_at": storage["updated_at"],
        },
    }


def canonical_scene_bytes(scene_document: dict[str, Any]) -> bytes:
    validated = validate_scene_document(scene_document)
    return json.dumps(
        validated,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")


def scene_digest(scene_document: dict[str, Any]) -> str:
    return hashlib.sha256(canonical_scene_bytes(scene_document)).hexdigest()


def store_document(
    scene_document: dict[str, Any],
    *,
    generation: int,
    created_at: str | None = None,
    updated_at: str | None = None,
    revision_id: str | None = None,
) -> dict[str, Any]:
    generation = _require_int(generation, "generation")
    if generation < 1:
        raise ManifestValidationError("generation must be >= 1")
    validated = validate_scene_document(scene_document)
    now = _utc_now() if created_at is None or updated_at is None else None
    created = created_at if created_at is not None else now
    updated = updated_at if updated_at is not None else now
    assert created is not None and updated is not None
    revision = revision_id or f"rev-{uuid4().hex}"
    revision = _require_revision_id(revision, "revision_id")
    digest = scene_digest(validated)
    document = {
        "schema": SCHEMA_ID,
        "scene": validated["scene"],
        "storage": {
            "generation": generation,
            "scene_digest": {
                "algorithm": DIGEST_ALGORITHM,
                "digest": digest,
            },
            "store_digest": {
                "algorithm": DIGEST_ALGORITHM,
                "digest": _store_metadata_digest(
                    generation=generation,
                    revision_id=revision,
                    scene_digest_value=digest,
                    created_at=created,
                    updated_at=updated,
                ),
            },
            "canonicalization": CANONICALIZATION,
            "revision_id": revision,
            "created_at": created,
            "updated_at": updated,
        },
    }
    return document


def store_document_bytes(document: dict[str, Any]) -> bytes:
    validated = validate_store_document(document)
    encoded = (
        json.dumps(
            validated,
            sort_keys=True,
            indent=2,
            ensure_ascii=True,
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")
    if len(encoded) > MAX_STORE_BYTES:
        raise ManifestValidationError(
            f"encoded scene exceeds {MAX_STORE_BYTES} bytes"
        )
    return encoded


def extract_scene_input(document: Any) -> dict[str, Any]:
    _require_object(document, "input")
    keys = set(document)
    if keys == {"schema", "scene"}:
        return validate_scene_document(document)
    if keys == {"schema", "scene", "storage"}:
        stored = validate_store_document(document)
        return {"schema": SCHEMA_ID, "scene": stored["scene"]}
    raise ManifestValidationError(
        "input must be a scene document or stored scene_manifest document"
    )


def _read_store_unlocked(path: Path, *, recover: bool) -> StoredSceneManifest:
    errors: list[str] = []
    found = False
    for recovered, candidate in (
        (False, path),
        (True, _backup_path(path)),
    ):
        try:
            document = validate_store_document(load_json_strict(candidate))
        except FileNotFoundError:
            if not recover and not recovered:
                break
            continue
        except ManifestValidationError as exc:
            found = True
            errors.append(f"{candidate}: {exc}")
            if not recover:
                break
            continue
        return _stored(path, document, recovered=recovered)
    if not found:
        raise FileNotFoundError(path)
    raise ManifestValidationError("; ".join(errors))


def read_store(path: Path, *, recover: bool = True) -> StoredSceneManifest:
    path = Path(path)
    _validate_primary_store_path(path)
    if not path.exists() and not _backup_path(path).exists():
        raise FileNotFoundError(path)
    with _store_lock(path, exclusive=False):
        return _read_store_unlocked(path, recover=recover)


def write_store(
    path: Path,
    scene_document: dict[str, Any],
    *,
    expected_generation: int,
    expected_store_digest: str | None = None,
) -> StoredSceneManifest:
    path = Path(path)
    _validate_primary_store_path(path)
    expected = _require_int(expected_generation, "expected_generation")
    if expected < 0:
        raise ManifestValidationError("expected_generation must be >= 0")
    with _store_lock(path, exclusive=True):
        current: StoredSceneManifest | None
        if path.exists() or _backup_path(path).exists():
            current = _read_store_unlocked(path, recover=True)
        else:
            current = None
        actual = current.generation if current else 0
        if expected != actual:
            raise ManifestConflictError(
                f"expected generation {expected}, found {actual}"
            )
        if current is None:
            if expected_store_digest is not None:
                raise ManifestConflictError(
                    "expected_store_digest must be omitted when creating generation 1"
                )
        else:
            if expected_store_digest is None:
                raise ManifestConflictError(
                    "expected_store_digest is required when updating an existing store"
                )
            expected_digest = _require_string(
                expected_store_digest, "expected_store_digest"
            )
            if DIGEST_RE.fullmatch(expected_digest) is None:
                raise ManifestValidationError(
                    "expected_store_digest must be a lowercase sha256 hex digest"
                )
            if expected_digest != current.store_digest:
                raise ManifestConflictError(
                    "expected store digest does not match the current revision"
                )
        now = _utc_now()
        created = current.document["storage"]["created_at"] if current else now
        if current is not None:
            previous_updated = current.document["storage"]["updated_at"]
            if datetime.fromisoformat(now) < datetime.fromisoformat(previous_updated):
                now = previous_updated
        document = store_document(
            scene_document,
            generation=actual + 1,
            created_at=created,
            updated_at=now,
        )
        encoded = store_document_bytes(document)
        if current is None:
            # A first-generation backup protects against interruption between
            # the two renames.
            _atomic_write_bytes(_backup_path(path), encoded)
        elif not current.recovered_from_backup:
            # Subsequent backups remain one known-good generation behind.
            _atomic_write_bytes(
                _backup_path(path), store_document_bytes(current.document)
            )
        _atomic_write_bytes(path, encoded)
        return _stored(path, validate_store_document(document), recovered=False)


def inspect_store(path: Path) -> dict[str, Any]:
    stored = read_store(path)
    scene = stored.document["scene"]
    return {
        "path": str(path),
        "schema": SCHEMA_ID,
        "generation": stored.generation,
        "scene_digest": stored.document["storage"]["scene_digest"],
        "store_digest": stored.document["storage"]["store_digest"],
        "revision_id": stored.revision_id,
        "canonicalization": CANONICALIZATION,
        "asset_references": len(scene["asset_references"]),
        "transforms": len(scene["transforms"]),
        "placed_entities": len(scene["placed_entities"]),
        "recovered_from_backup": stored.recovered_from_backup,
    }


@contextmanager
def open_verified_asset_references(
    scene_document: dict[str, Any],
    *,
    allowed_roots: Iterable[Path],
) -> Iterator[tuple[VerifiedAssetHandle, ...]]:
    """Keep verified asset inodes open for the complete runtime load window."""

    validate_scene_document(scene_document)
    roots = _canonical_allowed_roots(allowed_roots)
    grouped: dict[tuple[str, str, int], list[str]] = {}
    for asset in scene_document["scene"]["asset_references"]:
        locator = asset["locator"]
        if locator["scheme"] != "file":
            raise AssetVerificationError(
                f"asset {asset['id']!r} requires a configured "
                f"{locator['scheme']!r} resolver"
            )
        assert isinstance(asset["size_bytes"], int)
        key = (
            locator["value"],
            asset["content_hash"]["digest"],
            asset["size_bytes"],
        )
        grouped.setdefault(key, []).append(asset["id"])

    handles: list[VerifiedAssetHandle] = []
    try:
        for (locator_value, expected_hash, expected_size), asset_ids in grouped.items():
            path = _uri_path(locator_value)
            root, relative = _asset_location_beneath(path, roots)
            fd, actual_size, actual_hash = _open_sha256_regular_file(
                root,
                relative,
                display_path=path,
                expected_size=expected_size,
            )
            handle = VerifiedAssetHandle(
                asset_ids=tuple(asset_ids),
                path=path,
                fd=fd,
                size_bytes=actual_size,
                sha256=actual_hash,
            )
            handles.append(handle)
            if actual_hash != expected_hash:
                raise AssetVerificationError(
                    f"asset SHA256 mismatch for {path}: "
                    f"expected={expected_hash} actual={actual_hash}"
                )
        yield tuple(handles)
    finally:
        for handle in handles:
            try:
                os.close(handle.fd)
            except OSError:
                pass


def _validate_scene(scene: Any, path: str) -> None:
    scene = _require_object(scene, path)
    _require_exact_keys(
        scene,
        {
            "id",
            "coordinate_frame",
            "asset_references",
            "transforms",
            "placed_entities",
        },
        path,
    )
    _require_id(scene["id"], f"{path}.id")
    _validate_coordinate_frame(scene["coordinate_frame"], f"{path}.coordinate_frame")
    assets = _require_list(scene["asset_references"], f"{path}.asset_references")
    transforms = _require_list(scene["transforms"], f"{path}.transforms")
    entities = _require_list(scene["placed_entities"], f"{path}.placed_entities")
    if (
        len(assets) > MAX_ASSETS
        or len(transforms) > MAX_TRANSFORMS
        or len(entities) > MAX_ENTITIES
    ):
        raise ManifestValidationError(f"{path} exceeds collection limits")

    used_ids: set[str] = {scene["id"]}
    _reserve_id(scene["coordinate_frame"]["id"], used_ids)
    assets_by_id: dict[str, dict[str, Any]] = {}
    transforms_by_id: dict[str, dict[str, Any]] = {}
    for index, asset in enumerate(assets):
        asset_path = f"{path}.asset_references[{index}]"
        _validate_asset_reference(asset, asset_path)
        asset_id = asset["id"]
        _reserve_id(asset_id, used_ids)
        assets_by_id[asset_id] = asset
    _validate_asset_derivations(assets_by_id, path)
    for index, transform in enumerate(transforms):
        transform_path = f"{path}.transforms[{index}]"
        _validate_transform(transform, transform_path)
        transform_id = transform["id"]
        _reserve_id(transform_id, used_ids)
        transforms_by_id[transform_id] = transform
    for index, entity in enumerate(entities):
        _validate_entity(
            entity,
            f"{path}.placed_entities[{index}]",
            used_ids=used_ids,
            assets_by_id=assets_by_id,
            transforms_by_id=transforms_by_id,
        )


def _validate_asset_reference(asset: Any, path: str) -> None:
    asset = _require_object(asset, path)
    _require_exact_keys(
        asset,
        {
            "id",
            "role",
            "backend",
            "locator",
            "media_type",
            "content_hash",
            "size_bytes",
            "source_selector",
            "derived_from_asset_id",
        },
        path,
    )
    _require_id(asset["id"], f"{path}.id")
    role = _require_enum(asset["role"], ASSET_ROLES, f"{path}.role")
    backend = _require_enum(
        asset["backend"], BACKENDS_BY_ROLE[role], f"{path}.backend"
    )
    _require_enum(
        asset["media_type"],
        MEDIA_TYPES_BY_BACKEND[backend],
        f"{path}.media_type",
    )
    locator = _require_object(asset["locator"], f"{path}.locator")
    _require_exact_keys(locator, {"scheme", "value"}, f"{path}.locator")
    scheme = _require_enum(
        locator["scheme"],
        LOCATOR_SCHEMES_BY_BACKEND[backend],
        f"{path}.locator.scheme",
    )
    locator_value = _require_string(locator["value"], f"{path}.locator.value")
    size = asset["size_bytes"]
    if scheme == "file":
        _uri_path(locator_value)
        size = _require_int(size, f"{path}.size_bytes")
        if size < 1:
            raise ManifestValidationError(f"{path}.size_bytes must be >= 1")
    else:
        if size is not None:
            raise ManifestValidationError(
                f"{path}.size_bytes must be null for a UE package locator"
            )
        if UE_PACKAGE_RE.fullmatch(locator_value) is None or ".." in Path(
            locator_value
        ).parts:
            raise ManifestValidationError(
                f"{path}.locator.value is not a safe /Game package"
            )
    source_selector = asset["source_selector"]
    if source_selector is not None:
        source_selector = _require_string(
            source_selector, f"{path}.source_selector"
        )
        if len(source_selector) > 512:
            raise ManifestValidationError(
                f"{path}.source_selector exceeds 512 characters"
            )
    if backend == "physx_usd" and (
        source_selector is None or not source_selector.startswith("/")
    ):
        raise ManifestValidationError(
            f"{path}.source_selector must select an absolute PhysX prim"
        )
    _require_optional_id(
        asset["derived_from_asset_id"], f"{path}.derived_from_asset_id"
    )
    _validate_digest_meta(asset["content_hash"], f"{path}.content_hash")


def _validate_asset_derivations(
    assets_by_id: dict[str, dict[str, Any]], scene_path: str
) -> None:
    for asset_id, asset in assets_by_id.items():
        parent = asset["derived_from_asset_id"]
        if parent is not None and parent not in assets_by_id:
            raise ManifestValidationError(
                f"{scene_path}.asset_references {asset_id!r} derives from an unknown asset"
            )
        if parent == asset_id:
            raise ManifestValidationError(
                f"{scene_path}.asset_references {asset_id!r} cannot derive from itself"
            )
        if parent is not None:
            parent_asset = assets_by_id[parent]
            if parent_asset["role"] != asset["role"]:
                raise ManifestValidationError(
                    f"{scene_path}.asset_references {asset_id!r} cannot derive "
                    "across visual/collision roles"
                )
            allowed_sources = DERIVATION_SOURCES_BY_BACKEND.get(
                asset["backend"], frozenset()
            )
            if parent_asset["backend"] not in allowed_sources:
                raise ManifestValidationError(
                    f"{scene_path}.asset_references {asset_id!r} has an "
                    "unsupported backend derivation"
                )

    for start in assets_by_id:
        seen: set[str] = set()
        current: str | None = start
        while current is not None:
            if current in seen:
                raise ManifestValidationError(
                    f"{scene_path}.asset_references contains a derivation cycle"
                )
            seen.add(current)
            current = assets_by_id[current]["derived_from_asset_id"]


def _validate_coordinate_frame(frame: Any, path: str) -> None:
    frame = _require_object(frame, path)
    _require_exact_keys(
        frame,
        {"id", "meters_per_unit", "up_axis", "handedness"},
        path,
    )
    _require_id(frame["id"], f"{path}.id")
    meters_per_unit = _require_number(
        frame["meters_per_unit"], f"{path}.meters_per_unit"
    )
    if meters_per_unit <= 0:
        raise ManifestValidationError(f"{path}.meters_per_unit must be positive")
    up_axis = _require_enum(frame["up_axis"], UP_AXES, f"{path}.up_axis")
    handedness = _require_enum(
        frame["handedness"], HANDEDNESSES, f"{path}.handedness"
    )
    if (meters_per_unit, up_axis, handedness) != (1.0, "Z", "right"):
        raise ManifestValidationError(
            f"{path} must use Matrix canonical right-handed Z-up metres"
        )


def _validate_transform(transform: Any, path: str) -> None:
    transform = _require_object(transform, path)
    _require_exact_keys(
        transform,
        {"id", "translation", "rotation_xyzw", "scale"},
        path,
    )
    _require_id(transform["id"], f"{path}.id")
    _require_number_list(transform["translation"], 3, f"{path}.translation")
    rotation = _require_number_list(
        transform["rotation_xyzw"], 4, f"{path}.rotation_xyzw"
    )
    norm = math.sqrt(sum(value * value for value in rotation))
    if abs(norm - 1.0) > 1.0e-5:
        raise ManifestValidationError(
            f"{path}.rotation_xyzw must be a normalized quaternion"
        )
    scale = _require_number_list(transform["scale"], 3, f"{path}.scale")
    if any(value <= 0 for value in scale):
        raise ManifestValidationError(f"{path}.scale values must be positive")


def _validate_entity(
    entity: Any,
    path: str,
    *,
    used_ids: set[str],
    assets_by_id: dict[str, dict[str, Any]],
    transforms_by_id: dict[str, dict[str, Any]],
) -> None:
    entity = _require_object(entity, path)
    _require_exact_keys(
        entity,
        {
            "id",
            "kind",
            "transform_id",
            "visual_asset_id",
            "collision_asset_id",
            "physics_mode",
            "visible",
            "collision_enabled",
            "tags",
        },
        path,
    )
    entity_id = _require_id(entity["id"], f"{path}.id")
    _reserve_id(entity_id, used_ids)
    kind = _require_enum(entity["kind"], ENTITY_KINDS, f"{path}.kind")
    transform_id = _require_id(entity["transform_id"], f"{path}.transform_id")
    if transform_id not in transforms_by_id:
        raise ManifestValidationError(f"{path}.transform_id references unknown transform")
    visual_id = _require_optional_id(entity["visual_asset_id"], f"{path}.visual_asset_id")
    collision_id = _require_optional_id(
        entity["collision_asset_id"], f"{path}.collision_asset_id"
    )
    physics_mode = _require_enum(
        entity["physics_mode"], PHYSICS_MODES, f"{path}.physics_mode"
    )
    visible = _require_bool(entity["visible"], f"{path}.visible")
    collision_enabled = _require_bool(
        entity["collision_enabled"], f"{path}.collision_enabled"
    )
    tags = _require_list(entity["tags"], f"{path}.tags")
    if len(tags) > MAX_TAGS:
        raise ManifestValidationError(f"{path}.tags cannot exceed {MAX_TAGS} values")
    normalized_tags = [
        _require_id(tag, f"{path}.tags[{index}]")
        for index, tag in enumerate(tags)
    ]
    if len(normalized_tags) != len(set(normalized_tags)):
        raise ManifestValidationError(f"{path}.tags must be unique")
    if kind in {"scene", "prop"} and not (visual_id or collision_id):
        raise ManifestValidationError(
            f"{path} kind {kind!r} must reference at least one asset"
        )
    if visual_id is not None:
        asset = assets_by_id.get(visual_id)
        if asset is None or asset["role"] != VISUAL_ROLE:
            raise ManifestValidationError(
                f"{path}.visual_asset_id must reference a visual asset"
            )
    collision_asset = None
    if collision_id is not None:
        collision_asset = assets_by_id.get(collision_id)
        if collision_asset is None or collision_asset["role"] != COLLISION_ROLE:
            raise ManifestValidationError(
                f"{path}.collision_asset_id must reference a collision asset"
            )
    if visible and visual_id is None:
        raise ManifestValidationError(f"{path}.visible requires visual_asset_id")
    if collision_enabled and collision_id is None:
        raise ManifestValidationError(
            f"{path}.collision_enabled requires collision_asset_id"
        )
    if physics_mode != "none" and collision_id is None:
        raise ManifestValidationError(
            f"{path}.physics_mode {physics_mode!r} requires collision_asset_id"
        )
    if physics_mode == "none" and collision_enabled:
        raise ManifestValidationError(
            f"{path}.physics_mode 'none' cannot enable collision"
        )
    if (
        (collision_enabled or physics_mode != "none")
        and collision_asset is not None
        and collision_asset["backend"] != "mujoco_mesh"
    ):
        raise ManifestValidationError(
            f"{path} cannot enable Matrix physics from backend "
            f"{collision_asset['backend']!r}; compile a mujoco_mesh first"
        )


def _canonical_scene(scene: dict[str, Any]) -> dict[str, Any]:
    return {
        "id": scene["id"],
        "coordinate_frame": {
            "id": scene["coordinate_frame"]["id"],
            "meters_per_unit": _require_number(
                scene["coordinate_frame"]["meters_per_unit"],
                "canonical.coordinate_frame.meters_per_unit",
            ),
            "up_axis": scene["coordinate_frame"]["up_axis"],
            "handedness": scene["coordinate_frame"]["handedness"],
        },
        "asset_references": sorted(
            (
                {
                    "id": asset["id"],
                    "role": asset["role"],
                    "backend": asset["backend"],
                    "locator": {
                        "scheme": asset["locator"]["scheme"],
                        "value": asset["locator"]["value"],
                    },
                    "media_type": asset["media_type"],
                    "content_hash": {
                        "algorithm": DIGEST_ALGORITHM,
                        "digest": asset["content_hash"]["digest"],
                    },
                    "size_bytes": asset["size_bytes"],
                    "source_selector": asset["source_selector"],
                    "derived_from_asset_id": asset["derived_from_asset_id"],
                }
                for asset in scene["asset_references"]
            ),
            key=lambda item: item["id"],
        ),
        "transforms": sorted(
            (
                {
                    "id": transform["id"],
                    "translation": [
                        _require_number(value, "canonical.translation")
                        for value in transform["translation"]
                    ],
                    "rotation_xyzw": [
                        _require_number(value, "canonical.rotation_xyzw")
                        for value in transform["rotation_xyzw"]
                    ],
                    "scale": [
                        _require_number(value, "canonical.scale")
                        for value in transform["scale"]
                    ],
                }
                for transform in scene["transforms"]
            ),
            key=lambda item: item["id"],
        ),
        "placed_entities": sorted(
            (
                {
                    "id": entity["id"],
                    "kind": entity["kind"],
                    "transform_id": entity["transform_id"],
                    "visual_asset_id": entity["visual_asset_id"],
                    "collision_asset_id": entity["collision_asset_id"],
                    "physics_mode": entity["physics_mode"],
                    "visible": entity["visible"],
                    "collision_enabled": entity["collision_enabled"],
                    "tags": sorted(entity["tags"]),
                }
                for entity in scene["placed_entities"]
            ),
            key=lambda item: item["id"],
        ),
    }


def _validate_digest_meta(value: Any, path: str) -> dict[str, str]:
    value = _require_object(value, path)
    _require_exact_keys(value, {"algorithm", "digest"}, path)
    _require_equal(value["algorithm"], DIGEST_ALGORITHM, f"{path}.algorithm")
    digest = _require_string(value["digest"], f"{path}.digest")
    if DIGEST_RE.fullmatch(digest) is None:
        raise ManifestValidationError(
            f"{path}.digest must be a lowercase sha256 hex digest"
        )
    return {"algorithm": DIGEST_ALGORITHM, "digest": digest}


def _stored(path: Path, document: dict[str, Any], *, recovered: bool) -> StoredSceneManifest:
    return StoredSceneManifest(
        path=path,
        document=document,
        generation=document["storage"]["generation"],
        scene_digest=document["storage"]["scene_digest"]["digest"],
        store_digest=document["storage"]["store_digest"]["digest"],
        revision_id=document["storage"]["revision_id"],
        recovered_from_backup=recovered,
    )


def _reserve_id(identifier: str, used_ids: set[str]) -> None:
    if identifier in used_ids:
        raise ManifestValidationError(f"duplicate id {identifier!r}")
    used_ids.add(identifier)


def _backup_path(path: Path) -> Path:
    return path.with_name(path.name + ".bak")


def _lock_path(path: Path) -> Path:
    return path.with_name(path.name + ".lock")


def _validate_primary_store_path(path: Path) -> None:
    if not path.name:
        raise ManifestValidationError(
            f"scene primary path must name a file below its parent: {path}"
        )
    if path.name.endswith((".bak", ".lock")):
        raise ManifestValidationError(
            f"scene primary path uses a reserved sidecar suffix: {path}"
        )


def _reject_symlink(path: Path) -> None:
    try:
        mode = path.lstat().st_mode
    except FileNotFoundError:
        return
    if stat.S_ISLNK(mode):
        raise ManifestValidationError(f"scene path cannot be a symlink: {path}")


def _read_bytes_secure(path: Path) -> bytes:
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_NONBLOCK", 0)
    try:
        fd = os.open(path, flags)
    except FileNotFoundError:
        raise
    except OSError as exc:
        raise ManifestIOError(f"cannot open scene securely: {path}: {exc}") from exc
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode):
            raise ManifestIOError(f"scene is not a regular file: {path}")
        if info.st_size < 1 or info.st_size > MAX_STORE_BYTES:
            raise ManifestValidationError(
                f"scene size must be in [1, {MAX_STORE_BYTES}] bytes: {path}"
            )
        chunks: list[bytes] = []
        remaining = MAX_STORE_BYTES + 1
        while remaining:
            chunk = os.read(fd, min(1024 * 1024, remaining))
            if not chunk:
                break
            chunks.append(chunk)
            remaining -= len(chunk)
        payload = b"".join(chunks)
        if len(payload) < 1 or len(payload) > MAX_STORE_BYTES:
            raise ManifestValidationError(
                f"scene size must be in [1, {MAX_STORE_BYTES}] bytes: {path}"
            )
        return payload
    except SceneManifestError:
        raise
    except OSError as exc:
        raise ManifestIOError(f"cannot read scene securely: {path}: {exc}") from exc
    finally:
        os.close(fd)


@contextmanager
def _store_lock(path: Path, *, exclusive: bool) -> Iterator[None]:
    path.parent.mkdir(parents=True, exist_ok=True)
    lock_path = _lock_path(path)
    _reject_symlink(lock_path)
    flags = os.O_RDWR | os.O_CREAT | getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    fd = os.open(lock_path, flags, 0o600)
    try:
        os.fchmod(fd, 0o600)
        fcntl.flock(fd, fcntl.LOCK_EX if exclusive else fcntl.LOCK_SH)
        yield
    finally:
        try:
            fcntl.flock(fd, fcntl.LOCK_UN)
        finally:
            os.close(fd)


def _atomic_write_bytes(path: Path, data: bytes) -> None:
    if not data or len(data) > MAX_STORE_BYTES:
        raise ManifestValidationError("encoded scene has invalid size")
    path.parent.mkdir(parents=True, exist_ok=True)
    _reject_symlink(path)
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="wb",
            dir=path.parent,
            prefix=f".{path.name}.tmp.",
            suffix=f".{os.getpid()}.{uuid4().hex}",
            delete=False,
        ) as handle:
            temporary = Path(handle.name)
            os.fchmod(handle.fileno(), 0o600)
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        temporary = None
        _fsync_dir(path.parent)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def _fsync_dir(path: Path) -> None:
    fd = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def _uri_path(uri: str) -> Path:
    uri = _require_string(uri, "asset.uri")
    if "@" in uri:
        raise ManifestValidationError("asset.uri cannot contain '@' in USD refs")
    if uri.startswith("file://"):
        parsed = urlparse(uri)
        if (
            parsed.scheme != "file"
            or parsed.netloc not in {"", "localhost"}
            or parsed.query
            or parsed.fragment
            or parsed.params
        ):
            raise ManifestValidationError(f"unsupported file URI: {uri}")
        path_text = unquote(parsed.path)
    else:
        path_text = uri
    if "@" in path_text or any(
        ord(character) < 0x20 or ord(character) == 0x7F
        for character in path_text
    ):
        raise ManifestValidationError("asset.uri decodes to an unsafe path")
    path = Path(path_text)
    if not path.is_absolute():
        raise ManifestValidationError(f"asset.uri must resolve to an absolute path: {uri}")
    return path


def _canonical_allowed_roots(roots: Iterable[Path]) -> tuple[Path, ...]:
    canonical: list[Path] = []
    for root in roots:
        candidate = Path(root)
        if not candidate.is_absolute():
            raise AssetVerificationError(f"asset allowlist root is not absolute: {root}")
        try:
            resolved = candidate.resolve(strict=True)
        except (OSError, RuntimeError) as exc:
            raise AssetVerificationError(
                f"asset allowlist root is unavailable: {candidate}: {exc}"
            ) from exc
        if not resolved.is_dir():
            raise AssetVerificationError(
                f"asset allowlist root is not a directory: {resolved}"
            )
        canonical.append(resolved)
    if not canonical:
        raise AssetVerificationError("at least one asset allowlist root is required")
    return tuple(canonical)


def _asset_location_beneath(
    path: Path, roots: tuple[Path, ...]
) -> tuple[Path, Path]:
    for root in roots:
        try:
            relative = path.relative_to(root)
        except ValueError:
            continue
        if relative.parts and all(part not in {"", ".", ".."} for part in relative.parts):
            return root, relative
    raise AssetVerificationError(f"asset is outside configured allowlist roots: {path}")


def _open_sha256_regular_file(
    root: Path,
    relative: Path,
    *,
    display_path: Path,
    expected_size: int,
    chunk_size: int = 8 * 1024 * 1024,
) -> tuple[int, int, str]:
    nofollow = getattr(os, "O_NOFOLLOW", None)
    directory = getattr(os, "O_DIRECTORY", None)
    nonblock = getattr(os, "O_NONBLOCK", None)
    if nofollow is None or directory is None or nonblock is None:
        raise AssetVerificationError(
            "secure asset verification requires O_NOFOLLOW, O_DIRECTORY, and O_NONBLOCK"
        )
    directory_flags = os.O_RDONLY | nofollow | directory
    directory_flags |= getattr(os, "O_CLOEXEC", 0)
    try:
        current_fd = os.open(root, directory_flags)
    except OSError as exc:
        raise AssetVerificationError(
            f"asset allowlist root is unavailable: {root}: {exc}"
        ) from exc
    asset_fd: int | None = None
    keep_asset_fd = False
    try:
        for component in relative.parts[:-1]:
            next_fd = os.open(component, directory_flags, dir_fd=current_fd)
            os.close(current_fd)
            current_fd = next_fd
        file_flags = os.O_RDONLY | nofollow | nonblock
        file_flags |= getattr(os, "O_CLOEXEC", 0)
        asset_fd = os.open(relative.parts[-1], file_flags, dir_fd=current_fd)
        before = os.fstat(asset_fd)
        if not stat.S_ISREG(before.st_mode):
            raise AssetVerificationError(
                f"asset is not a regular file: {display_path}"
            )
        if before.st_size != expected_size:
            raise AssetVerificationError(
                f"asset size mismatch for {display_path}: "
                f"expected={expected_size} actual={before.st_size}"
            )
        digest = hashlib.sha256()
        while chunk := os.read(asset_fd, chunk_size):
            digest.update(chunk)
        after = os.fstat(asset_fd)
        if (before.st_size, before.st_mtime_ns) != (
            after.st_size,
            after.st_mtime_ns,
        ):
            raise AssetVerificationError(
                f"asset changed during verification: {display_path}"
            )
        os.lseek(asset_fd, 0, os.SEEK_SET)
        keep_asset_fd = True
        return asset_fd, after.st_size, digest.hexdigest()
    except AssetVerificationError:
        raise
    except OSError as exc:
        raise AssetVerificationError(
            f"cannot securely verify asset {display_path}: {exc}"
        ) from exc
    finally:
        if asset_fd is not None and not keep_asset_fd:
            os.close(asset_fd)
        os.close(current_fd)


def _store_metadata_digest(
    *,
    generation: int,
    revision_id: str,
    scene_digest_value: str,
    created_at: str,
    updated_at: str,
) -> str:
    payload = {
        "canonicalization": STORE_DIGEST_CANONICALIZATION,
        "created_at": created_at,
        "generation": generation,
        "revision_id": revision_id,
        "schema": SCHEMA_ID,
        "scene_digest": scene_digest_value,
        "updated_at": updated_at,
    }
    encoded = json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _require_object(value: Any, path: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ManifestValidationError(f"{path} must be an object")
    return value


def _require_exact_keys(value: dict[str, Any], expected: set[str], path: str) -> None:
    actual = set(value)
    missing = sorted(expected - actual)
    unknown = sorted(actual - expected)
    if missing:
        raise ManifestValidationError(f"{path} missing keys: {', '.join(missing)}")
    if unknown:
        raise ManifestValidationError(f"{path} unknown keys: {', '.join(unknown)}")


def _require_equal(value: Any, expected: str, path: str) -> None:
    if value != expected:
        raise ManifestValidationError(f"{path} must be {expected!r}")


def _require_string(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value:
        raise ManifestValidationError(f"{path} must be a non-empty string")
    if any(ord(character) < 0x20 or ord(character) == 0x7F for character in value):
        raise ManifestValidationError(f"{path} contains a control character")
    return value


def _require_int(value: Any, path: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ManifestValidationError(f"{path} must be an integer")
    return value


def _require_bool(value: Any, path: str) -> bool:
    if not isinstance(value, bool):
        raise ManifestValidationError(f"{path} must be a boolean")
    return value


def _require_id(value: Any, path: str) -> str:
    identifier = _require_string(value, path)
    if ID_RE.fullmatch(identifier) is None:
        raise ManifestValidationError(f"{path} has invalid id syntax")
    return identifier


def _require_optional_id(value: Any, path: str) -> str | None:
    return None if value is None else _require_id(value, path)


def _require_revision_id(value: Any, path: str) -> str:
    revision = _require_string(value, path)
    if REVISION_RE.fullmatch(revision) is None:
        raise ManifestValidationError(f"{path} must be rev- plus 32 lowercase hex digits")
    return revision


def _require_list(value: Any, path: str) -> list[Any]:
    if not isinstance(value, list):
        raise ManifestValidationError(f"{path} must be a list")
    return value


def _require_enum(value: Any, allowed: frozenset[str], path: str) -> str:
    text = _require_string(value, path)
    if text not in allowed:
        raise ManifestValidationError(
            f"{path} must be one of: {', '.join(sorted(allowed))}"
        )
    return text


def _require_number(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ManifestValidationError(f"{path} must be a finite number")
    try:
        number = float(value)
    except (OverflowError, ValueError) as exc:
        raise ManifestValidationError(f"{path} must be a finite number") from exc
    if not math.isfinite(number):
        raise ManifestValidationError(f"{path} must be a finite number")
    return 0.0 if number == 0.0 else number


def _require_number_list(value: Any, length: int, path: str) -> list[float]:
    items = _require_list(value, path)
    if len(items) != length:
        raise ManifestValidationError(f"{path} must contain exactly {length} numbers")
    return [
        _require_number(item, f"{path}[{index}]")
        for index, item in enumerate(items)
    ]


def _require_timestamp(value: Any, path: str) -> datetime:
    text = _require_string(value, path)
    try:
        parsed = datetime.fromisoformat(text)
    except ValueError as exc:
        raise ManifestValidationError(f"{path} must be ISO-8601") from exc
    if parsed.tzinfo is None:
        raise ManifestValidationError(f"{path} must include timezone")
    return parsed


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    validate_input = commands.add_parser("validate-input")
    validate_input.add_argument("path", type=Path)
    validate_store = commands.add_parser("validate-store")
    validate_store.add_argument("path", type=Path)
    inspect = commands.add_parser("inspect")
    inspect.add_argument("path", type=Path)
    for name in ("write", "update"):
        command = commands.add_parser(name)
        command.add_argument("path", type=Path)
        command.add_argument("--input", type=Path, required=True)
        command.add_argument("--expected-generation", type=int, required=True)
        command.add_argument(
            "--expected-store-digest",
            required=name == "update",
        )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "validate-input":
            document = extract_scene_input(load_json_strict(args.path))
            print(
                json.dumps(
                    {
                        "ok": True,
                        "scene_digest": scene_digest(document),
                        "assets": len(document["scene"]["asset_references"]),
                        "entities": len(document["scene"]["placed_entities"]),
                    },
                    sort_keys=True,
                )
            )
            return 0
        if args.command == "validate-store":
            stored = read_store(args.path)
            print(
                json.dumps(
                    {
                        "ok": True,
                        "generation": stored.generation,
                        "scene_digest": stored.scene_digest,
                        "store_digest": stored.store_digest,
                        "revision_id": stored.revision_id,
                        "recovered_from_backup": stored.recovered_from_backup,
                        "degraded": stored.recovered_from_backup,
                    },
                    sort_keys=True,
                )
            )
            return 0
        if args.command == "inspect":
            print(json.dumps(inspect_store(args.path), sort_keys=True))
            return 0
        if args.command in {"write", "update"}:
            document = extract_scene_input(load_json_strict(args.input))
            stored = write_store(
                args.path,
                document,
                expected_generation=args.expected_generation,
                expected_store_digest=args.expected_store_digest,
            )
            print(
                json.dumps(
                    {
                        "ok": True,
                        "path": str(args.path),
                        "generation": stored.generation,
                        "scene_digest": stored.scene_digest,
                        "store_digest": stored.store_digest,
                        "revision_id": stored.revision_id,
                    },
                    sort_keys=True,
                )
            )
            return 0
    except ManifestConflictError as exc:
        print(f"scene-manifest CAS conflict: {exc}", file=sys.stderr)
        return 3
    except (OSError, SceneManifestError) as exc:
        print(f"scene-manifest validation failed: {exc}", file=sys.stderr)
        return 1
    return 2


__all__ = [
    "ASSET_ROLES",
    "AssetVerificationError",
    "CANONICALIZATION",
    "COLLISION_ROLE",
    "BACKENDS_BY_ROLE",
    "DIGEST_ALGORITHM",
    "SCHEMA_ID",
    "STORE_DIGEST_CANONICALIZATION",
    "ManifestConflictError",
    "ManifestIOError",
    "ManifestValidationError",
    "StoredSceneManifest",
    "VerifiedAssetHandle",
    "VISUAL_ROLE",
    "canonical_scene_bytes",
    "extract_scene_input",
    "inspect_store",
    "load_json_strict",
    "loads_json_strict",
    "open_verified_asset_references",
    "read_store",
    "scene_digest",
    "store_document",
    "store_document_bytes",
    "validate_scene_document",
    "validate_store_document",
    "write_store",
]


if __name__ == "__main__":
    raise SystemExit(main())
