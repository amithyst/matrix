#!/usr/bin/env python3
"""Import immutable Matrix item packs from strict, recipe-driven source trees.

The importer copies bytes without conversion.  Recipes name files relative to
an explicit source root and map them to paths inside a content-addressed pack.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import errno
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import stat
import sys
import tempfile
from typing import Any, Iterable


SCRIPT_DIRECTORY = Path(__file__).resolve().parent
if str(SCRIPT_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIRECTORY))

import matrix_item_asset_pack as asset_pack


IMPORT_RECIPE_SCHEMA = "matrix-item-asset-import-recipe/v1"
IMPORT_RESULT_SCHEMA = "matrix-item-asset-import-result/v1"
MAX_RECIPE_BYTES = 2 * 1024 * 1024


class ItemAssetImportError(ValueError):
    """Fail-closed item asset import error."""


class ItemAssetRecipeError(ItemAssetImportError):
    pass


class ItemAssetInstallError(ItemAssetImportError):
    pass


@dataclass(frozen=True)
class RecipeFile:
    file_id: str
    source_path: PurePosixPath
    target_path: PurePosixPath
    role: str
    media_type: str
    format: str


@dataclass(frozen=True)
class ImportRecipe:
    path: Path
    pack: dict[str, Any]
    files: tuple[RecipeFile, ...]


@dataclass(frozen=True)
class ImportResult:
    pack_id: str
    digest: str
    manifest_path: Path
    created: bool

    @property
    def digest_ref(self) -> str:
        return f"sha256:{self.digest}"

    @property
    def idempotent(self) -> bool:
        return not self.created

    def summary(self) -> dict[str, Any]:
        return {
            "schema": IMPORT_RESULT_SCHEMA,
            "pack_id": self.pack_id,
            "digest_ref": self.digest_ref,
            "manifest_path": str(self.manifest_path),
            "created": self.created,
            "idempotent": self.idempotent,
        }


def load_import_recipe(recipe_path: Path) -> ImportRecipe:
    """Load and semantically validate one strict import recipe."""

    absolute = asset_pack._absolute_path(recipe_path)
    asset_pack._require_plain_directory(absolute.parent, "recipe directory")
    payload = asset_pack._read_regular_file(
        absolute,
        maximum_bytes=MAX_RECIPE_BYTES,
        description="item asset import recipe",
    )
    try:
        text = payload.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ItemAssetRecipeError(f"{absolute}: recipe is not UTF-8") from exc
    try:
        document = asset_pack.loads_json_strict(text, source=str(absolute))
        return _validate_recipe_document(document, path=absolute)
    except asset_pack.ItemAssetPackError as exc:
        raise ItemAssetRecipeError(str(exc)) from exc


def import_item_asset_pack(
    recipe_path: Path,
    source_root: Path,
    registry_root: Path,
) -> ImportResult:
    """Copy, hash, validate, and atomically install one recipe-defined pack."""

    recipe = load_import_recipe(recipe_path)
    source_root = asset_pack._absolute_path(source_root)
    registry_root = asset_pack._absolute_path(registry_root)
    try:
        asset_pack._require_plain_directory(source_root, "import source root")
        _ensure_registry_root(registry_root)
    except asset_pack.ItemAssetPackError as exc:
        raise ItemAssetInstallError(str(exc)) from exc

    staging_path = Path(
        tempfile.mkdtemp(prefix=".matrix-item-import-", dir=registry_root)
    )
    try:
        file_documents: list[dict[str, Any]] = []
        for entry in recipe.files:
            destination = staging_path.joinpath(*entry.target_path.parts)
            destination.parent.mkdir(parents=True, exist_ok=True)
            size_bytes, sha256 = _copy_and_hash_source(
                source_root=source_root,
                source_path=entry.source_path,
                destination=destination,
                description=f"recipe file {entry.file_id!r}",
            )
            file_documents.append(
                {
                    "file_id": entry.file_id,
                    "path": entry.target_path.as_posix(),
                    "size_bytes": size_bytes,
                    "sha256": sha256,
                    "role": entry.role,
                    "media_type": entry.media_type,
                    "format": entry.format,
                }
            )

        candidate = {
            "schema": asset_pack.ASSET_PACK_SCHEMA,
            "pack": {**recipe.pack, "files": file_documents},
        }
        normalized = _normalized_asset_pack(candidate)
        digest = asset_pack.asset_pack_digest(normalized)
        manifest_path = staging_path / asset_pack.REGISTRY_MANIFEST_NAME
        _write_manifest(manifest_path, normalized)
        _fsync_tree_directories(staging_path)

        try:
            staged_pack = asset_pack.load_asset_pack(manifest_path)
        except asset_pack.ItemAssetPackError as exc:
            raise ItemAssetInstallError(
                f"staged pack verification failed: {exc}"
            ) from exc
        if staged_pack.digest != digest:
            raise ItemAssetInstallError(
                "staged pack digest changed during verification"
            )

        prefix_directory = registry_root / "sha256" / digest[:2]
        _ensure_plain_child_directory(registry_root, registry_root / "sha256")
        _ensure_plain_child_directory(
            registry_root / "sha256", prefix_directory
        )
        destination_root = prefix_directory / digest
        destination_manifest = (
            destination_root / asset_pack.REGISTRY_MANIFEST_NAME
        )
        expected_paths = {
            entry.target_path.as_posix() for entry in recipe.files
        }

        if _path_exists_without_following(destination_root):
            _verify_existing_install(
                destination_root=destination_root,
                expected_document=normalized,
                expected_asset_paths=expected_paths,
                expected_digest=digest,
            )
            return ImportResult(
                pack_id=normalized["pack"]["pack_id"],
                digest=digest,
                manifest_path=destination_manifest,
                created=False,
            )

        try:
            os.rename(staging_path, destination_root)
        except OSError as exc:
            if exc.errno not in {
                errno.EEXIST,
                errno.ENOTEMPTY,
                errno.EISDIR,
            }:
                raise ItemAssetInstallError(
                    f"cannot atomically install pack {digest}: {exc}"
                ) from exc
            _verify_existing_install(
                destination_root=destination_root,
                expected_document=normalized,
                expected_asset_paths=expected_paths,
                expected_digest=digest,
            )
            return ImportResult(
                pack_id=normalized["pack"]["pack_id"],
                digest=digest,
                manifest_path=destination_manifest,
                created=False,
            )

        staging_path = Path()
        _fsync_directory(prefix_directory)
        return ImportResult(
            pack_id=normalized["pack"]["pack_id"],
            digest=digest,
            manifest_path=destination_manifest,
            created=True,
        )
    except (asset_pack.ItemAssetPackError, OSError) as exc:
        if isinstance(exc, ItemAssetImportError):
            raise
        raise ItemAssetInstallError(str(exc)) from exc
    finally:
        if staging_path != Path() and _path_exists_without_following(staging_path):
            shutil.rmtree(staging_path)


def _validate_recipe_document(
    document: Any,
    *,
    path: Path,
) -> ImportRecipe:
    root = _require_object(document, "document")
    _require_exact_keys(root, {"schema", "import"}, "document")
    if root["schema"] != IMPORT_RECIPE_SCHEMA:
        raise ItemAssetRecipeError(
            f"schema must equal {IMPORT_RECIPE_SCHEMA!r}"
        )
    body = _require_object(root["import"], "import")
    _require_exact_keys(body, {"pack", "files"}, "import")
    pack = _require_object(body["pack"], "import.pack")
    _require_exact_keys(
        pack,
        {
            "pack_id",
            "revision",
            "license",
            "provenance",
            "coordinate_frame",
            "items",
        },
        "import.pack",
    )
    raw_files = body["files"]
    if (
        not isinstance(raw_files, list)
        or not 1 <= len(raw_files) <= asset_pack.MAX_FILES
    ):
        raise ItemAssetRecipeError(
            f"import.files must contain 1..{asset_pack.MAX_FILES} entries"
        )

    files: list[RecipeFile] = []
    seen_sources: set[str] = set()
    target_paths: list[PurePosixPath] = []
    placeholder_files: list[dict[str, Any]] = []
    for index, value in enumerate(raw_files):
        name = f"import.files[{index}]"
        entry = _require_object(value, name)
        _require_exact_keys(
            entry,
            {
                "file_id",
                "source_path",
                "target_path",
                "role",
                "media_type",
                "format",
            },
            name,
        )
        try:
            source_path = asset_pack._safe_relative_path(
                entry["source_path"], f"{name}.source_path"
            )
            target_path = asset_pack._safe_relative_path(
                entry["target_path"], f"{name}.target_path"
            )
        except asset_pack.ItemAssetPackError as exc:
            raise ItemAssetRecipeError(str(exc)) from exc
        if target_path.parts[0] == asset_pack.REGISTRY_MANIFEST_NAME:
            raise ItemAssetRecipeError(
                f"{name}.target_path conflicts with the generated manifest"
            )
        source_text = source_path.as_posix()
        if source_text in seen_sources:
            raise ItemAssetRecipeError(
                f"duplicate source_path {source_text!r}"
            )
        seen_sources.add(source_text)
        target_paths.append(target_path)
        files.append(
            RecipeFile(
                file_id=entry["file_id"],
                source_path=source_path,
                target_path=target_path,
                role=entry["role"],
                media_type=entry["media_type"],
                format=entry["format"],
            )
        )
        placeholder_files.append(
            {
                "file_id": entry["file_id"],
                "path": target_path.as_posix(),
                "size_bytes": 1,
                "sha256": "0" * 64,
                "role": entry["role"],
                "media_type": entry["media_type"],
                "format": entry["format"],
            }
        )

    _reject_target_path_conflicts(target_paths)
    placeholder = {
        "schema": asset_pack.ASSET_PACK_SCHEMA,
        "pack": {**pack, "files": placeholder_files},
    }
    try:
        normalized = _normalized_asset_pack(placeholder)
    except asset_pack.ItemAssetPackError as exc:
        raise ItemAssetRecipeError(str(exc)) from exc
    normalized_pack = dict(normalized["pack"])
    normalized_pack.pop("files")
    return ImportRecipe(path=path, pack=normalized_pack, files=tuple(files))


def _normalized_asset_pack(document: Any) -> dict[str, Any]:
    canonical = asset_pack.canonical_asset_pack_bytes(document)
    result = json.loads(canonical.decode("utf-8"))
    if not isinstance(result, dict):  # pragma: no cover - canonical contract.
        raise AssertionError("canonical pack document is not an object")
    return result


def _copy_and_hash_source(
    *,
    source_root: Path,
    source_path: PurePosixPath,
    destination: Path,
    description: str,
) -> tuple[int, str]:
    source = asset_pack._asset_path_without_symlinks(
        source_root, source_path
    )
    source_descriptor, opened = asset_pack._open_regular_file(
        source,
        maximum_bytes=asset_pack.MAX_ASSET_BYTES,
        description=description,
    )
    destination_flags = (
        os.O_WRONLY
        | os.O_CREAT
        | os.O_EXCL
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    try:
        destination_descriptor = os.open(destination, destination_flags, 0o644)
    except OSError:
        os.close(source_descriptor)
        raise
    digest = hashlib.sha256()
    bytes_read = 0
    try:
        while True:
            chunk = os.read(source_descriptor, 1024 * 1024)
            if not chunk:
                break
            bytes_read += len(chunk)
            if bytes_read > asset_pack.MAX_ASSET_BYTES:
                raise ItemAssetInstallError(
                    f"{description} exceeds the asset size limit"
                )
            digest.update(chunk)
            _write_all(destination_descriptor, chunk)
        asset_pack._verify_open_file_stable(
            source_descriptor,
            opened,
            bytes_read=bytes_read,
            maximum_bytes=asset_pack.MAX_ASSET_BYTES,
            description=description,
            path=source,
        )
        os.fsync(destination_descriptor)
    finally:
        os.close(destination_descriptor)
        os.close(source_descriptor)
    return bytes_read, digest.hexdigest()


def _write_all(descriptor: int, payload: bytes) -> None:
    view = memoryview(payload)
    while view:
        written = os.write(descriptor, view)
        if written <= 0:  # pragma: no cover - os.write raises on failure.
            raise ItemAssetInstallError("short write while staging asset")
        view = view[written:]


def _write_manifest(path: Path, document: dict[str, Any]) -> None:
    payload = (
        json.dumps(
            document,
            sort_keys=True,
            indent=2,
            ensure_ascii=True,
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")
    flags = (
        os.O_WRONLY
        | os.O_CREAT
        | os.O_EXCL
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    descriptor = os.open(path, flags, 0o644)
    try:
        _write_all(descriptor, payload)
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    _fsync_directory(path.parent)


def _ensure_registry_root(path: Path) -> None:
    if _path_exists_without_following(path):
        asset_pack._require_plain_directory(path, "registry root")
        return
    asset_pack._require_plain_directory(path.parent, "registry parent")
    try:
        path.mkdir(mode=0o755)
    except FileExistsError:
        pass
    asset_pack._require_plain_directory(path, "registry root")
    _fsync_directory(path.parent)


def _ensure_plain_child_directory(parent: Path, child: Path) -> None:
    asset_pack._require_plain_directory(parent, "registry parent")
    if child.parent != parent:
        raise ItemAssetInstallError(
            f"registry child {child} is not directly beneath {parent}"
        )
    created = False
    if not _path_exists_without_following(child):
        try:
            child.mkdir(mode=0o755)
            created = True
        except FileExistsError:
            pass
    asset_pack._require_plain_directory(child, "registry directory")
    if created:
        _fsync_directory(parent)


def _verify_existing_install(
    *,
    destination_root: Path,
    expected_document: dict[str, Any],
    expected_asset_paths: set[str],
    expected_digest: str,
) -> None:
    expected_files = set(expected_asset_paths)
    expected_files.add(asset_pack.REGISTRY_MANIFEST_NAME)
    expected_directories: set[str] = set()
    for path_text in expected_asset_paths:
        parent = PurePosixPath(path_text).parent
        while parent != PurePosixPath("."):
            expected_directories.add(parent.as_posix())
            parent = parent.parent

    actual_files, actual_directories = _strict_tree_entries(destination_root)
    if actual_files != expected_files or actual_directories != expected_directories:
        raise ItemAssetInstallError(
            f"existing registry entry sha256:{expected_digest} has different "
            "tree content"
        )
    manifest = destination_root / asset_pack.REGISTRY_MANIFEST_NAME
    try:
        existing = asset_pack.load_asset_pack(manifest)
        raw = asset_pack._read_regular_file(
            manifest,
            maximum_bytes=asset_pack.MAX_MANIFEST_BYTES,
            description="existing registry manifest",
        ).decode("utf-8")
        existing_document = asset_pack.loads_json_strict(
            raw, source=str(manifest)
        )
        existing_canonical = asset_pack.canonical_asset_pack_bytes(
            existing_document
        )
        expected_canonical = asset_pack.canonical_asset_pack_bytes(
            expected_document
        )
    except (asset_pack.ItemAssetPackError, UnicodeDecodeError) as exc:
        raise ItemAssetInstallError(
            f"existing registry entry sha256:{expected_digest} is invalid: {exc}"
        ) from exc
    if (
        existing.digest != expected_digest
        or existing_canonical != expected_canonical
    ):
        raise ItemAssetInstallError(
            f"existing registry entry sha256:{expected_digest} has different "
            "manifest content"
        )


def _strict_tree_entries(root: Path) -> tuple[set[str], set[str]]:
    try:
        asset_pack._require_plain_directory(root, "registry pack root")
    except asset_pack.ItemAssetPackError as exc:
        raise ItemAssetInstallError(str(exc)) from exc
    files: set[str] = set()
    directories: set[str] = set()

    def visit(directory: Path, relative_parent: PurePosixPath) -> None:
        try:
            entries = list(os.scandir(directory))
        except OSError as exc:
            raise ItemAssetInstallError(
                f"cannot inspect registry pack tree {directory}: {exc}"
            ) from exc
        for entry in entries:
            relative = relative_parent / entry.name
            try:
                metadata = entry.stat(follow_symlinks=False)
            except OSError as exc:
                raise ItemAssetInstallError(
                    f"cannot inspect registry entry {entry.path}: {exc}"
                ) from exc
            if stat.S_ISLNK(metadata.st_mode):
                raise ItemAssetInstallError(
                    f"existing registry entry contains symlink: {entry.path}"
                )
            if stat.S_ISDIR(metadata.st_mode):
                directories.add(relative.as_posix())
                visit(Path(entry.path), relative)
            elif stat.S_ISREG(metadata.st_mode):
                files.add(relative.as_posix())
            else:
                raise ItemAssetInstallError(
                    f"existing registry entry is not regular: {entry.path}"
                )

    visit(root, PurePosixPath())
    return files, directories


def _reject_target_path_conflicts(paths: list[PurePosixPath]) -> None:
    for index, left in enumerate(paths):
        for right in paths[index + 1 :]:
            if left == right:
                raise ItemAssetRecipeError(
                    f"duplicate target_path {left.as_posix()!r}"
                )
            if (
                left.parts == right.parts[: len(left.parts)]
                or right.parts == left.parts[: len(right.parts)]
            ):
                raise ItemAssetRecipeError(
                    f"target paths conflict as file/directory: "
                    f"{left.as_posix()!r}, {right.as_posix()!r}"
                )


def _require_object(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ItemAssetRecipeError(f"{name} must be an object")
    return value


def _require_exact_keys(
    value: dict[str, Any],
    expected: set[str],
    name: str,
) -> None:
    actual = set(value)
    if actual == expected:
        return
    missing = sorted(expected - actual)
    unknown = sorted(actual - expected)
    details: list[str] = []
    if missing:
        details.append(f"missing={missing}")
    if unknown:
        details.append(f"unknown={unknown}")
    raise ItemAssetRecipeError(
        f"{name} has invalid fields ({', '.join(details)})"
    )


def _path_exists_without_following(path: Path) -> bool:
    try:
        path.lstat()
    except FileNotFoundError:
        return False
    except OSError as exc:
        raise ItemAssetInstallError(f"cannot inspect {path}: {exc}") from exc
    return True


def _fsync_directory(path: Path) -> None:
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
    descriptor = os.open(path, flags)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _fsync_tree_directories(root: Path) -> None:
    """Persist every staged directory entry before the atomic pack rename."""

    directories = [root]
    directories.extend(path for path in root.rglob("*") if path.is_dir())
    for directory in sorted(
        directories,
        key=lambda candidate: len(candidate.relative_to(root).parts),
        reverse=True,
    ):
        _fsync_directory(directory)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    import_parser = subparsers.add_parser(
        "import",
        help="install one recipe-defined immutable item asset pack",
    )
    import_parser.add_argument("recipe", type=Path)
    import_parser.add_argument("--source-root", required=True, type=Path)
    import_parser.add_argument("--registry-root", required=True, type=Path)
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    arguments = _build_parser().parse_args(
        list(argv) if argv is not None else None
    )
    try:
        if arguments.command != "import":  # pragma: no cover - argparse.
            raise AssertionError(arguments.command)
        result = import_item_asset_pack(
            arguments.recipe,
            arguments.source_root,
            arguments.registry_root,
        )
    except (ItemAssetImportError, asset_pack.ItemAssetPackError) as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result.summary(), sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
