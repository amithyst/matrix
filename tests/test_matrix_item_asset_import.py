from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


PACK = _load_module(
    "matrix_item_asset_pack",
    SCRIPTS / "matrix_item_asset_pack.py",
)
IMPORTER = _load_module(
    "matrix_item_asset_import",
    SCRIPTS / "matrix_item_asset_import.py",
)


class MatrixItemAssetImportTest(unittest.TestCase):
    def _mesh_bytes(self) -> bytes:
        return (
            b"solid generic_crate\n"
            b"facet normal 0 0 1\n"
            b"outer loop\n"
            b"vertex 0 0 0\n"
            b"vertex 1 0 0\n"
            b"vertex 0 1 0\n"
            b"endloop\n"
            b"endfacet\n"
            b"endsolid generic_crate\n"
        )

    def _recipe_document(self) -> dict[str, object]:
        return {
            "schema": "matrix-item-asset-import-recipe/v1",
            "import": {
                "pack": {
                    "pack_id": "benchmark.generic-props",
                    "revision": "dataset-v1",
                    "license": {
                        "spdx_id": "CC0-1.0",
                        "attribution": "",
                    },
                    "provenance": {
                        "source_name": "Generic Props Benchmark",
                        "source_uri": "https://example.invalid/generic-props",
                        "source_revision": "release-1",
                        "source_item_ids": ["crate-001"],
                    },
                    "coordinate_frame": {
                        "up_axis": "+Z",
                        "forward_axis": "+X",
                        "handedness": "right",
                        "meters_per_unit": 1.0,
                    },
                    "items": [
                        {
                            "item_id": "crate",
                            "label": "Generic crate",
                            "physics": {
                                "mass_kg": 4.0,
                                "collision": {
                                    "shape": "box",
                                    "half_extents_m": [0.25, 0.25, 0.25],
                                },
                            },
                            "visual_parts": [
                                {
                                    "part_id": "body",
                                    "file_id": "crate_mesh",
                                    "rgba": [0.7, 0.5, 0.3, 1.0],
                                    "scale": [1.0, 1.0, 1.0],
                                    "translation_m": [0.0, 0.0, 0.0],
                                    "rotation_wxyz": [1.0, 0.0, 0.0, 0.0],
                                }
                            ],
                        }
                    ],
                },
                "files": [
                    {
                        "file_id": "crate_mesh",
                        "source_path": "models/crate.stl",
                        "target_path": "meshes/crate.stl",
                        "role": "visual_mesh",
                        "media_type": "model/stl",
                        "format": "stl",
                    },
                    {
                        "file_id": "license_text",
                        "source_path": "LICENSE.txt",
                        "target_path": "evidence/LICENSE.txt",
                        "role": "license",
                        "media_type": "text/plain",
                        "format": "text",
                    },
                    {
                        "file_id": "source_readme",
                        "source_path": "README.md",
                        "target_path": "evidence/README.md",
                        "role": "provenance",
                        "media_type": "text/markdown",
                        "format": "markdown",
                    },
                ],
            },
        }

    def _write_source_tree(self, root: Path) -> dict[str, bytes]:
        payloads = {
            "models/crate.stl": self._mesh_bytes(),
            "LICENSE.txt": b"CC0 1.0 Universal\n",
            "README.md": b"# Generic Props\n\nSource evidence.\n",
        }
        for relative, payload in payloads.items():
            path = root.joinpath(*relative.split("/"))
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(payload)
        return payloads

    def _write_recipe(
        self,
        path: Path,
        document: dict[str, object] | None = None,
    ) -> Path:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(
                self._recipe_document() if document is None else document,
                indent=2,
            ),
            encoding="utf-8",
        )
        return path

    def test_imports_generic_stl_and_evidence_without_conversion(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source = root / "benchmark"
            source.mkdir()
            payloads = self._write_source_tree(source)
            recipe = self._write_recipe(
                root / "repository" / "config" / "items" / "crate.json"
            )
            registry = root / "registry"

            result = IMPORTER.import_item_asset_pack(
                recipe, source, registry
            )

            self.assertTrue(result.created)
            self.assertFalse(result.idempotent)
            self.assertRegex(result.digest_ref, r"^sha256:[0-9a-f]{64}$")
            self.assertEqual(
                result.manifest_path,
                registry
                / "sha256"
                / result.digest[:2]
                / result.digest
                / "matrix-item-asset-pack.json",
            )
            pack = PACK.resolve_registry_pack(
                registry, result.digest_ref
            )
            self.assertEqual(pack.pack_id, "benchmark.generic-props")
            self.assertEqual(
                {entry.role for entry in pack.files},
                {"visual_mesh", "license", "provenance"},
            )
            for entry in pack.files:
                expected = payloads[
                    {
                        "meshes/crate.stl": "models/crate.stl",
                        "evidence/LICENSE.txt": "LICENSE.txt",
                        "evidence/README.md": "README.md",
                    }[entry.relative_path.as_posix()]
                ]
                self.assertEqual(entry.path.read_bytes(), expected)
                self.assertEqual(entry.size_bytes, len(expected))
                self.assertEqual(
                    entry.sha256, hashlib.sha256(expected).hexdigest()
                )
            self.assertEqual(
                list(registry.glob(".matrix-item-import-*")), []
            )

    def test_same_recipe_and_bytes_are_idempotent(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source = root / "source"
            source.mkdir()
            self._write_source_tree(source)
            recipe = self._write_recipe(root / "recipe.json")
            registry = root / "registry"

            first = IMPORTER.import_item_asset_pack(
                recipe, source, registry
            )
            manifest_mtime = first.manifest_path.stat().st_mtime_ns
            second = IMPORTER.import_item_asset_pack(
                recipe, source, registry
            )

            self.assertTrue(first.created)
            self.assertFalse(second.created)
            self.assertTrue(second.idempotent)
            self.assertEqual(first.digest, second.digest)
            self.assertEqual(
                second.manifest_path.stat().st_mtime_ns, manifest_mtime
            )
            self.assertEqual(
                list(registry.glob(".matrix-item-import-*")), []
            )

    def test_existing_tampered_or_extra_content_fails_closed(self) -> None:
        for mutation in ("tamper", "extra"):
            with (
                self.subTest(mutation=mutation),
                tempfile.TemporaryDirectory() as temporary_directory,
            ):
                root = Path(temporary_directory)
                source = root / "source"
                source.mkdir()
                self._write_source_tree(source)
                recipe = self._write_recipe(root / "recipe.json")
                registry = root / "registry"
                result = IMPORTER.import_item_asset_pack(
                    recipe, source, registry
                )
                pack_root = result.manifest_path.parent
                if mutation == "tamper":
                    (pack_root / "meshes" / "crate.stl").write_bytes(
                        b"tampered"
                    )
                else:
                    (pack_root / "unexpected.txt").write_text(
                        "unexpected", encoding="utf-8"
                    )

                with self.assertRaisesRegex(
                    IMPORTER.ItemAssetInstallError,
                    "existing registry entry",
                ):
                    IMPORTER.import_item_asset_pack(
                        recipe, source, registry
                    )
                self.assertEqual(
                    list(registry.glob(".matrix-item-import-*")), []
                )

    def test_strict_json_rejects_duplicate_nan_and_unknown_fields(self) -> None:
        invalid_texts = (
            '{"schema":"matrix-item-asset-import-recipe/v1",'
            '"schema":"matrix-item-asset-import-recipe/v1","import":{}}',
            '{"schema":"matrix-item-asset-import-recipe/v1",'
            '"import":{"pack":{"mass":NaN},"files":[]}}',
        )
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            for index, text in enumerate(invalid_texts):
                with self.subTest(index=index):
                    path = root / f"invalid-{index}.json"
                    path.write_text(text, encoding="utf-8")
                    with self.assertRaisesRegex(
                        IMPORTER.ItemAssetRecipeError,
                        "duplicate key|non-finite",
                    ):
                        IMPORTER.load_import_recipe(path)

            unknown = self._recipe_document()
            unknown["import"]["files"][0]["weapon_type"] = (  # type: ignore[index]
                "hardcoded"
            )
            path = self._write_recipe(root / "unknown.json", unknown)
            with self.assertRaisesRegex(
                IMPORTER.ItemAssetRecipeError, "unknown"
            ):
                IMPORTER.load_import_recipe(path)

    def test_duplicate_source_and_target_path_conflicts_fail_closed(self) -> None:
        mutations: list[dict[str, object]] = []
        duplicate_source = self._recipe_document()
        duplicate_source["import"]["files"][1]["source_path"] = (  # type: ignore[index]
            "models/crate.stl"
        )
        mutations.append(duplicate_source)
        duplicate_target = self._recipe_document()
        duplicate_target["import"]["files"][1]["target_path"] = (  # type: ignore[index]
            "meshes/crate.stl"
        )
        mutations.append(duplicate_target)
        prefix_target = self._recipe_document()
        prefix_target["import"]["files"][1]["target_path"] = (  # type: ignore[index]
            "meshes"
        )
        mutations.append(prefix_target)

        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            for index, document in enumerate(mutations):
                with self.subTest(index=index):
                    path = self._write_recipe(
                        root / f"duplicate-{index}.json", document
                    )
                    with self.assertRaisesRegex(
                        IMPORTER.ItemAssetRecipeError,
                        "duplicate source_path|duplicate target_path|"
                        "target paths conflict",
                    ):
                        IMPORTER.load_import_recipe(path)

    def test_source_traversal_symlink_and_nonregular_fail_closed(self) -> None:
        traversal = self._recipe_document()
        traversal["import"]["files"][0]["source_path"] = (  # type: ignore[index]
            "../crate.stl"
        )
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            path = self._write_recipe(root / "traversal.json", traversal)
            with self.assertRaisesRegex(
                IMPORTER.ItemAssetRecipeError, "safe relative path"
            ):
                IMPORTER.load_import_recipe(path)

        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source = root / "source"
            source.mkdir()
            self._write_source_tree(source)
            recipe = self._write_recipe(root / "recipe.json")
            registry = root / "registry"
            original = source / "models" / "crate.stl"
            outside = root / "outside.stl"
            outside.write_bytes(original.read_bytes())
            original.unlink()
            original.symlink_to(outside)
            with self.assertRaisesRegex(
                IMPORTER.ItemAssetInstallError, "symlink"
            ):
                IMPORTER.import_item_asset_pack(
                    recipe, source, registry
                )

        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source = root / "source"
            source.mkdir()
            self._write_source_tree(source)
            recipe = self._write_recipe(root / "recipe.json")
            registry = root / "registry"
            target = source / "models" / "crate.stl"
            target.unlink()
            target.mkdir()
            with self.assertRaisesRegex(
                IMPORTER.ItemAssetInstallError, "regular file"
            ):
                IMPORTER.import_item_asset_pack(
                    recipe, source, registry
                )

    def test_source_path_is_not_part_of_canonical_pack_digest(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source = root / "source"
            source.mkdir()
            payloads = self._write_source_tree(source)
            alternate = source / "alternate" / "crate.stl"
            alternate.parent.mkdir()
            alternate.write_bytes(payloads["models/crate.stl"])
            registry = root / "registry"
            first_recipe = self._write_recipe(
                root / "first.json"
            )
            first = IMPORTER.import_item_asset_pack(
                first_recipe, source, registry
            )
            document = copy.deepcopy(self._recipe_document())
            document["import"]["files"][0]["source_path"] = (  # type: ignore[index]
                "alternate/crate.stl"
            )
            second_recipe = self._write_recipe(
                root / "second.json", document
            )
            second = IMPORTER.import_item_asset_pack(
                second_recipe, source, registry
            )

            self.assertEqual(first.digest, second.digest)
            self.assertTrue(second.idempotent)

    def test_recipe_schema_is_strict_json_and_closed(self) -> None:
        schema_path = (
            ROOT
            / "config"
            / "schemas"
            / "matrix-item-asset-import-recipe-v1.schema.json"
        )
        schema = PACK.loads_json_strict(
            schema_path.read_text(encoding="utf-8"),
            source=str(schema_path),
        )
        self.assertFalse(schema["additionalProperties"])
        self.assertEqual(
            schema["properties"]["schema"]["const"],
            IMPORTER.IMPORT_RECIPE_SCHEMA,
        )
        self.assertFalse(
            schema["$defs"]["sourceFile"]["additionalProperties"]
        )


if __name__ == "__main__":
    unittest.main()
