from __future__ import annotations

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "matrix_item_asset_pack",
        ROOT / "scripts" / "matrix_item_asset_pack.py",
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    import sys

    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


PACK = _load_module()


class MatrixItemAssetPackTest(unittest.TestCase):
    def _asset_bytes(self) -> bytes:
        return (
            b"solid verified_item\n"
            b"facet normal 0 0 1\n"
            b"outer loop\n"
            b"vertex 0 0 0\n"
            b"vertex 1 0 0\n"
            b"vertex 0 1 0\n"
            b"endloop\n"
            b"endfacet\n"
            b"endsolid verified_item\n"
        )

    def _pack_document(
        self,
        payload: bytes | None = None,
        *,
        asset_path: str = "meshes/item.stl",
        format_name: str = "stl",
    ) -> dict[str, object]:
        payload = self._asset_bytes() if payload is None else payload
        return {
            "schema": "matrix-item-asset-pack/v1",
            "pack": {
                "pack_id": "benchmark.example-props",
                "revision": "dataset-v1",
                "license": {
                    "spdx_id": "CC0-1.0",
                    "attribution": "",
                },
                "provenance": {
                    "source_name": "Example Props Benchmark",
                    "source_uri": "https://example.invalid/props",
                    "source_revision": "release-1",
                    "source_item_ids": ["source-item-17"],
                },
                "coordinate_frame": {
                    "up_axis": "+Z",
                    "forward_axis": "+X",
                    "handedness": "right",
                    "meters_per_unit": 1.0,
                },
                "files": [
                    {
                        "file_id": "body_mesh",
                        "path": asset_path,
                        "size_bytes": len(payload),
                        "sha256": hashlib.sha256(payload).hexdigest(),
                        "role": "visual_mesh",
                        "media_type": "model/stl",
                        "format": format_name,
                    }
                ],
                "items": [
                    {
                        "item_id": "test_prop",
                        "label": "Test prop",
                        "physics": {
                            "mass_kg": 1.2,
                            "collision": {
                                "shape": "box",
                                "half_extents_m": [0.1, 0.05, 0.05],
                            },
                        },
                        "visual_parts": [
                            {
                                "part_id": "body",
                                "file_id": "body_mesh",
                                "rgba": [0.8, 0.2, 0.1, 1.0],
                                "scale": [1.0, 1.0, 1.0],
                                "translation_m": [0.0, 0.0, 0.0],
                                "rotation_wxyz": [1.0, 0.0, 0.0, 0.0],
                            }
                        ],
                    }
                ],
            },
        }

    def _write_pack(
        self,
        root: Path,
        *,
        document: dict[str, object] | None = None,
        payload: bytes | None = None,
    ) -> tuple[Path, dict[str, object]]:
        payload = self._asset_bytes() if payload is None else payload
        document = self._pack_document(payload) if document is None else document
        raw_path = document["pack"]["files"][0]["path"]  # type: ignore[index]
        asset_path = root.joinpath(*str(raw_path).split("/"))
        asset_path.parent.mkdir(parents=True, exist_ok=True)
        asset_path.write_bytes(payload)
        manifest = root / "matrix-item-asset-pack.json"
        manifest.write_text(
            json.dumps(document, indent=2),
            encoding="utf-8",
        )
        return manifest, document

    def _inventory_document(self, digest: str) -> dict[str, object]:
        return {
            "schema": "matrix-item-inventory/v1",
            "inventory": {
                "inventory_id": "test-inventory",
                "entries": [
                    {
                        "slot_id": "primary_prop",
                        "pack_digest": f"sha256:{digest}",
                        "item_id": "test_prop",
                        "pool_size": 4,
                        "spawn": {
                            "distance_m": 1.5,
                            "height_m": 0.8,
                            "quaternion_wxyz": [2.0, 0.0, 0.0, 0.0],
                        },
                    }
                ],
            },
        }

    def _write_registry_pack(
        self,
        registry: Path,
        document: dict[str, object],
        payload: bytes | None = None,
    ) -> tuple[str, Path]:
        digest = PACK.asset_pack_digest(document)
        pack_root = registry / "sha256" / digest[:2] / digest
        manifest, _ = self._write_pack(
            pack_root,
            document=document,
            payload=payload,
        )
        return digest, manifest

    def test_load_pack_returns_verified_frozen_dtos(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            manifest, _ = self._write_pack(Path(temporary_directory))

            pack = PACK.load_asset_pack(manifest)

            self.assertEqual(pack.pack_id, "benchmark.example-props")
            self.assertRegex(pack.digest_ref, r"^sha256:[0-9a-f]{64}$")
            self.assertEqual(pack.license.spdx_id, "CC0-1.0")
            self.assertEqual(pack.provenance.source_item_ids, ("source-item-17",))
            self.assertEqual(pack.coordinate_frame.up_axis, "+Z")
            self.assertEqual(pack.files[0].path.read_bytes(), self._asset_bytes())
            self.assertEqual(pack.items[0].physics.mass_kg, 1.2)
            self.assertEqual(pack.items[0].visual_parts[0].asset_file.file_id, "body_mesh")
            with self.assertRaises(AttributeError):
                pack.pack_id = "changed"

    def test_license_and_provenance_files_share_the_pack_digest(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            document = self._pack_document()
            license_bytes = b"Creative Commons Zero v1.0 Universal\n"
            document["pack"]["files"].append(  # type: ignore[index]
                {
                    "file_id": "license_text",
                    "path": "License.txt",
                    "size_bytes": len(license_bytes),
                    "sha256": hashlib.sha256(license_bytes).hexdigest(),
                    "role": "license",
                    "media_type": "text/plain",
                    "format": "text",
                }
            )
            manifest, _ = self._write_pack(root, document=document)
            (root / "License.txt").write_bytes(license_bytes)

            pack = PACK.load_asset_pack(manifest)

            license_file = next(
                asset for asset in pack.files if asset.file_id == "license_text"
            )
            self.assertEqual(license_file.role, "license")
            self.assertEqual(license_file.media_type, "text/plain")
            tampered = dict(document)
            tampered["pack"] = dict(document["pack"])  # type: ignore[arg-type]
            tampered["pack"]["files"] = list(document["pack"]["files"])  # type: ignore[index]
            tampered["pack"]["files"][1] = dict(  # type: ignore[index]
                tampered["pack"]["files"][1]  # type: ignore[index]
            )
            tampered["pack"]["files"][1]["sha256"] = "0" * 64  # type: ignore[index]
            self.assertNotEqual(
                PACK.asset_pack_digest(document),
                PACK.asset_pack_digest(tampered),
            )

    def test_canonical_digest_is_location_and_json_key_order_independent(self) -> None:
        document = self._pack_document()
        reordered = {
            "pack": {
                key: document["pack"][key]  # type: ignore[index]
                for key in reversed(list(document["pack"]))  # type: ignore[arg-type]
            },
            "schema": document["schema"],
        }
        with tempfile.TemporaryDirectory() as first_directory, tempfile.TemporaryDirectory() as second_directory:
            first_manifest, _ = self._write_pack(
                Path(first_directory), document=document
            )
            second_manifest, _ = self._write_pack(
                Path(second_directory), document=reordered
            )

            first = PACK.load_asset_pack(first_manifest)
            second = PACK.load_asset_pack(second_manifest)

        self.assertEqual(first.digest, second.digest)
        self.assertEqual(
            first.digest,
            hashlib.sha256(PACK.canonical_asset_pack_bytes(document)).hexdigest(),
        )

    def test_registry_and_inventory_resolve_to_legacy_injector_spec(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            registry = root / "registry"
            registry.mkdir()
            document = self._pack_document()
            digest, manifest = self._write_registry_pack(registry, document)
            inventory_path = root / "inventory.json"
            inventory_path.write_text(
                json.dumps(self._inventory_document(digest)),
                encoding="utf-8",
            )

            pack = PACK.resolve_registry_pack(registry, f"sha256:{digest}")
            resolved = PACK.resolve_inventory(inventory_path, registry)
            specs = resolved.legacy_injector_specs()

            self.assertEqual(pack.manifest_path, manifest)
            self.assertEqual(len(resolved.items), 1)
            self.assertEqual(resolved.items[0].pack.digest, digest)
            self.assertEqual(resolved.items[0].spawn.quaternion_wxyz, (1.0, 0.0, 0.0, 0.0))
            self.assertEqual(specs[0].item_id, "primary_prop")
            self.assertEqual(specs[0].pool_size, 4)
            self.assertEqual(specs[0].visuals[0].mesh.suffix, ".stl")

    def test_registry_digest_mismatch_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            registry = Path(temporary_directory)
            document = self._pack_document()
            actual_digest = PACK.asset_pack_digest(document)
            wrong_digest = "0" * 64
            if wrong_digest == actual_digest:
                wrong_digest = "1" * 64
            wrong_root = registry / "sha256" / wrong_digest[:2] / wrong_digest
            self._write_pack(wrong_root, document=document)

            with self.assertRaisesRegex(
                PACK.ItemAssetVerificationError,
                "registry pack digest mismatch",
            ):
                PACK.resolve_registry_pack(registry, f"sha256:{wrong_digest}")

    def test_strict_json_rejects_duplicate_keys_and_non_finite_numbers(self) -> None:
        for text, expected in (
            ('{"schema":"one","schema":"two"}', "duplicate key"),
            ('{"value":NaN}', "non-finite"),
            ('{"value":Infinity}', "non-finite"),
        ):
            with self.subTest(text=text):
                with self.assertRaisesRegex(
                    PACK.ItemAssetValidationError, expected
                ):
                    PACK.loads_json_strict(text)

    def test_unknown_fields_fail_closed_at_every_manifest_layer(self) -> None:
        mutations = []
        root = self._pack_document()
        root["unknown"] = True
        mutations.append(root)
        pack = self._pack_document()
        pack["pack"]["unknown"] = True  # type: ignore[index]
        mutations.append(pack)
        file_document = self._pack_document()
        file_document["pack"]["files"][0]["unknown"] = True  # type: ignore[index]
        mutations.append(file_document)
        visual = self._pack_document()
        visual["pack"]["items"][0]["visual_parts"][0]["unknown"] = True  # type: ignore[index]
        mutations.append(visual)
        for index, document in enumerate(mutations):
            with self.subTest(index=index):
                with self.assertRaisesRegex(
                    PACK.ItemAssetValidationError, "unknown"
                ):
                    PACK.asset_pack_digest(document)

    def test_unknown_inventory_field_fails_closed(self) -> None:
        document = self._inventory_document("0" * 64)
        document["inventory"]["entries"][0]["weapon_type"] = "hardcoded"  # type: ignore[index]
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "inventory.json"
            path.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(
                PACK.ItemAssetValidationError, "unknown"
            ):
                PACK.load_inventory(path)

    def test_relative_path_traversal_absolute_and_backslash_fail_closed(self) -> None:
        for unsafe in (
            "../item.stl",
            "meshes/../../item.stl",
            "/tmp/item.stl",
            "meshes\\item.stl",
            "meshes//item.stl",
            "./meshes/item.stl",
        ):
            with self.subTest(path=unsafe):
                document = self._pack_document(asset_path=unsafe)
                with self.assertRaisesRegex(
                    PACK.ItemAssetValidationError, "safe relative path"
                ):
                    PACK.asset_pack_digest(document)

    def test_asset_symlink_and_symlink_parent_fail_closed(self) -> None:
        if not hasattr(os, "symlink"):
            self.skipTest("symlinks unavailable")
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            outside = root / "outside.stl"
            outside.write_bytes(self._asset_bytes())
            direct_root = root / "direct"
            direct_root.mkdir()
            document = self._pack_document(asset_path="item.stl")
            (direct_root / "item.stl").symlink_to(outside)
            direct_manifest = direct_root / "matrix-item-asset-pack.json"
            direct_manifest.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(
                PACK.ItemAssetVerificationError, "symlink"
            ):
                PACK.load_asset_pack(direct_manifest)

            parent_root = root / "parent"
            parent_root.mkdir()
            (parent_root / "meshes").symlink_to(root)
            nested_document = self._pack_document(asset_path="meshes/outside.stl")
            nested_manifest = parent_root / "matrix-item-asset-pack.json"
            nested_manifest.write_text(
                json.dumps(nested_document), encoding="utf-8"
            )
            with self.assertRaisesRegex(
                PACK.ItemAssetVerificationError, "symlink"
            ):
                PACK.load_asset_pack(nested_manifest)

    def test_non_regular_asset_size_and_hash_mismatch_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            directory_document = self._pack_document(asset_path="mesh.stl")
            (root / "mesh.stl").mkdir()
            manifest = root / "matrix-item-asset-pack.json"
            manifest.write_text(
                json.dumps(directory_document), encoding="utf-8"
            )
            with self.assertRaisesRegex(
                PACK.ItemAssetVerificationError, "regular file"
            ):
                PACK.load_asset_pack(manifest)

        for mutation, expected in (("size", "size mismatch"), ("hash", "SHA256 mismatch")):
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory() as temporary_directory:
                root = Path(temporary_directory)
                document = self._pack_document()
                if mutation == "size":
                    document["pack"]["files"][0]["size_bytes"] += 1  # type: ignore[index,operator]
                else:
                    document["pack"]["files"][0]["sha256"] = "0" * 64  # type: ignore[index]
                manifest, _ = self._write_pack(root, document=document)
                with self.assertRaisesRegex(
                    PACK.ItemAssetVerificationError, expected
                ):
                    PACK.load_asset_pack(manifest)

    def test_unknown_format_and_suffix_mismatch_fail_closed(self) -> None:
        for format_name, path in (
            ("exe", "item.exe"),
            ("glb", "item.stl"),
        ):
            with self.subTest(format=format_name, path=path):
                document = self._pack_document(
                    asset_path=path,
                    format_name=format_name,
                )
                with self.assertRaises(PACK.ItemAssetValidationError):
                    PACK.asset_pack_digest(document)

    def test_license_provenance_coordinate_and_physics_are_required_and_strict(self) -> None:
        mutations = []
        license_document = self._pack_document()
        license_document["pack"]["license"]["spdx_id"] = "MIT OR Apache-2.0"  # type: ignore[index]
        mutations.append(license_document)
        provenance = self._pack_document()
        provenance["pack"]["provenance"]["source_uri"] = "/local/machine/path"  # type: ignore[index]
        mutations.append(provenance)
        coordinate = self._pack_document()
        coordinate["pack"]["coordinate_frame"]["forward_axis"] = "-Z"  # type: ignore[index]
        mutations.append(coordinate)
        collision = self._pack_document()
        collision["pack"]["items"][0]["physics"]["collision"]["shape"] = "guess"  # type: ignore[index]
        mutations.append(collision)
        for index, document in enumerate(mutations):
            with self.subTest(index=index):
                with self.assertRaises(PACK.ItemAssetValidationError):
                    PACK.asset_pack_digest(document)

    def test_atomic_spdx_and_project_license_ref_are_supported(self) -> None:
        for license_id in ("MIT", "GPL-2.0-only", "LicenseRef-Benchmark-Research"):
            with self.subTest(license_id=license_id):
                document = self._pack_document()
                document["pack"]["license"]["spdx_id"] = license_id  # type: ignore[index]
                self.assertRegex(PACK.asset_pack_digest(document), r"^[0-9a-f]{64}$")
        document = self._pack_document()
        document["pack"]["license"]["spdx_id"] = "LicenseRef-"  # type: ignore[index]
        with self.assertRaises(PACK.ItemAssetValidationError):
            PACK.asset_pack_digest(document)

    def test_inventory_unknown_item_and_bad_digest_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            registry = root / "registry"
            registry.mkdir()
            document = self._pack_document()
            digest, _ = self._write_registry_pack(registry, document)
            inventory = self._inventory_document(digest)
            inventory["inventory"]["entries"][0]["item_id"] = "missing"  # type: ignore[index]
            inventory_path = root / "inventory.json"
            inventory_path.write_text(json.dumps(inventory), encoding="utf-8")
            with self.assertRaisesRegex(
                PACK.ItemInventoryResolutionError, "has no item"
            ):
                PACK.resolve_inventory(inventory_path, registry)

            inventory["inventory"]["entries"][0]["pack_digest"] = digest  # type: ignore[index]
            inventory_path.write_text(json.dumps(inventory), encoding="utf-8")
            with self.assertRaisesRegex(
                PACK.ItemAssetValidationError, "sha256:"
            ):
                PACK.load_inventory(inventory_path)

    def test_legacy_adapter_rejects_semantic_loss(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            registry = root / "registry"
            registry.mkdir()
            document = self._pack_document(
                payload=b"glTF bytes",
                asset_path="meshes/item.glb",
                format_name="glb",
            )
            digest, _ = self._write_registry_pack(
                registry,
                document,
                payload=b"glTF bytes",
            )
            inventory_path = root / "inventory.json"
            inventory_path.write_text(
                json.dumps(self._inventory_document(digest)),
                encoding="utf-8",
            )
            resolved = PACK.resolve_inventory(inventory_path, registry)
            with self.assertRaisesRegex(
                PACK.ItemInventoryResolutionError, "unsupported legacy"
            ):
                resolved.legacy_injector_specs()

        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            registry = root / "registry"
            registry.mkdir()
            document = self._pack_document()
            document["pack"]["coordinate_frame"]["forward_axis"] = "-X"  # type: ignore[index]
            digest, _ = self._write_registry_pack(registry, document)
            inventory_path = root / "inventory.json"
            inventory_path.write_text(
                json.dumps(self._inventory_document(digest)),
                encoding="utf-8",
            )
            resolved = PACK.resolve_inventory(inventory_path, registry)
            with self.assertRaisesRegex(
                PACK.ItemInventoryResolutionError, "coordinate-frame"
            ):
                resolved.legacy_injector_specs()

    def test_schema_files_are_valid_json_and_closed_at_root(self) -> None:
        for filename, schema_id in (
            ("matrix-item-asset-pack-v1.schema.json", "matrix-item-asset-pack/v1"),
            ("matrix-item-inventory-v1.schema.json", "matrix-item-inventory/v1"),
        ):
            with self.subTest(filename=filename):
                path = ROOT / "config" / "schemas" / filename
                schema = PACK.loads_json_strict(
                    path.read_text(encoding="utf-8"),
                    source=str(path),
                )
                self.assertFalse(schema["additionalProperties"])
                self.assertEqual(schema["properties"]["schema"]["const"], schema_id)


if __name__ == "__main__":
    unittest.main()
