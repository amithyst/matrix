from __future__ import annotations

import copy
from contextlib import redirect_stderr, redirect_stdout
import hashlib
import io
import json
from pathlib import Path
import stat
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = REPO_ROOT / "scripts"
FIXTURE = REPO_ROOT / "config/scenes/office-outside-ring.manifest.json"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import matrix_scene_manifest as MODULE  # noqa: E402


def valid_manifest() -> dict[str, object]:
    return json.loads(FIXTURE.read_text(encoding="utf-8"))


def with_translation(x: float) -> dict[str, object]:
    document = valid_manifest()
    document["scene"]["transforms"][0]["translation"] = [x, 0.0, 0.0]  # type: ignore[index]
    return document


class SceneManifestSchemaTest(unittest.TestCase):
    def assert_invalid(self, document: dict[str, object]) -> None:
        with self.assertRaises(MODULE.ManifestValidationError):
            MODULE.validate_scene_document(document)

    def test_office_fixture_is_valid_and_digest_is_stable(self) -> None:
        first = valid_manifest()
        second = copy.deepcopy(first)
        second["scene"]["asset_references"].reverse()  # type: ignore[index]
        second["scene"]["placed_entities"][0]["tags"].reverse()  # type: ignore[index]
        self.assertEqual(MODULE.scene_digest(first), MODULE.scene_digest(second))
        self.assertEqual(
            MODULE.validate_scene_document(first)["scene"]["coordinate_frame"],
            {
                "id": "frame.matrix_world",
                "meters_per_unit": 1.0,
                "up_axis": "Z",
                "handedness": "right",
            },
        )

    def test_strict_json_and_unknown_fields_fail_closed(self) -> None:
        with self.assertRaises(MODULE.ManifestValidationError):
            MODULE.loads_json_strict('{"schema":"a","schema":"b"}')
        with self.assertRaises(MODULE.ManifestValidationError):
            MODULE.loads_json_strict('{"value":NaN}')
        unknown = valid_manifest()
        unknown["scene"]["surprise"] = True  # type: ignore[index]
        self.assert_invalid(unknown)

    def test_coordinate_and_transform_contracts_are_strict(self) -> None:
        wrong_axis = valid_manifest()
        wrong_axis["scene"]["coordinate_frame"]["up_axis"] = "Y"  # type: ignore[index]
        self.assert_invalid(wrong_axis)
        bad_quaternion = valid_manifest()
        bad_quaternion["scene"]["transforms"][0]["rotation_xyzw"] = [  # type: ignore[index]
            0,
            0,
            0,
            2,
        ]
        self.assert_invalid(bad_quaternion)
        negative_scale = valid_manifest()
        negative_scale["scene"]["transforms"][0]["scale"] = [1, -1, 1]  # type: ignore[index]
        self.assert_invalid(negative_scale)

    def test_backend_role_media_and_locator_are_bound(self) -> None:
        wrong_role = valid_manifest()
        wrong_role["scene"]["asset_references"][0]["role"] = "collision"  # type: ignore[index]
        self.assert_invalid(wrong_role)

        wrong_media = valid_manifest()
        wrong_media["scene"]["asset_references"][1]["backend"] = (  # type: ignore[index]
            "mujoco_mesh"
        )
        self.assert_invalid(wrong_media)

        ue = valid_manifest()
        visual = ue["scene"]["asset_references"][0]  # type: ignore[index]
        visual.update(
            {
                "backend": "ue_cooked",
                "locator": {"scheme": "ue_package", "value": "/Game/Maps/3DGSWorld"},
                "media_type": "ue_asset",
                "size_bytes": None,
            }
        )
        MODULE.validate_scene_document(ue)
        with self.assertRaises(MODULE.AssetVerificationError):
            with MODULE.open_verified_asset_references(
                ue, allowed_roots=(REPO_ROOT,)
            ):
                pass

    def test_asset_derivation_references_and_cycles_fail_closed(self) -> None:
        dangling = valid_manifest()
        dangling["scene"]["asset_references"][1][
            "derived_from_asset_id"
        ] = "asset.missing"  # type: ignore[index]
        self.assert_invalid(dangling)

        cycle = valid_manifest()
        assets = cycle["scene"]["asset_references"]  # type: ignore[index]
        assets[0]["derived_from_asset_id"] = assets[1]["id"]
        assets[1]["derived_from_asset_id"] = assets[0]["id"]
        self.assert_invalid(cycle)

        converted = valid_manifest()
        converted_assets = converted["scene"]["asset_references"]  # type: ignore[index]
        mujoco = copy.deepcopy(converted_assets[1])
        mujoco.update(
            {
                "id": "asset.office_outside_ring.collision_mujoco",
                "backend": "mujoco_mesh",
                "media_type": "stl",
                "source_selector": None,
                "derived_from_asset_id": converted_assets[1]["id"],
            }
        )
        converted_assets.append(mujoco)
        converted_entity = converted["scene"]["placed_entities"][0]  # type: ignore[index]
        converted_entity["collision_asset_id"] = mujoco["id"]
        converted_entity["physics_mode"] = "static"
        converted_entity["collision_enabled"] = True
        MODULE.validate_scene_document(converted)

        mujoco["derived_from_asset_id"] = converted_assets[0]["id"]
        self.assert_invalid(converted)

    def test_entity_visual_collision_and_physics_references_are_strict(self) -> None:
        swapped = valid_manifest()
        entity = swapped["scene"]["placed_entities"][0]  # type: ignore[index]
        entity["visual_asset_id"], entity["collision_asset_id"] = (
            entity["collision_asset_id"],
            entity["visual_asset_id"],
        )
        self.assert_invalid(swapped)

        no_physics = valid_manifest()
        no_physics_entity = no_physics["scene"]["placed_entities"][0]  # type: ignore[index]
        no_physics_entity["physics_mode"] = "none"
        no_physics_entity["collision_enabled"] = True
        self.assert_invalid(no_physics)

        unsupported_runtime_collision = valid_manifest()
        unsupported_entity = unsupported_runtime_collision["scene"][
            "placed_entities"
        ][0]  # type: ignore[index]
        unsupported_entity["physics_mode"] = "static"
        unsupported_entity["collision_enabled"] = True
        self.assert_invalid(unsupported_runtime_collision)

    def test_store_metadata_tampering_fails_closed(self) -> None:
        stored = MODULE.store_document(valid_manifest(), generation=1)
        stored["storage"]["generation"] = 99
        with self.assertRaises(MODULE.ManifestValidationError):
            MODULE.validate_store_document(stored)
        with self.assertRaises(MODULE.ManifestValidationError):
            MODULE.store_document(valid_manifest(), generation=True)


class SceneManifestStoreTest(unittest.TestCase):
    def test_create_update_and_opaque_cas(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "scene.json"
            first = MODULE.write_store(path, valid_manifest(), expected_generation=0)
            second = MODULE.write_store(
                path,
                with_translation(1.0),
                expected_generation=first.generation,
                expected_store_digest=first.store_digest,
            )
            self.assertEqual(second.generation, 2)
            with self.assertRaises(MODULE.ManifestConflictError):
                MODULE.write_store(
                    path,
                    with_translation(2.0),
                    expected_generation=second.generation,
                    expected_store_digest=first.store_digest,
                )

    def test_store_digest_prevents_generation_aba_after_rollback(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "scene.json"
            first = MODULE.write_store(path, valid_manifest(), expected_generation=0)
            with patch.object(
                MODULE, "_utc_now", return_value="2030-01-01T00:00:00+00:00"
            ):
                original_second = MODULE.write_store(
                    path,
                    with_translation(1.0),
                    expected_generation=first.generation,
                    expected_store_digest=first.store_digest,
                )
            path.write_bytes(b'{"schema":')
            recovered = MODULE.read_store(path)
            with patch.object(
                MODULE, "_utc_now", return_value="2030-01-01T00:00:00+00:00"
            ):
                replacement_second = MODULE.write_store(
                    path,
                    with_translation(1.0),
                    expected_generation=recovered.generation,
                    expected_store_digest=recovered.store_digest,
                )
            self.assertEqual(original_second.generation, replacement_second.generation)
            self.assertNotEqual(original_second.revision_id, replacement_second.revision_id)
            self.assertNotEqual(original_second.store_digest, replacement_second.store_digest)
            with self.assertRaises(MODULE.ManifestConflictError):
                MODULE.write_store(
                    path,
                    with_translation(3.0),
                    expected_generation=original_second.generation,
                    expected_store_digest=original_second.store_digest,
                )

    def test_operational_read_error_never_falls_back_or_overwrites(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "scene.json"
            first = MODULE.write_store(path, valid_manifest(), expected_generation=0)
            second = MODULE.write_store(
                path,
                with_translation(1.0),
                expected_generation=first.generation,
                expected_store_digest=first.store_digest,
            )
            primary_bytes = path.read_bytes()
            real_read = MODULE._read_bytes_secure

            def fail_primary(candidate: Path) -> bytes:
                if candidate == path:
                    raise MODULE.ManifestIOError("simulated primary I/O failure")
                return real_read(candidate)

            with patch.object(MODULE, "_read_bytes_secure", side_effect=fail_primary):
                with self.assertRaises(MODULE.ManifestIOError):
                    MODULE.read_store(path)
                with self.assertRaises(MODULE.ManifestIOError):
                    MODULE.write_store(
                        path,
                        with_translation(2.0),
                        expected_generation=second.generation,
                        expected_store_digest=second.store_digest,
                    )
            self.assertEqual(path.read_bytes(), primary_bytes)

    def test_backup_fallback_is_side_effect_free_and_both_bad_fail(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "scene.json"
            first = MODULE.write_store(path, valid_manifest(), expected_generation=0)
            backup = path.with_name(path.name + ".bak")
            backup_bytes = backup.read_bytes()
            corrupt = b'{"schema":'
            path.write_bytes(corrupt)
            recovered = MODULE.read_store(path)
            self.assertTrue(recovered.recovered_from_backup)
            self.assertEqual(recovered.store_digest, first.store_digest)
            self.assertEqual(path.read_bytes(), corrupt)
            self.assertEqual(backup.read_bytes(), backup_bytes)
            backup.write_bytes(corrupt)
            with self.assertRaises(MODULE.ManifestValidationError):
                MODULE.read_store(path)

    def test_reserved_sidecars_and_recover_false_fail_closed(self) -> None:
        with self.assertRaises(MODULE.ManifestValidationError):
            MODULE.read_store(Path("/"))
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "scene.json"
            MODULE.write_store(path, valid_manifest(), expected_generation=0)
            backup = path.with_name(path.name + ".bak")
            with self.assertRaises(MODULE.ManifestValidationError):
                MODULE.read_store(backup)
            path.unlink()
            with self.assertRaises(FileNotFoundError):
                MODULE.read_store(path, recover=False)

    def test_permissions_backup_and_repeated_atomic_writes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            path = root / "scene.json"
            current = MODULE.write_store(path, valid_manifest(), expected_generation=0)
            for index in range(1, 21):
                current = MODULE.write_store(
                    path,
                    with_translation(float(index)),
                    expected_generation=current.generation,
                    expected_store_digest=current.store_digest,
                )
                self.assertEqual(MODULE.read_store(path).generation, current.generation)
            self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o600)
            self.assertEqual(
                stat.S_IMODE(path.with_name(path.name + ".bak").stat().st_mode),
                0o600,
            )
            self.assertEqual(
                [item for item in root.iterdir() if ".tmp." in item.name], []
            )

    def test_concurrent_readers_only_observe_complete_generations(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "scene.json"
            current = MODULE.write_store(path, valid_manifest(), expected_generation=0)
            stop = threading.Event()
            failures: list[BaseException] = []
            seen: list[int] = []

            def reader() -> None:
                try:
                    while not stop.is_set():
                        seen.append(MODULE.read_store(path).generation)
                except BaseException as exc:
                    failures.append(exc)

            thread = threading.Thread(target=reader)
            thread.start()
            try:
                for generation in range(2, 51):
                    current = MODULE.write_store(
                        path,
                        with_translation(float(generation)),
                        expected_generation=current.generation,
                        expected_store_digest=current.store_digest,
                    )
            finally:
                stop.set()
                thread.join(timeout=10)
            self.assertFalse(failures, failures)
            self.assertFalse(thread.is_alive())
            self.assertEqual(seen, sorted(seen))
            self.assertEqual(current.generation, 50)

    def test_wall_clock_rollback_is_clamped(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "scene.json"
            with patch.object(MODULE, "_utc_now", return_value="2030-01-01T00:00:00+00:00"):
                first = MODULE.write_store(path, valid_manifest(), expected_generation=0)
            with patch.object(MODULE, "_utc_now", return_value="2020-01-01T00:00:00+00:00"):
                second = MODULE.write_store(
                    path,
                    with_translation(1.0),
                    expected_generation=first.generation,
                    expected_store_digest=first.store_digest,
                )
            self.assertEqual(
                second.document["storage"]["updated_at"],
                first.document["storage"]["updated_at"],
            )

    def test_file_asset_verification_is_hash_size_and_allowlist_bound(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            visual = root / "visual.usdz"
            collision = root / "collision.usd"
            visual.write_bytes(b"visual")
            collision.write_bytes(b"collision")
            document = valid_manifest()
            assets = document["scene"]["asset_references"]  # type: ignore[index]
            for asset, path in zip(assets, (visual, collision), strict=True):
                payload = path.read_bytes()
                asset["locator"]["value"] = path.as_uri()
                asset["size_bytes"] = len(payload)
                asset["content_hash"]["digest"] = hashlib.sha256(payload).hexdigest()
            with MODULE.open_verified_asset_references(
                document, allowed_roots=(root,)
            ) as verified:
                self.assertEqual(
                    tuple(item.path for item in verified),
                    (visual, collision),
                )
                self.assertTrue(all(item.proc_path.exists() for item in verified))
            visual.write_bytes(b"VISUAL")
            with self.assertRaises(MODULE.AssetVerificationError):
                with MODULE.open_verified_asset_references(
                    document, allowed_roots=(root,)
                ):
                    pass
            with self.assertRaises(MODULE.AssetVerificationError):
                with MODULE.open_verified_asset_references(
                    document, allowed_roots=()
                ):
                    pass

    def test_verified_asset_handle_survives_path_replacement(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            visual = root / "visual.usdz"
            collision = root / "collision.usd"
            visual.write_bytes(b"visual-original")
            collision.write_bytes(b"collision-original")
            document = valid_manifest()
            assets = document["scene"]["asset_references"]  # type: ignore[index]
            for asset, path in zip(assets, (visual, collision), strict=True):
                payload = path.read_bytes()
                asset["locator"]["value"] = path.as_uri()
                asset["size_bytes"] = len(payload)
                asset["content_hash"]["digest"] = hashlib.sha256(payload).hexdigest()

            with MODULE.open_verified_asset_references(
                document, allowed_roots=(root,)
            ) as verified:
                replacement = root / "replacement.usdz"
                replacement.write_bytes(b"unverified-data")
                replacement.replace(visual)
                self.assertEqual(
                    verified[0].proc_path.read_bytes(),
                    b"visual-original",
                )
                self.assertEqual(visual.read_bytes(), b"unverified-data")

    def test_intermediate_asset_symlink_cannot_escape_allowlist(self) -> None:
        with tempfile.TemporaryDirectory() as allowed, tempfile.TemporaryDirectory() as outside:
            allowed_root = Path(allowed)
            outside_file = Path(outside) / "visual.usdz"
            outside_file.write_bytes(b"outside")
            linked = allowed_root / "linked"
            linked.symlink_to(Path(outside), target_is_directory=True)
            document = valid_manifest()
            assets = document["scene"]["asset_references"]  # type: ignore[index]
            for asset in assets:
                asset["locator"]["value"] = (linked / outside_file.name).as_uri()
                asset["size_bytes"] = len(b"outside")
                asset["content_hash"]["digest"] = hashlib.sha256(b"outside").hexdigest()
            with self.assertRaises(MODULE.AssetVerificationError):
                with MODULE.open_verified_asset_references(
                    document, allowed_roots=(allowed_root,)
                ):
                    pass


class SceneManifestCliTest(unittest.TestCase):
    def test_cli_validate_write_inspect_update_and_conflict(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "scene.json"
            code, output, error = self.run_cli(["validate-input", str(FIXTURE)])
            self.assertEqual((code, error), (0, ""))
            self.assertTrue(json.loads(output)["ok"])

            code, output, error = self.run_cli(
                [
                    "write",
                    str(path),
                    "--input",
                    str(FIXTURE),
                    "--expected-generation",
                    "0",
                ]
            )
            self.assertEqual((code, error), (0, ""))
            created = json.loads(output)
            code, output, error = self.run_cli(["inspect", str(path)])
            self.assertEqual((code, error), (0, ""))
            self.assertEqual(json.loads(output)["generation"], 1)

            code, output, error = self.run_cli(
                [
                    "update",
                    str(path),
                    "--input",
                    str(FIXTURE),
                    "--expected-generation",
                    "1",
                    "--expected-store-digest",
                    created["store_digest"],
                ]
            )
            self.assertEqual((code, error), (0, ""))
            self.assertEqual(json.loads(output)["generation"], 2)
            code, output, error = self.run_cli(
                [
                    "update",
                    str(path),
                    "--input",
                    str(FIXTURE),
                    "--expected-generation",
                    "1",
                    "--expected-store-digest",
                    created["store_digest"],
                ]
            )
            self.assertEqual(code, 3)
            self.assertEqual(output, "")
            self.assertIn("CAS conflict", error)

    @staticmethod
    def run_cli(argv: list[str]) -> tuple[int, str, str]:
        stdout = io.StringIO()
        stderr = io.StringIO()
        with redirect_stdout(stdout), redirect_stderr(stderr):
            code = MODULE.main(argv)
        return code, stdout.getvalue().strip(), stderr.getvalue().strip()


if __name__ == "__main__":
    unittest.main()
