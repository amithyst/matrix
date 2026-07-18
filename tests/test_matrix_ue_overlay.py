from __future__ import annotations

import hashlib
import importlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = REPO_ROOT / "scripts"
if os.fspath(SCRIPTS) not in sys.path:
    sys.path.insert(0, os.fspath(SCRIPTS))

OVERLAY = importlib.import_module("matrix_ue_overlay")
PRODUCTION_CONTRACT = (
    REPO_ROOT / "config/runtime/matrix-centered-camera-overlay-v3.json"
)


class MatrixUeOverlayTest(unittest.TestCase):
    CONTENTS = {
        f"{OVERLAY.STEM}.pak": b"fixture-pak\n",
        f"{OVERLAY.STEM}.utoc": b"fixture-utoc\n",
        f"{OVERLAY.STEM}.ucas": b"fixture-ucas\n",
    }

    @staticmethod
    def write_contract(path: Path, contents: dict[str, bytes]) -> None:
        payload = {
            "schema_version": 1,
            "overlay_version": 3,
            "overlay_id": "matrix-centered-camera-custom-v3",
            "stem": "pakchunk99-MatrixCentered-Linux_P",
            "runtime_directory": (
                "src/UeSim/Linux/zsibot_mujoco_ue/Saved/Paks/"
                "MatrixCenteredCameraActive"
            ),
            "mode": "centered",
            "scope": ["MujocoSim_Custom", "Spectator"],
            "supported_class": "MujocoSim_Custom_C",
            "files": [
                {
                    "name": name,
                    "size": len(data),
                    "sha256": hashlib.sha256(data).hexdigest(),
                }
                for name, data in contents.items()
            ],
        }
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(payload), encoding="utf-8")

    @staticmethod
    def write_bundle(path: Path, contents: dict[str, bytes]) -> None:
        path.mkdir(parents=True)
        for name, data in contents.items():
            (path / name).write_bytes(data)

    @staticmethod
    def load_fixture_contract(path: Path, contents: dict[str, bytes]) -> object:
        pinned = {
            name: (len(data), hashlib.sha256(data).hexdigest())
            for name, data in contents.items()
        }
        with mock.patch.object(OVERLAY, "PINNED_ARTIFACTS", pinned):
            return OVERLAY.load_contract(path)

    def fixture(self, temporary: str) -> tuple[Path, Path, object, Path]:
        root = Path(temporary) / "matrix"
        root.mkdir()
        contract_path = root / "contract.json"
        self.write_contract(contract_path, self.CONTENTS)
        bundle = Path(temporary) / "bundle"
        self.write_bundle(bundle, self.CONTENTS)
        contract = self.load_fixture_contract(contract_path, self.CONTENTS)
        return root, bundle, contract, contract_path

    def test_production_contract_pins_v3_artifacts_and_scope(self) -> None:
        payload = json.loads(PRODUCTION_CONTRACT.read_text(encoding="utf-8"))
        self.assertEqual(payload["overlay_version"], 3)
        self.assertEqual(payload["stem"], "pakchunk99-MatrixCentered-Linux_P")
        self.assertEqual(payload["scope"], ["MujocoSim_Custom", "Spectator"])
        self.assertEqual(payload["supported_class"], "MujocoSim_Custom_C")
        self.assertEqual(
            {entry["name"]: (entry["size"], entry["sha256"]) for entry in payload["files"]},
            {
                "pakchunk99-MatrixCentered-Linux_P.pak": (
                    339,
                    "b17dfaf284d60bef70d70dac05a32c74723afe689764628eb25cf8fdb9424487",
                ),
                "pakchunk99-MatrixCentered-Linux_P.utoc": (
                    554,
                    "6e95033e880fe2537e304317e2189c1ca5943f57acc3eb50ab439c26044afe9a",
                ),
                "pakchunk99-MatrixCentered-Linux_P.ucas": (
                    34423,
                    "f0fd22f538cb6d95c6e4e501c3aa5953247ba718a3e1cc4d218ce3f320c0c430",
                ),
            },
        )
        OVERLAY.load_contract(PRODUCTION_CONTRACT)

    def test_verify_bundle_accepts_only_exact_regular_inventory(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            _root, bundle, contract, _contract_path = self.fixture(temporary)
            self.assertEqual(OVERLAY.verify_bundle(bundle, contract), bundle)

            (bundle / "unexpected.txt").write_text("extra", encoding="utf-8")
            with self.assertRaisesRegex(OVERLAY.OverlayError, "extra"):
                OVERLAY.verify_bundle(bundle, contract)

    def test_verify_bundle_rejects_size_hash_and_symlink(self) -> None:
        cases = (b"short", b"fixture-paX\n")
        for replacement in cases:
            with self.subTest(replacement=replacement):
                with tempfile.TemporaryDirectory() as temporary:
                    _root, bundle, contract, _contract_path = self.fixture(temporary)
                    (bundle / f"{OVERLAY.STEM}.pak").write_bytes(replacement)
                    with self.assertRaisesRegex(
                        OVERLAY.OverlayError, "size mismatch|sha256 mismatch"
                    ):
                        OVERLAY.verify_bundle(bundle, contract)

        with tempfile.TemporaryDirectory() as temporary:
            _root, bundle, contract, _contract_path = self.fixture(temporary)
            artifact = bundle / f"{OVERLAY.STEM}.pak"
            artifact.unlink()
            artifact.symlink_to(bundle / f"{OVERLAY.STEM}.utoc")
            with self.assertRaisesRegex(OVERLAY.OverlayError, "not a regular file"):
                OVERLAY.verify_bundle(bundle, contract)

    def test_paths_and_contract_schema_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root, bundle, contract, contract_path = self.fixture(temporary)
            with self.assertRaisesRegex(OVERLAY.OverlayError, "differ from pinned v3"):
                OVERLAY.load_contract(contract_path)
            with self.assertRaisesRegex(OVERLAY.OverlayError, "absolute path"):
                OVERLAY.verify_bundle(Path("relative-bundle"), contract)
            with self.assertRaisesRegex(OVERLAY.OverlayError, "absolute path"):
                OVERLAY.install(Path("relative-project"), bundle, contract)

            linked_bundle = Path(temporary) / "linked-bundle"
            linked_bundle.symlink_to(bundle, target_is_directory=True)
            with self.assertRaisesRegex(OVERLAY.OverlayError, "symlink component"):
                OVERLAY.verify_bundle(linked_bundle, contract)

            payload = json.loads(contract_path.read_text(encoding="utf-8"))
            payload["scope"] = ["MujocoSim_Custom"]
            contract_path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(OVERLAY.OverlayError, "scope"):
                OVERLAY.load_contract(contract_path)

            outside = Path(temporary) / "outside"
            outside.mkdir()
            (root / "src").symlink_to(outside, target_is_directory=True)
            self.write_contract(contract_path, self.CONTENTS)
            contract = self.load_fixture_contract(contract_path, self.CONTENTS)
            with self.assertRaisesRegex(OVERLAY.OverlayError, "not a real directory"):
                OVERLAY.install(root, bundle, contract)

    def test_install_and_remove_are_closed_atomic_directory_transitions(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root, bundle, contract, _contract_path = self.fixture(temporary)
            active = OVERLAY.install(root, bundle, contract)
            self.assertEqual(active, root / OVERLAY.RUNTIME_DIRECTORY)
            self.assertEqual(set(path.name for path in active.iterdir()), set(self.CONTENTS))
            self.assertFalse(
                any(
                    path.name.startswith(OVERLAY.INSTALL_PREFIX)
                    for path in active.parent.iterdir()
                )
            )
            with self.assertRaisesRegex(OVERLAY.OverlayError, "already exists"):
                OVERLAY.install(root, bundle, contract)

            self.assertTrue(OVERLAY.remove(root, contract))
            self.assertFalse(active.exists())
            self.assertFalse(OVERLAY.remove(root, contract))
            self.assertFalse(
                any(
                    path.name.startswith(OVERLAY.REMOVE_PREFIX)
                    for path in active.parent.iterdir()
                )
            )

    def test_remove_and_purge_reject_unknown_or_corrupt_active_content(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root, bundle, contract, _contract_path = self.fixture(temporary)
            active = OVERLAY.install(root, bundle, contract)
            (active / "unexpected").write_text("do not delete", encoding="utf-8")
            with self.assertRaisesRegex(OVERLAY.OverlayError, "extra"):
                OVERLAY.remove(root, contract)
            with self.assertRaisesRegex(OVERLAY.OverlayError, "extra"):
                OVERLAY.purge_stale(root, contract)
            self.assertTrue((active / "unexpected").exists())

        with tempfile.TemporaryDirectory() as temporary:
            root, bundle, contract, _contract_path = self.fixture(temporary)
            active = OVERLAY.install(root, bundle, contract)
            corrupt = active / f"{OVERLAY.STEM}.pak"
            corrupt.unlink()
            corrupt.write_bytes(b"corrupt")
            with self.assertRaisesRegex(
                OVERLAY.OverlayError, "size mismatch|sha256 mismatch"
            ):
                OVERLAY.purge_stale(root, contract)
            self.assertTrue(active.exists())

    def test_purge_stale_removes_only_verified_helper_directories(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root, bundle, contract, _contract_path = self.fixture(temporary)
            active = OVERLAY.install(root, bundle, contract)
            parent = active.parent
            partial = parent / f"{OVERLAY.INSTALL_PREFIX}123-fixture"
            partial.mkdir()
            first_name = next(iter(self.CONTENTS))
            (partial / first_name).write_bytes(self.CONTENTS[first_name])
            unrelated = parent / "third-party.pak"
            unrelated.write_bytes(b"leave me")

            self.assertEqual(OVERLAY.purge_stale(root, contract), 2)
            self.assertFalse(active.exists())
            self.assertFalse(partial.exists())
            self.assertEqual(unrelated.read_bytes(), b"leave me")

    def test_cli_reports_failure_without_mutating_bad_bundle(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            _root, bundle, _contract, _contract_path = self.fixture(temporary)
            bad_file = bundle / f"{OVERLAY.STEM}.ucas"
            bad_file.write_bytes(b"bad")
            result = subprocess.run(
                [
                    sys.executable,
                    "-I",
                    os.fspath(SCRIPTS / "matrix_ue_overlay.py"),
                    "verify-bundle",
                    "--contract",
                    os.fspath(PRODUCTION_CONTRACT),
                    "--bundle",
                    os.fspath(bundle),
                ],
                text=True,
                capture_output=True,
                check=False,
            )
            self.assertEqual(result.returncode, 2)
            self.assertIn("size mismatch", result.stderr)
            self.assertEqual(bad_file.read_bytes(), b"bad")


if __name__ == "__main__":
    unittest.main()
