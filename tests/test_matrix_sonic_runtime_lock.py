from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "verify_matrix_sonic_runtime.py"
SPEC = importlib.util.spec_from_file_location("verify_matrix_sonic_runtime", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class MatrixSonicRuntimeLockTest(unittest.TestCase):
    def setUp(self) -> None:
        self.lock_path = REPO_ROOT / "config/runtime/matrix-sonic.lock.json"
        self.lock = MODULE.load_lock(self.lock_path)

    def test_release_packages_match_urban_contract(self) -> None:
        urban = json.loads(
            (REPO_ROOT / "research/urban_v1/scene.json").read_text(encoding="utf-8")
        )
        packages = {
            package["name"]: package
            for package in self.lock["matrix_release"]["packages"]
        }
        self.assertEqual(
            packages["Town10World"]["sha256"],
            urban["visual_source"]["release_sha256"],
        )
        self.assertEqual(sum(item["size"] for item in packages.values()), 7757662559)

    def test_runtime_file_identities_are_unique(self) -> None:
        identities = [
            (entry["root"], entry["path"])
            for entry in self.lock["runtime_files"]
        ]
        self.assertEqual(len(identities), len(set(identities)))

    def test_locked_acceptance_requires_no_fall(self) -> None:
        acceptance = self.lock["acceptance"]
        self.assertFalse(acceptance["fall_detected"])
        self.assertGreaterEqual(acceptance["physics_hz_min"], 195.0)
        self.assertGreaterEqual(acceptance["active_lowcmd_seconds_min"], 30.0)

    def test_two_host_profiles_use_repo_local_runtime(self) -> None:
        for profile in ("heyuan", "trna"):
            text = (REPO_ROOT / f"config/hosts/{profile}.env").read_text(
                encoding="utf-8"
            )
            self.assertIn("MATRIX_PROJECT_ROOT/outputs/runtime/matrix-sonic-v1", text)
            self.assertNotIn("TOKEN", text.upper())
            self.assertNotIn("PASSWORD", text.upper())

    def test_launcher_preserves_git_managed_config(self) -> None:
        text = (REPO_ROOT / "scripts/run_matrix_sonic.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("restore_tracked_config", text)
        self.assertIn(".matrix-sonic-launch.lock", text)
        self.assertIn("MATRIX_CPUSET_APPLIED", text)

    def test_chunk_installer_has_noninteractive_contract(self) -> None:
        text = (
            REPO_ROOT / "scripts/release_manager/install_chunks.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("MATRIX_MAPS", text)
        self.assertIn("MATRIX_ASSUME_YES", text)

    def test_artifact_packager_keeps_private_data_out_of_git(self) -> None:
        text = (
            REPO_ROOT / "scripts/package_matrix_sonic_artifacts.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("verify_matrix_sonic_runtime.py", text)
        self.assertIn("refusing to replace", text)
        self.assertIn("matrix-sonic-v1", text)

    def test_bootstrap_can_persist_an_ignored_runtime_path(self) -> None:
        text = (REPO_ROOT / "scripts/bootstrap_matrix_sonic.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("--runtime-root", text)
        self.assertIn("--write-local-env", text)
        self.assertIn(".matrix/local.env", text)


if __name__ == "__main__":
    unittest.main()
