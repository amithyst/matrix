from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
GENERATED = REPO_ROOT / "config/realscan/generated"


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class RealScanSceneTest(unittest.TestCase):
    def test_runtime_contract_keeps_matrix_3dgs_and_mujoco(self) -> None:
        lock = json.loads(
            (
                REPO_ROOT / "config/realscan/robot-training-ground.asset-lock.json"
            ).read_text()
        )
        self.assertEqual(lock["matrix_scene_id"], 18)
        self.assertEqual(lock["runtime"]["visual_backend"], "matrix_ue_threedgaussians")
        self.assertEqual(lock["runtime"]["physics_backend"], "sonic_mujoco")
        self.assertEqual(
            lock["runtime"]["forbidden_runtime_backends"],
            ["isaac_nurec", "isaac_physx"],
        )

    def test_generated_mujoco_proxy_matches_its_report(self) -> None:
        report = json.loads(
            (GENERATED / "robot_training_ground_mujoco_proxy.json").read_text()
        )
        self.assertEqual(report["physics_backend"], "mujoco")
        self.assertEqual(report["scope"], "lower_floor_navigation_proxy")
        self.assertGreaterEqual(report["boundary_box_count"], 300)
        self.assertLessEqual(report["boundary_box_count"], 900)
        for field in ("xml", "heightfield"):
            item = report[field]
            self.assertEqual(sha256_file(GENERATED / item["filename"]), item["sha256"])
        xml = (GENERATED / report["xml"]["filename"]).read_text()
        self.assertIn('<include file="xgb.xml" />', xml)
        self.assertIn('type="hfield"', xml)
        self.assertNotIn("physx", xml.lower())
        self.assertNotIn("isaac", xml.lower())

    def test_install_verifier_fails_closed_without_cooked_visual_bundle(self) -> None:
        completed = subprocess.run(
            [
                sys.executable,
                str(REPO_ROOT / "scripts/verify_realscan_scene_install.py"),
                "--project-root",
                str(REPO_ROOT),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(completed.returncode, 2)
        self.assertIn("installed physics asset", completed.stderr)

    def test_pico_launcher_fixes_scene_and_native_pico_source(self) -> None:
        launcher = REPO_ROOT / "scripts/run_matrix_pico_realscan.sh"
        completed = subprocess.run(
            ["bash", "-n", str(launcher)],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)
        text = launcher.read_text()
        self.assertIn("--scene 18", text)
        self.assertIn("run_matrix_pico.sh", text)

    def test_receipt_generator_hash_locks_a_cooked_trio(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            bundle = Path(temporary) / "bundle"
            bundle.mkdir()
            stem = "pakchunk88-RobotTrainingGround-Linux"
            expected = {}
            for suffix in (".pak", ".utoc", ".ucas"):
                package = bundle / f"{stem}{suffix}"
                package.write_bytes(f"fixture-{suffix}".encode())
                expected[package.name] = sha256_file(package)
            completed = subprocess.run(
                [
                    sys.executable,
                    str(REPO_ROOT / "scripts/create_realscan_scene_receipt.py"),
                    "--bundle-dir",
                    str(bundle),
                    "--ue-repository",
                    "xvirobotics/jszr_mujoco_ue2",
                    "--ue-commit",
                    "a" * 40,
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            receipt = json.loads((bundle / "receipt.json").read_text())
            self.assertEqual(receipt, json.loads(completed.stdout))
            self.assertEqual(receipt["map_name"], "/Game/Maps/RobotTrainingGround")
            self.assertEqual(
                {item["name"]: item["sha256"] for item in receipt["files"]},
                expected,
            )

    def test_receipt_generator_rejects_symlinked_or_mismatched_packages(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            bundle = root / "bundle"
            bundle.mkdir()
            stem = "pakchunk88-RobotTrainingGround-Linux"
            (bundle / f"{stem}.pak").write_bytes(b"pak")
            (bundle / f"{stem}.utoc").write_bytes(b"utoc")
            target = root / "outside.ucas"
            target.write_bytes(b"ucas")
            (bundle / f"{stem}.ucas").symlink_to(target)
            command = [
                sys.executable,
                str(REPO_ROOT / "scripts/create_realscan_scene_receipt.py"),
                "--bundle-dir",
                str(bundle),
                "--ue-repository",
                "xvirobotics/jszr_mujoco_ue2",
                "--ue-commit",
                "b" * 40,
            ]
            completed = subprocess.run(
                command, capture_output=True, text=True, check=False
            )
            self.assertEqual(completed.returncode, 2)
            self.assertIn("regular non-symlink", completed.stderr)
            self.assertFalse((bundle / "receipt.json").exists())

            (bundle / f"{stem}.ucas").unlink()
            (bundle / "pakchunk89-RobotTrainingGround-Linux.ucas").write_bytes(
                b"ucas"
            )
            completed = subprocess.run(
                command, capture_output=True, text=True, check=False
            )
            self.assertEqual(completed.returncode, 2)
            self.assertIn("matching pak/utoc/ucas trio", completed.stderr)

    def test_installer_and_verifier_accept_a_hash_locked_cooked_trio(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "matrix"
            bundle = Path(temporary) / "bundle"
            shutil.copytree(GENERATED, root / "config/realscan/generated")
            bundle.mkdir()
            files = []
            stem = "pakchunk88-RobotTrainingGround-Linux"
            for suffix in (".pak", ".utoc", ".ucas"):
                path = bundle / f"{stem}{suffix}"
                path.write_bytes(f"fixture-{suffix}".encode())
                files.append(
                    {
                        "name": path.name,
                        "size_bytes": path.stat().st_size,
                        "sha256": sha256_file(path),
                    }
                )
            receipt = {
                "schema": "matrix-realscan-ue-package-receipt/v1",
                "map_name": "/Game/Maps/RobotTrainingGround",
                "source_usdz_sha256": "2b67231becf613036d4acdec796cffcad9ae3e2456dd311a96f8a00932df85cd",
                "source_ply_sha256": "911399630534fa9df8b143c2437fd89c68176ec5fe53bb1317e7d2fec03b472c",
                "ue_project": {
                    "repository": "fixture/jszr_mujoco_ue2",
                    "commit": "a" * 40,
                },
                "files": files,
            }
            (bundle / "receipt.json").write_text(
                json.dumps(receipt, sort_keys=True) + "\n"
            )
            install = subprocess.run(
                [
                    sys.executable,
                    str(REPO_ROOT / "scripts/install_realscan_scene.py"),
                    "--project-root",
                    str(root),
                    "--visual-bundle-dir",
                    str(bundle),
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(install.returncode, 0, install.stderr)
            verified = json.loads(install.stdout)
            self.assertTrue(verified["ok"])
            self.assertEqual(verified["physics_backend"], "mujoco")
            self.assertEqual(verified["visual_backend"], "matrix_ue_threedgaussians")
            verify = subprocess.run(
                [
                    sys.executable,
                    str(REPO_ROOT / "scripts/verify_realscan_scene_install.py"),
                    "--project-root",
                    str(root),
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(verify.returncode, 0, verify.stderr)


if __name__ == "__main__":
    unittest.main()
