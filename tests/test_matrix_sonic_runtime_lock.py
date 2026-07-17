from __future__ import annotations

import copy
import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock


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
        self.assertIn(
            "src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux/zsibot_mujoco_ue",
            self.lock["matrix_release"]["installed_files"],
        )

    def test_runtime_file_identities_are_unique(self) -> None:
        identities = [
            (entry["root"], entry["path"])
            for entry in self.lock["runtime_files"]
        ]
        self.assertEqual(len(identities), len(set(identities)))
        tree_identities = [
            (entry["root"], entry["path"])
            for entry in self.lock["runtime_trees"]
        ]
        self.assertEqual(len(tree_identities), len(set(tree_identities)))
        self.assertIn(
            ("sonic", "gear_sonic_deploy/reference/example"), tree_identities
        )
        self.assertIn(
            ("sonic", "gear_sonic/data/robot_model/model_data/g1/meshes"),
            tree_identities,
        )
        self.assertIn(
            (
                "sonic",
                "gear_sonic_deploy/thirdparty/unitree_sdk2/thirdparty/lib/x86_64",
            ),
            tree_identities,
        )

        verifier = SCRIPT_PATH.read_text(encoding="utf-8")
        self.assertIn("native SONIC commit", verifier)
        self.assertIn("native SONIC Python API", verifier)
        self.assertNotIn("UDP/DDS bridge", verifier)

        serialized = json.dumps(self.lock).lower()
        for legacy in ("androidtwin", "aue-sim", "g1_sonic_sim_udp_dds_bridge"):
            self.assertNotIn(legacy, serialized)

    def test_runtime_tree_schema_requires_safe_unique_entries(self) -> None:
        for mutation in ("empty", "duplicate", "unsafe_root", "unsafe_path", "bad_sha"):
            lock = copy.deepcopy(self.lock)
            if mutation == "empty":
                lock["runtime_trees"] = []
            elif mutation == "duplicate":
                lock["runtime_trees"].append(copy.deepcopy(lock["runtime_trees"][0]))
            elif mutation == "unsafe_root":
                lock["runtime_trees"][0]["root"] = "../sonic"
            elif mutation == "unsafe_path":
                lock["runtime_trees"][0]["path"] = "../example"
            else:
                lock["runtime_trees"][0]["sha256"] = "A" * 64
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                MODULE.validate_schema(lock)

        lock = copy.deepcopy(self.lock)
        lock["runtime_trees"][0]["verification"] = "provisional checkout"
        with self.assertRaisesRegex(ValueError, "provisional runtime_trees"):
            MODULE.validate_schema(lock)

    def test_runtime_tree_hash_is_deterministic(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "tree"
            (root / "nested").mkdir(parents=True)
            (root / "nested/z.bin").write_bytes(b"\x00beta")
            (root / "a.txt").write_bytes(b"alpha\n")

            actual, count = MODULE.sha256_tree(root)
            self.assertEqual(count, 2)
            self.assertEqual(
                actual,
                "c9e60c95745e371d14527231805daf59cf5177251b330a585e1d8b270ec1c4f5",
            )

    def test_runtime_tree_hash_rejects_symlinks_and_non_regular_files(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "tree"
            root.mkdir()
            (root / "target").write_bytes(b"target")
            link = root / "link"
            link.symlink_to("target")
            with self.assertRaisesRegex(ValueError, "symlink"):
                MODULE.sha256_tree(root)

            link.unlink()
            fifo = root / "fifo"
            os.mkfifo(fifo)
            with self.assertRaisesRegex(ValueError, "non-regular"):
                MODULE.sha256_tree(root)

    def test_runtime_tree_attestation_covers_checkout_and_archive(self) -> None:
        relative = "gear_sonic_deploy/reference/example"
        with tempfile.TemporaryDirectory() as temporary:
            temporary_root = Path(temporary)
            checkout = temporary_root / "checkout"
            archive = temporary_root / "archive"
            for sonic_root in (checkout, archive):
                tree = sonic_root / relative
                tree.mkdir(parents=True)
                (tree / "reference.csv").write_bytes(b"locked reference\n")

            expected, _ = MODULE.sha256_tree(checkout / relative)
            checkout_ok, checkout_detail = MODULE.runtime_tree_attestation(
                checkout, relative, expected
            )
            archive_ok, archive_detail = MODULE.runtime_tree_attestation(
                archive, relative, expected
            )
            self.assertTrue(checkout_ok, checkout_detail)
            self.assertTrue(archive_ok, archive_detail)

            (archive / relative / "reference.csv").write_bytes(b"changed\n")
            archive_ok, _ = MODULE.runtime_tree_attestation(
                archive, relative, expected
            )
            self.assertFalse(archive_ok)

    def test_locked_acceptance_requires_no_fall(self) -> None:
        acceptance = self.lock["acceptance"]
        self.assertFalse(acceptance["fall_detected"])
        self.assertGreaterEqual(acceptance["physics_hz_min"], 195.0)
        self.assertGreater(acceptance["low_cmd_fresh_timeout_seconds"], 0.0)
        self.assertGreater(acceptance["root_displacement_xy_min_m"], 0.0)
        self.assertGreaterEqual(acceptance["active_lowcmd_seconds_min"], 30.0)
        self.assertEqual(self.lock["python"]["version"], "3.10")
        self.assertEqual(
            self.lock["python"]["soabi"], "cpython-310-x86_64-linux-gnu"
        )
        self.assertEqual(self.lock["python"]["machine"], "x86_64")
        requirements = REPO_ROOT / self.lock["python"]["requirements"]
        self.assertEqual(
            hashlib.sha256(requirements.read_bytes()).hexdigest(),
            self.lock["python"]["requirements_sha256"],
        )
        pins = MODULE.parse_pinned_requirements(requirements)
        for distribution, version in (
            ("torch", "2.12.1"),
            ("mujoco", "3.10.0"),
            ("pyzmq", "27.1.0"),
            ("cyclonedds", "0.10.2"),
            ("unitree-sdk2py", "1.0.1"),
        ):
            self.assertEqual(pins[distribution][1], version)
        self.assertEqual(
            pins["mujoco"][1], self.lock["python"]["mujoco"]
        )
        self.assertEqual(
            self.lock["python"]["wheelhouse_manifest_sha256"],
            "1044d7c4e04648ef2e58a87b201b247082d4a8ec2bcc265611d5e3a344c8304b",
        )
        self.assertEqual(self.lock["pico"]["version"], "1.0.2")
        self.assertEqual(
            self.lock["pico"]["wheel_filename"],
            "xrobotoolkit_sdk-1.0.2-cp310-cp310-linux_x86_64.whl",
        )
        self.assertEqual(
            self.lock["pico"]["wheel_sha256"],
            "6dda05341c23bcf1986148324fa86ce07b972705645b5582838259fe4ce1287c",
        )
        self.assertNotIn("manifest_sha256", self.lock["pico"])
        self.assertEqual(
            self.lock["pico"]["runtime_overlay_sha256"],
            "2348e1b0bf7f05cd13d95d628f04237b6b9fc50c6008b82f9bf4b046fc9373e6",
        )
        self.assertEqual(self.lock["schema_version"], 2)
        self.assertEqual(self.lock["runtime_id"], "matrix-sonic-native-v2")
        self.assertEqual(
            self.lock["source_revisions"]["gr00t_whole_body_control"]["commit"],
            "de083d71af8346b0124ab1ae79fd3623b52c3c9b",
        )

    def test_host_profiles_use_repo_local_runtime(self) -> None:
        for profile in ("heyuan", "trna", "zza"):
            text = (REPO_ROOT / f"config/hosts/{profile}.env").read_text(
                encoding="utf-8"
            )
            self.assertIn(
                "MATRIX_PROJECT_ROOT/outputs/runtime/matrix-sonic-native-v2", text
            )
            self.assertIn("MATRIX_SONIC_ROOT", text)
            self.assertNotIn("TOKEN", text.upper())
            self.assertNotIn("PASSWORD", text.upper())

        self.assertEqual(MODULE.HOST_PROFILES, ("heyuan", "trna", "zza"))
        trna = (REPO_ROOT / "config/hosts/trna.env").read_text(encoding="utf-8")
        self.assertIn("worktrees/sonic-matrix-native-final", trna)
        self.assertNotIn("code_bryce", trna)
        zza = (REPO_ROOT / "config/hosts/zza.env").read_text(encoding="utf-8")
        self.assertIn('DISPLAY="${DISPLAY:-:1}"', zza)
        self.assertIn("MATRIX_RUNTIME_ROOT/ros2-humble-prefix", zza)
        self.assertIn("MATRIX_CUDA_ROOT", zza)
        self.assertIn('PATH="$MATRIX_TOOLS_ROOT/bin:$PATH"', zza)

    def test_release_installs_do_not_dirty_the_checkout(self) -> None:
        text = (REPO_ROOT / ".gitignore").read_text(encoding="utf-8")
        for pattern in (
            ".venv*/",
            "bin/*",
            "dynamicmaps/*",
            "src/UeSim/Linux/Engine/*",
            "src/UeSim/Linux/zsibot_mujoco_ue/Binaries/",
            "src/UeSim/Linux/zsibot_mujoco_ue/Samples/",
            "src/UeSim/Linux/zsibot_mujoco_ue/Saved/",
        ):
            self.assertIn(pattern, text)

    def test_library_paths_use_profile_cuda_root(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            cuda_lib = root / "cuda/lib64"
            cuda_lib.mkdir(parents=True)
            with mock.patch.dict(os.environ, {"MATRIX_CUDA_ROOT": str(root / "cuda")}):
                paths = MODULE.library_paths(
                    root / "runtime", REPO_ROOT, root / "sonic"
                )
            self.assertIn(cuda_lib, paths)

    def test_launcher_preserves_git_managed_config(self) -> None:
        text = (REPO_ROOT / "scripts/run_matrix_sonic.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("restore_tracked_config", text)
        self.assertIn("MATRIX_SONIC_HOST_LOCK", text)
        self.assertIn("/tmp/matrix-sonic-${UID}.lock", text)
        self.assertIn("read_acceptance_lock", text)
        self.assertIn("MATRIX_CPUSET_APPLIED", text)
        self.assertIn("/usr/bin/env MATRIX_CPUSET_APPLIED=1", text)
        self.assertIn('prepend_library_dir "$MATRIX_CUDA_ROOT/lib"', text)
        self.assertNotIn("code_bryce", text)

        verifier = (
            REPO_ROOT / "scripts/verify_matrix_sonic_runtime.py"
        ).read_text(encoding="utf-8")
        self.assertIn('soname = "libcudart.so.12"', verifier)
        self.assertIn('os.environ.get("MATRIX_CUDA_ROOT", "/usr/local/cuda")', verifier)

        run_sim = (REPO_ROOT / "scripts/run_sim.sh").read_text(encoding="utf-8")
        self.assertIn('case "${MATRIX_SONIC,,}"', run_sim)
        self.assertIn("checked_mujoco=0", run_sim)
        self.assertIn('--mujoco "$checked_mujoco"', run_sim)
        self.assertIn("attempt < 150", run_sim)
        self.assertIn('--expected-parent-pid "$$"', run_sim)
        self.assertIn("RUN_SIM_PARENT_PID", run_sim)
        self.assertIn("handle_signal 143", run_sim)

        self.assertIn("forward_signal TERM 143", text)
        self.assertIn("FORWARDED_SIGNAL_EXIT_CODE", text)
        self.assertIn("Bounded qualification requires --profile", text)
        self.assertIn("MATRIX_SONIC_QUALIFIED_RUNTIME", text)

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
        self.assertIn("matrix-sonic-native-v2", text)
        self.assertIn("--sonic-root", text)
        self.assertIn("--python", text)
        self.assertIn('ACTUAL_SONIC_COMMIT="$(git -C "$SONIC_ROOT" rev-parse HEAD)"', text)
        self.assertIn('archive "$EXPECTED_SONIC_COMMIT"', text)
        self.assertIn("CRITICAL_SOURCE_PATHS", text)
        self.assertIn("SONIC_RUNTIME_FILES", text)
        self.assertIn("SONIC_RUNTIME_TREES", text)
        self.assertIn("only files and trees attested by the runtime lock", text)
        self.assertNotIn("for directory in policy planner reference thirdparty", text)
        self.assertIn("--untracked-files=all", text)
        self.assertIn("git-lfs.github.com/spec/v1", text)
        self.assertIn('"runtime_id": lock["runtime_id"]', text)
        self.assertIn('"release_ready": False', text)
        self.assertIn('--python "$RUNTIME_PYTHON"', text)
        self.assertNotIn("androidtwin", text.lower())
        self.assertNotIn("g1_sonic_sim_udp_dds_bridge", text)

        runtime_paths = {
            (entry["root"], entry["path"])
            for entry in self.lock["runtime_files"]
        }
        self.assertIn(
            (
                "sonic",
                "gear_sonic_deploy/target/release/g1_deploy_onnx_ref",
            ),
            runtime_paths,
        )

    def test_artifact_packager_rejects_output_inside_an_input_root(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            arguments = []
            for option in ("sonic-root", "inference-root", "visual-root", "wheelhouse"):
                directory = root / option
                directory.mkdir()
                arguments.extend((f"--{option}", str(directory)))
            output = root / "wheelhouse/artifact"
            result = subprocess.run(
                [
                    "bash",
                    str(REPO_ROOT / "scripts/package_matrix_sonic_artifacts.sh"),
                    *arguments,
                    "--python",
                    sys.executable,
                    "--output",
                    str(output),
                ],
                text=True,
                capture_output=True,
                check=False,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("must not be inside an input root", result.stderr)

    def test_bootstrap_can_persist_an_ignored_runtime_path(self) -> None:
        text = (REPO_ROOT / "scripts/bootstrap_matrix_sonic.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("--runtime-root", text)
        self.assertIn("--write-local-env", text)
        self.assertIn(".matrix/local.env", text)
        self.assertIn("--no-index", text)
        self.assertIn("--only-binary=:all:", text)
        self.assertIn("python-wheelhouse", text)
        self.assertIn('python_lock["requirements_sha256"]', text)
        self.assertIn("EXPECTED_REQUIREMENTS_SHA256", text)
        self.assertIn(".matrix-lock-requirements.sha256", text)
        self.assertIn("runtime lock or requirements changed", text)
        self.assertIn("-m pip check", text)
        self.assertNotIn("--require-hashes", text)
        self.assertNotIn(
            'PROJECT_ROOT/research/sonic_integration/requirements-trna.txt', text
        )
        self.assertIn('--python "$RUNTIME_PYTHON"', text)
        self.assertIn('PROFILE_FILE="$PROJECT_ROOT/config/hosts/$PROFILE.env"', text)
        self.assertIn(
            'deploy="$MATRIX_SONIC_ROOT/gear_sonic_deploy/target/release/g1_deploy_onnx_ref"',
            text,
        )

    def test_verifier_binds_python_source_and_wheelhouse_identity(self) -> None:
        verifier = SCRIPT_PATH.read_text(encoding="utf-8")
        for value in (
            "native runtime Python SOABI",
            "native runtime machine",
            "Python requirements lock",
            "Python wheelhouse inventory",
            "native PICO wheel artifact",
            "--pico-wheel",
            "gear_sonic import origin",
            "unitree_sdk2py Python package",
            "cyclonedds Python package",
            "archived SONIC critical source attestation",
            "Matrix tracked source clean",
            "git_checkout_root",
        ):
            self.assertIn(value, verifier)

    def test_wheelhouse_inventory_rejects_unlisted_files(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            wheelhouse = Path(temporary)
            wheel = wheelhouse / "locked-1.0-py3-none-any.whl"
            wheel.write_bytes(b"locked wheel")
            digest = hashlib.sha256(wheel.read_bytes()).hexdigest()
            manifest = wheelhouse / "SHA256SUMS"
            manifest.write_text(f"{digest}  {wheel.name}\n", encoding="utf-8")
            manifest_digest = hashlib.sha256(manifest.read_bytes()).hexdigest()

            checks = MODULE.verify_wheelhouse(wheelhouse, manifest_digest)
            self.assertTrue(all(ok for _, ok, _ in checks), checks)

            (wheelhouse / "extra.whl").write_bytes(b"not listed")
            checks = MODULE.verify_wheelhouse(wheelhouse, manifest_digest)
            inventory = {name: ok for name, ok, _ in checks}
            self.assertFalse(inventory["Python wheelhouse inventory"])

    def test_wheelhouse_rejects_wrong_python_or_platform_tags(self) -> None:
        for filename in (
            "wrong-1.0-cp311-cp311-manylinux_2_28_aarch64.whl",
            "wrong-1.0-cp310-cp310-macosx_11_0_x86_64.whl",
            "wrong-1.0-cp310-cp310-musllinux_1_2_x86_64.whl",
            "wrong-1.0-cp310-cp310-manylinux_2_36_x86_64.whl",
            "wrong-1.0-cp310-cp310-manylinux_2_035_x86_64.whl",
            "wrong-1.0-cp310-cp310-manylinux_3_20_x86_64.whl",
            "wrong-1.0-cp310-cp310-manylinux2015_x86_64.whl",
            "wrong-1.0-cp310-cp310-manylinux_2_x86_64.whl",
            (
                "wrong-1.0-cp310-cp310-"
                "manylinux_2_35_x86_64.manylinux_2_36_x86_64.whl"
            ),
            "garbage-cp310-none-any.whl",
        ):
            with self.subTest(filename=filename), tempfile.TemporaryDirectory() as temporary:
                wheelhouse = Path(temporary)
                wheel = wheelhouse / filename
                wheel.write_bytes(b"wrong ABI")
                digest = hashlib.sha256(wheel.read_bytes()).hexdigest()
                manifest = wheelhouse / "SHA256SUMS"
                manifest.write_text(f"{digest}  {wheel.name}\n", encoding="utf-8")

                checks = MODULE.verify_wheelhouse(
                    wheelhouse, hashlib.sha256(manifest.read_bytes()).hexdigest()
                )
                results = {name: ok for name, ok, _ in checks}
                self.assertFalse(results["Python wheelhouse compatibility"])

    def test_wheelhouse_accepts_locked_glibc_and_legacy_manylinux_tags(self) -> None:
        for filename in (
            "pure-1.0-py3-none-any.whl",
            "linux-1.0-cp310-cp310-linux_x86_64.whl",
            "legacy1-1.0-cp310-cp310-manylinux1_x86_64.whl",
            "legacy2010-1.0-cp310-cp310-manylinux2010_x86_64.whl",
            "legacy2014-1.0-cp310-cp310-manylinux2014_x86_64.whl",
            "pep600-1.0-cp310-cp310-manylinux_2_35_x86_64.whl",
            "stable-1.0-cp37-abi3-manylinux_2_17_x86_64.whl",
            (
                "compressed-1.0-cp310-cp310-"
                "manylinux_2_17_x86_64.manylinux2014_x86_64.whl"
            ),
        ):
            with self.subTest(filename=filename), tempfile.TemporaryDirectory() as temporary:
                wheelhouse = Path(temporary)
                wheel = wheelhouse / filename
                wheel.write_bytes(b"compatible wheel")
                digest = hashlib.sha256(wheel.read_bytes()).hexdigest()
                manifest = wheelhouse / "SHA256SUMS"
                manifest.write_text(f"{digest}  {wheel.name}\n", encoding="utf-8")

                checks = MODULE.verify_wheelhouse(
                    wheelhouse, hashlib.sha256(manifest.read_bytes()).hexdigest()
                )
                self.assertTrue(all(ok for _, ok, _ in checks), checks)

    def test_wheelhouse_rejects_nested_wheels_not_seen_by_pip_find_links(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            wheelhouse = Path(temporary)
            wheel = wheelhouse / "nested/locked-1.0-py3-none-any.whl"
            wheel.parent.mkdir()
            wheel.write_bytes(b"nested wheel")
            digest = hashlib.sha256(wheel.read_bytes()).hexdigest()
            manifest = wheelhouse / "SHA256SUMS"
            manifest.write_text(
                f"{digest}  nested/{wheel.name}\n", encoding="utf-8"
            )
            checks = MODULE.verify_wheelhouse(
                wheelhouse, hashlib.sha256(manifest.read_bytes()).hexdigest()
            )
            results = {name: ok for name, ok, _ in checks}
            self.assertFalse(results["Python wheelhouse compatibility"])

    def test_pico_wheel_must_match_locked_filename_and_sha(self) -> None:
        pico_lock = copy.deepcopy(self.lock["pico"])
        with tempfile.TemporaryDirectory() as temporary:
            wheel = Path(temporary) / pico_lock["wheel_filename"]
            wheel.write_bytes(b"locked PICO wheel")
            pico_lock["wheel_sha256"] = hashlib.sha256(wheel.read_bytes()).hexdigest()
            self.assertTrue(MODULE.verify_pico_wheel(wheel, pico_lock)[0])

            wheel.write_bytes(b"modified PICO wheel")
            ok, detail = MODULE.verify_pico_wheel(wheel, pico_lock)
            self.assertFalse(ok)
            self.assertIn("sha256 expected=", detail)

            renamed = wheel.with_name("unexpected-1.0-py3-none-any.whl")
            wheel.rename(renamed)
            pico_lock["wheel_sha256"] = hashlib.sha256(renamed.read_bytes()).hexdigest()
            ok, detail = MODULE.verify_pico_wheel(renamed, pico_lock)
            self.assertFalse(ok)
            self.assertIn("filename expected=", detail)

            missing = Path(temporary) / pico_lock["wheel_filename"]
            ok, detail = MODULE.verify_pico_wheel(missing, pico_lock)
            self.assertFalse(ok)
            self.assertIn("missing regular file", detail)

            with mock.patch.object(
                MODULE, "sha256_file", side_effect=OSError("unreadable")
            ):
                ok, detail = MODULE.verify_pico_wheel(renamed, pico_lock)
            self.assertFalse(ok)
            self.assertIn("cannot hash PICO wheel", detail)

    def test_pico_delivery_is_explicitly_external(self) -> None:
        lock = copy.deepcopy(self.lock)
        lock["pico"]["delivery"] = "bundled"
        with self.assertRaisesRegex(
            ValueError, "external-controlled-environment"
        ):
            MODULE.validate_schema(lock)

    def test_pico_wheel_identity_matches_lock_metadata(self) -> None:
        lock = copy.deepcopy(self.lock)
        lock["pico"]["wheel_filename"] = (
            "other_pkg-9.9-cp310-cp310-linux_x86_64.whl"
        )
        with self.assertRaisesRegex(
            ValueError, "distribution/version must match"
        ):
            MODULE.validate_schema(lock)

    def test_requirements_parser_accepts_only_unique_exact_pins(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            requirements = Path(temporary) / "requirements.txt"
            requirements.write_text(
                "NumPy==1.26.4\nunitree_sdk2py==1.0.1\n", encoding="utf-8"
            )
            pins = MODULE.parse_pinned_requirements(requirements)
            self.assertEqual(pins["numpy"], ("NumPy", "1.26.4"))
            self.assertEqual(
                pins["unitree-sdk2py"], ("unitree_sdk2py", "1.0.1")
            )

            requirements.write_text("numpy>=1.26\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "exact distribution==version"):
                MODULE.parse_pinned_requirements(requirements)

    def test_archive_marker_requires_locked_critical_source_hashes(self) -> None:
        lock = copy.deepcopy(self.lock)
        critical_paths = lock["source_revisions"][
            "gr00t_whole_body_control"
        ]["critical_source_paths"]
        critical_identities = {
            ("sonic", relative) for relative in critical_paths
        }
        lock["runtime_files"] = [
            entry
            for entry in lock["runtime_files"]
            if (entry["root"], entry["path"]) not in critical_identities
        ]
        with tempfile.TemporaryDirectory() as temporary:
            sonic_root = Path(temporary)
            for relative in critical_paths:
                path = sonic_root / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(f"source:{relative}\n", encoding="utf-8")
                lock["runtime_files"].append(
                    {
                        "root": "sonic",
                        "path": relative,
                        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                    }
                )

            ok, detail = MODULE.archived_source_attestation(lock, sonic_root)
            self.assertTrue(ok, detail)
            lock["runtime_files"].pop()
            ok, detail = MODULE.archived_source_attestation(lock, sonic_root)
            self.assertFalse(ok)
            self.assertIn("not locked", detail)

    def test_bootstrap_supports_python_without_ensurepip(self) -> None:
        text = (REPO_ROOT / "scripts/bootstrap_matrix_sonic.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("import ensurepip", text)
        self.assertIn('"$BOOTSTRAP_PYTHON" -m venv --without-pip', text)
        self.assertIn(".matrix-external-pip", text)
        self.assertIn('--target "$audit_site_packages"', text)
        self.assertIn("--ignore-installed", text)
        self.assertIn("Recreating non-isolated .venv-audit", text)
        self.assertIn("Recreating incomplete .venv-audit", text)

    def test_release_cache_is_materialized_without_network_or_symlinks(self) -> None:
        bootstrap = (
            REPO_ROOT / "scripts/bootstrap_matrix_sonic.sh"
        ).read_text(encoding="utf-8")
        installer = (
            REPO_ROOT / "scripts/release_manager/install_chunks.sh"
        ).read_text(encoding="utf-8")

        self.assertIn('ln "$source_path" "$destination_path"', bootstrap)
        self.assertIn("cp --reflink=auto", bootstrap)
        self.assertNotIn('ln -sfn "$source_path"', bootstrap)
        self.assertIn("MATRIX_OFFLINE=1", bootstrap)
        self.assertIn('/usr/bin/env "${INSTALL_ENV[@]}"', bootstrap)
        self.assertIn('MATRIX_OFFLINE="${MATRIX_OFFLINE:-0}"', installer)
        self.assertIn("离线模式下禁止下载", installer)

    def test_local_runtime_override_precedes_profile_derived_paths(self) -> None:
        for script_name in ("bootstrap_matrix_sonic.sh", "run_matrix_sonic.sh"):
            text = (REPO_ROOT / "scripts" / script_name).read_text(
                encoding="utf-8"
            )
            local_env_source = text.index(
                'source "$PROJECT_ROOT/.matrix/local.env"'
            )
            profile_source = text.index(
                'source "$PROFILE_FILE"'
                if script_name == "bootstrap_matrix_sonic.sh"
                else 'source "$PROFILE_FILE"'
            )
            self.assertLess(local_env_source, profile_source, script_name)

        bootstrap = (
            REPO_ROOT / "scripts/bootstrap_matrix_sonic.sh"
        ).read_text(encoding="utf-8")
        override_assignment = bootstrap.index(
            'export MATRIX_RUNTIME_ROOT="$RUNTIME_OVERRIDE"'
        )
        profile_source = bootstrap.index(
            'source "$PROFILE_FILE"'
        )
        self.assertLess(override_assignment, profile_source)

    def test_active_launch_path_has_no_androidtwin_dependency(self) -> None:
        for relative in (
            "scripts/run_matrix_sonic.py",
            "scripts/run_matrix_sonic.sh",
            "scripts/run_matrix_sonic_overworld_v1.sh",
            "scripts/run_sim.sh",
            "scripts/bootstrap_matrix_sonic.sh",
            "scripts/package_matrix_sonic_artifacts.sh",
        ):
            text = (REPO_ROOT / relative).read_text(encoding="utf-8").lower()
            self.assertNotIn("androidtwin", text, relative)
            self.assertNotIn("matrix_aue_root", text, relative)
            self.assertNotIn("androidtwin_", text, relative)

        overworld = (
            REPO_ROOT / "scripts/run_matrix_sonic_overworld_v1.sh"
        ).read_text(encoding="utf-8")
        self.assertIn('--sonic-root "$MATRIX_SONIC_ROOT"', overworld)
        self.assertIn('--pico-wheel "$MATRIX_PICO_WHEEL"', overworld)
        self.assertIn("Locked PICO acceptance requires --profile", overworld)
        for gate in (
            '--min-active-seconds "$MIN_ACTIVE_SECONDS"',
            '--min-displacement-m "$MIN_DISPLACEMENT_M"',
            '--min-final-x "$FINAL_X_MIN"',
            '--min-forward-x-m "$MIN_FORWARD_X_M"',
            '--min-physics-hz "$MIN_PHYSICS_HZ"',
            '--min-rtf "$MIN_RTF"',
            "--qualified-runtime",
            '--scenario-layout-sha256 "${QUALIFICATION_HASHES[1]}"',
        ):
            self.assertIn(gate, overworld)
        for forbidden_case in (
            "--layout) LAYOUT=",
            "--spawn-x) SPAWN_X=",
            "--spawn-y) SPAWN_Y=",
            "--spawn-z) SPAWN_Z=",
            "--spawn-yaw) SPAWN_YAW=",
        ):
            self.assertNotIn(forbidden_case, overworld)
        overworld_lock = overworld.index('if ! flock -n 9; then')
        status_cleanup = overworld.index('rm -f -- "$STATUS_FILE"')
        compose = overworld.index('"$PROJECT_ROOT/scripts/compose_overworld_scene.py"')
        self.assertLess(overworld_lock, status_cleanup)
        self.assertLess(status_cleanup, compose)
        for legacy_argument in (
            "--aue-root",
            "--gear-sonic-root",
            "--unitree-sdk-root",
        ):
            self.assertNotIn(legacy_argument, overworld)

        primary = (REPO_ROOT / "scripts/run_matrix_sonic.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn('--pico-wheel "$MATRIX_PICO_WHEEL"', primary)
        self.assertIn("Locked PICO acceptance requires --profile", primary)
        primary_status_cleanup = primary.index(
            'rm -f -- "$MATRIX_SONIC_STATUS_FILE"'
        )
        primary_lock = primary.index('if ! flock -n 9; then')
        primary_verifier = primary.index(
            '"$PROJECT_ROOT/scripts/verify_matrix_sonic_runtime.py"'
        )
        self.assertLess(primary_lock, primary_status_cleanup)
        self.assertLess(primary_status_cleanup, primary_verifier)

    def test_rejected_concurrent_launch_preserves_active_status(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            lock_path = root / "matrix-sonic.lock"
            fake_bin = root / "bin"
            fake_bin.mkdir()
            fake_jq = fake_bin / "jq"
            fake_jq.write_text("#!/usr/bin/env bash\nexit 99\n", encoding="utf-8")
            fake_jq.chmod(0o755)

            with lock_path.open("w", encoding="utf-8") as lock_stream:
                fcntl.flock(lock_stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                for script_name, status_variable in (
                    ("run_matrix_sonic.sh", "MATRIX_SONIC_STATUS_FILE"),
                    (
                        "run_matrix_sonic_overworld_v1.sh",
                        "MATRIX_OVERWORLD_STATUS_FILE",
                    ),
                ):
                    with self.subTest(script=script_name):
                        status = root / f"{script_name}.status"
                        status.write_text("active-run-sentinel\n", encoding="utf-8")
                        environment = os.environ.copy()
                        environment.update(
                            {
                                "MATRIX_SONIC_HOST_LOCK": str(lock_path),
                                status_variable: str(status),
                                "PATH": f"{fake_bin}{os.pathsep}{environment['PATH']}",
                            }
                        )
                        environment.pop("MATRIX_PROFILE", None)
                        environment.pop("MATRIX_CPUSET", None)
                        result = subprocess.run(
                            ["bash", str(REPO_ROOT / "scripts" / script_name)],
                            cwd=REPO_ROOT,
                            env=environment,
                            text=True,
                            capture_output=True,
                            check=False,
                        )
                        self.assertNotEqual(result.returncode, 0)
                        self.assertIn("Another Matrix SONIC launcher", result.stderr)
                        self.assertEqual(
                            status.read_text(encoding="utf-8"),
                            "active-run-sentinel\n",
                        )

    def test_bounded_launch_requires_profile_verification(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fake_bin = root / "bin"
            fake_bin.mkdir()
            fake_jq = fake_bin / "jq"
            fake_jq.write_text("#!/usr/bin/env bash\nexit 99\n", encoding="utf-8")
            fake_jq.chmod(0o755)
            for script_name, extra_args, expected_error in (
                (
                    "run_matrix_sonic.sh",
                    ("--max-seconds", "1"),
                    "Bounded qualification requires --profile",
                ),
                (
                    "run_matrix_sonic_overworld_v1.sh",
                    (),
                    "Bounded Overworld qualification requires --profile",
                ),
            ):
                with self.subTest(script=script_name):
                    environment = os.environ.copy()
                    environment.update(
                        {
                            "MATRIX_SONIC_HOST_LOCK": str(
                                root / f"{script_name}.lock"
                            ),
                            "PATH": f"{fake_bin}{os.pathsep}{environment['PATH']}",
                        }
                    )
                    environment.pop("MATRIX_PROFILE", None)
                    environment.pop("MATRIX_CPUSET", None)
                    environment.pop("MATRIX_VERIFY_RUNTIME", None)
                    result = subprocess.run(
                        [
                            "bash",
                            str(REPO_ROOT / "scripts" / script_name),
                            *extra_args,
                        ],
                        cwd=REPO_ROOT,
                        env=environment,
                        text=True,
                        capture_output=True,
                        check=False,
                    )
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn(expected_error, result.stderr)


if __name__ == "__main__":
    unittest.main()
