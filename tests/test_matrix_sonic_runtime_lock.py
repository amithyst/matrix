from __future__ import annotations

import base64
import copy
import csv
import fcntl
import hashlib
import io
import importlib.util
import json
import os
from pathlib import Path
import shutil
import shlex
import subprocess
import sys
import tempfile
import unittest
from unittest import mock
import zipfile


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "verify_matrix_sonic_runtime.py"
SPEC = importlib.util.spec_from_file_location("verify_matrix_sonic_runtime", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)

LOCAL_ENV_SCRIPT = REPO_ROOT / "scripts" / "update_matrix_local_env.py"
LOCAL_ENV_SPEC = importlib.util.spec_from_file_location(
    "update_matrix_local_env", LOCAL_ENV_SCRIPT
)
assert LOCAL_ENV_SPEC is not None and LOCAL_ENV_SPEC.loader is not None
LOCAL_ENV_MODULE = importlib.util.module_from_spec(LOCAL_ENV_SPEC)
LOCAL_ENV_SPEC.loader.exec_module(LOCAL_ENV_MODULE)


def write_test_wheel(
    wheelhouse: Path,
    filename: str,
    files: dict[str, bytes],
) -> tuple[Path, str, dict[str, bytes]]:
    distribution, version, _, _, _ = MODULE.parse_wheel_filename(filename)
    stem = f"{distribution.replace('.', '_')}-{version}"
    record_path = f"{stem}.dist-info/RECORD"
    wheel_files = dict(files)
    output = io.StringIO(newline="")
    writer = csv.writer(output, lineterminator="\n")
    for relative, content in sorted(wheel_files.items()):
        digest = base64.urlsafe_b64encode(hashlib.sha256(content).digest()).rstrip(b"=")
        writer.writerow((relative, f"sha256={digest.decode('ascii')}", len(content)))
    writer.writerow((record_path, "", ""))
    wheel_files[record_path] = output.getvalue().encode("utf-8")

    wheel = wheelhouse / filename
    with zipfile.ZipFile(wheel, "w", compression=zipfile.ZIP_STORED) as archive:
        for relative, content in wheel_files.items():
            archive.writestr(relative, content)
    return wheel, record_path, wheel_files


class MatrixSonicRuntimeLockTest(unittest.TestCase):
    def setUp(self) -> None:
        self.lock_path = REPO_ROOT / "config/runtime/matrix-sonic.lock.json"
        self.lock = MODULE.load_lock(self.lock_path)

    def test_policy_slot_manifest_is_schema_checked_and_content_locked(self) -> None:
        MODULE.validate_schema(self.lock)
        MODULE.validate_policy_manifest_files(self.lock, REPO_ROOT)
        entry = self.lock["policy_slots"]["manifests"][0]
        manifest = REPO_ROOT / entry["path"]
        self.assertEqual(
            hashlib.sha256(manifest.read_bytes()).hexdigest(),
            entry["sha256"],
        )

        bad_hash = copy.deepcopy(self.lock)
        bad_hash["policy_slots"]["manifests"][0]["sha256"] = "0" * 64
        with self.assertRaisesRegex(ValueError, "SHA256 mismatch"):
            MODULE.validate_policy_manifest_files(bad_hash, REPO_ROOT)

        unsafe = copy.deepcopy(self.lock)
        unsafe["policy_slots"]["manifests"][0]["path"] = "../candidate.json"
        with self.assertRaisesRegex(ValueError, "invalid or duplicate"):
            MODULE.validate_schema(unsafe)

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
        installed = {
            entry["path"]: entry
            for entry in self.lock["matrix_release"]["installed_files"]
        }
        self.assertIn(
            "src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux/zsibot_mujoco_ue",
            installed,
        )
        self.assertEqual(
            installed[
                "src/robot_mujoco/zsibot_robots/xgb/scene_terrain_t10.xml"
            ]["sha256"],
            "7784452106dc0bce57588d3c148a6117798c583a7675b6414ca9d40139ee7df6",
        )
        self.assertEqual(
            self.lock["matrix_release"]["installed_trees"][0]["sha256"],
            "9ebc024fa07ddf2deb6a9939bb276dea03b1c6d9e5dfee932b181800b7811232",
        )
        for required in (
            "src/UeSim/Linux/Engine/Binaries/Linux/libEOSSDK-Linux-Shipping.so",
            "src/robot_mujoco/zsibot_robots/xgb/height_field.png",
            "src/robot_mujoco/zsibot_robots/xgb/unitree_hfield.png",
        ):
            self.assertIn(required, installed)
        installed_trees = {
            entry["path"] for entry in self.lock["matrix_release"]["installed_trees"]
        }
        for required in (
            "src/UeSim/Linux/zsibot_mujoco_ue/Content/Paks",
            "src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux",
            "src/UeSim/Linux/Engine/Plugins/Runtime/OpenCV/Binaries/ThirdParty/Linux",
            "src/UeSim/Linux/zsibot_mujoco_ue/Content/model/xgb",
        ):
            self.assertIn(required, installed_trees)

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
        self.assertIn(("visual", "meshes"), tree_identities)
        self.assertIn(("native", "usr/lib"), tree_identities)
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

        runtime_files = {
            (entry["root"], entry["path"]): entry["sha256"]
            for entry in self.lock["runtime_files"]
        }
        pico = self.lock["pico"]
        self.assertEqual(
            runtime_files[("sonic", pico["runtime_overlay"])],
            pico["runtime_overlay_sha256"],
        )
        if self.lock["python"]["machine"] == "x86_64":
            self.assertFalse(
                any(
                    "/aarch64/" in path
                    for root, path in runtime_files
                    if root == "sonic"
                )
            )

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
            ("urdf2mjcf", "0.1.3"),
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
            "49a5f8f138793a78dd7339b77d8df887af14ffaa34acdb25b0c01e3cbf1265f7",
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
            "a38e57630e679f25c527f50db907050077e8d5d6",
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
        self.assertIn("worktrees/sonic-matrix-recovery-v77", trna)
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
        for source in ("x11-core-gated", "x11-absolute", "ue-final-pov"):
            self.assertIn(
                f'"${{MATRIX_GAME_CAMERA_YAW_SOURCE:-fixed}}" == "{source}"',
                run_sim,
            )
            self.assertIn(f'"$GAME_CAMERA_YAW_SOURCE" == "{source}"', text)
        self.assertIn(
            "Qualified game control rejects experimental camera yaw sources",
            run_sim,
        )
        self.assertIn(
            "Bounded game-control qualification rejects experimental camera yaw sources",
            text,
        )

        self.assertIn("forward_signal TERM 143", text)
        self.assertIn("FORWARDED_SIGNAL_EXIT_CODE", text)
        self.assertIn("Bounded qualification requires --profile", text)
        self.assertIn("MATRIX_SONIC_QUALIFIED_RUNTIME", text)

        check_env = (REPO_ROOT / "scripts/check_env.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("motion_controller_is_disabled", check_env)
        self.assertIn(
            "Matrix motion controller: disabled by selected runtime topology",
            check_env,
        )

    def test_physical_recovery_launch_contract_is_fail_closed(self) -> None:
        launcher = (REPO_ROOT / "scripts/run_matrix_sonic.sh").read_text(
            encoding="utf-8"
        )
        run_sim = (REPO_ROOT / "scripts/run_sim.sh").read_text(encoding="utf-8")
        heyuan = (REPO_ROOT / "config/hosts/heyuan.env").read_text(
            encoding="utf-8"
        )

        self.assertIn('GAME_FALL_RECOVERY="physical"', launcher)
        self.assertIn('GAME_FALL_RECOVERY="off"', launcher)
        self.assertIn("select_physical_recovery_python", launcher)
        self.assertIn("import numpy, onnxruntime", launcher)
        self.assertIn("ChannelPublisher, ChannelSubscriber", launcher)
        self.assertIn("LowCmd_, LowState_", launcher)
        self.assertIn("MATRIX_PHYSICAL_RECOVERY_PYTHON", launcher)
        self.assertIn("MATRIX_SONIC_FAIL_ON_FALL=0", launcher)
        for argument in (
            "--physical-recovery-worker",
            "--physical-recovery-python",
            "--physical-recovery-model",
            "--physical-recovery-fallback-model",
            "--physical-recovery-fallback-after-seconds",
            "--physical-recovery-stable-hold-seconds",
            "--physical-recovery-control-socket",
            "--physical-recovery-kungfu-model",
            "--physical-recovery-kungfu-motion",
            "--physical-recovery-kungfu-reference-frame",
            "--physical-recovery-kungfu-gain-scale",
        ):
            self.assertIn(argument, launcher)
            self.assertIn(argument, run_sim)
        for argument in (
            "--physical-recovery-kungfu-model-sha256",
            "--physical-recovery-kungfu-model-data-sha256",
            "--physical-recovery-kungfu-motion-sha256",
        ):
            self.assertIn(argument, run_sim)
        self.assertIn("import numpy, onnxruntime", run_sim)
        self.assertIn("ChannelPublisher, ChannelSubscriber", run_sim)
        self.assertIn("MATRIX_GAME_FALL_RECOVERY=physical", run_sim)
        self.assertIn("host|amp|kungfu", run_sim)
        self.assertIn(
            'MATRIX_PHYSICAL_RECOVERY_INITIAL_CONTROLLER:-kungfu',
            heyuan,
        )
        self.assertIn("MATRIX_PHYSICAL_RECOVERY_HANDOFF:-amp", heyuan)
        self.assertIn("1307-sonic-default-frame15689.npz", heyuan)
        self.assertIn(
            "164fb1be98102a6e0ca45ecf9aaf5fd1dedcd28e0cd53bc3bdbd80c9b94ee863",
            heyuan,
        )
        self.assertIn("MATRIX_KUNGFU_RECOVERY_REFERENCE_FRAME:-15689", heyuan)
        self.assertIn("MATRIX_PHYSICAL_RECOVERY_STABLE_HOLD_SECONDS:-0.2", heyuan)
        self.assertIn("host_prone_v1_0322.onnx", heyuan)
        self.assertIn(
            "62abf58c9a3d50dbe22ba1f950f288795fb3ae54bd3ca6221cc12cb1d45de155",
            heyuan,
        )
        self.assertIn("Fallback stays opt-in", heyuan)
        self.assertIn(
            "/home/kaijie/matrix-artifacts/g1-host-getup-v1", heyuan
        )
        self.assertIn("$MATRIX_PHYSICAL_RECOVERY_ARTIFACT_ROOT/venv/bin/python", heyuan)
        self.assertIn(
            "/home/kaijie/worktrees/sonic-matrix-recovery-gate-20260719",
            heyuan,
        )
        trna = (REPO_ROOT / "config/hosts/trna.env").read_text(encoding="utf-8")
        self.assertIn('MATRIX_GAME_AUTO_RESPAWN:-off', trna)
        self.assertIn('miniconda3/envs/sonic-h2-sim/bin/python', trna)
        self.assertIn('MATRIX_PHYSICAL_RECOVERY_INITIAL_CONTROLLER:-kungfu', trna)
        self.assertIn('MATRIX_PHYSICAL_RECOVERY_HANDOFF:-sonic', trna)
        self.assertIn('MATRIX_PHYSICAL_RECOVERY_RESIDENT_POLICIES:-1', trna)
        self.assertIn('MATRIX_PHYSICAL_RECOVERY_EXECUTION_PROVIDER:-cuda', trna)
        self.assertIn('1307-sonic-default-frame15689.npz', trna)
        self.assertIn('MATRIX_KUNGFU_RECOVERY_REFERENCE_FRAME:-15689', trna)
        self.assertIn('MATRIX_PHYSICAL_RECOVERY_STABLE_HOLD_SECONDS:-1.5', trna)
        self.assertIn('"$PROFILE" == "trna"', launcher)

    def test_trna_short_game_launch_defaults_are_profile_driven(self) -> None:
        launcher = (REPO_ROOT / "scripts/run_matrix_sonic.sh").read_text(
            encoding="utf-8"
        )
        profile = REPO_ROOT / "config/hosts/trna.env"
        profile_text = profile.read_text(encoding="utf-8")
        expected_profile_defaults = (
            'MATRIX_SONIC_CONTROL_SOURCE:-game',
            'MATRIX_GAME_INPUT_SOURCE:-auto',
            'MATRIX_GAME_CAMERA_YAW_SOURCE:-ue-final-pov',
            'MATRIX_GAME_LOOK_BUTTON:-left',
            'MATRIX_GAME_MOUSE_SENSITIVITY_DEG:-0.12',
            'MATRIX_GAME_CAMERA_YAW_SIGN:-1',
            'MATRIX_GAME_CAMERA_YAW_OFFSET_DEG:-0.0',
            "MATRIX_CENTERED_CAMERA_OVERLAY_BUNDLE-$HOME/"
            "matrix-artifacts/matrix-centered-camera-custom-v1",
            "MATRIX_PROFILE_VERIFY_RUNTIME_DEFAULT:-0",
        )
        for default in expected_profile_defaults:
            self.assertIn(default, profile_text)
        self.assertNotIn("export MATRIX_VERIFY_RUNTIME=", profile_text)
        self.assertIn("unset LD_LIBRARY_PATH PYTHONPATH", profile_text)
        self.assertIn(
            'if [[ -n "$PROFILE" && "${MATRIX_VERIFY_RUNTIME:-1}" != "0" ]]',
            launcher,
        )
        self.assertIn(
            'MATRIX_PROFILE_VERIFY_RUNTIME_DEFAULT:-1',
            launcher,
        )

        control_default = 'CONTROL_SOURCE="${MATRIX_SONIC_CONTROL_SOURCE:-planner}"'
        self.assertIn(control_default, launcher)
        self.assertIn('--control-source) CONTROL_SOURCE="$2"', launcher)
        self.assertLess(
            launcher.index('source "$PROFILE_FILE"'),
            launcher.index(control_default),
        )
        self.assertLess(
            launcher.index(control_default),
            launcher.index('while [[ $# -gt 0 ]]'),
        )

        names = (
            "MATRIX_SONIC_CONTROL_SOURCE",
            "MATRIX_GAME_INPUT_SOURCE",
            "MATRIX_GAME_CAMERA_YAW_SOURCE",
            "MATRIX_GAME_LOOK_BUTTON",
            "MATRIX_GAME_MOUSE_SENSITIVITY_DEG",
            "MATRIX_GAME_CAMERA_YAW_SIGN",
            "MATRIX_GAME_CAMERA_YAW_OFFSET_DEG",
            "MATRIX_CENTERED_CAMERA_OVERLAY_BUNDLE",
            "MATRIX_VERIFY_RUNTIME",
            "LD_LIBRARY_PATH",
            "PYTHONPATH",
        )
        emit = " ".join(f'"${{{name}-}}"' for name in names)
        command = (
            'set -euo pipefail; source "$1"; '
            f"printf '%s\\0' {emit}"
        )

        def load_profile(overrides: dict[str, str]) -> list[str]:
            environment = {
                "HOME": "/home/trna",
                "MATRIX_PROJECT_ROOT": "/matrix",
                "PATH": os.environ.get("PATH", "/usr/bin:/bin"),
                **overrides,
            }
            result = subprocess.run(
                ["bash", "-c", command, "bash", os.fspath(profile)],
                env=environment,
                capture_output=True,
                check=False,
            )
            self.assertEqual(result.returncode, 0, result.stderr.decode())
            return [
                field.decode("utf-8")
                for field in result.stdout.removesuffix(b"\0").split(b"\0")
            ]

        self.assertEqual(
            load_profile({}),
            [
                "game",
                "auto",
                "ue-final-pov",
                "left",
                "0.12",
                "1",
                "0.0",
                "/home/trna/matrix-artifacts/"
                "matrix-centered-camera-custom-v1",
                "",
                "",
                "",
            ],
        )
        overrides = {
            "MATRIX_SONIC_CONTROL_SOURCE": "planner",
            "MATRIX_GAME_INPUT_SOURCE": "keyboard",
            "MATRIX_GAME_CAMERA_YAW_SOURCE": "fixed",
            "MATRIX_GAME_LOOK_BUTTON": "right",
            "MATRIX_GAME_MOUSE_SENSITIVITY_DEG": "0.25",
            "MATRIX_GAME_CAMERA_YAW_SIGN": "-1",
            "MATRIX_GAME_CAMERA_YAW_OFFSET_DEG": "90.0",
            "MATRIX_CENTERED_CAMERA_OVERLAY_BUNDLE": "",
            "MATRIX_VERIFY_RUNTIME": "0",
            "LD_LIBRARY_PATH": "/tmp/host-libraries",
            "PYTHONPATH": "/tmp/host-python",
        }
        self.assertEqual(
            load_profile(overrides),
            [*list(overrides.values())[:-2], "", ""],
        )

    def test_env_check_skips_mc_only_for_external_control_topology(self) -> None:
        command = [
            "bash",
            str(REPO_ROOT / "scripts/check_env.sh"),
            "runtime",
            "--robot",
            "custom",
            "--scene",
            "2",
            "--mujoco",
            "0",
            "--offscreen",
            "1",
            "--skip-ldd",
        ]
        for overrides in (
            {"MATRIX_DISABLE_MC": "1", "MATRIX_SONIC": "0"},
            {"MATRIX_DISABLE_MC": "0", "MATRIX_SONIC": "TrUe"},
        ):
            with self.subTest(overrides=overrides):
                result = subprocess.run(
                    command,
                    env={**os.environ, **overrides},
                    text=True,
                    capture_output=True,
                    check=False,
                )
                output = result.stdout + result.stderr
                self.assertIn(
                    "Matrix motion controller: disabled by selected runtime topology",
                    output,
                )
                self.assertNotIn("src/robot_mc/run_mc.sh", output)

        result = subprocess.run(
            command,
            env={
                **os.environ,
                "MATRIX_DISABLE_MC": "0",
                "MATRIX_SONIC": "0",
            },
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertIn("src/robot_mc/run_mc.sh", result.stdout + result.stderr)

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
            for option in (
                "sonic-root",
                "inference-root",
                "visual-root",
                "native-deps",
                "wheelhouse",
            ):
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
        self.assertIn("update_matrix_local_env.py", text)
        self.assertIn("--no-index", text)
        self.assertIn("--only-binary=:all:", text)
        self.assertIn("--no-compile", text)
        self.assertIn("PYTHONDONTWRITEBYTECODE=1", text)
        self.assertIn("matrix-wheel-record-v3-no-compile", text)
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
            "native runtime Python isolation",
            "Python requirements lock",
            "Python wheelhouse inventory",
            "Python wheel RECORD metadata",
            "native runtime Python installed wheel files",
            "native runtime Python site-packages inventory",
            "native PICO wheel artifact",
            "native PICO Python isolation",
            "native PICO SDK wheel installation",
            "--pico-wheel",
            "gear_sonic import origin",
            "unitree_sdk2py Python package",
            "cyclonedds Python package",
            "archived SONIC critical source attestation",
            "Matrix tracked source clean",
            "Matrix ignored source overlays absent",
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

    def test_wheel_records_attest_installed_bytes_and_loadable_inventory(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            wheelhouse = root / "wheelhouse"
            site_packages = root / "venv/lib/python3.10/site-packages"
            runtime_python = root / "venv/bin/python"
            wheelhouse.mkdir()
            site_packages.mkdir(parents=True)
            metadata = b"Metadata-Version: 2.1\nName: demo-pkg\nVersion: 1.0\n"
            wheel, _, installed = write_test_wheel(
                wheelhouse,
                "demo_pkg-1.0-py3-none-any.whl",
                {
                    "demo_pkg/__init__.py": b"VALUE = 1\n",
                    "demo_pkg-1.0.dist-info/METADATA": metadata,
                },
            )
            digest = hashlib.sha256(wheel.read_bytes()).hexdigest()
            (wheelhouse / "SHA256SUMS").write_text(
                f"{digest}  {wheel.name}\n", encoding="utf-8"
            )
            for relative, content in installed.items():
                path = site_packages / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(content)
            (site_packages / "demo_pkg-1.0.dist-info/INSTALLER").write_text(
                "pip\n", encoding="utf-8"
            )
            external_pip = root / "venv/.matrix-pip-runner/pip/__init__.py"
            external_pip.parent.mkdir(parents=True)
            external_pip.write_text("# installer only\n", encoding="utf-8")

            checks = MODULE.verify_python_wheel_records(
                wheelhouse,
                site_packages,
                {"demo-pkg": ("demo-pkg", "1.0")},
                runtime_python,
            )
            self.assertTrue(all(ok for _, ok, _ in checks), checks)

            package = site_packages / "demo_pkg/__init__.py"
            package.write_text("VALUE = 2\n", encoding="utf-8")
            results = {
                name: ok
                for name, ok, _ in MODULE.verify_python_wheel_records(
                    wheelhouse,
                    site_packages,
                    {"demo-pkg": ("demo-pkg", "1.0")},
                    runtime_python,
                )
            }
            self.assertFalse(results["native runtime Python installed wheel files"])

            package.write_bytes(installed["demo_pkg/__init__.py"])
            (site_packages / "injected.py").write_text(
                "raise RuntimeError('loaded')\n", encoding="utf-8"
            )
            results = {
                name: ok
                for name, ok, _ in MODULE.verify_python_wheel_records(
                    wheelhouse,
                    site_packages,
                    {"demo-pkg": ("demo-pkg", "1.0")},
                    runtime_python,
                )
            }
            self.assertFalse(
                results["native runtime Python site-packages inventory"]
            )

            (site_packages / "injected.py").unlink()
            cache = site_packages / "demo_pkg/__pycache__/__init__.cpython-310.pyc"
            cache.parent.mkdir()
            cache.write_bytes(b"generated cache")
            checks = MODULE.verify_python_wheel_records(
                wheelhouse,
                site_packages,
                {"demo-pkg": ("demo-pkg", "1.0")},
                runtime_python,
            )
            results = {name: ok for name, ok, _ in checks}
            self.assertFalse(
                results["native runtime Python site-packages inventory"]
            )

    def test_target_wheel_records_map_scripts_data_and_owned_cache(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            wheelhouse = root / "wheelhouse"
            site_packages = root / "site-packages"
            runtime_python = root / "venv/bin/python"
            wheelhouse.mkdir()
            site_packages.mkdir()
            stem = "demo_pkg-1.0"
            owned_cache = "demo_pkg/__pycache__/locked.cpython-310.pyc"
            wheel_script = f"{stem}.data/scripts/native-helper"
            wheel_pythonw_script = f"{stem}.data/scripts/gui-helper"
            wheel_data = f"{stem}.data/data/share/man/man1/demo.1"
            wheel, _, archived = write_test_wheel(
                wheelhouse,
                "demo_pkg-1.0-py3-none-any.whl",
                {
                    "demo_pkg/__init__.py": b"VALUE = 1\n",
                    owned_cache: b"locked pyc bytes",
                    wheel_script: b"#!python\nfrom demo_pkg import main\nmain()",
                    wheel_pythonw_script: (
                        b"#!pythonw\r\nfrom demo_pkg import gui\ngui()"
                    ),
                    wheel_data: b"locked manual page",
                    f"{stem}.dist-info/entry_points.txt": (
                        b"[console_scripts]\n"
                        b"demo-tool = demo_pkg:main\n"
                    ),
                },
            )
            digest = hashlib.sha256(wheel.read_bytes()).hexdigest()
            (wheelhouse / "SHA256SUMS").write_text(
                f"{digest}  {wheel.name}\n", encoding="utf-8"
            )
            installed_contents: dict[str, bytes] = {}
            rewritten_scripts = {
                wheel_script: b"from demo_pkg import main\nmain()",
                wheel_pythonw_script: b"from demo_pkg import gui\ngui()",
            }
            for source_path, content in archived.items():
                installed_path = MODULE._wheel_record_site_path(
                    source_path, stem, target_install=True
                )
                if installed_path is None:
                    continue
                if source_path in rewritten_scripts:
                    content = (
                        f"#!{runtime_python}\n".encode("utf-8")
                        + rewritten_scripts[source_path]
                    )
                destination = site_packages / installed_path
                destination.parent.mkdir(parents=True, exist_ok=True)
                destination.write_bytes(content)
                installed_contents[installed_path] = content

            generated_entry_point = site_packages / "bin/demo-tool"
            generated_entry_point.parent.mkdir(parents=True, exist_ok=True)
            generated_wrapper = MODULE._entry_point_wrapper_bytes(
                "demo_pkg:main", os.fsencode(str(runtime_python))
            )
            generated_entry_point.write_bytes(generated_wrapper)
            checks = MODULE.verify_python_wheel_records(
                wheelhouse,
                site_packages,
                {"demo-pkg": ("demo-pkg", "1.0")},
                runtime_python,
            )
            self.assertTrue(all(ok for _, ok, _ in checks), checks)

            generated_cache = (
                site_packages / "demo_pkg/__pycache__/__init__.cpython-310.pyc"
            )
            generated_cache.write_bytes(b"unowned executable cache")
            results = {
                name: ok
                for name, ok, _ in MODULE.verify_python_wheel_records(
                    wheelhouse,
                    site_packages,
                    {"demo-pkg": ("demo-pkg", "1.0")},
                    runtime_python,
                )
            }
            self.assertFalse(
                results["native runtime Python site-packages inventory"]
            )
            generated_cache.unlink()

            generated_entry_point.write_bytes(b"#!/bin/sh\nmalicious\n")
            results = {
                name: ok
                for name, ok, _ in MODULE.verify_python_wheel_records(
                    wheelhouse,
                    site_packages,
                    {"demo-pkg": ("demo-pkg", "1.0")},
                    runtime_python,
                )
            }
            self.assertFalse(
                results["native runtime Python installed wheel files"]
            )
            generated_entry_point.write_bytes(generated_wrapper)

            generated_entry_point.unlink()
            results = {
                name: ok
                for name, ok, _ in MODULE.verify_python_wheel_records(
                    wheelhouse,
                    site_packages,
                    {"demo-pkg": ("demo-pkg", "1.0")},
                    runtime_python,
                )
            }
            self.assertFalse(
                results["native runtime Python installed wheel files"]
            )
            generated_entry_point.write_bytes(generated_wrapper)

            for installed_path, source_path in (
                (owned_cache, owned_cache),
                ("bin/native-helper", wheel_script),
                ("bin/gui-helper", wheel_pythonw_script),
                ("share/man/man1/demo.1", wheel_data),
            ):
                with self.subTest(installed_path=installed_path):
                    destination = site_packages / installed_path
                    destination.write_bytes(b"modified")
                    results = {
                        name: ok
                        for name, ok, _ in MODULE.verify_python_wheel_records(
                            wheelhouse,
                            site_packages,
                            {"demo-pkg": ("demo-pkg", "1.0")},
                            runtime_python,
                        )
                    }
                    self.assertFalse(
                        results["native runtime Python installed wheel files"]
                    )
                    destination.write_bytes(installed_contents[installed_path])

            owned_cache_path = site_packages / owned_cache
            owned_cache_path.unlink()
            results = {
                name: ok
                for name, ok, _ in MODULE.verify_python_wheel_records(
                    wheelhouse,
                    site_packages,
                    {"demo-pkg": ("demo-pkg", "1.0")},
                    runtime_python,
                )
            }
            self.assertFalse(
                results["native runtime Python installed wheel files"]
            )
            owned_cache_path.write_bytes(installed_contents[owned_cache])

            for source_path in (
                f"{stem}.data/headers/demo.h",
                f"{stem}.data/data/lib/python3.10/site-packages/demo.py",
            ):
                with self.subTest(source_path=source_path), self.assertRaises(
                    ValueError
                ):
                    MODULE._wheel_record_site_path(
                        source_path, stem, target_install=True
                    )

            undeclared = site_packages / "bin/evil"
            undeclared.write_bytes(b"not declared by locked metadata")
            results = {
                name: ok
                for name, ok, _ in MODULE.verify_python_wheel_records(
                    wheelhouse,
                    site_packages,
                    {"demo-pkg": ("demo-pkg", "1.0")},
                    runtime_python,
                )
            }
            self.assertFalse(
                results["native runtime Python site-packages inventory"]
            )

            undeclared.unlink()
            cache_payload = site_packages / "demo_pkg/__pycache__/evil.so"
            cache_payload.write_bytes(b"loadable cache payload")
            results = {
                name: ok
                for name, ok, _ in MODULE.verify_python_wheel_records(
                    wheelhouse,
                    site_packages,
                    {"demo-pkg": ("demo-pkg", "1.0")},
                    runtime_python,
                )
            }
            self.assertFalse(
                results["native runtime Python site-packages inventory"]
            )

    def test_entry_point_script_allowlist_rejects_unsafe_names(self) -> None:
        self.assertEqual(
            MODULE._wheel_entry_point_script_paths(
                b"[console_scripts]\ncli-tool = package.cli:main\n"
                b"[gui_scripts]\nviewer = package.gui:main\n"
                b"[unrelated]\nplugin = package.plugin:value\n"
            ),
            {
                "bin/cli-tool": "package.cli:main",
                "bin/viewer": "package.gui:main",
            },
        )
        self.assertEqual(
            MODULE._entry_point_wrapper_bytes(
                "package.cli:main", b"/locked/bin/python"
            ),
            (
                b"#!/locked/bin/python\n"
                b"# -*- coding: utf-8 -*-\n"
                b"import re\n"
                b"import sys\n"
                b"from package.cli import main\n"
                b"if __name__ == '__main__':\n"
                b"    sys.argv[0] = re.sub(r'(-script\\.pyw|\\.exe)?$', '', "
                b"sys.argv[0])\n"
                b"    sys.exit(main())\n"
            ),
        )
        for content in (
            b"[console_scripts]\nevil.py = package:main\n",
            b"[console_scripts]\n../evil = package:main\n",
            (
                b"[console_scripts]\nsame = package:main\n"
                b"[gui_scripts]\nsame = package:main\n"
            ),
            b"[DEFAULT]\nimplicit = package:main\n[console_scripts]\nok = p:m\n",
        ):
            with self.subTest(content=content), self.assertRaises(ValueError):
                MODULE._wheel_entry_point_script_paths(content)

    def test_entry_point_paths_reject_cross_wheel_and_record_collisions(self) -> None:
        for collision in ("cross-wheel", "record"):
            with self.subTest(collision=collision), tempfile.TemporaryDirectory(
            ) as temporary:
                root = Path(temporary)
                wheelhouse = root / "wheelhouse"
                site_packages = root / "site-packages"
                wheelhouse.mkdir()
                site_packages.mkdir()
                wheel_one, _, _ = write_test_wheel(
                    wheelhouse,
                    "first-1.0-py3-none-any.whl",
                    {
                        "first-1.0.dist-info/entry_points.txt": (
                            b"[console_scripts]\nshared = first:main\n"
                        ),
                        **(
                            {
                                "first-1.0.data/scripts/shared": (
                                    b"#!python\nfrom first import main\nmain()"
                                )
                            }
                            if collision == "record"
                            else {}
                        ),
                    },
                )
                wheels = [wheel_one]
                pins = {"first": ("first", "1.0")}
                if collision == "cross-wheel":
                    wheel_two, _, _ = write_test_wheel(
                        wheelhouse,
                        "second-1.0-py3-none-any.whl",
                        {
                            "second-1.0.dist-info/entry_points.txt": (
                                b"[gui_scripts]\nshared = second:main\n"
                            )
                        },
                    )
                    wheels.append(wheel_two)
                    pins["second"] = ("second", "1.0")
                (wheelhouse / "SHA256SUMS").write_text(
                    "".join(
                        f"{hashlib.sha256(wheel.read_bytes()).hexdigest()}  "
                        f"{wheel.name}\n"
                        for wheel in wheels
                    ),
                    encoding="utf-8",
                )
                results = {
                    name: ok
                    for name, ok, _ in MODULE.verify_python_wheel_records(
                        wheelhouse,
                        site_packages,
                        pins,
                        root / "venv/bin/python",
                    )
                }
                self.assertFalse(results["Python wheel RECORD metadata"])

    def test_python_identity_probe_is_executable_json(self) -> None:
        marker = "MATRIX_TEST_IDENTITY_JSON="
        result = subprocess.run(
            [sys.executable, "-c", MODULE.python_identity_probe_code(marker)],
            env={**os.environ, "PYTHONNOUSERSITE": "1"},
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        line = next(
            value for value in result.stdout.splitlines() if value.startswith(marker)
        )
        identity = json.loads(line[len(marker) :])
        self.assertEqual(
            set(identity),
            {
                "version",
                "soabi",
                "machine",
                "prefix",
                "base_prefix",
                "executable",
                "path",
                "purelib",
                "platlib",
                "stdlib",
                "platstdlib",
                "user_site_enabled",
            },
        )
        self.assertIs(identity["user_site_enabled"], False)
        with self.assertRaises(ValueError):
            MODULE.python_identity_probe_code("unsafe marker=")

    def test_wheel_records_reject_unhashed_or_unlocked_distributions(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            wheelhouse = root / "wheelhouse"
            site_packages = root / "site-packages"
            runtime_python = root / "venv/bin/python"
            wheelhouse.mkdir()
            site_packages.mkdir()
            wheel, _, _ = write_test_wheel(
                wheelhouse,
                "demo-1.0-py3-none-any.whl",
                {"demo.py": b"VALUE = 1\n"},
            )
            digest = hashlib.sha256(wheel.read_bytes()).hexdigest()
            (wheelhouse / "SHA256SUMS").write_text(
                f"{digest}  {wheel.name}\n", encoding="utf-8"
            )

            checks = MODULE.verify_python_wheel_records(
                wheelhouse,
                site_packages,
                {"missing": ("missing", "1.0")},
                runtime_python,
            )
            results = {name: ok for name, ok, _ in checks}
            self.assertFalse(results["Python wheel RECORD metadata"])

            with zipfile.ZipFile(wheel, "r") as archive:
                files = {
                    info.filename: archive.read(info.filename)
                    for info in archive.infolist()
                }
            record_path = "demo-1.0.dist-info/RECORD"
            rows = list(
                csv.reader(io.StringIO(files[record_path].decode("utf-8")))
            )
            for row in rows:
                if row[0] == "demo.py":
                    row[1:] = ["", ""]
            output = io.StringIO(newline="")
            csv.writer(output, lineterminator="\n").writerows(rows)
            files[record_path] = output.getvalue().encode("utf-8")
            with zipfile.ZipFile(wheel, "w") as archive:
                for relative, content in files.items():
                    archive.writestr(relative, content)
            digest = hashlib.sha256(wheel.read_bytes()).hexdigest()
            (wheelhouse / "SHA256SUMS").write_text(
                f"{digest}  {wheel.name}\n", encoding="utf-8"
            )
            checks = MODULE.verify_python_wheel_records(
                wheelhouse,
                site_packages,
                {"demo": ("demo", "1.0")},
                runtime_python,
            )
            results = {name: ok for name, ok, _ in checks}
            self.assertFalse(results["Python wheel RECORD metadata"])

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

    def test_pico_installed_sdk_bytes_are_bound_to_locked_wheel_record(self) -> None:
        pico_lock = copy.deepcopy(self.lock["pico"])
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            wheelhouse = root / "wheelhouse"
            site_packages = root / "venv/lib/python3.10/site-packages"
            wheelhouse.mkdir()
            site_packages.mkdir(parents=True)
            wheel_distribution, _, _, _, _ = MODULE.parse_wheel_filename(
                pico_lock["wheel_filename"]
            )
            stem = f"{wheel_distribution}-{pico_lock['version']}"
            extension = "xrobotoolkit_sdk.cpython-310-x86_64-linux-gnu.so"
            wheel, _, installed = write_test_wheel(
                wheelhouse,
                pico_lock["wheel_filename"],
                {
                    extension: b"locked extension bytes",
                    f"{stem}.dist-info/top_level.txt": b"xrobotoolkit_sdk\n",
                    f"{stem}.dist-info/METADATA": (
                        b"Metadata-Version: 2.1\nName: xrobotoolkit-sdk\n"
                        b"Version: 1.0.2\n"
                    ),
                },
            )
            pico_lock["wheel_sha256"] = hashlib.sha256(wheel.read_bytes()).hexdigest()
            for relative, content in installed.items():
                path = site_packages / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(content)
            (site_packages / "pip").mkdir()
            (site_packages / "pip/__init__.py").write_text(
                "# unrelated controlled dependency\n", encoding="utf-8"
            )

            ok, detail = MODULE.verify_installed_pico_wheel(
                wheel, site_packages, pico_lock
            )
            self.assertTrue(ok, detail)

            generated_cache = (
                site_packages
                / "xrobotoolkit_sdk/__pycache__/generated.cpython-310.pyc"
            )
            generated_cache.parent.mkdir(parents=True)
            generated_cache.write_bytes(b"runtime-generated cache")
            ok, detail = MODULE.verify_installed_pico_wheel(
                wheel, site_packages, pico_lock
            )
            self.assertFalse(ok)
            self.assertIn("unowned PICO import file", detail)
            generated_cache.unlink()

            installed_extension = site_packages / extension
            installed_extension.write_bytes(b"modified extension")
            ok, detail = MODULE.verify_installed_pico_wheel(
                wheel, site_packages, pico_lock
            )
            self.assertFalse(ok)
            self.assertIn(extension, detail)

            installed_extension.write_bytes(installed[extension])
            injected = site_packages / "xrobotoolkit_sdk.py"
            injected.write_text("raise RuntimeError\n", encoding="utf-8")
            ok, detail = MODULE.verify_installed_pico_wheel(
                wheel, site_packages, pico_lock
            )
            self.assertFalse(ok)
            self.assertIn("unowned PICO import file", detail)

            injected.unlink()
            startup_hook = site_packages / "evil.pth"
            startup_hook.write_text("/tmp/evil\n", encoding="utf-8")
            ok, detail = MODULE.verify_installed_pico_wheel(
                wheel, site_packages, pico_lock
            )
            self.assertFalse(ok)
            self.assertIn("evil.pth", detail)

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

    def test_matrix_source_overlay_attestation_rejects_importable_code(self) -> None:
        lock = {
            "matrix_release": {
                "installed_files": [
                    {
                        "path": "src/UeSim/Linux/Engine/Binaries/Linux/locked.so"
                    }
                ],
                "installed_trees": [
                    {
                        "path": "src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux"
                    }
                ],
            }
        }
        clean_inventory = subprocess.CompletedProcess(
            [],
            0,
            stdout=(
                b"scripts/__pycache__/safe.cpython-310.pyc\0"
                b".venv-audit/lib/python3.10/site-packages/pip/__init__.py\0"
                b"src/robot_mc/build/generated.so\0"
                b"src/UeSim/Linux/Engine/Binaries/Linux/locked.so\0"
            ),
            stderr=b"",
        )
        with mock.patch.object(
            MODULE.subprocess,
            "run",
            side_effect=[clean_inventory, clean_inventory],
        ):
            ok, detail = MODULE.matrix_source_overlay_attestation(
                REPO_ROOT, lock
            )
        self.assertTrue(ok, detail)

        empty = subprocess.CompletedProcess([], 0, stdout=b"", stderr=b"")
        injected = subprocess.CompletedProcess(
            [],
            0,
            stdout=(
                b"scripts/json.py\0"
                b"scripts/payload.pyc\0"
                b"scripts/native_override.so\0"
            ),
            stderr=b"",
        )
        with mock.patch.object(
            MODULE.subprocess,
            "run",
            side_effect=[empty, injected],
        ):
            ok, detail = MODULE.matrix_source_overlay_attestation(
                REPO_ROOT, lock
            )
        self.assertFalse(ok)
        self.assertIn("scripts/json.py", detail)
        self.assertIn("scripts/payload.pyc", detail)
        self.assertIn("scripts/native_override.so", detail)

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
        self.assertIn("matrix-wheel-record-v3-no-compile", text)
        self.assertIn("ensurepip did not create a usable pip package", text)
        self.assertIn('find "$audit_site_packages"', text)
        self.assertIn('--target "$audit_site_packages"', text)
        self.assertIn("--ignore-installed", text)
        self.assertIn("Recreating non-isolated .venv-audit", text)
        self.assertIn("Recreating incomplete .venv-audit", text)

    def test_external_pip_marker_is_scoped_to_pip_check(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            python = root / "venv/bin/python"
            python.parent.mkdir(parents=True)
            python.touch()
            (root / "venv/lib/python3.10/site-packages").mkdir(parents=True)
            pip_root = root / "venv/.matrix-pip-runner"
            (pip_root / "pip").mkdir(parents=True)
            (pip_root / "pip/__init__.py").write_text("", encoding="utf-8")
            (root / "venv/.matrix-external-pip").write_text(
                f"{pip_root}\n", encoding="utf-8"
            )

            base = {"PYTHONNOUSERSITE": "1"}
            pip_env, error = MODULE.pip_check_environment(str(python), base)
            self.assertIsNone(error)
            self.assertEqual(
                pip_env["PYTHONPATH"],
                os.pathsep.join(
                    (
                        str(root / "venv/lib/python3.10/site-packages"),
                        str(pip_root),
                    )
                ),
            )
            self.assertNotIn("PYTHONPATH", base)

            (root / "venv/.matrix-external-pip").write_text("", encoding="utf-8")
            _, error = MODULE.pip_check_environment(str(python), base)
            self.assertIn("must contain one path", error or "")

    def test_runtime_python_isolation_rejects_system_site_packages(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            matrix_root = root / "matrix"
            venv = matrix_root / ".venv-audit"
            site_packages = venv / "lib/python3.10/site-packages"
            site_packages.mkdir(parents=True)
            matrix_root.mkdir(exist_ok=True)
            configuration = venv / "pyvenv.cfg"
            configuration.write_text(
                "home = /usr/bin\ninclude-system-site-packages = false\n",
                encoding="utf-8",
            )
            identity = {
                "version": "3.10",
                "prefix": str(venv),
                "base_prefix": "/usr",
                "executable": str(venv / "bin/python"),
                "purelib": str(site_packages),
                "platlib": str(site_packages),
                "stdlib": "/usr/lib/python3.10",
                "platstdlib": "/usr/lib/python3.10",
                "user_site_enabled": False,
                "path": [
                    "",
                    "/usr/lib/python310.zip",
                    "/usr/lib/python3.10",
                    "/usr/lib/python3.10/lib-dynload",
                    str(site_packages),
                    str(site_packages / "cmeel.prefix/lib/python3.10/site-packages"),
                ],
            }

            ok, detail = MODULE.verify_python_isolation(
                venv, site_packages, matrix_root, identity
            )
            self.assertTrue(ok, detail)

            decoy_identity = copy.deepcopy(identity)
            decoy_identity["prefix"] = "/actual/venv"
            decoy_identity["purelib"] = "/actual/venv/lib/python3.10/site-packages"
            decoy_identity["platlib"] = "/actual/venv/lib/python3.10/site-packages"
            decoy_identity["path"] = [
                "",
                "/usr/lib/python3.10",
                "/actual/venv/lib/python3.10/site-packages",
            ]
            ok, detail = MODULE.verify_python_isolation(
                venv, site_packages, matrix_root, decoy_identity
            )
            self.assertFalse(ok)
            self.assertIn("runtime prefix escapes venv", detail)

            configuration.write_text(
                "include-system-site-packages = true\n", encoding="utf-8"
            )
            identity["path"].append("/usr/lib/python3/dist-packages")
            ok, detail = MODULE.verify_python_isolation(
                venv, site_packages, matrix_root, identity
            )
            self.assertFalse(ok)
            self.assertIn("must occur once as false", detail)
            self.assertIn("/usr/lib/python3/dist-packages", detail)

    def test_local_env_update_preserves_unrelated_overrides(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            local_env = Path(temporary) / ".matrix/local.env"
            local_env.parent.mkdir(parents=True)
            local_env.write_text(
                "# host overrides\n"
                "export MATRIX_RUNTIME_ROOT=/old/runtime\n"
                "export MATRIX_PICO_PYTHON=/opt/pico/bin/python\n"
                "MATRIX_PICO_WHEEL='/opt/pico/pico wheel.whl'\n",
                encoding="utf-8",
            )
            os.chmod(local_env, 0o640)

            LOCAL_ENV_MODULE.update_export(
                local_env, "MATRIX_RUNTIME_ROOT", "/new/runtime root"
            )

            updated = local_env.read_text(encoding="utf-8")
            self.assertIn("export MATRIX_RUNTIME_ROOT='/new/runtime root'", updated)
            self.assertIn("export MATRIX_PICO_PYTHON=/opt/pico/bin/python", updated)
            self.assertIn("MATRIX_PICO_WHEEL='/opt/pico/pico wheel.whl'", updated)
            self.assertEqual(updated.count("MATRIX_RUNTIME_ROOT="), 1)
            self.assertEqual(local_env.stat().st_mode & 0o777, 0o640)

            parsed = LOCAL_ENV_MODULE.parse_local_env(local_env)
            self.assertEqual(parsed["MATRIX_RUNTIME_ROOT"], "/new/runtime root")
            self.assertEqual(parsed["MATRIX_PICO_PYTHON"], "/opt/pico/bin/python")
            self.assertEqual(parsed["MATRIX_PICO_WHEEL"], "/opt/pico/pico wheel.whl")

    def test_local_env_parser_never_evaluates_shell_content(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            local_env = root / "local.env"
            marker = root / "executed"
            local_env.write_text(
                f"MATRIX_RUNTIME_ROOT='$(touch {marker})'\n",
                encoding="utf-8",
            )
            parsed = LOCAL_ENV_MODULE.parse_local_env(local_env)
            self.assertEqual(parsed["MATRIX_RUNTIME_ROOT"], f"$(touch {marker})")
            self.assertFalse(marker.exists())

            for payload, message in (
                ("PATH=/tmp/evil\n", "not allowlisted"),
                ("MATRIX_RUNTIME_ROOT=/tmp; touch /tmp/evil\n", "one shell-quoted word"),
                ("trap evil EXIT\n", "invalid local env syntax"),
                (
                    "MATRIX_RUNTIME_ROOT=/one\nMATRIX_RUNTIME_ROOT=/two\n",
                    "duplicate",
                ),
            ):
                with self.subTest(payload=payload):
                    local_env.write_text(payload, encoding="utf-8")
                    with self.assertRaisesRegex(ValueError, message):
                        LOCAL_ENV_MODULE.parse_local_env(local_env)

    def test_shell_local_env_loader_exports_only_parsed_values(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            scripts = root / "scripts"
            scripts.mkdir()
            shutil.copy2(LOCAL_ENV_SCRIPT, scripts / LOCAL_ENV_SCRIPT.name)
            local_env = root / ".matrix/local.env"
            local_env.parent.mkdir()
            local_env.write_text(
                "MATRIX_RUNTIME_ROOT='/runtime with space'\n",
                encoding="utf-8",
            )
            loader = REPO_ROOT / "scripts/matrix_local_env.sh"
            command = (
                f"source {shlex.quote(str(loader))}; "
                f"load_matrix_local_env {shlex.quote(str(root))}; "
                "printf '%s' \"$MATRIX_RUNTIME_ROOT\""
            )
            result = subprocess.run(
                ["bash", "-c", command],
                text=True,
                capture_output=True,
                check=False,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(result.stdout, "/runtime with space")

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
            self.assertNotIn('source "$PROJECT_ROOT/.matrix/local.env"', text)
            local_env_source = text.index('load_matrix_local_env "$PROJECT_ROOT"')
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
        primary_bytecode_guard = primary.index(
            "export PYTHONDONTWRITEBYTECODE=1"
        )
        primary_qualification_gate = primary.index(
            'if [[ "$QUALIFICATION_REQUESTED" == "1" ]]'
        )
        self.assertLess(primary_lock, primary_status_cleanup)
        self.assertLess(primary_status_cleanup, primary_verifier)
        self.assertLess(primary_bytecode_guard, primary_qualification_gate)
        for launcher in (primary, overworld):
            self.assertIn("--require-git-sonic", launcher)
            self.assertIn("VERIFY_RUNTIME_ARGS+=(--fast)", launcher)
            self.assertIn("Bounded qualification requires locked", launcher)
            self.assertIn(
                "Bounded qualification rejects inherited LD_LIBRARY_PATH/PYTHONPATH",
                launcher,
            )
            self.assertIn("PYTHONDONTWRITEBYTECODE=1", launcher)
            self.assertIn(
                "mktemp -d /tmp/matrix-qualified-pycache.XXXXXX", launcher
            )
        self.assertIn("MATRIX_CPUSET_APPLIED", overworld)

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

    def test_bounded_launch_rejects_alternate_roots_and_skip_overrides(self) -> None:
        launcher = REPO_ROOT / "scripts/run_matrix_sonic.sh"
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            cases = (
                ({"SIM_LAUNCHER_ROOT": str(root / "other")}, "alternate Matrix"),
                ({"MATRIX_ROOT": str(root / "other")}, "alternate Matrix"),
                ({"SIM_LAUNCHER_SKIP_CUSTOM_URDF_WRAPPER": "1"}, "skip overrides"),
                ({"MATRIX_SKIP_ENV_CHECK": "1"}, "skip overrides"),
            )
            for overrides, expected in cases:
                with self.subTest(overrides=overrides):
                    environment = os.environ.copy()
                    environment.update(overrides)
                    environment["MATRIX_SONIC_HOST_LOCK"] = str(
                        root / f"{next(iter(overrides))}.lock"
                    )
                    environment.pop("MATRIX_PROFILE", None)
                    environment.pop("MATRIX_CPUSET", None)
                    result = subprocess.run(
                        [
                            "bash",
                            str(launcher),
                            "--profile",
                            "trna",
                            "--control-source",
                            "planner",
                            "--max-seconds",
                            "1",
                        ],
                        cwd=REPO_ROOT,
                        env=environment,
                        text=True,
                        capture_output=True,
                        check=False,
                    )
                    self.assertEqual(result.returncode, 2, result.stderr)
                    self.assertIn(expected, result.stderr)


if __name__ == "__main__":
    unittest.main()
