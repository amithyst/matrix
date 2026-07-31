from __future__ import annotations

from contextlib import redirect_stderr, redirect_stdout
import copy
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "verify_matrix_bfm_isaac_runtime.py"
SPEC = importlib.util.spec_from_file_location(
    "verify_matrix_bfm_isaac_runtime", SCRIPT_PATH
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


MATRIX_MUJOCO_JOINT_ORDER = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)

ISAACLAB_JOINT_ORDER = (
    "left_hip_pitch_joint",
    "right_hip_pitch_joint",
    "waist_yaw_joint",
    "left_hip_roll_joint",
    "right_hip_roll_joint",
    "waist_roll_joint",
    "left_hip_yaw_joint",
    "right_hip_yaw_joint",
    "waist_pitch_joint",
    "left_knee_joint",
    "right_knee_joint",
    "left_shoulder_pitch_joint",
    "right_shoulder_pitch_joint",
    "left_ankle_pitch_joint",
    "right_ankle_pitch_joint",
    "left_shoulder_roll_joint",
    "right_shoulder_roll_joint",
    "left_ankle_roll_joint",
    "right_ankle_roll_joint",
    "left_shoulder_yaw_joint",
    "right_shoulder_yaw_joint",
    "left_elbow_joint",
    "right_elbow_joint",
    "left_wrist_roll_joint",
    "right_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "right_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_wrist_yaw_joint",
)

ISAACLAB_TO_MATRIX_SOURCE_INDICES = (
    0,
    3,
    6,
    9,
    13,
    17,
    1,
    4,
    7,
    10,
    14,
    18,
    2,
    5,
    8,
    11,
    15,
    19,
    21,
    23,
    25,
    27,
    12,
    16,
    20,
    22,
    24,
    26,
    28,
)


class MatrixBfmIsaacRuntimeVerifierTest(unittest.TestCase):
    def setUp(self) -> None:
        self.lock_path = REPO_ROOT / "config/runtime/matrix-bfm-isaac.lock.json"
        self.lock = MODULE.load_lock(self.lock_path)

    def test_repository_matrix_port_files_match_lock(self) -> None:
        for entry in self.lock["matrix_port"]["critical_files"]:
            path = REPO_ROOT / entry["path"]
            self.assertTrue(path.is_file(), entry["path"])
            self.assertFalse(path.is_symlink(), entry["path"])
            self.assertEqual(
                MODULE.sha256_file(path),
                entry["sha256"],
                entry["path"],
            )

    def runtime_report(self, *, wall_seconds: float = 2.0) -> dict[str, object]:
        control_steps = 100
        return {
            "ok": True,
            "failure": None,
            "physics_dt": 0.005,
            "control_hz": 50.0,
            "physics_device": "cpu",
            "reference_device": "cuda:0",
            "physics_command_write_mode": "implicit_once_per_control_interval",
            "physics_command_writes_per_articulation_per_control_step": 1,
            "articulation_updates_per_articulation_per_control_step": 1,
            "articulation_update_dt_s": 0.02,
            "teacher_onnx_session": copy.deepcopy(
                MODULE.EXPECTED_EXECUTION_CONTRACT["teacher_onnx_session"]
            ),
            "requested_control_steps": control_steps,
            "completed_control_steps": control_steps,
            "fall_count": 0,
            "recovery_count": 0,
            "awaiting_recovery_final": False,
            "matrix_state_frames_sent": control_steps,
            "matrix_state_frames_dropped": 0,
            "mode": "schedule",
            "schedule": [
                ["stand", 0.25],
                ["walk", 0.25],
                ["jog", 0.25],
                ["turn_left", 0.25],
                ["turn_right", 0.25],
                ["rotate_left", 0.25],
                ["rotate_right", 0.25],
            ],
            "observed_gaits": ["stand", "walk", "jog"],
            "height_raycast_hits_min": 121,
            "height_query_paths_last": ["/World/Collision/terrain"],
            "root_clearance_min": 0.70,
            "root_clearance_max": 0.80,
            "reference_source": "robo_pfnn_formal7168",
            "reference_source_hz": 60.0,
            "reference_output_hz": 50.0,
            "reference_buffer_swap_count": 7,
            "reference_pending_elapsed_steps_max": 4,
            "reference_root_xy_error_p95_m": 0.03,
            "reference_root_yaw_error_p95_rad": 0.10,
            "reference_root_tilt_error_p95_rad": 0.06,
            "reference_joint_tracking_rmse_rad": 0.07,
            "control_loop_wall_s": wall_seconds,
            "control_step_wall_ms_p95": 19.0 if wall_seconds == 2.0 else 80.0,
            "simulation_realtime_factor": (control_steps / 50.0) / wall_seconds,
        }

    def relay_status(self) -> dict[str, object]:
        control_steps = 100
        return {
            "schema": MODULE.RELAY_STATUS_SCHEMA,
            "ok": True,
            "input_contract": self.lock["wire_contract"]["input"],
            "output_contract": self.lock["wire_contract"]["matrix_output"],
            "stats": {
                "received": control_steps,
                "invalid": 0,
                "sequence_gaps": 0,
                "duplicates": 0,
                "out_of_order": 0,
                "non_grid_time": 0,
                "first_sequence": 0,
                "last_sequence": control_steps - 1,
            },
            "boundary_guard": {
                "armed": True,
                "warning_events": 0,
                "stop_events": 0,
                "stop_pulses": 0,
                "command_errors": 0,
                "hard_violations": 0,
                "first_stop_root_xy": None,
                "minimum_edge_distance_m": 100.0,
            },
        }

    def resolved_video_settings(self) -> dict[str, object]:
        return {
            "schema": MODULE.VIDEO_SETTINGS_SCHEMA,
            "resolution": "1280x720",
            "resolution_width": 1280,
            "resolution_height": 720,
            "window_mode": "borderless",
            "fps_limit": 30,
            "quality": "low",
            "camera_smoothing": "medium",
            "screen_percentage": 100,
        }

    def acceptance_runtime_args(self, root: Path) -> list[str]:
        return [
            "--matrix-root",
            str(root / "matrix"),
            "--runtime-root",
            str(root / "runtime"),
            "--runtime-python",
            str(root / "isaac/bin/python"),
            "--physics-asset-root",
            str(root / "physics"),
            "--collision-root",
            str(root / "collision"),
            "--teacher-profile",
            str(root / "teacher-profile.toml"),
            "--visual-venv",
            str(root / "visual-venv"),
            "--matrix-visual-root",
            str(root / "matrix-visual"),
            "--material-bridge",
            str(
                root
                / "matrix"
                / self.lock["ue_material_bridge"]["relative_path"]
            ),
        ]

    def acceptance_verifier_patches(self) -> mock._patch:
        runtime_check = [MODULE.Check("fixture_runtime", True, "verified")]
        return mock.patch.multiple(
            MODULE,
            verify_matrix_port=mock.Mock(
                return_value=(runtime_check, "a" * 40)
            ),
            verify_runtime_checkout=mock.Mock(return_value=runtime_check),
            verify_isaac_runtime=mock.Mock(return_value=runtime_check),
            verify_physics_assets=mock.Mock(return_value=runtime_check),
            verify_scene_assets=mock.Mock(return_value=runtime_check),
            verify_teacher_profile=mock.Mock(return_value=runtime_check),
            verify_visual_venv=mock.Mock(return_value=runtime_check),
            verify_matrix_visual=mock.Mock(return_value=runtime_check),
            verify_ue_material_bridge=mock.Mock(
                return_value=(
                    runtime_check,
                    {
                        "relative_path": self.lock["ue_material_bridge"][
                            "relative_path"
                        ],
                        "sha256": self.lock["ue_material_bridge"]["sha256"],
                        "expected_sha256": self.lock["ue_material_bridge"][
                            "sha256"
                        ],
                        "ue_binary_relative_path": self.lock[
                            "ue_material_bridge"
                        ]["ue_binary_relative_path"],
                        "ue_binary_build_id": self.lock["ue_material_bridge"][
                            "ue_binary_build_id"
                        ],
                        "expected_ue_binary_build_id": self.lock[
                            "ue_material_bridge"
                        ]["ue_binary_build_id"],
                    },
                )
            ),
        )

    def test_lock_pins_matrix_order_and_isaaclab_source_indices(self) -> None:
        locked_order = tuple(self.lock["wire_contract"]["joint_order"])
        self.assertEqual(locked_order, MATRIX_MUJOCO_JOINT_ORDER)
        self.assertEqual(
            tuple(ISAACLAB_JOINT_ORDER.index(name) for name in locked_order),
            ISAACLAB_TO_MATRIX_SOURCE_INDICES,
        )
        self.assertEqual(
            tuple(self.lock["wire_contract"]["isaaclab_joint_order"]),
            ISAACLAB_JOINT_ORDER,
        )
        self.assertEqual(
            tuple(
                self.lock["wire_contract"][
                    "isaaclab_to_matrix_source_indices"
                ]
            ),
            ISAACLAB_TO_MATRIX_SOURCE_INDICES,
        )

    def test_launcher_binds_verified_assets_to_the_runtime_config(self) -> None:
        launcher = (REPO_ROOT / "scripts/run_matrix_bfm_isaac.sh").read_text(
            encoding="utf-8"
        )

        self.assertIn('"g1_usd": (physics_root /', launcher)
        self.assertIn('"scene_root": collision_root', launcher)
        self.assertIn('"collision_usd": (', launcher)
        self.assertIn('"bfm_sonic_repo": bfm_source_root', launcher)
        self.assertIn(
            "Refusing to verify one source/asset closure and run another",
            launcher,
        )

    def test_co_resident_renderer_uses_locked_isolated_video_contract(self) -> None:
        launcher = (REPO_ROOT / "scripts/run_matrix_bfm_isaac.sh").read_text(
            encoding="utf-8"
        )
        run_sim = (REPO_ROOT / "scripts/run_sim.sh").read_text(
            encoding="utf-8"
        )
        settings = json.loads(
            (
                REPO_ROOT
                / "config/runtime/matrix-bfm-isaac-video-settings.json"
            ).read_text(encoding="utf-8")
        )

        self.assertEqual(
            settings,
            {
                "schema": "matrix_bfm_isaac_video_settings.v1",
                "resolution": "1280x720",
                "window_mode": "borderless",
                "fps_limit": 30,
                "quality": "low",
                "camera_smoothing": "medium",
                "screen_percentage": 100,
            },
        )
        for variable in (
            "MATRIX_VIDEO_APPLIED_WIDTH",
            "MATRIX_VIDEO_APPLIED_HEIGHT",
            "MATRIX_VIDEO_APPLIED_WINDOW_MODE",
            "MATRIX_VIDEO_APPLIED_FPS_LIMIT",
            "MATRIX_VIDEO_APPLIED_QUALITY",
            "MATRIX_VIDEO_APPLIED_CAMERA_SMOOTHING",
            "MATRIX_VIDEO_APPLIED_CAMERA_DISTANCE_CM",
            "MATRIX_VIDEO_APPLIED_REVISION",
            "MATRIX_VIDEO_APPLIED_JSON",
        ):
            self.assertIn(f"-u {variable}", launcher)
            self.assertIn(variable, run_sim)
        self.assertIn("MATRIX_BFM_ISAAC_RENDERER_VIDEO_LOCKED=1", launcher)
        self.assertIn(
            'MATRIX_BFM_ISAAC_RENDER_SCREEN_PERCENTAGE="$BFM_VIDEO_SCREEN_PERCENTAGE"',
            launcher,
        )
        self.assertNotIn("OPERATOR_UE_EXTRA_EXEC_CMDS", launcher)
        self.assertIn(
            "Qualified BFM/Isaac rejects MATRIX_UE_EXTRA_EXEC_CMDS",
            launcher,
        )
        self.assertIn("Generic video state reached the locked BFM renderer", run_sim)
        self.assertIn(
            'UE_EXEC_CMDS="${UE_EXEC_CMDS},r.ScreenPercentage '
            '${VIDEO_SCREEN_PERCENTAGE}"',
            run_sim,
        )
        self.assertIn("resolved-video-settings.json", launcher)
        self.assertIn('--video-settings "$RESOLVED_VIDEO_SETTINGS"', launcher)

    def test_material_bridge_contract_is_pinned_and_verified_by_launcher(self) -> None:
        launcher = (REPO_ROOT / "scripts/run_matrix_bfm_isaac.sh").read_text(
            encoding="utf-8"
        )
        bootstrap = (
            REPO_ROOT / "scripts/bootstrap_matrix_bfm_isaac.sh"
        ).read_text(encoding="utf-8")
        bridge = self.lock["ue_material_bridge"]

        self.assertEqual(
            bridge,
            {
                "relative_path": (
                    "outputs/runtime/matrix-ue-material-fix/"
                    "libmatrix_ue_material_fix.so"
                ),
                "sha256": (
                    "9f64dd949bd44be61a11dcbbe3e5a49f6ef6f6f318c4771a24385e9781840b96"
                ),
                "ue_binary_relative_path": (
                    "src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux/"
                    "zsibot_mujoco_ue"
                ),
                "ue_binary_build_id": "056e17b8675b1006",
            },
        )
        for source in (launcher, bootstrap):
            self.assertIn("--expected-sha256", source)
            self.assertIn("--expected-ue-build-id", source)
        self.assertIn('--material-bridge "$MATERIAL_BRIDGE_PATH"', launcher)
        self.assertIn(
            'MATRIX_UE_MATERIAL_FIX_PRELOAD="$MATERIAL_BRIDGE_PATH"',
            launcher,
        )
        self.assertIn("rejects material bridge overrides", launcher)

    def test_runtime_checkout_rejects_untracked_files(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "runtime"
            root.mkdir()
            subprocess.run(("git", "init", "-q", str(root)), check=True)
            subprocess.run(
                ("git", "-C", str(root), "config", "user.email", "test@example.com"),
                check=True,
            )
            subprocess.run(
                ("git", "-C", str(root), "config", "user.name", "Test"),
                check=True,
            )
            (root / "tracked.txt").write_text("locked\n", encoding="utf-8")
            subprocess.run(("git", "-C", str(root), "add", "tracked.txt"), check=True)
            subprocess.run(
                ("git", "-C", str(root), "commit", "-qm", "fixture"),
                check=True,
            )
            commit = subprocess.run(
                ("git", "-C", str(root), "rev-parse", "HEAD"),
                check=True,
                capture_output=True,
                text=True,
            ).stdout.strip()
            lock = copy.deepcopy(self.lock)
            lock["bfm_runtime"]["commit"] = commit
            lock["bfm_runtime"]["critical_files"] = []
            (root / "untracked.py").write_text("raise SystemExit\n", encoding="utf-8")

            checks = MODULE.verify_runtime_checkout(lock, root)

            checkout = next(
                check for check in checks if check.name == "runtime_checkout_clean"
            )
            self.assertFalse(checkout.ok)
            self.assertIn("untracked.py", checkout.detail)

    def test_matrix_port_requires_clean_ancestor_and_locked_files(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "matrix"
            critical = root / "scripts/launcher.sh"
            critical.parent.mkdir(parents=True)
            critical.write_text("locked\n", encoding="utf-8")
            subprocess.run(("git", "init", "-q", str(root)), check=True)
            subprocess.run(
                ("git", "-C", str(root), "config", "user.email", "test@example.com"),
                check=True,
            )
            subprocess.run(
                ("git", "-C", str(root), "config", "user.name", "Test"),
                check=True,
            )
            subprocess.run(("git", "-C", str(root), "add", "."), check=True)
            subprocess.run(
                ("git", "-C", str(root), "commit", "-qm", "fixture"),
                check=True,
            )
            commit = subprocess.run(
                ("git", "-C", str(root), "rev-parse", "HEAD"),
                check=True,
                capture_output=True,
                text=True,
            ).stdout.strip()
            lock = copy.deepcopy(self.lock)
            lock["matrix_port"]["base_commit"] = commit
            lock["matrix_port"]["critical_files"] = [
                {
                    "path": "scripts/launcher.sh",
                    "sha256": MODULE.sha256_file(critical),
                }
            ]

            checks, actual_commit = MODULE.verify_matrix_port(lock, root)
            self.assertEqual(actual_commit, commit)
            self.assertTrue(all(check.ok for check in checks), checks)

            critical.write_text("drift\n", encoding="utf-8")
            checks, _ = MODULE.verify_matrix_port(lock, root)
            by_name = {check.name: check for check in checks}
            self.assertFalse(by_name["matrix_port_checkout_clean"].ok)
            self.assertFalse(
                by_name["matrix_port_file:scripts/launcher.sh"].ok
            )

    def test_isaac_runtime_locks_editable_checkout_and_exact_overlay(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            checkout = root / "IsaacLab"
            module = checkout / "source/isaaclab/isaaclab/__init__.py"
            converter = (
                checkout
                / "source/isaaclab/isaaclab/sim/converters/urdf_converter.py"
            )
            prims = checkout / "source/isaaclab/isaaclab/sim/utils/prims.py"
            unused_kit = checkout / "apps/isaaclab.python.sonic.kit"
            for path in (module, converter, prims):
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("baseline\n", encoding="utf-8")
            subprocess.run(("git", "init", "-q", str(checkout)), check=True)
            subprocess.run(
                ("git", "-C", str(checkout), "config", "user.email", "test@example.com"),
                check=True,
            )
            subprocess.run(
                ("git", "-C", str(checkout), "config", "user.name", "Test"),
                check=True,
            )
            subprocess.run(("git", "-C", str(checkout), "add", "."), check=True)
            subprocess.run(
                ("git", "-C", str(checkout), "commit", "-qm", "fixture"),
                check=True,
            )
            commit = subprocess.run(
                ("git", "-C", str(checkout), "rev-parse", "HEAD"),
                check=True,
                capture_output=True,
                text=True,
            ).stdout.strip()
            converter.write_text("compat converter\n", encoding="utf-8")
            prims.write_text("compat prims\n", encoding="utf-8")
            unused_kit.parent.mkdir(parents=True, exist_ok=True)
            unused_kit.write_text("unused experience\n", encoding="utf-8")

            lock = copy.deepcopy(self.lock)
            isaaclab = lock["isaac_runtime"]["isaaclab"]
            isaaclab["commit"] = commit
            for entry in isaaclab["critical_files"]:
                entry["sha256"] = MODULE.sha256_file(checkout / entry["path"])

            venv = root / "venv"
            runtime_python = venv / "bin/python"
            runtime_python.parent.mkdir(parents=True)
            (venv / "pyvenv.cfg").write_text(
                "include-system-site-packages = true\n", encoding="utf-8"
            )
            payload = {
                "implementation": lock["isaac_runtime"]["python_implementation"],
                "version": lock["isaac_runtime"]["python_version"],
                "machine": lock["isaac_runtime"]["platform_machine"],
                "prefix": str(venv.resolve()),
                "base_prefix": str((root / "base-python").resolve()),
                "module_file": str(module.resolve()),
                "checkout_root": str(checkout.resolve()),
                "distributions": lock["isaac_runtime"]["distributions"],
            }
            runtime_python.write_text(
                "#!/bin/sh\nprintf '%s\\n' "
                + repr(json.dumps(payload, sort_keys=True))
                + "\n",
                encoding="utf-8",
            )
            runtime_python.chmod(0o755)

            checks = MODULE.verify_isaac_runtime(lock, runtime_python)
            self.assertTrue(all(check.ok for check in checks), checks)

            (checkout / "unexpected.py").write_text("drift\n", encoding="utf-8")
            checks = MODULE.verify_isaac_runtime(lock, runtime_python)
            by_name = {check.name: check for check in checks}
            self.assertFalse(by_name["isaaclab_checkout_status"].ok)

    def test_schema_rejects_isaac_runtime_overlay_drift(self) -> None:
        mutated = copy.deepcopy(self.lock)
        mutated["isaac_runtime"]["isaaclab"]["allowed_status"].append(
            "?? unexpected.py"
        )

        with self.assertRaisesRegex(ValueError, "checkout status allowlist"):
            MODULE.validate_schema(mutated)

    def test_schema_rejects_execution_contract_drift(self) -> None:
        mutated = copy.deepcopy(self.lock)
        mutated["execution_contract"]["physics_device"] = "cuda:0"

        with self.assertRaisesRegex(ValueError, "execution contract"):
            MODULE.validate_schema(mutated)

    def test_schema_rejects_a_semantically_reordered_joint_contract(self) -> None:
        mutated = copy.deepcopy(self.lock)
        order = mutated["wire_contract"]["joint_order"]
        order[0], order[1] = order[1], order[0]
        with self.assertRaisesRegex(ValueError, "frozen Matrix order"):
            MODULE.validate_schema(mutated)

    def test_schema_rejects_visual_import_python_drift(self) -> None:
        mutated = copy.deepcopy(self.lock)
        mutated["visual_import"]["python_version"] = "3.11"

        with self.assertRaisesRegex(ValueError, "visual-import closure"):
            MODULE.validate_schema(mutated)

    def test_schema_rejects_moon_asset_hash_drift(self) -> None:
        mutated = copy.deepcopy(self.lock)
        mutated["scene_assets"]["collision_usd_sha256"] = "not-a-hash"

        with self.assertRaisesRegex(ValueError, "lowercase SHA256"):
            MODULE.validate_schema(mutated)

    def test_schema_rejects_material_bridge_contract_drift(self) -> None:
        mutations = (
            ("relative_path", "outputs/runtime/alternate.so"),
            ("sha256", "not-a-hash"),
            ("ue_binary_relative_path", "src/UeSim/alternate"),
            ("ue_binary_build_id", "0" * 16),
        )
        for field, value in mutations:
            with self.subTest(field=field):
                mutated = copy.deepcopy(self.lock)
                mutated["ue_material_bridge"][field] = value
                with self.assertRaises(ValueError):
                    MODULE.validate_schema(mutated)

    def test_material_bridge_verifier_rejects_tamper_symlink_and_wrong_path(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "matrix"
            bridge = root / self.lock["ue_material_bridge"]["relative_path"]
            ue_binary = (
                root
                / self.lock["ue_material_bridge"]["ue_binary_relative_path"]
            )
            bridge.parent.mkdir(parents=True)
            ue_binary.parent.mkdir(parents=True)
            bridge.write_bytes(b"audited bridge fixture")
            bridge.chmod(0o755)
            ue_binary.write_bytes(b"ue fixture")
            lock = copy.deepcopy(self.lock)
            lock["ue_material_bridge"]["sha256"] = MODULE.sha256_file(bridge)
            readelf = subprocess.CompletedProcess(
                args=("readelf",),
                returncode=0,
                stdout="    Build ID: 056e17b8675b1006\n",
                stderr="",
            )
            with mock.patch.object(MODULE.subprocess, "run", return_value=readelf):
                checks, evidence = MODULE.verify_ue_material_bridge(
                    lock, root, bridge
                )
            self.assertTrue(all(check.ok for check in checks), checks)
            self.assertEqual(evidence["sha256"], lock["ue_material_bridge"]["sha256"])

            wrong_readelf = subprocess.CompletedProcess(
                args=("readelf",),
                returncode=0,
                stdout=f"    Build ID: {'0' * 16}\n",
                stderr="",
            )
            with mock.patch.object(
                MODULE.subprocess, "run", return_value=wrong_readelf
            ):
                checks, _ = MODULE.verify_ue_material_bridge(lock, root, bridge)
            self.assertFalse(
                {check.name: check for check in checks}[
                    "matrix_ue_binary_build_id"
                ].ok
            )

            bridge.write_bytes(b"tampered")
            with mock.patch.object(MODULE.subprocess, "run", return_value=readelf):
                checks, _ = MODULE.verify_ue_material_bridge(lock, root, bridge)
            self.assertFalse(
                {check.name: check for check in checks}[
                    "ue_material_bridge_sha256"
                ].ok
            )

            bridge.unlink()
            target = root / "target.so"
            target.write_bytes(b"audited bridge fixture")
            target.chmod(0o755)
            bridge.symlink_to(target)
            with mock.patch.object(MODULE.subprocess, "run", return_value=readelf):
                checks, _ = MODULE.verify_ue_material_bridge(lock, root, bridge)
            self.assertFalse(
                {check.name: check for check in checks}[
                    "ue_material_bridge_regular_file"
                ].ok
            )

            alternate = root / "alternate.so"
            alternate.write_bytes(b"audited bridge fixture")
            alternate.chmod(0o755)
            with mock.patch.object(MODULE.subprocess, "run", return_value=readelf):
                checks, _ = MODULE.verify_ue_material_bridge(
                    lock, root, alternate
                )
            self.assertFalse(
                {check.name: check for check in checks}[
                    "ue_material_bridge_path"
                ].ok
            )

    def test_repository_visual_lock_files_match_the_lock(self) -> None:
        checks = MODULE.verify_visual_lock_files(self.lock, REPO_ROOT)

        self.assertTrue(checks)
        self.assertTrue(all(check.ok for check in checks), checks)

    def test_matrix_visual_requires_the_exact_urdf_mesh_closure(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            matrix_root = root / "matrix"
            visual_root = root / "visual"
            manifest = matrix_root / "config/runtime/visual.SHA256SUMS"
            mesh = visual_root / "meshes/link.STL"
            urdf = visual_root / "g1.urdf"
            manifest.parent.mkdir(parents=True)
            mesh.parent.mkdir(parents=True)
            mesh.write_bytes(b"mesh")
            urdf.write_text(
                '<robot name="g1"><link name="pelvis"><visual><geometry>'
                '<mesh filename="meshes/link.STL"/>'
                "</geometry></visual></link></robot>\n",
                encoding="utf-8",
            )
            manifest_text = (
                f"{hashlib.sha256(urdf.read_bytes()).hexdigest()}  g1.urdf\n"
                f"{hashlib.sha256(mesh.read_bytes()).hexdigest()}  "
                "meshes/link.STL\n"
            )
            manifest.write_text(manifest_text, encoding="utf-8")
            lock = copy.deepcopy(self.lock)
            lock["matrix_visual"] = {
                "manifest": "config/runtime/visual.SHA256SUMS",
                "manifest_sha256": hashlib.sha256(
                    manifest_text.encode()
                ).hexdigest(),
                "file_count": 2,
                "urdf": "g1.urdf",
                "urdf_sha256": hashlib.sha256(urdf.read_bytes()).hexdigest(),
            }

            checks = MODULE.verify_matrix_visual(
                lock, matrix_root, visual_root
            )
            self.assertTrue(all(check.ok for check in checks), checks)

            mesh.write_bytes(b"tampered")
            checks = MODULE.verify_matrix_visual(
                lock, matrix_root, visual_root
            )
            by_name = {check.name: check for check in checks}
            self.assertFalse(by_name["matrix_visual_files"].ok)

    def test_visual_wheelhouse_requires_exact_hashed_inventory(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            matrix_root = root / "matrix"
            config_root = matrix_root / "config/runtime"
            wheelhouse = root / "wheelhouse"
            config_root.mkdir(parents=True)
            wheelhouse.mkdir()
            requirements = config_root / "requirements.txt"
            requirements.write_text("demo-package==1.2.3\n", encoding="utf-8")
            wheel_name = "demo_package-1.2.3-py3-none-any.whl"
            wheel_bytes = b"locked-wheel-bytes"
            wheel_hash = hashlib.sha256(wheel_bytes).hexdigest()
            manifest_text = f"{wheel_hash}  {wheel_name}\n"
            locked_manifest = config_root / "SHA256SUMS"
            locked_manifest.write_text(manifest_text, encoding="utf-8")
            (wheelhouse / "SHA256SUMS").write_text(
                manifest_text, encoding="utf-8"
            )
            (wheelhouse / wheel_name).write_bytes(wheel_bytes)
            lock = copy.deepcopy(self.lock)
            visual = lock["visual_import"]
            visual.update(
                {
                    "requirements": "config/runtime/requirements.txt",
                    "requirements_sha256": hashlib.sha256(
                        requirements.read_bytes()
                    ).hexdigest(),
                    "wheelhouse_manifest": "config/runtime/SHA256SUMS",
                    "wheelhouse_manifest_sha256": hashlib.sha256(
                        manifest_text.encode()
                    ).hexdigest(),
                    "wheel_count": 1,
                    "wheel_bytes": len(wheel_bytes),
                }
            )

            checks = MODULE.verify_visual_wheelhouse(
                lock, matrix_root, wheelhouse
            )
            self.assertTrue(all(check.ok for check in checks), checks)

            (wheelhouse / "unlisted.whl").write_bytes(b"extra")
            checks = MODULE.verify_visual_wheelhouse(
                lock, matrix_root, wheelhouse
            )
            inventory = {
                check.name: check for check in checks
            }["visual_wheelhouse_inventory"]
            self.assertFalse(inventory.ok)

            (wheelhouse / "unlisted.whl").unlink()
            (wheelhouse / wheel_name).write_bytes(b"tampered")
            checks = MODULE.verify_visual_wheelhouse(
                lock, matrix_root, wheelhouse
            )
            hashes = {
                check.name: check for check in checks
            }["visual_wheelhouse_hashes"]
            self.assertFalse(hashes.ok)

    def test_visual_venv_missing_marker_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            venv = Path(temporary) / "venv"
            (venv / "bin").mkdir(parents=True)
            (venv / "pyvenv.cfg").write_text(
                "include-system-site-packages = false\n", encoding="utf-8"
            )

            checks = MODULE.verify_visual_venv(self.lock, REPO_ROOT, venv)
            by_name = {check.name: check for check in checks}
            self.assertFalse(by_name["visual_venv_marker"].ok)
            self.assertFalse(by_name["visual_venv_python"].ok)

    def test_scene_assets_bind_visual_and_collision_hashes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            matrix_root = root / "matrix"
            collision_root = root / "collision"
            visual_bin = matrix_root / "dynamicmaps/moonworld.bin"
            collision_usd = collision_root / "collision.usda"
            manifest_path = collision_root / "manifest.json"
            visual_bin.parent.mkdir(parents=True)
            collision_root.mkdir()
            visual_bin.write_bytes(b"visual")
            collision_usd.write_bytes(b"collision")
            lock = copy.deepcopy(self.lock)
            assets = lock["scene_assets"]
            assets["visual_bin_bytes"] = visual_bin.stat().st_size
            assets["visual_bin_sha256"] = hashlib.sha256(
                visual_bin.read_bytes()
            ).hexdigest()
            assets["collision_usd_sha256"] = hashlib.sha256(
                collision_usd.read_bytes()
            ).hexdigest()
            scene = lock["scene_collision_contract"]
            manifest = {
                "schema_version": 2,
                "source": "/source/moonworld.bin",
                "source_sha256": assets["visual_bin_sha256"],
                "source_size": 6000,
                "source_resolution_m": 0.1,
                "sample_stride": 4,
                "collision_resolution_m": 0.4,
                "patch_size_m": 240.0,
                "center_x_m": 23.0,
                "center_y_m": 13.0,
                "patch_side": 601,
                "vertex_count": 361201,
                "quad_count": 360000,
                "x_min_m": scene["x_min_m"],
                "x_max_m": scene["x_max_m"],
                "y_min_m": scene["y_min_m"],
                "y_max_m": scene["y_max_m"],
                "z_min_m": -20.0,
                "z_max_m": 9.0,
                "ground_z_m": -2.0390634536743164,
                "collision": "/collision/collision.usda",
                "collision_sha256": assets["collision_usd_sha256"],
            }
            manifest_path.write_text(
                json.dumps(manifest), encoding="utf-8"
            )
            assets["collision_manifest_sha256"] = hashlib.sha256(
                manifest_path.read_bytes()
            ).hexdigest()

            checks = MODULE.verify_scene_assets(
                lock, matrix_root, collision_root
            )
            self.assertTrue(all(check.ok for check in checks), checks)

            collision_usd.write_bytes(b"tampered")
            checks = MODULE.verify_scene_assets(
                lock, matrix_root, collision_root
            )
            by_name = {check.name: check for check in checks}
            self.assertFalse(by_name["moon_collision_usd"].ok)

    def test_correctness_and_realtime_checks_pass_independently(self) -> None:
        checks, metrics = MODULE.verify_evidence(
            self.lock, self.runtime_report(), self.relay_status()
        )

        correctness = [check for check in checks if check.gate == "correctness"]
        realtime = [check for check in checks if check.gate == "realtime"]
        self.assertTrue(correctness)
        self.assertTrue(realtime)
        self.assertTrue(all(check.ok for check in correctness))
        self.assertTrue(all(check.ok for check in realtime))
        self.assertEqual(metrics["control_hz_wall"], 50.0)
        self.assertEqual(metrics["physics_hz_wall"], 200.0)
        self.assertEqual(metrics["simulation_realtime_factor"], 1.0)

    def test_slow_wall_clock_fails_only_realtime_gate(self) -> None:
        checks, metrics = MODULE.verify_evidence(
            self.lock,
            self.runtime_report(wall_seconds=8.0),
            self.relay_status(),
        )

        correctness = [check for check in checks if check.gate == "correctness"]
        realtime = [check for check in checks if check.gate == "realtime"]
        self.assertTrue(all(check.ok for check in correctness))
        self.assertFalse(all(check.ok for check in realtime))
        self.assertEqual(metrics["control_hz_wall"], 12.5)
        self.assertEqual(metrics["physics_hz_wall"], 50.0)
        self.assertEqual(metrics["simulation_realtime_factor"], 0.25)

    def test_wrong_execution_topology_fails_correctness(self) -> None:
        report = self.runtime_report()
        report["physics_device"] = "cuda:0"
        report["teacher_onnx_session"]["intra_op_num_threads"] = 24

        checks, _ = MODULE.verify_evidence(
            self.lock,
            report,
            self.relay_status(),
        )
        by_name = {check.name: check for check in checks}
        self.assertFalse(by_name["physics_device"].ok)
        self.assertFalse(by_name["teacher_onnx_session"].ok)
        self.assertEqual(by_name["physics_device"].gate, "correctness")
        self.assertEqual(by_name["teacher_onnx_session"].gate, "correctness")

    def test_short_walk_only_smoke_fails_command_coverage(self) -> None:
        report = self.runtime_report()
        report["schedule"] = [["stand", 1.0], ["walk", 2.0]]
        report["observed_gaits"] = ["stand", "walk"]

        checks, _ = MODULE.verify_evidence(
            self.lock, report, self.relay_status()
        )
        by_name = {check.name: check for check in checks}
        self.assertFalse(by_name["command_schedule_coverage"].ok)
        self.assertEqual(by_name["command_schedule_coverage"].gate, "correctness")
        self.assertFalse(by_name["observed_gait_coverage"].ok)
        self.assertTrue(
            all(
                check.ok
                for check in checks
                if check.gate == "correctness"
                and check.name
                not in {"command_schedule_coverage", "observed_gait_coverage"}
            )
        )

    def test_interactive_mode_does_not_claim_automated_command_coverage(self) -> None:
        report = self.runtime_report()
        report["mode"] = "interactive"
        report["schedule"] = None

        checks, _ = MODULE.verify_evidence(
            self.lock,
            report,
            self.relay_status(),
        )
        by_name = {check.name: check for check in checks}

        self.assertFalse(by_name["command_schedule_coverage"].ok)
        self.assertEqual(by_name["command_schedule_coverage"].gate, "manual")
        self.assertIn("mode=interactive", by_name["command_schedule_coverage"].detail)
        self.assertTrue(by_name["locked_four_substeps_per_action"].ok)
        self.assertIn(
            "locked_runtime_source_and_clock_contract",
            by_name["locked_four_substeps_per_action"].detail,
        )

    def test_correctness_only_mode_does_not_hide_realtime_failure(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            report_path = root / "report.json"
            relay_path = root / "relay.json"
            video_path = root / "video.json"
            strict_output = root / "strict.json"
            diagnostic_output = root / "diagnostic.json"
            report_path.write_text(
                json.dumps(self.runtime_report(wall_seconds=8.0)), encoding="utf-8"
            )
            relay_path.write_text(json.dumps(self.relay_status()), encoding="utf-8")
            video_path.write_text(
                json.dumps(self.resolved_video_settings()), encoding="utf-8"
            )
            common = [
                "--lock",
                str(self.lock_path),
                "--report",
                str(report_path),
                "--relay-status",
                str(relay_path),
                "--video-settings",
                str(video_path),
            ]

            common += self.acceptance_runtime_args(root)
            with self.acceptance_verifier_patches(), redirect_stdout(
                io.StringIO()
            ), redirect_stderr(io.StringIO()):
                strict_exit = MODULE.main(common + ["--output", str(strict_output)])
                diagnostic_exit = MODULE.main(
                    common
                    + [
                        "--correctness-only",
                        "--output",
                        str(diagnostic_output),
                    ]
                )

            strict = json.loads(strict_output.read_text(encoding="utf-8"))
            diagnostic = json.loads(diagnostic_output.read_text(encoding="utf-8"))
            self.assertEqual(strict_exit, 1)
            self.assertTrue(strict["correctness_ok"])
            self.assertFalse(strict["realtime_ok"])
            self.assertFalse(strict["overall_ok"])
            self.assertEqual(
                strict["resolved_video_settings"], self.resolved_video_settings()
            )
            self.assertEqual(strict["schema"], MODULE.ACCEPTANCE_SCHEMA)
            self.assertEqual(diagnostic_exit, 0)
            self.assertTrue(diagnostic["correctness_ok"])
            self.assertFalse(diagnostic["realtime_ok"])
            self.assertTrue(diagnostic["correctness_only"])
            self.assertTrue(diagnostic["overall_ok"])

    def test_interactive_manual_review_blocks_overall_success(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            report = self.runtime_report(wall_seconds=8.0)
            report["mode"] = "interactive"
            report["schedule"] = None
            report_path = root / "report.json"
            relay_path = root / "relay.json"
            video_path = root / "video.json"
            output_path = root / "acceptance.json"
            report_path.write_text(json.dumps(report), encoding="utf-8")
            relay_path.write_text(
                json.dumps(self.relay_status()), encoding="utf-8"
            )
            video_path.write_text(
                json.dumps(self.resolved_video_settings()), encoding="utf-8"
            )

            with self.acceptance_verifier_patches(), redirect_stdout(
                io.StringIO()
            ), redirect_stderr(io.StringIO()):
                exit_code = MODULE.main(
                    [
                        "--lock",
                        str(self.lock_path),
                        "--report",
                        str(report_path),
                        "--relay-status",
                        str(relay_path),
                        "--video-settings",
                        str(video_path),
                        *self.acceptance_runtime_args(root),
                        "--output",
                        str(output_path),
                        "--correctness-only",
                    ]
                )

            acceptance = json.loads(output_path.read_text(encoding="utf-8"))
            self.assertEqual(exit_code, 1)
            self.assertTrue(acceptance["correctness_ok"])
            self.assertFalse(acceptance["realtime_ok"])
            self.assertFalse(acceptance["manual_ok"])
            self.assertTrue(acceptance["manual_review_required"])
            self.assertFalse(acceptance["overall_ok"])

    def test_invalid_resolved_video_settings_fail_correctness(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            report_path = root / "report.json"
            relay_path = root / "relay.json"
            video_path = root / "video.json"
            output_path = root / "acceptance.json"
            report_path.write_text(
                json.dumps(self.runtime_report()), encoding="utf-8"
            )
            relay_path.write_text(
                json.dumps(self.relay_status()), encoding="utf-8"
            )
            invalid = self.resolved_video_settings()
            invalid["unknown"] = True
            video_path.write_text(json.dumps(invalid), encoding="utf-8")

            with self.acceptance_verifier_patches(), redirect_stdout(
                io.StringIO()
            ), redirect_stderr(io.StringIO()):
                exit_code = MODULE.main(
                    [
                        "--lock",
                        str(self.lock_path),
                        "--report",
                        str(report_path),
                        "--relay-status",
                        str(relay_path),
                        "--video-settings",
                        str(video_path),
                        *self.acceptance_runtime_args(root),
                        "--output",
                        str(output_path),
                    ]
                )

            acceptance = json.loads(output_path.read_text(encoding="utf-8"))
            self.assertEqual(exit_code, 1)
            self.assertFalse(acceptance["correctness_ok"])
            self.assertIsNone(acceptance["resolved_video_settings"])

    def test_acceptance_evidence_rejects_an_incomplete_runtime_closure(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            report_path = root / "report.json"
            relay_path = root / "relay.json"
            video_path = root / "video.json"
            report_path.write_text(
                json.dumps(self.runtime_report()), encoding="utf-8"
            )
            relay_path.write_text(
                json.dumps(self.relay_status()), encoding="utf-8"
            )
            video_path.write_text(
                json.dumps(self.resolved_video_settings()), encoding="utf-8"
            )

            with redirect_stderr(io.StringIO()) as stderr:
                with self.assertRaises(SystemExit) as raised:
                    MODULE.main(
                        [
                            "--lock",
                            str(self.lock_path),
                            "--report",
                            str(report_path),
                            "--relay-status",
                            str(relay_path),
                            "--video-settings",
                            str(video_path),
                        ]
                    )
            self.assertEqual(raised.exception.code, 2)
            self.assertIn("complete runtime closure", stderr.getvalue())
            self.assertIn("--material-bridge", stderr.getvalue())

    def test_runtime_only_verification_cannot_write_an_acceptance_file(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output_path = root / "acceptance.json"
            with redirect_stderr(io.StringIO()) as stderr:
                with self.assertRaises(SystemExit) as raised:
                    MODULE.main(
                        [
                            "--lock",
                            str(self.lock_path),
                            "--runtime-root",
                            str(root / "runtime"),
                            "--output",
                            str(output_path),
                        ]
                    )
            self.assertEqual(raised.exception.code, 2)
            self.assertIn(
                "require complete acceptance evidence", stderr.getvalue()
            )
            self.assertFalse(output_path.exists())


if __name__ == "__main__":
    unittest.main()
