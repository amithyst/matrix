from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock
import xml.etree.ElementTree as ET


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = REPO_ROOT / "scripts"
if os.fspath(SCRIPTS) not in sys.path:
    sys.path.insert(0, os.fspath(SCRIPTS))


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


REPLAY = load_module(
    "replay_matrix_physics_trace", SCRIPTS / "replay_matrix_physics_trace.py"
)
STAGE = load_module(
    "stage_matrix_trace_model", SCRIPTS / "stage_matrix_trace_model.py"
)
POSTFLIGHT = load_module(
    "verify_matrix_scene6_task_video",
    SCRIPTS / "verify_matrix_scene6_task_video.py",
)


def write_fixture(root: Path, *, frame_count: int = 2) -> tuple[Path, Path, Path]:
    model_root = root / "model"
    meshes = model_root / "meshes"
    meshes.mkdir(parents=True)
    (meshes / "pelvis.stl").write_bytes(b"solid pelvis\nendsolid pelvis\n")
    motors = "\n".join(
        f'<motor name="motor_{index}" joint="{joint}" />'
        for index, joint in enumerate(STAGE.CANONICAL_ACTUATOR_JOINTS)
    )
    robot = model_root / "g1_29dof_dex3.scene6.xml"
    robot.write_text(
        f"""<mujoco model="G1 Dex3 task">
  <compiler meshdir="meshes" />
  <asset><mesh name="pelvis_mesh" file="pelvis.stl" /></asset>
  <worldbody>
    <body name="pelvis"><freejoint name="root" />
      <geom type="mesh" mesh="pelvis_mesh" />
    </body>
    <body name="pick_cube"><freejoint name="pick_cube_joint" />
      <geom type="box" size="0.03 0.03 0.03" />
    </body>
  </worldbody>
  <actuator>{motors}</actuator>
</mujoco>
""",
        encoding="utf-8",
    )
    scene = model_root / "scene6_house_task.xml"
    scene.write_text(
        """<mujoco model="scene6"><include file="g1_29dof_dex3.scene6.xml" />
<worldbody><geom name="worktop" type="box" size="1 1 0.1" /></worldbody>
</mujoco>
""",
        encoding="utf-8",
    )
    frames = [
        {
            "step": index,
            "time_s": index * 0.04,
            "qpos": [float(index)] + [0.0] * 56,
            "qvel": [0.0] * 55,
            "ctrl": [0.0] * 43,
            "controller_phase": "navigation" if index == 0 else "place",
        }
        for index in range(frame_count)
    ]
    trace = root / "physics-trace.json"
    trace.write_text(
        json.dumps(
            {
                "schema_id": "twinbot.physics_trace.mujoco.v0",
                "physics_trace_id": "trace_fixture",
                "physics_backend": "mujoco",
                "model_path": str(scene),
                "render_robot_model_path": str(robot),
                "render_robot_model_sha256": REPLAY._sha256(robot),
                "dimensions": {"nq": 57, "nv": 55, "nu": 43},
                "physics_timestep_s": 0.002,
                "sample_fps": 25.0,
                "persistent_world_state": True,
                "status": "succeeded",
                "control": {
                    "controller": "behavior_tree_controller_switching",
                    "mode": "persistent_matrix_home_world_v0",
                },
                "scene_context": {
                    "scene_number": 6,
                    "map_name": "/Game/Maps/HouseWorld",
                    "physics_execution": "offline_mujoco_persistent_world",
                    "intended_render_mode": "matrix_ue_trace_replay",
                    "manipulation_assistance": (
                        "contact_gated_wrist_cube_weld_and_anchored_stance"
                    ),
                },
                "transitions": [
                    {"phase": phase, "time_s": index * 0.01}
                    for index, phase in enumerate(
                        REPLAY.REQUIRED_TRANSITION_SUBSEQUENCE
                    )
                ],
                "frames": frames,
            }
        ),
        encoding="utf-8",
    )
    return trace, scene, robot


def matrix_roots(root: Path) -> dict[str, Path]:
    roots = {
        name: root / relative
        for name, relative in STAGE.TARGET_RELATIVE_ROOTS.items()
    }
    for target in roots.values():
        target.parent.mkdir(parents=True, exist_ok=True)
    return roots


class TraceValidationTest(unittest.TestCase):
    def test_accepts_full_scene6_task_trace_and_reports_truthful_boundary(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace, scene, _robot = write_fixture(Path(temporary))
            validated = REPLAY.validate_trace(trace)

            self.assertEqual(validated.dimensions, (57, 55, 43))
            self.assertEqual(validated.model_path, scene.resolve())
            inspection = validated.inspection()
            self.assertEqual(inspection["source_frame_count"], 2)
            self.assertEqual(inspection["fps"], 25.0)
            self.assertEqual(
                inspection["physics_execution"], "offline_mujoco_persistent_world"
            )
            self.assertEqual(inspection["render_mode"], "matrix_ue_trace_replay")

    def test_rejects_failed_wrong_shape_and_undisclosed_assistance(self) -> None:
        mutations = (
            (lambda payload: payload.update(status="failed"), "only a succeeded"),
            (
                lambda payload: payload["frames"][0].update(qpos=[0.0] * 56),
                "shape must be 57",
            ),
            (
                lambda payload: payload["scene_context"].update(
                    manipulation_assistance="pure_friction_grasp"
                ),
                "must disclose",
            ),
        )
        for mutate, message in mutations:
            with self.subTest(message=message), tempfile.TemporaryDirectory() as temporary:
                trace, _scene, _robot = write_fixture(Path(temporary))
                payload = json.loads(trace.read_text(encoding="utf-8"))
                mutate(payload)
                trace.write_text(json.dumps(payload), encoding="utf-8")
                with self.assertRaisesRegex(REPLAY.TraceValidationError, message):
                    REPLAY.validate_trace(trace)

    def test_rejects_duplicate_keys_nonfinite_numbers_and_symlink_trace(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            trace, _scene, _robot = write_fixture(root)
            duplicate = root / "duplicate.json"
            duplicate.write_text('{"schema_id":"a","schema_id":"b"}', encoding="utf-8")
            with self.assertRaisesRegex(REPLAY.TraceValidationError, "duplicate"):
                REPLAY.validate_trace(duplicate)

            payload = trace.read_text(encoding="utf-8").replace("0.0", "NaN", 1)
            trace.write_text(payload, encoding="utf-8")
            with self.assertRaisesRegex(REPLAY.TraceValidationError, "non-finite"):
                REPLAY.validate_trace(trace)

            trace, _scene, _robot = write_fixture(root / "second")
            link = root / "trace-link.json"
            link.symlink_to(trace)
            with self.assertRaisesRegex(REPLAY.TraceValidationError, "non-symlink"):
                REPLAY.validate_trace(link)

    def test_packet_matches_matrix_variable_vector_wire_format(self) -> None:
        frame = {
            "time_s": 1.25,
            "qpos": tuple(float(index) for index in range(57)),
            "qvel": tuple(float(index) for index in range(55)),
            "ctrl": tuple(float(index) for index in range(43)),
        }
        packet = REPLAY.pack_render_packet(frame)

        self.assertEqual(len(packet), 1260)
        sim_time, nq = struct.unpack_from("<dI", packet, 0)
        self.assertEqual(sim_time, 1.25)
        self.assertEqual(nq, 57)
        qvel_size_offset = 12 + 57 * 8
        self.assertEqual(struct.unpack_from("<I", packet, qvel_size_offset)[0], 55)
        ctrl_size_offset = qvel_size_offset + 4 + 55 * 8
        self.assertEqual(struct.unpack_from("<I", packet, ctrl_size_offset)[0], 43)


class TraceReplayTest(unittest.TestCase):
    def test_replay_writes_fresh_status_and_hashed_summary(self) -> None:
        class FakeSocket:
            def __init__(self) -> None:
                self.packets: list[tuple[bytes, tuple[str, int]]] = []

            def sendto(self, payload: bytes, address: tuple[str, int]) -> int:
                self.packets.append((payload, address))
                return len(payload)

            def close(self) -> None:
                return None

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            trace, _scene, _robot = write_fixture(root, frame_count=1)
            validated = REPLAY.validate_trace(trace)
            fake_socket = FakeSocket()
            status = root / "status.json"
            summary = root / "summary.json"

            with mock.patch.object(REPLAY.socket, "socket", return_value=fake_socket):
                result = REPLAY.replay(
                    validated,
                    status_path=status,
                    summary_path=summary,
                    pre_roll_s=0.0,
                    final_hold_s=0.0,
                    ue_pid=None,
                )

            self.assertTrue(result["passed"])
            self.assertEqual(result["packets"]["sent"], 1)
            self.assertEqual(fake_socket.packets[0][1], ("127.0.0.1", 9999))
            written_status = json.loads(status.read_text(encoding="utf-8"))
            self.assertTrue(written_status["completed"])
            self.assertFalse(written_status["active_lowcmd"])
            self.assertTrue(written_status["passed"])
            self.assertIsNone(written_status["physics_step_hz"])
            self.assertIsNone(written_status["rtf"])
            self.assertFalse(written_status["dds_lowcmd_active"])
            self.assertEqual(
                written_status["active_lowcmd_semantics"],
                "legacy_recorder_readiness_gate_no_dds_lowcmd",
            )
            self.assertEqual(written_status["dimensions"], {"nq": 57, "nv": 55, "nu": 43})
            written_summary = json.loads(summary.read_text(encoding="utf-8"))
            self.assertEqual(written_summary["trace"]["sha256"], validated.sha256)
            self.assertEqual(
                written_summary["model"]["sha256"],
                validated.render_model_sha256,
            )
            self.assertEqual(
                written_summary["scene_model"]["sha256"], validated.model_sha256
            )

    def test_sigterm_during_final_hold_is_clean_after_all_trace_frames(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            trace, _scene, _robot = write_fixture(root, frame_count=1)
            status = root / "status.json"
            summary = root / "summary.json"
            process = subprocess.Popen(
                [
                    sys.executable,
                    os.fspath(SCRIPTS / "replay_matrix_physics_trace.py"),
                    "--trace",
                    os.fspath(trace),
                    "--status-file",
                    os.fspath(status),
                    "--summary",
                    os.fspath(summary),
                    "--pre-roll",
                    "0",
                    "--final-hold",
                    "5",
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                try:
                    payload = json.loads(status.read_text(encoding="utf-8"))
                except (OSError, json.JSONDecodeError):
                    payload = {}
                if payload.get("packet_count", 0) >= 1:
                    break
                time.sleep(0.01)
            else:
                process.kill()
                self.fail("replay did not enter final hold")
            process.terminate()
            stdout, stderr = process.communicate(timeout=5.0)
            self.assertEqual(process.returncode, 0, (stdout, stderr))
            result = json.loads(summary.read_text(encoding="utf-8"))
            self.assertTrue(result["passed"])
            self.assertEqual(result["packets"]["trace_sent"], 1)
            self.assertEqual(
                result["completion"],
                "trace_complete_final_hold_stopped_by_launcher",
            )

    def test_repeated_sigterm_during_summary_write_keeps_receipts_atomic(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            trace, _scene, _robot = write_fixture(root, frame_count=1)
            status = root / "status.json"
            summary = root / "summary.json"
            child = r'''
import importlib.util
import os
from pathlib import Path
import signal
import sys

script, trace, status, summary = map(Path, sys.argv[1:])
spec = importlib.util.spec_from_file_location("matrix_replay_signal_test", script)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)
original_atomic_json = module._atomic_json

def interrupt_summary(path, payload):
    if payload.get("schema_id") == module.SUMMARY_SCHEMA:
        os.kill(os.getpid(), signal.SIGTERM)
    original_atomic_json(path, payload)

module._atomic_json = interrupt_summary
validated = module.validate_trace(trace)
result = module.replay(
    validated,
    status_path=status,
    summary_path=summary,
    pre_roll_s=0.0,
    final_hold_s=0.0,
    ue_pid=None,
)
raise SystemExit(0 if result["passed"] else 2)
'''
            process = subprocess.run(
                [
                    sys.executable,
                    "-c",
                    child,
                    os.fspath(SCRIPTS / "replay_matrix_physics_trace.py"),
                    os.fspath(trace),
                    os.fspath(status),
                    os.fspath(summary),
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=5.0,
            )

            self.assertEqual(process.returncode, 0, (process.stdout, process.stderr))
            written_summary = json.loads(summary.read_text(encoding="utf-8"))
            written_status = json.loads(status.read_text(encoding="utf-8"))
            self.assertTrue(written_summary["passed"])
            self.assertTrue(written_status["completed"])
            self.assertTrue(written_status["passed"])
            self.assertFalse(written_status["active_lowcmd"])


class ModelStageTest(unittest.TestCase):
    def test_rejects_noncanonical_full_hand_actuator_order(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            trace, _scene, robot = write_fixture(root / "source")
            tree = ET.parse(robot)
            actuator = tree.getroot().find("actuator")
            children = list(actuator)
            actuator.remove(children[0])
            actuator.insert(1, children[0])
            tree.write(robot, encoding="utf-8")
            trace_payload = json.loads(trace.read_text(encoding="utf-8"))
            trace_payload["render_robot_model_sha256"] = REPLAY._sha256(robot)
            trace.write_text(json.dumps(trace_payload), encoding="utf-8")
            matrix = root / "matrix"
            matrix_roots(matrix)

            with self.assertRaisesRegex(
                STAGE.ModelStageError, r"canonical G1\+Dex3"
            ):
                STAGE.stage(
                    matrix_root=matrix,
                    trace_path=trace,
                    model_override=None,
                    state_dir=root / "state",
                )

    def test_stages_mesh_closure_to_both_roots_and_restores_current_xml(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            trace, _scene, _robot = write_fixture(root / "source")
            matrix = root / "matrix"
            roots = matrix_roots(matrix)
            originals = {
                "mujoco": b"<mujoco model=\"old-mujoco\"/>\n",
                "ue": b"<mujoco model=\"old-ue\"/>\n",
            }
            for name, target in roots.items():
                target.mkdir()
                (target / "current.xml").write_bytes(originals[name])
                (target / "current.xml").chmod(0o664)
            runtime_config = matrix / STAGE.RUNTIME_MUTATION_RELATIVE_PATHS["config_json"]
            runtime_config.parent.mkdir(parents=True)
            runtime_config.write_bytes(b'{"original":true}\n')
            runtime_config.chmod(0o664)
            state_dir = root / "state"

            state = STAGE.stage(
                matrix_root=matrix,
                trace_path=trace,
                model_override=None,
                state_dir=state_dir,
            )

            self.assertTrue(state["active"])
            self.assertEqual(state["dimensions"], {"nq": 57, "nv": 55, "nu": 43})
            self.assertEqual(state["mesh_closure"]["file_count"], 1)
            for target in roots.values():
                current = ET.parse(target / "current.xml").getroot()
                meshdir = current.find("compiler").get("meshdir")
                self.assertFalse(Path(meshdir).is_absolute())
                self.assertEqual(
                    (target / meshdir / "pelvis.stl").read_bytes(),
                    b"solid pelvis\nendsolid pelvis\n",
                )
                self.assertEqual(
                    (target / "current.xml").stat().st_mode & 0o777,
                    0o664,
                )

            runtime_config.write_bytes(b'{"mutated":true}\n')
            runtime_config.chmod(0o600)
            generated_scene = (
                matrix / STAGE.RUNTIME_MUTATION_RELATIVE_PATHS["ue_custom_scene"]
            )
            generated_scene.parent.mkdir(parents=True, exist_ok=True)
            generated_scene.write_text("generated\n", encoding="utf-8")

            restored = STAGE.restore(matrix_root=matrix, state_dir=state_dir)
            self.assertFalse(restored["active"])
            for name, target in roots.items():
                self.assertEqual((target / "current.xml").read_bytes(), originals[name])
                self.assertEqual(
                    (target / "current.xml").stat().st_mode & 0o777,
                    0o664,
                )
            self.assertEqual(runtime_config.read_bytes(), b'{"original":true}\n')
            self.assertEqual(runtime_config.stat().st_mode & 0o777, 0o664)
            self.assertFalse(generated_scene.exists())

    def test_restore_removes_new_current_and_refuses_post_stage_drift(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            trace, _scene, _robot = write_fixture(root / "source")
            matrix = root / "matrix"
            roots = matrix_roots(matrix)
            state_dir = root / "state"
            STAGE.stage(
                matrix_root=matrix,
                trace_path=trace,
                model_override=None,
                state_dir=state_dir,
            )
            (roots["ue"] / "current.xml").write_text("changed", encoding="utf-8")

            with self.assertRaisesRegex(STAGE.ModelStageError, "changed after staging"):
                STAGE.restore(matrix_root=matrix, state_dir=state_dir)

            # Restore the expected staged bytes, then the original absence can
            # be reinstated for both roots.
            expected = (roots["mujoco"] / "current.xml").read_bytes()
            (roots["ue"] / "current.xml").write_bytes(expected)
            STAGE.restore(matrix_root=matrix, state_dir=state_dir)
            for target in roots.values():
                self.assertFalse((target / "current.xml").exists())


class ShellIntegrationContractTest(unittest.TestCase):
    def test_run_sim_external_replay_is_exclusive_and_supervised(self) -> None:
        source = (SCRIPTS / "run_sim.sh").read_text(encoding="utf-8")
        self.assertIn("MATRIX_EXTERNAL_REPLAY and MATRIX_SONIC are mutually exclusive", source)
        self.assertIn("! $MATRIX_EXTERNAL_REPLAY_ENABLED", source)
        self.assertIn(
            'wait -n -p COMPLETED_PID "$TRACE_REPLAY_PID" "$UE_SUPERVISOR_PID"',
            source,
        )
        self.assertLess(
            source.index('echo "[INFO] Starting UE"'),
            source.index(
                'wait_for_ue_map_ready "$UE_LOG" "$UE_LOG_START_OFFSET" "$MAPNAME"'
            ),
        )
        self.assertLess(
            source.index(
                'wait_for_ue_map_ready "$UE_LOG" "$UE_LOG_START_OFFSET" "$MAPNAME"'
            ),
            source.index('echo "[INFO] Starting Matrix UE physics-trace replay"'),
        )
        self.assertIn(
            "LogGlobalStatus: LoadMap Load map complete {sys.argv[3]}", source
        )
        self.assertIn("|| $MATRIX_EXTERNAL_REPLAY_ENABLED", source)
        self.assertIn(
            'local ue_target="$ue_model_root/custom/scene_terrain_custom.xml"',
            source,
        )
        self.assertIn('robot["use_custom_urdf"] = True', source)
        self.assertIn(
            'robot["custom_urdf"] = "custom/scene_terrain_custom.xml"',
            source,
        )
        self.assertIn("模型加载成功，开始初始化传感器/网格/线程", source)
        self.assertIn('print("model-failed")', source)
        self.assertIn(
            'XML_FILE="src/robot_mujoco/zsibot_robots/custom/current.xml"',
            source,
        )
        self.assertIn("Staged Matrix replay robot XML not found", source)

    def test_recording_contract_uses_status_and_fixed_25_fps(self) -> None:
        source = (SCRIPTS / "record_matrix_scene6_task_video.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("--ready status", source)
        self.assertIn("--fps 25", source)
        self.assertIn("frames / 25.0", source)
        self.assertIn("hold > capture", source)
        self.assertIn("--wait-launcher-exit-timeout", source)
        self.assertIn("not live SONIC manipulation", source)
        self.assertNotIn("jq", source)
        self.assertIn("verify_matrix_scene6_task_video.py", source)
        launcher = (SCRIPTS / "run_matrix_scene6_trace_replay.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("/tmp/matrix-sonic-${UID}.lock", launcher)
        self.assertNotIn("matrix-scene6-trace-replay.lock", launcher)
        self.assertIn("Recovering prior Matrix scene6 stage journal", launcher)


class VideoPostflightTest(unittest.TestCase):
    def test_requires_video_replay_and_restore_to_all_pass(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            matrix = root / "matrix"
            matrix.mkdir()
            subprocess.run(["git", "init", "-q", str(matrix)], check=True)
            subprocess.run(
                ["git", "-C", str(matrix), "config", "user.name", "Matrix Test"],
                check=True,
            )
            subprocess.run(
                [
                    "git",
                    "-C",
                    str(matrix),
                    "config",
                    "user.email",
                    "matrix-test@example.invalid",
                ],
                check=True,
            )
            subprocess.run(
                ["git", "-C", str(matrix), "commit", "-q", "--allow-empty", "-m", "fixture"],
                check=True,
            )
            output = root / "task.mp4"
            output.write_bytes(b"verified-video")
            video_sha = POSTFLIGHT._sha256(output)
            trace_sha = "a" * 64
            summary = root / "summary.json"
            summary.write_text(
                json.dumps(
                    {
                        "schema_id": "matrix.physics_trace_replay.summary.v1",
                        "passed": True,
                        "failure": None,
                        "completion": "scheduled_replay_complete",
                        "physics_execution": "offline_mujoco_persistent_world",
                        "render_mode": "matrix_ue_trace_replay",
                        "dimensions": {"nq": 57, "nv": 55, "nu": 43},
                        "source_frame_count": 10,
                        "packets": {"trace_sent": 10, "sent": 20, "expected": 20},
                        "trace": {"sha256": trace_sha},
                        "model": {"sha256": "b" * 64},
                        "scene_model": {"sha256": "c" * 64},
                    }
                ),
                encoding="utf-8",
            )
            restore = root / "restore.json"
            restore.write_text(
                json.dumps(
                    {
                        "schema_id": "matrix.physics_trace_model_stage.v1",
                        "active": False,
                        "phase": "restored",
                        "restored_targets": ["mujoco", "ue"],
                        "runtime_files": {"config": {}},
                        "restored_runtime_files": ["config"],
                        "trace": {"sha256": trace_sha},
                        "scene_model": {"sha256": "c" * 64},
                        "robot_model": {"sha256": "b" * 64},
                    }
                ),
                encoding="utf-8",
            )
            metadata = root / "metadata.json"
            metadata.write_text(
                json.dumps(
                    {
                        "capture": {"requested_fps": 25.0},
                        "quality": {"passed": True},
                        "video": {
                            "path": str(output),
                            "sha256": video_sha,
                            "fps": 25.0,
                            "width": 1920,
                            "height": 1008,
                            "duration_s": 5.0,
                            "decoded_frames": 125,
                        },
                        "sonic_status": {
                            "before": {
                                "active_lowcmd": True,
                                "active_lowcmd_semantics": (
                                    "legacy_recorder_readiness_gate_no_dds_lowcmd"
                                ),
                                "dds_lowcmd_active": False,
                            },
                            "after": {
                                "active_lowcmd": False,
                                "active_lowcmd_semantics": (
                                    "legacy_recorder_readiness_gate_no_dds_lowcmd"
                                ),
                                "dds_lowcmd_active": False,
                                "completed": True,
                                "passed": True,
                            },
                        },
                        "launcher": {
                            "return_code": 0,
                            "stopped_by_recorder": False,
                        },
                    }
                ),
                encoding="utf-8",
            )

            receipt = POSTFLIGHT.verify(
                output=output,
                metadata_path=metadata,
                summary_path=summary,
                restore_path=restore,
                matrix_root=matrix,
            )
            self.assertTrue(receipt["passed"])

            summary_payload = json.loads(summary.read_text(encoding="utf-8"))
            summary_payload["completion"] = (
                "trace_complete_final_hold_stopped_by_launcher"
            )
            summary.write_text(json.dumps(summary_payload), encoding="utf-8")
            with self.assertRaisesRegex(POSTFLIGHT.PostflightError, "final hold"):
                POSTFLIGHT.verify(
                    output=output,
                    metadata_path=metadata,
                    summary_path=summary,
                    restore_path=restore,
                    matrix_root=matrix,
                )
            summary_payload["completion"] = "scheduled_replay_complete"
            summary.write_text(json.dumps(summary_payload), encoding="utf-8")

            metadata_payload = json.loads(metadata.read_text(encoding="utf-8"))
            metadata_payload["launcher"]["return_code"] = 143
            metadata.write_text(json.dumps(metadata_payload), encoding="utf-8")
            with self.assertRaisesRegex(POSTFLIGHT.PostflightError, "naturally"):
                POSTFLIGHT.verify(
                    output=output,
                    metadata_path=metadata,
                    summary_path=summary,
                    restore_path=restore,
                    matrix_root=matrix,
                )
            metadata_payload["launcher"]["return_code"] = 0
            metadata_payload["launcher"]["stopped_by_recorder"] = True
            metadata.write_text(json.dumps(metadata_payload), encoding="utf-8")
            with self.assertRaisesRegex(POSTFLIGHT.PostflightError, "naturally"):
                POSTFLIGHT.verify(
                    output=output,
                    metadata_path=metadata,
                    summary_path=summary,
                    restore_path=restore,
                    matrix_root=matrix,
                )
            metadata_payload["launcher"]["stopped_by_recorder"] = False
            metadata_payload["sonic_status"]["after"]["completed"] = False
            metadata.write_text(json.dumps(metadata_payload), encoding="utf-8")
            with self.assertRaisesRegex(POSTFLIGHT.PostflightError, "final status"):
                POSTFLIGHT.verify(
                    output=output,
                    metadata_path=metadata,
                    summary_path=summary,
                    restore_path=restore,
                    matrix_root=matrix,
                )
            metadata_payload["sonic_status"]["after"]["completed"] = True
            metadata.write_text(json.dumps(metadata_payload), encoding="utf-8")

            payload = json.loads(summary.read_text(encoding="utf-8"))
            payload["passed"] = False
            summary.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(POSTFLIGHT.PostflightError, "did not pass"):
                POSTFLIGHT.verify(
                    output=output,
                    metadata_path=metadata,
                    summary_path=summary,
                    restore_path=restore,
                    matrix_root=matrix,
                )


if __name__ == "__main__":
    unittest.main()
