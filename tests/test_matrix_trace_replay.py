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
CAMERA = load_module(
    "matrix_scene6_camera_receipt", SCRIPTS / "matrix_scene6_camera_receipt.py"
)
POSTFLIGHT = load_module(
    "verify_matrix_scene6_task_video",
    SCRIPTS / "verify_matrix_scene6_task_video.py",
)


def write_camera_receipt_fixture(root: Path) -> tuple[Path, str]:
    path = root / "camera-receipt.json"
    path.write_text(
        json.dumps(
            {
                "schema_id": "matrix.scene6_camera_receipt.v1",
                "mode": "robot",
            }
        )
        + "\n",
        encoding="utf-8",
    )
    return path.resolve(), REPLAY._sha256(path)


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
            camera_receipt, camera_sha256 = write_camera_receipt_fixture(root)

            with mock.patch.object(REPLAY.socket, "socket", return_value=fake_socket):
                result = REPLAY.replay(
                    validated,
                    status_path=status,
                    summary_path=summary,
                    pre_roll_s=0.0,
                    final_hold_s=0.0,
                    ue_pid=None,
                    camera_receipt_path=camera_receipt,
                    expected_camera_receipt_sha256=camera_sha256,
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
            self.assertEqual(
                written_summary["camera_receipt"]["sha256"], camera_sha256
            )
            self.assertEqual(written_status["camera_receipt"], result["camera_receipt"])

    def test_camera_receipt_digest_mismatch_fails_before_status_or_packets(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            trace, _scene, _robot = write_fixture(root, frame_count=1)
            validated = REPLAY.validate_trace(trace)
            camera_receipt, _camera_sha256 = write_camera_receipt_fixture(root)
            status = root / "status.json"
            summary = root / "summary.json"

            with self.assertRaisesRegex(
                REPLAY.TraceValidationError, "changed before replay startup"
            ), mock.patch.object(REPLAY.socket, "socket") as socket_factory:
                REPLAY.replay(
                    validated,
                    status_path=status,
                    summary_path=summary,
                    pre_roll_s=0.0,
                    final_hold_s=0.0,
                    ue_pid=None,
                    camera_receipt_path=camera_receipt,
                    expected_camera_receipt_sha256="0" * 64,
                )

            socket_factory.assert_not_called()
            self.assertFalse(status.exists())
            self.assertFalse(summary.exists())

    def test_sigterm_during_final_hold_is_clean_after_all_trace_frames(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            trace, _scene, _robot = write_fixture(root, frame_count=1)
            status = root / "status.json"
            summary = root / "summary.json"
            camera_receipt, camera_sha256 = write_camera_receipt_fixture(root)
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
                    "--camera-receipt",
                    os.fspath(camera_receipt),
                    "--camera-receipt-sha256",
                    camera_sha256,
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
            camera_receipt, camera_sha256 = write_camera_receipt_fixture(root)
            child = r'''
import importlib.util
import os
from pathlib import Path
import signal
import sys

script, trace, status, summary, camera = map(Path, sys.argv[1:])
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
    camera_receipt_path=camera,
    expected_camera_receipt_sha256=module._sha256(camera),
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
                    os.fspath(camera_receipt),
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


class CameraReceiptTest(unittest.TestCase):
    def test_robot_receipt_binds_actual_commands_and_fresh_confirmation(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary).resolve()
            ready = root / "ready.json"
            CAMERA.confirm_ready(
                output=ready,
                mode="robot",
                framing_label="front-review",
            )
            receipt_path = root / "camera.json"
            exec_cmds = (
                "t.MaxFPS 25,"
                "set Engine.SpringArmComponent TargetArmLength 180,"
                "viewclass MujocoSim_Custom_C"
            )
            payload = CAMERA.write_receipt(
                output=receipt_path,
                mode="robot",
                spring_arm_cm=180.0,
                ue_exec_cmds=exec_cmds,
                project_root=root,
                contract=None,
                bundle=None,
                ue_log=None,
                log_offset=None,
                ready_file=ready,
            )

            self.assertEqual(CAMERA.load_receipt(receipt_path), payload)
            self.assertEqual(payload["camera_ready"]["framing_label"], "front-review")
            self.assertEqual(payload["camera_commands"][-1], "viewclass MujocoSim_Custom_C")

            rejected_commands = (
                (exec_cmds + ",ViewClass Spectator_C", "viewclass differs"),
                (
                    exec_cmds
                    + ",SET engine.springarmcomponent TARGETARMLENGTH 500",
                    "arm differs",
                ),
                (exec_cmds + "; ViewClass Spectator_C", "command separator"),
            )
            for bad_commands, expected_error in rejected_commands:
                with self.subTest(commands=bad_commands), self.assertRaisesRegex(
                    CAMERA.CameraReceiptError, expected_error
                ):
                    CAMERA.write_receipt(
                        output=root / "bad.json",
                        mode="robot",
                        spring_arm_cm=180.0,
                        ue_exec_cmds=bad_commands,
                        project_root=root,
                        contract=None,
                        bundle=None,
                        ue_log=None,
                        log_offset=None,
                        ready_file=ready,
                    )

            final_write_wins = (
                "ViewClass Spectator_C,"
                "SET Engine.SpringArmComponent TargetArmLength 500,"
                "viewclass mujocosim_custom_c,"
                "set engine.springarmcomponent targetarmlength 180"
            )
            accepted = CAMERA.write_receipt(
                output=root / "last-write-wins.json",
                mode="robot",
                spring_arm_cm=180.0,
                ue_exec_cmds=final_write_wins,
                project_root=root,
                contract=None,
                bundle=None,
                ue_log=None,
                log_offset=None,
                ready_file=ready,
            )
            self.assertEqual(accepted["ue_exec_cmds"], final_write_wins)

    def test_spectator_receipt_pins_active_bundle_and_exact_mount_segment(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary).resolve()
            contract_path = root / "contract.json"
            contract_path.write_text("{}\n", encoding="utf-8")
            bundle = root / "bundle"
            bundle.mkdir()
            active = root / CAMERA.overlay.RUNTIME_DIRECTORY
            active.mkdir(parents=True)
            ue_log = root / "src/UeSim/Linux/zsibot_mujoco_ue.log"
            ue_log.parent.mkdir(parents=True, exist_ok=True)
            ue_log.write_text(
                "LogPakFile: Display: Found Pak file "
                "../../../zsibot_mujoco_ue/Saved/Paks/"
                "MatrixCenteredCameraActive/"
                "pakchunk99-MatrixCentered-Linux_P.pak attempting to mount.\n"
                "LogPakFile: Display: Mounted IoStore container "
                '"../../../zsibot_mujoco_ue/Saved/Paks/'
                "MatrixCenteredCameraActive/"
                'pakchunk99-MatrixCentered-Linux_P.utoc"\n',
                encoding="utf-8",
            )
            artifacts = tuple(
                CAMERA.overlay.Artifact(name=name, size=size, sha256=digest)
                for name, (size, digest) in sorted(
                    CAMERA.overlay.PINNED_ARTIFACTS.items()
                )
            )
            contract = CAMERA.overlay.Contract(
                path=contract_path,
                artifacts=artifacts,
            )
            receipt_path = root / "camera.json"
            with mock.patch.object(
                CAMERA.overlay, "load_contract", return_value=contract
            ), mock.patch.object(
                CAMERA.overlay, "verify_bundle", return_value=bundle
            ), mock.patch.object(CAMERA.overlay, "_verify_directory"):
                payload = CAMERA.write_receipt(
                    output=receipt_path,
                    mode="spectator-overlay",
                    spring_arm_cm=180.0,
                    ue_exec_cmds=(
                        "t.MaxFPS 25,"
                        "set Engine.SpringArmComponent TargetArmLength 180,"
                        "viewclass Spectator_C"
                    ),
                    project_root=root,
                    contract=contract_path,
                    bundle=bundle,
                    ue_log=ue_log,
                    log_offset=0,
                    ready_file=None,
                )

            mount = payload["overlay"]["mount"]
            self.assertEqual(mount["start_offset"], 0)
            self.assertEqual(mount["end_offset"], ue_log.stat().st_size)
            self.assertEqual(mount["segment_size"], ue_log.stat().st_size)
            self.assertEqual(
                payload["overlay"]["bundle"]["artifacts"],
                [
                    {"name": artifact.name, "size": artifact.size, "sha256": artifact.sha256}
                    for artifact in artifacts
                ],
            )
            self.assertEqual(CAMERA.load_receipt(receipt_path), payload)
            active.rmdir()
            with ue_log.open("a", encoding="utf-8") as stream:
                stream.write("LogTemp: later UE output\n")
            with mock.patch.object(
                CAMERA.overlay, "load_contract", return_value=contract
            ), mock.patch.object(
                CAMERA.overlay, "verify_bundle", return_value=bundle
            ):
                self.assertEqual(
                    CAMERA.revalidate_receipt_evidence(payload, project_root=root),
                    payload,
                )
            active.mkdir()
            with mock.patch.object(
                CAMERA.overlay, "load_contract", return_value=contract
            ), mock.patch.object(
                CAMERA.overlay, "verify_bundle", return_value=bundle
            ), self.assertRaisesRegex(
                CAMERA.CameraReceiptError, "remains after cleanup"
            ):
                CAMERA.revalidate_receipt_evidence(payload, project_root=root)
            active.rmdir()

            forged = json.loads(json.dumps(payload))
            forged["overlay"]["contract"]["path"] = os.fspath(
                root / "missing-contract.json"
            )
            forged["overlay"]["bundle"]["path"] = os.fspath(
                root / "missing-bundle"
            )
            forged["overlay"]["mount"]["ue_log"] = os.fspath(root / "missing.log")
            self.assertEqual(CAMERA.validate_receipt_payload(forged), forged)
            with self.assertRaisesRegex(
                CAMERA.CameraReceiptError, "no longer verifies"
            ):
                CAMERA.revalidate_receipt_evidence(forged, project_root=root)

            original_log = ue_log.read_bytes()
            ue_log.write_bytes(b"X" + original_log[1:])
            with mock.patch.object(
                CAMERA.overlay, "load_contract", return_value=contract
            ), mock.patch.object(
                CAMERA.overlay, "verify_bundle", return_value=bundle
            ), self.assertRaisesRegex(
                CAMERA.CameraReceiptError, "segment SHA256"
            ):
                CAMERA.revalidate_receipt_evidence(payload, project_root=root)


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
        self.assertIn("MATRIX_EXTERNAL_REPLAY_CENTERED_CAMERA", source)
        self.assertIn("External replay centered camera requires", source)
        self.assertIn('CAMERA_CONFIGURATION_CONTEXT="external replay"', source)
        self.assertIn('GAME_CAMERA_VIEW_CLASS="Spectator_C"', source)
        self.assertIn('trap \'finalize_exit "$?"\' EXIT', source)
        self.assertIn("cleanup || cleanup_exit=$?", source)
        self.assertIn("trap '' SIGINT SIGTERM SIGHUP", source)

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
        self.assertIn("MATRIX_SCENE6_CAMERA_DISTANCE_CM:-180", launcher)
        self.assertIn("TargetArmLength ${CAMERA_DISTANCE_CM}", launcher)
        self.assertIn("viewclass MujocoSim_Custom_C", launcher)
        self.assertIn("--camera-mode robot|spectator-overlay", launcher)
        self.assertIn("MATRIX_EXTERNAL_REPLAY_CENTERED_CAMERA=1", launcher)
        self.assertIn("requested_viewclass=$CAMERA_VIEW_CLASS", launcher)
        self.assertIn("matrix_ue_overlay.py\" purge-stale", launcher)
        self.assertIn("matrix_ue_overlay.py\" verify-bundle", launcher)
        self.assertLess(
            launcher.index("matrix_ue_overlay.py\" purge-stale"),
            launcher.index('"${STAGE_COMMAND[@]}"'),
        )
        self.assertIn("matrix_scene6_camera", source)
        self.assertIn("matrix.scene6_video_metadata.v2", source)
        self.assertIn("--camera-receipt", source)

    def test_scene6_camera_arguments_fail_closed_before_staging(self) -> None:
        launcher = SCRIPTS / "run_matrix_scene6_trace_replay.sh"
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            trace = root / "trace.json"
            trace.write_text("{}\n", encoding="utf-8")

            cases = (
                (
                    ["--camera-mode", "sideways"],
                    "--camera-mode must be robot or spectator-overlay",
                ),
                (
                    ["--camera-distance-cm", "79"],
                    "--camera-distance-cm must be within 80..500",
                ),
                (
                    ["--camera-mode", "spectator-overlay"],
                    "--overlay-bundle is required for spectator-overlay",
                ),
            )
            for arguments, expected in cases:
                with self.subTest(arguments=arguments):
                    result = subprocess.run(
                        [
                            "/bin/bash",
                            os.fspath(launcher),
                            "--trace",
                            os.fspath(trace),
                            *arguments,
                        ],
                        text=True,
                        capture_output=True,
                        check=False,
                    )
                    self.assertEqual(result.returncode, 2)
                    self.assertIn(expected, result.stderr)

            bundle = root / "bundle"
            bundle.mkdir()
            linked_bundle = root / "bundle-link"
            linked_bundle.symlink_to(bundle, target_is_directory=True)
            linked = subprocess.run(
                [
                    "/bin/bash",
                    os.fspath(launcher),
                    "--trace",
                    os.fspath(trace),
                    "--camera-mode",
                    "spectator-overlay",
                    "--overlay-bundle",
                    os.fspath(linked_bundle),
                ],
                text=True,
                capture_output=True,
                check=False,
            )
            self.assertEqual(linked.returncode, 2)
            self.assertIn("--overlay-bundle must not be a symlink", linked.stderr)

    def test_scene6_outputs_cannot_alias_overlay_contract_or_bundle(self) -> None:
        launcher = SCRIPTS / "run_matrix_scene6_trace_replay.sh"
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary).resolve()
            matrix = root / "matrix"
            contract = matrix / "config/runtime/matrix-centered-camera-overlay-v3.json"
            contract.parent.mkdir(parents=True)
            contract_bytes = b'{"protected":true}\n'
            contract.write_bytes(contract_bytes)
            trace = root / "trace.json"
            trace.write_text("{}\n", encoding="utf-8")
            bundle = root / "bundle"
            bundle.mkdir()
            bundle_file = bundle / "pakchunk99-MatrixCentered-Linux_P.pak"
            bundle_file.write_bytes(b"protected-pak")
            active = (
                matrix
                / "src/UeSim/Linux/zsibot_mujoco_ue/Saved/Paks/MatrixCenteredCameraActive"
            )

            cases = (
                (["--status-file", os.fspath(contract)], os.fspath(contract)),
                (
                    [
                        "--camera-mode",
                        "spectator-overlay",
                        "--overlay-bundle",
                        os.fspath(bundle),
                        "--camera-receipt",
                        os.fspath(bundle / "new.json"),
                    ],
                    os.fspath(bundle / "new.json"),
                ),
                (
                    ["--camera-receipt", os.fspath(active / "receipt.json")],
                    os.fspath(active / "receipt.json"),
                ),
            )
            for arguments, protected_path in cases:
                with self.subTest(path=protected_path):
                    result = subprocess.run(
                        [
                            "/bin/bash",
                            os.fspath(launcher),
                            "--matrix-root",
                            os.fspath(matrix),
                            "--trace",
                            os.fspath(trace),
                            *arguments,
                        ],
                        text=True,
                        capture_output=True,
                        check=False,
                    )
                    self.assertEqual(result.returncode, 2)
                    self.assertIn("protected camera input", result.stderr)
                    self.assertEqual(contract.read_bytes(), contract_bytes)
                    self.assertEqual(bundle_file.read_bytes(), b"protected-pak")
                    self.assertFalse((bundle / "new.json").exists())
                    self.assertFalse(active.exists())


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
                        "schema_id": "matrix.physics_trace_replay.summary.v2",
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
            camera_payload = {
                "schema_id": "matrix.scene6_camera_receipt.v1",
                "mode": "robot",
                "requested_view_class": "MujocoSim_Custom_C",
                "spring_arm_cm": 180.0,
                "ue_exec_cmds": (
                    "t.MaxFPS 25,"
                    "set Engine.SpringArmComponent TargetArmLength 180,"
                    "viewclass MujocoSim_Custom_C"
                ),
                "camera_commands": [
                    "set Engine.SpringArmComponent TargetArmLength 180",
                    "viewclass MujocoSim_Custom_C",
                ],
                "overlay": None,
                "camera_ready": None,
                "created_unix_ns": 1,
            }
            camera_receipt = root / "camera.json"
            camera_receipt.write_text(json.dumps(camera_payload), encoding="utf-8")
            camera_binding = {
                "schema_id": "matrix.physics_trace_replay.camera_binding.v1",
                "path": str(camera_receipt.resolve()),
                "sha256": POSTFLIGHT._sha256(camera_receipt),
                "size_bytes": camera_receipt.stat().st_size,
                "receipt_schema_id": "matrix.scene6_camera_receipt.v1",
            }
            summary_payload = json.loads(summary.read_text(encoding="utf-8"))
            summary_payload["camera_receipt"] = camera_binding
            summary.write_text(json.dumps(summary_payload), encoding="utf-8")
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
                                "schema_id": "matrix.physics_trace_replay.status.v2",
                                "active_lowcmd": True,
                                "active_lowcmd_semantics": (
                                    "legacy_recorder_readiness_gate_no_dds_lowcmd"
                                ),
                                "dds_lowcmd_active": False,
                                "camera_receipt": camera_binding,
                            },
                            "after": {
                                "schema_id": "matrix.physics_trace_replay.status.v2",
                                "active_lowcmd": False,
                                "active_lowcmd_semantics": (
                                    "legacy_recorder_readiness_gate_no_dds_lowcmd"
                                ),
                                "dds_lowcmd_active": False,
                                "completed": True,
                                "passed": True,
                                "camera_receipt": camera_binding,
                            },
                        },
                        "launcher": {
                            "return_code": 0,
                            "stopped_by_recorder": False,
                        },
                        "matrix_scene6_extension_schema": (
                            "matrix.scene6_video_metadata.v2"
                        ),
                        "matrix_scene6_camera": camera_payload,
                    }
                ),
                encoding="utf-8",
            )

            receipt = POSTFLIGHT.verify(
                output=output,
                metadata_path=metadata,
                summary_path=summary,
                restore_path=restore,
                camera_receipt_path=camera_receipt,
                matrix_root=matrix,
            )
            self.assertTrue(receipt["passed"])
            self.assertEqual(receipt["camera"]["mode"], "robot")

            metadata_payload = json.loads(metadata.read_text(encoding="utf-8"))
            forged_camera = dict(camera_payload)
            forged_camera["created_unix_ns"] = 2
            camera_receipt.write_text(json.dumps(forged_camera), encoding="utf-8")
            metadata_payload["matrix_scene6_camera"] = forged_camera
            metadata.write_text(json.dumps(metadata_payload), encoding="utf-8")
            with self.assertRaisesRegex(
                POSTFLIGHT.PostflightError, "summary.*camera receipt"
            ):
                POSTFLIGHT.verify(
                    output=output,
                    metadata_path=metadata,
                    summary_path=summary,
                    restore_path=restore,
                    camera_receipt_path=camera_receipt,
                    matrix_root=matrix,
                )
            camera_receipt.write_text(json.dumps(camera_payload), encoding="utf-8")
            metadata_payload["matrix_scene6_camera"] = camera_payload
            metadata.write_text(json.dumps(metadata_payload), encoding="utf-8")

            camera_provenance = metadata_payload.pop("matrix_scene6_camera")
            metadata.write_text(json.dumps(metadata_payload), encoding="utf-8")
            with self.assertRaisesRegex(
                POSTFLIGHT.PostflightError, "camera receipt"
            ):
                POSTFLIGHT.verify(
                    output=output,
                    metadata_path=metadata,
                    summary_path=summary,
                    restore_path=restore,
                    camera_receipt_path=camera_receipt,
                    matrix_root=matrix,
                )
            metadata_payload["matrix_scene6_camera"] = camera_provenance
            metadata.write_text(json.dumps(metadata_payload), encoding="utf-8")

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
                    camera_receipt_path=camera_receipt,
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
                    camera_receipt_path=camera_receipt,
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
                    camera_receipt_path=camera_receipt,
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
                    camera_receipt_path=camera_receipt,
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
                    camera_receipt_path=camera_receipt,
                    matrix_root=matrix,
                )


if __name__ == "__main__":
    unittest.main()
