from __future__ import annotations

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
PREPARE = load_module(
    "prepare_matrix_scene6_replay_trace",
    SCRIPTS / "prepare_matrix_scene6_replay_trace.py",
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, sort_keys=True) + "\n", encoding="utf-8")


def write_fixture(root: Path) -> dict[str, Path]:
    model_root = root / "model"
    model_root.mkdir(parents=True)
    robot = model_root / "g1_29dof_dex3.scene6.xml"
    joints = "\n".join(
        f'<joint name="joint_{index}" type="hinge" />' for index in range(43)
    )
    motors = "\n".join(
        f'<motor name="motor_{index}" joint="joint_{index}" />'
        for index in range(43)
    )
    robot.write_text(
        f"""<mujoco model="G1 Dex3 Scene6">
  <worldbody>
    <body name="pelvis"><joint name="floating_base_joint" type="free" />
      {joints}
    </body>
    <body name="pick_cube"><freejoint name="pick_cube_joint" /></body>
  </worldbody>
  <actuator>{motors}</actuator>
</mujoco>
""",
        encoding="utf-8",
    )
    scene = model_root / "scene6_house_task.xml"
    scene.write_text(
        f"""<mujoco model="Scene6">
  <include file="{robot.name}" />
  <worldbody><geom name="worktop" type="box" size="1 1 0.1" /></worldbody>
</mujoco>
""",
        encoding="utf-8",
    )

    phases = (
        "world_ready",
        "navigation",
        "dock_arm_clearance",
        "dock_with_pregrasp",
        "assisted_stance_settle",
        "manipulation_anchor_active",
        "pick_place_contact_stabilized",
        "contact_validated",
        "grasp_stabilizer_active",
        "cube_supported_on_worktop",
        "grasp_stabilizer_released",
    )
    session_id = "world_fixture"
    world_id = "matrix_home_fixture"
    frames = [
        {
            "step": index + 1,
            "time_s": (index + 1) * PREPARE.PHYSICS_TIMESTEP_S,
            "qpos": [float(index)] + [0.0] * 56,
            "qvel": [0.0] * 55,
            "ctrl": [0.0] * 43,
            "controller_phase": phase,
        }
        for index, phase in enumerate(phases)
    ]
    trace_payload = {
        "schema_id": REPLAY.TRACE_SCHEMA,
        "physics_trace_id": "trace_fixture_v2",
        "physics_backend": "mujoco",
        "model_path": f"/tmp/source/{scene.name}",
        "world_instance_id": world_id,
        "simulation_session_id": session_id,
        "persistent_world_state": True,
        "status": "succeeded",
        "control": {
            "controller": PREPARE.CONTROLLER,
            "mode": PREPARE.CONTROL_MODE,
            "balance_assist": PREPARE.BALANCE_ASSISTANCE,
            "locomotion_policy_id": "amo_fixture",
        },
        "transitions": [
            {
                "phase": phase,
                "time_s": index * PREPARE.PHYSICS_TIMESTEP_S,
                "frame_index": index,
                "simulation_session_id": session_id,
                "world_instance_id": world_id,
            }
            for index, phase in enumerate(phases)
        ],
        "frames": frames,
        "scene_context": {
            "scene_id": PREPARE.SCENE_ID,
            "scene_number": PREPARE.SCENE_NUMBER,
            "environment_ref": PREPARE.ENVIRONMENT_REF,
            "control_sequence": list(PREPARE.CONTROL_SEQUENCE),
        },
    }
    trace = root / "physics-trace.source.json"
    write_json(trace, trace_payload)

    stage_base = {
        "status": "success",
        "simulation_session_id": session_id,
        "world_instance_id": world_id,
    }
    summary_payload = {
        "schema_id": PREPARE.SUMMARY_SCHEMA,
        "status": "succeeded",
        "trace": "/tmp/source/physics-trace.json",
        "frames": len(frames),
        "results": {
            "navigation": {
                **stage_base,
                "control": {
                    "policy_id": "amo_fixture",
                    "balance_assist": PREPARE.BALANCE_ASSISTANCE,
                },
            },
            "docking": {**stage_base, "locomotion_policy_id": "amo_fixture"},
            "handover": {
                **stage_base,
                "assisted": True,
                "assistance_mode": PREPARE.HANDOVER_ASSISTANCE,
            },
            "manipulation": {
                **stage_base,
                "assisted": True,
                "assistance_mode": PREPARE.GRASP_ASSISTANCE,
                "phases": {
                    "contact_validated": True,
                    "lifted": True,
                    "moved_to_target": True,
                    "released": True,
                },
            },
        },
    }
    summary = root / "summary.source.json"
    write_json(summary, summary_payload)

    video_frames = len(frames) + PREPARE.VIDEO_FINAL_HOLD_FRAMES
    validation_payload = {
        "schema_id": PREPARE.VALIDATION_SCHEMA,
        "status": "passed",
        "scene": PREPARE.SCENE_ID,
        "source_trace": {
            "trace_id": trace_payload["physics_trace_id"],
            "physics_frames": len(frames),
            "status": "succeeded",
        },
        "control_sequence": list(PREPARE.CONTROL_SEQUENCE),
        "assistance": {
            "used": True,
            "balance": PREPARE.BALANCE_ASSISTANCE,
            "handover": PREPARE.HANDOVER_ASSISTANCE,
            "grasp": PREPARE.GRASP_ASSISTANCE,
        },
        "task_result": {
            "validated_digit_contacts": list(PREPARE.EXPECTED_CONTACTS),
            "cube_supported_before_stabilizer_release": True,
            "cube_supported_after_hand_opening": True,
            "stabilizer_active_when_hand_opened": False,
        },
        "video": {
            "fps": PREPARE.SAMPLE_FPS,
            "frames": video_frames,
            "duration_s": video_frames / PREPARE.SAMPLE_FPS,
        },
    }
    validation = root / "validation.json"
    write_json(validation, validation_payload)
    return {
        "source_trace": trace,
        "source_summary": summary,
        "validation": validation,
        "scene_model": scene,
        "render_robot_model": robot,
    }


def prepare(paths: dict[str, Path], output: Path, receipt: Path, **kwargs):
    return PREPARE.prepare_projection(
        source_trace_path=paths["source_trace"],
        source_summary_path=paths["source_summary"],
        validation_path=paths["validation"],
        scene_model_path=paths["scene_model"],
        render_robot_model_path=paths["render_robot_model"],
        output_trace_path=output,
        receipt_path=receipt,
        **kwargs,
    )


class Scene6ReplayTraceProjectionTest(unittest.TestCase):
    def test_projects_deterministically_and_preserves_frames(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            paths = write_fixture(root)
            source_before = paths["source_trace"].read_bytes()
            output = root / "replay-trace.json"
            receipt_path = root / "projection-receipt.json"

            receipt = prepare(paths, output, receipt_path)

            self.assertEqual(paths["source_trace"].read_bytes(), source_before)
            source_payload = json.loads(source_before)
            projected = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(projected["frames"], source_payload["frames"])
            self.assertEqual(
                projected["dimensions"], {"nq": 57, "nv": 55, "nu": 43}
            )
            self.assertEqual(projected["physics_timestep_s"], 0.002)
            self.assertEqual(projected["sample_fps"], 25.0)
            self.assertEqual(
                projected["scene_context"]["map_name"], "/Game/Maps/HouseWorld"
            )
            self.assertEqual(
                projected["scene_context"]["manipulation_assistance"],
                PREPARE.REPLAY_ASSISTANCE_DISCLOSURE,
            )
            validated = REPLAY.validate_trace(output)
            self.assertEqual(len(validated.frames), len(source_payload["frames"]))
            self.assertEqual(
                receipt["outputs"]["replay_trace"]["sha256"], sha256(output)
            )
            self.assertEqual(
                receipt["inputs"]["source_trace"]["sha256"],
                sha256(paths["source_trace"]),
            )
            self.assertEqual(
                receipt["inputs"]["source_trace"]["frames_sha256"],
                receipt["outputs"]["replay_trace"]["frames_sha256"],
            )

            second_output = root / "replay-trace-second.json"
            second_receipt = root / "projection-receipt-second.json"
            prepare(paths, second_output, second_receipt)
            self.assertEqual(output.read_bytes(), second_output.read_bytes())

    def test_cross_artifact_mismatches_fail_closed(self) -> None:
        mutations = (
            (
                "source_summary",
                lambda payload: payload.update(frames=payload["frames"] + 1),
                "summary frame count",
            ),
            (
                "validation",
                lambda payload: payload["source_trace"].update(trace_id="other"),
                "trace_id differs",
            ),
            (
                "source_trace",
                lambda payload: payload["scene_context"].update(scene_number=21),
                "scene_context.scene_number",
            ),
            (
                "source_trace",
                lambda payload: payload["control"].update(controller="other"),
                "controller/mode",
            ),
            (
                "validation",
                lambda payload: payload.update(status="failed"),
                "status must be passed",
            ),
        )
        for filename, mutate, message in mutations:
            with self.subTest(
                message=message
            ), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                paths = write_fixture(root)
                payload = json.loads(paths[filename].read_text(encoding="utf-8"))
                mutate(payload)
                write_json(paths[filename], payload)
                output = root / "out.json"
                receipt = root / "receipt.json"
                with self.assertRaisesRegex(PREPARE.ProjectionError, message):
                    prepare(paths, output, receipt)
                self.assertFalse(output.exists())
                self.assertFalse(receipt.exists())

    def test_vectors_sampling_transitions_and_video_sampling_are_strict(self) -> None:
        mutations = (
            (
                "source_trace",
                lambda payload: payload["frames"][0].update(qpos=[0.0] * 56),
                "shape must be 57",
            ),
            (
                "source_trace",
                lambda payload: payload["frames"][1].update(time_s=0.003),
                "align to",
            ),
            (
                "source_trace",
                lambda payload: payload.update(
                    transitions=[
                        item
                        for item in payload["transitions"]
                        if item["phase"] != "contact_validated"
                    ]
                ),
                "missing ordered task transition",
            ),
            (
                "source_trace",
                lambda payload: payload.update(
                    transitions=[
                        item
                        for item in payload["transitions"]
                        if item["phase"] != "dock_arm_clearance"
                    ]
                ),
                "missing the v7 dock_arm_clearance transition chain",
            ),
            (
                "source_trace",
                lambda payload: (
                    payload["transitions"][2].update(phase="dock_with_pregrasp"),
                    payload["transitions"][3].update(phase="dock_arm_clearance"),
                ),
                "v7 arm-clearance transition order",
            ),
            (
                "validation",
                lambda payload: payload["video"].update(fps=24.0),
                "video fps must be 25",
            ),
        )
        for filename, mutate, message in mutations:
            with self.subTest(
                message=message
            ), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                paths = write_fixture(root)
                payload = json.loads(paths[filename].read_text(encoding="utf-8"))
                mutate(payload)
                write_json(paths[filename], payload)
                with self.assertRaisesRegex(PREPARE.ProjectionError, message):
                    prepare(paths, root / "out.json", root / "receipt.json")

    def test_regular_models_expected_hashes_and_atomic_self_check(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            paths = write_fixture(root)
            robot_link = root / "robot-link.xml"
            robot_link.symlink_to(paths["render_robot_model"])
            linked = dict(paths, render_robot_model=robot_link)
            with self.assertRaisesRegex(
                PREPARE.ProjectionError, "must not be a symlink"
            ):
                prepare(linked, root / "linked-out.json", root / "linked-receipt.json")

            with self.assertRaisesRegex(PREPARE.ProjectionError, "SHA256 mismatch"):
                prepare(
                    paths,
                    root / "hash-out.json",
                    root / "hash-receipt.json",
                    expected_hashes={"source_trace": "0" * 64},
                )

            output = root / "self-check-out.json"
            receipt = root / "self-check-receipt.json"
            with mock.patch.object(
                PREPARE.replay,
                "validate_trace",
                side_effect=REPLAY.TraceValidationError("injected self-check failure"),
            ), self.assertRaisesRegex(PREPARE.ProjectionError, "self-check failed"):
                prepare(paths, output, receipt)
            self.assertFalse(output.exists())
            self.assertFalse(receipt.exists())
            self.assertEqual(list(root.glob(".*.tmp")), [])

    def test_cli_accepts_real_artifact_style_paths_and_hash_pins(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            paths = write_fixture(root)
            output = root / "bundle" / "physics-trace.matrix-replay.json"
            receipt = root / "bundle" / "projection-receipt.json"
            command = [
                sys.executable,
                os.fspath(SCRIPTS / "prepare_matrix_scene6_replay_trace.py"),
                "--source-trace",
                os.fspath(paths["source_trace"]),
                "--source-summary",
                os.fspath(paths["source_summary"]),
                "--validation",
                os.fspath(paths["validation"]),
                "--scene-model",
                os.fspath(paths["scene_model"]),
                "--render-robot-model",
                os.fspath(paths["render_robot_model"]),
                "--output-trace",
                os.fspath(output),
                "--receipt",
                os.fspath(receipt),
                "--expected-source-trace-sha256",
                sha256(paths["source_trace"]),
            ]
            completed = subprocess.run(
                command, text=True, capture_output=True, check=False
            )

            self.assertEqual(completed.returncode, 0, completed.stderr)
            stdout = json.loads(completed.stdout)
            self.assertTrue(stdout["passed"])
            self.assertEqual(stdout["replay_trace"]["sha256"], sha256(output))
            self.assertEqual(stdout["receipt"]["sha256"], sha256(receipt))


if __name__ == "__main__":
    unittest.main()
