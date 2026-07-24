#!/usr/bin/env python3
"""Project a validated TwinBot Scene6 source trace into Matrix replay form.

The TwinBot trace is the physics authority.  This tool only adds the metadata
and explicit model bindings required by Matrix's UE trace replayer; it never
edits, resamples, or regenerates a source frame.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import stat
import sys
import tempfile
from typing import Any, Mapping
import xml.etree.ElementTree as ET

import replay_matrix_physics_trace as replay


SUMMARY_SCHEMA = "twinbot.scene6_arm_clearance_demo.v7"
VALIDATION_SCHEMA = "twinbot.scene6.render_validation.v1"
PROJECTION_SCHEMA = "matrix.scene6_replay_trace_projection.v1"
RECEIPT_SCHEMA = "matrix.scene6_replay_trace_projection.receipt.v1"

SCENE_ID = "matrix_house_scene6"
SCENE_NUMBER = 6
ENVIRONMENT_REF = "matrix://scene/6/HouseWorld"
MAP_NAME = "/Game/Maps/HouseWorld"
CONTROLLER = "behavior_tree_controller_switching"
CONTROL_MODE = "persistent_matrix_home_world_v0"
BALANCE_ASSISTANCE = "pelvis_height_attitude_external_wrench"
HANDOVER_ASSISTANCE = "continuous_stance_pose_settle"
GRASP_ASSISTANCE = "contact_validated_grasp_stabilization"
REPLAY_ASSISTANCE_DISCLOSURE = (
    "contact_gated_wrist_cube_weld_and_anchored_stance"
)
CONTROL_SEQUENCE = (
    "navigation_natural_arms",
    "raise_arm_at_staging",
    "dock_with_raised_arm",
    "smooth_handover",
    "pick_place",
)
SUMMARY_STAGES = ("navigation", "docking", "handover", "manipulation")
EXPECTED_CONTACTS = (
    "right_hand_middle_0_link",
    "right_hand_thumb_2_link",
)
PHYSICS_TIMESTEP_S = 0.002
SAMPLE_FPS = 25.0
SAMPLE_PERIOD_S = 1.0 / SAMPLE_FPS
VIDEO_FINAL_HOLD_FRAMES = 25
EXPECTED_DIMS = (57, 55, 43)

MAX_TRACE_BYTES = 512 * 1024 * 1024
MAX_JSON_BYTES = 4 * 1024 * 1024
MAX_MODEL_BYTES = 64 * 1024 * 1024


class ProjectionError(ValueError):
    """Raised before a replay projection can be safely published."""


@dataclass(frozen=True)
class Artifact:
    label: str
    path: Path
    contents: bytes
    sha256: str
    size_bytes: int
    device: int
    inode: int
    mtime_ns: int
    max_bytes: int

    def binding(self, **extra: Any) -> dict[str, Any]:
        return {
            "path": os.fspath(self.path),
            "sha256": self.sha256,
            "size_bytes": self.size_bytes,
            **extra,
        }


def _reject_constant(value: str) -> None:
    raise ProjectionError(f"non-finite JSON number is forbidden: {value}")


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ProjectionError(f"duplicate JSON key is forbidden: {key}")
        result[key] = value
    return result


def _read_regular(path: Path, *, label: str, max_bytes: int) -> Artifact:
    supplied = path.expanduser()
    if supplied.is_symlink():
        raise ProjectionError(f"{label} must not be a symlink: {supplied}")
    try:
        resolved = supplied.resolve(strict=True)
    except OSError as exc:
        raise ProjectionError(f"cannot resolve {label}: {supplied}: {exc}") from exc

    flags = os.O_RDONLY | os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        descriptor = os.open(resolved, flags)
    except OSError as exc:
        raise ProjectionError(f"cannot open {label}: {resolved}: {exc}") from exc
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode):
            raise ProjectionError(f"{label} must be a regular file: {resolved}")
        if before.st_size <= 0 or before.st_size > max_bytes:
            raise ProjectionError(
                f"{label} size must be in 1..{max_bytes} bytes, got {before.st_size}"
            )
        blocks: list[bytes] = []
        remaining = before.st_size
        while remaining:
            block = os.read(descriptor, min(1024 * 1024, remaining))
            if not block:
                raise ProjectionError(f"{label} was truncated while being read")
            blocks.append(block)
            remaining -= len(block)
        if os.read(descriptor, 1):
            raise ProjectionError(f"{label} grew while being read")
        after = os.fstat(descriptor)
    finally:
        os.close(descriptor)

    metadata_before = (
        before.st_dev,
        before.st_ino,
        before.st_size,
        before.st_mtime_ns,
    )
    metadata_after = (
        after.st_dev,
        after.st_ino,
        after.st_size,
        after.st_mtime_ns,
    )
    if metadata_before != metadata_after:
        raise ProjectionError(f"{label} changed while being read: {resolved}")
    try:
        current = os.stat(resolved, follow_symlinks=False)
    except OSError as exc:
        raise ProjectionError(f"cannot re-stat {label}: {resolved}: {exc}") from exc
    if (
        current.st_dev,
        current.st_ino,
        current.st_size,
        current.st_mtime_ns,
    ) != metadata_after:
        raise ProjectionError(f"{label} path changed while being read: {resolved}")

    contents = b"".join(blocks)
    return Artifact(
        label=label,
        path=resolved,
        contents=contents,
        sha256=hashlib.sha256(contents).hexdigest(),
        size_bytes=len(contents),
        device=after.st_dev,
        inode=after.st_ino,
        mtime_ns=after.st_mtime_ns,
        max_bytes=max_bytes,
    )


def _parse_json(artifact: Artifact) -> dict[str, Any]:
    try:
        text = artifact.contents.decode("utf-8")
        payload = json.loads(
            text,
            parse_constant=_reject_constant,
            object_pairs_hook=_unique_object,
        )
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ProjectionError(f"invalid {artifact.label} JSON: {exc}") from exc
    if not isinstance(payload, dict):
        raise ProjectionError(f"{artifact.label} root must be an object")
    return payload


def _mapping(value: Any, *, field: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ProjectionError(f"{field} must be an object")
    return value


def _nonempty_string(value: Any, *, field: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ProjectionError(f"{field} must be a non-empty string")
    return value


def _finite(value: Any, *, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProjectionError(f"{field} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ProjectionError(f"{field} must be finite")
    return result


def _exact_integer(value: Any, *, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ProjectionError(f"{field} must be an integer")
    return value


def _vector(value: Any, *, length: int, field: str) -> None:
    if not isinstance(value, list) or len(value) != length:
        actual = len(value) if isinstance(value, list) else type(value).__name__
        raise ProjectionError(f"{field} shape must be {length}, got {actual}")
    for index, item in enumerate(value):
        _finite(item, field=f"{field}[{index}]")


def _aligned_ticks(value: float, *, field: str) -> int:
    ticks = round(value / PHYSICS_TIMESTEP_S)
    if ticks < 0 or not math.isclose(
        value,
        ticks * PHYSICS_TIMESTEP_S,
        rel_tol=0.0,
        abs_tol=1e-9,
    ):
        raise ProjectionError(
            f"{field} must align to the {PHYSICS_TIMESTEP_S}s physics timestep"
        )
    return ticks


def _canonical_bytes(payload: Any) -> bytes:
    try:
        return (
            json.dumps(
                payload,
                ensure_ascii=False,
                allow_nan=False,
                sort_keys=True,
                separators=(",", ":"),
            )
            + "\n"
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise ProjectionError(f"cannot encode deterministic JSON: {exc}") from exc


def _pretty_json_bytes(payload: Any) -> bytes:
    try:
        return (
            json.dumps(
                payload,
                ensure_ascii=False,
                allow_nan=False,
                indent=2,
                sort_keys=True,
            )
            + "\n"
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise ProjectionError(f"cannot encode deterministic JSON: {exc}") from exc


def _frame_digest(frames: Any) -> str:
    return hashlib.sha256(_canonical_bytes(frames)).hexdigest()


def _require_absent_or_equal(
    mapping: Mapping[str, Any], key: str, expected: Any, *, field: str
) -> None:
    if key in mapping and mapping[key] != expected:
        raise ProjectionError(
            f"{field}.{key} conflicts with replay projection: "
            f"expected {expected!r}, got {mapping[key]!r}"
        )


def _validate_frames(trace: dict[str, Any]) -> tuple[list[Any], float, float]:
    frames = trace.get("frames")
    if not isinstance(frames, list) or not frames:
        raise ProjectionError("source trace frames must be a non-empty array")
    previous_time: float | None = None
    previous_ticks: int | None = None
    for index, raw_frame in enumerate(frames):
        frame = _mapping(raw_frame, field=f"frames[{index}]")
        step = _exact_integer(frame.get("step"), field=f"frames[{index}].step")
        if step < 0:
            raise ProjectionError(f"frames[{index}].step must be non-negative")
        frame_time = _finite(frame.get("time_s"), field=f"frames[{index}].time_s")
        ticks = _aligned_ticks(frame_time, field=f"frames[{index}].time_s")
        if index == 0 and ticks != 1:
            raise ProjectionError(
                "the first source frame must be physics tick 1 (0.002s)"
            )
        if previous_time is not None:
            assert previous_ticks is not None
            if frame_time <= previous_time:
                raise ProjectionError(f"frames[{index}].time_s must increase strictly")
            tick_delta = ticks - previous_ticks
            if tick_delta < 1 or tick_delta > round(
                SAMPLE_PERIOD_S / PHYSICS_TIMESTEP_S
            ):
                raise ProjectionError(
                    f"frames[{index}].time_s violates the 25fps/0.002s sampling bound"
                )
        previous_time = frame_time
        previous_ticks = ticks
        for vector_name, length in zip(("qpos", "qvel", "ctrl"), EXPECTED_DIMS):
            _vector(
                frame.get(vector_name),
                length=length,
                field=f"frames[{index}].{vector_name}",
            )
        _nonempty_string(
            frame.get("controller_phase"),
            field=f"frames[{index}].controller_phase",
        )
    assert previous_time is not None
    return frames, float(frames[0]["time_s"]), previous_time


def _validate_transitions(
    trace: dict[str, Any], frames: list[Any], *, session_id: str, world_id: str
) -> tuple[str, ...]:
    transitions = trace.get("transitions")
    if not isinstance(transitions, list) or not transitions:
        raise ProjectionError("source trace transitions must be a non-empty array")
    phases: list[str] = []
    phase_bindings: dict[str, tuple[float, int]] = {}
    previous_time: float | None = None
    previous_index: int | None = None
    for index, raw_transition in enumerate(transitions):
        transition = _mapping(raw_transition, field=f"transitions[{index}]")
        phase = _nonempty_string(
            transition.get("phase"), field=f"transitions[{index}].phase"
        )
        if phase in phases:
            raise ProjectionError(f"duplicate transition phase is forbidden: {phase}")
        transition_time = _finite(
            transition.get("time_s"), field=f"transitions[{index}].time_s"
        )
        _aligned_ticks(transition_time, field=f"transitions[{index}].time_s")
        frame_index = _exact_integer(
            transition.get("frame_index"),
            field=f"transitions[{index}].frame_index",
        )
        if frame_index < 0 or frame_index >= len(frames):
            raise ProjectionError(f"transitions[{index}].frame_index is out of range")
        if previous_time is not None and transition_time < previous_time:
            raise ProjectionError("source transition time regressed")
        if previous_index is not None and frame_index < previous_index:
            raise ProjectionError("source transition frame index regressed")
        frame_time = _finite(
            frames[frame_index].get("time_s"),
            field=f"frames[{frame_index}].time_s",
        )
        if not math.isclose(
            frame_time - transition_time,
            PHYSICS_TIMESTEP_S,
            rel_tol=0.0,
            abs_tol=1e-9,
        ):
            raise ProjectionError(
                f"transitions[{index}] does not bind the first post-transition frame"
            )
        if transition.get("simulation_session_id") != session_id:
            raise ProjectionError(
                f"transitions[{index}].simulation_session_id differs from the trace"
            )
        if transition.get("world_instance_id") != world_id:
            raise ProjectionError(
                f"transitions[{index}].world_instance_id differs from the trace"
            )
        phases.append(phase)
        phase_bindings[phase] = (transition_time, frame_index)
        previous_time = transition_time
        previous_index = frame_index

    arm_clearance_sequence = (
        "world_ready",
        "navigation",
        "dock_arm_clearance",
        "dock_with_pregrasp",
    )
    try:
        arm_clearance_indices = [
            phases.index(phase) for phase in arm_clearance_sequence
        ]
    except ValueError as exc:
        raise ProjectionError(
            "source trace is missing the v7 dock_arm_clearance transition chain"
        ) from exc
    if arm_clearance_indices != sorted(arm_clearance_indices) or len(
        set(arm_clearance_indices)
    ) != len(arm_clearance_indices):
        raise ProjectionError(
            "source trace v7 arm-clearance transition order is invalid"
        )
    world_binding = phase_bindings["world_ready"]
    navigation_binding = phase_bindings["navigation"]
    clearance_binding = phase_bindings["dock_arm_clearance"]
    docking_binding = phase_bindings["dock_with_pregrasp"]
    if not (
        clearance_binding[0] > max(world_binding[0], navigation_binding[0])
        and clearance_binding[1] > max(world_binding[1], navigation_binding[1])
        and docking_binding[0] > clearance_binding[0]
        and docking_binding[1] > clearance_binding[1]
    ):
        raise ProjectionError(
            "source trace dock_arm_clearance must be strictly between navigation "
            "and dock_with_pregrasp in time and frame index"
        )

    search_from = 0
    for required in replay.REQUIRED_TRANSITION_SUBSEQUENCE:
        try:
            found = phases.index(required, search_from)
        except ValueError as exc:
            raise ProjectionError(
                f"source trace is missing ordered task transition: {required}"
            ) from exc
        search_from = found + 1
    if "manipulation_anchor_active" not in phases:
        raise ProjectionError("source trace does not disclose manipulation anchoring")
    return tuple(phases)


def _validate_trace_source(trace: dict[str, Any]) -> dict[str, Any]:
    if trace.get("schema_id") != replay.TRACE_SCHEMA:
        raise ProjectionError(f"source trace schema must be {replay.TRACE_SCHEMA}")
    if trace.get("physics_backend") != "mujoco":
        raise ProjectionError("source trace physics_backend must be 'mujoco'")
    if trace.get("persistent_world_state") is not True:
        raise ProjectionError("source trace persistent_world_state must be true")
    if trace.get("status") != "succeeded":
        raise ProjectionError("source trace status must be succeeded")
    if "matrix_replay_projection" in trace:
        raise ProjectionError("source trace is already a Matrix replay projection")

    trace_id = _nonempty_string(
        trace.get("physics_trace_id"), field="physics_trace_id"
    )
    session_id = _nonempty_string(
        trace.get("simulation_session_id"), field="simulation_session_id"
    )
    world_id = _nonempty_string(
        trace.get("world_instance_id"), field="world_instance_id"
    )
    control = _mapping(trace.get("control"), field="control")
    if control.get("controller") != CONTROLLER or control.get("mode") != CONTROL_MODE:
        raise ProjectionError(
            "source trace controller/mode is not the Scene6 task chain"
        )
    if control.get("balance_assist") != BALANCE_ASSISTANCE:
        raise ProjectionError("source trace balance assistance is unexpected")
    policy_id = _nonempty_string(
        control.get("locomotion_policy_id"), field="control.locomotion_policy_id"
    )

    scene = _mapping(trace.get("scene_context"), field="scene_context")
    expected_scene = {
        "scene_id": SCENE_ID,
        "scene_number": SCENE_NUMBER,
        "environment_ref": ENVIRONMENT_REF,
    }
    for key, expected in expected_scene.items():
        if scene.get(key) != expected:
            raise ProjectionError(
                f"scene_context.{key} must be {expected!r}, got {scene.get(key)!r}"
            )
    if tuple(scene.get("control_sequence", ())) != CONTROL_SEQUENCE:
        raise ProjectionError("scene_context.control_sequence is unexpected")
    for key, expected in (
        ("map_name", MAP_NAME),
        ("physics_execution", replay.PHYSICS_EXECUTION),
        ("intended_render_mode", replay.RENDER_MODE),
        ("manipulation_assistance", REPLAY_ASSISTANCE_DISCLOSURE),
    ):
        _require_absent_or_equal(scene, key, expected, field="scene_context")
    _require_absent_or_equal(
        trace,
        "dimensions",
        {"nq": EXPECTED_DIMS[0], "nv": EXPECTED_DIMS[1], "nu": EXPECTED_DIMS[2]},
        field="trace",
    )
    _require_absent_or_equal(
        trace, "physics_timestep_s", PHYSICS_TIMESTEP_S, field="trace"
    )
    _require_absent_or_equal(trace, "sample_fps", SAMPLE_FPS, field="trace")

    frames, first_time, last_time = _validate_frames(trace)
    phases = _validate_transitions(
        trace, frames, session_id=session_id, world_id=world_id
    )
    return {
        "trace_id": trace_id,
        "session_id": session_id,
        "world_id": world_id,
        "policy_id": policy_id,
        "control": control,
        "scene": scene,
        "frames": frames,
        "frame_count": len(frames),
        "frame_sha256": _frame_digest(frames),
        "first_time_s": first_time,
        "last_time_s": last_time,
        "transition_phases": phases,
    }


def _validate_summary(summary: dict[str, Any], identity: dict[str, Any]) -> None:
    if summary.get("schema_id") != SUMMARY_SCHEMA:
        raise ProjectionError(f"source summary schema must be {SUMMARY_SCHEMA}")
    if summary.get("status") != "succeeded":
        raise ProjectionError("source summary status must be succeeded")
    if _exact_integer(summary.get("frames"), field="summary.frames") != identity[
        "frame_count"
    ]:
        raise ProjectionError("source summary frame count differs from the trace")
    _nonempty_string(summary.get("trace"), field="summary.trace")
    results = _mapping(summary.get("results"), field="summary.results")
    for stage_name in SUMMARY_STAGES:
        stage = _mapping(results.get(stage_name), field=f"summary.results.{stage_name}")
        if stage.get("status") != "success":
            raise ProjectionError(f"source summary {stage_name} did not succeed")
        for key, expected in (
            ("simulation_session_id", identity["session_id"]),
            ("world_instance_id", identity["world_id"]),
        ):
            if stage.get(key) != expected:
                raise ProjectionError(
                    f"source summary {stage_name}.{key} differs from the trace"
                )

    navigation_control = _mapping(
        results["navigation"].get("control"),
        field="summary.results.navigation.control",
    )
    if navigation_control.get("policy_id") != identity["policy_id"]:
        raise ProjectionError("source summary locomotion policy differs from the trace")
    if navigation_control.get("balance_assist") != BALANCE_ASSISTANCE:
        raise ProjectionError(
            "source summary balance assistance differs from the trace"
        )
    if results["docking"].get("locomotion_policy_id") != identity["policy_id"]:
        raise ProjectionError("source summary docking policy differs from the trace")
    if (
        results["handover"].get("assisted") is not True
        or results["handover"].get("assistance_mode") != HANDOVER_ASSISTANCE
    ):
        raise ProjectionError("source summary handover assistance is unexpected")
    if (
        results["manipulation"].get("assisted") is not True
        or results["manipulation"].get("assistance_mode") != GRASP_ASSISTANCE
    ):
        raise ProjectionError("source summary grasp assistance is unexpected")
    manipulation_phases = _mapping(
        results["manipulation"].get("phases"),
        field="summary.results.manipulation.phases",
    )
    for phase in ("contact_validated", "lifted", "moved_to_target", "released"):
        if manipulation_phases.get(phase) is not True:
            raise ProjectionError(f"source summary manipulation phase failed: {phase}")


def _validate_group_validation(
    validation: dict[str, Any], identity: dict[str, Any]
) -> None:
    if validation.get("schema_id") != VALIDATION_SCHEMA:
        raise ProjectionError(f"group validation schema must be {VALIDATION_SCHEMA}")
    if validation.get("status") != "passed":
        raise ProjectionError("group validation status must be passed")
    if validation.get("scene") != SCENE_ID:
        raise ProjectionError("group validation scene differs from the trace")
    source = _mapping(validation.get("source_trace"), field="validation.source_trace")
    expected_source = {
        "trace_id": identity["trace_id"],
        "physics_frames": identity["frame_count"],
        "status": "succeeded",
    }
    for key, expected in expected_source.items():
        if source.get(key) != expected:
            raise ProjectionError(
                f"group validation source_trace.{key} differs from the trace"
            )
    if tuple(validation.get("control_sequence", ())) != CONTROL_SEQUENCE:
        raise ProjectionError(
            "group validation control sequence differs from the trace"
        )

    assistance = _mapping(
        validation.get("assistance"), field="validation.assistance"
    )
    expected_assistance = {
        "used": True,
        "balance": BALANCE_ASSISTANCE,
        "handover": HANDOVER_ASSISTANCE,
        "grasp": GRASP_ASSISTANCE,
    }
    for key, expected in expected_assistance.items():
        if assistance.get(key) != expected:
            raise ProjectionError(f"group validation assistance.{key} is unexpected")

    task = _mapping(validation.get("task_result"), field="validation.task_result")
    if tuple(task.get("validated_digit_contacts", ())) != EXPECTED_CONTACTS:
        raise ProjectionError(
            "group validation lacks the expected opposing digit contacts"
        )
    for field, expected in (
        ("cube_supported_before_stabilizer_release", True),
        ("cube_supported_after_hand_opening", True),
        ("stabilizer_active_when_hand_opened", False),
    ):
        if task.get(field) is not expected:
            raise ProjectionError(f"group validation task_result.{field} is unexpected")

    video = _mapping(validation.get("video"), field="validation.video")
    fps = _finite(video.get("fps"), field="validation.video.fps")
    if not math.isclose(fps, SAMPLE_FPS, rel_tol=0.0, abs_tol=1e-9):
        raise ProjectionError("group validation video fps must be 25")
    video_frames = _exact_integer(
        video.get("frames"), field="validation.video.frames"
    )
    if video_frames != identity["frame_count"] + VIDEO_FINAL_HOLD_FRAMES:
        raise ProjectionError(
            "group validation video/source frame counts are inconsistent"
        )
    duration = _finite(video.get("duration_s"), field="validation.video.duration_s")
    if not math.isclose(
        duration, video_frames / SAMPLE_FPS, rel_tol=0.0, abs_tol=1e-9
    ):
        raise ProjectionError("group validation video duration is inconsistent")


def _parse_xml(artifact: Artifact) -> ET.Element:
    try:
        root = ET.fromstring(artifact.contents)
    except ET.ParseError as exc:
        raise ProjectionError(f"invalid {artifact.label} XML: {exc}") from exc
    if root.tag != "mujoco":
        raise ProjectionError(f"{artifact.label} root must be <mujoco>")
    return root


def _infer_robot_dimensions(root: ET.Element) -> tuple[int, int, int]:
    nq = 0
    nv = 0
    for joint in root.findall(".//worldbody//joint"):
        joint_type = joint.get("type", "hinge")
        if joint_type in ("hinge", "slide"):
            nq += 1
            nv += 1
        elif joint_type == "ball":
            nq += 4
            nv += 3
        elif joint_type == "free":
            nq += 7
            nv += 6
        else:
            raise ProjectionError(
                f"render robot model has unknown joint type: {joint_type}"
            )
    freejoints = root.findall(".//worldbody//freejoint")
    nq += 7 * len(freejoints)
    nv += 6 * len(freejoints)
    nu = sum(len(list(actuator)) for actuator in root.findall(".//actuator"))
    return nq, nv, nu


def _validate_models(
    trace: dict[str, Any], scene_model: Artifact, render_model: Artifact
) -> None:
    if scene_model.path == render_model.path:
        raise ProjectionError("scene and render robot models must be distinct files")
    configured_model = _nonempty_string(trace.get("model_path"), field="model_path")
    if Path(configured_model).name != scene_model.path.name:
        raise ProjectionError("source trace model basename differs from --scene-model")

    scene_root = _parse_xml(scene_model)
    render_root = _parse_xml(render_model)
    includes = [
        Path(include.get("file", "")).name
        for include in scene_root.findall(".//include")
    ]
    if includes.count(render_model.path.name) != 1:
        raise ProjectionError(
            "scene model must include the selected render robot model exactly once"
        )
    if scene_root.find('.//geom[@name="worktop"]') is None:
        raise ProjectionError("scene model is missing the Scene6 worktop")
    if render_root.find('.//body[@name="pick_cube"]') is None:
        raise ProjectionError("render robot model is missing pick_cube")
    dimensions = _infer_robot_dimensions(render_root)
    if dimensions != EXPECTED_DIMS:
        raise ProjectionError(
            "render robot model dimensions must be exactly 57/55/43, got "
            + "/".join(str(value) for value in dimensions)
        )
    if "render_robot_model_path" in trace:
        configured_render = _nonempty_string(
            trace.get("render_robot_model_path"), field="render_robot_model_path"
        )
        if Path(configured_render).name != render_model.path.name:
            raise ProjectionError(
                "source trace render model basename differs from --render-robot-model"
            )
    if (
        "render_robot_model_sha256" in trace
        and trace.get("render_robot_model_sha256") != render_model.sha256
    ):
        raise ProjectionError("source trace render robot model SHA256 conflicts")


def _validate_expected_hash(
    artifact: Artifact, expected: str | None
) -> None:
    if expected is None:
        return
    if len(expected) != 64 or any(
        character not in "0123456789abcdef" for character in expected
    ):
        raise ProjectionError(f"expected {artifact.label} SHA256 is invalid")
    if artifact.sha256 != expected:
        raise ProjectionError(
            f"{artifact.label} SHA256 mismatch: expected {expected}, "
            f"got {artifact.sha256}"
        )


def _assert_unchanged(artifact: Artifact) -> None:
    current = _read_regular(
        artifact.path, label=artifact.label, max_bytes=artifact.max_bytes
    )
    if (
        current.device,
        current.inode,
        current.size_bytes,
        current.mtime_ns,
        current.sha256,
    ) != (
        artifact.device,
        artifact.inode,
        artifact.size_bytes,
        artifact.mtime_ns,
        artifact.sha256,
    ):
        raise ProjectionError(f"{artifact.label} changed before projection publish")


def _destination(path: Path, *, label: str) -> Path:
    supplied = path.expanduser()
    if not supplied.is_absolute():
        supplied = Path.cwd() / supplied
    parent = supplied.parent.resolve(strict=False)
    destination = parent / supplied.name
    if not destination.name:
        raise ProjectionError(f"{label} path is invalid")
    if destination.is_symlink() or destination.exists():
        raise ProjectionError(
            f"{label} already exists; refusing to overwrite: {destination}"
        )
    return destination


def _write_temp(destination: Path, contents: bytes) -> Path:
    destination.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        dir=destination.parent, prefix=f".{destination.name}.", suffix=".tmp"
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(contents)
            stream.flush()
            os.fsync(stream.fileno())
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise
    return temporary


def _fsync_directory(path: Path) -> None:
    flags = os.O_RDONLY | os.O_CLOEXEC
    if hasattr(os, "O_DIRECTORY"):
        flags |= os.O_DIRECTORY
    descriptor = os.open(path, flags)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def prepare_projection(
    *,
    source_trace_path: Path,
    source_summary_path: Path,
    validation_path: Path,
    scene_model_path: Path,
    render_robot_model_path: Path,
    output_trace_path: Path,
    receipt_path: Path,
    expected_hashes: Mapping[str, str | None] | None = None,
) -> dict[str, Any]:
    """Validate all authorities and atomically publish a replay projection."""

    artifacts = {
        "source_trace": _read_regular(
            source_trace_path, label="source trace", max_bytes=MAX_TRACE_BYTES
        ),
        "source_summary": _read_regular(
            source_summary_path, label="source summary", max_bytes=MAX_JSON_BYTES
        ),
        "validation": _read_regular(
            validation_path, label="group validation", max_bytes=MAX_JSON_BYTES
        ),
        "scene_model": _read_regular(
            scene_model_path, label="scene model", max_bytes=MAX_MODEL_BYTES
        ),
        "render_robot_model": _read_regular(
            render_robot_model_path,
            label="render robot model",
            max_bytes=MAX_MODEL_BYTES,
        ),
    }
    expected_hashes = expected_hashes or {}
    for name, artifact in artifacts.items():
        _validate_expected_hash(artifact, expected_hashes.get(name))

    trace = _parse_json(artifacts["source_trace"])
    summary = _parse_json(artifacts["source_summary"])
    validation = _parse_json(artifacts["validation"])
    identity = _validate_trace_source(trace)
    _validate_summary(summary, identity)
    _validate_group_validation(validation, identity)
    _validate_models(
        trace, artifacts["scene_model"], artifacts["render_robot_model"]
    )

    output_trace = _destination(output_trace_path, label="output replay trace")
    output_receipt = _destination(receipt_path, label="projection receipt")
    if output_trace == output_receipt:
        raise ProjectionError("output replay trace and receipt paths must be distinct")
    input_paths = {artifact.path for artifact in artifacts.values()}
    if output_trace in input_paths or output_receipt in input_paths:
        raise ProjectionError("projection outputs must not replace an input artifact")

    dimensions = {
        "nq": EXPECTED_DIMS[0],
        "nv": EXPECTED_DIMS[1],
        "nu": EXPECTED_DIMS[2],
    }
    projection_inputs = {
        name: artifact.sha256 for name, artifact in sorted(artifacts.items())
    }
    projection = dict(trace)
    projected_scene = dict(identity["scene"])
    projected_scene.update(
        {
            "map_name": MAP_NAME,
            "physics_execution": replay.PHYSICS_EXECUTION,
            "intended_render_mode": replay.RENDER_MODE,
            "manipulation_assistance": REPLAY_ASSISTANCE_DISCLOSURE,
        }
    )
    projection.update(
        {
            "model_path": os.fspath(artifacts["scene_model"].path),
            "render_robot_model_path": os.fspath(
                artifacts["render_robot_model"].path
            ),
            "render_robot_model_sha256": artifacts[
                "render_robot_model"
            ].sha256,
            "dimensions": dimensions,
            "physics_timestep_s": PHYSICS_TIMESTEP_S,
            "sample_fps": SAMPLE_FPS,
            "scene_context": projected_scene,
            "matrix_replay_projection": {
                "schema_id": PROJECTION_SCHEMA,
                "operation": "metadata_enrichment_only_frames_preserved",
                "source_frame_count": identity["frame_count"],
                "source_frames_sha256": identity["frame_sha256"],
                "inputs_sha256": projection_inputs,
            },
        }
    )
    if projection["frames"] != trace["frames"]:
        raise ProjectionError("internal error: projection changed source frames")
    projection_bytes = _pretty_json_bytes(projection)
    projection_sha256 = hashlib.sha256(projection_bytes).hexdigest()

    trace_temporary: Path | None = None
    receipt_temporary: Path | None = None
    published_trace = False
    try:
        trace_temporary = _write_temp(output_trace, projection_bytes)
        try:
            staged_payload = _parse_json(
                _read_regular(
                    trace_temporary,
                    label="staged replay trace",
                    max_bytes=MAX_TRACE_BYTES,
                )
            )
            if staged_payload.get("frames") != trace.get("frames"):
                raise ProjectionError("staged replay trace changed source frames")
            if _frame_digest(staged_payload["frames"]) != identity["frame_sha256"]:
                raise ProjectionError(
                    "staged replay frame digest differs from the source"
                )
            validated = replay.validate_trace(trace_temporary)
        except replay.TraceValidationError as exc:
            raise ProjectionError(
                f"Matrix validate_trace self-check failed: {exc}"
            ) from exc
        if (
            validated.sha256 != projection_sha256
            or validated.trace_id != identity["trace_id"]
            or len(validated.frames) != identity["frame_count"]
            or validated.dimensions != EXPECTED_DIMS
            or validated.model_sha256 != artifacts["scene_model"].sha256
            or validated.render_model_sha256
            != artifacts["render_robot_model"].sha256
        ):
            raise ProjectionError("Matrix validate_trace self-check identity drifted")

        receipt = {
            "schema_id": RECEIPT_SCHEMA,
            "passed": True,
            "operation": "metadata_enrichment_only_frames_preserved",
            "physics_execution": replay.PHYSICS_EXECUTION,
            "render_mode": replay.RENDER_MODE,
            "trace_identity": {
                "physics_trace_id": identity["trace_id"],
                "source_frame_count": identity["frame_count"],
                "source_frames_sha256": identity["frame_sha256"],
                "source_time_range_s": [
                    identity["first_time_s"],
                    identity["last_time_s"],
                ],
                "dimensions": dimensions,
                "physics_timestep_s": PHYSICS_TIMESTEP_S,
                "sample_fps": SAMPLE_FPS,
            },
            "inputs": {
                "source_trace": artifacts["source_trace"].binding(
                    schema_id=replay.TRACE_SCHEMA,
                    frames_sha256=identity["frame_sha256"],
                ),
                "source_summary": artifacts["source_summary"].binding(
                    schema_id=SUMMARY_SCHEMA
                ),
                "validation": artifacts["validation"].binding(
                    schema_id=VALIDATION_SCHEMA
                ),
                "scene_model": artifacts["scene_model"].binding(),
                "render_robot_model": artifacts["render_robot_model"].binding(),
            },
            "outputs": {
                "replay_trace": {
                    "path": os.fspath(output_trace),
                    "sha256": projection_sha256,
                    "size_bytes": len(projection_bytes),
                    "schema_id": replay.TRACE_SCHEMA,
                    "frames_sha256": identity["frame_sha256"],
                }
            },
            "self_validation": {
                "validator": "replay_matrix_physics_trace.validate_trace",
                "passed": True,
                "trace_sha256": validated.sha256,
                "scene_model_sha256": validated.model_sha256,
                "render_robot_model_sha256": validated.render_model_sha256,
            },
        }
        receipt_bytes = _pretty_json_bytes(receipt)
        receipt_temporary = _write_temp(output_receipt, receipt_bytes)

        for artifact in artifacts.values():
            _assert_unchanged(artifact)
        if output_trace.exists() or output_trace.is_symlink():
            raise ProjectionError("output replay trace appeared before publish")
        if output_receipt.exists() or output_receipt.is_symlink():
            raise ProjectionError("projection receipt appeared before publish")

        os.replace(trace_temporary, output_trace)
        trace_temporary = None
        published_trace = True
        _fsync_directory(output_trace.parent)
        written_trace = _read_regular(
            output_trace, label="published replay trace", max_bytes=MAX_TRACE_BYTES
        )
        if written_trace.sha256 != projection_sha256:
            raise ProjectionError("published replay trace SHA256 drifted")
        os.replace(receipt_temporary, output_receipt)
        receipt_temporary = None
        _fsync_directory(output_receipt.parent)
        written_receipt = _read_regular(
            output_receipt,
            label="published projection receipt",
            max_bytes=MAX_JSON_BYTES,
        )
        if written_receipt.contents != receipt_bytes:
            raise ProjectionError("published projection receipt bytes drifted")
        return receipt
    except BaseException:
        if published_trace and not output_receipt.exists():
            output_trace.unlink(missing_ok=True)
            try:
                _fsync_directory(output_trace.parent)
            except OSError:
                pass
        raise
    finally:
        if trace_temporary is not None:
            trace_temporary.unlink(missing_ok=True)
        if receipt_temporary is not None:
            receipt_temporary.unlink(missing_ok=True)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-trace", type=Path, required=True)
    parser.add_argument("--source-summary", type=Path, required=True)
    parser.add_argument("--validation", type=Path, required=True)
    parser.add_argument("--scene-model", type=Path, required=True)
    parser.add_argument("--render-robot-model", type=Path, required=True)
    parser.add_argument("--output-trace", type=Path, required=True)
    parser.add_argument("--receipt", type=Path, required=True)
    for name in (
        "source-trace",
        "source-summary",
        "validation",
        "scene-model",
        "render-robot-model",
    ):
        parser.add_argument(f"--expected-{name}-sha256")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    expected_hashes = {
        "source_trace": args.expected_source_trace_sha256,
        "source_summary": args.expected_source_summary_sha256,
        "validation": args.expected_validation_sha256,
        "scene_model": args.expected_scene_model_sha256,
        "render_robot_model": args.expected_render_robot_model_sha256,
    }
    try:
        receipt = prepare_projection(
            source_trace_path=args.source_trace,
            source_summary_path=args.source_summary,
            validation_path=args.validation,
            scene_model_path=args.scene_model,
            render_robot_model_path=args.render_robot_model,
            output_trace_path=args.output_trace,
            receipt_path=args.receipt,
            expected_hashes=expected_hashes,
        )
    except (OSError, ProjectionError) as exc:
        print(f"Scene6 replay trace projection failed: {exc}", file=sys.stderr)
        return 2
    receipt_path = args.receipt.expanduser().resolve()
    receipt_sha256 = hashlib.sha256(receipt_path.read_bytes()).hexdigest()
    print(
        json.dumps(
            {
                "passed": receipt["passed"],
                "replay_trace": receipt["outputs"]["replay_trace"],
                "receipt": {
                    "path": os.fspath(receipt_path),
                    "sha256": receipt_sha256,
                },
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
