#!/usr/bin/env python3
"""Verify visible Scene6 task motion at trace-defined semantic phases.

The recorder's generic quality gate can prove that an MP4 is decodable and is
not byte-for-byte static.  It cannot prove that the UE robot actually followed
the replay.  This verifier decodes a small, deterministic set of MP4 frames
with ffmpeg, then computes all motion metrics with Python's standard library.
"""

from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
from typing import Any, Sequence


RECEIPT_SCHEMA = "matrix.scene6_visual_motion_receipt.v1"
ALGORITHM_ID = "trace_phase_rgb_luma_diff.v1"
TRACE_SCHEMA = "twinbot.physics_trace.mujoco.v0"
EXPECTED_FPS = 25.0
DECODE_WIDTH = 320
DECODE_HEIGHT = 180
PIXEL_FORMAT = "rgb24"
LUMA_CHANGE_THRESHOLD = 12
RGB_CHANGE_THRESHOLD = 16
ROI_NORMALIZED = (0.16, 0.12, 0.86, 0.95)
SAMPLE_FRACTIONS = (0.08, 0.36, 0.68, 0.94)
REQUIRED_STAGE_NAMES = (
    "navigation",
    "raise_dock",
    "grasp",
    "carry_lower",
    "release_settle",
)
REQUIRED_TRANSITION_PHASES = (
    "world_ready",
    "dock_arm_clearance",
    "dock_with_pregrasp",
    "assisted_stance_settle",
    "pick_place_contact_stabilized",
    "contact_validated",
    "grasp_stabilizer_active",
    "cube_supported_on_worktop",
    "grasp_stabilizer_released",
)

# Floors deliberately sit above H.264 reconstruction noise.  The pre-roll
# noise comparison below raises them further for a visually noisy renderer.
STAGE_SPECS = (
    {
        "name": "navigation",
        "start": "world_ready",
        "end": "dock_arm_clearance",
        "scope": "full_frame",
        "minimum_mean_luma_excess": 0.6,
        "minimum_changed_fraction": 0.025,
    },
    {
        "name": "raise_dock",
        "start": "dock_arm_clearance",
        "end": "assisted_stance_settle",
        "contains": "dock_with_pregrasp",
        "scope": "central_robot_table_roi",
        "minimum_mean_luma_excess": 0.12,
        "minimum_changed_fraction": 0.006,
    },
    {
        "name": "grasp",
        "start": "assisted_stance_settle",
        "end": "grasp_stabilizer_active",
        "scope": "central_robot_table_roi",
        "minimum_mean_luma_excess": 0.025,
        "minimum_changed_fraction": 0.0015,
    },
    {
        "name": "carry_lower",
        "start": "grasp_stabilizer_active",
        "end": "cube_supported_on_worktop",
        "scope": "central_robot_table_roi",
        "minimum_mean_luma_excess": 0.5,
        "minimum_changed_fraction": 0.03,
    },
    {
        "name": "release_settle",
        "start": "cube_supported_on_worktop",
        "end": None,
        "contains": "grasp_stabilizer_released",
        "scope": "central_robot_table_roi",
        "minimum_mean_luma_excess": 0.2,
        "minimum_changed_fraction": 0.015,
    },
)


class VisualMotionError(RuntimeError):
    pass


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _regular_file(path: Path, *, label: str) -> Path:
    path = path.expanduser()
    if path.is_symlink() or not path.is_file():
        raise VisualMotionError(f"{label} must be a regular non-symlink file: {path}")
    return path.resolve()


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    path = path.expanduser().resolve()
    if path.is_symlink() or path.is_dir():
        raise VisualMotionError(f"receipt must not be a symlink or directory: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        dir=path.parent,
        prefix=f".{path.name}.",
        delete=False,
    ) as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
        temporary = Path(stream.name)
    os.chmod(temporary, 0o664)
    os.replace(temporary, path)


def _finite_number(value: Any, *, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise VisualMotionError(f"{label} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise VisualMotionError(f"{label} must be finite")
    return result


def _load_trace(path: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    path = _regular_file(path, label="trace")
    if path.stat().st_size <= 0 or path.stat().st_size > 1024 * 1024 * 1024:
        raise VisualMotionError("trace size is outside 1..1073741824 bytes")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise VisualMotionError(f"invalid trace JSON: {exc}") from exc
    if not isinstance(payload, dict) or payload.get("schema_id") != TRACE_SCHEMA:
        raise VisualMotionError(f"trace schema must be {TRACE_SCHEMA}")
    fps = _finite_number(payload.get("sample_fps"), label="trace sample_fps")
    if not math.isclose(fps, EXPECTED_FPS, abs_tol=1e-9):
        raise VisualMotionError("trace sample_fps must be 25")

    frames = payload.get("frames")
    if not isinstance(frames, list) or len(frames) < 2:
        raise VisualMotionError("trace frames must contain at least two entries")
    frame_times: list[float] = []
    for index, frame in enumerate(frames):
        if not isinstance(frame, dict):
            raise VisualMotionError(f"frames[{index}] must be an object")
        time_s = _finite_number(frame.get("time_s"), label=f"frames[{index}].time_s")
        if frame_times and time_s < frame_times[-1]:
            raise VisualMotionError("trace frame time regressed")
        frame_times.append(time_s)

    transitions = payload.get("transitions")
    if not isinstance(transitions, list) or not transitions:
        raise VisualMotionError("trace transitions must be a non-empty array")
    normalized_transitions: list[dict[str, Any]] = []
    previous_time: float | None = None
    previous_frame_index: int | None = None
    for index, transition in enumerate(transitions):
        if not isinstance(transition, dict):
            raise VisualMotionError(f"transitions[{index}] must be an object")
        phase = transition.get("phase")
        if not isinstance(phase, str) or not phase:
            raise VisualMotionError(f"transitions[{index}].phase is invalid")
        time_s = _finite_number(
            transition.get("time_s"), label=f"transitions[{index}].time_s"
        )
        if previous_time is not None and time_s < previous_time:
            raise VisualMotionError("trace transition time regressed")
        previous_time = time_s
        raw_frame_index = transition.get("frame_index")
        if raw_frame_index is None:
            frame_index = min(
                len(frame_times) - 1,
                bisect.bisect_left(frame_times, time_s),
            )
        elif (
            isinstance(raw_frame_index, bool)
            or not isinstance(raw_frame_index, int)
            or not 0 <= raw_frame_index < len(frame_times)
        ):
            raise VisualMotionError(
                f"transitions[{index}].frame_index is outside the trace"
            )
        else:
            frame_index = raw_frame_index
        if previous_frame_index is not None and frame_index < previous_frame_index:
            raise VisualMotionError("trace transition frame index regressed")
        previous_frame_index = frame_index
        normalized_transitions.append(
            {"phase": phase, "time_s": time_s, "frame_index": frame_index}
        )

    trace_end_s = frame_times[-1] + 1.0 / fps
    for transition in normalized_transitions:
        if not frame_times[0] - 1.0 / fps <= transition["time_s"] <= trace_end_s:
            raise VisualMotionError(
                "trace transition lies outside the recorded frame interval: "
                f"{transition['phase']}"
            )
    return payload, {
        "path": str(path),
        "sha256": _sha256(path),
        "size_bytes": path.stat().st_size,
        "schema_id": TRACE_SCHEMA,
        "fps": fps,
        "frame_count": len(frames),
        "first_time_s": frame_times[0],
        "last_time_s": frame_times[-1],
        "end_exclusive_time_s": trace_end_s,
        "end_exclusive_frame_index": len(frames),
        "transitions": normalized_transitions,
        "_frame_times": frame_times,
    }


def _select_stage_intervals(trace: dict[str, Any]) -> list[dict[str, Any]]:
    transitions = trace["transitions"]
    cursor = 0
    selected: dict[str, dict[str, Any]] = {}
    for phase in REQUIRED_TRANSITION_PHASES:
        found: dict[str, Any] | None = None
        while cursor < len(transitions):
            candidate = transitions[cursor]
            cursor += 1
            if candidate["phase"] == phase:
                found = candidate
                break
        if found is None:
            raise VisualMotionError(
                "trace is missing ordered visual-motion phase: " + phase
            )
        selected[phase] = found

    intervals: list[dict[str, Any]] = []
    for spec in STAGE_SPECS:
        start = selected[str(spec["start"])]
        if spec["end"] is None:
            end_phase = "trace_end"
            end_time = trace["end_exclusive_time_s"]
            end_frame_index = trace["end_exclusive_frame_index"]
        else:
            end_phase = str(spec["end"])
            end_time = selected[end_phase]["time_s"]
            end_frame_index = selected[end_phase]["frame_index"]
        duration_frames = end_frame_index - start["frame_index"]
        if duration_frames < 4:
            raise VisualMotionError(
                f"visual-motion phase {spec['name']} is too short to sample: "
                f"{duration_frames} frames"
            )
        contains = spec.get("contains")
        if contains is not None:
            contained_time = selected[str(contains)]["time_s"]
            if not start["time_s"] <= contained_time <= end_time:
                raise VisualMotionError(
                    f"visual-motion phase {spec['name']} does not contain {contains}"
                )
        intervals.append(
            {
                **spec,
                "start_phase": start["phase"],
                "start_trace_time_s": start["time_s"],
                "start_source_frame_index": start["frame_index"],
                "end_phase": end_phase,
                "end_trace_time_s": end_time,
                "end_source_frame_index": end_frame_index,
                "duration_source_frames": duration_frames,
                "duration_s": duration_frames / EXPECTED_FPS,
            }
        )
    return intervals


def _sample_points(
    intervals: list[dict[str, Any]],
    *,
    trace: dict[str, Any],
    pre_roll_s: float,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    if not math.isfinite(pre_roll_s) or pre_roll_s < 0.5:
        raise VisualMotionError(
            "pre-roll must be at least 0.5s to measure a static encoding baseline"
        )
    baseline_times = (min(0.15, pre_roll_s * 0.2), pre_roll_s * 0.72)
    baseline_points = [
        {
            "label": f"pre_roll_{index}",
            "trace_time_s": None,
            "requested_video_time_s": round(time_s, 6),
            "frame_index": max(0, int(round(time_s * EXPECTED_FPS))),
        }
        for index, time_s in enumerate(baseline_times)
    ]
    if baseline_points[0]["frame_index"] == baseline_points[1]["frame_index"]:
        raise VisualMotionError("pre-roll baseline samples collapse to one video frame")

    for interval in intervals:
        points: list[dict[str, Any]] = []
        for index, fraction in enumerate(SAMPLE_FRACTIONS):
            source_frame_index = int(
                round(
                    interval["start_source_frame_index"]
                    + fraction * interval["duration_source_frames"]
                )
            )
            source_frame_index = min(trace["frame_count"] - 1, source_frame_index)
            video_frame_index = round(pre_roll_s * EXPECTED_FPS) + source_frame_index
            trace_time = trace["_frame_times"][source_frame_index]
            video_time = video_frame_index / EXPECTED_FPS
            points.append(
                {
                    "label": f"sample_{index}",
                    "trace_time_s": round(trace_time, 6),
                    "source_frame_index": source_frame_index,
                    "requested_video_time_s": round(video_time, 6),
                    "frame_index": video_frame_index,
                }
            )
        if len({point["frame_index"] for point in points}) != len(points):
            raise VisualMotionError(
                f"visual-motion phase {interval['name']} samples are not distinct"
            )
        interval["sample_points"] = points
    return baseline_points, intervals


def _resolve_ffmpeg(explicit: Path | None) -> Path:
    if explicit is not None:
        candidate = explicit.expanduser().resolve()
        if not candidate.is_file() or not os.access(candidate, os.X_OK):
            raise VisualMotionError(f"ffmpeg is not executable: {candidate}")
        return candidate
    found = shutil.which("ffmpeg")
    if not found:
        raise VisualMotionError("ffmpeg is required for visual-motion verification")
    return Path(found).resolve()


def _ffmpeg_version(ffmpeg: Path) -> str:
    try:
        result = subprocess.run(
            [str(ffmpeg), "-version"],
            check=True,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=15.0,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise VisualMotionError(f"could not query ffmpeg: {exc}") from exc
    return result.stdout.splitlines()[0].strip()


def _decode_frames(
    ffmpeg: Path, video: Path, frame_indices: Sequence[int]
) -> dict[int, bytes]:
    ordered = sorted(set(frame_indices))
    if not ordered or ordered[0] < 0:
        raise VisualMotionError("decoded frame indices must be non-negative")
    expression = "+".join(f"eq(n\\,{index})" for index in ordered)
    filter_graph = (
        f"select={expression},"
        f"scale={DECODE_WIDTH}:{DECODE_HEIGHT}:flags=bilinear,format={PIXEL_FORMAT}"
    )
    command = [
        str(ffmpeg),
        "-hide_banner",
        "-loglevel",
        "error",
        "-i",
        str(video),
        "-an",
        "-sn",
        "-vf",
        filter_graph,
        "-vsync",
        "0",
        "-frames:v",
        str(len(ordered)),
        "-f",
        "rawvideo",
        "-pix_fmt",
        PIXEL_FORMAT,
        "pipe:1",
    ]
    try:
        result = subprocess.run(
            command,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=180.0,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise VisualMotionError(f"ffmpeg frame decode failed: {exc}") from exc
    if result.returncode != 0:
        message = result.stderr.decode("utf-8", errors="replace").strip()
        raise VisualMotionError(f"ffmpeg frame decode failed: {message[-1000:]}")
    frame_bytes = DECODE_WIDTH * DECODE_HEIGHT * 3
    expected_bytes = frame_bytes * len(ordered)
    if len(result.stdout) != expected_bytes:
        raise VisualMotionError(
            "ffmpeg did not decode every requested frame: "
            f"expected {expected_bytes} bytes, got {len(result.stdout)}"
        )
    return {
        frame_index: result.stdout[offset * frame_bytes : (offset + 1) * frame_bytes]
        for offset, frame_index in enumerate(ordered)
    }


def _pixel_roi() -> tuple[int, int, int, int]:
    x0, y0, x1, y1 = ROI_NORMALIZED
    return (
        int(round(x0 * DECODE_WIDTH)),
        int(round(y0 * DECODE_HEIGHT)),
        int(round(x1 * DECODE_WIDTH)),
        int(round(y1 * DECODE_HEIGHT)),
    )


def _region_diff(
    first: bytes,
    second: bytes,
    *,
    bounds: tuple[int, int, int, int],
) -> dict[str, Any]:
    if len(first) != len(second) or len(first) != DECODE_WIDTH * DECODE_HEIGHT * 3:
        raise VisualMotionError("decoded RGB frame size drifted")
    x0, y0, x1, y1 = bounds
    if not (0 <= x0 < x1 <= DECODE_WIDTH and 0 <= y0 < y1 <= DECODE_HEIGHT):
        raise VisualMotionError("motion ROI is outside the decoded frame")
    pixel_count = (x1 - x0) * (y1 - y0)
    luma_sum = 0
    luma_excess_sum = 0
    luma_square_sum = 0
    luma_changed = 0
    rgb_changed = 0
    histogram = [0] * 256
    for y in range(y0, y1):
        offset = (y * DECODE_WIDTH + x0) * 3
        end = (y * DECODE_WIDTH + x1) * 3
        for byte_index in range(offset, end, 3):
            first_r, first_g, first_b = first[byte_index : byte_index + 3]
            second_r, second_g, second_b = second[byte_index : byte_index + 3]
            first_luma = (77 * first_r + 150 * first_g + 29 * first_b + 128) >> 8
            second_luma = (77 * second_r + 150 * second_g + 29 * second_b + 128) >> 8
            luma_difference = abs(first_luma - second_luma)
            luma_sum += luma_difference
            luma_excess_sum += max(0, luma_difference - LUMA_CHANGE_THRESHOLD)
            luma_square_sum += luma_difference * luma_difference
            histogram[luma_difference] += 1
            if luma_difference >= LUMA_CHANGE_THRESHOLD:
                luma_changed += 1
            if max(
                abs(first_r - second_r),
                abs(first_g - second_g),
                abs(first_b - second_b),
            ) >= RGB_CHANGE_THRESHOLD:
                rgb_changed += 1
    percentile_target = max(1, math.ceil(pixel_count * 0.95))
    cumulative = 0
    percentile_95 = 0
    for difference, count in enumerate(histogram):
        cumulative += count
        if cumulative >= percentile_target:
            percentile_95 = difference
            break
    return {
        "pixel_count": pixel_count,
        "mean_luma_abs_diff": round(luma_sum / pixel_count, 6),
        "mean_luma_excess_over_threshold": round(
            luma_excess_sum / pixel_count, 6
        ),
        "rms_luma_diff": round(math.sqrt(luma_square_sum / pixel_count), 6),
        "p95_luma_abs_diff": percentile_95,
        "changed_luma_pixels": luma_changed,
        "changed_luma_fraction": round(luma_changed / pixel_count, 8),
        "changed_rgb_pixels": rgb_changed,
        "changed_rgb_fraction": round(rgb_changed / pixel_count, 8),
    }


def _frame_pair_diff(first: bytes, second: bytes) -> dict[str, Any]:
    return {
        "full_frame": _region_diff(
            first,
            second,
            bounds=(0, 0, DECODE_WIDTH, DECODE_HEIGHT),
        ),
        "central_robot_table_roi": _region_diff(
            first,
            second,
            bounds=_pixel_roi(),
        ),
    }


def _evaluate_stages(
    intervals: list[dict[str, Any]],
    frames: dict[int, bytes],
    baseline: dict[str, Any],
) -> tuple[list[dict[str, Any]], list[str]]:
    results: list[dict[str, Any]] = []
    failures: list[str] = []
    for interval in intervals:
        points = interval["sample_points"]
        pair_indices = ((0, 1), (1, 2), (2, 3), (0, 3))
        comparisons: list[dict[str, Any]] = []
        scope = str(interval["scope"])
        baseline_scope = baseline["diff"][scope]
        required_excess = max(
            float(interval["minimum_mean_luma_excess"]),
            float(baseline_scope["mean_luma_excess_over_threshold"]) * 3.0
            + 0.01,
        )
        required_fraction = max(
            float(interval["minimum_changed_fraction"]),
            float(baseline_scope["changed_luma_fraction"]) * 3.0 + 0.0005,
        )
        passing_comparisons: list[str] = []
        for left, right in pair_indices:
            first_point = points[left]
            second_point = points[right]
            diff = _frame_pair_diff(
                frames[first_point["frame_index"]],
                frames[second_point["frame_index"]],
            )
            label = f"{first_point['label']}->{second_point['label']}"
            scope_metrics = diff[scope]
            comparison_passed = (
                scope_metrics["mean_luma_excess_over_threshold"] >= required_excess
                and scope_metrics["changed_luma_fraction"] >= required_fraction
            )
            if comparison_passed:
                passing_comparisons.append(label)
            comparisons.append(
                {
                    "label": label,
                    "first_frame_index": first_point["frame_index"],
                    "second_frame_index": second_point["frame_index"],
                    "full_frame": diff["full_frame"],
                    "central_robot_table_roi": diff["central_robot_table_roi"],
                    "passed_for_stage_scope": comparison_passed,
                }
            )
        peak_mean = max(item[scope]["mean_luma_abs_diff"] for item in comparisons)
        peak_excess = max(
            item[scope]["mean_luma_excess_over_threshold"] for item in comparisons
        )
        peak_fraction = max(
            item[scope]["changed_luma_fraction"] for item in comparisons
        )
        passed = bool(passing_comparisons)
        failure = None
        if not passed:
            failure = (
                f"{interval['name']} has no {scope} comparison above motion gate: "
                f"peak excess={peak_excess:.6f} required={required_excess:.6f}, "
                f"peak changed_fraction={peak_fraction:.8f} "
                f"required={required_fraction:.8f}"
            )
            failures.append(failure)
        results.append(
            {
                "name": interval["name"],
                "scope": scope,
                "start_phase": interval["start_phase"],
                "end_phase": interval["end_phase"],
                "start_trace_time_s": interval["start_trace_time_s"],
                "end_trace_time_s": interval["end_trace_time_s"],
                "duration_s": round(interval["duration_s"], 6),
                "sample_points": points,
                "required": {
                    "mean_luma_excess_over_threshold": round(required_excess, 8),
                    "changed_luma_fraction": round(required_fraction, 8),
                    "absolute_floor_mean_luma_excess_over_threshold": interval[
                        "minimum_mean_luma_excess"
                    ],
                    "absolute_floor_changed_luma_fraction": interval[
                        "minimum_changed_fraction"
                    ],
                    "baseline_multiplier": 3.0,
                },
                "observed": {
                    "peak_mean_luma_abs_diff": peak_mean,
                    "peak_mean_luma_excess_over_threshold": peak_excess,
                    "peak_changed_luma_fraction": peak_fraction,
                    "passing_comparisons": passing_comparisons,
                },
                "comparisons": comparisons,
                "passed": passed,
                "failure": failure,
            }
        )
    return results, failures


def analyze(
    *,
    video: Path,
    trace_path: Path,
    pre_roll_s: float,
    ffmpeg_path: Path | None = None,
) -> dict[str, Any]:
    video = _regular_file(video, label="video")
    payload, trace = _load_trace(trace_path)
    del payload
    intervals = _select_stage_intervals(trace)
    baseline_points, intervals = _sample_points(
        intervals,
        trace=trace,
        pre_roll_s=pre_roll_s,
    )
    ffmpeg = _resolve_ffmpeg(ffmpeg_path)
    all_points = baseline_points + [
        point for interval in intervals for point in interval["sample_points"]
    ]
    frames = _decode_frames(
        ffmpeg, video, [point["frame_index"] for point in all_points]
    )
    baseline_diff = _frame_pair_diff(
        frames[baseline_points[0]["frame_index"]],
        frames[baseline_points[1]["frame_index"]],
    )
    baseline = {
        "purpose": "pre_roll_static_encoding_noise_floor",
        "sample_points": baseline_points,
        "diff": baseline_diff,
    }
    stages, failures = _evaluate_stages(intervals, frames, baseline)
    return {
        "schema_id": RECEIPT_SCHEMA,
        "passed": not failures,
        "failures": failures,
        "video": {
            "path": str(video),
            "sha256": _sha256(video),
            "size_bytes": video.stat().st_size,
        },
        "trace": {
            key: value for key, value in trace.items() if not key.startswith("_")
        },
        "alignment": {
            "pre_roll_s": pre_roll_s,
            "mapping": "video_frame=round(pre_roll*fps)+source_frame_index",
            "fps": EXPECTED_FPS,
        },
        "analyzer": {
            "algorithm_id": ALGORITHM_ID,
            "ffmpeg_path": str(ffmpeg),
            "ffmpeg_version": _ffmpeg_version(ffmpeg),
            "decode": {
                "width": DECODE_WIDTH,
                "height": DECODE_HEIGHT,
                "pixel_format": PIXEL_FORMAT,
                "selected_frame_count": len(frames),
            },
            "luma_change_threshold": LUMA_CHANGE_THRESHOLD,
            "rgb_change_threshold": RGB_CHANGE_THRESHOLD,
            "roi": {
                "label": "central_robot_table_roi",
                "normalized_xyxy": list(ROI_NORMALIZED),
                "pixel_xyxy": list(_pixel_roi()),
            },
        },
        "baseline": baseline,
        "stages": stages,
    }


def failure_receipt(
    *, video: Path, trace_path: Path, pre_roll_s: float, failure: str
) -> dict[str, Any]:
    def binding(path: Path) -> dict[str, Any]:
        expanded = path.expanduser()
        result: dict[str, Any] = {"path": str(expanded.resolve())}
        if expanded.is_file() and not expanded.is_symlink():
            result.update(
                sha256=_sha256(expanded), size_bytes=expanded.stat().st_size
            )
        return result

    return {
        "schema_id": RECEIPT_SCHEMA,
        "passed": False,
        "failures": [failure],
        "video": binding(video),
        "trace": binding(trace_path),
        "alignment": {"pre_roll_s": pre_roll_s, "fps": EXPECTED_FPS},
        "analyzer": {"algorithm_id": ALGORITHM_ID},
        "baseline": None,
        "stages": [],
    }


def write_receipt(
    *,
    video: Path,
    trace_path: Path,
    receipt_path: Path,
    pre_roll_s: float,
    ffmpeg_path: Path | None = None,
) -> dict[str, Any]:
    try:
        receipt = analyze(
            video=video,
            trace_path=trace_path,
            pre_roll_s=pre_roll_s,
            ffmpeg_path=ffmpeg_path,
        )
    except (OSError, ValueError, VisualMotionError) as exc:
        receipt = failure_receipt(
            video=video,
            trace_path=trace_path,
            pre_roll_s=pre_roll_s,
            failure=str(exc),
        )
    _atomic_json(receipt_path, receipt)
    return receipt


def validate_receipt_payload(
    payload: object,
    *,
    expected_video: Path,
    expected_video_sha256: str,
    expected_trace_sha256: str,
) -> dict[str, Any]:
    if not isinstance(payload, dict) or payload.get("schema_id") != RECEIPT_SCHEMA:
        raise VisualMotionError("visual-motion receipt schema is invalid")
    if payload.get("passed") is not True or payload.get("failures") != []:
        raise VisualMotionError(
            f"visual-motion receipt did not pass: {payload.get('failures')}"
        )
    analyzer = payload.get("analyzer")
    if not isinstance(analyzer, dict) or analyzer.get("algorithm_id") != ALGORITHM_ID:
        raise VisualMotionError("visual-motion receipt algorithm is invalid")
    video = payload.get("video")
    if not isinstance(video, dict):
        raise VisualMotionError("visual-motion receipt video binding is absent")
    try:
        bound_video = Path(str(video.get("path"))).resolve()
    except (OSError, ValueError) as exc:
        raise VisualMotionError("visual-motion receipt video path is invalid") from exc
    if bound_video != expected_video.resolve() or video.get("sha256") != expected_video_sha256:
        raise VisualMotionError("visual-motion receipt does not bind the accepted MP4")
    trace = payload.get("trace")
    if not isinstance(trace, dict) or trace.get("sha256") != expected_trace_sha256:
        raise VisualMotionError("visual-motion receipt trace SHA256 is invalid")
    transitions = trace.get("transitions")
    if not isinstance(transitions, list):
        raise VisualMotionError("visual-motion receipt trace transitions are absent")
    transition_phases = [
        item.get("phase") if isinstance(item, dict) else None for item in transitions
    ]
    cursor = 0
    for required_phase in REQUIRED_TRANSITION_PHASES:
        try:
            cursor = transition_phases.index(required_phase, cursor) + 1
        except ValueError as exc:
            raise VisualMotionError(
                "visual-motion receipt is missing ordered trace phase: "
                + required_phase
            ) from exc
    stages = payload.get("stages")
    if not isinstance(stages, list) or tuple(
        item.get("name") if isinstance(item, dict) else None for item in stages
    ) != REQUIRED_STAGE_NAMES:
        raise VisualMotionError("visual-motion receipt stages are incomplete or out of order")
    for stage, specification in zip(stages, STAGE_SPECS):
        if stage.get("passed") is not True or stage.get("failure") is not None:
            raise VisualMotionError(
                f"visual-motion stage did not pass: {stage.get('name')}"
            )
        if stage.get("scope") != specification["scope"]:
            raise VisualMotionError(
                f"visual-motion stage scope drifted: {stage.get('name')}"
            )
        samples = stage.get("sample_points")
        comparisons = stage.get("comparisons")
        observed = stage.get("observed")
        if (
            not isinstance(samples, list)
            or len(samples) != len(SAMPLE_FRACTIONS)
            or not isinstance(comparisons, list)
            or len(comparisons) < 4
            or not isinstance(observed, dict)
            or not observed.get("passing_comparisons")
        ):
            raise VisualMotionError(
                f"visual-motion stage evidence is incomplete: {stage.get('name')}"
            )
    return payload


def load_receipt(
    path: Path,
    *,
    expected_video: Path,
    expected_video_sha256: str,
    expected_trace_sha256: str,
) -> dict[str, Any]:
    path = _regular_file(path, label="visual-motion receipt")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise VisualMotionError(f"invalid visual-motion receipt JSON: {exc}") from exc
    return validate_receipt_payload(
        payload,
        expected_video=expected_video,
        expected_video_sha256=expected_video_sha256,
        expected_trace_sha256=expected_trace_sha256,
    )


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", type=Path, required=True)
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--receipt", type=Path, required=True)
    parser.add_argument("--pre-roll-seconds", type=float, default=2.0)
    parser.add_argument("--ffmpeg", type=Path)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    receipt = write_receipt(
        video=args.video,
        trace_path=args.trace,
        receipt_path=args.receipt,
        pre_roll_s=args.pre_roll_seconds,
        ffmpeg_path=args.ffmpeg,
    )
    if receipt["passed"]:
        print(
            "[matrix-scene6-visual-motion] passed "
            f"video={receipt['video']['path']} stages={len(receipt['stages'])}",
            flush=True,
        )
        return 0
    print(
        "[matrix-scene6-visual-motion] ERROR: " + "; ".join(receipt["failures"]),
        file=os.sys.stderr,
    )
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
