from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts" / "verify_matrix_scene6_visual_motion.py"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "verify_matrix_scene6_visual_motion_test", SCRIPT
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


MOTION = load_module()


def write_trace(root: Path, *, omit_phase: str | None = None) -> Path:
    transitions = [
        ("world_ready", 0.0),
        ("dock_arm_clearance", 0.9),
        ("dock_with_pregrasp", 1.5),
        ("assisted_stance_settle", 3.0),
        ("pick_place_contact_stabilized", 3.2),
        ("contact_validated", 3.5),
        ("grasp_stabilizer_active", 4.5),
        ("cube_supported_on_worktop", 6.5),
        ("grasp_stabilizer_released", 7.3),
    ]
    trace = root / "physics-trace.json"
    trace.write_text(
        json.dumps(
            {
                "schema_id": "twinbot.physics_trace.mujoco.v0",
                "sample_fps": 25.0,
                "transitions": [
                    {
                        "phase": phase,
                        "time_s": time_s,
                        "frame_index": int(round(time_s * 25.0)),
                    }
                    for phase, time_s in transitions
                    if phase != omit_phase
                ],
                "frames": [
                    {"time_s": index / 25.0}
                    for index in range(int(8.0 * 25.0) + 1)
                ],
            }
        ),
        encoding="utf-8",
    )
    return trace


def solid_frame(value: int = 40) -> bytes:
    return bytes((value, value, value)) * (MOTION.DECODE_WIDTH * MOTION.DECODE_HEIGHT)


def moving_frame(frame_index: int) -> bytes:
    video_time = frame_index / MOTION.EXPECTED_FPS
    trace_time = max(0.0, video_time - 1.0)
    width = MOTION.DECODE_WIDTH
    height = MOTION.DECODE_HEIGHT
    pixels = bytearray(width * height * 3)

    navigation_progress = min(trace_time, 1.5) / 1.5
    background_shift = int(round(42.0 * navigation_progress))
    for y in range(height):
        for x in range(width):
            value = 32 + ((x // 12 + y // 12) % 2) * 18 + background_shift
            offset = (y * width + x) * 3
            pixels[offset : offset + 3] = bytes((value, value + 3, value + 6))

    center_x = 160
    center_y = 105
    rectangle_width = 44
    rectangle_height = 72
    red, green, blue = 210, 95, 48
    if 1.5 <= trace_time < 3.0:
        progress = (trace_time - 1.5) / 1.5
        center_y -= int(round(30 * progress))
    elif 3.0 <= trace_time < 4.5:
        progress = (trace_time - 3.0) / 1.5
        rectangle_width += int(round(34 * progress))
        green += int(round(90 * progress))
    elif 4.5 <= trace_time < 6.5:
        progress = (trace_time - 4.5) / 2.0
        center_x += int(round(58 * progress))
        center_y += int(round(34 * progress))
    elif trace_time >= 6.5:
        progress = min(1.0, (trace_time - 6.5) / 1.54)
        rectangle_width -= int(round(26 * progress))
        rectangle_height -= int(round(24 * progress))
        blue += int(round(130 * progress))

    x0 = center_x - rectangle_width // 2
    x1 = center_x + rectangle_width // 2
    y0 = center_y - rectangle_height // 2
    y1 = center_y + rectangle_height // 2
    for y in range(max(0, y0), min(height, y1)):
        for x in range(max(0, x0), min(width, x1)):
            offset = (y * width + x) * 3
            pixels[offset : offset + 3] = bytes((red, min(green, 255), min(blue, 255)))
    return bytes(pixels)


class VisualMotionGateTest(unittest.TestCase):
    def test_recorder_and_postflight_require_visual_motion_receipt(self) -> None:
        recorder = (REPO_ROOT / "scripts" / "record_matrix_scene6_task_video.sh").read_text(
            encoding="utf-8"
        )
        postflight = (REPO_ROOT / "scripts" / "verify_matrix_scene6_task_video.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("verify_matrix_scene6_visual_motion.py", recorder)
        self.assertIn('--visual-motion-receipt "$VISUAL_MOTION_RECEIPT"', recorder)
        self.assertIn(
            'parser.add_argument("--visual-motion-receipt", type=Path, required=True)',
            postflight,
        )

    def analyze_with_decoder(self, root: Path, decoder) -> dict:
        trace = write_trace(root)
        video = root / "task.mp4"
        video.write_bytes(b"synthetic-video-binding")

        def decode(_ffmpeg, _video, frame_indices):
            return {index: decoder(index) for index in sorted(set(frame_indices))}

        with (
            mock.patch.object(
                MOTION, "_resolve_ffmpeg", return_value=Path("/test/ffmpeg")
            ),
            mock.patch.object(MOTION, "_ffmpeg_version", return_value="ffmpeg test"),
            mock.patch.object(MOTION, "_decode_frames", side_effect=decode),
        ):
            return MOTION.analyze(video=video, trace_path=trace, pre_roll_s=1.0)

    def test_static_decoded_video_is_rejected_at_every_semantic_stage(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            receipt = self.analyze_with_decoder(Path(temporary), lambda _index: solid_frame())

        self.assertFalse(receipt["passed"])
        self.assertEqual(
            [stage["name"] for stage in receipt["stages"]],
            list(MOTION.REQUIRED_STAGE_NAMES),
        )
        self.assertTrue(all(not stage["passed"] for stage in receipt["stages"]))
        self.assertEqual(len(receipt["failures"]), len(MOTION.REQUIRED_STAGE_NAMES))

    def test_obvious_motion_passes_full_frame_and_robot_table_roi_gates(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            receipt = self.analyze_with_decoder(Path(temporary), moving_frame)

        self.assertTrue(receipt["passed"], receipt["failures"])
        self.assertEqual(receipt["failures"], [])
        self.assertTrue(all(stage["passed"] for stage in receipt["stages"]))
        self.assertEqual(receipt["stages"][0]["scope"], "full_frame")
        self.assertTrue(
            all(
                stage["scope"] == "central_robot_table_roi"
                for stage in receipt["stages"][1:]
            )
        )
        self.assertEqual(receipt["baseline"]["diff"]["full_frame"]["mean_luma_abs_diff"], 0.0)

    def test_missing_transition_phase_writes_failed_receipt_without_decoding(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            trace = write_trace(root, omit_phase="contact_validated")
            video = root / "task.mp4"
            video.write_bytes(b"synthetic-video-binding")
            receipt_path = root / "task.visual-motion.json"

            with mock.patch.object(MOTION, "_decode_frames") as decode:
                receipt = MOTION.write_receipt(
                    video=video,
                    trace_path=trace,
                    receipt_path=receipt_path,
                    pre_roll_s=1.0,
                )

            self.assertFalse(receipt["passed"])
            self.assertIn("contact_validated", receipt["failures"][0])
            self.assertEqual(
                json.loads(receipt_path.read_text(encoding="utf-8")), receipt
            )
            decode.assert_not_called()

    def test_missing_pre_dock_arm_clearance_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            trace = write_trace(root, omit_phase="dock_arm_clearance")
            video = root / "task.mp4"
            video.write_bytes(b"synthetic-video-binding")
            receipt = MOTION.write_receipt(
                video=video,
                trace_path=trace,
                receipt_path=root / "task.visual-motion.json",
                pre_roll_s=1.0,
            )

        self.assertFalse(receipt["passed"])
        self.assertIn("dock_arm_clearance", receipt["failures"][0])

    def test_receipt_validation_rejects_failed_or_incomplete_evidence(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            receipt = self.analyze_with_decoder(root, moving_frame)
            video = root / "task.mp4"
            expected_video_sha = MOTION._sha256(video)
            expected_trace_sha = receipt["trace"]["sha256"]
            validated = MOTION.validate_receipt_payload(
                receipt,
                expected_video=video,
                expected_video_sha256=expected_video_sha,
                expected_trace_sha256=expected_trace_sha,
            )
            self.assertIs(validated, receipt)

            forged = json.loads(json.dumps(receipt))
            forged["stages"][2]["passed"] = False
            with self.assertRaisesRegex(MOTION.VisualMotionError, "stage did not pass"):
                MOTION.validate_receipt_payload(
                    forged,
                    expected_video=video,
                    expected_video_sha256=expected_video_sha,
                    expected_trace_sha256=expected_trace_sha,
                )


if __name__ == "__main__":
    unittest.main()
