from __future__ import annotations

import importlib.util
import hashlib
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "record_matrix_sonic_video.py"
SPEC = importlib.util.spec_from_file_location("record_matrix_sonic_video", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


WINDOW_TREE = """
xwininfo: Window id: 0x6a2 (the root window) (has no name)

  Root window id: 0x6a2 (the root window) (has no name)
  Parent window id: 0x0 (none)
     2 children:
     0x740002e "zsibot_mujoco_ue (64-bit Development SF_VULKAN_SM6) ": ("zsibot_mujoco_ue" "zsibot_mujoco_ue")  1920x1080+10+61  +355+288
     0x3000001 "Terminal": ("xfce4-terminal" "Xfce4-terminal")  1000x700+0+0  +0+0
"""

WINDOW_DETAILS = """
xwininfo: Window id: 0x740002e "zsibot_mujoco_ue (64-bit Development SF_VULKAN_SM6) "

  Absolute upper-left X:  355
  Absolute upper-left Y:  288
  Relative upper-left X:  10
  Relative upper-left Y:  61
  Width: 1920
  Height: 1080
  Map State: IsViewable
"""

FFMPEG_PROBE = """
  Duration: 00:00:04.00, start: 0.000000, bitrate: 1137 kb/s
  Stream #0:0[0x1](und): Video: h264 (High) (avc1 / 0x31637661), yuv420p(progressive), 1920x1080, 1133 kb/s, 30 fps, 30 tbr, 15360 tbn (default)
"""


def window() -> object:
    return MODULE.WindowInfo(
        id_hex="0x740002e",
        id_decimal=int("0x740002e", 16),
        title="zsibot_mujoco_ue (64-bit Development SF_VULKAN_SM6) ",
        instance="zsibot_mujoco_ue",
        class_name="zsibot_mujoco_ue",
        x=355,
        y=288,
        width=1920,
        height=1080,
    )


class MatrixSonicVideoTest(unittest.TestCase):
    def test_parses_exact_matrix_window(self) -> None:
        matches = MODULE.parse_window_tree(
            WINDOW_TREE, MODULE.DEFAULT_WINDOW_TITLE_REGEX
        )
        self.assertEqual(
            matches,
            [
                (
                    "0x740002e",
                    "zsibot_mujoco_ue (64-bit Development SF_VULKAN_SM6) ",
                    "zsibot_mujoco_ue",
                    "zsibot_mujoco_ue",
                )
            ],
        )
        self.assertEqual(
            MODULE.parse_window_geometry(WINDOW_DETAILS), (355, 288, 1920, 1080)
        )

    def test_capture_command_uses_window_without_rescaling(self) -> None:
        command = MODULE.build_capture_command(
            Path("/opt/ffmpeg"),
            Path("/tmp/video.mp4"),
            window=window(),
            display=":0",
            duration_s=15.0,
            fps=60.0,
            draw_mouse=False,
            encoder_args=["-c:v", "libx264", "-crf", "20"],
        )
        self.assertEqual(command[command.index("-window_id") + 1], str(int("0x740002e", 16)))
        self.assertEqual(command[command.index("-framerate") + 1], "60")
        self.assertNotIn("-vf", command)
        self.assertFalse(any("scale=" in value for value in command))

    def test_parses_ffmpeg_video_and_progress(self) -> None:
        parsed = MODULE.parse_ffmpeg_probe(
            FFMPEG_PROBE, "frame=1\nprogress=continue\nframe=120\nprogress=end\n"
        )
        self.assertEqual(parsed["codec"], "h264")
        self.assertEqual(parsed["pixel_format"], "yuv420p")
        self.assertEqual((parsed["width"], parsed["height"]), (1920, 1080))
        self.assertEqual(parsed["fps"], 30.0)
        self.assertEqual(parsed["duration_s"], 4.0)
        self.assertEqual(parsed["decoded_frames"], 120)

    def test_accepts_native_dynamic_video(self) -> None:
        probe = MODULE.VideoProbe(
            codec="h264",
            pixel_format="yuv420p",
            width=1920,
            height=1080,
            fps=30.0,
            duration_s=15.0,
            decoded_frames=450,
            y_min=20.0,
            y_avg=148.0,
            y_max=226.0,
            saturation_avg=7.8,
            sampled_frames=75,
            unique_sample_hashes=75,
        )
        quality = MODULE.evaluate_video_quality(
            probe,
            requested_duration_s=15.0,
            requested_fps=30.0,
            window=window(),
            file_size=1_000_000,
            allow_static=False,
            allow_short=False,
        )
        self.assertTrue(quality["passed"])
        self.assertEqual(quality["failures"], [])
        self.assertEqual(quality["decoded_frame_ratio"], 1.0)

    def test_rejects_compounded_duration_and_fps_frame_loss(self) -> None:
        probe = MODULE.VideoProbe(
            codec="h264",
            pixel_format="yuv420p",
            width=1920,
            height=1080,
            fps=27.0,
            duration_s=9.0,
            decoded_frames=243,
            y_min=20.0,
            y_avg=148.0,
            y_max=226.0,
            saturation_avg=7.8,
            sampled_frames=45,
            unique_sample_hashes=45,
        )
        quality = MODULE.evaluate_video_quality(
            probe,
            requested_duration_s=10.0,
            requested_fps=30.0,
            window=window(),
            file_size=1_000_000,
            allow_static=False,
            allow_short=False,
        )
        self.assertFalse(quality["passed"])
        self.assertEqual(quality["decoded_frame_ratio"], 0.81)
        self.assertTrue(
            any("90% of requested" in failure for failure in quality["failures"])
        )

    def test_rejects_scaled_short_uniform_static_video(self) -> None:
        probe = MODULE.VideoProbe(
            codec="h264",
            pixel_format="yuv420p",
            width=1280,
            height=720,
            fps=20.0,
            duration_s=5.0,
            decoded_frames=80,
            y_min=0.0,
            y_avg=0.0,
            y_max=0.0,
            saturation_avg=0.0,
            sampled_frames=25,
            unique_sample_hashes=1,
        )
        quality = MODULE.evaluate_video_quality(
            probe,
            requested_duration_s=15.0,
            requested_fps=30.0,
            window=window(),
            file_size=1000,
            allow_static=False,
            allow_short=False,
        )
        self.assertFalse(quality["passed"])
        self.assertGreaterEqual(len(quality["failures"]), 6)

    def test_source_provenance_hashes_runtime_assets(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            urdf = root / "robot.urdf"
            model = root / "physics.xml"
            launcher = root / "scripts" / "run_matrix_sonic_urban_v1.sh"
            contract = root / "research" / "urban_v1" / "scene.json"
            launcher.parent.mkdir(parents=True)
            contract.parent.mkdir(parents=True)
            urdf.write_bytes(b"visual robot")
            model.write_bytes(b"physics model")
            launcher.write_bytes(b"urban launcher")
            contract.write_bytes(b"urban contract")

            provenance = MODULE._source_provenance(
                root,
                [
                    "bash",
                    "scripts/run_matrix_sonic_urban_v1.sh",
                    "--urdf",
                    str(urdf),
                ],
                {"model": str(model)},
            )

            self.assertEqual(
                provenance["visual_urdf"]["sha256"],
                hashlib.sha256(b"visual robot").hexdigest(),
            )
            self.assertEqual(
                provenance["physics_model"]["sha256"],
                hashlib.sha256(b"physics model").hexdigest(),
            )
            self.assertEqual(provenance["visual_urdf"]["size_bytes"], 12)
            self.assertEqual(provenance["physics_model"]["size_bytes"], 13)
            self.assertEqual(
                provenance["launcher_entrypoint"]["sha256"],
                hashlib.sha256(b"urban launcher").hexdigest(),
            )
            self.assertEqual(
                provenance["urban_v1_contract"]["sha256"],
                hashlib.sha256(b"urban contract").hexdigest(),
            )

    def test_preview_strip_is_generated_atomically(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            video = root / "capture.mp4"
            output = root / "capture.preview.jpg"
            video.write_bytes(b"video")

            def fake_run(command: object, **_: object) -> subprocess.CompletedProcess[str]:
                values = [str(value) for value in command]
                Path(values[-1]).write_bytes(b"preview")
                return subprocess.CompletedProcess(values, 0, "", "")

            with mock.patch.object(MODULE, "_run_text", side_effect=fake_run) as run:
                MODULE.create_preview_strip(
                    Path("/opt/ffmpeg"),
                    video,
                    output,
                    duration_s=10.0,
                )

            command = [str(value) for value in run.call_args.args[0]]
            self.assertEqual(output.read_bytes(), b"preview")
            self.assertIn("fps=0.5,scale=384:-1,tile=5x1", command)
            self.assertFalse((root / ".capture.preview.partial.jpg").exists())


if __name__ == "__main__":
    unittest.main()
