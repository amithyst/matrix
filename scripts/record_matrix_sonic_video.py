#!/usr/bin/env python3
"""Record the native Matrix UE window around the canonical SONIC launcher."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
from typing import Any, Sequence


DEFAULT_WINDOW_TITLE_REGEX = r"^zsibot_mujoco_ue \(64-bit Development SF_VULKAN_SM6\)"
WINDOW_LINE_RE = re.compile(
    r'^\s*(0x[0-9a-fA-F]+)\s+"([^"]*)":\s+'
    r'\("([^"]*)"\s+"([^"]*)"\)'
)
GEOMETRY_PATTERNS = {
    "x": re.compile(r"Absolute upper-left X:\s*(-?\d+)"),
    "y": re.compile(r"Absolute upper-left Y:\s*(-?\d+)"),
    "width": re.compile(r"^\s*Width:\s*(\d+)", re.MULTILINE),
    "height": re.compile(r"^\s*Height:\s*(\d+)", re.MULTILINE),
}


class VideoCaptureError(RuntimeError):
    """Raised when a reproducible Matrix recording cannot be produced."""


@dataclass(frozen=True)
class WindowInfo:
    id_hex: str
    id_decimal: int
    title: str
    instance: str
    class_name: str
    x: int
    y: int
    width: int
    height: int


@dataclass(frozen=True)
class VideoProbe:
    codec: str
    pixel_format: str
    width: int
    height: int
    fps: float
    duration_s: float
    decoded_frames: int
    y_min: float
    y_avg: float
    y_max: float
    saturation_avg: float
    sampled_frames: int
    unique_sample_hashes: int


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--metadata", type=Path)
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument(
        "--encoder", choices=("auto", "libx264", "h264_nvenc"), default="auto"
    )
    parser.add_argument("--crf", type=int, default=20)
    parser.add_argument("--x264-preset", default="veryfast")
    parser.add_argument("--nvenc-preset", default="p4")
    parser.add_argument("--ffmpeg", type=Path)
    parser.add_argument("--display", default=os.environ.get("DISPLAY", ":0"))
    parser.add_argument("--xauthority", type=Path)
    parser.add_argument("--window-id")
    parser.add_argument(
        "--window-title-regex", default=DEFAULT_WINDOW_TITLE_REGEX
    )
    parser.add_argument("--window-timeout", type=float, default=90.0)
    parser.add_argument(
        "--ready", choices=("active", "status", "window"), default="active"
    )
    parser.add_argument("--ready-active-seconds", type=float, default=8.0)
    parser.add_argument("--ready-timeout", type=float, default=180.0)
    parser.add_argument("--status-max-age", type=float, default=5.0)
    parser.add_argument("--status-file", type=Path)
    parser.add_argument("--attach", action="store_true")
    parser.add_argument("--keep-running", action="store_true")
    parser.add_argument("--draw-mouse", action="store_true")
    parser.add_argument("--allow-static", action="store_true")
    parser.add_argument("--allow-short", action="store_true")
    parser.add_argument("--quality-sample-fps", type=float, default=5.0)
    parser.add_argument("--notes", default="")
    parser.add_argument(
        "launcher",
        nargs=argparse.REMAINDER,
        help="normal Matrix launcher command after --; omit only with --attach",
    )
    args = parser.parse_args(argv)
    if args.launcher and args.launcher[0] == "--":
        args.launcher = args.launcher[1:]
    if args.attach and args.launcher:
        parser.error("--attach cannot be combined with a launcher command")
    if not args.attach and not args.launcher:
        parser.error("provide --attach or a launcher command after --")
    if args.duration <= 0.0 or not math.isfinite(args.duration):
        parser.error("--duration must be a positive finite number")
    if args.fps <= 0.0 or not math.isfinite(args.fps):
        parser.error("--fps must be a positive finite number")
    if args.window_timeout <= 0.0 or args.ready_timeout <= 0.0:
        parser.error("window/ready timeouts must be positive")
    if args.ready_active_seconds < 0.0:
        parser.error("--ready-active-seconds must be non-negative")
    if args.status_max_age <= 0.0:
        parser.error("--status-max-age must be positive")
    if not 0 <= args.crf <= 51:
        parser.error("--crf must be between 0 and 51")
    if args.quality_sample_fps <= 0.0:
        parser.error("--quality-sample-fps must be positive")
    return args


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        dir=path.parent,
        prefix=f".{path.name}.",
        delete=False,
    ) as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
        temporary = Path(stream.name)
    os.replace(temporary, path)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _run_text(
    command: Sequence[str],
    *,
    env: dict[str, str] | None = None,
    check: bool = True,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        list(command),
        env=env,
        check=check,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def _default_xauthority() -> Path:
    configured = os.environ.get("XAUTHORITY")
    candidates = [
        Path(configured).expanduser() if configured else None,
        Path(f"/run/user/{os.getuid()}/gdm/Xauthority"),
        Path.home() / ".Xauthority",
    ]
    for candidate in candidates:
        if candidate is not None and candidate.is_file():
            return candidate.resolve()
    raise VideoCaptureError(
        "no Xauthority file found; pass --xauthority explicitly"
    )


def _x11_environment(display: str, xauthority: Path) -> dict[str, str]:
    environment = os.environ.copy()
    environment["DISPLAY"] = display
    environment["XAUTHORITY"] = str(xauthority)
    return environment


def parse_window_tree(
    text: str, title_regex: str
) -> list[tuple[str, str, str, str]]:
    try:
        title_pattern = re.compile(title_regex)
    except re.error as exc:
        raise VideoCaptureError(f"invalid window title regex: {exc}") from exc
    matches: list[tuple[str, str, str, str]] = []
    for line in text.splitlines():
        match = WINDOW_LINE_RE.search(line)
        if match is None or title_pattern.search(match.group(2)) is None:
            continue
        matches.append(
            (match.group(1), match.group(2), match.group(3), match.group(4))
        )
    return matches


def parse_window_geometry(text: str) -> tuple[int, int, int, int]:
    values: dict[str, int] = {}
    for key, pattern in GEOMETRY_PATTERNS.items():
        match = pattern.search(text)
        if match is None:
            raise VideoCaptureError(f"xwininfo output is missing {key}")
        values[key] = int(match.group(1))
    if values["width"] <= 0 or values["height"] <= 0:
        raise VideoCaptureError("window dimensions must be positive")
    return values["x"], values["y"], values["width"], values["height"]


def _window_details(
    id_hex: str,
    title: str,
    instance: str,
    class_name: str,
    *,
    env: dict[str, str],
) -> WindowInfo:
    result = _run_text(["xwininfo", "-id", id_hex], env=env)
    x, y, width, height = parse_window_geometry(result.stdout)
    return WindowInfo(
        id_hex=id_hex.lower(),
        id_decimal=int(id_hex, 16),
        title=title,
        instance=instance,
        class_name=class_name,
        x=x,
        y=y,
        width=width,
        height=height,
    )


def find_windows(title_regex: str, *, env: dict[str, str]) -> list[WindowInfo]:
    result = _run_text(["xwininfo", "-root", "-tree"], env=env)
    return [
        _window_details(*match, env=env)
        for match in parse_window_tree(result.stdout, title_regex)
    ]


def _window_by_id(id_value: str, *, env: dict[str, str]) -> WindowInfo:
    try:
        id_decimal = int(id_value, 0)
    except ValueError as exc:
        raise VideoCaptureError(f"invalid --window-id: {id_value}") from exc
    id_hex = hex(id_decimal)
    tree = _run_text(["xwininfo", "-root", "-tree"], env=env).stdout
    title = ""
    instance = ""
    class_name = ""
    for line in tree.splitlines():
        match = WINDOW_LINE_RE.search(line)
        if match is not None and int(match.group(1), 16) == id_decimal:
            title, instance, class_name = match.group(2), match.group(3), match.group(4)
            break
    return _window_details(
        id_hex, title, instance, class_name, env=env
    )


def wait_for_window(
    title_regex: str,
    *,
    env: dict[str, str],
    timeout_s: float,
    launcher: subprocess.Popen[Any] | None,
) -> WindowInfo:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if launcher is not None and launcher.poll() is not None:
            raise VideoCaptureError(
                f"Matrix launcher exited before the UE window appeared: {launcher.returncode}"
            )
        windows = find_windows(title_regex, env=env)
        if len(windows) == 1:
            return windows[0]
        if len(windows) > 1:
            ids = ", ".join(window.id_hex for window in windows)
            raise VideoCaptureError(
                f"multiple Matrix UE windows match ({ids}); pass --window-id"
            )
        time.sleep(0.5)
    raise VideoCaptureError(
        f"Matrix UE window did not appear within {timeout_s:g}s"
    )


def _read_status(path: Path) -> dict[str, Any] | None:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return payload if isinstance(payload, dict) else None


def wait_for_status(
    path: Path,
    *,
    mode: str,
    active_seconds: float,
    timeout_s: float,
    max_age_s: float,
    launcher: subprocess.Popen[Any] | None,
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if launcher is not None and launcher.poll() is not None:
            raise VideoCaptureError(
                f"Matrix launcher exited before SONIC became ready: {launcher.returncode}"
            )
        status = _read_status(path)
        try:
            status_age = time.time() - path.stat().st_mtime
        except OSError:
            status_age = math.inf
        if status is not None and 0.0 <= status_age <= max_age_s:
            active = bool(status.get("active_lowcmd"))
            try:
                elapsed = float(status.get("active_elapsed_s", 0.0))
            except (TypeError, ValueError):
                elapsed = 0.0
            if mode == "status" and active:
                return status
            if mode == "active" and active and elapsed >= active_seconds:
                return status
        time.sleep(0.25)
    raise VideoCaptureError(
        f"fresh SONIC status did not reach {mode!r} readiness within {timeout_s:g}s: {path}"
    )


def resolve_ffmpeg(explicit: Path | None) -> Path:
    configured = explicit or (
        Path(os.environ["MATRIX_FFMPEG"])
        if os.environ.get("MATRIX_FFMPEG")
        else None
    )
    if configured is not None:
        candidate = configured.expanduser().resolve()
        if not candidate.is_file() or not os.access(candidate, os.X_OK):
            raise VideoCaptureError(f"ffmpeg is not executable: {candidate}")
        return candidate
    system_ffmpeg = shutil.which("ffmpeg")
    if system_ffmpeg:
        return Path(system_ffmpeg).resolve()
    try:
        import imageio_ffmpeg
    except ImportError as exc:
        raise VideoCaptureError(
            "ffmpeg is unavailable; install research/sonic_integration/requirements-trna.txt "
            "or pass MATRIX_FFMPEG=/path/to/ffmpeg"
        ) from exc
    candidate = Path(imageio_ffmpeg.get_ffmpeg_exe()).resolve()
    if not candidate.is_file() or not os.access(candidate, os.X_OK):
        raise VideoCaptureError(f"imageio-ffmpeg binary is not executable: {candidate}")
    return candidate


def _ffmpeg_version(ffmpeg: Path) -> str:
    result = _run_text([str(ffmpeg), "-version"])
    return result.stdout.splitlines()[0] if result.stdout else "unknown"


def _encoder_arguments(
    encoder: str,
    *,
    crf: int,
    x264_preset: str,
    nvenc_preset: str,
) -> list[str]:
    if encoder == "libx264":
        return ["-c:v", "libx264", "-preset", x264_preset, "-crf", str(crf)]
    if encoder == "h264_nvenc":
        return [
            "-c:v",
            "h264_nvenc",
            "-preset",
            nvenc_preset,
            "-rc",
            "vbr",
            "-cq",
            str(crf),
            "-b:v",
            "0",
        ]
    raise VideoCaptureError(f"unsupported encoder: {encoder}")


def _encoder_works(ffmpeg: Path, encoder_args: Sequence[str]) -> tuple[bool, str]:
    command = [
        str(ffmpeg),
        "-hide_banner",
        "-loglevel",
        "error",
        "-f",
        "lavfi",
        "-i",
        "color=size=64x64:rate=1",
        "-frames:v",
        "1",
        *encoder_args,
        "-f",
        "null",
        "-",
    ]
    result = _run_text(command, check=False)
    return result.returncode == 0, result.stderr.strip()


def select_encoder(
    ffmpeg: Path,
    requested: str,
    *,
    crf: int,
    x264_preset: str,
    nvenc_preset: str,
) -> tuple[str, list[str], str]:
    candidates = (
        ("h264_nvenc", "libx264") if requested == "auto" else (requested,)
    )
    errors: list[str] = []
    for candidate in candidates:
        arguments = _encoder_arguments(
            candidate,
            crf=crf,
            x264_preset=x264_preset,
            nvenc_preset=nvenc_preset,
        )
        works, error = _encoder_works(ffmpeg, arguments)
        if works:
            reason = (
                "auto-selected after runtime encoder probe"
                if requested == "auto"
                else "explicit encoder passed runtime probe"
            )
            return candidate, arguments, reason
        errors.append(f"{candidate}: {error or 'probe failed'}")
    raise VideoCaptureError("no usable H.264 encoder: " + "; ".join(errors))


def build_capture_command(
    ffmpeg: Path,
    output: Path,
    *,
    window: WindowInfo,
    display: str,
    duration_s: float,
    fps: float,
    draw_mouse: bool,
    encoder_args: Sequence[str],
) -> list[str]:
    return [
        str(ffmpeg),
        "-hide_banner",
        "-loglevel",
        "warning",
        "-y",
        "-nostdin",
        "-f",
        "x11grab",
        "-framerate",
        f"{fps:g}",
        "-window_id",
        str(window.id_decimal),
        "-draw_mouse",
        "1" if draw_mouse else "0",
        "-i",
        display,
        "-t",
        f"{duration_s:g}",
        *encoder_args,
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(output),
    ]


def parse_ffmpeg_probe(text: str, progress: str) -> dict[str, Any]:
    duration_match = re.search(r"Duration:\s*(\d+):(\d+):(\d+(?:\.\d+)?)", text)
    video_line = next((line for line in text.splitlines() if "Video:" in line), "")
    codec_match = re.search(r"Video:\s*([^,\s]+)", video_line)
    pixel_match = re.search(r"Video:\s*[^,]+,\s*([^,(\s]+)", video_line)
    resolution_match = re.search(r"\b(\d{2,5})x(\d{2,5})\b", video_line)
    fps_match = re.search(r"([0-9]+(?:\.[0-9]+)?)\s+fps\b", video_line)
    if not all((duration_match, codec_match, pixel_match, resolution_match, fps_match)):
        raise VideoCaptureError(f"could not parse ffmpeg video probe: {video_line}")
    hours, minutes, seconds = duration_match.groups()
    duration_s = int(hours) * 3600 + int(minutes) * 60 + float(seconds)
    frames = [
        int(match.group(1))
        for match in re.finditer(r"^frame=(\d+)\s*$", progress, re.MULTILINE)
    ]
    return {
        "codec": codec_match.group(1),
        "pixel_format": pixel_match.group(1),
        "width": int(resolution_match.group(1)),
        "height": int(resolution_match.group(2)),
        "fps": float(fps_match.group(1)),
        "duration_s": duration_s,
        "decoded_frames": frames[-1] if frames else 0,
    }


def _parse_signal_stats(text: str) -> dict[str, float]:
    names = {
        "y_min": "YMIN",
        "y_avg": "YAVG",
        "y_max": "YMAX",
        "saturation_avg": "SATAVG",
    }
    values: dict[str, float] = {}
    for key, ffmpeg_name in names.items():
        match = re.search(
            rf"lavfi\.signalstats\.{ffmpeg_name}=([-+0-9.eE]+)", text
        )
        if match is None:
            raise VideoCaptureError(f"ffmpeg signalstats is missing {ffmpeg_name}")
        values[key] = float(match.group(1))
    return values


def inspect_video(
    ffmpeg: Path, path: Path, *, sample_fps: float
) -> VideoProbe:
    decode = _run_text(
        [
            str(ffmpeg),
            "-hide_banner",
            "-nostats",
            "-progress",
            "pipe:1",
            "-i",
            str(path),
            "-map",
            "0:v:0",
            "-f",
            "null",
            "-",
        ]
    )
    parsed = parse_ffmpeg_probe(decode.stderr, decode.stdout)
    signal_result = _run_text(
        [
            str(ffmpeg),
            "-hide_banner",
            "-i",
            str(path),
            "-map",
            "0:v:0",
            "-frames:v",
            "1",
            "-vf",
            "signalstats,metadata=print",
            "-f",
            "null",
            "-",
        ]
    )
    signal_values = _parse_signal_stats(signal_result.stderr)
    hashes = _run_text(
        [
            str(ffmpeg),
            "-hide_banner",
            "-loglevel",
            "error",
            "-i",
            str(path),
            "-vf",
            f"fps={sample_fps:g},scale=64:36,format=gray",
            "-f",
            "framemd5",
            "-",
        ]
    )
    frame_hashes = [
        line.rsplit(",", 1)[-1].strip()
        for line in hashes.stdout.splitlines()
        if line and not line.startswith("#") and "," in line
    ]
    return VideoProbe(
        **parsed,
        **signal_values,
        sampled_frames=len(frame_hashes),
        unique_sample_hashes=len(set(frame_hashes)),
    )


def create_preview_strip(
    ffmpeg: Path,
    video: Path,
    output: Path,
    *,
    duration_s: float,
    frame_count: int = 5,
    frame_width: int = 384,
) -> None:
    if duration_s <= 0.0 or frame_count <= 0 or frame_width <= 0:
        raise VideoCaptureError("preview dimensions and duration must be positive")
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(f".{output.stem}.partial{output.suffix}")
    if temporary.exists():
        temporary.unlink()
    sample_fps = frame_count / duration_s
    _run_text(
        [
            str(ffmpeg),
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            "-i",
            str(video),
            "-vf",
            f"fps={sample_fps:.9g},scale={frame_width}:-1,tile={frame_count}x1",
            "-frames:v",
            "1",
            "-q:v",
            "2",
            str(temporary),
        ]
    )
    if not temporary.is_file() or temporary.stat().st_size == 0:
        raise VideoCaptureError("ffmpeg did not produce a preview strip")
    os.replace(temporary, output)


def evaluate_video_quality(
    probe: VideoProbe,
    *,
    requested_duration_s: float,
    requested_fps: float,
    window: WindowInfo,
    file_size: int,
    allow_static: bool,
    allow_short: bool,
) -> dict[str, Any]:
    failures: list[str] = []
    if (probe.width, probe.height) != (window.width, window.height):
        failures.append(
            f"resolution {probe.width}x{probe.height} != window {window.width}x{window.height}"
        )
    if not allow_short and probe.duration_s < requested_duration_s * 0.90:
        failures.append(
            f"duration {probe.duration_s:.3f}s < 90% of requested {requested_duration_s:.3f}s"
        )
    if probe.fps < requested_fps * 0.90:
        failures.append(
            f"stream fps {probe.fps:.3f} < 90% of requested {requested_fps:.3f}"
        )
    requested_frames = requested_duration_s * requested_fps
    if (
        not allow_short
        and requested_frames > 0.0
        and probe.decoded_frames < requested_frames * 0.90
    ):
        failures.append(
            f"decoded frames {probe.decoded_frames} < 90% of requested {requested_frames:.1f}"
        )
    stream_frames = probe.duration_s * probe.fps
    if stream_frames > 0.0 and probe.decoded_frames < stream_frames * 0.90:
        failures.append(
            f"decoded frames {probe.decoded_frames} < 90% of stream expectation "
            f"{stream_frames:.1f}"
        )
    if file_size < 16 * 1024:
        failures.append(f"video is too small: {file_size} bytes")
    if probe.y_avg <= 2.0 or probe.y_avg >= 253.0 or probe.y_max - probe.y_min < 8.0:
        failures.append(
            "first frame is visually uniform or clipped: "
            f"YMIN={probe.y_min:g} YAVG={probe.y_avg:g} YMAX={probe.y_max:g}"
        )
    if (
        not allow_static
        and requested_duration_s >= 1.0
        and probe.sampled_frames >= 2
        and probe.unique_sample_hashes < 2
    ):
        failures.append("sampled video frames are static")
    return {
        "passed": not failures,
        "failures": failures,
        "resolution_matches_window": (probe.width, probe.height)
        == (window.width, window.height),
        "duration_ratio": round(probe.duration_s / requested_duration_s, 6),
        "fps_ratio": round(probe.fps / requested_fps, 6),
        "decoded_frame_ratio": round(probe.decoded_frames / requested_frames, 6),
    }


def _git_info(repo_root: Path) -> dict[str, Any]:
    def git(*arguments: str) -> str:
        result = _run_text(["git", "-C", str(repo_root), *arguments])
        return result.stdout.strip()

    status = git("status", "--porcelain")
    return {
        "commit": git("rev-parse", "HEAD"),
        "branch": git("branch", "--show-current"),
        "dirty": bool(status),
        "dirty_path_count": len(status.splitlines()) if status else 0,
    }


def _source_provenance(
    repo_root: Path,
    launcher_command: Sequence[str],
    status: dict[str, Any] | None,
) -> dict[str, dict[str, Any]]:
    candidates: dict[str, Path] = {
        "recorder_python": repo_root / "scripts" / "record_matrix_sonic_video.py",
        "recorder_shell": repo_root / "scripts" / "record_matrix_sonic_video.sh",
        "matrix_sonic_launcher": repo_root / "scripts" / "run_matrix_sonic.sh",
        "matrix_launcher": repo_root / "scripts" / "run_sim.sh",
        "matrix_sonic_runtime": repo_root / "scripts" / "run_matrix_sonic.py",
        "urdf_visual_material_pipeline": repo_root
        / "scripts"
        / "apply_urdf_visual_materials.py",
        "trna_requirements": repo_root
        / "research"
        / "sonic_integration"
        / "requirements-trna.txt",
    }
    for argument in launcher_command:
        candidate = Path(argument).expanduser()
        if not candidate.is_absolute():
            candidate = repo_root / candidate
        if candidate.is_file() and candidate.suffix in {".py", ".sh"}:
            candidates["launcher_entrypoint"] = candidate
            break
    if any(Path(argument).name == "run_matrix_sonic_urban_v1.sh" for argument in launcher_command):
        candidates["urban_v1_contract"] = (
            repo_root / "research" / "urban_v1" / "scene.json"
        )
    for index, argument in enumerate(launcher_command[:-1]):
        if argument == "--urdf":
            urdf = Path(launcher_command[index + 1]).expanduser()
            candidates["visual_urdf"] = (
                urdf if urdf.is_absolute() else repo_root / urdf
            )
            break
    if status is not None and status.get("model"):
        candidates["physics_model"] = Path(str(status["model"])).expanduser()

    provenance: dict[str, dict[str, Any]] = {}
    for key, candidate in candidates.items():
        path = candidate.resolve()
        if path.is_file():
            provenance[key] = {
                "path": str(path),
                "size_bytes": path.stat().st_size,
                "sha256": _sha256(path),
            }
    return provenance


def _terminate_process(process: subprocess.Popen[Any], *, interrupt: bool) -> int | None:
    if process.poll() is not None:
        return process.returncode
    signal_to_send = signal.SIGINT if interrupt else signal.SIGTERM
    try:
        os.killpg(process.pid, signal_to_send)
    except ProcessLookupError:
        return process.poll()
    try:
        return process.wait(timeout=15.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        return process.wait(timeout=5.0)


def _command_string(command: Sequence[str]) -> str:
    return shlex.join(str(item) for item in command)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    repo_root = Path(__file__).resolve().parents[1]
    output = args.output.expanduser().resolve()
    if output.suffix.lower() != ".mp4":
        raise SystemExit("--output must use the .mp4 extension")
    metadata_path = (
        args.metadata.expanduser().resolve()
        if args.metadata is not None
        else output.with_suffix(".json")
    )
    status_file = (
        args.status_file.expanduser().resolve()
        if args.status_file is not None
        else repo_root / "outputs" / "matrix_sonic_status.json"
    )
    xauthority = (
        args.xauthority.expanduser().resolve()
        if args.xauthority is not None
        else _default_xauthority()
    )
    if not xauthority.is_file():
        raise SystemExit(f"Xauthority file does not exist: {xauthority}")
    if shutil.which("xwininfo") is None:
        raise SystemExit("xwininfo is required to discover the Matrix UE window")
    x11_env = _x11_environment(args.display, xauthority)

    launcher: subprocess.Popen[Any] | None = None
    launcher_log_stream = None
    ffmpeg_process: subprocess.Popen[Any] | None = None
    ffmpeg_log_stream = None
    launcher_stopped_by_recorder = False
    output.parent.mkdir(parents=True, exist_ok=True)
    partial = output.with_name(f".{output.stem}.partial.mp4")
    rejected = output.with_name(f"{output.stem}.rejected.mp4")
    launch_log = output.with_suffix(".launch.log")
    ffmpeg_log = output.with_suffix(".ffmpeg.log")
    for stale in (partial, rejected):
        if stale.exists():
            stale.unlink()

    session_started_at = datetime.now(timezone.utc)
    recording_started_at: datetime | None = None
    recording_finished_at: datetime | None = None
    status_before: dict[str, Any] | None = None
    status_after: dict[str, Any] | None = None
    final_video = partial
    try:
        ffmpeg = resolve_ffmpeg(args.ffmpeg)
        encoder, encoder_args, encoder_reason = select_encoder(
            ffmpeg,
            args.encoder,
            crf=args.crf,
            x264_preset=args.x264_preset,
            nvenc_preset=args.nvenc_preset,
        )

        if args.attach:
            launcher_command: list[str] = []
        else:
            launcher_command = [str(item) for item in args.launcher]
            if status_file.exists():
                status_file.unlink()
            launch_environment = os.environ.copy()
            launch_environment["MATRIX_SONIC_STATUS_FILE"] = str(status_file)
            launch_environment["DISPLAY"] = args.display
            launch_environment["XAUTHORITY"] = str(xauthority)
            launch_log.parent.mkdir(parents=True, exist_ok=True)
            launcher_log_stream = launch_log.open("w", encoding="utf-8")
            launcher = subprocess.Popen(
                launcher_command,
                cwd=repo_root,
                env=launch_environment,
                stdout=launcher_log_stream,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                text=True,
            )
            print(
                f"[matrix-video] launcher pid={launcher.pid} log={launch_log}",
                flush=True,
            )

        if args.window_id:
            window = _window_by_id(args.window_id, env=x11_env)
        else:
            window = wait_for_window(
                args.window_title_regex,
                env=x11_env,
                timeout_s=args.window_timeout,
                launcher=launcher,
            )
        print(
            "[matrix-video] window "
            f"id={window.id_hex} title={window.title!r} "
            f"size={window.width}x{window.height} pos={window.x},{window.y}",
            flush=True,
        )

        if args.ready == "window":
            status_before = _read_status(status_file)
        else:
            status_before = wait_for_status(
                status_file,
                mode=args.ready,
                active_seconds=args.ready_active_seconds,
                timeout_s=args.ready_timeout,
                max_age_s=args.status_max_age,
                launcher=launcher,
            )
            print(
                "[matrix-video] SONIC ready "
                f"active_elapsed_s={status_before.get('active_elapsed_s')} "
                f"physics_hz={status_before.get('physics_step_hz')} "
                f"rtf={status_before.get('rtf')}",
                flush=True,
            )

        capture_command = build_capture_command(
            ffmpeg,
            partial,
            window=window,
            display=args.display,
            duration_s=args.duration,
            fps=args.fps,
            draw_mouse=args.draw_mouse,
            encoder_args=encoder_args,
        )
        ffmpeg_log_stream = ffmpeg_log.open("w", encoding="utf-8")
        recording_started_at = datetime.now(timezone.utc)
        ffmpeg_process = subprocess.Popen(
            capture_command,
            env=x11_env,
            stdout=ffmpeg_log_stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            text=True,
        )
        print(
            f"[matrix-video] recording {args.duration:g}s at {args.fps:g}fps "
            f"encoder={encoder} output={output}",
            flush=True,
        )
        while ffmpeg_process.poll() is None:
            if launcher is not None and launcher.poll() is not None:
                _terminate_process(ffmpeg_process, interrupt=True)
                raise VideoCaptureError(
                    f"Matrix launcher exited during capture: {launcher.returncode}"
                )
            time.sleep(0.2)
        if ffmpeg_process.returncode != 0:
            raise VideoCaptureError(
                f"ffmpeg capture failed with code {ffmpeg_process.returncode}; see {ffmpeg_log}"
            )
        recording_finished_at = datetime.now(timezone.utc)
        ffmpeg_process = None
        if ffmpeg_log_stream is not None:
            ffmpeg_log_stream.close()
            ffmpeg_log_stream = None

        status_after = _read_status(status_file)
        if launcher is not None and not args.keep_running:
            launcher_stopped_by_recorder = True
            _terminate_process(launcher, interrupt=False)

        probe = inspect_video(
            ffmpeg, partial, sample_fps=args.quality_sample_fps
        )
        quality = evaluate_video_quality(
            probe,
            requested_duration_s=args.duration,
            requested_fps=args.fps,
            window=window,
            file_size=partial.stat().st_size,
            allow_static=args.allow_static,
            allow_short=args.allow_short,
        )
        if quality["passed"]:
            os.replace(partial, output)
            final_video = output
        else:
            os.replace(partial, rejected)
            final_video = rejected

        preview = final_video.with_name(f"{final_video.stem}.preview.jpg")
        create_preview_strip(
            ffmpeg,
            final_video,
            preview,
            duration_s=probe.duration_s,
        )

        metadata_finished_at = datetime.now(timezone.utc)
        metadata = {
            "schema_version": 1,
            "capture": {
                "session_started_at": session_started_at.isoformat(),
                "recording_started_at": recording_started_at.isoformat(),
                "recording_finished_at": recording_finished_at.isoformat(),
                "metadata_finished_at": metadata_finished_at.isoformat(),
                "host": socket.gethostname(),
                "mode": "attach" if args.attach else "launch",
                "requested_duration_s": args.duration,
                "requested_fps": args.fps,
                "draw_mouse": bool(args.draw_mouse),
                "notes": args.notes,
            },
            "repository": {
                "root": str(repo_root),
                **_git_info(repo_root),
                "source_files": _source_provenance(
                    repo_root, launcher_command, status_after or status_before
                ),
            },
            "launcher": {
                "command": launcher_command,
                "command_string": _command_string(launcher_command),
                "log": str(launch_log) if launcher_command else None,
                "pid": launcher.pid if launcher is not None else None,
                "return_code": launcher.poll() if launcher is not None else None,
                "stopped_by_recorder": launcher_stopped_by_recorder,
                "keep_running": bool(args.keep_running),
            },
            "window": asdict(window),
            "ffmpeg": {
                "path": str(ffmpeg),
                "version": _ffmpeg_version(ffmpeg),
                "encoder": encoder,
                "encoder_reason": encoder_reason,
                "encoder_arguments": encoder_args,
                "capture_command": capture_command,
                "log": str(ffmpeg_log),
            },
            "video": {
                "path": str(final_video),
                "accepted_path": str(output),
                "sha256": _sha256(final_video),
                "size_bytes": final_video.stat().st_size,
                "preview": {
                    "path": str(preview),
                    "sha256": _sha256(preview),
                    "size_bytes": preview.stat().st_size,
                },
                **asdict(probe),
            },
            "quality": quality,
            "sonic_status": {
                "path": str(status_file),
                "readiness": args.ready,
                "ready_active_seconds": args.ready_active_seconds,
                "before": status_before,
                "after": status_after,
            },
        }
        _atomic_json(metadata_path, metadata)
        print(
            f"[matrix-video] metadata={metadata_path} quality={quality['passed']} "
            f"sha256={metadata['video']['sha256']}",
            flush=True,
        )
        if not quality["passed"]:
            raise VideoCaptureError(
                f"video failed quality gates and was kept at {rejected}: "
                + "; ".join(quality["failures"])
            )
        print(f"[matrix-video] wrote {output}", flush=True)
        return 0
    except KeyboardInterrupt:
        print("[matrix-video] interrupted", file=sys.stderr)
        return 130
    except (OSError, subprocess.SubprocessError, VideoCaptureError) as exc:
        print(f"[matrix-video] ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        if ffmpeg_process is not None and ffmpeg_process.poll() is None:
            _terminate_process(ffmpeg_process, interrupt=True)
        if ffmpeg_log_stream is not None:
            ffmpeg_log_stream.close()
        if launcher is not None and not args.keep_running and launcher.poll() is None:
            _terminate_process(launcher, interrupt=False)
        if launcher_log_stream is not None:
            launcher_log_stream.close()


if __name__ == "__main__":
    raise SystemExit(main())
