#!/usr/bin/env python3
"""Fail closed unless Matrix scene6 video, replay, and restore all passed."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
from typing import Any, Sequence


class PostflightError(RuntimeError):
    pass


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _file(path: Path, *, label: str) -> Path:
    path = path.expanduser()
    if path.is_symlink() or not path.is_file():
        raise PostflightError(f"{label} must be a regular non-symlink file: {path}")
    return path.resolve()


def _json(path: Path, *, label: str) -> tuple[Path, dict[str, Any]]:
    path = _file(path, label=label)
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise PostflightError(f"invalid {label}: {exc}") from exc
    if not isinstance(payload, dict):
        raise PostflightError(f"{label} root must be an object")
    return path, payload


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    if path.is_symlink() or path.is_dir():
        raise PostflightError(
            f"postflight receipt must not be a symlink or directory: {path}"
        )
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


def _git_identity(matrix_root: Path) -> dict[str, Any]:
    matrix_root = matrix_root.expanduser().resolve()
    if not (matrix_root / ".git").exists():
        raise PostflightError(f"Matrix root is not a Git checkout: {matrix_root}")

    def git(*arguments: str) -> str:
        try:
            result = subprocess.run(
                ["git", "-C", str(matrix_root), *arguments],
                check=True,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=15.0,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            raise PostflightError(f"could not inspect Matrix Git state: {exc}") from exc
        return result.stdout.strip()

    status = git("status", "--porcelain", "--untracked-files=all")
    if status:
        raise PostflightError("Matrix checkout is dirty after replay:\n" + status)
    return {
        "root": str(matrix_root),
        "commit": git("rev-parse", "HEAD"),
        "branch": git("branch", "--show-current"),
        "clean": True,
    }


def _udp_9999_inodes() -> set[str]:
    inodes: set[str] = set()
    for protocol in ("udp", "udp6"):
        try:
            lines = Path(f"/proc/net/{protocol}").read_text(
                encoding="ascii", errors="strict"
            ).splitlines()[1:]
        except OSError:
            continue
        for line in lines:
            fields = line.split()
            if len(fields) < 10:
                continue
            try:
                port = int(fields[1].rsplit(":", 1)[1], 16)
            except (IndexError, ValueError):
                continue
            if port == 9999:
                inodes.add(fields[9])
    return inodes


def _residual_matrix_processes() -> list[dict[str, Any]]:
    patterns = (
        "zsibot_mujoco_ue",
        "replay_matrix_physics_trace.py",
        "supervise_matrix_ue.py",
        "robot_mujoco",
        "mc_ctrl",
        "scripts/run_sim.sh",
        "run_matrix_scene6_trace_replay.sh",
    )
    residual: list[dict[str, Any]] = []
    current_pid = os.getpid()
    excluded_pids = {current_pid}
    ancestor_pid = os.getppid()
    while ancestor_pid > 1 and ancestor_pid not in excluded_pids:
        excluded_pids.add(ancestor_pid)
        try:
            fields = Path(f"/proc/{ancestor_pid}/stat").read_text(
                encoding="utf-8"
            ).split()
            ancestor_pid = int(fields[3])
        except (OSError, IndexError, ValueError):
            break
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit() or int(proc.name) in excluded_pids:
            continue
        try:
            command = (proc / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                "utf-8", errors="replace"
            ).strip()
        except OSError:
            continue
        if command and any(pattern in command for pattern in patterns):
            residual.append({"pid": int(proc.name), "command": command[:512]})
    return sorted(residual, key=lambda item: item["pid"])


def verify(
    *,
    output: Path,
    metadata_path: Path,
    summary_path: Path,
    restore_path: Path,
    matrix_root: Path,
) -> dict[str, Any]:
    output = _file(output, label="accepted MP4")
    metadata_path, metadata = _json(metadata_path, label="video metadata")
    summary_path, summary = _json(summary_path, label="replay summary")
    restore_path, restore = _json(restore_path, label="model restore receipt")

    if summary.get("schema_id") != "matrix.physics_trace_replay.summary.v1":
        raise PostflightError("unexpected replay summary schema")
    if summary.get("passed") is not True:
        raise PostflightError(f"trace replay did not pass: {summary.get('failure')}")
    if summary.get("completion") != "scheduled_replay_complete":
        raise PostflightError("trace replay did not complete its scheduled final hold")
    if summary.get("physics_execution") != "offline_mujoco_persistent_world":
        raise PostflightError("replay summary physics boundary drifted")
    if summary.get("render_mode") != "matrix_ue_trace_replay":
        raise PostflightError("replay summary render boundary drifted")
    if summary.get("dimensions") != {"nq": 57, "nv": 55, "nu": 43}:
        raise PostflightError("replay dimensions are not 57/55/43")
    frame_count = summary.get("source_frame_count")
    packets = summary.get("packets")
    if (
        isinstance(frame_count, bool)
        or not isinstance(frame_count, int)
        or frame_count <= 0
        or not isinstance(packets, dict)
        or packets.get("trace_sent") != frame_count
        or packets.get("sent") != packets.get("expected")
    ):
        raise PostflightError("replay did not complete every scheduled packet")

    if restore.get("schema_id") != "matrix.physics_trace_model_stage.v1":
        raise PostflightError("unexpected model restore schema")
    if restore.get("active") is not False or restore.get("phase") != "restored":
        raise PostflightError("Matrix model/runtime transaction was not restored")
    if set(restore.get("restored_targets") or []) != {"mujoco", "ue"}:
        raise PostflightError("both Matrix current.xml targets were not restored")
    restored_runtime = restore.get("restored_runtime_files")
    runtime_inventory = restore.get("runtime_files")
    if (
        not isinstance(restored_runtime, list)
        or not isinstance(runtime_inventory, dict)
        or set(restored_runtime) != set(runtime_inventory)
    ):
        raise PostflightError("Matrix run_sim mutation targets were not all restored")
    if (restore.get("trace") or {}).get("sha256") != (
        summary.get("trace") or {}
    ).get("sha256"):
        raise PostflightError("restore receipt and replay summary trace hashes differ")
    if (restore.get("scene_model") or {}).get("sha256") != (
        summary.get("scene_model") or {}
    ).get("sha256"):
        raise PostflightError("restore receipt and replay summary model hashes differ")
    if (restore.get("robot_model") or {}).get("sha256") != (
        summary.get("model") or {}
    ).get("sha256"):
        raise PostflightError("restore receipt and replay render-model hashes differ")
    for label, digest in (
        ("trace", (summary.get("trace") or {}).get("sha256")),
        ("model", (summary.get("model") or {}).get("sha256")),
        ("scene model", (summary.get("scene_model") or {}).get("sha256")),
    ):
        if (
            not isinstance(digest, str)
            or len(digest) != 64
            or any(character not in "0123456789abcdef" for character in digest)
        ):
            raise PostflightError(f"invalid {label} SHA256")

    quality = metadata.get("quality")
    video = metadata.get("video")
    capture = metadata.get("capture")
    if not isinstance(quality, dict) or quality.get("passed") is not True:
        raise PostflightError("video quality gates did not pass")
    if not isinstance(video, dict) or not isinstance(capture, dict):
        raise PostflightError("video metadata is incomplete")
    if Path(str(video.get("path"))).resolve() != output:
        raise PostflightError("video metadata does not reference the accepted MP4")
    output_sha = _sha256(output)
    if video.get("sha256") != output_sha:
        raise PostflightError("accepted MP4 hash differs from video metadata")
    requested_fps = capture.get("requested_fps")
    observed_fps = video.get("fps")
    if (
        isinstance(requested_fps, bool)
        or not isinstance(requested_fps, (int, float))
        or not math.isclose(float(requested_fps), 25.0, abs_tol=1e-9)
        or isinstance(observed_fps, bool)
        or not isinstance(observed_fps, (int, float))
        or not math.isclose(float(observed_fps), 25.0, abs_tol=0.05)
    ):
        raise PostflightError("video is not verified at 25 FPS")
    for field in ("width", "height", "decoded_frames"):
        value = video.get(field)
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise PostflightError(f"video {field} must be a positive integer")
    duration = video.get("duration_s")
    if (
        isinstance(duration, bool)
        or not isinstance(duration, (int, float))
        or not math.isfinite(float(duration))
        or float(duration) <= 0.0
    ):
        raise PostflightError("video duration must be positive and finite")
    status_before = ((metadata.get("sonic_status") or {}).get("before") or {})
    if (
        status_before.get("active_lowcmd") is not True
        or status_before.get("active_lowcmd_semantics")
        != "legacy_recorder_readiness_gate_no_dds_lowcmd"
        or status_before.get("dds_lowcmd_active") is not False
    ):
        raise PostflightError("video readiness lacks the explicit no-DDS replay boundary")
    launcher = metadata.get("launcher")
    launcher_return_code = (
        launcher.get("return_code") if isinstance(launcher, dict) else None
    )
    if (
        not isinstance(launcher, dict)
        or isinstance(launcher_return_code, bool)
        or launcher_return_code != 0
        or launcher.get("stopped_by_recorder") is not False
    ):
        raise PostflightError("Matrix launcher did not finish naturally with code 0")
    status_after = ((metadata.get("sonic_status") or {}).get("after") or {})
    if (
        status_after.get("active_lowcmd") is not False
        or status_after.get("completed") is not True
        or status_after.get("passed") is not True
        or status_after.get("active_lowcmd_semantics")
        != "legacy_recorder_readiness_gate_no_dds_lowcmd"
        or status_after.get("dds_lowcmd_active") is not False
    ):
        raise PostflightError("Matrix replay final status is not complete and inactive")

    repository = _git_identity(matrix_root)
    udp_inodes = _udp_9999_inodes()
    if udp_inodes:
        raise PostflightError(
            "UDP receiver 9999 remains after Matrix replay: "
            + ",".join(sorted(udp_inodes))
        )
    residual_processes = _residual_matrix_processes()
    if residual_processes:
        raise PostflightError(
            "Matrix replay left residual processes: "
            + "; ".join(
                f"pid={item['pid']} {item['command']}" for item in residual_processes
            )
        )

    return {
        "schema_id": "matrix.scene6_twinbot_video_postflight.v1",
        "passed": True,
        "physics_execution": "offline_mujoco_persistent_world",
        "render_mode": "matrix_ue_trace_replay",
        "repository": repository,
        "runtime_cleanup": {
            "udp_9999_released": True,
            "residual_processes": [],
        },
        "video": {
            "path": str(output),
            "sha256": output_sha,
            "size_bytes": output.stat().st_size,
            "fps": float(observed_fps),
            "width": video.get("width"),
            "height": video.get("height"),
            "duration_s": video.get("duration_s"),
            "decoded_frames": video.get("decoded_frames"),
        },
        "trace": summary.get("trace"),
        "model": summary.get("model"),
        "replay_summary": {
            "path": str(summary_path),
            "sha256": _sha256(summary_path),
            "completion": summary.get("completion"),
            "source_frame_count": frame_count,
        },
        "restore_receipt": {
            "path": str(restore_path),
            "sha256": _sha256(restore_path),
        },
        "video_metadata": {
            "path": str(metadata_path),
            "sha256": _sha256(metadata_path),
        },
    }


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--metadata", type=Path, required=True)
    parser.add_argument("--replay-summary", type=Path, required=True)
    parser.add_argument("--restore-receipt", type=Path, required=True)
    parser.add_argument("--matrix-root", type=Path, required=True)
    parser.add_argument("--receipt", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    try:
        receipt = verify(
            output=args.output,
            metadata_path=args.metadata,
            summary_path=args.replay_summary,
            restore_path=args.restore_receipt,
            matrix_root=args.matrix_root,
        )
        _atomic_json(args.receipt.expanduser().resolve(), receipt)
    except (OSError, ValueError, PostflightError) as exc:
        print(f"[matrix-scene6-video] ERROR: {exc}", file=os.sys.stderr)
        return 2
    print(
        "[matrix-scene6-video] verified "
        f"video={receipt['video']['path']} sha256={receipt['video']['sha256']}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
