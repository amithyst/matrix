#!/usr/bin/env python3
"""Create and validate current-run Matrix scene6 camera evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
import time
from typing import Any, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if os.fspath(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, os.fspath(SCRIPT_DIR))

import matrix_ue_overlay as overlay  # noqa: E402


RECEIPT_SCHEMA = "matrix.scene6_camera_receipt.v1"
READY_SCHEMA = "matrix.scene6_camera_ready.v1"
MODES = {"robot", "spectator-overlay"}
VIEW_CLASSES = {
    "robot": "MujocoSim_Custom_C",
    "spectator-overlay": "Spectator_C",
}
FRAMING_LABEL_RE = re.compile(r"[a-z0-9][a-z0-9-]{0,63}\Z")
RECEIPT_KEYS = {
    "schema_id",
    "mode",
    "requested_view_class",
    "spring_arm_cm",
    "ue_exec_cmds",
    "camera_commands",
    "overlay",
    "camera_ready",
    "created_unix_ns",
}
READY_KEYS = {
    "schema_id",
    "ready",
    "mode",
    "framing_label",
    "created_unix_ns",
}
OVERLAY_KEYS = {
    "overlay_id",
    "overlay_version",
    "contract",
    "bundle",
    "active_directory",
    "mount",
}
CONTRACT_EVIDENCE_KEYS = {"path", "sha256"}
BUNDLE_EVIDENCE_KEYS = {"path", "artifacts"}
ARTIFACT_KEYS = {"name", "size", "sha256"}
MOUNT_EVIDENCE_KEYS = {
    "ue_log",
    "start_offset",
    "end_offset",
    "segment_size",
    "segment_sha256",
    "found_line",
    "mounted_line",
}
SHA256_RE = re.compile(r"[0-9a-f]{64}\Z")
VIEWCLASS_COMMAND_RE = re.compile(
    r"viewclass[ \t]+(?P<view_class>[A-Za-z_][A-Za-z0-9_]{0,126}_C)[ \t]*\Z",
    re.IGNORECASE,
)
TARGET_ARM_COMMAND_RE = re.compile(
    r"set[ \t]+Engine\.SpringArmComponent[ \t]+TargetArmLength[ \t]+"
    r"(?P<distance>(?:0|[1-9][0-9]*)(?:\.[0-9]+)?)[ \t]*\Z",
    re.IGNORECASE,
)
VIEWCLASS_PREFIX_RE = re.compile(r"viewclass\b", re.IGNORECASE)
SPRING_ARM_REFERENCE_RE = re.compile(
    r"Engine\.SpringArmComponent\b", re.IGNORECASE
)
TARGET_ARM_REFERENCE_RE = re.compile(r"TargetArmLength\b", re.IGNORECASE)
SET_PREFIX_RE = re.compile(r"set\b", re.IGNORECASE)


class CameraReceiptError(RuntimeError):
    """Camera evidence is absent, stale, or inconsistent."""


def _reject_duplicate_keys(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise CameraReceiptError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _regular_file(path: Path, *, label: str) -> Path:
    if not path.is_absolute():
        raise CameraReceiptError(f"{label} must be absolute: {path}")
    overlay._require_regular_file(path, label)
    return path


def _directory(path: Path, *, label: str) -> Path:
    if not path.is_absolute():
        raise CameraReceiptError(f"{label} must be absolute: {path}")
    overlay._require_directory(path, label)
    return path


def _load_json(path: Path, *, label: str, maximum_size: int = 128 * 1024) -> dict:
    path = _regular_file(path, label=label)
    if path.stat().st_size > maximum_size:
        raise CameraReceiptError(f"{label} is unexpectedly large: {path}")
    try:
        value = json.loads(
            path.read_text(encoding="utf-8"),
            object_pairs_hook=_reject_duplicate_keys,
            parse_constant=lambda value: (_ for _ in ()).throw(
                CameraReceiptError(f"non-finite JSON constant: {value}")
            ),
        )
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise CameraReceiptError(f"cannot parse {label}: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise CameraReceiptError(f"{label} root must be an object")
    return value


def _output_path(path: Path) -> Path:
    if not path.is_absolute():
        raise CameraReceiptError(f"output path must be absolute: {path}")
    if path.is_symlink() or path.is_dir():
        raise CameraReceiptError(f"output must not be a symlink or directory: {path}")
    _directory(path.parent, label="output parent")
    return path


def _atomic_json(path: Path, payload: dict) -> None:
    path = _output_path(path)
    temporary = path.with_name(f".{path.name}.tmp.{os.getpid()}")
    if temporary.exists() or temporary.is_symlink():
        raise CameraReceiptError(f"temporary output already exists: {temporary}")
    try:
        temporary.write_text(
            json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        os.chmod(temporary, 0o644)
        os.replace(temporary, path)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def _positive_timestamp(value: object, *, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise CameraReceiptError(f"{label} must be a positive integer")
    return value


def _absolute_string(value: object, *, label: str) -> str:
    if not isinstance(value, str) or not Path(value).is_absolute():
        raise CameraReceiptError(f"{label} must be an absolute path string")
    return value


def _mode(value: object) -> str:
    if not isinstance(value, str) or value not in MODES:
        raise CameraReceiptError(f"camera mode must be one of {sorted(MODES)}")
    return value


def validate_ready_payload(payload: object, *, expected_mode: str) -> dict:
    if not isinstance(payload, dict) or set(payload) != READY_KEYS:
        raise CameraReceiptError("camera-ready payload has an invalid schema")
    mode = _mode(payload.get("mode"))
    if mode != expected_mode or payload.get("ready") is not True:
        raise CameraReceiptError("camera-ready payload does not match this launch")
    label = payload.get("framing_label")
    if not isinstance(label, str) or FRAMING_LABEL_RE.fullmatch(label) is None:
        raise CameraReceiptError("camera-ready framing_label is invalid")
    _positive_timestamp(payload.get("created_unix_ns"), label="created_unix_ns")
    if payload.get("schema_id") != READY_SCHEMA:
        raise CameraReceiptError("unexpected camera-ready schema")
    return payload


def load_ready(path: Path, *, expected_mode: str) -> dict:
    return validate_ready_payload(
        _load_json(path, label="camera-ready file"), expected_mode=expected_mode
    )


def confirm_ready(*, output: Path, mode: str, framing_label: str) -> dict:
    payload = {
        "schema_id": READY_SCHEMA,
        "ready": True,
        "mode": _mode(mode),
        "framing_label": framing_label,
        "created_unix_ns": time.time_ns(),
    }
    validate_ready_payload(payload, expected_mode=mode)
    _atomic_json(output, payload)
    return payload


def _camera_commands(ue_exec_cmds: str, *, mode: str, distance_cm: float) -> list[str]:
    if not isinstance(ue_exec_cmds, str) or not ue_exec_cmds:
        raise CameraReceiptError("UE ExecCmds must be non-empty")
    if any(separator in ue_exec_cmds for separator in ("\r", "\n", ";", "|")):
        raise CameraReceiptError("UE ExecCmds contain an unsupported command separator")
    ordered = [command.strip() for command in ue_exec_cmds.split(",") if command.strip()]
    camera_commands = [
        command
        for command in ordered
        if SPRING_ARM_REFERENCE_RE.search(command)
        or VIEWCLASS_PREFIX_RE.match(command)
    ]
    view_matches: list[tuple[str, re.Match[str]]] = []
    arm_matches: list[tuple[str, re.Match[str]]] = []
    for command in camera_commands:
        view_match = VIEWCLASS_COMMAND_RE.fullmatch(command)
        if VIEWCLASS_PREFIX_RE.match(command):
            if view_match is None:
                raise CameraReceiptError(
                    f"unsupported camera-sensitive UE command: {command}"
                )
            view_matches.append((command, view_match))
        arm_match = TARGET_ARM_COMMAND_RE.fullmatch(command)
        if (
            SET_PREFIX_RE.match(command)
            and SPRING_ARM_REFERENCE_RE.search(command)
            and TARGET_ARM_REFERENCE_RE.search(command)
        ):
            if arm_match is None:
                raise CameraReceiptError(
                    f"unsupported camera-sensitive UE command: {command}"
                )
            arm_matches.append((command, arm_match))
    if (
        not view_matches
        or view_matches[-1][1].group("view_class").casefold()
        != VIEW_CLASSES[mode].casefold()
    ):
        raise CameraReceiptError("final UE viewclass differs from the requested mode")
    try:
        actual_distance = float(arm_matches[-1][1].group("distance"))
    except (IndexError, ValueError) as exc:
        raise CameraReceiptError("final UE camera arm command is missing") from exc
    if not math.isfinite(actual_distance) or not math.isclose(
        actual_distance, distance_cm, rel_tol=0.0, abs_tol=1e-9
    ):
        raise CameraReceiptError("final UE camera arm differs from the receipt")
    return camera_commands


def _bounded_mount_segment(
    *, ue_log: Path, start_offset: int, segment_size: int
) -> bytes:
    ue_log = _regular_file(ue_log, label="UE log")
    if (
        isinstance(start_offset, bool)
        or not isinstance(start_offset, int)
        or start_offset < 0
        or isinstance(segment_size, bool)
        or not isinstance(segment_size, int)
        or segment_size < 0
    ):
        raise CameraReceiptError("UE log segment bounds are invalid")
    with ue_log.open("rb") as stream:
        stream.seek(start_offset)
        segment = stream.read(segment_size)
    if len(segment) != segment_size:
        raise CameraReceiptError("UE log no longer contains the recorded mount segment")
    return segment


def _logged_path(line: str, pattern: re.Pattern[str]) -> str | None:
    match = pattern.fullmatch(line)
    if match is None:
        return None
    return match.group("path").strip().strip("\"'").replace("\\", "/")


def _mount_evidence(*, ue_log: Path, start_offset: int) -> dict:
    ue_log = _regular_file(ue_log, label="UE log")
    if isinstance(start_offset, bool) or start_offset < 0:
        raise CameraReceiptError("UE log start offset must be non-negative")
    if ue_log.stat().st_size < start_offset:
        raise CameraReceiptError("UE log was truncated after the launch boundary")
    with ue_log.open("rb") as stream:
        stream.seek(start_offset)
        segment_bytes = stream.read()
    segment = segment_bytes.decode("utf-8", errors="replace")
    stem_lines = [line for line in segment.splitlines() if overlay.STEM in line]
    if any("Failed" in line for line in stem_lines):
        raise CameraReceiptError("UE current-run overlay log contains Failed")
    prefix = r"^\s*(?:\[[^\]\r\n]*\]\s*)*LogPakFile:\s*Display:\s*"
    found_pattern = re.compile(
        prefix + r"Found Pak file (?P<path>.+?) attempting to mount\.?\s*$"
    )
    mounted_pattern = re.compile(
        prefix + r"Mounted IoStore container (?P<path>.+?)\s*$"
    )
    expected_directory = "/zsibot_mujoco_ue/Saved/Paks/MatrixCenteredCameraActive/"
    expected_found = expected_directory + f"{overlay.STEM}.pak"
    expected_mounted = expected_directory + f"{overlay.STEM}.utoc"
    found_lines = [
        line
        for line in stem_lines
        if (_logged_path(line, found_pattern) or "").endswith(expected_found)
    ]
    mounted_lines = [
        line
        for line in stem_lines
        if (_logged_path(line, mounted_pattern) or "").endswith(expected_mounted)
    ]
    if not found_lines or not mounted_lines:
        raise CameraReceiptError("UE log lacks exact active-directory mount evidence")
    return {
        "ue_log": os.fspath(ue_log),
        "start_offset": start_offset,
        "end_offset": start_offset + len(segment_bytes),
        "segment_size": len(segment_bytes),
        "segment_sha256": hashlib.sha256(segment_bytes).hexdigest(),
        "found_line": found_lines[-1],
        "mounted_line": mounted_lines[-1],
    }


def _revalidate_mount_evidence(payload: dict) -> None:
    ue_log = Path(_absolute_string(payload.get("ue_log"), label="UE log"))
    start_offset = payload.get("start_offset")
    segment_size = payload.get("segment_size")
    end_offset = payload.get("end_offset")
    if (
        isinstance(start_offset, bool)
        or not isinstance(start_offset, int)
        or isinstance(segment_size, bool)
        or not isinstance(segment_size, int)
        or isinstance(end_offset, bool)
        or not isinstance(end_offset, int)
        or start_offset < 0
        or segment_size < 0
        or end_offset != start_offset + segment_size
    ):
        raise CameraReceiptError("camera receipt mount bounds drifted")
    segment_bytes = _bounded_mount_segment(
        ue_log=ue_log, start_offset=start_offset, segment_size=segment_size
    )
    if hashlib.sha256(segment_bytes).hexdigest() != payload.get("segment_sha256"):
        raise CameraReceiptError("UE mount log segment SHA256 drifted")
    segment_lines = segment_bytes.decode("utf-8", errors="replace").splitlines()
    if any(
        overlay.STEM in line and "Failed" in line for line in segment_lines
    ):
        raise CameraReceiptError("UE mount log segment contains an overlay failure")
    found_line = payload.get("found_line")
    mounted_line = payload.get("mounted_line")
    if not isinstance(found_line, str) or not isinstance(mounted_line, str):
        raise CameraReceiptError("camera receipt mount lines are invalid")
    try:
        found_index = segment_lines.index(found_line)
        mounted_index = segment_lines.index(mounted_line, found_index + 1)
    except ValueError as exc:
        raise CameraReceiptError(
            "camera receipt mount lines are absent or out of order"
        ) from exc
    expected_directory = "/zsibot_mujoco_ue/Saved/Paks/MatrixCenteredCameraActive/"
    prefix = r"^\s*(?:\[[^\]\r\n]*\]\s*)*LogPakFile:\s*Display:\s*"
    found_pattern = re.compile(
        prefix + r"Found Pak file (?P<path>.+?) attempting to mount\.?\s*$"
    )
    mounted_pattern = re.compile(
        prefix + r"Mounted IoStore container (?P<path>.+?)\s*$"
    )
    if not (
        (_logged_path(found_line, found_pattern) or "").endswith(
            expected_directory + f"{overlay.STEM}.pak"
        )
        and (_logged_path(mounted_line, mounted_pattern) or "").endswith(
            expected_directory + f"{overlay.STEM}.utoc"
        )
        and found_index < mounted_index
    ):
        raise CameraReceiptError("camera receipt mount evidence path drifted")


def _overlay_evidence(
    *, project_root: Path, contract_path: Path, bundle_path: Path, ue_log: Path, log_offset: int
) -> dict:
    project_root = _directory(project_root, label="Matrix project root")
    contract = overlay.load_contract(contract_path)
    bundle = overlay.verify_bundle(bundle_path, contract)
    active = project_root / overlay.RUNTIME_DIRECTORY
    overlay._verify_directory(active, contract)
    artifacts = [
        {"name": artifact.name, "size": artifact.size, "sha256": artifact.sha256}
        for artifact in sorted(contract.artifacts, key=lambda value: value.name)
    ]
    return {
        "overlay_id": overlay.OVERLAY_ID,
        "overlay_version": overlay.OVERLAY_VERSION,
        "contract": {
            "path": os.fspath(contract.path),
            "sha256": _sha256(contract.path),
        },
        "bundle": {"path": os.fspath(bundle), "artifacts": artifacts},
        "active_directory": os.fspath(active),
        "mount": _mount_evidence(ue_log=ue_log, start_offset=log_offset),
    }


def validate_receipt_payload(payload: object) -> dict:
    if not isinstance(payload, dict) or set(payload) != RECEIPT_KEYS:
        raise CameraReceiptError("camera receipt has an invalid schema")
    if payload.get("schema_id") != RECEIPT_SCHEMA:
        raise CameraReceiptError("unexpected camera receipt schema")
    mode = _mode(payload.get("mode"))
    if payload.get("requested_view_class") != VIEW_CLASSES[mode]:
        raise CameraReceiptError("camera receipt viewclass is invalid")
    distance = payload.get("spring_arm_cm")
    if (
        isinstance(distance, bool)
        or not isinstance(distance, (int, float))
        or not math.isfinite(float(distance))
        or not 80.0 <= float(distance) <= 500.0
    ):
        raise CameraReceiptError("camera receipt arm distance is invalid")
    camera_commands = _camera_commands(
        payload.get("ue_exec_cmds"), mode=mode, distance_cm=float(distance)
    )
    if payload.get("camera_commands") != camera_commands:
        raise CameraReceiptError("camera receipt command projection drifted")
    if mode == "spectator-overlay":
        overlay_payload = payload.get("overlay")
        if not isinstance(overlay_payload, dict) or set(overlay_payload) != OVERLAY_KEYS:
            raise CameraReceiptError("spectator camera receipt lacks overlay evidence")
        if (
            overlay_payload.get("overlay_id") != overlay.OVERLAY_ID
            or overlay_payload.get("overlay_version") != overlay.OVERLAY_VERSION
        ):
            raise CameraReceiptError("camera receipt overlay identity drifted")
        contract_payload = overlay_payload.get("contract")
        bundle_payload = overlay_payload.get("bundle")
        mount_payload = overlay_payload.get("mount")
        if (
            not isinstance(contract_payload, dict)
            or set(contract_payload) != CONTRACT_EVIDENCE_KEYS
            or not isinstance(bundle_payload, dict)
            or set(bundle_payload) != BUNDLE_EVIDENCE_KEYS
            or not isinstance(mount_payload, dict)
            or set(mount_payload) != MOUNT_EVIDENCE_KEYS
        ):
            raise CameraReceiptError("camera receipt overlay evidence schema drifted")
        _absolute_string(contract_payload.get("path"), label="overlay contract")
        _absolute_string(bundle_payload.get("path"), label="overlay bundle")
        _absolute_string(
            overlay_payload.get("active_directory"), label="overlay active directory"
        )
        _absolute_string(mount_payload.get("ue_log"), label="UE log")
        if (
            not isinstance(contract_payload.get("sha256"), str)
            or SHA256_RE.fullmatch(contract_payload["sha256"]) is None
            or not isinstance(mount_payload.get("segment_sha256"), str)
            or SHA256_RE.fullmatch(mount_payload["segment_sha256"]) is None
            or isinstance(mount_payload.get("start_offset"), bool)
            or not isinstance(mount_payload.get("start_offset"), int)
            or mount_payload["start_offset"] < 0
            or isinstance(mount_payload.get("end_offset"), bool)
            or not isinstance(mount_payload.get("end_offset"), int)
            or isinstance(mount_payload.get("segment_size"), bool)
            or not isinstance(mount_payload.get("segment_size"), int)
            or mount_payload["segment_size"] < 0
            or mount_payload["end_offset"]
            != mount_payload["start_offset"] + mount_payload["segment_size"]
        ):
            raise CameraReceiptError("camera receipt overlay hashes/offset are invalid")
        artifacts = bundle_payload.get("artifacts")
        if not isinstance(artifacts, list) or len(artifacts) != 3:
            raise CameraReceiptError("camera receipt must bind three overlay artifacts")
        observed_artifacts = {}
        for artifact in artifacts:
            if not isinstance(artifact, dict) or set(artifact) != ARTIFACT_KEYS:
                raise CameraReceiptError("camera receipt overlay artifact schema drifted")
            name = artifact.get("name")
            size = artifact.get("size")
            digest = artifact.get("sha256")
            if (
                not isinstance(name, str)
                or isinstance(size, bool)
                or not isinstance(size, int)
                or not isinstance(digest, str)
            ):
                raise CameraReceiptError("camera receipt overlay artifact is invalid")
            observed_artifacts[name] = (size, digest)
        if observed_artifacts != overlay.PINNED_ARTIFACTS:
            raise CameraReceiptError("camera receipt overlay artifacts differ from pinned v3")
        expected_directory = "/zsibot_mujoco_ue/Saved/Paks/MatrixCenteredCameraActive/"
        if (
            expected_directory + f"{overlay.STEM}.pak"
            not in str(mount_payload.get("found_line"))
            or expected_directory + f"{overlay.STEM}.utoc"
            not in str(mount_payload.get("mounted_line"))
        ):
            raise CameraReceiptError("camera receipt mount evidence path drifted")
    elif payload.get("overlay") is not None:
        raise CameraReceiptError("robot camera receipt must not claim an overlay")
    ready_payload = payload.get("camera_ready")
    created_unix_ns = _positive_timestamp(
        payload.get("created_unix_ns"), label="created_unix_ns"
    )
    if ready_payload is not None:
        validate_ready_payload(ready_payload, expected_mode=mode)
        if ready_payload["created_unix_ns"] > created_unix_ns:
            raise CameraReceiptError("camera-ready confirmation postdates the receipt")
    return payload


def load_receipt(path: Path) -> dict:
    return validate_receipt_payload(_load_json(path, label="camera receipt"))


def revalidate_receipt_evidence(payload: object, *, project_root: Path) -> dict:
    """Re-open durable inputs and verify the current-run receipt after cleanup."""

    payload = validate_receipt_payload(payload)
    try:
        root = _directory(project_root.resolve(), label="Matrix project root")
    except (overlay.OverlayError, OSError) as exc:
        raise CameraReceiptError(f"Matrix project evidence is invalid: {exc}") from exc
    expected_active = root / overlay.RUNTIME_DIRECTORY
    if os.path.lexists(expected_active):
        raise CameraReceiptError("overlay active directory remains after cleanup")
    overlay_payload = payload.get("overlay")
    if overlay_payload is None:
        return payload

    contract_payload = overlay_payload["contract"]
    bundle_payload = overlay_payload["bundle"]
    contract_path = Path(contract_payload["path"])
    bundle_path = Path(bundle_payload["path"])
    try:
        contract = overlay.load_contract(contract_path)
        bundle = overlay.verify_bundle(bundle_path, contract)
    except (overlay.OverlayError, OSError) as exc:
        raise CameraReceiptError(f"overlay evidence no longer verifies: {exc}") from exc
    if _sha256(contract.path) != contract_payload["sha256"]:
        raise CameraReceiptError("overlay contract SHA256 drifted")
    expected_artifacts = [
        {"name": artifact.name, "size": artifact.size, "sha256": artifact.sha256}
        for artifact in sorted(contract.artifacts, key=lambda value: value.name)
    ]
    if os.fspath(bundle) != bundle_payload["path"]:
        raise CameraReceiptError("overlay bundle path drifted")
    if bundle_payload["artifacts"] != expected_artifacts:
        raise CameraReceiptError("overlay bundle evidence drifted")

    if overlay_payload["active_directory"] != os.fspath(expected_active):
        raise CameraReceiptError("overlay active-directory provenance drifted")
    expected_log = root / "src/UeSim/Linux/zsibot_mujoco_ue.log"
    if overlay_payload["mount"]["ue_log"] != os.fspath(expected_log):
        raise CameraReceiptError("camera receipt references an unexpected UE log")
    try:
        _revalidate_mount_evidence(overlay_payload["mount"])
    except (overlay.OverlayError, OSError) as exc:
        raise CameraReceiptError(f"UE mount evidence no longer verifies: {exc}") from exc
    return payload


def write_receipt(
    *,
    output: Path,
    mode: str,
    spring_arm_cm: float,
    ue_exec_cmds: str,
    project_root: Path,
    contract: Path | None,
    bundle: Path | None,
    ue_log: Path | None,
    log_offset: int | None,
    ready_file: Path | None,
) -> dict:
    mode = _mode(mode)
    if not math.isfinite(spring_arm_cm) or not 80.0 <= spring_arm_cm <= 500.0:
        raise CameraReceiptError("spring arm must be within 80..500 cm")
    camera_commands = _camera_commands(
        ue_exec_cmds, mode=mode, distance_cm=spring_arm_cm
    )
    if mode == "spectator-overlay":
        if any(value is None for value in (contract, bundle, ue_log, log_offset)):
            raise CameraReceiptError("spectator receipt requires overlay and log inputs")
        overlay_payload = _overlay_evidence(
            project_root=project_root,
            contract_path=contract,
            bundle_path=bundle,
            ue_log=ue_log,
            log_offset=log_offset,
        )
    else:
        if any(value is not None for value in (contract, bundle, ue_log, log_offset)):
            raise CameraReceiptError("robot receipt rejects overlay-only inputs")
        _directory(project_root, label="Matrix project root")
        overlay_payload = None
    ready_payload = load_ready(ready_file, expected_mode=mode) if ready_file else None
    payload = {
        "schema_id": RECEIPT_SCHEMA,
        "mode": mode,
        "requested_view_class": VIEW_CLASSES[mode],
        "spring_arm_cm": spring_arm_cm,
        "ue_exec_cmds": ue_exec_cmds,
        "camera_commands": camera_commands,
        "overlay": overlay_payload,
        "camera_ready": ready_payload,
        "created_unix_ns": time.time_ns(),
    }
    validate_receipt_payload(payload)
    _atomic_json(output, payload)
    return payload


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    confirm = subparsers.add_parser("confirm")
    confirm.add_argument("--output", type=Path, required=True)
    confirm.add_argument("--mode", choices=sorted(MODES), required=True)
    confirm.add_argument("--framing-label", required=True)

    inspect_ready = subparsers.add_parser("inspect-ready")
    inspect_ready.add_argument("--file", type=Path, required=True)
    inspect_ready.add_argument("--mode", choices=sorted(MODES), required=True)

    write = subparsers.add_parser("write")
    write.add_argument("--output", type=Path, required=True)
    write.add_argument("--mode", choices=sorted(MODES), required=True)
    write.add_argument("--spring-arm-cm", type=float, required=True)
    write.add_argument("--ue-exec-cmds", required=True)
    write.add_argument("--project-root", type=Path, required=True)
    write.add_argument("--contract", type=Path)
    write.add_argument("--bundle", type=Path)
    write.add_argument("--ue-log", type=Path)
    write.add_argument("--ue-log-start-offset", type=int)
    write.add_argument("--ready-file", type=Path)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "confirm":
            payload = confirm_ready(
                output=args.output,
                mode=args.mode,
                framing_label=args.framing_label,
            )
        elif args.command == "inspect-ready":
            payload = load_ready(args.file, expected_mode=args.mode)
        else:
            payload = write_receipt(
                output=args.output,
                mode=args.mode,
                spring_arm_cm=args.spring_arm_cm,
                ue_exec_cmds=args.ue_exec_cmds,
                project_root=args.project_root,
                contract=args.contract,
                bundle=args.bundle,
                ue_log=args.ue_log,
                log_offset=args.ue_log_start_offset,
                ready_file=args.ready_file,
            )
    except (CameraReceiptError, overlay.OverlayError, OSError) as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 2
    print(json.dumps(payload, sort_keys=True, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
