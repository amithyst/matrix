#!/usr/bin/env python3
"""Create and validate bounded Git provenance for the Matrix ESC panel."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import subprocess
from typing import Any


SCHEMA_ID = "matrix-build-info/v1"
PROFILES = ("local", "heyuan", "trna", "zza")
CONTROL_SOURCES = ("planner", "game", "pico", "external")
MAX_JSON_BYTES = 128 * 1024
MAX_FILES = 16
MAX_DIRTY_PATHS = 16
_COMMIT_RE = re.compile(r"^(?:[0-9a-f]{40}|[0-9a-f]{64})$")
_SHORT_COMMIT_RE = re.compile(r"^[0-9a-f]{7,16}$")
_PROFILE_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")
_STATUS_RE = re.compile(r"^[A-Z?][A-Z0-9?]{0,3}$")

SCENE_NAMES = {
    0: "CustomWorld",
    1: "Warehouse",
    2: "Town10World",
    3: "YardWorld",
    4: "CrowdWorld",
    5: "VeniceWorld",
    6: "HouseWorld",
    7: "RunningWorld",
    8: "Town10Zombie",
    9: "IROSFlatWorld",
    10: "IROSSlopedWorld",
    11: "IROSFlatWorld2025",
    12: "IROSSloppedWorld2025",
    13: "OfficeWorld",
    14: "3DGSWorld",
    15: "MoonWorld",
    20: "CalibrationRoom",
    21: "ApartmentWorld",
    22: "Laboratory",
}


class BuildInfoError(ValueError):
    """Raised when provenance input violates the bounded display contract."""


def _strict_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise BuildInfoError(f"duplicate key {key!r}")
        result[key] = value
    return result


def _reject_constant(value: str) -> None:
    raise BuildInfoError(f"non-finite JSON number {value}")


def _bounded_text(value: object, *, field: str, maximum: int) -> str:
    if not isinstance(value, str):
        raise BuildInfoError(f"{field} must be a string")
    cleaned = "".join(
        character if character >= " " and character != "\x7f" else " "
        for character in value
    )
    cleaned = " ".join(cleaned.split())
    if len(cleaned) > maximum:
        cleaned = cleaned[: max(0, maximum - 1)].rstrip() + "..."
    return cleaned


def _optional_text(
    value: object,
    *,
    field: str,
    maximum: int,
) -> str | None:
    if value is None:
        return None
    return _bounded_text(value, field=field, maximum=maximum)


def _non_negative_int(value: object, *, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise BuildInfoError(f"{field} must be a non-negative integer")
    return value


def _optional_count(value: object, *, field: str) -> int | None:
    if value is None:
        return None
    return _non_negative_int(value, field=field)


def unavailable_build_info(
    *,
    profile: str,
    scene_id: int,
    control_source: str,
    error: str,
    launch_available: bool = True,
) -> dict[str, object]:
    return {
        "schema": SCHEMA_ID,
        "available": False,
        "launch_available": launch_available,
        "profile": profile,
        "scene_id": scene_id,
        "scene_name": SCENE_NAMES.get(scene_id, f"Scene {scene_id}"),
        "control_source": control_source,
        "branch": None,
        "commit": None,
        "short_commit": None,
        "subject": None,
        "body": None,
        "author": None,
        "authored_at": None,
        "changed_files": 0,
        "additions": 0,
        "deletions": 0,
        "files": [],
        "files_truncated": False,
        "dirty": False,
        "dirty_files": 0,
        "dirty_paths": [],
        "dirty_paths_truncated": False,
        "error": _bounded_text(error, field="error", maximum=512),
    }


def validate_build_info(value: object) -> dict[str, object]:
    if not isinstance(value, dict):
        raise BuildInfoError("build info must be an object")
    expected = {
        "schema",
        "available",
        "launch_available",
        "profile",
        "scene_id",
        "scene_name",
        "control_source",
        "branch",
        "commit",
        "short_commit",
        "subject",
        "body",
        "author",
        "authored_at",
        "changed_files",
        "additions",
        "deletions",
        "files",
        "files_truncated",
        "dirty",
        "dirty_files",
        "dirty_paths",
        "dirty_paths_truncated",
        "error",
    }
    if set(value) != expected:
        missing = sorted(expected - set(value))
        extra = sorted(set(value) - expected)
        raise BuildInfoError(
            f"build info keys are invalid: missing={missing} extra={extra}"
        )
    if value.get("schema") != SCHEMA_ID:
        raise BuildInfoError("build info schema is invalid")
    if type(value.get("available")) is not bool:
        raise BuildInfoError("available must be boolean")
    if type(value.get("launch_available")) is not bool:
        raise BuildInfoError("launch_available must be boolean")
    profile = _bounded_text(value.get("profile"), field="profile", maximum=64)
    if _PROFILE_RE.fullmatch(profile) is None:
        raise BuildInfoError("profile is invalid")
    scene_id = value.get("scene_id")
    if isinstance(scene_id, bool) or not isinstance(scene_id, int):
        raise BuildInfoError("scene_id must be an integer")
    if not 0 <= scene_id <= 99:
        raise BuildInfoError("scene_id must be in [0, 99]")
    scene_name = _bounded_text(
        value.get("scene_name"), field="scene_name", maximum=96
    )
    control_source = _bounded_text(
        value.get("control_source"), field="control_source", maximum=16
    )
    if control_source not in CONTROL_SOURCES:
        raise BuildInfoError("control_source is invalid")
    available = value["available"]
    launch_available = value["launch_available"]
    branch = _optional_text(value.get("branch"), field="branch", maximum=256)
    commit = _optional_text(value.get("commit"), field="commit", maximum=64)
    short_commit = _optional_text(
        value.get("short_commit"), field="short_commit", maximum=16
    )
    subject = _optional_text(value.get("subject"), field="subject", maximum=512)
    body = _optional_text(value.get("body"), field="body", maximum=1024)
    author = _optional_text(value.get("author"), field="author", maximum=256)
    authored_at = _optional_text(
        value.get("authored_at"), field="authored_at", maximum=64
    )
    error = _optional_text(value.get("error"), field="error", maximum=512)
    if available:
        if not launch_available:
            raise BuildInfoError("Git provenance requires launch metadata")
        if (
            not branch
            or commit is None
            or _COMMIT_RE.fullmatch(commit) is None
            or short_commit is None
            or _SHORT_COMMIT_RE.fullmatch(short_commit) is None
            or not subject
            or not author
            or not authored_at
            or error is not None
        ):
            raise BuildInfoError("available build info is incomplete")
    elif error is None:
        raise BuildInfoError("unavailable build info requires an error")
    changed_files = _non_negative_int(
        value.get("changed_files"), field="changed_files"
    )
    additions = _non_negative_int(value.get("additions"), field="additions")
    deletions = _non_negative_int(value.get("deletions"), field="deletions")
    files_value = value.get("files")
    if not isinstance(files_value, list) or len(files_value) > MAX_FILES:
        raise BuildInfoError("files must be a bounded list")
    files: list[dict[str, object]] = []
    for index, item in enumerate(files_value):
        if not isinstance(item, dict) or set(item) != {
            "status",
            "path",
            "additions",
            "deletions",
        }:
            raise BuildInfoError(f"files[{index}] has invalid keys")
        status = _bounded_text(
            item.get("status"), field=f"files[{index}].status", maximum=4
        )
        if _STATUS_RE.fullmatch(status) is None:
            raise BuildInfoError(f"files[{index}].status is invalid")
        path = _bounded_text(
            item.get("path"), field=f"files[{index}].path", maximum=512
        )
        if not path:
            raise BuildInfoError(f"files[{index}].path is empty")
        files.append(
            {
                "status": status,
                "path": path,
                "additions": _optional_count(
                    item.get("additions"),
                    field=f"files[{index}].additions",
                ),
                "deletions": _optional_count(
                    item.get("deletions"),
                    field=f"files[{index}].deletions",
                ),
            }
        )
    files_truncated = value.get("files_truncated")
    dirty = value.get("dirty")
    dirty_paths_truncated = value.get("dirty_paths_truncated")
    if type(files_truncated) is not bool or type(dirty) is not bool:
        raise BuildInfoError("file and dirty flags must be boolean")
    if type(dirty_paths_truncated) is not bool:
        raise BuildInfoError("dirty_paths_truncated must be boolean")
    dirty_files = _non_negative_int(value.get("dirty_files"), field="dirty_files")
    dirty_paths_value = value.get("dirty_paths")
    if (
        not isinstance(dirty_paths_value, list)
        or len(dirty_paths_value) > MAX_DIRTY_PATHS
    ):
        raise BuildInfoError("dirty_paths must be a bounded list")
    dirty_paths = [
        _bounded_text(item, field="dirty_paths item", maximum=512)
        for item in dirty_paths_value
    ]
    if any(not item for item in dirty_paths):
        raise BuildInfoError("dirty_paths contains an empty path")
    if dirty != (dirty_files > 0):
        raise BuildInfoError("dirty flag does not match dirty_files")
    if changed_files < len(files):
        raise BuildInfoError("changed_files is smaller than the file sample")
    return {
        "schema": SCHEMA_ID,
        "available": available,
        "launch_available": launch_available,
        "profile": profile,
        "scene_id": scene_id,
        "scene_name": scene_name,
        "control_source": control_source,
        "branch": branch,
        "commit": commit,
        "short_commit": short_commit,
        "subject": subject,
        "body": body,
        "author": author,
        "authored_at": authored_at,
        "changed_files": changed_files,
        "additions": additions,
        "deletions": deletions,
        "files": files,
        "files_truncated": files_truncated,
        "dirty": dirty,
        "dirty_files": dirty_files,
        "dirty_paths": dirty_paths,
        "dirty_paths_truncated": dirty_paths_truncated,
        "error": error,
    }


def parse_build_info_json(text: str) -> dict[str, object]:
    if not isinstance(text, str) or not text:
        raise BuildInfoError("build info JSON is empty")
    if len(text.encode("utf-8")) > MAX_JSON_BYTES:
        raise BuildInfoError("build info JSON is oversized")
    try:
        value = json.loads(
            text,
            object_pairs_hook=_strict_object,
            parse_constant=_reject_constant,
        )
    except BuildInfoError:
        raise
    except (UnicodeError, json.JSONDecodeError, ValueError) as exc:
        raise BuildInfoError(f"invalid build info JSON: {exc}") from exc
    return validate_build_info(value)


def _git(repo_root: Path, *arguments: str) -> str:
    try:
        completed = subprocess.run(
            ["git", "-C", os.fspath(repo_root), *arguments],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=5.0,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise BuildInfoError(f"git {' '.join(arguments)} failed: {exc}") from exc
    return completed.stdout


def _name_status(repo_root: Path, base: str | None, commit: str) -> list[tuple[str, str]]:
    if base is None:
        output = _git(
            repo_root,
            "-c",
            "core.quotepath=false",
            "diff-tree",
            "--root",
            "--no-commit-id",
            "--name-status",
            "-r",
            "-M",
            commit,
        )
    else:
        output = _git(
            repo_root,
            "-c",
            "core.quotepath=false",
            "diff",
            "--name-status",
            "-M",
            base,
            commit,
        )
    result: list[tuple[str, str]] = []
    for line in output.splitlines():
        fields = line.split("\t")
        if len(fields) < 2:
            continue
        status = fields[0][:4]
        path = " -> ".join(fields[1:])
        result.append((status, path))
    return result


def _numstat(
    repo_root: Path,
    base: str | None,
    commit: str,
) -> tuple[dict[str, tuple[int | None, int | None]], int, int]:
    if base is None:
        output = _git(
            repo_root,
            "-c",
            "core.quotepath=false",
            "diff-tree",
            "--root",
            "--no-commit-id",
            "--numstat",
            "-r",
            "-M",
            commit,
        )
    else:
        output = _git(
            repo_root,
            "-c",
            "core.quotepath=false",
            "diff",
            "--numstat",
            "-M",
            base,
            commit,
        )
    by_path: dict[str, tuple[int | None, int | None]] = {}
    total_additions = 0
    total_deletions = 0
    for line in output.splitlines():
        fields = line.split("\t", 2)
        if len(fields) != 3:
            continue
        additions = int(fields[0]) if fields[0].isdigit() else None
        deletions = int(fields[1]) if fields[1].isdigit() else None
        path = fields[2]
        by_path[path] = (additions, deletions)
        total_additions += additions or 0
        total_deletions += deletions or 0
    return by_path, total_additions, total_deletions


def collect_build_info(
    repo_root: Path,
    *,
    profile: str,
    scene_id: int,
    control_source: str,
) -> dict[str, object]:
    if not isinstance(profile, str) or _PROFILE_RE.fullmatch(profile) is None:
        raise BuildInfoError(f"invalid profile: {profile}")
    if isinstance(scene_id, bool) or not isinstance(scene_id, int) or not 0 <= scene_id <= 99:
        raise BuildInfoError("scene_id must be in [0, 99]")
    if control_source not in CONTROL_SOURCES:
        raise BuildInfoError(f"unsupported control source: {control_source}")
    root = Path(repo_root).resolve(strict=False)
    if not root.is_dir():
        raise BuildInfoError(f"repository root is not a directory: {root}")
    fallback = unavailable_build_info(
        profile=profile,
        scene_id=scene_id,
        control_source=control_source,
        error="Git provenance is unavailable",
    )
    try:
        commit_fields = _git(
            root,
            "log",
            "-1",
            "--format=%H%x00%h%x00%s%x00%b%x00%an%x00%aI",
        ).rstrip("\n").split("\x00")
        if len(commit_fields) != 6:
            raise BuildInfoError("git log returned an invalid field count")
        commit, short_commit, subject, body, author, authored_at = commit_fields
        branch = _git(root, "branch", "--show-current").strip()
        if not branch:
            branch = f"detached@{short_commit}"
        parent_fields = _git(root, "rev-list", "--parents", "-n", "1", commit).split()
        base = parent_fields[1] if len(parent_fields) > 1 else None
        statuses = _name_status(root, base, commit)
        numstat, additions, deletions = _numstat(root, base, commit)
        files: list[dict[str, object]] = []
        for status, path in statuses[:MAX_FILES]:
            counts = numstat.get(path)
            if counts is None and " -> " in path:
                counts = numstat.get(path.split(" -> ")[-1])
            file_additions, file_deletions = counts or (None, None)
            files.append(
                {
                    "status": status,
                    "path": path,
                    "additions": file_additions,
                    "deletions": file_deletions,
                }
            )
        dirty_output = _git(
            root,
            "-c",
            "core.quotepath=false",
            "status",
            "--short",
            "--untracked-files=normal",
        )
        dirty_lines = [line for line in dirty_output.splitlines() if line]
        dirty_paths = [line[3:] if len(line) > 3 else line for line in dirty_lines]
        value = {
            **fallback,
            "available": True,
            "branch": branch,
            "commit": commit,
            "short_commit": short_commit,
            "subject": subject,
            "body": body,
            "author": author,
            "authored_at": authored_at,
            "changed_files": len(statuses),
            "additions": additions,
            "deletions": deletions,
            "files": files,
            "files_truncated": len(statuses) > len(files),
            "dirty": bool(dirty_lines),
            "dirty_files": len(dirty_lines),
            "dirty_paths": dirty_paths[:MAX_DIRTY_PATHS],
            "dirty_paths_truncated": len(dirty_paths) > MAX_DIRTY_PATHS,
            "error": None,
        }
        return validate_build_info(value)
    except BuildInfoError as exc:
        fallback["error"] = _bounded_text(str(exc), field="error", maximum=512)
        return validate_build_info(fallback)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, required=True)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--scene", type=int, required=True)
    parser.add_argument(
        "--control-source",
        choices=CONTROL_SOURCES,
        required=True,
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    try:
        value = collect_build_info(
            args.repo_root,
            profile=args.profile,
            scene_id=args.scene,
            control_source=args.control_source,
        )
    except BuildInfoError as exc:
        raise SystemExit(str(exc)) from exc
    print(
        json.dumps(
            value,
            allow_nan=False,
            ensure_ascii=True,
            separators=(",", ":"),
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
