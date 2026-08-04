#!/usr/bin/env python3
"""Fail-closed verification for the Matrix RobotTrainingGround install."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import sys


SCHEMA = "matrix-realscan-ue-package-receipt/v1"
MAP_NAME = "/Game/Maps/RobotTrainingGround"
SOURCE_USDZ_SHA256 = "2b67231becf613036d4acdec796cffcad9ae3e2456dd311a96f8a00932df85cd"
SOURCE_PLY_SHA256 = "911399630534fa9df8b143c2437fd89c68176ec5fe53bb1317e7d2fec03b472c"
PHYSICS_REPORT = Path(
    "config/realscan/generated/robot_training_ground_mujoco_proxy.json"
)
PHYSICS_XML = "scene_terrain_robot_training_ground.xml"
PHYSICS_HEIGHTFIELD = "robot_training_ground_lower_floor.png"
RECEIPT = Path(
    "src/UeSim/Linux/zsibot_mujoco_ue/Saved/Paks/RobotTrainingGroundActive/receipt.json"
)
PAKS = Path("src/UeSim/Linux/zsibot_mujoco_ue/Content/Paks")
_SHA256_RE = re.compile(r"[0-9a-f]{64}\Z")
_COMMIT_RE = re.compile(r"(?:[0-9a-f]{40}|[0-9a-f]{64})\Z")
_PACKAGE_RE = re.compile(r"pakchunk[0-9A-Za-z_.-]+-Linux\.(?:pak|utoc|ucas)\Z")


class VerificationError(RuntimeError):
    """Installed scene bytes do not match their locked receipt."""


def regular_file(path: Path, label: str) -> None:
    if not path.is_file() or path.is_symlink():
        raise VerificationError(f"{label} must be a regular non-symlink file: {path}")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def strict_json(path: Path, label: str) -> dict[str, object]:
    regular_file(path, label)
    try:
        value = json.loads(
            path.read_text(),
            object_pairs_hook=lambda pairs: _strict_object(pairs, label),
            parse_constant=lambda value: (_ for _ in ()).throw(
                ValueError(f"non-finite JSON value {value}")
            ),
        )
    except (OSError, UnicodeError, ValueError) as exc:
        raise VerificationError(f"cannot read {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise VerificationError(f"{label} must be an object")
    return value


def _strict_object(pairs: list[tuple[str, object]], label: str) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate key in {label}: {key}")
        result[key] = value
    return result


def verify_physics(project_root: Path) -> dict[str, str]:
    report = strict_json(project_root / PHYSICS_REPORT, "physics report")
    if (
        report.get("schema") != "matrix-realscan-mujoco-proxy/v1"
        or report.get("physics_backend") != "mujoco"
        or report.get("scene") != "robot-training-ground_20260715-142024"
    ):
        raise VerificationError("physics report identity is invalid")
    expected: dict[str, str] = {}
    for field, filename in (("xml", PHYSICS_XML), ("heightfield", PHYSICS_HEIGHTFIELD)):
        item = report.get(field)
        if (
            not isinstance(item, dict)
            or item.get("filename") != filename
            or not isinstance(item.get("sha256"), str)
            or _SHA256_RE.fullmatch(item["sha256"]) is None
        ):
            raise VerificationError(f"physics report {field} record is invalid")
        source = project_root / "config/realscan/generated" / filename
        regular_file(source, f"generated {field}")
        if sha256_file(source) != item["sha256"]:
            raise VerificationError(f"generated {field} hash mismatch")
        expected[filename] = item["sha256"]

    destinations = [
        project_root / "src/robot_mujoco/zsibot_robots/xgb",
        project_root / "src/UeSim/Linux/zsibot_mujoco_ue/Content/model/xgb",
    ]
    for destination in destinations:
        for filename, digest in expected.items():
            installed = destination / filename
            regular_file(installed, "installed physics asset")
            if sha256_file(installed) != digest:
                raise VerificationError(f"installed physics hash mismatch: {installed}")
    return expected


def verify_visual_receipt(project_root: Path) -> dict[str, object]:
    receipt = strict_json(project_root / RECEIPT, "visual receipt")
    expected_keys = {
        "schema",
        "map_name",
        "source_usdz_sha256",
        "source_ply_sha256",
        "ue_project",
        "files",
    }
    if set(receipt) != expected_keys:
        raise VerificationError("visual receipt keys are invalid")
    if (
        receipt.get("schema") != SCHEMA
        or receipt.get("map_name") != MAP_NAME
        or receipt.get("source_usdz_sha256") != SOURCE_USDZ_SHA256
        or receipt.get("source_ply_sha256") != SOURCE_PLY_SHA256
        or not isinstance(receipt.get("ue_project"), dict)
        or set(receipt["ue_project"]) != {"repository", "commit"}
    ):
        raise VerificationError("visual receipt identity is invalid")
    ue_project = receipt["ue_project"]
    if (
        not isinstance(ue_project.get("repository"), str)
        or not ue_project["repository"]
        or not isinstance(ue_project.get("commit"), str)
        or _COMMIT_RE.fullmatch(ue_project["commit"]) is None
    ):
        raise VerificationError("visual receipt UE provenance is invalid")
    files = receipt.get("files")
    if not isinstance(files, list) or len(files) != 3:
        raise VerificationError("visual receipt must name one pak/utoc/ucas trio")
    suffixes: set[str] = set()
    stems: set[str] = set()
    seen: set[str] = set()
    for item in files:
        if not isinstance(item, dict) or set(item) != {"name", "size_bytes", "sha256"}:
            raise VerificationError("visual package record is invalid")
        name = item.get("name")
        size = item.get("size_bytes")
        digest = item.get("sha256")
        if (
            not isinstance(name, str)
            or _PACKAGE_RE.fullmatch(name) is None
            or name in seen
            or type(size) is not int
            or size <= 0
            or not isinstance(digest, str)
            or _SHA256_RE.fullmatch(digest) is None
        ):
            raise VerificationError("visual package identity is invalid")
        seen.add(name)
        suffixes.add(Path(name).suffix)
        stems.add(name.rsplit(".", 1)[0])
        package = project_root / PAKS / name
        regular_file(package, "visual package")
        if package.stat().st_size != size or sha256_file(package) != digest:
            raise VerificationError(f"visual package hash mismatch: {package}")
    if suffixes != {".pak", ".utoc", ".ucas"} or len(stems) != 1:
        raise VerificationError("visual package files are not one matching trio")
    return receipt


def verify(project_root: Path) -> dict[str, object]:
    if not project_root.is_dir() or project_root.is_symlink():
        raise VerificationError("project root must be a real directory")
    physics = verify_physics(project_root)
    receipt = verify_visual_receipt(project_root)
    return {
        "schema": "matrix-realscan-install-verification/v1",
        "ok": True,
        "project_root": str(project_root),
        "map_name": MAP_NAME,
        "physics_backend": "mujoco",
        "visual_backend": "matrix_ue_threedgaussians",
        "physics_files": sorted(physics),
        "visual_files": [item["name"] for item in receipt["files"]],
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--project-root", type=Path, required=True)
    args = parser.parse_args(argv)
    print(json.dumps(verify(args.project_root.resolve()), sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except VerificationError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc
