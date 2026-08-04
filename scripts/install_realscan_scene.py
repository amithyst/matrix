#!/usr/bin/env python3
"""Install a verified RobotTrainingGround physics and cooked visual bundle."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile

from verify_realscan_scene_install import (
    MAP_NAME,
    PHYSICS_HEIGHTFIELD,
    PHYSICS_XML,
    RECEIPT,
    SCHEMA,
    SOURCE_PLY_SHA256,
    SOURCE_USDZ_SHA256,
    VerificationError,
    _COMMIT_RE,
    _PACKAGE_RE,
    _SHA256_RE,
    sha256_file,
    strict_json,
    verify,
)


def copy_exact(source: Path, destination: Path, digest: str) -> None:
    if not source.is_file() or source.is_symlink() or sha256_file(source) != digest:
        raise VerificationError(f"source asset is invalid: {source}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists() or destination.is_symlink():
        if (
            destination.is_file()
            and not destination.is_symlink()
            and destination.stat().st_size == source.stat().st_size
            and sha256_file(destination) == digest
        ):
            return
        raise VerificationError(
            f"refusing to replace a different installed file: {destination}"
        )
    with tempfile.NamedTemporaryFile(dir=destination.parent, delete=False) as temporary:
        temporary_path = Path(temporary.name)
        with source.open("rb") as stream:
            shutil.copyfileobj(stream, temporary, length=8 * 1024 * 1024)
        temporary.flush()
        os.fsync(temporary.fileno())
    try:
        if sha256_file(temporary_path) != digest:
            raise VerificationError(f"staged copy hash mismatch: {destination}")
        os.chmod(temporary_path, 0o644)
        os.replace(temporary_path, destination)
    finally:
        if temporary_path.exists():
            temporary_path.unlink()


def validate_bundle_receipt(bundle_dir: Path) -> dict[str, object]:
    receipt_path = bundle_dir / "receipt.json"
    receipt = strict_json(receipt_path, "visual bundle receipt")
    if (
        set(receipt)
        != {
            "schema",
            "map_name",
            "source_usdz_sha256",
            "source_ply_sha256",
            "ue_project",
            "files",
        }
        or receipt.get("schema") != SCHEMA
        or receipt.get("map_name") != MAP_NAME
        or receipt.get("source_usdz_sha256") != SOURCE_USDZ_SHA256
        or receipt.get("source_ply_sha256") != SOURCE_PLY_SHA256
        or not isinstance(receipt.get("ue_project"), dict)
        or set(receipt["ue_project"]) != {"repository", "commit"}
    ):
        raise VerificationError("visual bundle receipt identity is invalid")
    ue_project = receipt["ue_project"]
    if (
        not isinstance(ue_project.get("repository"), str)
        or not ue_project["repository"]
        or not isinstance(ue_project.get("commit"), str)
        or _COMMIT_RE.fullmatch(ue_project["commit"]) is None
    ):
        raise VerificationError("visual bundle UE provenance is invalid")
    files = receipt.get("files")
    if not isinstance(files, list) or len(files) != 3:
        raise VerificationError("visual bundle must contain a pak/utoc/ucas trio")
    suffixes: set[str] = set()
    stems: set[str] = set()
    seen: set[str] = set()
    for item in files:
        if not isinstance(item, dict) or set(item) != {"name", "size_bytes", "sha256"}:
            raise VerificationError("visual bundle file record is invalid")
        name = item.get("name")
        digest = item.get("sha256")
        size = item.get("size_bytes")
        if (
            not isinstance(name, str)
            or _PACKAGE_RE.fullmatch(name) is None
            or name in seen
            or not isinstance(digest, str)
            or _SHA256_RE.fullmatch(digest) is None
            or type(size) is not int
            or size <= 0
        ):
            raise VerificationError("visual bundle package identity is invalid")
        seen.add(name)
        suffixes.add(Path(name).suffix)
        stems.add(name.rsplit(".", 1)[0])
        source = bundle_dir / name
        if (
            not source.is_file()
            or source.is_symlink()
            or source.stat().st_size != size
            or sha256_file(source) != digest
        ):
            raise VerificationError(f"visual bundle file mismatch: {source}")
    if suffixes != {".pak", ".utoc", ".ucas"} or len(stems) != 1:
        raise VerificationError("visual bundle files are not one matching trio")
    return receipt


def install(project_root: Path, bundle_dir: Path) -> dict[str, object]:
    project_root = project_root.resolve()
    bundle_dir = bundle_dir.resolve()
    receipt = validate_bundle_receipt(bundle_dir)
    physics_report = strict_json(
        project_root
        / "config/realscan/generated/robot_training_ground_mujoco_proxy.json",
        "physics report",
    )
    physics_records = {
        PHYSICS_XML: physics_report.get("xml"),
        PHYSICS_HEIGHTFIELD: physics_report.get("heightfield"),
    }
    physics_destinations = [
        project_root / "src/robot_mujoco/zsibot_robots/xgb",
        project_root / "src/UeSim/Linux/zsibot_mujoco_ue/Content/model/xgb",
    ]
    for filename, record in physics_records.items():
        if not isinstance(record, dict) or not isinstance(record.get("sha256"), str):
            raise VerificationError(f"physics report is missing {filename}")
        source = project_root / "config/realscan/generated" / filename
        for destination in physics_destinations:
            copy_exact(source, destination / filename, record["sha256"])

    paks = project_root / "src/UeSim/Linux/zsibot_mujoco_ue/Content/Paks"
    for item in receipt["files"]:
        copy_exact(bundle_dir / item["name"], paks / item["name"], item["sha256"])
    receipt_destination = project_root / RECEIPT
    copy_exact(
        bundle_dir / "receipt.json",
        receipt_destination,
        sha256_file(bundle_dir / "receipt.json"),
    )
    return verify(project_root)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--project-root", type=Path, required=True)
    parser.add_argument("--visual-bundle-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    print(
        json.dumps(install(args.project_root, args.visual_bundle_dir), sort_keys=True)
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except VerificationError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc
