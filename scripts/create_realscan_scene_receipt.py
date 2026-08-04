#!/usr/bin/env python3
"""Create a strict provenance receipt for a cooked RobotTrainingGround trio."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

from verify_realscan_scene_install import (
    MAP_NAME,
    SCHEMA,
    SOURCE_PLY_SHA256,
    SOURCE_USDZ_SHA256,
    VerificationError,
    _COMMIT_RE,
    _PACKAGE_RE,
    regular_file,
    sha256_file,
)


def validate_repository(value: str) -> str:
    if value != value.strip() or not value or len(value) > 1024:
        raise VerificationError("UE repository must be a non-empty bounded string")
    if any(ord(character) < 32 or ord(character) == 127 for character in value):
        raise VerificationError("UE repository contains control characters")
    return value


def create_receipt(bundle_dir: Path, repository: str, commit: str) -> dict[str, object]:
    if not bundle_dir.is_dir() or bundle_dir.is_symlink():
        raise VerificationError(
            f"bundle directory must be a real non-symlink directory: {bundle_dir}"
        )
    repository = validate_repository(repository)
    if _COMMIT_RE.fullmatch(commit) is None:
        raise VerificationError("UE commit must be 40 or 64 lowercase hex characters")
    output = bundle_dir / "receipt.json"
    if output.exists() or output.is_symlink():
        raise VerificationError(f"refusing to overwrite receipt: {output}")

    packages = sorted(
        (
            path
            for path in bundle_dir.iterdir()
            if _PACKAGE_RE.fullmatch(path.name) is not None
        ),
        key=lambda path: path.suffix,
    )
    if len(packages) != 3:
        raise VerificationError("bundle must contain exactly one pak/utoc/ucas trio")
    suffixes = {path.suffix for path in packages}
    stems = {path.name.rsplit(".", 1)[0] for path in packages}
    if suffixes != {".pak", ".utoc", ".ucas"} or len(stems) != 1:
        raise VerificationError("cooked files are not one matching pak/utoc/ucas trio")

    files: list[dict[str, object]] = []
    for package in packages:
        regular_file(package, "cooked package")
        size = package.stat().st_size
        if size <= 0:
            raise VerificationError(f"cooked package is empty: {package}")
        files.append(
            {
                "name": package.name,
                "size_bytes": size,
                "sha256": sha256_file(package),
            }
        )
    receipt: dict[str, object] = {
        "schema": SCHEMA,
        "map_name": MAP_NAME,
        "source_usdz_sha256": SOURCE_USDZ_SHA256,
        "source_ply_sha256": SOURCE_PLY_SHA256,
        "ue_project": {"repository": repository, "commit": commit},
        "files": files,
    }
    with output.open("x", encoding="utf-8") as stream:
        json.dump(receipt, stream, indent=2, sort_keys=True)
        stream.write("\n")
    return receipt


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bundle-dir", type=Path, required=True)
    parser.add_argument("--ue-repository", required=True)
    parser.add_argument("--ue-commit", required=True)
    args = parser.parse_args(argv)
    receipt = create_receipt(
        args.bundle_dir.absolute(), args.ue_repository, args.ue_commit
    )
    print(json.dumps(receipt, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except VerificationError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc
