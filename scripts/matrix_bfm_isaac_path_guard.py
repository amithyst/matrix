#!/usr/bin/env python3
"""Canonical path guard for colleague-owned Matrix deployment trees."""

from __future__ import annotations

import argparse
import os
from pathlib import Path


DEFAULT_PROTECTED_ROOT = Path("/home/trna/matrix")
UNIX_SOCKET_PATH_MAX_BYTES = 107


def canonical(path: Path) -> Path:
    return Path(path).expanduser().resolve(strict=False)


def is_same_or_descendant(candidate: Path, root: Path) -> bool:
    candidate = canonical(candidate)
    root = canonical(root)
    return candidate == root or root in candidate.parents


def validate_path(candidate: Path, protected: Path, *, mode: str) -> Path:
    candidate = canonical(candidate)
    protected = canonical(protected)
    if mode == "subtree":
        forbidden = is_same_or_descendant(candidate, protected)
    elif mode == "overlap":
        forbidden = is_same_or_descendant(
            candidate, protected
        ) or is_same_or_descendant(protected, candidate)
    else:
        raise ValueError("path guard mode must be subtree or overlap")
    if forbidden:
        raise ValueError(
            f"path overlaps protected colleague tree {protected}: {candidate}"
        )
    return candidate


def validate_unix_socket_path(candidate: Path) -> Path:
    """Reject pathname-based AF_UNIX addresses that Linux cannot bind."""

    encoded_length = len(os.fsencode(candidate))
    if encoded_length > UNIX_SOCKET_PATH_MAX_BYTES:
        raise ValueError(
            "AF_UNIX path exceeds the Linux pathname limit: "
            f"bytes={encoded_length} max={UNIX_SOCKET_PATH_MAX_BYTES} "
            f"path={candidate}"
        )
    return candidate


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path)
    parser.add_argument("--protected", type=Path, default=DEFAULT_PROTECTED_ROOT)
    parser.add_argument("--mode", choices=("subtree", "overlap"), default="subtree")
    parser.add_argument(
        "--unix-socket",
        action="store_true",
        help="also require the resolved path to fit Linux sockaddr_un.sun_path",
    )
    args = parser.parse_args(argv)
    try:
        resolved = validate_path(args.path, args.protected, mode=args.mode)
        if args.unix_socket:
            validate_unix_socket_path(resolved)
    except ValueError as exc:
        parser.error(str(exc))
    print(resolved)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
