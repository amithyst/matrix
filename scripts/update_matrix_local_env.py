#!/usr/bin/env python3
"""Atomically update one exported value in Matrix's ignored local env file."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import re
import shlex
import stat
import sys


_NAME = re.compile(r"[A-Za-z_][A-Za-z0-9_]*\Z")
LOCAL_ENV_ALLOWLIST = frozenset(
    {
        "MATRIX_PICO_PYTHON",
        "MATRIX_PICO_WHEEL",
        "MATRIX_RUNTIME_ROOT",
        "MATRIX_SONIC_ROOT",
    }
)


def parse_local_env(path: Path) -> dict[str, str]:
    """Parse a data-only local override file without shell evaluation."""

    if path.is_symlink() or not path.is_file():
        raise ValueError(f"local env must be a regular non-symlink file: {path}")
    values: dict[str, str] = {}
    assignment = re.compile(
        r"^\s*(?:export\s+)?([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.*?)\s*$"
    )
    for line_number, raw_line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        match = assignment.fullmatch(raw_line)
        if match is None:
            raise ValueError(f"invalid local env syntax on line {line_number}")
        name, encoded_value = match.groups()
        if name not in LOCAL_ENV_ALLOWLIST:
            raise ValueError(
                f"local env variable is not allowlisted on line {line_number}: {name}"
            )
        if name in values:
            raise ValueError(f"duplicate local env variable on line {line_number}: {name}")
        if encoded_value == "":
            value = ""
        else:
            try:
                words = shlex.split(encoded_value, comments=False, posix=True)
            except ValueError as exc:
                raise ValueError(
                    f"invalid local env value on line {line_number}: {exc}"
                ) from exc
            if len(words) != 1:
                raise ValueError(
                    f"local env value must be exactly one shell-quoted word on line "
                    f"{line_number}"
                )
            value = words[0]
        if any(character in value for character in ("\0", "\r", "\n")):
            raise ValueError(f"local env value contains a forbidden byte on line {line_number}")
        values[name] = value
    return values


def update_export(path: Path, name: str, value: str) -> None:
    """Replace ``name`` once while preserving every unrelated line verbatim."""

    if _NAME.fullmatch(name) is None:
        raise ValueError(f"invalid environment variable name: {name!r}")
    if name not in LOCAL_ENV_ALLOWLIST:
        raise ValueError(f"environment variable is not allowlisted: {name}")
    if any(character in value for character in ("\0", "\r", "\n")):
        raise ValueError("environment value contains a forbidden byte")
    if path.is_symlink():
        raise ValueError(f"refusing to replace symlinked local env: {path}")

    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        parse_local_env(path)
    existing = path.read_text(encoding="utf-8") if path.exists() else ""
    assignment = re.compile(rf"^\s*(?:export\s+)?{re.escape(name)}\s*=")
    replacement = f"export {name}={shlex.quote(value)}\n"
    output: list[str] = []
    replaced = False
    for line in existing.splitlines(keepends=True):
        if assignment.match(line):
            if not replaced:
                output.append(replacement)
                replaced = True
            continue
        output.append(line)

    if not replaced:
        if output and not output[-1].endswith(("\n", "\r")):
            output[-1] += "\n"
        output.append(replacement)

    mode = stat.S_IMODE(path.stat().st_mode) if path.exists() else 0o600
    temporary = path.with_name(f".{path.name}.tmp.{os.getpid()}")
    try:
        with temporary.open("x", encoding="utf-8", newline="") as stream:
            stream.write("".join(output))
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, mode)
        os.replace(temporary, path)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--emit0",
        action="store_true",
        help="validate a local env and emit alternating NUL-delimited names/values",
    )
    parser.add_argument("path", type=Path)
    parser.add_argument("name", nargs="?")
    parser.add_argument("value", nargs="?")
    args = parser.parse_args()
    if args.emit0:
        if args.name is not None or args.value is not None:
            parser.error("--emit0 accepts only the local env path")
        try:
            values = parse_local_env(args.path)
        except (OSError, UnicodeError, ValueError) as exc:
            parser.error(str(exc))
        payload = bytearray()
        for name, value in values.items():
            payload.extend(name.encode("ascii"))
            payload.append(0)
            payload.extend(value.encode("utf-8"))
            payload.append(0)
        sys.stdout.buffer.write(payload)
    else:
        if args.name is None or args.value is None:
            parser.error("update mode requires path, name, and value")
        update_export(args.path, args.name, args.value)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
