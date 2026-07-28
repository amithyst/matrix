#!/usr/bin/env python3
"""Send focus-independent key pulses to the frozen BFM Unix adapter."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import socket


ALLOWED_KEYS = frozenset(
    {
        "W",
        "S",
        "A",
        "D",
        "Q",
        "E",
        "R",
        "C",
        "SPACE",
        "BACKSPACE",
        "ESCAPE",
        "LEFT_SHIFT",
        "RIGHT_SHIFT",
        "LEFT",
        "RIGHT",
        "UP",
        "DOWN",
    }
)


def send_key_event(
    sender: socket.socket,
    path: Path,
    key: str,
    pressed: bool,
) -> None:
    normalized = str(key).upper()
    if normalized not in ALLOWED_KEYS:
        raise ValueError(f"unsupported BFM key: {key}")
    sender.sendto(
        json.dumps(
            {"key": normalized, "pressed": bool(pressed)},
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8"),
        str(path),
    )


def send_key_pulses(path: Path, keys: list[str]) -> None:
    path = Path(path)
    if not path.is_socket():
        raise FileNotFoundError(f"BFM keyboard socket is unavailable: {path}")
    sender = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        for key in keys:
            send_key_event(sender, path, key, True)
            send_key_event(sender, path, key, False)
    finally:
        sender.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", type=Path, required=True)
    parser.add_argument("--key", action="append", required=True)
    args = parser.parse_args(argv)
    try:
        send_key_pulses(args.socket, args.key)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
