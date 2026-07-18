#!/usr/bin/env python3
"""Validate and atomically persist Matrix mouse-speed launch settings.

The cooked UE process consumes ``SDL_MOUSE_RELATIVE_SPEED_SCALE`` only at
process startup.  This module is deliberately pure standard library so the
top-level shell launcher and the supervised input provider use one parser and
one set of bounds for the same settings file.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import tempfile


PROFILE_LOCAL = "local"
PROFILE_REMOTE = "remote"
DEFAULT_REMOTE_SPEED_SCALE = 0.5
MIN_REMOTE_SPEED_SCALE = 0.2
MAX_REMOTE_SPEED_SCALE = 1.0
SPEED_SCALE_STEP = 0.1


@dataclass(frozen=True)
class MouseSettings:
    profile: str = PROFILE_LOCAL
    speed_scale: float = DEFAULT_REMOTE_SPEED_SCALE

    def __post_init__(self) -> None:
        if self.profile not in {PROFILE_LOCAL, PROFILE_REMOTE}:
            raise ValueError(f"unsupported mouse profile: {self.profile!r}")
        if (
            isinstance(self.speed_scale, bool)
            or not isinstance(self.speed_scale, (int, float))
            or not math.isfinite(float(self.speed_scale))
            or not MIN_REMOTE_SPEED_SCALE
            <= float(self.speed_scale)
            <= MAX_REMOTE_SPEED_SCALE
        ):
            raise ValueError("mouse speed scale must be finite and in [0.2, 1.0]")
        object.__setattr__(self, "speed_scale", float(self.speed_scale))

    @property
    def effective_scale(self) -> float:
        return 1.0 if self.profile == PROFILE_LOCAL else self.speed_scale

    def persisted_mapping(self) -> dict[str, object]:
        return {
            "version": 1,
            "profile": self.profile,
            "speed_scale": self.speed_scale,
        }


@dataclass(frozen=True)
class SettingsLoad:
    settings: MouseSettings
    status: str
    error: str | None = None


def default_settings_file() -> Path:
    root = Path(os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config"))
    return root / "matrix" / "mouse-control.json"


def load_settings(path: Path) -> SettingsLoad:
    """Load a versioned file; missing/invalid state safely becomes Local."""

    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return SettingsLoad(MouseSettings(), "missing")
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        return SettingsLoad(MouseSettings(), "invalid", f"cannot read settings: {exc}")
    try:
        if not isinstance(value, dict) or set(value) != {
            "version",
            "profile",
            "speed_scale",
        }:
            raise ValueError("expected exactly version/profile/speed_scale")
        if value.get("version") != 1:
            raise ValueError("unsupported settings version")
        settings = MouseSettings(
            profile=value.get("profile"),
            speed_scale=value.get("speed_scale"),
        )
    except (TypeError, ValueError) as exc:
        return SettingsLoad(MouseSettings(), "invalid", f"invalid settings: {exc}")
    return SettingsLoad(settings, "loaded")


def atomic_save_settings(path: Path, settings: MouseSettings) -> None:
    """Replace one user settings file atomically with private permissions."""

    if not path.is_absolute():
        raise ValueError("mouse settings path must be absolute")
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            delete=False,
        ) as stream:
            temporary = Path(stream.name)
            os.fchmod(stream.fileno(), 0o600)
            json.dump(settings.persisted_mapping(), stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        temporary = None
        directory_fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        if temporary is not None:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass


def _launch_fields(path: Path) -> int:
    if not path.is_absolute():
        raise SystemExit("mouse settings path must be absolute")
    loaded = load_settings(path)
    if loaded.error:
        print(
            f"[WARN] {loaded.error}; using Local 1.00x for this launch",
            file=os.sys.stderr,
        )
    settings = loaded.settings
    print(
        f"{settings.profile}\t{settings.effective_scale:.6f}\t{loaded.status}"
    )
    return 0


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    launch = subparsers.add_parser("launch-fields")
    launch.add_argument("--file", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if args.command == "launch-fields":
        return _launch_fields(args.file)
    raise AssertionError(f"unhandled command: {args.command}")


if __name__ == "__main__":
    raise SystemExit(main())
