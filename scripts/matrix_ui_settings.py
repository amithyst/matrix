#!/usr/bin/env python3
"""Validate and atomically persist Matrix operator-interface settings."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import tempfile


FONT_SCALE_STEPS = (0.80, 0.90, 1.00, 1.10, 1.20, 1.30, 1.40, 1.50)
DEFAULT_FONT_SCALE = 1.00
MIN_FONT_SCALE = FONT_SCALE_STEPS[0]
MAX_FONT_SCALE = FONT_SCALE_STEPS[-1]


def canonical_font_scale(value: object) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
    ):
        raise ValueError("font scale must be finite and use a supported preset")
    number = float(value)
    for preset in FONT_SCALE_STEPS:
        if math.isclose(number, preset, rel_tol=0.0, abs_tol=1e-9):
            return preset
    raise ValueError(
        "font scale must use one of: "
        + ", ".join(f"{preset:.2f}" for preset in FONT_SCALE_STEPS)
    )


def step_font_scale(value: object, direction: int) -> float:
    if (
        isinstance(direction, bool)
        or not isinstance(direction, int)
        or direction not in {-1, 1}
    ):
        raise ValueError("font scale direction must be -1 or 1")
    canonical = canonical_font_scale(value)
    index = FONT_SCALE_STEPS.index(canonical)
    return FONT_SCALE_STEPS[
        max(0, min(len(FONT_SCALE_STEPS) - 1, index + direction))
    ]


@dataclass(frozen=True)
class UiSettings:
    font_scale: float = DEFAULT_FONT_SCALE

    def __post_init__(self) -> None:
        object.__setattr__(self, "font_scale", canonical_font_scale(self.font_scale))

    def persisted_mapping(self) -> dict[str, object]:
        return {"version": 1, "font_scale": self.font_scale}


@dataclass(frozen=True)
class SettingsLoad:
    settings: UiSettings
    status: str
    error: str | None = None


def default_settings_file() -> Path:
    root = Path(os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config"))
    return root / "matrix" / "ui-settings.json"


def load_settings(path: Path) -> SettingsLoad:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return SettingsLoad(UiSettings(), "missing")
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        return SettingsLoad(UiSettings(), "invalid", f"cannot read settings: {exc}")
    try:
        if not isinstance(value, dict) or set(value) != {"version", "font_scale"}:
            raise ValueError("expected exactly version/font_scale")
        if value.get("version") != 1:
            raise ValueError("unsupported settings version")
        settings = UiSettings(font_scale=value.get("font_scale"))
    except (TypeError, ValueError) as exc:
        return SettingsLoad(UiSettings(), "invalid", f"invalid settings: {exc}")
    return SettingsLoad(settings, "loaded")


def atomic_save_settings(path: Path, settings: UiSettings) -> None:
    if not path.is_absolute():
        raise ValueError("UI settings path must be absolute")
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
