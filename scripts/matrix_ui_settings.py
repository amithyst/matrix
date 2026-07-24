#!/usr/bin/env python3
"""Validate and atomically persist Matrix operator-interface settings."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import re
import tempfile


FONT_SCALE_STEPS = (0.80, 0.90, 1.00, 1.10, 1.20, 1.30, 1.40, 1.50)
DEFAULT_FONT_SCALE = 1.00
MIN_FONT_SCALE = FONT_SCALE_STEPS[0]
MAX_FONT_SCALE = FONT_SCALE_STEPS[-1]
MIN_FONT_SIZE = 1
MAX_FONT_SIZE = 22
DEFAULT_FONT_SIZE = 13
_PROFILE_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,63}\Z")


def canonical_host_profile(value: object) -> str:
    if not isinstance(value, str) or _PROFILE_RE.fullmatch(value) is None:
        raise ValueError(
            "host profile must use 1-64 ASCII letters, digits, dot, underscore, or dash"
        )
    return value


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


def font_size_for_scale(value: object) -> int:
    canonical = canonical_font_scale(value)
    return max(8, int(round(13 * canonical)))


def canonical_font_size(value: object) -> int:
    if isinstance(value, bool) or type(value) is not int:
        raise ValueError("font size must be an integer in [1, 22]")
    if not MIN_FONT_SIZE <= value <= MAX_FONT_SIZE:
        raise ValueError("font size must be an integer in [1, 22]")
    return value


def step_font_size(value: object, direction: int) -> int:
    if (
        isinstance(direction, bool)
        or not isinstance(direction, int)
        or direction not in {-1, 1}
    ):
        raise ValueError("font size direction must be -1 or 1")
    return max(
        MIN_FONT_SIZE,
        min(MAX_FONT_SIZE, canonical_font_size(value) + direction),
    )


@dataclass(frozen=True)
class UiSettings:
    font_scale: float = DEFAULT_FONT_SCALE
    font_size: int | None = None

    def __post_init__(self) -> None:
        scale = canonical_font_scale(self.font_scale)
        object.__setattr__(self, "font_scale", scale)
        object.__setattr__(
            self,
            "font_size",
            font_size_for_scale(scale)
            if self.font_size is None
            else canonical_font_size(self.font_size),
        )
    def persisted_mapping(self) -> dict[str, object]:
        return {
            "version": 2,
            "font_scale": self.font_scale,
            "font_size": self.font_size,
        }


@dataclass(frozen=True)
class SettingsLoad:
    settings: UiSettings
    status: str
    error: str | None = None


def default_settings_file(
    profile: object | None = None,
    *,
    config_home: Path | str | None = None,
) -> Path:
    selected = (
        os.environ.get("MATRIX_HOST_PROFILE")
        or os.environ.get("PROFILE")
        or "local"
        if profile is None
        else profile
    )
    host_profile = canonical_host_profile(selected)
    if config_home is None:
        configured = os.environ.get("XDG_CONFIG_HOME")
        root = Path(configured) if configured else Path.home() / ".config"
    else:
        root = Path(config_home)
    return root.expanduser() / "matrix" / "hosts" / host_profile / "ui-settings.json"


def legacy_settings_file(
    *,
    config_home: Path | str | None = None,
) -> Path:
    if config_home is None:
        configured = os.environ.get("XDG_CONFIG_HOME")
        root = Path(configured) if configured else Path.home() / ".config"
    else:
        root = Path(config_home)
    return root.expanduser() / "matrix" / "ui-settings.json"


def load_settings(path: Path) -> SettingsLoad:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return SettingsLoad(UiSettings(), "missing")
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        return SettingsLoad(UiSettings(), "invalid", f"cannot read settings: {exc}")
    try:
        if not isinstance(value, dict):
            raise ValueError("expected settings object")
        version = value.get("version")
        if version == 1:
            if set(value) != {"version", "font_scale"}:
                raise ValueError("expected exactly version/font_scale")
            settings = UiSettings(
                font_scale=value.get("font_scale"),
                font_size=font_size_for_scale(value.get("font_scale")),
            )
        elif version == 2:
            if set(value) != {"version", "font_scale", "font_size"}:
                raise ValueError("expected exactly version/font_scale/font_size")
            settings = UiSettings(
                font_scale=value.get("font_scale"),
                font_size=value.get("font_size"),
            )
        else:
            raise ValueError("unsupported settings version")
    except (TypeError, ValueError) as exc:
        return SettingsLoad(UiSettings(), "invalid", f"invalid settings: {exc}")
    return SettingsLoad(settings, "loaded")


def load_settings_with_legacy_fallback(path: Path) -> SettingsLoad:
    loaded = load_settings(path)
    if loaded.status != "missing":
        return loaded
    parts = path.parts
    matrix_index = len(parts) - 4
    is_host_file = bool(
        matrix_index >= 0
        and parts[matrix_index] == "matrix"
        and parts[matrix_index + 1] == "hosts"
        and parts[matrix_index + 3] == "ui-settings.json"
    )
    if not is_host_file:
        return loaded
    legacy = Path(*parts[: matrix_index + 1]) / "ui-settings.json"
    legacy_loaded = load_settings(legacy)
    if legacy_loaded.status == "loaded":
        return SettingsLoad(legacy_loaded.settings, "loaded_legacy")
    return loaded


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
