#!/usr/bin/env python3
"""Resolve the locked BFM/Isaac renderer settings and bounded A/B overrides."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
from typing import Mapping


SCHEMA = "matrix_bfm_isaac_video_settings.v1"
MAX_SETTINGS_BYTES = 16 * 1024
EXPECTED_FIELDS = {
    "schema",
    "resolution",
    "window_mode",
    "fps_limit",
    "quality",
    "camera_smoothing",
    "screen_percentage",
}
RESOLUTIONS = {
    "1280x720": (1280, 720),
    "1600x900": (1600, 900),
    "1920x1080": (1920, 1080),
    "2560x1440": (2560, 1440),
}
WINDOW_MODES = {"windowed", "borderless", "fullscreen"}
FPS_LIMITS = {30, 60, 90, 120}
QUALITY_PRESETS = {"low", "medium", "high", "epic"}
CAMERA_SMOOTHING_PRESETS = {"off", "low", "medium", "high"}
OVERRIDE_ENVIRONMENT = {
    "resolution": "MATRIX_BFM_ISAAC_VIDEO_RESOLUTION",
    "window_mode": "MATRIX_BFM_ISAAC_VIDEO_WINDOW_MODE",
    "fps_limit": "MATRIX_BFM_ISAAC_UE_MAX_FPS",
    "quality": "MATRIX_BFM_ISAAC_VIDEO_QUALITY",
    "camera_smoothing": "MATRIX_BFM_ISAAC_VIDEO_CAMERA_SMOOTHING",
    "screen_percentage": "MATRIX_BFM_ISAAC_SCREEN_PERCENTAGE",
}


class VideoSettingsError(ValueError):
    """The tracked settings file or one explicit BFM override is invalid."""


def _strict_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
    value: dict[str, object] = {}
    for key, item in pairs:
        if key in value:
            raise VideoSettingsError(f"duplicate settings field: {key}")
        value[key] = item
    return value


def _integer(value: object, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise VideoSettingsError(f"{label} must be an integer")
    return value


def validate_settings(value: object) -> dict[str, object]:
    if not isinstance(value, dict) or set(value) != EXPECTED_FIELDS:
        raise VideoSettingsError(
            "settings must contain exactly schema/resolution/window_mode/"
            "fps_limit/quality/camera_smoothing/screen_percentage"
        )
    if value["schema"] != SCHEMA:
        raise VideoSettingsError(f"settings schema must be {SCHEMA}")
    if (
        not isinstance(value["resolution"], str)
        or value["resolution"] not in RESOLUTIONS
    ):
        raise VideoSettingsError("resolution is not a supported bounded preset")
    if (
        not isinstance(value["window_mode"], str)
        or value["window_mode"] not in WINDOW_MODES
    ):
        raise VideoSettingsError("window_mode is not a supported bounded preset")
    fps_limit = _integer(value["fps_limit"], "fps_limit")
    if fps_limit not in FPS_LIMITS:
        raise VideoSettingsError("fps_limit must be one of 30/60/90/120")
    if (
        not isinstance(value["quality"], str)
        or value["quality"] not in QUALITY_PRESETS
    ):
        raise VideoSettingsError("quality is not a supported bounded preset")
    if (
        not isinstance(value["camera_smoothing"], str)
        or value["camera_smoothing"] not in CAMERA_SMOOTHING_PRESETS
    ):
        raise VideoSettingsError(
            "camera_smoothing is not a supported bounded preset"
        )
    screen_percentage = _integer(
        value["screen_percentage"], "screen_percentage"
    )
    if not 25 <= screen_percentage <= 200:
        raise VideoSettingsError("screen_percentage must be in [25, 200]")
    return dict(value)


def load_settings(path: Path) -> dict[str, object]:
    if path.is_symlink() or not path.is_file():
        raise VideoSettingsError(f"settings path must be a regular file: {path}")
    if path.stat().st_size > MAX_SETTINGS_BYTES:
        raise VideoSettingsError("settings file exceeds the size limit")
    try:
        value = json.loads(
            path.read_text(encoding="utf-8"), object_pairs_hook=_strict_object
        )
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise VideoSettingsError(f"could not load settings: {exc}") from exc
    return validate_settings(value)


def resolve_settings(
    path: Path, environment: Mapping[str, str]
) -> dict[str, object]:
    value = load_settings(path)
    for field, variable in OVERRIDE_ENVIRONMENT.items():
        if variable not in environment:
            continue
        raw = environment[variable]
        if field in {"fps_limit", "screen_percentage"}:
            try:
                value[field] = int(raw, 10)
            except ValueError as exc:
                raise VideoSettingsError(f"{variable} must be an integer") from exc
        else:
            value[field] = raw
    return validate_settings(value)


def runtime_mapping(value: Mapping[str, object]) -> dict[str, object]:
    resolution = str(value["resolution"])
    width, height = RESOLUTIONS[resolution]
    return {
        **value,
        "resolution_width": width,
        "resolution_height": height,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--file", required=True, type=Path)
    parser.add_argument(
        "--format", choices=("json", "lines"), default="json"
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        value = runtime_mapping(resolve_settings(args.file, os.environ))
    except VideoSettingsError as exc:
        print(f"[ERROR] Invalid BFM/Isaac video settings: {exc}", file=sys.stderr)
        return 2
    if args.format == "json":
        print(json.dumps(value, sort_keys=True, separators=(",", ":")))
    else:
        for field in (
            "resolution_width",
            "resolution_height",
            "window_mode",
            "fps_limit",
            "quality",
            "camera_smoothing",
            "screen_percentage",
        ):
            print(value[field])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
