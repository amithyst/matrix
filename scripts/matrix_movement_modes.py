#!/usr/bin/env python3
"""Shared movement-mode identifiers and stable operator-facing metadata."""

from __future__ import annotations

from typing import Final


CAMERA_FACE: Final = "camera_face"
CAMERA_STRAFE: Final = "camera_strafe"
BODY_RELATIVE: Final = "body_relative"
MOVEMENT_MODES: Final = (CAMERA_FACE, CAMERA_STRAFE, BODY_RELATIVE)
DEFAULT_MOVEMENT_MODE: Final = CAMERA_FACE

_MODE_METADATA: Final = {
    CAMERA_FACE: {
        "translation_frame": "camera",
        "facing_policy": "face_movement",
    },
    CAMERA_STRAFE: {
        "translation_frame": "camera",
        "facing_policy": "hold_body_heading",
    },
    BODY_RELATIVE: {
        "translation_frame": "robot_body",
        "facing_policy": "hold_body_heading",
    },
}


def validate_movement_mode(value: object) -> str:
    if not isinstance(value, str) or value not in MOVEMENT_MODES:
        raise ValueError(
            "movement mode must be one of " + ", ".join(MOVEMENT_MODES)
        )
    return value


def next_movement_mode(value: object) -> str:
    current = validate_movement_mode(value)
    return MOVEMENT_MODES[(MOVEMENT_MODES.index(current) + 1) % len(MOVEMENT_MODES)]


def movement_mode_metadata(value: object) -> dict[str, str]:
    mode = validate_movement_mode(value)
    return {"mode": mode, **_MODE_METADATA[mode]}


__all__ = [
    "BODY_RELATIVE",
    "CAMERA_FACE",
    "CAMERA_STRAFE",
    "DEFAULT_MOVEMENT_MODE",
    "MOVEMENT_MODES",
    "movement_mode_metadata",
    "next_movement_mode",
    "validate_movement_mode",
]
