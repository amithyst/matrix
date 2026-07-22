#!/usr/bin/env python3
"""Show a supervised, MC-style X11 settings overlay above Matrix UE.

The cooked Matrix build has no project sources with which to add a native UMG
pause widget.  This process therefore renders a large pointer-driven panel, a
modal InputOnly shield, four click-through crosshair bars, and a shaped visible
proxy for UE's hidden cursor directly on X11.  The override-redirect controls do
not take keyboard focus.  Clicking the command field holds an active X11
keyboard grab only for the bounded editor lifetime.  The child publishes strict
UI *intents* over an inherited socket, while the provider remains the sole owner
of parsing, persistence, the neutral-frame gate, calibration state, and restart
authority.
"""

from __future__ import annotations

import argparse
import ctypes
import ctypes.util
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import signal
import socket
import sys
import tempfile
import time
from typing import Any, Callable

_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

from matrix_mouse_settings import (
    MAX_REMOTE_SPEED_SCALE,
    MIN_REMOTE_SPEED_SCALE,
    canonical_remote_speed_scale,
)
from matrix_mc_commands import MAX_COMMAND_CHARS
from matrix_motion_settings import (
    DOUBLE_TAP_SPEED_FIELD,
    GEARS,
    MotionSettings,
    MotionSettingsError,
    SPEED_FIELD,
    step_motion_speed,
)


_IS_VIEWABLE = 2
_CW_OVERRIDE_REDIRECT = 1 << 9
_CW_EVENT_MASK = 1 << 11
_SHAPE_BOUNDING = 0
_SHAPE_INPUT = 2
_INPUT_ONLY = 2
_BAD_WINDOW = 3
_BAD_DRAWABLE = 9
_X_REQUEST_GET_WINDOW_ATTRIBUTES = 3
_X_REQUEST_QUERY_TREE = 15
_X_REQUEST_GET_GEOMETRY = 14
_X_REQUEST_GET_PROPERTY = 20
_X_REQUEST_TRANSLATE_COORDINATES = 40
_KEY_PRESS = 2
_KEY_RELEASE = 3
_BUTTON_PRESS = 4
_BUTTON_RELEASE = 5
_MOTION_NOTIFY = 6
_KEY_PRESS_MASK = 1 << 0
_KEY_RELEASE_MASK = 1 << 1
_BUTTON_PRESS_MASK = 1 << 2
_BUTTON_RELEASE_MASK = 1 << 3
_BUTTON_1_MOTION_MASK = 1 << 8
_GRAB_SUCCESS = 0
_GRAB_MODE_ASYNC = 1
_CURRENT_TIME = 0
_PR_SET_PDEATHSIG = 1
_CURSOR_WIDTH = 20
_CURSOR_HEIGHT = 28
_MIN_CLIENT_WIDTH = 480
_MIN_CLIENT_HEIGHT = 360
_MAX_COMMAND_HISTORY = 24
_MAX_INTENT_PACKET_BYTES = 2048
_MIN_OVERLAY_FONT_SIZE = 1
_DEFAULT_OVERLAY_FONT_SIZE = 13
_MAX_OVERLAY_FONT_SIZE = 22
_LARGE_FONT_SIZE_DELTA = 5
_BODY_FONT_CANDIDATES = (b"10x20", b"9x15", b"fixed")
_LARGE_FONT_CANDIDATES = (b"12x24", b"10x20", b"fixed")
_XFT_FONT_FAMILIES = ("Noto Sans CJK SC", "WenQuanYi Micro Hei", "sans")


def _xft_font_candidates(font_size: int, *, bold: bool) -> tuple[bytes, ...]:
    weight = ":weight=bold" if bold else ""
    return tuple(
        f"{family}:size={font_size}{weight}".encode("ascii")
        for family in _XFT_FONT_FAMILIES
    )


_XFT_BODY_FONT_CANDIDATES = _xft_font_candidates(
    _DEFAULT_OVERLAY_FONT_SIZE,
    bold=False,
)
_XFT_LARGE_FONT_CANDIDATES = _xft_font_candidates(
    _DEFAULT_OVERLAY_FONT_SIZE + _LARGE_FONT_SIZE_DELTA,
    bold=True,
)

_XK_BACK_SPACE = 0xFF08
_XK_RETURN = 0xFF0D
_XK_ESCAPE = 0xFF1B
_XK_HOME = 0xFF50
_XK_LEFT = 0xFF51
_XK_UP = 0xFF52
_XK_RIGHT = 0xFF53
_XK_DOWN = 0xFF54
_XK_END = 0xFF57
_XK_KP_ENTER = 0xFF8D
_XK_DELETE = 0xFFFF


class XWindowAttributes(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_int),
        ("y", ctypes.c_int),
        ("width", ctypes.c_int),
        ("height", ctypes.c_int),
        ("border_width", ctypes.c_int),
        ("depth", ctypes.c_int),
        ("visual", ctypes.c_void_p),
        ("root", ctypes.c_ulong),
        ("window_class", ctypes.c_int),
        ("bit_gravity", ctypes.c_int),
        ("win_gravity", ctypes.c_int),
        ("backing_store", ctypes.c_int),
        ("backing_planes", ctypes.c_ulong),
        ("backing_pixel", ctypes.c_ulong),
        ("save_under", ctypes.c_int),
        ("colormap", ctypes.c_ulong),
        ("map_installed", ctypes.c_int),
        ("map_state", ctypes.c_int),
        ("all_event_masks", ctypes.c_long),
        ("your_event_mask", ctypes.c_long),
        ("do_not_propagate_mask", ctypes.c_long),
        ("override_redirect", ctypes.c_int),
        ("screen", ctypes.c_void_p),
    ]


class XSetWindowAttributes(ctypes.Structure):
    _fields_ = [
        ("background_pixmap", ctypes.c_ulong),
        ("background_pixel", ctypes.c_ulong),
        ("border_pixmap", ctypes.c_ulong),
        ("border_pixel", ctypes.c_ulong),
        ("bit_gravity", ctypes.c_int),
        ("win_gravity", ctypes.c_int),
        ("backing_store", ctypes.c_int),
        ("backing_planes", ctypes.c_ulong),
        ("backing_pixel", ctypes.c_ulong),
        ("save_under", ctypes.c_int),
        ("event_mask", ctypes.c_long),
        ("do_not_propagate_mask", ctypes.c_long),
        ("override_redirect", ctypes.c_int),
        ("colormap", ctypes.c_ulong),
        ("cursor", ctypes.c_ulong),
    ]


class XColor(ctypes.Structure):
    _fields_ = [
        ("pixel", ctypes.c_ulong),
        ("red", ctypes.c_ushort),
        ("green", ctypes.c_ushort),
        ("blue", ctypes.c_ushort),
        ("flags", ctypes.c_char),
        ("pad", ctypes.c_char),
    ]


class XRectangle(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_short),
        ("y", ctypes.c_short),
        ("width", ctypes.c_ushort),
        ("height", ctypes.c_ushort),
    ]


class XButtonEvent(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_int),
        ("serial", ctypes.c_ulong),
        ("send_event", ctypes.c_int),
        ("display", ctypes.c_void_p),
        ("window", ctypes.c_ulong),
        ("root", ctypes.c_ulong),
        ("subwindow", ctypes.c_ulong),
        ("time", ctypes.c_ulong),
        ("x", ctypes.c_int),
        ("y", ctypes.c_int),
        ("x_root", ctypes.c_int),
        ("y_root", ctypes.c_int),
        ("state", ctypes.c_uint),
        ("button", ctypes.c_uint),
        ("same_screen", ctypes.c_int),
    ]


class XKeyEvent(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_int),
        ("serial", ctypes.c_ulong),
        ("send_event", ctypes.c_int),
        ("display", ctypes.c_void_p),
        ("window", ctypes.c_ulong),
        ("root", ctypes.c_ulong),
        ("subwindow", ctypes.c_ulong),
        ("time", ctypes.c_ulong),
        ("x", ctypes.c_int),
        ("y", ctypes.c_int),
        ("x_root", ctypes.c_int),
        ("y_root", ctypes.c_int),
        ("state", ctypes.c_uint),
        ("keycode", ctypes.c_uint),
        ("same_screen", ctypes.c_int),
    ]


class XMotionEvent(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_int),
        ("serial", ctypes.c_ulong),
        ("send_event", ctypes.c_int),
        ("display", ctypes.c_void_p),
        ("window", ctypes.c_ulong),
        ("root", ctypes.c_ulong),
        ("subwindow", ctypes.c_ulong),
        ("time", ctypes.c_ulong),
        ("x", ctypes.c_int),
        ("y", ctypes.c_int),
        ("x_root", ctypes.c_int),
        ("y_root", ctypes.c_int),
        ("state", ctypes.c_uint),
        ("is_hint", ctypes.c_char),
        ("same_screen", ctypes.c_int),
    ]


class XEvent(ctypes.Union):
    _fields_ = [
        ("type", ctypes.c_int),
        ("xbutton", XButtonEvent),
        ("xkey", XKeyEvent),
        ("xmotion", XMotionEvent),
        ("padding", ctypes.c_long * 24),
    ]


class XErrorEvent(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_int),
        ("display", ctypes.c_void_p),
        ("resourceid", ctypes.c_ulong),
        ("serial", ctypes.c_ulong),
        ("error_code", ctypes.c_ubyte),
        ("request_code", ctypes.c_ubyte),
        ("minor_code", ctypes.c_ubyte),
    ]


_X_ERROR_HANDLER = ctypes.CFUNCTYPE(
    ctypes.c_int,
    ctypes.c_void_p,
    ctypes.POINTER(XErrorEvent),
)


class XFontStruct(ctypes.Structure):
    # Only the leading fields are accessed; Xlib owns the full allocation.
    _fields_ = [("ext_data", ctypes.c_void_p), ("fid", ctypes.c_ulong)]


class XRenderColor(ctypes.Structure):
    _fields_ = [
        ("red", ctypes.c_ushort),
        ("green", ctypes.c_ushort),
        ("blue", ctypes.c_ushort),
        ("alpha", ctypes.c_ushort),
    ]


class XftColor(ctypes.Structure):
    _fields_ = [("pixel", ctypes.c_ulong), ("color", XRenderColor)]


class XGlyphInfo(ctypes.Structure):
    _fields_ = [
        ("width", ctypes.c_ushort),
        ("height", ctypes.c_ushort),
        ("x", ctypes.c_short),
        ("y", ctypes.c_short),
        ("xOff", ctypes.c_short),
        ("yOff", ctypes.c_short),
    ]


@dataclass(frozen=True)
class X11ErrorRecord:
    operation: str
    resource_id: int
    serial: int
    error_code: int
    request_code: int
    minor_code: int

    def mapping(self) -> dict[str, int | str]:
        return {
            "operation": self.operation,
            "resource_id": self.resource_id,
            "serial": self.serial,
            "error_code": self.error_code,
            "request_code": self.request_code,
            "minor_code": self.minor_code,
        }


@dataclass(frozen=True)
class _RecoverableWindowErrorTrap:
    operation: str
    resource_id: int
    error_signatures: tuple[tuple[int, int], ...]


@dataclass(frozen=True)
class WindowGeometry:
    window: int
    x: int
    y: int
    width: int
    height: int

    @property
    def centre(self) -> tuple[int, int]:
        return (self.x + self.width // 2, self.y + self.height // 2)


def polygon_scanline_rectangles(
    vertices: tuple[tuple[int, int], ...],
) -> tuple[tuple[int, int, int, int], ...]:
    """Rasterize a simple polygon into one-pixel-high XFixes rectangles."""

    if len(vertices) < 3:
        raise ValueError("a cursor polygon requires at least three vertices")
    minimum_y = min(y for _x, y in vertices)
    maximum_y = max(y for _x, y in vertices)
    rectangles: list[tuple[int, int, int, int]] = []
    for row in range(minimum_y, maximum_y):
        scan_y = row + 0.5
        intersections: list[float] = []
        for index, (x1, y1) in enumerate(vertices):
            x2, y2 = vertices[(index + 1) % len(vertices)]
            if y1 == y2 or not (min(y1, y2) <= scan_y < max(y1, y2)):
                continue
            fraction = (scan_y - y1) / (y2 - y1)
            intersections.append(x1 + fraction * (x2 - x1))
        intersections.sort()
        for index in range(0, len(intersections) - 1, 2):
            left = math.floor(intersections[index])
            right = math.ceil(intersections[index + 1])
            if right > left:
                rectangles.append((left, row, right - left, 1))
    if not rectangles:
        raise ValueError("cursor polygon rasterized to an empty region")
    return tuple(rectangles)


# A conventional north-west arrow.  Both layers share window origin (0, 0),
# which is the real X11 pointer hotspot; the black outer tip remains visible at
# that exact pixel and the white inset makes the proxy clear on light/dark maps.
_CURSOR_SHADOW_RECTANGLES = polygon_scanline_rectangles(
    ((0, 0), (0, 21), (5, 16), (10, 27), (15, 25), (10, 15), (19, 15))
)
_CURSOR_FOREGROUND_RECTANGLES = polygon_scanline_rectangles(
    ((2, 4), (2, 16), (5, 13), (10, 23), (12, 22), (7, 12), (14, 12))
)


def overlay_supported(geometry: WindowGeometry) -> bool:
    """Small clients fail safely instead of presenting overlapping controls."""

    return bool(
        geometry.width >= _MIN_CLIENT_WIDTH
        and geometry.height >= _MIN_CLIENT_HEIGHT
    )


def overlay_layout(geometry: WindowGeometry) -> dict[str, tuple[int, int, int, int]]:
    """Return root-coordinate geometry for the panel and generous controls."""

    if not overlay_supported(geometry):
        raise ValueError("Matrix client is too small for the safe settings panel")
    centre_x, centre_y = geometry.centre
    compact = geometry.width < 900 or geometry.height < 650
    outer_margin = 16 if compact else 32
    panel_width = min(1180, geometry.width - 2 * outer_margin)
    panel_height = min(790, geometry.height - 2 * outer_margin)
    panel_x = centre_x - panel_width // 2
    panel_y = centre_y - panel_height // 2
    margin = max(18, min(64, panel_width // 18))
    gap = max(10, min(28, panel_width // 36))
    button_height = max(
        36,
        min(52 if compact else 76, panel_height // 8),
    )
    safe_half_size = 50
    centre_panel_y = panel_y + panel_height // 2
    tab_height = 32 if compact else 46
    tab_y = panel_y + (30 if compact else 76)
    tab_gap = 4 if compact else 8
    tab_width = max(1, (panel_width - 2 * margin - 2 * tab_gap) // 3)
    profile_y = centre_panel_y - safe_half_size - gap - button_height
    speed_y = centre_panel_y + safe_half_size + gap
    profile_width = max(1, (panel_width - 2 * margin - gap) // 2)
    speed_width = max(48, min(132, (panel_width - 2 * margin) // 4))
    apply_height = max(42, min(80, button_height + 6))
    footer_space = 8 if compact else 42
    apply_y = panel_y + panel_height - footer_space - apply_height
    console_left = panel_x + margin
    console_width = panel_width - 2 * margin
    console_top = tab_y + tab_height + gap
    console_bottom = apply_y - (6 if compact else gap)
    console_height = max(1, console_bottom - console_top)
    command_input_height = min(28 if compact else 42, max(22, console_height))
    command_input_y = max(console_top, console_bottom - command_input_height)
    result_space = max(0, command_input_y - console_top - 4)
    command_result_height = min(24, result_space)
    command_result_y = command_input_y - 4 - command_result_height
    history_height = max(0, command_result_y - console_top - 4)
    speed_value = (
        panel_x + margin + speed_width,
        speed_y,
        panel_width - 2 * margin - 2 * speed_width,
        button_height,
    )
    recovery_top = centre_panel_y + safe_half_size + gap
    recovery_bottom = apply_y - gap
    recovery_height = max(1, recovery_bottom - recovery_top)
    candidate_gap = 6 if compact else 12
    candidate_width = max(
        1,
        (panel_width - 2 * margin - 2 * candidate_gap) // 3,
    )
    candidate_height = max(28, min(button_height, recovery_height - 30))
    candidate_y = recovery_bottom - candidate_height - (4 if compact else 10)
    locomotion_bottom = centre_panel_y - safe_half_size - gap
    locomotion_top = tab_y + tab_height + gap
    locomotion_height = max(1, locomotion_bottom - locomotion_top)
    locomotion_candidate_gap = 6 if compact else 12
    locomotion_candidate_width = max(
        1,
        (panel_width - 2 * margin - 2 * locomotion_candidate_gap) // 3,
    )
    locomotion_candidate_height = max(
        20,
        min(
            button_height,
            max(20, locomotion_height - (4 if compact else 34)),
        ),
    )
    locomotion_candidate_y = max(
        locomotion_top,
        locomotion_bottom - locomotion_candidate_height,
    )
    motion_outer_gap = 6 if compact else 12
    motion_row_gap = 4 if compact else 8
    motion_top = max(
        tab_y + tab_height + motion_outer_gap,
        profile_y + button_height + motion_outer_gap,
    )
    motion_bottom = speed_y - motion_outer_gap
    motion_row_height = max(
        1,
        (motion_bottom - motion_top - 2 * motion_row_gap) // 3,
    )
    motion_left_x = panel_x + margin
    motion_left_width = max(
        1,
        centre_x - safe_half_size - gap - motion_left_x,
    )
    motion_right_x = centre_x + safe_half_size + gap
    motion_right_width = max(
        1,
        panel_x + panel_width - margin - motion_right_x,
    )
    font_slider_width = max(190, min(340, panel_width // 3))
    result = {
        "shield": (geometry.x, geometry.y, geometry.width, geometry.height),
        "panel": (panel_x, panel_y, panel_width, panel_height),
        "title": (
            panel_x + (24 if compact else 40),
            panel_y + (2 if compact else 24),
            panel_width - (48 if compact else 80),
            18 if compact else 32,
        ),
        "font_size_slider": (
            panel_x + panel_width - margin - font_slider_width,
            panel_y + (2 if compact else 24),
            font_slider_width,
            24 if compact else 32,
        ),
        "tab_loadout": (
            panel_x + margin,
            tab_y,
            tab_width,
            tab_height,
        ),
        "tab_settings": (
            panel_x + margin + tab_width + tab_gap,
            tab_y,
            tab_width,
            tab_height,
        ),
        "tab_console": (
            panel_x + margin + 2 * (tab_width + tab_gap),
            tab_y,
            tab_width,
            tab_height,
        ),
        "horizontal-shadow": (centre_x - 34, centre_y - 3, 69, 7),
        "vertical-shadow": (centre_x - 3, centre_y - 34, 7, 69),
        "horizontal": (centre_x - 32, centre_y - 1, 65, 3),
        "vertical": (centre_x - 1, centre_y - 32, 3, 65),
        "profile_local": (
            panel_x + margin,
            profile_y,
            profile_width,
            button_height,
        ),
        "profile_remote": (
            panel_x + margin + profile_width + gap,
            profile_y,
            profile_width,
            button_height,
        ),
        "speed_down": (panel_x + margin, speed_y, speed_width, button_height),
        "speed_value": speed_value,
        "speed_up": (
            panel_x + panel_width - margin - speed_width,
            speed_y,
            speed_width,
            button_height,
        ),
        "apply_return": (
            panel_x + margin,
            apply_y,
            max(1, panel_width - 2 * margin),
            apply_height,
        ),
        "command_history": (
            console_left,
            console_top,
            console_width,
            history_height,
        ),
        "command_result": (
            console_left,
            command_result_y,
            console_width,
            command_result_height,
        ),
        "command_input": (
            console_left,
            command_input_y,
            console_width,
            command_input_height,
        ),
        "crosshair_safe": (
            centre_x - safe_half_size,
            centre_y - safe_half_size,
            safe_half_size * 2,
            safe_half_size * 2,
        ),
        "locomotion_slot": (
            panel_x + margin,
            locomotion_top,
            panel_width - 2 * margin,
            locomotion_height,
        ),
        "recovery_slot": (
            panel_x + margin,
            recovery_top,
            panel_width - 2 * margin,
            recovery_height,
        ),
    }
    for index in range(3):
        result[f"recovery_policy_{index}"] = (
            panel_x + margin + index * (candidate_width + candidate_gap),
            candidate_y,
            candidate_width,
            candidate_height,
        )
    for index in range(3):
        result[f"locomotion_policy_{index}"] = (
            panel_x
            + margin
            + index
            * (locomotion_candidate_width + locomotion_candidate_gap),
            locomotion_candidate_y,
            locomotion_candidate_width,
            max(1, locomotion_bottom - locomotion_candidate_y),
        )
    for row, gear in enumerate(GEARS):
        row_y = motion_top + row * (motion_row_height + motion_row_gap)
        for field, cell_x, cell_width in (
            (SPEED_FIELD, motion_left_x, motion_left_width),
            (DOUBLE_TAP_SPEED_FIELD, motion_right_x, motion_right_width),
        ):
            button_width = 24 if compact else max(32, min(52, cell_width // 4))
            value_width = max(1, cell_width - 2 * button_width)
            stem = f"motion_{gear}_{field}"
            result[f"{stem}_down"] = (
                cell_x,
                row_y,
                button_width,
                motion_row_height,
            )
            result[f"{stem}_value"] = (
                cell_x + button_width,
                row_y,
                value_width,
                motion_row_height,
            )
            result[f"{stem}_up"] = (
                cell_x + button_width + value_width,
                row_y,
                button_width,
                motion_row_height,
            )
    return result


def font_slider_track(
    rectangle: tuple[int, int, int, int],
) -> tuple[int, int, int, int]:
    """Return the root-coordinate font-slider track inside its hit target."""

    x, y, width, height = rectangle
    label_width = min(104, max(96, width // 3))
    right_padding = max(8, min(16, width // 20))
    track_x = x + label_width
    track_right = x + width - right_padding
    return (track_x, y + height // 2 - 2, max(1, track_right - track_x), 4)


def font_size_from_slider(
    rectangle: tuple[int, int, int, int],
    root_x: int,
) -> int:
    """Map one root-coordinate slider position to the bounded integer size."""

    track_x, _track_y, track_width, _track_height = font_slider_track(rectangle)
    usable_width = max(1, track_width - 1)
    offset = max(0, min(usable_width, root_x - track_x))
    span = _MAX_OVERLAY_FONT_SIZE - _MIN_OVERLAY_FONT_SIZE
    step = int(math.floor((offset / usable_width) * span + 0.5))
    return _MIN_OVERLAY_FONT_SIZE + step


_PANEL_ACTIONS = (
    "profile_local",
    "profile_remote",
    "speed_down",
    "speed_up",
    "apply_return",
)

_MOTION_GEAR_LABELS = {
    "slow": ("慢速", "S"),
    "walk": ("行走", "W"),
    "run": ("奔跑", "R"),
}
_MOTION_FIELD_LABELS = {
    SPEED_FIELD: ("基础", "基"),
    DOUBLE_TAP_SPEED_FIELD: ("双击", "双"),
}
_MOTION_CONTROL_SPECS = tuple(
    (gear, field)
    for gear in GEARS
    for field in (SPEED_FIELD, DOUBLE_TAP_SPEED_FIELD)
)
_MOTION_STEP_ACTION_DETAILS = {
    f"motion_{gear}_{field}_{suffix}": (gear, field, direction)
    for gear, field in _MOTION_CONTROL_SPECS
    for suffix, direction in (("down", -1), ("up", 1))
}
_MOTION_STEP_ACTIONS = tuple(_MOTION_STEP_ACTION_DETAILS)

_PANEL_TABS = ("tab_loadout", "tab_settings", "tab_console")
_OVERLAY_LOCAL_HIT_TARGETS = ("font_size_slider",)
_LOCOMOTION_POLICY_HIT_TARGETS = tuple(
    f"locomotion_policy_{index}" for index in range(3)
)
_POLICY_HIT_TARGETS = tuple(f"recovery_policy_{index}" for index in range(3))
_PANEL_HIT_TARGETS = (
    _PANEL_TABS
    + _PANEL_ACTIONS
    + _MOTION_STEP_ACTIONS
    + ("command_input",)
    + _OVERLAY_LOCAL_HIT_TARGETS
    + _LOCOMOTION_POLICY_HIT_TARGETS
    + _POLICY_HIT_TARGETS
)


def point_in_rectangle(
    point: tuple[int, int], rectangle: tuple[int, int, int, int]
) -> bool:
    x, y = point
    left, top, width, height = rectangle
    return left <= x < left + width and top <= y < top + height


def panel_action_at(
    layout: dict[str, tuple[int, int, int, int]],
    root_x: int,
    root_y: int,
    *,
    page: str | None = None,
) -> str | None:
    """Hit-test X11 root coordinates, including remote-desktop absolute input."""

    targets = _PANEL_HIT_TARGETS
    if page == "loadout":
        targets = (
            _PANEL_TABS
            + ("apply_return",)
            + _LOCOMOTION_POLICY_HIT_TARGETS
            + _POLICY_HIT_TARGETS
        )
    elif page == "settings":
        targets = (
            _PANEL_TABS
            + _PANEL_ACTIONS
            + _MOTION_STEP_ACTIONS
            + _OVERLAY_LOCAL_HIT_TARGETS
        )
    elif page == "console":
        targets = _PANEL_TABS + ("apply_return", "command_input")
    for action in targets:
        rectangle = layout.get(action)
        if rectangle is not None and point_in_rectangle((root_x, root_y), rectangle):
            return action
    return None


@dataclass(frozen=True)
class StrategyPolicyModel:
    policy_id: str
    resident: bool
    available: bool
    display_name: str | None = None
    unavailable_reason: str | None = None


@dataclass(frozen=True)
class StrategyLoadoutModel:
    available: bool
    status: str
    active_slot: str
    locomotion_policy_id: str
    recovery_policy_id: str
    locomotion_candidates: tuple[StrategyPolicyModel, ...]
    recovery_candidates: tuple[StrategyPolicyModel, ...]
    pending_policy_id: str | None

    def policy_enabled(self, policy_id: str, *, slot: str = "recovery") -> bool:
        if not self.available or self.status in {"loading", "switching"}:
            return False
        selected = (
            self.locomotion_policy_id
            if slot == "locomotion"
            else self.recovery_policy_id
        )
        candidates = (
            self.locomotion_candidates
            if slot == "locomotion"
            else self.recovery_candidates
        )
        if policy_id == selected:
            return False
        return any(
            candidate.policy_id == policy_id
            and candidate.available
            and candidate.resident
            for candidate in candidates
        )


def strategy_loadout_model(state: dict[str, object]) -> StrategyLoadoutModel:
    raw = state.get("strategy_loadout")
    if not isinstance(raw, dict) or raw.get("version") != 1:
        return StrategyLoadoutModel(
            False,
            "unavailable",
            "locomotion",
            "sonic",
            "kungfu",
            (),
            (),
            None,
        )
    status = raw.get("status")
    if status not in {"unavailable", "loading", "ready", "switching"}:
        status = "unavailable"
    active_slot = raw.get("active_slot")
    if active_slot not in {"locomotion", "recovery"}:
        active_slot = "locomotion"
    locomotion = "sonic"
    recovery = "kungfu"
    locomotion_candidates: list[StrategyPolicyModel] = []
    recovery_candidates: list[StrategyPolicyModel] = []
    slots = raw.get("slots")
    if isinstance(slots, list):
        for slot in slots:
            if not isinstance(slot, dict):
                continue
            slot_id = slot.get("slot")
            selected = slot.get("selected_policy_id")
            if slot_id == "locomotion" and isinstance(selected, str):
                locomotion = selected
                raw_candidates = slot.get("candidates")
                if isinstance(raw_candidates, list):
                    for candidate in raw_candidates[:3]:
                        if not isinstance(candidate, dict):
                            continue
                        policy_id = candidate.get("policy_id")
                        if not isinstance(policy_id, str) or not policy_id:
                            continue
                        locomotion_candidates.append(
                            StrategyPolicyModel(
                                policy_id=policy_id,
                                resident=candidate.get("resident") is True,
                                available=candidate.get("available") is True,
                                display_name=(
                                    candidate.get("name")
                                    if isinstance(candidate.get("name"), str)
                                    else None
                                ),
                                unavailable_reason=(
                                    candidate.get("unavailable_reason")
                                    if isinstance(
                                        candidate.get("unavailable_reason"), str
                                    )
                                    else None
                                ),
                            )
                        )
            elif slot_id == "recovery" and isinstance(selected, str):
                recovery = selected
                raw_candidates = slot.get("candidates")
                if isinstance(raw_candidates, list):
                    for candidate in raw_candidates[:3]:
                        if not isinstance(candidate, dict):
                            continue
                        policy_id = candidate.get("policy_id")
                        if not isinstance(policy_id, str) or not policy_id:
                            continue
                        recovery_candidates.append(
                            StrategyPolicyModel(
                                policy_id=policy_id,
                                resident=candidate.get("resident") is True,
                                available=candidate.get("available") is True,
                                display_name=(
                                    candidate.get("name")
                                    if isinstance(candidate.get("name"), str)
                                    else None
                                ),
                                unavailable_reason=(
                                    candidate.get("unavailable_reason")
                                    if isinstance(
                                        candidate.get("unavailable_reason"), str
                                    )
                                    else None
                                ),
                            )
                        )
    pending = raw.get("pending")
    pending_policy_id = (
        pending.get("policy_id")
        if isinstance(pending, dict) and isinstance(pending.get("policy_id"), str)
        else None
    )
    return StrategyLoadoutModel(
        available=raw.get("available") is True,
        status=status,
        active_slot=active_slot,
        locomotion_policy_id=locomotion,
        recovery_policy_id=recovery,
        locomotion_candidates=tuple(locomotion_candidates),
        recovery_candidates=tuple(recovery_candidates),
        pending_policy_id=pending_policy_id,
    )


def read_active_state(path: Path) -> bool:
    """Fail closed (hidden) on missing, partial, stale-version, or invalid state."""

    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, UnicodeError, json.JSONDecodeError):
        return False
    return bool(
        isinstance(value, dict)
        and value.get("version") == 1
        and value.get("active") is True
    )


def read_overlay_state(path: Path) -> dict[str, object] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, UnicodeError, json.JSONDecodeError):
        return None
    if not isinstance(value, dict) or value.get("version") != 1:
        return None
    return value


@dataclass(frozen=True)
class SettingsPanelModel:
    current_profile: str
    current_scale: float
    next_profile: str
    next_scale: float
    pending_restart: bool
    restart_available: bool
    restart_requested: bool
    base_mirror_gain: float
    effective_mirror_gain: float
    status: str
    error: str | None

    @property
    def apply_label(self) -> str:
        if self.restart_requested or self.status == "restarting":
            return "RELOADING MATRIX..."
        if self.pending_restart and not self.restart_available:
            return "APPLY UNAVAILABLE"
        if self.pending_restart:
            return "RETURN TO GAME & APPLY"
        return "RETURN TO GAME"

    def action_enabled(self, action: str) -> bool:
        controls_disabled = self.restart_requested or self.status == "restarting"
        if action in {"profile_local", "profile_remote"}:
            return not controls_disabled
        if action == "speed_down":
            return bool(
                not controls_disabled
                and self.next_profile == "Remote"
                and self.next_scale > MIN_REMOTE_SPEED_SCALE
            )
        if action == "speed_up":
            return bool(
                not controls_disabled
                and self.next_profile == "Remote"
                and self.next_scale < MAX_REMOTE_SPEED_SCALE
            )
        if action == "apply_return":
            return bool(
                not controls_disabled
                and (not self.pending_restart or self.restart_available)
            )
        return False


@dataclass(frozen=True)
class MotionSettingsPanelModel:
    settings: MotionSettings
    available: bool
    load_status: str
    error: str | None

    def value(self, gear: str, field: str) -> float:
        return self.settings.value_for_path(f"control.motion.gears.{gear}.{field}")

    def action_enabled(self, action: str) -> bool:
        return motion_step_target(self, action) is not None


def _motion_settings_candidate(state: dict[str, object]) -> object:
    direct = state.get("motion_settings")
    if direct is not None:
        return direct
    game_commands = state.get("game_commands")
    if (
        isinstance(game_commands, dict)
        and game_commands.get("motion_settings") is not None
    ):
        return game_commands.get("motion_settings")
    console = state.get("command_console")
    console = console if isinstance(console, dict) else {}
    data = console.get("data")
    data = data if isinstance(data, dict) else {}
    return data.get("motion_settings")


def motion_settings_panel_model(state: dict[str, object]) -> MotionSettingsPanelModel:
    """Validate the six runtime-owned motion values used by panel step buttons."""

    raw = _motion_settings_candidate(state)
    load_status = "unavailable"
    load_error: str | None = None
    snapshot = raw
    if isinstance(raw, dict) and "settings" in raw:
        snapshot = raw.get("settings")
        if isinstance(raw.get("load_status"), str):
            load_status = raw["load_status"]
        if isinstance(raw.get("load_error"), str) and raw.get("load_error"):
            load_error = str(raw["load_error"])
    try:
        settings = MotionSettings.from_mapping(snapshot)
    except (MotionSettingsError, TypeError, ValueError) as exc:
        return MotionSettingsPanelModel(
            settings=MotionSettings(),
            available=False,
            load_status="unavailable",
            error=(
                "motion settings unavailable"
                if raw is None
                else f"invalid motion settings telemetry: {exc}"
            ),
        )
    return MotionSettingsPanelModel(
        settings=settings,
        available=True,
        load_status=load_status if load_status != "unavailable" else "loaded",
        error=load_error,
    )


def motion_step_target(
    model: MotionSettingsPanelModel,
    action: str,
) -> float | None:
    """Return the adjacent validated preset for one strict panel action."""

    if not isinstance(model, MotionSettingsPanelModel):
        raise TypeError("motion panel model is required")
    details = _MOTION_STEP_ACTION_DETAILS.get(action)
    if details is None:
        raise ValueError(f"unsupported motion panel action: {action}")
    if not model.available:
        return None
    gear, field, direction = details
    path = f"control.motion.gears.{gear}.{field}"
    current = model.settings.value_for_path(path)
    target = step_motion_speed(model.settings, path, direction)
    return None if math.isclose(target, current, rel_tol=0.0, abs_tol=1e-12) else target


def motion_step_command(
    model: MotionSettingsPanelModel,
    action: str,
) -> str | None:
    """Build one standard MC data command without mutating any local config."""

    target = motion_step_target(model, action)
    if target is None:
        return None
    gear, field, _direction = _MOTION_STEP_ACTION_DETAILS[action]
    return (
        f"/data modify entity @s control.motion.gears.{gear}.{field} "
        f"set value {target:.2f}"
    )


def motion_value_label(
    model: MotionSettingsPanelModel,
    gear: str,
    field: str,
    *,
    compact: bool,
) -> str:
    """Return a bounded label for one of the six visible motion values."""

    if (gear, field) not in _MOTION_CONTROL_SPECS:
        raise ValueError("unsupported motion value label")
    value = model.value(gear, field)
    if compact:
        compact_value = f"{value:.2f}".rstrip("0").rstrip(".")
        if compact_value.startswith("0."):
            compact_value = compact_value[1:]
        return (
            f"{_MOTION_GEAR_LABELS[gear][1]}"
            f"{_MOTION_FIELD_LABELS[field][1]}{compact_value}"
        )
    return (
        f"{_MOTION_GEAR_LABELS[gear][0]}{_MOTION_FIELD_LABELS[field][0]} "
        f"{value:.2f} m/s"
    )


_COMMAND_STATUSES = frozenset(
    {"unavailable", "idle", "editing", "pending", "success", "error", "restarting"}
)
_WARNING_DISPLAY_ALIASES = {
    "已兼容执行；标准命令是 /summon": (
        "Accepted /summom alias; standard command is /summon"
    ),
}


def _bounded_status_text(value: object, *, maximum: int) -> str | None:
    if not isinstance(value, str) or not value:
        return None
    cleaned = "".join(
        character if ord(character) >= 0x20 and ord(character) != 0x7F else "?"
        for character in value[:maximum]
    )
    return cleaned or None


@dataclass(frozen=True)
class CommandConsoleStatus:
    available: bool
    provider_editing: bool
    in_flight: bool
    status: str
    request_id: str | None
    sequence: int | None
    result_revision: int
    ok: bool | None
    code: str | None
    message: str | None
    warning: str | None
    restart_required: bool
    outcome_unknown: bool

    @property
    def result_identity(self) -> tuple[object, ...]:
        """Identity used to distinguish a new response from stale state JSON."""

        return (
            self.in_flight,
            self.status,
            self.request_id,
            self.sequence,
            self.result_revision,
            self.ok,
            self.code,
            self.message,
            self.warning,
            self.restart_required,
            self.outcome_unknown,
        )


def command_console_status(state: dict[str, object]) -> CommandConsoleStatus:
    """Validate the provider's command result before rendering or gating input."""

    raw = state.get("command_console")
    raw = raw if isinstance(raw, dict) else {}
    status_value = raw.get("status")
    status = (
        status_value
        if isinstance(status_value, str) and status_value in _COMMAND_STATUSES
        else "unavailable"
    )
    sequence_value = raw.get("sequence")
    sequence = (
        sequence_value
        if type(sequence_value) is int and 1 <= sequence_value < 2**63
        else None
    )
    revision_value = raw.get("result_revision", 0)
    result_revision = (
        revision_value
        if type(revision_value) is int and 0 <= revision_value < 2**63
        else 0
    )
    ok_value = raw.get("ok")
    ok = ok_value if type(ok_value) is bool else None
    warning = _bounded_status_text(raw.get("warning"), maximum=512)
    warning = _WARNING_DISPLAY_ALIASES.get(warning, warning)
    return CommandConsoleStatus(
        available=raw.get("available") is True,
        provider_editing=raw.get("editing") is True,
        in_flight=raw.get("in_flight") is True,
        status=status,
        request_id=_bounded_status_text(raw.get("request_id"), maximum=128),
        sequence=sequence,
        result_revision=result_revision,
        ok=ok,
        code=_bounded_status_text(raw.get("code"), maximum=64),
        message=_bounded_status_text(raw.get("message"), maximum=512),
        warning=warning,
        restart_required=raw.get("restart_required") is True,
        outcome_unknown=raw.get("outcome_unknown") is True,
    )


@dataclass(frozen=True)
class CommandEditOutcome:
    action: str | None = None
    command: str | None = None


class CommandLineEditor:
    """Small bounded ASCII editor; execution remains provider/runtime authority."""

    def __init__(self) -> None:
        self.text = ""
        self.cursor = 0
        self.history: list[str] = []
        self.history_index: int | None = None
        self._history_draft = ""
        self.editing = False
        self.pending = False
        self.revision = 0
        self._pending_baseline: tuple[object, ...] | None = None
        self._pending_acknowledged = False

    def _changed(self) -> None:
        self.revision += 1

    def begin(self) -> bool:
        if self.editing or self.pending:
            return False
        self.editing = True
        self.history_index = None
        self._history_draft = ""
        self._changed()
        return True

    def end(self, *, force: bool = False) -> bool:
        if self.pending and not force:
            return False
        changed = bool(self.editing or self.text or self.pending)
        self.editing = False
        self.pending = False
        self.text = ""
        self.cursor = 0
        self.history_index = None
        self._history_draft = ""
        self._pending_baseline = None
        self._pending_acknowledged = False
        if changed:
            self._changed()
        return changed

    def _leave_history_navigation(self) -> None:
        self.history_index = None
        self._history_draft = ""

    def _replace_text(self, text: str) -> None:
        self.text = text[:MAX_COMMAND_CHARS]
        self.cursor = len(self.text)
        self._changed()

    def _history_up(self) -> bool:
        if not self.history:
            return False
        if self.history_index is None:
            self._history_draft = self.text
            self.history_index = len(self.history) - 1
        elif self.history_index > 0:
            self.history_index -= 1
        else:
            return False
        self._replace_text(self.history[self.history_index])
        return True

    def _history_down(self) -> bool:
        if self.history_index is None:
            return False
        if self.history_index + 1 < len(self.history):
            self.history_index += 1
            replacement = self.history[self.history_index]
        else:
            self.history_index = None
            replacement = self._history_draft
            self._history_draft = ""
        self._replace_text(replacement)
        return True

    def _submit(self, status: CommandConsoleStatus) -> CommandEditOutcome:
        if (
            not status.available
            or status.in_flight
            or status.outcome_unknown
            or status.status in {"pending", "restarting", "unavailable"}
        ):
            return CommandEditOutcome()
        command = self.text.strip()
        if not command:
            return CommandEditOutcome()
        if len(command) > MAX_COMMAND_CHARS or any(
            ord(character) < 0x20 or ord(character) > 0x7E
            for character in command
        ):
            return CommandEditOutcome()
        if not self.history or self.history[-1] != command:
            self.history.append(command)
            del self.history[:-_MAX_COMMAND_HISTORY]
        self.text = ""
        self.cursor = 0
        self.history_index = None
        self._history_draft = ""
        self.pending = True
        self._pending_baseline = status.result_identity
        self._pending_acknowledged = False
        self._changed()
        return CommandEditOutcome(action="submit", command=command)

    def reconcile(self, status: CommandConsoleStatus) -> bool:
        """Clear the local pending latch only after a new terminal provider result."""

        if not self.pending:
            return False
        if (
            status.in_flight
            or status.status in {"pending", "restarting"}
            or status.result_identity != self._pending_baseline
        ):
            self._pending_acknowledged = True
        if (
            self._pending_acknowledged
            and not status.in_flight
            and status.status in {"success", "error"}
        ):
            self.pending = False
            self._pending_baseline = None
            self._pending_acknowledged = False
            self._changed()
            return True
        return False

    def handle_key(
        self,
        *,
        keysym: int,
        printable: str,
        status: CommandConsoleStatus,
    ) -> CommandEditOutcome:
        if not self.editing or self.pending:
            return CommandEditOutcome()
        if keysym == _XK_ESCAPE:
            self.end()
            return CommandEditOutcome(action="end")
        if keysym in {_XK_RETURN, _XK_KP_ENTER}:
            return self._submit(status)
        if keysym == _XK_LEFT:
            if self.cursor > 0:
                self.cursor -= 1
                self._changed()
            return CommandEditOutcome()
        if keysym == _XK_RIGHT:
            if self.cursor < len(self.text):
                self.cursor += 1
                self._changed()
            return CommandEditOutcome()
        if keysym == _XK_HOME:
            if self.cursor != 0:
                self.cursor = 0
                self._changed()
            return CommandEditOutcome()
        if keysym == _XK_END:
            if self.cursor != len(self.text):
                self.cursor = len(self.text)
                self._changed()
            return CommandEditOutcome()
        if keysym == _XK_UP:
            self._history_up()
            return CommandEditOutcome()
        if keysym == _XK_DOWN:
            self._history_down()
            return CommandEditOutcome()
        if keysym == _XK_BACK_SPACE:
            if self.cursor > 0:
                self._leave_history_navigation()
                self.text = self.text[: self.cursor - 1] + self.text[self.cursor :]
                self.cursor -= 1
                self._changed()
            return CommandEditOutcome()
        if keysym == _XK_DELETE:
            if self.cursor < len(self.text):
                self._leave_history_navigation()
                self.text = self.text[: self.cursor] + self.text[self.cursor + 1 :]
                self._changed()
            return CommandEditOutcome()
        if printable and all(0x20 <= ord(character) <= 0x7E for character in printable):
            available = MAX_COMMAND_CHARS - len(self.text)
            addition = printable[:available]
            if addition:
                self._leave_history_navigation()
                self.text = self.text[: self.cursor] + addition + self.text[self.cursor :]
                self.cursor += len(addition)
                self._changed()
        return CommandEditOutcome()

    def display_line(self, maximum_characters: int) -> str:
        """Return a cursor-bearing window into the current command."""

        maximum = max(1, int(maximum_characters))
        if not self.editing and not self.text:
            return "点击输入 /summon、/tp 或 /policy"[:maximum]
        content_width = max(1, maximum - 1)
        start = max(0, self.cursor - content_width // 2)
        start = min(start, max(0, len(self.text) - content_width))
        visible = self.text[start : start + content_width]
        cursor = min(len(visible), max(0, self.cursor - start))
        return (visible[:cursor] + "|" + visible[cursor:])[:maximum]


def settings_panel_model(state: dict[str, object]) -> SettingsPanelModel:
    """Validate untrusted JSON state before it reaches any drawing primitive."""

    settings = state.get("mouse_settings")
    settings = settings if isinstance(settings, dict) else {}
    current = settings.get("current")
    current = current if isinstance(current, dict) else {}
    next_launch = settings.get("next_launch")
    next_launch = next_launch if isinstance(next_launch, dict) else {}
    restart = state.get("restart")
    restart = restart if isinstance(restart, dict) else {}
    mirror = state.get("mirror_sensitivity")
    mirror = mirror if isinstance(mirror, dict) else {}
    apply_return = state.get("apply_return")
    apply_return = apply_return if isinstance(apply_return, dict) else {}

    def profile(value: object) -> str:
        return "Remote" if value == "remote" else "Local"

    def finite(value: object, fallback: float) -> float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return fallback
        number = float(value)
        return number if math.isfinite(number) else fallback

    def preset(value: object) -> float:
        try:
            return canonical_remote_speed_scale(value)
        except ValueError:
            return 1.0

    current_scale = preset(current.get("effective_scale"))
    next_scale = preset(next_launch.get("effective_scale"))
    pending = settings.get("pending_restart") is True
    requested = restart.get("requested") is True
    restart_available = restart.get("available") is True
    persistence_error = settings.get("persistence_error")
    restart_error = restart.get("error")
    action_error = apply_return.get("error")
    error_value = next(
        (
            value
            for value in (persistence_error, restart_error, action_error)
            if isinstance(value, str) and value
        ),
        None,
    )
    if pending and not requested and not restart_available and error_value is None:
        error_value = "whole-runtime reload is unavailable"
    action_status = apply_return.get("status")
    status = action_status if isinstance(action_status, str) else "idle"
    if requested:
        status = "restarting"
    elif error_value is not None:
        status = "error"
    elif status not in {"waiting_neutral", "returning", "restarting", "error"}:
        status = "pending" if pending else "ready"
    return SettingsPanelModel(
        current_profile=profile(current.get("profile")),
        current_scale=current_scale,
        next_profile=profile(next_launch.get("profile")),
        next_scale=next_scale,
        pending_restart=pending,
        restart_available=restart_available,
        restart_requested=requested,
        base_mirror_gain=finite(
            mirror.get("base_deg_per_raw_unit", mirror.get("base_deg_per_px")),
            0.0,
        ),
        effective_mirror_gain=finite(
            mirror.get(
                "effective_deg_per_raw_unit",
                mirror.get("effective_deg_per_px"),
            ),
            0.0,
        ),
        status=status,
        error=error_value,
    )


def settings_hint_lines(state: dict[str, object]) -> tuple[bytes, bytes, bytes]:
    """Keep a compact textual representation for logs and unit diagnostics."""

    model = settings_panel_model(state)
    line1 = (
        f"CURRENT APPLIED (SDL): {model.current_profile} "
        f"{model.current_scale:.2f}x | "
        f"NEXT LAUNCH: {model.next_profile} {model.next_scale:.2f}x | "
        f"{'PENDING RESTART' if model.pending_restart else 'CURRENT'}"
    )
    line2 = (
        f"XI2 raw mirror: base {model.base_mirror_gain:.3f} -> effective "
        f"{model.effective_mirror_gain:.3f} deg/raw | "
        "presets 0.01-0.10/0.01, 0.20-1.00/0.10 | "
        f"{'ERROR' if model.error else 'SAVED'}"
    )
    line3 = (
        f"Mouse buttons configure ({model.apply_label}) | "
        "Click command box; Enter runs; ESC leaves editor first"
    )
    encoded = tuple(
        line.encode("ascii", errors="replace")
        for line in (line1, line2, line3)
    )
    return (encoded[0], encoded[1], encoded[2])


def atomic_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        dir=path.parent,
        prefix=f".{path.name}.",
        delete=False,
    ) as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
        temporary = Path(stream.name)
    os.replace(temporary, path)


class PointerActionPublisher:
    """Publish strict one-shot overlay intents on the inherited seqpacket."""

    def __init__(self, *, file_descriptor: int, session: str) -> None:
        if file_descriptor < 0:
            raise ValueError("action file descriptor must be non-negative")
        if not session or len(session) > 128:
            raise ValueError("action session must be non-empty and bounded")
        self._socket = socket.socket(fileno=file_descriptor)
        self._socket.setblocking(False)
        self._session = session
        self._sequence = 0

    def _publish(self, kind: str, extra: dict[str, object]) -> None:
        self._sequence += 1
        payload = json.dumps(
            {
                "version": 1,
                "session": self._session,
                "sequence": self._sequence,
                "kind": kind,
                **extra,
            },
            separators=(",", ":"),
            sort_keys=True,
            allow_nan=False,
        ).encode("ascii")
        if len(payload) > _MAX_INTENT_PACKET_BYTES:
            raise RuntimeError("overlay intent packet is oversized")
        try:
            sent = self._socket.send(payload)
        except BlockingIOError as exc:
            raise RuntimeError("overlay intent channel is full") from exc
        if sent != len(payload):
            raise RuntimeError(
                f"partial overlay intent packet: sent {sent}/{len(payload)}"
            )

    def publish(self, action: str) -> None:
        """Compatibility entry point for the existing pointer actions."""

        if action not in _PANEL_ACTIONS:
            raise ValueError(f"unsupported pointer action: {action}")
        self._publish("action", {"action": action})

    def publish_command_edit(self, active: bool) -> None:
        if type(active) is not bool:
            raise ValueError("command edit active flag must be boolean")
        self._publish("command_edit", {"active": active})

    def publish_command_submit(self, command: str) -> None:
        if (
            not isinstance(command, str)
            or not command
            or len(command) > MAX_COMMAND_CHARS
            or any(
                ord(character) < 0x20 or ord(character) > 0x7E
                for character in command
            )
        ):
            raise ValueError("command submit text must be bounded printable ASCII")
        self._publish("command_submit", {"command": command})

    def publish_strategy_select(self, slot: str, policy_id: str) -> None:
        if slot not in {"locomotion", "recovery"}:
            raise ValueError("strategy slot is invalid")
        if (
            not isinstance(policy_id, str)
            or not policy_id
            or len(policy_id) > 64
            or any(
                not (
                    character.isascii()
                    and (character.isalnum() or character in "._-")
                )
                for character in policy_id
            )
        ):
            raise ValueError("strategy policy id is invalid")
        self._publish(
            "strategy_select",
            {"slot": slot, "policy_id": policy_id.lower()},
        )

    def close(self) -> None:
        self._socket.close()


def arm_parent_death_signal(expected_parent_pid: int) -> None:
    """Ask Linux to terminate this overlay if its provider disappears."""

    if expected_parent_pid <= 1:
        raise ValueError("expected parent PID must be greater than 1")
    libc = ctypes.CDLL(None, use_errno=True)
    prctl = libc.prctl
    prctl.argtypes = [
        ctypes.c_int,
        ctypes.c_ulong,
        ctypes.c_ulong,
        ctypes.c_ulong,
        ctypes.c_ulong,
    ]
    prctl.restype = ctypes.c_int
    if prctl(_PR_SET_PDEATHSIG, signal.SIGTERM, 0, 0, 0) != 0:
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error))
    # The parent may have died between fork/exec and prctl().
    if os.getppid() != expected_parent_pid:
        raise RuntimeError(
            f"overlay parent changed: expected {expected_parent_pid}, "
            f"actual {os.getppid()}"
        )


class X11CalibrationOverlay:
    """Raw Xlib/XFixes overlay with no Python GUI-package dependency."""

    _RAISE_INTERVAL_S = 1.0

    _VISUAL_WINDOW_ORDER = (
        "panel",
        "horizontal-shadow",
        "vertical-shadow",
        "horizontal",
        "vertical",
    )
    _CURSOR_WINDOW_ORDER = ("cursor-shadow", "cursor")
    _WINDOW_ORDER = ("shield",) + _VISUAL_WINDOW_ORDER + _CURSOR_WINDOW_ORDER

    def __init__(
        self,
        *,
        display_name: str | None,
        expected_ue_pid: int,
        x11: Any | None = None,
        xfixes: Any | None = None,
        xft: Any | None = None,
    ) -> None:
        if expected_ue_pid <= 1:
            raise ValueError("expected UE PID must be greater than 1")
        x11_injected = x11 is not None
        if x11 is None:
            name = ctypes.util.find_library("X11")
            if not name:
                raise RuntimeError("libX11 was not found")
            x11 = ctypes.CDLL(name)
        if xfixes is None:
            name = ctypes.util.find_library("Xfixes")
            if not name:
                raise RuntimeError("libXfixes was not found")
            xfixes = ctypes.CDLL(name)
        if xft is None and not x11_injected:
            name = ctypes.util.find_library("Xft")
            if name:
                xft = ctypes.CDLL(name)
        self._x11 = x11
        self._xfixes = xfixes
        self._xft = xft
        self._x_error_handler_callback: _X_ERROR_HANDLER | None = None
        self._previous_x_error_handler_address: int | None = None
        self._previous_x_error_handler: _X_ERROR_HANDLER | None = None
        self._window_error_trap: _RecoverableWindowErrorTrap | None = None
        self._trapped_window_error: X11ErrorRecord | None = None
        self._recoverable_window_error_count = 0
        self._bad_window_count = 0
        self._bad_drawable_count = 0
        self._last_recoverable_window_error: X11ErrorRecord | None = None
        self._last_bad_window: X11ErrorRecord | None = None
        self._configure_signatures()
        encoded_display = display_name.encode() if display_name else None
        self._display = self._x11.XOpenDisplay(encoded_display)
        if not self._display:
            label = display_name or os.environ.get("DISPLAY", "<unset>")
            raise RuntimeError(f"cannot open X11 display {label}")
        self._install_x_error_handler()
        self._screen = int(self._x11.XDefaultScreen(self._display))
        self._root = int(self._x11.XRootWindow(self._display, self._screen))
        self._visual = (
            self._x11.XDefaultVisual(self._display, self._screen)
            if self._xft is not None
            else None
        )
        self._colormap = int(
            self._x11.XDefaultColormap(self._display, self._screen)
        )
        self._pid_atom = int(
            self._x11.XInternAtom(self._display, b"_NET_WM_PID", 0)
        )
        extension_event = ctypes.c_int()
        extension_error = ctypes.c_int()
        if not self._xfixes.XFixesQueryExtension(
            self._display,
            ctypes.byref(extension_event),
            ctypes.byref(extension_error),
        ):
            self.close()
            raise RuntimeError("XFixes extension is unavailable")
        self.expected_ue_pid = expected_ue_pid
        self._windows: dict[str, int] = {}
        self._panel_gc: int | None = None
        self._body_font: ctypes.POINTER(XFontStruct) | None = None
        self._large_font: ctypes.POINTER(XFontStruct) | None = None
        self._body_font_name: str | None = None
        self._large_font_name: str | None = None
        self._xft_draw: int | None = None
        self._xft_body_font: int | None = None
        self._xft_large_font: int | None = None
        self._xft_body_font_name: str | None = None
        self._xft_large_font_name: str | None = None
        self._xft_colours: dict[int, XftColor] = {}
        self._font_size = _DEFAULT_OVERLAY_FONT_SIZE
        self._last_rendered_font_size: int | None = None
        self._font_slider_dragging = False
        self._colours: dict[str, int] = {}
        self._visible = False
        self._cursor_visible = False
        self._last_layout: dict[str, tuple[int, int, int, int]] | None = None
        self._last_geometry: WindowGeometry | None = None
        self._last_panel_model: SettingsPanelModel | None = None
        self._last_motion_model: MotionSettingsPanelModel | None = None
        self._last_strategy_model: StrategyLoadoutModel | None = None
        self._last_page: str | None = None
        self._last_command_status = command_console_status({})
        self._last_command_revision = -1
        self._last_pointer: tuple[int, int] | None = None
        self._last_raise_s: float | None = None
        self._pressed_action: str | None = None
        self._pressed_window: int | None = None
        self._target_window: int | None = None
        self._command_editor = CommandLineEditor()
        self._keyboard_grabbed = False
        self._deferred_ungrab_keycode: int | None = None
        self._active_page = "loadout"
        self._create_windows()

    def _configure_signatures(self) -> None:
        signatures = {
            "XOpenDisplay": ([ctypes.c_char_p], ctypes.c_void_p),
            "XDefaultScreen": ([ctypes.c_void_p], ctypes.c_int),
            "XDefaultVisual": ([ctypes.c_void_p, ctypes.c_int], ctypes.c_void_p),
            "XRootWindow": ([ctypes.c_void_p, ctypes.c_int], ctypes.c_ulong),
            "XDefaultColormap": ([ctypes.c_void_p, ctypes.c_int], ctypes.c_ulong),
            "XBlackPixel": ([ctypes.c_void_p, ctypes.c_int], ctypes.c_ulong),
            "XWhitePixel": ([ctypes.c_void_p, ctypes.c_int], ctypes.c_ulong),
            "XInternAtom": (
                [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int],
                ctypes.c_ulong,
            ),
            "XGetWindowProperty": (
                [
                    ctypes.c_void_p,
                    ctypes.c_ulong,
                    ctypes.c_ulong,
                    ctypes.c_long,
                    ctypes.c_long,
                    ctypes.c_int,
                    ctypes.c_ulong,
                    ctypes.POINTER(ctypes.c_ulong),
                    ctypes.POINTER(ctypes.c_int),
                    ctypes.POINTER(ctypes.c_ulong),
                    ctypes.POINTER(ctypes.c_ulong),
                    ctypes.POINTER(ctypes.POINTER(ctypes.c_ubyte)),
                ],
                ctypes.c_int,
            ),
            "XQueryPointer": (
                [
                    ctypes.c_void_p,
                    ctypes.c_ulong,
                    ctypes.POINTER(ctypes.c_ulong),
                    ctypes.POINTER(ctypes.c_ulong),
                    ctypes.POINTER(ctypes.c_int),
                    ctypes.POINTER(ctypes.c_int),
                    ctypes.POINTER(ctypes.c_int),
                    ctypes.POINTER(ctypes.c_int),
                    ctypes.POINTER(ctypes.c_uint),
                ],
                ctypes.c_int,
            ),
            "XQueryTree": (
                [
                    ctypes.c_void_p,
                    ctypes.c_ulong,
                    ctypes.POINTER(ctypes.c_ulong),
                    ctypes.POINTER(ctypes.c_ulong),
                    ctypes.POINTER(ctypes.POINTER(ctypes.c_ulong)),
                    ctypes.POINTER(ctypes.c_uint),
                ],
                ctypes.c_int,
            ),
            "XGetWindowAttributes": (
                [ctypes.c_void_p, ctypes.c_ulong, ctypes.POINTER(XWindowAttributes)],
                ctypes.c_int,
            ),
            "XTranslateCoordinates": (
                [
                    ctypes.c_void_p,
                    ctypes.c_ulong,
                    ctypes.c_ulong,
                    ctypes.c_int,
                    ctypes.c_int,
                    ctypes.POINTER(ctypes.c_int),
                    ctypes.POINTER(ctypes.c_int),
                    ctypes.POINTER(ctypes.c_ulong),
                ],
                ctypes.c_int,
            ),
            "XAllocNamedColor": (
                [
                    ctypes.c_void_p,
                    ctypes.c_ulong,
                    ctypes.c_char_p,
                    ctypes.POINTER(XColor),
                    ctypes.POINTER(XColor),
                ],
                ctypes.c_int,
            ),
            "XCreateSimpleWindow": (
                [
                    ctypes.c_void_p,
                    ctypes.c_ulong,
                    ctypes.c_int,
                    ctypes.c_int,
                    ctypes.c_uint,
                    ctypes.c_uint,
                    ctypes.c_uint,
                    ctypes.c_ulong,
                    ctypes.c_ulong,
                ],
                ctypes.c_ulong,
            ),
            "XCreateWindow": (
                [
                    ctypes.c_void_p,
                    ctypes.c_ulong,
                    ctypes.c_int,
                    ctypes.c_int,
                    ctypes.c_uint,
                    ctypes.c_uint,
                    ctypes.c_uint,
                    ctypes.c_int,
                    ctypes.c_uint,
                    ctypes.c_void_p,
                    ctypes.c_ulong,
                    ctypes.POINTER(XSetWindowAttributes),
                ],
                ctypes.c_ulong,
            ),
            "XChangeWindowAttributes": (
                [
                    ctypes.c_void_p,
                    ctypes.c_ulong,
                    ctypes.c_ulong,
                    ctypes.POINTER(XSetWindowAttributes),
                ],
                ctypes.c_int,
            ),
            "XSelectInput": (
                [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_long],
                ctypes.c_int,
            ),
            "XGrabKeyboard": (
                [
                    ctypes.c_void_p,
                    ctypes.c_ulong,
                    ctypes.c_int,
                    ctypes.c_int,
                    ctypes.c_int,
                    ctypes.c_ulong,
                ],
                ctypes.c_int,
            ),
            "XUngrabKeyboard": (
                [ctypes.c_void_p, ctypes.c_ulong],
                ctypes.c_int,
            ),
            "XLookupString": (
                [
                    ctypes.POINTER(XKeyEvent),
                    ctypes.c_void_p,
                    ctypes.c_int,
                    ctypes.POINTER(ctypes.c_ulong),
                    ctypes.c_void_p,
                ],
                ctypes.c_int,
            ),
            "XQueryKeymap": ([ctypes.c_void_p, ctypes.c_void_p], ctypes.c_int),
            "XStoreName": (
                [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_char_p],
                ctypes.c_int,
            ),
            "XCreateGC": (
                [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_ulong, ctypes.c_void_p],
                ctypes.c_void_p,
            ),
            "XSetForeground": (
                [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_ulong],
                ctypes.c_int,
            ),
            "XSetFont": (
                [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_ulong],
                ctypes.c_int,
            ),
            "XLoadQueryFont": (
                [ctypes.c_void_p, ctypes.c_char_p],
                ctypes.POINTER(XFontStruct),
            ),
            "XFreeFont": (
                [ctypes.c_void_p, ctypes.POINTER(XFontStruct)],
                ctypes.c_int,
            ),
            "XTextWidth": (
                [ctypes.POINTER(XFontStruct), ctypes.c_char_p, ctypes.c_int],
                ctypes.c_int,
            ),
            "XMoveResizeWindow": (
                [
                    ctypes.c_void_p,
                    ctypes.c_ulong,
                    ctypes.c_int,
                    ctypes.c_int,
                    ctypes.c_uint,
                    ctypes.c_uint,
                ],
                ctypes.c_int,
            ),
            "XMapRaised": ([ctypes.c_void_p, ctypes.c_ulong], ctypes.c_int),
            "XRaiseWindow": ([ctypes.c_void_p, ctypes.c_ulong], ctypes.c_int),
            "XUnmapWindow": ([ctypes.c_void_p, ctypes.c_ulong], ctypes.c_int),
            "XClearWindow": ([ctypes.c_void_p, ctypes.c_ulong], ctypes.c_int),
            "XFillRectangle": (
                [
                    ctypes.c_void_p,
                    ctypes.c_ulong,
                    ctypes.c_void_p,
                    ctypes.c_int,
                    ctypes.c_int,
                    ctypes.c_uint,
                    ctypes.c_uint,
                ],
                ctypes.c_int,
            ),
            "XDrawRectangle": (
                [
                    ctypes.c_void_p,
                    ctypes.c_ulong,
                    ctypes.c_void_p,
                    ctypes.c_int,
                    ctypes.c_int,
                    ctypes.c_uint,
                    ctypes.c_uint,
                ],
                ctypes.c_int,
            ),
            "XDrawString": (
                [
                    ctypes.c_void_p,
                    ctypes.c_ulong,
                    ctypes.c_void_p,
                    ctypes.c_int,
                    ctypes.c_int,
                    ctypes.c_char_p,
                    ctypes.c_int,
                ],
                ctypes.c_int,
            ),
            "XPending": ([ctypes.c_void_p], ctypes.c_int),
            "XNextEvent": (
                [ctypes.c_void_p, ctypes.POINTER(XEvent)],
                ctypes.c_int,
            ),
            "XFlush": ([ctypes.c_void_p], ctypes.c_int),
            "XSync": ([ctypes.c_void_p, ctypes.c_int], ctypes.c_int),
            "XSetErrorHandler": ([ctypes.c_void_p], ctypes.c_void_p),
            "XDestroyWindow": ([ctypes.c_void_p, ctypes.c_ulong], ctypes.c_int),
            "XFreeGC": ([ctypes.c_void_p, ctypes.c_void_p], ctypes.c_int),
            "XFree": ([ctypes.c_void_p], ctypes.c_int),
            "XCloseDisplay": ([ctypes.c_void_p], ctypes.c_int),
        }
        for name, (argtypes, restype) in signatures.items():
            function = getattr(self._x11, name)
            function.argtypes = argtypes
            function.restype = restype
        fix_signatures = {
            "XFixesQueryExtension": (
                [
                    ctypes.c_void_p,
                    ctypes.POINTER(ctypes.c_int),
                    ctypes.POINTER(ctypes.c_int),
                ],
                ctypes.c_int,
            ),
            "XFixesCreateRegion": (
                [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int],
                ctypes.c_ulong,
            ),
            "XFixesSetWindowShapeRegion": (
                [
                    ctypes.c_void_p,
                    ctypes.c_ulong,
                    ctypes.c_int,
                    ctypes.c_int,
                    ctypes.c_int,
                    ctypes.c_ulong,
                ],
                None,
            ),
            "XFixesDestroyRegion": (
                [ctypes.c_void_p, ctypes.c_ulong],
                None,
            ),
        }
        for name, (argtypes, restype) in fix_signatures.items():
            function = getattr(self._xfixes, name)
            function.argtypes = argtypes
            function.restype = restype
        if self._xft is not None:
            xft_signatures = {
                "XftDrawCreate": (
                    [
                        ctypes.c_void_p,
                        ctypes.c_ulong,
                        ctypes.c_void_p,
                        ctypes.c_ulong,
                    ],
                    ctypes.c_void_p,
                ),
                "XftDrawDestroy": ([ctypes.c_void_p], None),
                "XftFontOpenName": (
                    [ctypes.c_void_p, ctypes.c_int, ctypes.c_char_p],
                    ctypes.c_void_p,
                ),
                "XftFontClose": (
                    [ctypes.c_void_p, ctypes.c_void_p],
                    None,
                ),
                "XftColorAllocName": (
                    [
                        ctypes.c_void_p,
                        ctypes.c_void_p,
                        ctypes.c_ulong,
                        ctypes.c_char_p,
                        ctypes.POINTER(XftColor),
                    ],
                    ctypes.c_int,
                ),
                "XftColorFree": (
                    [
                        ctypes.c_void_p,
                        ctypes.c_void_p,
                        ctypes.c_ulong,
                        ctypes.POINTER(XftColor),
                    ],
                    None,
                ),
                "XftTextExtentsUtf8": (
                    [
                        ctypes.c_void_p,
                        ctypes.c_void_p,
                        ctypes.c_char_p,
                        ctypes.c_int,
                        ctypes.POINTER(XGlyphInfo),
                    ],
                    None,
                ),
                "XftDrawStringUtf8": (
                    [
                        ctypes.c_void_p,
                        ctypes.POINTER(XftColor),
                        ctypes.c_void_p,
                        ctypes.c_int,
                        ctypes.c_int,
                        ctypes.c_char_p,
                        ctypes.c_int,
                    ],
                    None,
                ),
            }
            for name, (argtypes, restype) in xft_signatures.items():
                function = getattr(self._xft, name)
                function.argtypes = argtypes
                function.restype = restype

    def _install_x_error_handler(self) -> None:
        callback = _X_ERROR_HANDLER(self._handle_x_error)
        previous = self._x11.XSetErrorHandler(
            ctypes.cast(callback, ctypes.c_void_p)
        )
        previous_address = int(previous) if previous else None
        self._x_error_handler_callback = callback
        self._previous_x_error_handler_address = previous_address
        self._previous_x_error_handler = (
            _X_ERROR_HANDLER(previous_address) if previous_address is not None else None
        )

    def _restore_x_error_handler(self) -> None:
        if getattr(self, "_x_error_handler_callback", None) is None:
            return
        previous = getattr(self, "_previous_x_error_handler_address", None)
        self._x11.XSetErrorHandler(
            ctypes.c_void_p(previous) if previous is not None else None
        )
        self._x_error_handler_callback = None
        self._previous_x_error_handler_address = None
        self._previous_x_error_handler = None

    def _handle_x_error(
        self,
        display: ctypes.c_void_p,
        event_pointer: ctypes.POINTER(XErrorEvent),
    ) -> int:
        event = event_pointer.contents
        trap = self._window_error_trap
        if (
            trap is not None
            and int(ctypes.cast(display, ctypes.c_void_p).value or 0)
            == int(self._display)
            and int(event.resourceid) == trap.resource_id
            and (int(event.error_code), int(event.request_code))
            in trap.error_signatures
        ):
            self._trapped_window_error = X11ErrorRecord(
                operation=trap.operation,
                resource_id=int(event.resourceid),
                serial=int(event.serial),
                error_code=int(event.error_code),
                request_code=int(event.request_code),
                minor_code=int(event.minor_code),
            )
            return 0
        previous = self._previous_x_error_handler
        if previous is None:
            # Xlib supplies a default handler, so this is defensive only.  An
            # unhandled protocol error must retain fatal semantics.
            print(
                "matrix-calibration-overlay ERROR missing prior Xlib handler "
                f"for code={int(event.error_code)} "
                f"request={int(event.request_code)}.{int(event.minor_code)}",
                file=sys.stderr,
                flush=True,
            )
            os._exit(1)
        return int(previous(display, event_pointer))

    def _window_probe(
        self,
        operation: str,
        window: int,
        request_code: int,
        callback: Callable[[], Any],
        *,
        additional_error_signatures: tuple[tuple[int, int], ...] = (),
    ) -> tuple[Any, bool]:
        """Run one reply-bearing target query with a precise window-error trap.

        Every caller is synchronous in Xlib, so its protocol error is delivered
        before ``callback`` returns and before this scoped trap is removed.
        """

        if self._window_error_trap is not None:
            raise RuntimeError("nested X11 window-error traps are not supported")
        self._trapped_window_error = None
        self._window_error_trap = _RecoverableWindowErrorTrap(
            operation=operation,
            resource_id=window,
            error_signatures=(
                (_BAD_WINDOW, request_code),
                *additional_error_signatures,
            ),
        )
        try:
            result = callback()
        finally:
            self._window_error_trap = None
        record = self._trapped_window_error
        self._trapped_window_error = None
        if record is None:
            return (result, False)
        self._recoverable_window_error_count += 1
        self._last_recoverable_window_error = record
        if record.error_code == _BAD_WINDOW:
            self._bad_window_count += 1
            self._last_bad_window = record
            error_name = "BadWindow"
        elif record.error_code == _BAD_DRAWABLE:
            self._bad_drawable_count += 1
            error_name = "BadDrawable"
        else:  # The trap signatures currently admit only the two errors above.
            error_name = f"XError{record.error_code}"
        # Window churn is usually a single event.  Bound repeated diagnostics
        # while retaining logarithmic evidence if a client thrashes.
        count = self._recoverable_window_error_count
        if count <= 4 or count & (count - 1) == 0:
            print(
                f"matrix-calibration-overlay WARN ignored {error_name} "
                f"operation={record.operation} "
                f"resource=0x{record.resource_id:x} "
                f"request={record.request_code}.{record.minor_code} "
                f"serial={record.serial} count={count}",
                file=sys.stderr,
                flush=True,
            )
        return (result, True)

    @property
    def x11_diagnostics(self) -> dict[str, object]:
        return {
            "recoverable_window_error_count": self._recoverable_window_error_count,
            "bad_window_count": self._bad_window_count,
            "bad_drawable_count": self._bad_drawable_count,
            "last_recoverable_window_error": (
                self._last_recoverable_window_error.mapping()
                if self._last_recoverable_window_error is not None
                else None
            ),
            "last_bad_window": (
                self._last_bad_window.mapping()
                if self._last_bad_window is not None
                else None
            ),
        }

    def _named_colour(self, name: bytes, fallback: int) -> int:
        screen = XColor()
        exact = XColor()
        colormap = self._x11.XDefaultColormap(self._display, self._screen)
        if self._x11.XAllocNamedColor(
            self._display,
            colormap,
            name,
            ctypes.byref(screen),
            ctypes.byref(exact),
        ):
            return int(screen.pixel)
        return fallback

    def _make_click_through(self, window: int) -> None:
        attributes = XSetWindowAttributes()
        attributes.override_redirect = 1
        self._x11.XChangeWindowAttributes(
            self._display,
            window,
            _CW_OVERRIDE_REDIRECT,
            ctypes.byref(attributes),
        )
        empty = self._xfixes.XFixesCreateRegion(self._display, None, 0)
        if not empty:
            raise RuntimeError("cannot create empty XFixes input region")
        try:
            self._xfixes.XFixesSetWindowShapeRegion(
                self._display,
                window,
                _SHAPE_INPUT,
                0,
                0,
                empty,
            )
        finally:
            self._xfixes.XFixesDestroyRegion(self._display, empty)

    def _make_interactive(self, window: int) -> None:
        attributes = XSetWindowAttributes()
        attributes.override_redirect = 1
        attributes.event_mask = (
            _BUTTON_PRESS_MASK
            | _BUTTON_RELEASE_MASK
            | _BUTTON_1_MOTION_MASK
            | _KEY_PRESS_MASK
            | _KEY_RELEASE_MASK
        )
        self._x11.XChangeWindowAttributes(
            self._display,
            window,
            _CW_OVERRIDE_REDIRECT | _CW_EVENT_MASK,
            ctypes.byref(attributes),
        )
        self._x11.XSelectInput(
            self._display,
            window,
            attributes.event_mask,
        )

    def _grab_keyboard(self) -> None:
        if self._keyboard_grabbed:
            return
        if not self._visible:
            raise RuntimeError("refusing to grab the keyboard while the panel is hidden")
        # Mapping is asynchronous.  Synchronize before the grab so Xlib cannot
        # legitimately answer GrabNotViewable for the just-mapped panel.
        self._x11.XSync(self._display, 0)
        result = int(
            self._x11.XGrabKeyboard(
                self._display,
                self._windows["panel"],
                0,
                _GRAB_MODE_ASYNC,
                _GRAB_MODE_ASYNC,
                _CURRENT_TIME,
            )
        )
        if result != _GRAB_SUCCESS:
            raise RuntimeError(f"cannot grab command keyboard input: X11 status {result}")
        self._keyboard_grabbed = True
        self._deferred_ungrab_keycode = None
        self._x11.XFlush(self._display)

    def _ungrab_keyboard(self) -> None:
        if not getattr(self, "_keyboard_grabbed", False):
            return
        try:
            self._x11.XUngrabKeyboard(self._display, _CURRENT_TIME)
            self._x11.XFlush(self._display)
        finally:
            self._keyboard_grabbed = False
            self._deferred_ungrab_keycode = None

    def _release_key_is_still_down(self, keycode: int) -> bool:
        if not 0 <= keycode <= 255:
            return False
        keymap = ctypes.create_string_buffer(32)
        if not self._x11.XQueryKeymap(self._display, keymap):
            # A failed query cannot prove that the physical Escape was released.
            return True
        return bool(keymap.raw[keycode >> 3] & (1 << (keycode & 7)))

    def _set_bounding_shape(
        self,
        window: int,
        rectangles: tuple[tuple[int, int, int, int], ...],
    ) -> None:
        x_rectangles = (XRectangle * len(rectangles))(
            *(XRectangle(x, y, width, height) for x, y, width, height in rectangles)
        )
        region = self._xfixes.XFixesCreateRegion(
            self._display,
            ctypes.cast(x_rectangles, ctypes.c_void_p),
            len(rectangles),
        )
        if not region:
            raise RuntimeError("cannot create shaped cursor XFixes region")
        try:
            self._xfixes.XFixesSetWindowShapeRegion(
                self._display,
                window,
                _SHAPE_BOUNDING,
                0,
                0,
                region,
            )
        finally:
            self._xfixes.XFixesDestroyRegion(self._display, region)

    def _create_windows(self) -> None:
        black = int(self._x11.XBlackPixel(self._display, self._screen))
        white = int(self._x11.XWhitePixel(self._display, self._screen))
        colour_names = {
            "white": b"#f4f7fb",
            "muted": b"#a9b4c3",
            "button": b"#26313b",
            "selected": b"#087f8c",
            "disabled": b"#343a40",
            "apply": b"#23845f",
            "pending": b"#d18b2c",
            "error": b"#d64b5f",
            "outline": b"#71808e",
            "cyan": b"#25c2d1",
            "panel": b"#14191e",
        }
        accent = self._named_colour(colour_names["cyan"], white)
        panel_background = self._named_colour(colour_names["panel"], black)
        colours = {
            "panel": panel_background,
            "horizontal-shadow": black,
            "vertical-shadow": black,
            "horizontal": accent,
            "vertical": accent,
            "cursor-shadow": black,
            "cursor": white,
        }
        self._colours = {
            key: self._named_colour(name, white)
            for key, name in colour_names.items()
            if key != "panel"
        }
        try:
            for name in self._WINDOW_ORDER:
                width, height = (
                    (_CURSOR_WIDTH, _CURSOR_HEIGHT)
                    if name in self._CURSOR_WINDOW_ORDER
                    else (1, 1)
                )
                if name == "shield":
                    attributes = XSetWindowAttributes()
                    attributes.override_redirect = 1
                    attributes.event_mask = _BUTTON_PRESS_MASK | _BUTTON_RELEASE_MASK
                    window = int(
                        self._x11.XCreateWindow(
                            self._display,
                            self._root,
                            -100,
                            -100,
                            width,
                            height,
                            0,
                            0,
                            _INPUT_ONLY,
                            None,
                            _CW_OVERRIDE_REDIRECT | _CW_EVENT_MASK,
                            ctypes.byref(attributes),
                        )
                    )
                else:
                    window = int(self._x11.XCreateSimpleWindow(
                        self._display,
                        self._root,
                        -100,
                        -100,
                        width,
                        height,
                        0,
                        black,
                        colours[name],
                    ))
                if not window:
                    raise RuntimeError(f"cannot create overlay window {name}")
                self._windows[name] = window
                self._x11.XStoreName(
                    self._display,
                    window,
                    f"Matrix Calibration {name}".encode("ascii"),
                )
                if name in {"shield", "panel"}:
                    self._make_interactive(window)
                else:
                    self._make_click_through(window)
                if name == "cursor-shadow":
                    self._set_bounding_shape(window, _CURSOR_SHADOW_RECTANGLES)
                elif name == "cursor":
                    self._set_bounding_shape(window, _CURSOR_FOREGROUND_RECTANGLES)
            panel = self._windows["panel"]
            gc = self._x11.XCreateGC(self._display, panel, 0, None)
            if not gc:
                raise RuntimeError("cannot create overlay panel graphics context")
            self._panel_gc = int(gc)
            self._x11.XSetForeground(self._display, gc, white)
            self._body_font, self._body_font_name = self._load_font(
                _BODY_FONT_CANDIDATES
            )
            self._large_font, self._large_font_name = self._load_font(
                _LARGE_FONT_CANDIDATES
            )
            if self._xft is not None:
                self._initialize_xft(panel, colour_names)
            self._x11.XSync(self._display, 0)
        except Exception:
            self.close()
            raise

    def _load_font(
        self, candidates: tuple[bytes, ...]
    ) -> tuple[ctypes.POINTER(XFontStruct), str]:
        for name in candidates:
            font = self._x11.XLoadQueryFont(self._display, name)
            if font:
                return (font, name.decode("ascii"))
        raise RuntimeError("cannot load an X11 overlay font")

    def _load_xft_font(self, candidates: tuple[bytes, ...]) -> tuple[int, str]:
        assert self._xft is not None
        for name in candidates:
            font = self._xft.XftFontOpenName(self._display, self._screen, name)
            if font:
                return (int(font), name.decode("ascii"))
        raise RuntimeError("cannot load a UTF-8 Xft overlay font")

    def _initialize_xft(
        self,
        panel: int,
        colour_names: dict[str, bytes],
    ) -> None:
        assert self._xft is not None
        if not self._visual:
            raise RuntimeError("Xft requires the default X11 visual")
        draw = self._xft.XftDrawCreate(
            self._display,
            panel,
            self._visual,
            self._colormap,
        )
        if not draw:
            raise RuntimeError("cannot create the UTF-8 Xft drawing context")
        self._xft_draw = int(draw)
        self._xft_body_font, self._xft_body_font_name = self._load_xft_font(
            _xft_font_candidates(self._font_size, bold=False)
        )
        self._xft_large_font, self._xft_large_font_name = self._load_xft_font(
            _xft_font_candidates(
                self._font_size + _LARGE_FONT_SIZE_DELTA,
                bold=True,
            )
        )
        for key, pixel in self._colours.items():
            colour = XftColor()
            name = colour_names.get(key, colour_names["white"])
            if not self._xft.XftColorAllocName(
                self._display,
                self._visual,
                self._colormap,
                name,
                ctypes.byref(colour),
            ):
                raise RuntimeError(f"cannot allocate Xft colour {key}")
            self._xft_colours[pixel] = colour

    def _set_font_size(self, font_size: int) -> bool:
        """Atomically replace both Xft fonts while the overlay stays live."""

        if (
            type(font_size) is not int
            or not _MIN_OVERLAY_FONT_SIZE <= font_size <= _MAX_OVERLAY_FONT_SIZE
        ):
            raise ValueError("overlay font size is outside the supported range")
        if font_size == getattr(self, "_font_size", _DEFAULT_OVERLAY_FONT_SIZE):
            return False
        if self._xft is None or getattr(self, "_xft_draw", None) is None:
            return False
        try:
            body_font, body_name = self._load_xft_font(
                _xft_font_candidates(font_size, bold=False)
            )
        except RuntimeError:
            return False
        try:
            large_font, large_name = self._load_xft_font(
                _xft_font_candidates(
                    font_size + _LARGE_FONT_SIZE_DELTA,
                    bold=True,
                )
            )
        except RuntimeError:
            self._xft.XftFontClose(self._display, ctypes.c_void_p(body_font))
            return False

        previous_body = self._xft_body_font
        previous_large = self._xft_large_font
        self._xft_body_font = body_font
        self._xft_body_font_name = body_name
        self._xft_large_font = large_font
        self._xft_large_font_name = large_name
        self._font_size = font_size
        for previous in (previous_body, previous_large):
            if previous is not None:
                self._xft.XftFontClose(
                    self._display,
                    ctypes.c_void_p(previous),
                )
        return True

    @property
    def font_diagnostics(self) -> dict[str, str | int | bool | None]:
        return {
            "backend": "xft-utf8" if self._xft_draw is not None else "xlib-core",
            "body": self._xft_body_font_name or self._body_font_name,
            "large": self._xft_large_font_name or self._large_font_name,
            "size": self._font_size,
            "adjustable": self._xft_draw is not None,
        }

    def _window_pid(self, window: int) -> int | None:
        if not self._pid_atom:
            return None
        actual_type = ctypes.c_ulong()
        actual_format = ctypes.c_int()
        item_count = ctypes.c_ulong()
        bytes_after = ctypes.c_ulong()
        data = ctypes.POINTER(ctypes.c_ubyte)()
        status, bad_window = self._window_probe(
            "XGetWindowProperty",
            window,
            _X_REQUEST_GET_PROPERTY,
            lambda: self._x11.XGetWindowProperty(
                self._display,
                window,
                self._pid_atom,
                0,
                1,
                0,
                0,
                ctypes.byref(actual_type),
                ctypes.byref(actual_format),
                ctypes.byref(item_count),
                ctypes.byref(bytes_after),
                ctypes.byref(data),
            ),
        )
        try:
            if (
                bad_window
                or status != 0
                or actual_format.value != 32
                or item_count.value < 1
            ):
                return None
            return int(ctypes.cast(data, ctypes.POINTER(ctypes.c_ulong))[0])
        finally:
            if data:
                self._x11.XFree(data)

    def _children(self, window: int) -> list[int]:
        root = ctypes.c_ulong()
        parent = ctypes.c_ulong()
        children = ctypes.POINTER(ctypes.c_ulong)()
        count = ctypes.c_uint()
        ok, bad_window = self._window_probe(
            "XQueryTree",
            window,
            _X_REQUEST_QUERY_TREE,
            lambda: self._x11.XQueryTree(
                self._display,
                window,
                ctypes.byref(root),
                ctypes.byref(parent),
                ctypes.byref(children),
                ctypes.byref(count),
            ),
        )
        try:
            if bad_window or not ok:
                return []
            return [int(children[index]) for index in range(count.value)]
        finally:
            if children:
                self._x11.XFree(children)

    def _geometry(self, window: int) -> WindowGeometry | None:
        attributes = XWindowAttributes()
        attributes_ok, stale_window = self._window_probe(
            "XGetWindowAttributes",
            window,
            _X_REQUEST_GET_WINDOW_ATTRIBUTES,
            lambda: self._x11.XGetWindowAttributes(
                self._display,
                window,
                ctypes.byref(attributes),
            ),
            # libX11 implements XGetWindowAttributes with consecutive
            # GetWindowAttributes and GetGeometry requests.  If destruction
            # lands between them, the second request returns BadDrawable.
            additional_error_signatures=((
                _BAD_DRAWABLE,
                _X_REQUEST_GET_GEOMETRY,
            ),),
        )
        if stale_window or not attributes_ok:
            return None
        if (
            attributes.map_state != _IS_VIEWABLE
            or attributes.width <= 1
            or attributes.height <= 1
        ):
            return None
        root_x = ctypes.c_int()
        root_y = ctypes.c_int()
        child = ctypes.c_ulong()
        translated, bad_window = self._window_probe(
            "XTranslateCoordinates",
            window,
            _X_REQUEST_TRANSLATE_COORDINATES,
            lambda: self._x11.XTranslateCoordinates(
                self._display,
                window,
                self._root,
                0,
                0,
                ctypes.byref(root_x),
                ctypes.byref(root_y),
                ctypes.byref(child),
            ),
        )
        if bad_window or not translated:
            return None
        return WindowGeometry(
            window=window,
            x=root_x.value,
            y=root_y.value,
            width=attributes.width,
            height=attributes.height,
        )

    def find_target(self) -> WindowGeometry | None:
        if self._target_window is not None:
            if self._window_pid(self._target_window) == self.expected_ue_pid:
                cached = self._geometry(self._target_window)
                if cached is not None:
                    return cached
            self._target_window = None
        candidates: list[WindowGeometry] = []
        pending = self._children(self._root)
        visited = 0
        while pending and visited < 20_000:
            window = pending.pop()
            visited += 1
            if self._window_pid(window) == self.expected_ue_pid:
                geometry = self._geometry(window)
                if geometry is not None:
                    candidates.append(geometry)
            pending.extend(self._children(window))
        if not candidates:
            return None
        selected = max(candidates, key=lambda item: item.width * item.height)
        self._target_window = selected.window
        return selected

    def pointer_position(self) -> tuple[int, int] | None:
        root_return = ctypes.c_ulong()
        child_return = ctypes.c_ulong()
        root_x = ctypes.c_int()
        root_y = ctypes.c_int()
        window_x = ctypes.c_int()
        window_y = ctypes.c_int()
        mask = ctypes.c_uint()
        if not self._x11.XQueryPointer(
            self._display,
            self._root,
            ctypes.byref(root_return),
            ctypes.byref(child_return),
            ctypes.byref(root_x),
            ctypes.byref(root_y),
            ctypes.byref(window_x),
            ctypes.byref(window_y),
            ctypes.byref(mask),
        ):
            return None
        return (root_x.value, root_y.value)

    def _panel_rectangle(
        self,
        layout: dict[str, tuple[int, int, int, int]],
        name: str,
    ) -> tuple[int, int, int, int]:
        panel_x, panel_y, _panel_width, _panel_height = layout["panel"]
        x, y, width, height = layout[name]
        return (x - panel_x, y - panel_y, width, height)

    def _draw_text(
        self,
        message: str,
        *,
        x: int,
        y: int,
        colour: int,
        large: bool = False,
        centred_in: tuple[int, int, int, int] | None = None,
    ) -> None:
        xft_draw = getattr(self, "_xft_draw", None)
        xft_font = (
            getattr(self, "_xft_large_font", None)
            if large
            else getattr(self, "_xft_body_font", None)
        )
        xft_colour = getattr(self, "_xft_colours", {}).get(colour)
        if (
            xft_draw is not None
            and xft_font is not None
            and xft_colour is not None
            and self._xft is not None
        ):
            encoded_utf8 = message[:160].encode("utf-8")
            if centred_in is not None:
                extents = XGlyphInfo()
                self._xft.XftTextExtentsUtf8(
                    self._display,
                    ctypes.c_void_p(xft_font),
                    encoded_utf8,
                    len(encoded_utf8),
                    ctypes.byref(extents),
                )
                left, top, width, height = centred_in
                x = left + max(4, (width - int(extents.xOff)) // 2)
                y = top + height // 2 + (8 if large else 6)
            self._xft.XftDrawStringUtf8(
                ctypes.c_void_p(xft_draw),
                ctypes.byref(xft_colour),
                ctypes.c_void_p(xft_font),
                x,
                y,
                encoded_utf8,
                len(encoded_utf8),
            )
            return
        gc = ctypes.c_void_p(self._panel_gc)
        font = self._large_font if large else self._body_font
        assert font is not None
        encoded = message.encode("ascii", errors="replace")[:160]
        self._x11.XSetForeground(self._display, gc, colour)
        self._x11.XSetFont(self._display, gc, font.contents.fid)
        if centred_in is not None:
            left, top, width, height = centred_in
            text_width = int(self._x11.XTextWidth(font, encoded, len(encoded)))
            x = left + max(4, (width - text_width) // 2)
            y = top + height // 2 + (9 if large else 6)
        self._x11.XDrawString(
            self._display,
            self._windows["panel"],
            gc,
            x,
            y,
            encoded,
            len(encoded),
        )

    def _draw_button(
        self,
        layout: dict[str, tuple[int, int, int, int]],
        name: str,
        label: str,
        *,
        fill: int,
        disabled: bool = False,
    ) -> None:
        x, y, width, height = self._panel_rectangle(layout, name)
        panel = self._windows["panel"]
        gc = ctypes.c_void_p(self._panel_gc)
        self._x11.XSetForeground(self._display, gc, fill)
        self._x11.XFillRectangle(
            self._display, panel, gc, x, y, width, height
        )
        self._x11.XSetForeground(
            self._display,
            gc,
            self._colours["disabled" if disabled else "outline"],
        )
        self._x11.XDrawRectangle(
            self._display,
            panel,
            gc,
            x,
            y,
            max(1, width - 1),
            max(1, height - 1),
        )
        self._draw_text(
            label,
            x=0,
            y=0,
            colour=self._colours["muted" if disabled else "white"],
            large=height >= 38,
            centred_in=(x, y, width, height),
        )

    @staticmethod
    def _clip_console_line(value: str, width: int) -> str:
        maximum = max(1, width // 10)
        if len(value) <= maximum:
            return value
        if maximum <= 3:
            return value[:maximum]
        return value[: maximum - 3] + "..."

    def _draw_command_console(
        self,
        layout: dict[str, tuple[int, int, int, int]],
        status: CommandConsoleStatus,
    ) -> None:
        editor = self._command_editor
        history_x, history_y, history_width, history_height = self._panel_rectangle(
            layout, "command_history"
        )
        if history_height >= 18:
            self._draw_text(
                "最近命令",
                x=history_x,
                y=history_y + 14,
                colour=self._colours["muted"],
            )
            available_lines = max(0, history_height // 20 - 1)
            recent = editor.history[-available_lines:] if available_lines else []
            for index, command in enumerate(recent):
                self._draw_text(
                    self._clip_console_line(f"> {command}", history_width),
                    x=history_x,
                    y=history_y + 34 + index * 20,
                    colour=self._colours["white"],
                )

        result_x, result_y, result_width, result_height = self._panel_rectangle(
            layout, "command_result"
        )
        if editor.pending or status.in_flight or status.status in {"pending", "restarting"}:
            result_text = "[处理中] 等待 Matrix 确认..."
            result_colour = self._colours["pending"]
        elif status.status == "success":
            result_text = f"[完成 {status.code or 'OK'}] 命令已执行"
            if status.warning:
                result_text += " | 有兼容性提示"
            result_colour = self._colours["apply"]
        elif status.status == "error":
            result_text = f"[失败 {status.code or 'ERROR'}] 命令未执行"
            result_colour = self._colours["error"]
        elif not status.available:
            result_text = "[不可用] 本次运行未启用命令通道"
            result_colour = self._colours["disabled"]
        else:
            result_text = "命令台就绪"
            result_colour = self._colours["muted"]
        if result_height >= 14:
            self._draw_text(
                self._clip_console_line(result_text, result_width),
                x=result_x,
                y=result_y + min(result_height - 2, 16),
                colour=result_colour,
            )

        input_x, input_y, input_width, input_height = self._panel_rectangle(
            layout, "command_input"
        )
        panel = self._windows["panel"]
        gc = ctypes.c_void_p(self._panel_gc)
        if editor.pending:
            fill = self._colours["disabled"]
        elif editor.editing:
            fill = self._colours["selected"]
        else:
            fill = self._colours["button"]
        self._x11.XSetForeground(self._display, gc, fill)
        self._x11.XFillRectangle(
            self._display,
            panel,
            gc,
            input_x,
            input_y,
            input_width,
            input_height,
        )
        self._x11.XSetForeground(
            self._display,
            gc,
            self._colours["pending" if editor.pending else "outline"],
        )
        self._x11.XDrawRectangle(
            self._display,
            panel,
            gc,
            input_x,
            input_y,
            max(1, input_width - 1),
            max(1, input_height - 1),
        )
        maximum_characters = max(1, (input_width - 28) // 10)
        input_text = editor.display_line(maximum_characters)
        self._draw_text(
            self._clip_console_line(f"> {input_text}", input_width - 18),
            x=input_x + 10,
            y=input_y + input_height // 2 + 6,
            colour=self._colours["muted" if editor.pending else "white"],
        )

    def _draw_settings_page_legacy(
        self,
        layout: dict[str, tuple[int, int, int, int]],
        model: SettingsPanelModel,
        command_status: CommandConsoleStatus | None = None,
    ) -> None:
        _panel_x, _panel_y, panel_width, panel_height = layout["panel"]
        compact = panel_height < 600
        panel = self._windows["panel"]
        self._x11.XClearWindow(self._display, panel)
        title_x, title_y, _title_width, title_height = self._panel_rectangle(
            layout, "title"
        )
        self._draw_text(
            "MATRIX SETTINGS + COMMANDS",
            x=title_x,
            y=title_y + title_height - (6 if compact else 4),
            colour=self._colours["white"],
            large=not compact,
        )
        if not compact:
            self._draw_text(
                f"Currently applied: {model.current_profile} "
                f"{model.current_scale:.2f}x",
                x=40,
                y=max(64, panel_height // 9),
                colour=self._colours["muted"],
            )
        local_selected = model.next_profile == "Local"
        controls_disabled = model.restart_requested or model.status == "restarting"
        self._draw_button(
            layout,
            "profile_local",
            "LOCAL",
            fill=self._colours["selected" if local_selected else "button"],
            disabled=controls_disabled,
        )
        self._draw_button(
            layout,
            "profile_remote",
            "REMOTE",
            fill=self._colours["selected" if not local_selected else "button"],
            disabled=controls_disabled,
        )
        speed_down_disabled = not model.action_enabled("speed_down")
        speed_up_disabled = not model.action_enabled("speed_up")
        self._draw_button(
            layout,
            "speed_down",
            "-",
            fill=self._colours["disabled" if speed_down_disabled else "button"],
            disabled=speed_down_disabled,
        )
        self._draw_button(
            layout,
            "speed_up",
            "+",
            fill=self._colours["disabled" if speed_up_disabled else "button"],
            disabled=speed_up_disabled,
        )
        speed_value = self._panel_rectangle(layout, "speed_value")
        self._draw_text(
            "REMOTE SPEED",
            x=0,
            y=0,
            colour=self._colours["muted"],
            centred_in=(
                speed_value[0],
                speed_value[1] - 10,
                speed_value[2],
                speed_value[3],
            ),
        )
        self._draw_text(
            f"{model.next_scale:.2f}x",
            x=0,
            y=0,
            colour=self._colours["white"],
            large=True,
            centred_in=(
                speed_value[0],
                speed_value[1] + 12,
                speed_value[2],
                speed_value[3],
            ),
        )
        if model.status == "restarting":
            status_text = "Reloading the complete Matrix runtime - keep controls released"
            status_colour = self._colours["pending"]
        elif model.status == "waiting_neutral":
            status_text = "Preparing a safe neutral frame..."
            status_colour = self._colours["pending"]
        elif model.error is not None:
            status_text = f"Could not apply: {model.error}"
            status_colour = self._colours["error"]
        elif model.pending_restart:
            status_text = "Apply/Return will reload Matrix with the saved changes"
            status_colour = self._colours["pending"]
        else:
            status_text = "No reload needed"
            status_colour = self._colours["muted"]
        if not compact:
            profile_y = self._panel_rectangle(layout, "profile_local")[1]
            self._draw_text(
                status_text,
                x=40,
                y=max(96, profile_y - 42),
                colour=status_colour,
            )
            self._draw_text(
                "Fine: 0.01-0.10 by 0.01 | Coarse: 0.20-1.00 by 0.10",
                x=40,
                y=max(116, profile_y - 20),
                colour=self._colours["muted"],
            )
        self._draw_command_console(
            layout,
            command_status
            or getattr(self, "_last_command_status", command_console_status({})),
        )
        apply_disabled = not model.action_enabled("apply_return")
        apply_fill = self._colours[
            "disabled"
            if apply_disabled
            else ("pending" if model.pending_restart else "apply")
        ]
        self._draw_button(
            layout,
            "apply_return",
            model.apply_label,
            fill=apply_fill,
            disabled=apply_disabled,
        )
        if not compact:
            footer = "Click command box to type | Enter: run | Esc: leave editor, then back"
            self._draw_text(
                footer,
                x=40,
                y=max(18, panel_height - 10),
                colour=self._colours["muted"],
            )

    def _fill_panel_band(
        self,
        layout: dict[str, tuple[int, int, int, int]],
        name: str,
        *,
        fill: int,
        outline: int,
    ) -> tuple[int, int, int, int]:
        rectangle = self._panel_rectangle(layout, name)
        x, y, width, height = rectangle
        panel = self._windows["panel"]
        gc = ctypes.c_void_p(self._panel_gc)
        self._x11.XSetForeground(self._display, gc, fill)
        self._x11.XFillRectangle(
            self._display, panel, gc, x, y, width, height
        )
        self._x11.XSetForeground(self._display, gc, outline)
        self._x11.XDrawRectangle(
            self._display,
            panel,
            gc,
            x,
            y,
            max(1, width - 1),
            max(1, height - 1),
        )
        return rectangle

    def _draw_tabs(
        self,
        layout: dict[str, tuple[int, int, int, int]],
        page: str,
    ) -> None:
        for name, label, target_page in (
            ("tab_loadout", "策略装配", "loadout"),
            ("tab_settings", "控制设置", "settings"),
            ("tab_console", "命令台", "console"),
        ):
            self._draw_button(
                layout,
                name,
                label,
                fill=self._colours[
                    "selected" if page == target_page else "button"
                ],
            )

    def _draw_font_size_slider(
        self,
        layout: dict[str, tuple[int, int, int, int]],
    ) -> None:
        rectangle = self._panel_rectangle(layout, "font_size_slider")
        x, y, width, height = rectangle
        panel = self._windows["panel"]
        gc = ctypes.c_void_p(self._panel_gc)
        adjustable = bool(
            getattr(self, "_xft", None) is not None
            and getattr(self, "_xft_draw", None) is not None
        )
        self._x11.XSetForeground(self._display, gc, self._colours["button"])
        self._x11.XFillRectangle(
            self._display,
            panel,
            gc,
            x,
            y,
            width,
            height,
        )
        self._x11.XSetForeground(self._display, gc, self._colours["outline"])
        self._x11.XDrawRectangle(
            self._display,
            panel,
            gc,
            x,
            y,
            max(1, width - 1),
            max(1, height - 1),
        )

        panel_x, panel_y, _panel_width, _panel_height = layout["panel"]
        track_root = font_slider_track(layout["font_size_slider"])
        track_x = track_root[0] - panel_x
        track_y = track_root[1] - panel_y
        track_width = track_root[2]
        track_height = track_root[3]
        self._x11.XSetForeground(
            self._display,
            gc,
            self._colours["muted" if adjustable else "disabled"],
        )
        self._x11.XFillRectangle(
            self._display,
            panel,
            gc,
            track_x,
            track_y,
            track_width,
            track_height,
        )
        font_size = getattr(self, "_font_size", _DEFAULT_OVERLAY_FONT_SIZE)
        span = _MAX_OVERLAY_FONT_SIZE - _MIN_OVERLAY_FONT_SIZE
        fraction = (font_size - _MIN_OVERLAY_FONT_SIZE) / span
        knob_x = track_x + int(round(max(0, track_width - 1) * fraction))
        if adjustable:
            self._x11.XSetForeground(self._display, gc, self._colours["cyan"])
            self._x11.XFillRectangle(
                self._display,
                panel,
                gc,
                track_x,
                track_y,
                max(1, knob_x - track_x + 1),
                track_height,
            )
        knob_width = 9
        knob_height = max(10, height - 10)
        self._x11.XSetForeground(
            self._display,
            gc,
            self._colours["white" if adjustable else "disabled"],
        )
        self._x11.XFillRectangle(
            self._display,
            panel,
            gc,
            knob_x - knob_width // 2,
            y + (height - knob_height) // 2,
            knob_width,
            knob_height,
        )
        self._draw_text(
            f"字号 {font_size}px" if adjustable else "字号固定",
            x=x + 8,
            y=y + height // 2 + 6,
            colour=self._colours["white" if adjustable else "muted"],
        )

    @staticmethod
    def _policy_display_name(policy_id: str) -> str:
        return {
            "sonic": "SONIC",
            "bfm-sonic-teacher50k": "BFM Teacher50k",
            "kungfu": "KungFu",
            "host": "HoST",
            "amp": "AMP",
        }.get(policy_id, policy_id.upper())

    def _draw_loadout_page(
        self,
        layout: dict[str, tuple[int, int, int, int]],
        model: StrategyLoadoutModel,
    ) -> None:
        locomotion = self._fill_panel_band(
            layout,
            "locomotion_slot",
            fill=self._colours["button"],
            outline=self._colours[
                "cyan" if model.active_slot == "locomotion" else "outline"
            ],
        )
        compact = layout["panel"][3] < 500
        if not compact:
            self._draw_text(
                "移动策略槽",
                x=locomotion[0] + 18,
                y=locomotion[1] + 26,
                colour=self._colours["muted"],
            )
            self._draw_text(
                "当前控制" if model.active_slot == "locomotion" else "已装配",
                x=max(18, locomotion[0] + locomotion[2] - 120),
                y=locomotion[1] + 30,
                colour=self._colours[
                    "cyan" if model.active_slot == "locomotion" else "muted"
                ],
            )
        locomotion_candidates = model.locomotion_candidates[:3]
        for index, candidate in enumerate(locomotion_candidates):
            selected = candidate.policy_id == model.locomotion_policy_id
            pending = candidate.policy_id == model.pending_policy_id
            enabled = model.policy_enabled(candidate.policy_id, slot="locomotion")
            fill_name = (
                "pending"
                if pending
                else ("selected" if selected else ("button" if enabled else "disabled"))
            )
            label = candidate.display_name or self._policy_display_name(
                candidate.policy_id
            )
            if pending:
                label = f"{label} · 切换中"
            elif not candidate.available or not candidate.resident:
                label = f"{label} · 未就绪"
            self._draw_button(
                layout,
                f"locomotion_policy_{index}",
                label,
                fill=self._colours[fill_name],
                disabled=not enabled and not selected and not pending,
            )
        if not locomotion_candidates:
            self._draw_text(
                "移动策略尚未就绪",
                x=0,
                y=0,
                colour=self._colours["pending"],
                centred_in=locomotion,
            )

        recovery = self._panel_rectangle(layout, "recovery_slot")
        if not compact and recovery[3] >= 70:
            self._draw_text(
                "起身策略槽",
                x=recovery[0],
                y=recovery[1] + 20,
                colour=self._colours[
                    "cyan" if model.active_slot == "recovery" else "muted"
                ],
            )
        candidates = model.recovery_candidates[:3]
        for index, candidate in enumerate(candidates):
            selected = candidate.policy_id == model.recovery_policy_id
            pending = candidate.policy_id == model.pending_policy_id
            enabled = model.policy_enabled(candidate.policy_id)
            fill_name = (
                "pending"
                if pending
                else ("selected" if selected else ("button" if enabled else "disabled"))
            )
            label = self._policy_display_name(candidate.policy_id)
            if pending:
                label = f"{label} · 切换中"
            self._draw_button(
                layout,
                f"recovery_policy_{index}",
                label,
                fill=self._colours[fill_name],
                disabled=not enabled and not selected and not pending,
            )
        if not candidates:
            self._draw_text(
                "起身策略尚未就绪",
                x=0,
                y=0,
                colour=self._colours["pending"],
                centred_in=recovery,
            )

    def _draw_control_settings_page(
        self,
        layout: dict[str, tuple[int, int, int, int]],
        model: SettingsPanelModel,
        motion_model: MotionSettingsPanelModel,
        command_status: CommandConsoleStatus,
    ) -> None:
        local_selected = model.next_profile == "Local"
        controls_disabled = model.restart_requested or model.status == "restarting"
        self._draw_button(
            layout,
            "profile_local",
            "本机控制",
            fill=self._colours["selected" if local_selected else "button"],
            disabled=controls_disabled,
        )
        self._draw_button(
            layout,
            "profile_remote",
            "远程控制",
            fill=self._colours["selected" if not local_selected else "button"],
            disabled=controls_disabled,
        )
        down_disabled = not model.action_enabled("speed_down")
        up_disabled = not model.action_enabled("speed_up")
        self._draw_button(
            layout,
            "speed_down",
            "-",
            fill=self._colours["disabled" if down_disabled else "button"],
            disabled=down_disabled,
        )
        self._draw_button(
            layout,
            "speed_up",
            "+",
            fill=self._colours["disabled" if up_disabled else "button"],
            disabled=up_disabled,
        )
        speed_value = self._panel_rectangle(layout, "speed_value")
        self._draw_text(
            "远程鼠标速度",
            x=0,
            y=0,
            colour=self._colours["muted"],
            centred_in=(
                speed_value[0],
                speed_value[1] - 10,
                speed_value[2],
                speed_value[3],
            ),
        )
        self._draw_text(
            f"{model.next_scale:.2f}x",
            x=0,
            y=0,
            colour=self._colours["white"],
            large=True,
            centred_in=(
                speed_value[0],
                speed_value[1] + 10,
                speed_value[2],
                speed_value[3],
            ),
        )
        command_blocked = bool(
            command_status.in_flight
            or command_status.restart_required
            or command_status.outcome_unknown
            or command_status.status in {"pending", "restarting"}
            or self._command_editor.editing
            or self._command_editor.pending
        )
        compact_motion_labels = bool(
            layout["panel"][2] < 800 or layout["panel"][3] < 600
        )
        for gear, field in _MOTION_CONTROL_SPECS:
            stem = f"motion_{gear}_{field}"
            for suffix in ("down", "up"):
                action = f"{stem}_{suffix}"
                disabled = bool(
                    controls_disabled
                    or command_blocked
                    or not motion_model.action_enabled(action)
                )
                self._draw_button(
                    layout,
                    action,
                    "-" if suffix == "down" else "+",
                    fill=self._colours["disabled" if disabled else "button"],
                    disabled=disabled,
                )
            self._draw_text(
                motion_value_label(
                    motion_model,
                    gear,
                    field,
                    compact=compact_motion_labels,
                ),
                x=0,
                y=0,
                colour=self._colours[
                    "white" if motion_model.available else "muted"
                ],
                centred_in=self._panel_rectangle(layout, f"{stem}_value"),
            )
        if layout["panel"][3] >= 500:
            status = (
                "正在重载 Matrix"
                if model.status == "restarting"
                else (
                    "设置已保存，返回后生效"
                    if model.pending_restart
                    else "当前设置已生效"
                )
            )
            self._draw_text(
                status,
                x=self._panel_rectangle(layout, "profile_local")[0],
                y=max(92, self._panel_rectangle(layout, "profile_local")[1] - 18),
                colour=self._colours[
                    "pending" if model.pending_restart else "muted"
                ],
            )
            self._draw_text(
                "精细 0.01-0.10 / 粗调 0.20-1.00",
                x=self._panel_rectangle(layout, "profile_local")[0],
                y=max(112, self._panel_rectangle(layout, "profile_local")[1] - 40),
                colour=self._colours["muted"],
            )

    @staticmethod
    def _apply_label_chinese(model: SettingsPanelModel) -> str:
        if model.restart_requested or model.status == "restarting":
            return "正在重载 MATRIX..."
        if model.pending_restart and not model.restart_available:
            return "暂时无法应用"
        if model.pending_restart:
            return "返回游戏并应用"
        return "返回游戏"

    def _draw_panel(
        self,
        layout: dict[str, tuple[int, int, int, int]],
        model: SettingsPanelModel,
        command_status: CommandConsoleStatus | None = None,
        strategy_model: StrategyLoadoutModel | None = None,
        motion_model: MotionSettingsPanelModel | None = None,
    ) -> None:
        _panel_x, _panel_y, panel_width, panel_height = layout["panel"]
        panel = self._windows["panel"]
        self._x11.XClearWindow(self._display, panel)
        page = getattr(self, "_active_page", "settings")
        title_x, title_y, _title_width, title_height = self._panel_rectangle(
            layout, "title"
        )
        self._draw_text(
            "MATRIX 战术终端",
            x=title_x,
            y=title_y + title_height - 4,
            colour=self._colours["white"],
            large=panel_height >= 500,
        )
        if page == "settings":
            self._draw_font_size_slider(layout)
        self._draw_tabs(layout, page)
        if page == "loadout":
            self._draw_loadout_page(
                layout,
                strategy_model or strategy_loadout_model({}),
            )
        elif page == "settings":
            self._draw_control_settings_page(
                layout,
                model,
                motion_model or motion_settings_panel_model({}),
                command_status
                or getattr(self, "_last_command_status", command_console_status({})),
            )
        else:
            self._draw_command_console(
                layout,
                command_status
                or getattr(self, "_last_command_status", command_console_status({})),
            )
        apply_disabled = not model.action_enabled("apply_return")
        self._draw_button(
            layout,
            "apply_return",
            self._apply_label_chinese(model),
            fill=self._colours[
                "disabled"
                if apply_disabled
                else ("pending" if model.pending_restart else "apply")
            ],
            disabled=apply_disabled,
        )

    def _begin_command_editing(self, publisher: PointerActionPublisher) -> bool:
        status = self._last_command_status
        if (
            not self._visible
            or not status.available
            or self._command_editor.pending
            or status.in_flight
            or status.restart_required
            or status.outcome_unknown
            or status.status in {"pending", "restarting", "unavailable"}
            or self._deferred_ungrab_keycode is not None
            or self._keyboard_grabbed
            or not self._command_editor.begin()
        ):
            return False
        try:
            self._grab_keyboard()
            publisher.publish_command_edit(True)
        except Exception:
            self._command_editor.end(force=True)
            self._ungrab_keyboard()
            raise
        return True

    def _force_end_command_editing(
        self,
        publisher: PointerActionPublisher | None,
    ) -> bool:
        was_editing = self._command_editor.editing
        if not was_editing and not self._keyboard_grabbed:
            return False
        self._command_editor.end(force=True)
        try:
            if was_editing and publisher is not None:
                publisher.publish_command_edit(False)
        finally:
            self._ungrab_keyboard()
        return True

    def _lookup_key(self, event: XKeyEvent) -> tuple[int, str]:
        buffer = ctypes.create_string_buffer(32)
        keysym = ctypes.c_ulong()
        count = int(
            self._x11.XLookupString(
                ctypes.byref(event),
                ctypes.cast(buffer, ctypes.c_void_p),
                len(buffer) - 1,
                ctypes.byref(keysym),
                None,
            )
        )
        if count <= 0:
            return (int(keysym.value), "")
        raw = bytes(buffer.raw[: min(count, len(buffer) - 1)])
        try:
            printable = raw.decode("ascii")
        except UnicodeDecodeError:
            printable = ""
        return (int(keysym.value), printable)

    def _handle_key_press(
        self,
        event: XKeyEvent,
        publisher: PointerActionPublisher,
    ) -> int:
        if not self._visible or not self._keyboard_grabbed:
            return 0
        keysym, printable = self._lookup_key(event)
        outcome = self._command_editor.handle_key(
            keysym=keysym,
            printable=printable,
            status=self._last_command_status,
        )
        if outcome.action == "submit":
            assert outcome.command is not None
            publisher.publish_command_submit(outcome.command)
            return 1
        if outcome.action == "end":
            publisher.publish_command_edit(False)
            # Keep the active grab through the physical Escape release.  This
            # prevents its release or auto-repeat presses from leaking into the
            # still-focused UE window after command editing ends.
            keycode = int(event.keycode)
            if not 8 <= keycode <= 255:
                # A real X11 keyboard event cannot carry an out-of-range
                # keycode.  Escalate to the main fail-closed exception path;
                # close() will immediately release the grab there.
                raise RuntimeError(f"invalid Escape keycode from X11: {keycode}")
            self._deferred_ungrab_keycode = keycode
            return 1
        return 0

    def _handle_key_release(self, event: XKeyEvent) -> None:
        deferred = self._deferred_ungrab_keycode
        if (
            deferred is not None
            and int(event.keycode) == deferred
            and not self._release_key_is_still_down(deferred)
        ):
            self._ungrab_keyboard()

    def _set_font_size_from_root_x(self, root_x: int) -> bool:
        layout = self._last_layout
        if (
            layout is None
            or getattr(self, "_active_page", "loadout") != "settings"
        ):
            return False
        return self._set_font_size(
            font_size_from_slider(layout["font_size_slider"], root_x)
        )

    def drain_pointer_actions(self, publisher: PointerActionPublisher) -> int:
        """Drain bounded keyboard intents and completed left-button clicks."""

        emitted = 0
        while self._x11.XPending(self._display) > 0:
            event = XEvent()
            self._x11.XNextEvent(self._display, ctypes.byref(event))
            event_type = int(event.type)
            if event_type == _KEY_PRESS:
                emitted += self._handle_key_press(event.xkey, publisher)
                continue
            if event_type == _KEY_RELEASE:
                self._handle_key_release(event.xkey)
                continue
            if event_type == _MOTION_NOTIFY:
                if self._font_slider_dragging and self._visible:
                    self._set_font_size_from_root_x(event.xmotion.x_root)
                continue
            if event_type not in {_BUTTON_PRESS, _BUTTON_RELEASE}:
                continue
            button = event.xbutton
            if button.button != 1:
                continue
            layout = self._last_layout
            action = (
                panel_action_at(
                    layout,
                    button.x_root,
                    button.y_root,
                    page=getattr(self, "_active_page", "loadout"),
                )
                if layout is not None
                else None
            )
            if event_type == _BUTTON_PRESS:
                self._pressed_action = action
                self._pressed_window = int(button.window)
                if action == "font_size_slider":
                    self._font_slider_dragging = bool(
                        self._xft is not None
                        and getattr(self, "_xft_draw", None) is not None
                    )
                    if self._font_slider_dragging:
                        self._set_font_size_from_root_x(button.x_root)
            elif event_type == _BUTTON_RELEASE:
                pressed = self._pressed_action
                pressed_window = self._pressed_window
                self._pressed_action = None
                self._pressed_window = None
                if pressed == "font_size_slider":
                    if self._font_slider_dragging and self._visible:
                        self._set_font_size_from_root_x(button.x_root)
                    self._font_slider_dragging = False
                    continue
                if (
                    pressed is None
                    or action != pressed
                    or pressed_window != int(button.window)
                    or not self._visible
                ):
                    continue
                if action in _PANEL_TABS:
                    next_page = action.removeprefix("tab_")
                    if next_page != getattr(self, "_active_page", "loadout"):
                        if self._force_end_command_editing(publisher):
                            emitted += 1
                        self._active_page = next_page
                        self._last_page = None
                    continue
                if action == "command_input":
                    if self._begin_command_editing(publisher):
                        emitted += 1
                elif action.startswith("recovery_policy_"):
                    strategy = getattr(self, "_last_strategy_model", None)
                    try:
                        policy_index = int(action.rsplit("_", 1)[1])
                    except (IndexError, ValueError):
                        continue
                    if (
                        strategy is not None
                        and policy_index < len(strategy.recovery_candidates)
                    ):
                        candidate = strategy.recovery_candidates[policy_index]
                        if strategy.policy_enabled(candidate.policy_id):
                            publisher.publish_strategy_select(
                                "recovery",
                                candidate.policy_id,
                            )
                            emitted += 1
                elif action.startswith("locomotion_policy_"):
                    strategy = getattr(self, "_last_strategy_model", None)
                    try:
                        policy_index = int(action.rsplit("_", 1)[1])
                    except (IndexError, ValueError):
                        continue
                    if (
                        strategy is not None
                        and policy_index < len(strategy.locomotion_candidates)
                    ):
                        candidate = strategy.locomotion_candidates[policy_index]
                        if strategy.policy_enabled(
                            candidate.policy_id,
                            slot="locomotion",
                        ):
                            publisher.publish_strategy_select(
                                "locomotion",
                                candidate.policy_id,
                            )
                            emitted += 1
                elif action in _MOTION_STEP_ACTIONS:
                    motion_model = getattr(self, "_last_motion_model", None)
                    panel_model = self._last_panel_model
                    command = (
                        motion_step_command(motion_model, action)
                        if motion_model is not None
                        else None
                    )
                    if (
                        command is not None
                        and panel_model is not None
                        and not panel_model.restart_requested
                        and panel_model.status != "restarting"
                        and not self._command_editor.editing
                        and not self._command_editor.pending
                        and self._last_command_status.available
                        and not self._last_command_status.in_flight
                        and not self._last_command_status.restart_required
                        and not self._last_command_status.outcome_unknown
                        and self._last_command_status.status
                        not in {"pending", "restarting"}
                    ):
                        publisher.publish_command_submit(command)
                        emitted += 1
                elif (
                    self._last_panel_model is not None
                    and self._last_panel_model.action_enabled(action)
                    and not self._command_editor.editing
                    and not self._command_editor.pending
                    and not self._last_command_status.in_flight
                    and not self._last_command_status.restart_required
                    and not self._last_command_status.outcome_unknown
                    and self._last_command_status.status
                    not in {"pending", "restarting"}
                ):
                    publisher.publish(action)
                    emitted += 1
        return emitted

    def show(
        self,
        geometry: WindowGeometry,
        pointer: tuple[int, int],
        state: dict[str, object],
        *,
        now_s: float | None = None,
    ) -> None:
        now = time.monotonic() if now_s is None else now_s
        first_show = not self._visible
        geometry_changed = geometry != self._last_geometry
        model = settings_panel_model(state)
        motion_model = motion_settings_panel_model(state)
        strategy_model = strategy_loadout_model(state)
        command_status = command_console_status(state)
        self._command_editor.reconcile(command_status)
        model_changed = bool(
            model != self._last_panel_model
            or motion_model != getattr(self, "_last_motion_model", None)
            or strategy_model != getattr(self, "_last_strategy_model", None)
            or getattr(self, "_font_size", _DEFAULT_OVERLAY_FONT_SIZE)
            != getattr(self, "_last_rendered_font_size", None)
            or getattr(self, "_active_page", "loadout")
            != getattr(self, "_last_page", None)
            or command_status != self._last_command_status
            or self._command_editor.revision != self._last_command_revision
        )
        if geometry_changed or self._last_layout is None:
            layout = overlay_layout(geometry)
        else:
            layout = self._last_layout
        static_order = ("shield",) + self._VISUAL_WINDOW_ORDER
        if first_show or geometry_changed:
            for name in static_order:
                window = self._windows[name]
                x, y, width, height = layout[name]
                self._x11.XMoveResizeWindow(
                    self._display, window, x, y, width, height
                )
        raise_due = bool(
            first_show
            or geometry_changed
            or self._last_raise_s is None
            or now - self._last_raise_s >= self._RAISE_INTERVAL_S
        )
        if first_show:
            for name in static_order:
                self._x11.XMapRaised(self._display, self._windows[name])
        elif raise_due:
            for name in static_order:
                self._x11.XRaiseWindow(self._display, self._windows[name])
        if first_show or geometry_changed or model_changed:
            self._draw_panel(
                layout,
                model,
                command_status,
                strategy_model,
                motion_model,
            )
        pointer_x, pointer_y = pointer
        pointer_changed = pointer != self._last_pointer
        if not self._cursor_visible or pointer_changed:
            for name in self._CURSOR_WINDOW_ORDER:
                window = self._windows[name]
                self._x11.XMoveResizeWindow(
                    self._display,
                    window,
                    pointer_x,
                    pointer_y,
                    _CURSOR_WIDTH,
                    _CURSOR_HEIGHT,
                )
        if not self._cursor_visible:
            for name in self._CURSOR_WINDOW_ORDER:
                window = self._windows[name]
                self._x11.XMapRaised(self._display, window)
        elif raise_due:
            for name in self._CURSOR_WINDOW_ORDER:
                window = self._windows[name]
                self._x11.XRaiseWindow(self._display, window)
        if (
            first_show
            or geometry_changed
            or model_changed
            or pointer_changed
            or raise_due
        ):
            self._x11.XFlush(self._display)
        self._last_layout = layout
        self._last_geometry = geometry
        self._last_panel_model = model
        self._last_motion_model = motion_model
        self._last_strategy_model = strategy_model
        self._last_rendered_font_size = getattr(
            self,
            "_font_size",
            _DEFAULT_OVERLAY_FONT_SIZE,
        )
        self._last_page = getattr(self, "_active_page", "loadout")
        self._last_command_status = command_status
        self._last_command_revision = self._command_editor.revision
        self._last_pointer = pointer
        if raise_due:
            self._last_raise_s = now
        self._visible = True
        self._cursor_visible = True

    def hide(self, publisher: PointerActionPublisher | None = None) -> None:
        self._force_end_command_editing(publisher)
        if not self._visible and not self._cursor_visible:
            return
        for window in self._windows.values():
            self._x11.XUnmapWindow(self._display, window)
        self._x11.XFlush(self._display)
        self._visible = False
        self._cursor_visible = False
        self._last_layout = None
        self._last_geometry = None
        self._last_panel_model = None
        self._last_motion_model = None
        self._last_strategy_model = None
        self._last_page = None
        self._last_command_status = command_console_status({})
        self._last_command_revision = self._command_editor.revision
        self._last_pointer = None
        self._last_raise_s = None
        self._pressed_action = None
        self._pressed_window = None
        self._font_slider_dragging = False
        self._last_rendered_font_size = None
        self._active_page = "loadout"

    def close(self) -> None:
        display = getattr(self, "_display", None)
        if not display:
            return
        editor = getattr(self, "_command_editor", None)
        if editor is not None:
            editor.end(force=True)
        self._ungrab_keyboard()
        xft = getattr(self, "_xft", None)
        visual = getattr(self, "_visual", None)
        colormap = getattr(self, "_colormap", None)
        if xft is not None and visual and colormap is not None:
            for colour in getattr(self, "_xft_colours", {}).values():
                xft.XftColorFree(
                    display,
                    visual,
                    colormap,
                    ctypes.byref(colour),
                )
            getattr(self, "_xft_colours", {}).clear()
            for attribute in ("_xft_body_font", "_xft_large_font"):
                font = getattr(self, attribute, None)
                if font is not None:
                    xft.XftFontClose(display, ctypes.c_void_p(font))
                    setattr(self, attribute, None)
            xft_draw = getattr(self, "_xft_draw", None)
            if xft_draw is not None:
                xft.XftDrawDestroy(ctypes.c_void_p(xft_draw))
                self._xft_draw = None
        panel_gc = getattr(self, "_panel_gc", None)
        if panel_gc is not None:
            self._x11.XFreeGC(display, ctypes.c_void_p(panel_gc))
            self._panel_gc = None
        for attribute in ("_body_font", "_large_font"):
            font = getattr(self, attribute, None)
            if font is not None:
                self._x11.XFreeFont(display, font)
                setattr(self, attribute, None)
        windows = getattr(self, "_windows", {})
        for window in windows.values():
            self._x11.XDestroyWindow(display, window)
        windows.clear()
        self._x11.XSync(display, 0)
        # XSync has delivered every request issued while our scoped handler was
        # active.  Restore the process-global Xlib handler before invalidating
        # this display pointer.
        self._restore_x_error_handler()
        self._x11.XCloseDisplay(display)
        self._display = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--state-file", type=Path, required=True)
    parser.add_argument("--status-file", type=Path, required=True)
    parser.add_argument("--expected-ue-pid", type=int, required=True)
    parser.add_argument("--expected-parent-pid", type=int, required=True)
    parser.add_argument("--action-fd", type=int, required=True)
    parser.add_argument("--action-session", required=True)
    parser.add_argument("--display", default=os.environ.get("DISPLAY"))
    parser.add_argument("--poll-hz", type=float, default=30.0)
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    for name in ("state_file", "status_file"):
        path = getattr(args, name)
        if not path.is_absolute():
            raise SystemExit(f"--{name.replace('_', '-')} must be an absolute path")
        if not path.parent.is_dir():
            raise SystemExit(
                f"--{name.replace('_', '-')} parent does not exist: {path.parent}"
            )
    if args.expected_ue_pid <= 1:
        raise SystemExit("--expected-ue-pid must be greater than 1")
    if args.expected_parent_pid <= 1:
        raise SystemExit("--expected-parent-pid must be greater than 1")
    if args.action_fd < 0:
        raise SystemExit("--action-fd must be non-negative")
    if not args.action_session or len(args.action_session) > 128:
        raise SystemExit("--action-session must be non-empty and bounded")
    if not math.isfinite(args.poll_hz) or not 1.0 <= args.poll_hz <= 120.0:
        raise SystemExit("--poll-hz must be finite and in [1, 120]")


def main() -> int:
    args = parse_args()
    validate_args(args)
    running = True

    def stop(_signum: int, _frame: object) -> None:
        nonlocal running
        running = False

    previous_handlers = {
        signum: signal.signal(signum, stop)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    overlay: X11CalibrationOverlay | None = None
    action_publisher: PointerActionPublisher | None = None
    font_diagnostics: dict[str, str | int | bool | None] | None = None
    x11_diagnostics: dict[str, object] | None = None
    return_code = 0
    exit_reason = "signal"
    try:
        arm_parent_death_signal(args.expected_parent_pid)
        action_publisher = PointerActionPublisher(
            file_descriptor=args.action_fd,
            session=args.action_session,
        )
        overlay = X11CalibrationOverlay(
            display_name=args.display,
            expected_ue_pid=args.expected_ue_pid,
        )
        font_diagnostics = overlay.font_diagnostics
        x11_diagnostics = overlay.x11_diagnostics
        atomic_json(
            args.status_file,
            {
                "ready": True,
                "pid": os.getpid(),
                "expected_ue_pid": args.expected_ue_pid,
                "fonts": font_diagnostics,
                "x11": x11_diagnostics,
            },
        )
        interval = 1.0 / args.poll_hz
        while running:
            if os.getppid() != args.expected_parent_pid:
                exit_reason = "parent_exit"
                break
            state = read_overlay_state(args.state_file)
            if state is not None and state.get("active") is True:
                target = overlay.find_target()
                pointer = overlay.pointer_position()
                if (
                    target is None
                    or pointer is None
                    or not overlay_supported(target)
                ):
                    overlay.hide(action_publisher)
                else:
                    overlay.show(target, pointer, state)
                assert action_publisher is not None
                overlay.drain_pointer_actions(action_publisher)
            else:
                overlay.hide(action_publisher)
                assert action_publisher is not None
                overlay.drain_pointer_actions(action_publisher)
            time.sleep(interval)
    except Exception as exc:
        return_code = 1
        exit_reason = f"error:{type(exc).__name__}:{exc}"
        print(f"matrix-calibration-overlay ERROR {exc}", flush=True)
    finally:
        if overlay is not None:
            # Capture the live font selection and any recovered X11 race before
            # close tears down the display and its process-global error handler.
            font_diagnostics = overlay.font_diagnostics
            x11_diagnostics = overlay.x11_diagnostics
            overlay.close()
        if action_publisher is not None:
            action_publisher.close()
        try:
            atomic_json(
                args.status_file,
                {
                    "ready": False,
                    "pid": os.getpid(),
                    "expected_ue_pid": args.expected_ue_pid,
                    "exit_reason": exit_reason,
                    "fonts": font_diagnostics,
                    "x11": x11_diagnostics,
                },
            )
        except OSError:
            pass
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
