#!/usr/bin/env python3
"""Show a supervised, MC-style X11 settings overlay above Matrix UE.

The cooked Matrix build has no project sources with which to add a native UMG
pause widget.  This process therefore renders a large pointer-driven panel, a
modal InputOnly shield, four click-through crosshair bars, and a shaped visible
proxy for UE's hidden cursor directly on X11.  The override-redirect controls do
not take keyboard focus.  The child can publish bounded pointer *intents* over
an inherited socket, while the provider remains the sole owner of persistence,
the neutral-frame gate, calibration state, and restart authority.
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


_IS_VIEWABLE = 2
_CW_OVERRIDE_REDIRECT = 1 << 9
_CW_EVENT_MASK = 1 << 11
_SHAPE_BOUNDING = 0
_SHAPE_INPUT = 2
_INPUT_ONLY = 2
_BUTTON_PRESS = 4
_BUTTON_RELEASE = 5
_BUTTON_PRESS_MASK = 1 << 2
_BUTTON_RELEASE_MASK = 1 << 3
_PR_SET_PDEATHSIG = 1
_CURSOR_WIDTH = 20
_CURSOR_HEIGHT = 28
_MIN_CLIENT_WIDTH = 480
_MIN_CLIENT_HEIGHT = 360
_BODY_FONT_CANDIDATES = (b"10x20", b"9x15", b"fixed")
_LARGE_FONT_CANDIDATES = (b"12x24", b"10x20", b"fixed")


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


class XEvent(ctypes.Union):
    _fields_ = [
        ("type", ctypes.c_int),
        ("xbutton", XButtonEvent),
        ("padding", ctypes.c_long * 24),
    ]


class XFontStruct(ctypes.Structure):
    # Only the leading fields are accessed; Xlib owns the full allocation.
    _fields_ = [("ext_data", ctypes.c_void_p), ("fid", ctypes.c_ulong)]


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
    speed_y = panel_y + panel_height // 2 - safe_half_size - button_height
    profile_y = speed_y - gap - button_height
    profile_width = max(1, (panel_width - 2 * margin - gap) // 2)
    speed_width = max(48, min(132, (panel_width - 2 * margin) // 4))
    apply_height = max(42, min(80, button_height + 6))
    footer_space = 30 if compact else 42
    apply_y = panel_y + panel_height - footer_space - apply_height
    speed_value = (
        panel_x + margin + speed_width,
        speed_y,
        panel_width - 2 * margin - 2 * speed_width,
        button_height,
    )
    return {
        "shield": (geometry.x, geometry.y, geometry.width, geometry.height),
        "panel": (panel_x, panel_y, panel_width, panel_height),
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
        "crosshair_safe": (
            centre_x - safe_half_size,
            centre_y - safe_half_size,
            safe_half_size * 2,
            safe_half_size * 2,
        ),
    }


_PANEL_ACTIONS = (
    "profile_local",
    "profile_remote",
    "speed_down",
    "speed_up",
    "apply_return",
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
) -> str | None:
    """Hit-test X11 root coordinates, including remote-desktop absolute input."""

    for action in _PANEL_ACTIONS:
        rectangle = layout.get(action)
        if rectangle is not None and point_in_rectangle((root_x, root_y), rectangle):
            return action
    return None


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
        base_mirror_gain=finite(mirror.get("base_deg_per_px"), 0.0),
        effective_mirror_gain=finite(mirror.get("effective_deg_per_px"), 0.0),
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
        f"x11 mirror: base {model.base_mirror_gain:.3f} -> effective "
        f"{model.effective_mirror_gain:.3f} deg/px | "
        "presets 0.01-0.10/0.01, 0.20-1.00/0.10 | "
        f"{'ERROR' if model.error else 'SAVED'}"
    )
    line3 = (
        f"Click or M/-/+ to configure | Enter: {model.apply_label} | "
        "F9: fallback  F10/F12: MouseLock  ESC: return"
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
    """Publish one bounded JSON packet per completed pointer click."""

    def __init__(self, *, file_descriptor: int, session: str) -> None:
        if file_descriptor < 0:
            raise ValueError("action file descriptor must be non-negative")
        if not session or len(session) > 128:
            raise ValueError("action session must be non-empty and bounded")
        self._socket = socket.socket(fileno=file_descriptor)
        self._socket.setblocking(False)
        self._session = session
        self._sequence = 0

    def publish(self, action: str) -> None:
        if action not in _PANEL_ACTIONS:
            raise ValueError(f"unsupported pointer action: {action}")
        self._sequence += 1
        payload = json.dumps(
            {
                "version": 1,
                "session": self._session,
                "sequence": self._sequence,
                "action": action,
            },
            separators=(",", ":"),
            sort_keys=True,
        ).encode("ascii")
        if len(payload) >= 1024:
            raise RuntimeError("pointer action packet is oversized")
        try:
            self._socket.send(payload)
        except BlockingIOError as exc:
            raise RuntimeError("pointer action channel is full") from exc

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
    ) -> None:
        if expected_ue_pid <= 1:
            raise ValueError("expected UE PID must be greater than 1")
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
        self._x11 = x11
        self._xfixes = xfixes
        self._configure_signatures()
        encoded_display = display_name.encode() if display_name else None
        self._display = self._x11.XOpenDisplay(encoded_display)
        if not self._display:
            label = display_name or os.environ.get("DISPLAY", "<unset>")
            raise RuntimeError(f"cannot open X11 display {label}")
        self._screen = int(self._x11.XDefaultScreen(self._display))
        self._root = int(self._x11.XRootWindow(self._display, self._screen))
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
        self._colours: dict[str, int] = {}
        self._visible = False
        self._cursor_visible = False
        self._last_layout: dict[str, tuple[int, int, int, int]] | None = None
        self._last_geometry: WindowGeometry | None = None
        self._last_panel_model: SettingsPanelModel | None = None
        self._last_pointer: tuple[int, int] | None = None
        self._last_raise_s: float | None = None
        self._pressed_action: str | None = None
        self._pressed_window: int | None = None
        self._target_window: int | None = None
        self._create_windows()

    def _configure_signatures(self) -> None:
        signatures = {
            "XOpenDisplay": ([ctypes.c_char_p], ctypes.c_void_p),
            "XDefaultScreen": ([ctypes.c_void_p], ctypes.c_int),
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
        attributes.event_mask = _BUTTON_PRESS_MASK | _BUTTON_RELEASE_MASK
        self._x11.XChangeWindowAttributes(
            self._display,
            window,
            _CW_OVERRIDE_REDIRECT | _CW_EVENT_MASK,
            ctypes.byref(attributes),
        )
        self._x11.XSelectInput(
            self._display,
            window,
            _BUTTON_PRESS_MASK | _BUTTON_RELEASE_MASK,
        )

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
        accent = self._named_colour(b"#ff3158", white)
        panel_background = self._named_colour(b"#151a24", black)
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
            "white": white,
            "muted": self._named_colour(b"#aeb8ca", white),
            "button": self._named_colour(b"#293345", white),
            "selected": self._named_colour(b"#3975e8", white),
            "disabled": self._named_colour(b"#343946", white),
            "apply": self._named_colour(b"#25845c", white),
            "pending": self._named_colour(b"#c07a28", white),
            "error": self._named_colour(b"#cf4655", white),
            "outline": self._named_colour(b"#71809a", white),
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

    @property
    def font_diagnostics(self) -> dict[str, str | None]:
        return {
            "body": self._body_font_name,
            "large": self._large_font_name,
        }

    def _window_pid(self, window: int) -> int | None:
        if not self._pid_atom:
            return None
        actual_type = ctypes.c_ulong()
        actual_format = ctypes.c_int()
        item_count = ctypes.c_ulong()
        bytes_after = ctypes.c_ulong()
        data = ctypes.POINTER(ctypes.c_ubyte)()
        status = self._x11.XGetWindowProperty(
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
        )
        try:
            if status != 0 or actual_format.value != 32 or item_count.value < 1:
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
        ok = self._x11.XQueryTree(
            self._display,
            window,
            ctypes.byref(root),
            ctypes.byref(parent),
            ctypes.byref(children),
            ctypes.byref(count),
        )
        try:
            if not ok:
                return []
            return [int(children[index]) for index in range(count.value)]
        finally:
            if children:
                self._x11.XFree(children)

    def _geometry(self, window: int) -> WindowGeometry | None:
        attributes = XWindowAttributes()
        if not self._x11.XGetWindowAttributes(
            self._display, window, ctypes.byref(attributes)
        ):
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
        if not self._x11.XTranslateCoordinates(
            self._display,
            window,
            self._root,
            0,
            0,
            ctypes.byref(root_x),
            ctypes.byref(root_y),
            ctypes.byref(child),
        ):
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
            large=True,
            centred_in=(x, y, width, height),
        )

    def _draw_panel(
        self,
        layout: dict[str, tuple[int, int, int, int]],
        model: SettingsPanelModel,
    ) -> None:
        _panel_x, _panel_y, panel_width, panel_height = layout["panel"]
        compact = panel_height < 600
        panel = self._windows["panel"]
        self._x11.XClearWindow(self._display, panel)
        self._draw_text(
            "MATRIX MOUSE CONTROLS",
            x=24 if compact else 40,
            y=20 if compact else max(36, panel_height // 14),
            colour=self._colours["white"],
            large=True,
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
        status_y = panel_height // 2 + (62 if compact else 72)
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
        self._draw_text(
            status_text,
            x=32,
            y=status_y,
            colour=status_colour,
        )
        if not compact:
            self._draw_text(
                "Fine: 0.01-0.10 by 0.01 | Coarse: 0.20-1.00 by 0.10",
                x=32,
                y=status_y + 22,
                colour=self._colours["muted"],
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
        footer = "Enter: Apply/Return   Esc: Back   F9: fallback   F10/F12: MouseLock"
        self._draw_text(
            footer,
            x=20 if compact else 40,
            y=max(18, panel_height - 10),
            colour=self._colours["muted"],
        )

    def drain_pointer_actions(self, publisher: PointerActionPublisher) -> int:
        """Commit only a left-button release inside its original large button."""

        emitted = 0
        while self._x11.XPending(self._display) > 0:
            event = XEvent()
            self._x11.XNextEvent(self._display, ctypes.byref(event))
            button = event.xbutton
            if button.button != 1:
                continue
            layout = self._last_layout
            action = (
                panel_action_at(layout, button.x_root, button.y_root)
                if layout is not None
                else None
            )
            if event.type == _BUTTON_PRESS:
                self._pressed_action = action
                self._pressed_window = int(button.window)
            elif event.type == _BUTTON_RELEASE:
                pressed = self._pressed_action
                pressed_window = self._pressed_window
                self._pressed_action = None
                self._pressed_window = None
                if (
                    pressed is not None
                    and action == pressed
                    and pressed_window == int(button.window)
                    and self._last_panel_model is not None
                    and self._last_panel_model.action_enabled(action)
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
        model_changed = model != self._last_panel_model
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
            self._draw_panel(layout, model)
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
        self._last_pointer = pointer
        if raise_due:
            self._last_raise_s = now
        self._visible = True
        self._cursor_visible = True

    def hide(self) -> None:
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
        self._last_pointer = None
        self._last_raise_s = None
        self._pressed_action = None
        self._pressed_window = None

    def close(self) -> None:
        display = getattr(self, "_display", None)
        if not display:
            return
        if self._panel_gc is not None:
            self._x11.XFreeGC(display, ctypes.c_void_p(self._panel_gc))
            self._panel_gc = None
        for attribute in ("_body_font", "_large_font"):
            font = getattr(self, attribute, None)
            if font is not None:
                self._x11.XFreeFont(display, font)
                setattr(self, attribute, None)
        for window in self._windows.values():
            self._x11.XDestroyWindow(display, window)
        self._windows.clear()
        self._x11.XSync(display, 0)
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
    font_diagnostics: dict[str, str | None] | None = None
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
        atomic_json(
            args.status_file,
            {
                "ready": True,
                "pid": os.getpid(),
                "expected_ue_pid": args.expected_ue_pid,
                "fonts": font_diagnostics,
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
                    overlay.hide()
                else:
                    overlay.show(target, pointer, state)
                assert action_publisher is not None
                overlay.drain_pointer_actions(action_publisher)
            else:
                overlay.hide()
                assert action_publisher is not None
                overlay.drain_pointer_actions(action_publisher)
            time.sleep(interval)
    except Exception as exc:
        return_code = 1
        exit_reason = f"error:{type(exc).__name__}:{exc}"
        print(f"matrix-calibration-overlay ERROR {exc}", flush=True)
    finally:
        if overlay is not None:
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
                },
            )
        except OSError:
            pass
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
