#!/usr/bin/env python3
"""Show a supervised, click-through X11 calibration overlay above Matrix UE.

The cooked Matrix build has no project sources with which to add a native UMG
pause widget.  This process therefore renders four tiny crosshair bars, one
hint window, and a shaped visible proxy for UE's hidden cursor directly on X11.
Every overlay window is override-redirect and has an empty XFixes input shape:
it neither takes keyboard focus nor receives mouse clicks.  The windows follow
the mapped UE client carrying ``_NET_WM_PID``.
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
import tempfile
import time
from typing import Any, Callable


_IS_VIEWABLE = 2
_CW_OVERRIDE_REDIRECT = 1 << 9
_SHAPE_BOUNDING = 0
_SHAPE_INPUT = 2
_PR_SET_PDEATHSIG = 1
_CURSOR_WIDTH = 20
_CURSOR_HEIGHT = 28


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


def overlay_layout(geometry: WindowGeometry) -> dict[str, tuple[int, int, int, int]]:
    """Return screen-space rectangles whose bars intersect the exact centre."""

    centre_x, centre_y = geometry.centre
    hint_width = min(900, max(240, geometry.width - 32))
    hint_height = 70
    hint_x = centre_x - hint_width // 2
    hint_y = centre_y + 48
    if hint_y + hint_height > geometry.y + geometry.height:
        hint_y = centre_y - 48 - hint_height
    return {
        "horizontal-shadow": (centre_x - 34, centre_y - 3, 69, 7),
        "vertical-shadow": (centre_x - 3, centre_y - 34, 7, 69),
        "horizontal": (centre_x - 32, centre_y - 1, 65, 3),
        "vertical": (centre_x - 1, centre_y - 32, 3, 65),
        "hint": (hint_x, hint_y, hint_width, hint_height),
    }


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


def settings_hint_lines(state: dict[str, object]) -> tuple[bytes, bytes, bytes]:
    """Render only validated state supplied by the supervised provider."""

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

    def profile(value: object) -> str:
        return "Remote" if value == "remote" else "Local"

    def finite(value: object, fallback: float) -> float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return fallback
        number = float(value)
        return number if math.isfinite(number) else fallback

    current_scale = finite(current.get("effective_scale"), 1.0)
    next_scale = finite(next_launch.get("effective_scale"), 1.0)
    pending = settings.get("pending_restart") is True
    requested = restart.get("requested") is True
    restart_available = restart.get("available") is True
    persistence_error = settings.get("persistence_error")
    line1 = (
        f"CURRENT APPLIED (SDL): {profile(current.get('profile'))} "
        f"{current_scale:.2f}x | "
        f"NEXT LAUNCH: {profile(next_launch.get('profile'))} {next_scale:.2f}x | "
        f"{'PENDING RESTART' if pending else 'CURRENT'}"
    )
    base = finite(mirror.get("base_deg_per_px"), 0.0)
    effective = finite(mirror.get("effective_deg_per_px"), 0.0)
    line2 = (
        f"x11 mirror: base {base:.3f} -> effective {effective:.3f} deg/px | "
        f"{'SAVE ERROR' if persistence_error else 'SAVED'}"
    )
    if requested:
        action = "RESTART REQUESTED - keep controls released"
    elif pending and restart_available and not persistence_error:
        action = "F9: Apply & Restart"
    else:
        action = "F9: unavailable"
    line3 = (
        f"M: Local/Remote  -/+: next speed  {action} | "
        "F10: center  F12: MouseLock  ESC: return"
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

    _STATIC_WINDOW_ORDER = (
        "horizontal-shadow",
        "vertical-shadow",
        "horizontal",
        "vertical",
        "hint",
    )
    _CURSOR_WINDOW_ORDER = ("cursor-shadow", "cursor")
    _WINDOW_ORDER = _STATIC_WINDOW_ORDER + _CURSOR_WINDOW_ORDER

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
        self._hint_gc: int | None = None
        self._visible = False
        self._cursor_visible = False
        self._last_layout: dict[str, tuple[int, int, int, int]] | None = None
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
            "XChangeWindowAttributes": (
                [
                    ctypes.c_void_p,
                    ctypes.c_ulong,
                    ctypes.c_ulong,
                    ctypes.POINTER(XSetWindowAttributes),
                ],
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
        colours = {
            "horizontal-shadow": black,
            "vertical-shadow": black,
            "horizontal": accent,
            "vertical": accent,
            "hint": black,
            "cursor-shadow": black,
            "cursor": white,
        }
        try:
            for name in self._WINDOW_ORDER:
                width, height = (
                    (_CURSOR_WIDTH, _CURSOR_HEIGHT)
                    if name in self._CURSOR_WINDOW_ORDER
                    else (1, 1)
                )
                window = int(
                    self._x11.XCreateSimpleWindow(
                        self._display,
                        self._root,
                        -100,
                        -100,
                        width,
                        height,
                        0,
                        black,
                        colours[name],
                    )
                )
                if not window:
                    raise RuntimeError(f"cannot create overlay window {name}")
                self._windows[name] = window
                self._x11.XStoreName(
                    self._display,
                    window,
                    f"Matrix Calibration {name}".encode("ascii"),
                )
                self._make_click_through(window)
                if name == "cursor-shadow":
                    self._set_bounding_shape(window, _CURSOR_SHADOW_RECTANGLES)
                elif name == "cursor":
                    self._set_bounding_shape(window, _CURSOR_FOREGROUND_RECTANGLES)
            hint = self._windows["hint"]
            gc = self._x11.XCreateGC(self._display, hint, 0, None)
            if not gc:
                raise RuntimeError("cannot create overlay hint graphics context")
            self._hint_gc = int(gc)
            self._x11.XSetForeground(self._display, gc, white)
            self._x11.XSync(self._display, 0)
        except Exception:
            self.close()
            raise

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

    def show(
        self,
        geometry: WindowGeometry,
        pointer: tuple[int, int],
        hint_lines: tuple[bytes, bytes, bytes],
    ) -> None:
        layout = overlay_layout(geometry)
        for name in self._STATIC_WINDOW_ORDER:
            window = self._windows[name]
            x, y, width, height = layout[name]
            self._x11.XMoveResizeWindow(
                self._display, window, x, y, width, height
            )
            if not self._visible:
                self._x11.XMapRaised(self._display, window)
            else:
                self._x11.XRaiseWindow(self._display, window)
        hint = self._windows["hint"]
        self._x11.XClearWindow(self._display, hint)
        for index, message in enumerate(hint_lines):
            self._x11.XDrawString(
                self._display,
                hint,
                ctypes.c_void_p(self._hint_gc),
                12,
                18 + index * 21,
                message,
                len(message),
            )
        pointer_x, pointer_y = pointer
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
                self._x11.XMapRaised(self._display, window)
            else:
                self._x11.XRaiseWindow(self._display, window)
        self._x11.XFlush(self._display)
        self._last_layout = layout
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

    def close(self) -> None:
        display = getattr(self, "_display", None)
        if not display:
            return
        if self._hint_gc is not None:
            self._x11.XFreeGC(display, ctypes.c_void_p(self._hint_gc))
            self._hint_gc = None
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
    return_code = 0
    exit_reason = "signal"
    try:
        arm_parent_death_signal(args.expected_parent_pid)
        overlay = X11CalibrationOverlay(
            display_name=args.display,
            expected_ue_pid=args.expected_ue_pid,
        )
        atomic_json(
            args.status_file,
            {
                "ready": True,
                "pid": os.getpid(),
                "expected_ue_pid": args.expected_ue_pid,
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
                if target is None or pointer is None:
                    overlay.hide()
                else:
                    overlay.show(target, pointer, settings_hint_lines(state))
            else:
                overlay.hide()
            time.sleep(interval)
    except Exception as exc:
        return_code = 1
        exit_reason = f"error:{type(exc).__name__}:{exc}"
        print(f"matrix-calibration-overlay ERROR {exc}", flush=True)
    finally:
        if overlay is not None:
            overlay.close()
        try:
            atomic_json(
                args.status_file,
                {
                    "ready": False,
                    "pid": os.getpid(),
                    "expected_ue_pid": args.expected_ue_pid,
                    "exit_reason": exit_reason,
                },
            )
        except OSError:
            pass
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
