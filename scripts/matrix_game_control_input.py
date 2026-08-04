#!/usr/bin/env python3
"""Capture local Matrix UI input and publish strict game-control snapshots.

This is the operator-side adapter for :mod:`matrix_game_control`.  It does not
publish SONIC planner messages: the physics runtime remains the only owner of
that native wire.  Complete input snapshots instead travel over a local Linux
``AF_UNIX/SOCK_SEQPACKET`` connection, using the schema and encoder owned by the
control core.

The default backend polls X11 with ``libX11`` and Linux ``/dev/input/js*``
directly, so no pygame, evdev, or Python Xlib package is required.  CARLA and
the supervised UE final-POV reader are imported only when explicitly selected.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import ctypes
import ctypes.util
from dataclasses import dataclass
import errno
import glob
import importlib
import json
import math
import os
from pathlib import Path
import re
import signal
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import traceback
from typing import Any, Callable, Iterator, Mapping, Protocol

from matrix_build_info import (
    BuildInfoError,
    parse_build_info_json,
    unavailable_build_info,
)
from matrix_mouse_settings import (
    PROFILE_LOCAL,
    PROFILE_REMOTE,
    MouseSettings,
    atomic_save_settings,
    canonical_remote_speed_scale,
    default_settings_file,
    load_settings,
    step_remote_speed_scale,
)
from matrix_motion_settings import (
    CAMERA_HEADING_SNAP_ERROR_PATH,
    GEAR_RUN,
    GEAR_SLOW,
    GEAR_WALK,
    GAIT_START_HEADING_ERROR_PATH,
    GAIT_STOP_HEADING_ERROR_PATH,
    KEYBOARD_LOOK_RATE_PATH,
    KEYBOARD_TURN_RATE_PATH,
    MotionSettings,
    MotionSettingsError,
    MotionSettingsPersistenceError,
    MotionSettingsStore,
    SPEED_FIELD,
)
from matrix_ui_settings import (
    UiSettings,
    atomic_save_settings as atomic_save_ui_settings,
    canonical_font_size,
    load_settings_with_legacy_fallback as load_ui_settings,
    step_font_size,
)
from matrix_video_settings import (
    CAMERA_DISTANCE_CM_FIELD,
    VideoSettingsError,
    VideoSettingsPersistenceError,
    VideoSettingsStore,
)
from matrix_restart_request import (
    RestartRequest,
    atomic_write_request,
    read_capability,
)
from matrix_game_control import (
    InputSnapshot,
    KeySnapshot,
    MAX_PACKET_BYTES,
    MoveStickSnapshot,
    apply_radial_deadzone,
    encode_input_packet,
    wrap_angle_rad,
)
from matrix_movement_modes import (
    DEFAULT_MOVEMENT_MODE,
    next_movement_mode,
    validate_movement_mode,
)
from matrixctl import MatrixEngineInputClient
from matrix_mc_commands import (
    CommandParseError,
    CommandProtocolError,
    GameCommandRequest,
    MAX_RUNTIME_PAUSE_EPOCH,
    MovementModeSet,
    MAX_COMMAND_CHARS,
    MAX_COMMAND_PACKET_BYTES,
    MotionSettingSet,
    RuntimePauseSet,
    WORLD_SCENE_TARGETS,
    decode_command_response,
    encode_command_request,
    parse_mc_command,
)


DEFAULT_SOCKET = Path(
    os.environ.get(
        "MATRIX_GAME_INPUT_SOCKET",
        os.fspath(
            Path(os.environ.get("XDG_RUNTIME_DIR", tempfile.gettempdir()))
            / f"matrix-game-control-{os.getuid()}.sock"
        ),
    )
)

_MOTION_PANEL_ACTIONS: dict[str, tuple[str, int]] = {
    "motion_slow_speed_down": (
        f"control.motion.gears.{GEAR_SLOW}.{SPEED_FIELD}",
        -1,
    ),
    "motion_slow_speed_up": (
        f"control.motion.gears.{GEAR_SLOW}.{SPEED_FIELD}",
        1,
    ),
    "motion_walk_speed_down": (
        f"control.motion.gears.{GEAR_WALK}.{SPEED_FIELD}",
        -1,
    ),
    "motion_walk_speed_up": (
        f"control.motion.gears.{GEAR_WALK}.{SPEED_FIELD}",
        1,
    ),
    "motion_run_speed_down": (
        f"control.motion.gears.{GEAR_RUN}.{SPEED_FIELD}",
        -1,
    ),
    "motion_run_speed_up": (
        f"control.motion.gears.{GEAR_RUN}.{SPEED_FIELD}",
        1,
    ),
    "motion_turn_rate_down": (KEYBOARD_TURN_RATE_PATH, -1),
    "motion_turn_rate_up": (KEYBOARD_TURN_RATE_PATH, 1),
    "motion_look_rate_down": (KEYBOARD_LOOK_RATE_PATH, -1),
    "motion_look_rate_up": (KEYBOARD_LOOK_RATE_PATH, 1),
    "motion_gait_start_heading_error_down": (GAIT_START_HEADING_ERROR_PATH, -1),
    "motion_gait_start_heading_error_up": (GAIT_START_HEADING_ERROR_PATH, 1),
    "motion_gait_stop_heading_error_down": (GAIT_STOP_HEADING_ERROR_PATH, -1),
    "motion_gait_stop_heading_error_up": (GAIT_STOP_HEADING_ERROR_PATH, 1),
    "motion_camera_heading_snap_error_down": (
        CAMERA_HEADING_SNAP_ERROR_PATH,
        -1,
    ),
    "motion_camera_heading_snap_error_up": (
        CAMERA_HEADING_SNAP_ERROR_PATH,
        1,
    ),
}
_VIDEO_PANEL_ACTIONS: dict[str, tuple[str, int]] = {
    "video_camera_distance_down": (CAMERA_DISTANCE_CM_FIELD, -1),
    "video_camera_distance_up": (CAMERA_DISTANCE_CM_FIELD, 1),
}
_MOVEMENT_MODE_ACTIONS = frozenset(
    f"movement_mode_{movement_mode}"
    for movement_mode in ("camera_face", "camera_strafe", "body_relative")
)
_UI_PANEL_ACTIONS = frozenset({"font_down", "font_up"})
_JS_EVENT = struct.Struct("IhBB")
_JS_EVENT_BUTTON = 0x01
_JS_EVENT_AXIS = 0x02
_JS_EVENT_INIT = 0x80
DEFAULT_CARLA_WRITE_READBACK_TOLERANCE_RAD = math.radians(0.5)


_X11_BAD_WINDOW = 3
_X11_KEY_PRESS = 2
_X11_KEY_RELEASE = 3
_X11_GRAB_MODE_ASYNC = 1
_X11_LOCK_MASK = 1 << 1
_X11_MOD2_MASK = 1 << 4
_X11_ERROR_HANDLER_LOCK = threading.RLock()
_X11_UI_GRAB_MODIFIERS = (
    0,
    _X11_LOCK_MASK,
    _X11_MOD2_MASK,
    _X11_LOCK_MASK | _X11_MOD2_MASK,
)
_X11_UI_GRAB_KEY_NAMES = ("escape", "q", "e")
_MAX_X11_GRABBED_UI_EVENTS_PER_POLL = 128


class _XErrorEvent(ctypes.Structure):
    """Public ``XErrorEvent`` layout from Xlib.h."""

    _fields_ = (
        ("type", ctypes.c_int),
        ("display", ctypes.c_void_p),
        ("resourceid", ctypes.c_ulong),
        ("serial", ctypes.c_ulong),
        ("error_code", ctypes.c_ubyte),
        ("request_code", ctypes.c_ubyte),
        ("minor_code", ctypes.c_ubyte),
    )


class _XKeyEvent(ctypes.Structure):
    """Public ``XKeyEvent`` prefix from Xlib.h."""

    _fields_ = (
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
    )


_X11_ERROR_HANDLER = ctypes.CFUNCTYPE(
    ctypes.c_int,
    ctypes.c_void_p,
    ctypes.POINTER(_XErrorEvent),
)


@dataclass
class _X11FocusErrorScope:
    """Errors and window IDs owned by one synchronous focus-chain query."""

    windows: set[int]
    label: str = "focus query"
    stale_window: int | None = None
    unexpected_error: tuple[int, int, int, int] | None = None


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


@dataclass(frozen=True)
class KeyboardMouseSample:
    w: bool = False
    a: bool = False
    s: bool = False
    d: bool = False
    q: bool = False
    e: bool = False
    v: bool = False
    x: bool = False
    j: bool = False
    k: bool = False
    l: bool = False
    u: bool = False
    i: bool = False
    o: bool = False
    arrow_left: bool = False
    arrow_up: bool = False
    arrow_right: bool = False
    arrow_down: bool = False
    ctrl: bool = False
    alt: bool = False
    shift: bool = False
    escape: bool = False
    mouse_mode: bool = False
    mouse_speed_down: bool = False
    mouse_speed_up: bool = False
    apply_restart: bool = False
    apply_return: bool = False
    movement_mode_cycle: bool = False
    mouse_dx: float = 0.0
    mouse_dy: float = 0.0
    camera_dragging: bool = False
    focused: bool = False
    focus_title: str | None = None
    focus_pid: int | None = None

    def keys(self, *, movement_enabled: bool = True) -> KeySnapshot:
        return KeySnapshot(
            w=self.w and movement_enabled,
            a=self.a and movement_enabled,
            s=self.s and movement_enabled,
            d=self.d and movement_enabled,
            # Q/E and V remain available as actions in every control mode.
            q=self.q,
            e=self.e,
            v=self.v,
            x=self.x and movement_enabled,
            j=self.j and movement_enabled,
            k=self.k and movement_enabled,
            l=self.l and movement_enabled,
            u=self.u and movement_enabled,
            i=self.i and movement_enabled,
            o=self.o and movement_enabled,
            ctrl=self.ctrl and movement_enabled,
            alt=self.alt and movement_enabled,
            shift=self.shift and movement_enabled,
        )


@dataclass(frozen=True)
class GamepadSample:
    forward: float = 0.0
    right: float = 0.0
    look_yaw: float = 0.0
    look_pitch: float = 0.0
    connected: bool = False


class CalibrationModeController:
    """Toggle a fail-closed calibration mode on focused Escape press edges.

    Escape is deliberately handled outside the wire protocol.  While active,
    :func:`apply_calibration_interlock` publishes an unfocused, fully neutral
    snapshot.  The existing control core therefore performs its immediate
    safe-stop and owns the neutral re-arm sequence when calibration ends.
    """

    def __init__(self) -> None:
        self.active = False
        self._escape_was_down = False
        self.toggle_count = 0

    def update(self, *, escape_pressed: bool, ue_focused: bool) -> bool:
        toggled = False
        if (
            escape_pressed
            and not self._escape_was_down
            and (ue_focused or self.active)
        ):
            self.active = not self.active
            self.toggle_count += 1
            toggled = True
        self._escape_was_down = escape_pressed
        return toggled

    def exit(self) -> bool:
        """Leave the ESC interlock without synthesizing an Escape key press."""

        if not self.active:
            return False
        self.active = False
        self.toggle_count += 1
        return True


class StartupShortcutArming:
    """Require release of ESC/F9 once in every newly launched generation."""

    def __init__(self) -> None:
        self.armed = False

    def update(self, *, escape_pressed: bool, restart_pressed: bool) -> bool:
        if not self.armed and not escape_pressed and not restart_pressed:
            self.armed = True
        return self.armed


@dataclass(frozen=True)
class AppliedMouseSettings:
    profile: str
    effective_scale: float

    def __post_init__(self) -> None:
        if self.profile not in {PROFILE_LOCAL, PROFILE_REMOTE}:
            raise ValueError(f"unsupported applied mouse profile: {self.profile}")
        try:
            canonical = canonical_remote_speed_scale(self.effective_scale)
        except ValueError as exc:
            raise ValueError(f"invalid applied mouse scale: {exc}") from exc
        if self.profile == PROFILE_LOCAL and canonical != 1.0:
            raise ValueError("Local applied mouse profile must be 1.0x")
        object.__setattr__(self, "effective_scale", canonical)


class MouseSettingsController:
    """Edit next-launch settings only while the ESC interlock is active."""

    def __init__(
        self,
        *,
        path: Path,
        desired: MouseSettings,
        load_status: str,
        load_error: str | None,
    ) -> None:
        self.path = path
        self.desired = desired
        self.load_status = load_status
        self.persistence_error = load_error
        self.change_count = 0
        self._mode_was_down = False
        self._down_was_down = False
        self._up_was_down = False

    def _replace(self, replacement: MouseSettings) -> bool:
        if replacement == self.desired:
            return False
        self.desired = replacement
        self.change_count += 1
        try:
            atomic_save_settings(self.path, replacement)
            self.persistence_error = None
            self.load_status = "saved"
        except (OSError, ValueError) as exc:
            self.persistence_error = str(exc)
        return True

    def update(
        self,
        *,
        active: bool,
        mode_pressed: bool,
        slower_pressed: bool,
        faster_pressed: bool,
    ) -> bool:
        mode_edge = mode_pressed and not self._mode_was_down
        slower_edge = slower_pressed and not self._down_was_down
        faster_edge = faster_pressed and not self._up_was_down
        self._mode_was_down = mode_pressed
        self._down_was_down = slower_pressed
        self._up_was_down = faster_pressed
        if not active:
            return False

        profile = self.desired.profile
        speed_scale = self.desired.speed_scale
        if mode_edge:
            profile = PROFILE_REMOTE if profile == PROFILE_LOCAL else PROFILE_LOCAL
        if profile == PROFILE_REMOTE:
            if slower_edge and not faster_edge:
                speed_scale = step_remote_speed_scale(speed_scale, -1)
            elif faster_edge and not slower_edge:
                speed_scale = step_remote_speed_scale(speed_scale, 1)
        return self._replace(MouseSettings(profile=profile, speed_scale=speed_scale))

    def apply_panel_action(self, action: str, *, active: bool) -> bool:
        """Apply one validated click without emulating a held keyboard key."""

        if not active:
            return False
        profile = self.desired.profile
        speed_scale = self.desired.speed_scale
        if action == "profile_local":
            profile = PROFILE_LOCAL
        elif action == "profile_remote":
            profile = PROFILE_REMOTE
        elif action == "speed_down" and profile == PROFILE_REMOTE:
            speed_scale = step_remote_speed_scale(speed_scale, -1)
        elif action == "speed_up" and profile == PROFILE_REMOTE:
            speed_scale = step_remote_speed_scale(speed_scale, 1)
        else:
            return False
        return self._replace(
            MouseSettings(profile=profile, speed_scale=speed_scale)
        )

    def pending_restart(self, applied: AppliedMouseSettings) -> bool:
        return bool(
            self.desired.profile != applied.profile
            or not math.isclose(
                self.desired.effective_scale,
                applied.effective_scale,
                rel_tol=1e-9,
                abs_tol=1e-9,
            )
        )

    def live_mapping(self, applied: AppliedMouseSettings) -> dict[str, object]:
        return {
            "settings_file": os.fspath(self.path),
            "current": {
                "profile": applied.profile,
                "effective_scale": applied.effective_scale,
            },
            "next_launch": {
                "profile": self.desired.profile,
                "speed_scale": self.desired.speed_scale,
                "effective_scale": self.desired.effective_scale,
            },
            "pending_restart": self.pending_restart(applied),
            "load_status": self.load_status,
            "persistence_error": self.persistence_error,
            "change_count": self.change_count,
        }


class UiSettingsController:
    """Persist operator-facing UI settings from the ESC overlay."""

    def __init__(
        self,
        *,
        path: Path | None,
        desired: UiSettings,
        load_status: str,
        load_error: str | None,
    ) -> None:
        self.path = path
        self.desired = desired
        self.load_status = load_status
        self.persistence_error = load_error
        self.change_count = 0

    def _replace(self, replacement: UiSettings) -> bool:
        if replacement == self.desired:
            return False
        self.desired = replacement
        self.change_count += 1
        if self.path is None:
            self.persistence_error = "UI settings file is unavailable"
            return True
        try:
            atomic_save_ui_settings(self.path, replacement)
            self.persistence_error = None
            self.load_status = "saved"
        except (OSError, ValueError) as exc:
            self.persistence_error = str(exc)
        return True

    def apply_panel_action(self, action: str, *, active: bool) -> bool:
        if not active or action not in _UI_PANEL_ACTIONS:
            return False
        direction = -1 if action == "font_down" else 1
        return self._replace(
            UiSettings(
                font_scale=self.desired.font_scale,
                font_size=step_font_size(self.desired.font_size, direction),
            )
        )

    def apply_font_size(self, font_size: int, *, active: bool) -> bool:
        if not active:
            return False
        return self._replace(
            UiSettings(
                font_scale=self.desired.font_scale,
                font_size=canonical_font_size(font_size),
            )
        )

    def live_mapping(self) -> dict[str, object]:
        return {
            "settings_file": os.fspath(self.path) if self.path is not None else None,
            "font_scale": self.desired.font_scale,
            "font_size": self.desired.font_size,
            "load_status": self.load_status,
            "persistence_error": self.persistence_error,
            "change_count": self.change_count,
        }


def motion_settings_live_mapping(
    store: MotionSettingsStore | None,
    *,
    applied: MotionSettings | None,
    change_count: int,
    persistence_error: str | None,
) -> dict[str, object]:
    if store is None:
        return {"available": False, "pending_restart": False}
    try:
        store.reload_if_changed()
    except (MotionSettingsError, OSError, ValueError) as exc:
        persistence_error = str(exc)
    mapping = store.mapping()
    mapping.update(
        {
            "available": True,
            "pending_restart": (
                applied is not None and store.settings != applied
            ),
            "persistence_error": persistence_error,
            "change_count": change_count,
        }
    )
    return mapping


def sync_confirmed_movement_mode(
    movement_mode: object,
    *,
    store: MotionSettingsStore | None,
    applied: MotionSettings | None,
) -> tuple[str, MotionSettings | None, bool, str | None]:
    """Mirror a runtime-confirmed movement mode into the ESC settings model.

    Movement-mode buttons send a hot runtime command first.  The ESC panel,
    however, highlights the selected button from the persisted motion settings
    model.  Keeping the sync on the confirmed response path makes button clicks
    and keyboard cycling share the same source of truth without changing the
    locomotion core.
    """

    confirmed = validate_movement_mode(movement_mode)
    if store is None:
        return confirmed, applied, False, None
    try:
        modification = store.modify_movement_mode(confirmed)
    except (
        MotionSettingsError,
        MotionSettingsPersistenceError,
        OSError,
        ValueError,
    ) as exc:
        return confirmed, applied, False, str(exc)
    return confirmed, modification.settings, modification.changed, None


def locked_sonic_strategy_loadout() -> dict[str, object]:
    """Describe the fixed stable runtime without exposing switch intents.

    The accepted walking line has exactly one locomotion policy and deliberately
    has no recovery worker.  This telemetry lets the main ESC presentation show
    that truth while the input provider's action allowlist remains unchanged.
    """

    return {
        "version": 1,
        "available": True,
        "status": "locked",
        "active_slot": "locomotion",
        "pending": None,
        "slots": [
            {
                "slot": "locomotion",
                "selected_policy_id": "sonic",
                "locked": True,
                "candidates": [
                    {
                        "policy_id": "sonic",
                        "name": "SONIC",
                        "resident": True,
                        "available": True,
                        "unavailable_reason": None,
                        "switch_mode": "disabled",
                    }
                ],
                "switch_mode": "disabled",
            },
            {
                "slot": "recovery",
                "selected_policy_id": "off",
                "locked": True,
                "candidates": [],
                "switch_mode": "disabled",
            },
        ],
        "resident_models": [
            {
                "policy_id": "sonic",
                "name": "SONIC",
                "resident": True,
                "available": True,
                "unavailable_reason": None,
            }
        ],
    }


def video_settings_live_mapping(
    store: VideoSettingsStore | None,
    *,
    applied_runtime: dict[str, object] | None,
    change_count: int,
    persistence_error: str | None,
) -> dict[str, object]:
    if store is None:
        return {"available": False, "pending_restart": False}
    mapping = store.mapping()
    pending_restart = False
    if applied_runtime is not None:
        pending_restart = store.settings.runtime_mapping() != applied_runtime
    mapping.update(
        {
            "available": True,
            "current": applied_runtime or store.settings.runtime_mapping(),
            "next_launch": store.settings.runtime_mapping(),
            "pending_restart": pending_restart,
            "persistence_error": persistence_error,
            "change_count": change_count,
        }
    )
    return mapping


def first_settings_error(*values: str | None) -> str | None:
    for value in values:
        if value:
            return value
    return None


class RuntimeRestartRequester:
    """Write one private request polled by the top-level launcher."""

    def __init__(
        self,
        *,
        request_file: Path | None,
        capability_file: Path | None,
        launcher_pid: int | None,
    ) -> None:
        self.request_file = request_file
        self.capability_file = capability_file
        self.launcher_pid = launcher_pid
        self.requested = False
        self.error: str | None = None

    @property
    def available(self) -> bool:
        return bool(
            self.request_file is not None
            and self.request_file.is_absolute()
            and self.request_file.parent.is_dir()
            and self.capability_file is not None
            and self.capability_file.is_absolute()
            and self.capability_file.is_file()
            and type(self.launcher_pid) is int
            and self.launcher_pid > 1
            and not self.requested
        )

    def request(self) -> bool:
        if not self.available:
            self.error = "whole-runtime restart channel is unavailable"
            return False
        assert self.request_file is not None
        assert self.capability_file is not None
        assert self.launcher_pid is not None
        try:
            nonce = read_capability(self.capability_file)
            atomic_write_request(
                self.request_file,
                RestartRequest(
                    launcher_pid=self.launcher_pid,
                    provider_pid=os.getpid(),
                    nonce=nonce,
                ),
            )
            self.requested = True
            self.error = None
            return True
        except (OSError, ValueError) as exc:
            self.error = str(exc)
            return False

    def mapping(self) -> dict[str, object]:
        return {
            "available": self.available,
            "requested": self.requested,
            "error": self.error,
        }


class ApplyRestartKey:
    """Accept only a fresh F9 edge while every safety precondition is true."""

    def __init__(self) -> None:
        self._was_down = False

    def update(
        self,
        *,
        pressed: bool,
        calibration_active: bool,
        neutral_frame_ready: bool,
        pending_restart: bool,
        persistence_ok: bool,
        requester: RuntimeRestartRequester,
    ) -> bool:
        edge = pressed and not self._was_down
        self._was_down = pressed
        if not (
            edge
            and calibration_active
            and neutral_frame_ready
            and pending_restart
            and persistence_ok
            and requester.available
        ):
            return False
        return requester.request()


class MovementModeCycleKey:
    """Accept one F6 edge after the physical key has been released once."""

    def __init__(self) -> None:
        self._was_down = False
        self._armed = False

    def update(self, pressed: bool, *, enabled: bool) -> bool:
        if type(pressed) is not bool or type(enabled) is not bool:
            raise TypeError("movement-mode key state must be boolean")
        edge = bool(pressed and not self._was_down and self._armed and enabled)
        if not pressed:
            self._armed = True
        self._was_down = pressed
        return edge


class ApplyReturnController:
    """Turn Enter/a panel click into a safe return or deferred restart.

    A click is an intent, not restart authority.  Pending changes remain in the
    ESC interlock until the provider has successfully delivered a neutral frame
    and the existing private :class:`RuntimeRestartRequester` accepts them.
    """

    def __init__(self) -> None:
        self._enter_armed = False
        self.pending_intent = False
        self.status = "idle"
        self.error: str | None = None

    def update(
        self,
        *,
        enter_pressed: bool,
        clicked: bool,
        ue_focused: bool,
        panel_was_active: bool,
        calibration: CalibrationModeController,
        neutral_frame_ready: bool,
        pending_restart: bool,
        persistence_error: str | None,
        requester: RuntimeRestartRequester,
    ) -> tuple[bool, bool]:
        """Return ``(left_calibration, requested_restart)`` for this frame."""

        if not calibration.active:
            self._enter_armed = False
            self.pending_intent = False
            self.status = "idle"
            self.error = None
            return (False, False)
        # Enter is globally visible through XQueryKeymap.  Treat it as a panel
        # key only after this activation has observed a focused release.  This
        # rejects ESC+Enter entry, terminal Enter, and a key held across an
        # Alt-Tab/focus transition.
        keyboard_trigger = False
        if not panel_was_active or not ue_focused:
            self._enter_armed = False
        elif not enter_pressed:
            self._enter_armed = True
        elif self._enter_armed:
            keyboard_trigger = True
            self._enter_armed = False
        triggered = bool(clicked or keyboard_trigger)
        if requester.requested:
            self.pending_intent = False
            self.status = "restarting"
            return (False, False)
        if triggered:
            self.pending_intent = True
            self.error = None
            self.status = "waiting_neutral"
        if not self.pending_intent:
            return (False, False)
        if not neutral_frame_ready:
            self.status = "waiting_neutral"
            return (False, False)
        if not pending_restart:
            self.pending_intent = False
            self.status = "returning"
            calibration.exit()
            return (True, False)
        if persistence_error is not None:
            self.pending_intent = False
            self.status = "error"
            self.error = f"settings were not saved: {persistence_error}"
            return (False, False)
        if not requester.available:
            self.pending_intent = False
            self.status = "error"
            self.error = "whole-runtime restart channel is unavailable"
            return (False, False)
        self.pending_intent = False
        if requester.request():
            self.status = "restarting"
            self.error = None
            return (False, True)
        self.status = "error"
        self.error = requester.error or "whole-runtime restart request failed"
        return (False, False)

    def mapping(self) -> dict[str, object]:
        return {
            "enter_armed": self._enter_armed,
            "pending_intent": self.pending_intent,
            "status": self.status,
            "error": self.error,
        }

    def cancel_pending(self) -> bool:
        """Cancel a deferred Apply/Return when command editing takes ownership."""

        changed = self.pending_intent or self.status == "waiting_neutral"
        self.pending_intent = False
        if self.status == "waiting_neutral":
            self.status = "idle"
            self.error = None
        return changed


def calibration_interlock_required(
    *, panel_was_active: bool, panel_active: bool
) -> bool:
    """Keep the complete exit frame neutral for both ESC and UI returns."""

    return bool(panel_active or (panel_was_active and not panel_active))


def apply_calibration_interlock(
    keyboard: KeyboardMouseSample,
    gamepad: GamepadSample,
    *,
    active: bool,
) -> tuple[KeyboardMouseSample, GamepadSample]:
    """Return locomotion-neutral, unfocused inputs while calibrating.

    V keeps its physical level only to preserve the core's edge memory; an
    unfocused snapshot cannot execute its mode toggle.
    """

    if not active:
        return keyboard, gamepad
    return (
        KeyboardMouseSample(
            # Preserve the physical level of V while unfocused so the core's
            # edge detector cannot mistake a held key for a fresh press when
            # calibration ends.  focused=False prevents it from toggling here.
            v=keyboard.v,
            focused=False,
            focus_title=keyboard.focus_title,
            focus_pid=keyboard.focus_pid,
        ),
        GamepadSample(),
    )


def select_physical_inputs(
    keyboard: KeyboardMouseSample,
    gamepad: GamepadSample,
    *,
    source: str,
) -> tuple[KeySnapshot, MoveStickSnapshot, float]:
    """Apply explicit source arbitration without combining locomotion axes.

    ``auto`` carries both devices; the core's documented digital-WASD priority
    makes arbitration deterministic.  Explicit modes zero the other device's
    locomotion fields.  Mouse look remains available in auto/keyboard mode and
    right-stick look remains available in auto/gamepad mode.
    """
    if source not in {"auto", "keyboard", "gamepad"}:
        raise ValueError(f"unsupported input source: {source}")
    keyboard_move = source in {"auto", "keyboard"}
    gamepad_move = source in {"auto", "gamepad"} and gamepad.connected
    keys = keyboard.keys(movement_enabled=keyboard_move)
    stick = MoveStickSnapshot(
        right=_clamp(gamepad.right, -1.0, 1.0) if gamepad_move else 0.0,
        forward=_clamp(gamepad.forward, -1.0, 1.0) if gamepad_move else 0.0,
    )
    if source == "keyboard":
        look_yaw = 0.0
    else:
        look_yaw = _clamp(gamepad.look_yaw, -1.0, 1.0) if gamepad.connected else 0.0
    return keys, stick, look_yaw


def effective_input_source(requested: str, camera_yaw_source: str) -> str:
    """Gate gamepad locomotion on an observed camera direction.

    With ``fixed`` or any X11 mirror the adapter cannot observe native UE
    right-stick camera response.  The mirrors observe input-side motion, but
    packaged-UE consumption has not been verified and none is a final rendered
    camera readback.  Auto therefore degrades to keyboard-only, while an
    explicit gamepad request fails instead of silently diverging.  CARLA and
    ``ue-final-pov`` provide an observed yaw and retain the requested source.
    """
    if requested not in {"auto", "keyboard", "gamepad"}:
        raise ValueError(f"unsupported input source: {requested}")
    if camera_yaw_source not in {
        "fixed",
        "x11-mirror",
        "x11-core-gated",
        "x11-absolute",
        "ue-final-pov",
        "carla",
    }:
        raise ValueError(f"unsupported camera yaw source: {camera_yaw_source}")
    if camera_yaw_source in {"carla", "ue-final-pov"}:
        return requested
    if requested == "gamepad":
        raise ValueError(
            "gamepad input requires an observed CARLA or UE final-POV camera yaw"
        )
    return "keyboard" if requested == "auto" else requested


def captures_xi2_drag_boundaries(camera_yaw_source: str) -> bool:
    """Whether XI2 must observe native look-button boundaries for a source."""

    if camera_yaw_source not in {
        "fixed",
        "x11-mirror",
        "x11-core-gated",
        "x11-absolute",
        "ue-final-pov",
        "carla",
    }:
        raise ValueError(f"unsupported camera yaw source: {camera_yaw_source}")
    # ue-final-pov gets yaw from UE memory, but still needs XI2's raw
    # press/motion/release edges to distinguish operator look input from
    # automatic robot-follow camera rotation.
    return camera_yaw_source in {
        "x11-mirror",
        "x11-core-gated",
        "ue-final-pov",
    }


def gamepad_input_available(
    source: str,
    *,
    connected: bool,
    previous_connected: bool | None,
) -> bool:
    """Interlock disconnect/reconnect edges before analog motion is accepted."""

    if source not in {"auto", "keyboard", "gamepad"}:
        raise ValueError(f"unsupported input source: {source}")
    if source == "keyboard":
        return True
    if source == "gamepad" and not connected:
        return False
    # A hotplug edge forces one unfocused frame.  The core then requires a
    # genuinely centered stick before a newly connected controller can move.
    if previous_connected is not None and connected != previous_connected:
        return False
    return True


class CameraYawTracker:
    """Track a provider-frame yaw from calibrated local pointer motion.

    This is only an input-side mirror of the packaged UI.  XI2 raw motion is a
    common SDL relative-input source and the launcher requests that mode, but
    this adapter cannot prove what the packaged UE build consumed.  It does
    not itself rotate or read back the visible camera.
    """

    def __init__(
        self,
        initial_yaw_rad: float,
        *,
        mouse_radians_per_pixel: float,
        gamepad_radians_per_second: float,
    ) -> None:
        self._yaw = wrap_angle_rad(initial_yaw_rad)
        self._mouse_scale = float(mouse_radians_per_pixel)
        self._gamepad_rate = float(gamepad_radians_per_second)

    @property
    def yaw(self) -> float:
        return self._yaw

    def update(
        self,
        *,
        dt: float,
        mouse_dx: float,
        gamepad_look_yaw: float,
        observed_yaw_rad: float | None = None,
    ) -> float:
        if observed_yaw_rad is not None:
            if not math.isfinite(observed_yaw_rad):
                raise ValueError("observed camera yaw must be finite")
            self._yaw = wrap_angle_rad(observed_yaw_rad)
            return self._yaw
        # Do not sum two look devices.  A non-zero mouse delta wins that frame.
        if abs(mouse_dx) > 1e-9:
            delta = mouse_dx * self._mouse_scale
        else:
            delta = (
                _clamp(gamepad_look_yaw, -1.0, 1.0)
                * self._gamepad_rate
                * max(0.0, dt)
            )
        self._yaw = wrap_angle_rad(self._yaw + delta)
        return self._yaw


class KeyboardCameraLookIntegrator:
    """Convert held arrow keys into bounded integer pointer deltas."""

    _MAX_FRAME_SECONDS = 0.05

    def __init__(self) -> None:
        self._residual_x = 0.0
        self._residual_y = 0.0
        self.generated_batches = 0
        self.last_dx = 0
        self.last_dy = 0

    def _reset(self) -> None:
        self._residual_x = 0.0
        self._residual_y = 0.0
        self.last_dx = 0
        self.last_dy = 0

    def update(
        self,
        keyboard: KeyboardMouseSample,
        *,
        dt: float,
        rate_deg_s: float,
        degrees_per_pixel: float,
        enabled: bool,
    ) -> tuple[int, int]:
        if not isinstance(keyboard, KeyboardMouseSample):
            raise TypeError("keyboard sample is required")
        if not math.isfinite(dt) or dt < 0.0:
            raise ValueError("dt must be finite and non-negative")
        for name, value in (
            ("rate_deg_s", rate_deg_s),
            ("degrees_per_pixel", degrees_per_pixel),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")
        if type(enabled) is not bool:
            raise TypeError("enabled must be boolean")
        yaw_axis = int(keyboard.arrow_right) - int(keyboard.arrow_left)
        pitch_axis = int(keyboard.arrow_down) - int(keyboard.arrow_up)
        if not enabled or (yaw_axis == 0 and pitch_axis == 0):
            self._reset()
            return (0, 0)

        frame_seconds = min(dt, self._MAX_FRAME_SECONDS)
        pixels = rate_deg_s * frame_seconds / degrees_per_pixel
        if yaw_axis:
            self._residual_x += yaw_axis * pixels
        else:
            self._residual_x = 0.0
        if pitch_axis:
            self._residual_y += pitch_axis * pixels
        else:
            self._residual_y = 0.0
        dx = math.trunc(self._residual_x)
        dy = math.trunc(self._residual_y)
        self._residual_x -= dx
        self._residual_y -= dy
        self.last_dx = dx
        self.last_dy = dy
        if dx or dy:
            self.generated_batches += 1
        return (dx, dy)

    @property
    def telemetry(self) -> dict[str, object]:
        return {
            "source": "x11-arrow-keys",
            "mapping": {
                "left_right": "camera_yaw",
                "up_down": "camera_pitch",
            },
            "maximum_frame_seconds": self._MAX_FRAME_SECONDS,
            "generated_batches": self.generated_batches,
            "last_dx": self.last_dx,
            "last_dy": self.last_dy,
        }


def keyboard_camera_arrow_active(keyboard: KeyboardMouseSample) -> bool:
    if not isinstance(keyboard, KeyboardMouseSample):
        raise TypeError("keyboard sample is required")
    return bool(
        keyboard.arrow_left != keyboard.arrow_right
        or keyboard.arrow_up != keyboard.arrow_down
    )


class EngineCameraLookWorker:
    """Coalesce provider deltas and perform bridge I/O off the 50 Hz loop."""

    _MAX_PENDING_DELTA = 4096
    _RETRY_SECONDS = 0.5

    def __init__(
        self,
        endpoint: Path,
        capability_file: Path,
        *,
        button: str,
        timeout_seconds: float = 0.2,
        client_factory: Callable[..., Any] = MatrixEngineInputClient,
    ) -> None:
        if not endpoint.is_absolute() or not capability_file.is_absolute():
            raise ValueError("engine camera endpoint paths must be absolute")
        if button not in {"left", "middle", "right"}:
            raise ValueError("engine camera look button is invalid")
        if not math.isfinite(timeout_seconds) or not 0.05 <= timeout_seconds <= 1.0:
            raise ValueError("engine camera timeout must be in [0.05, 1.0]")
        if not callable(client_factory):
            raise TypeError("engine camera client factory must be callable")
        self._endpoint = endpoint
        self._capability_file = capability_file
        self._button = button
        self._timeout_seconds = timeout_seconds
        self._client_factory = client_factory
        self._condition = threading.Condition()
        self._thread: threading.Thread | None = None
        self._stop_requested = False
        self._pending_dx = 0
        self._pending_dy = 0
        self._drag_requested = False
        self._release_requested = False
        self._retry_not_before = 0.0
        self._status = "stopped"
        self._available = False
        self._capability_compatible: bool | None = None
        self._last_error: str | None = None
        self.submitted_batches = 0
        self.coalesced_batches = 0
        self.emitted_batches = 0
        self.dropped_batches = 0
        self.release_requests = 0
        self.releases_emitted = 0

    def start(self) -> None:
        with self._condition:
            if self._thread is not None:
                return
            self._status = "probing"
            self._stop_requested = False
            self._thread = threading.Thread(
                target=self._run,
                name="matrix-engine-camera-look",
                daemon=True,
            )
            self._thread.start()

    def _request(self, action: str, payload: dict[str, object]) -> dict[str, object]:
        client = self._client_factory(
            self._endpoint,
            self._capability_file,
            timeout_seconds=self._timeout_seconds,
        )
        try:
            client.connect()
            return client.request(action, payload)
        finally:
            client.close()

    def _record_success(self) -> None:
        with self._condition:
            self._available = True
            self._capability_compatible = True
            self._status = "ready"
            self._last_error = None
            self._retry_not_before = 0.0

    def _record_error(self, exc: Exception, *, retryable: bool = True) -> None:
        with self._condition:
            self._available = False
            if not retryable:
                self._capability_compatible = False
            self._status = "unavailable" if retryable else "unsupported"
            self._last_error = f"{type(exc).__name__}: {exc}"
            self._retry_not_before = (
                time.monotonic() + self._RETRY_SECONDS if retryable else math.inf
            )
            self._pending_dx = 0
            self._pending_dy = 0

    def _run(self) -> None:
        try:
            response = self._request("status", {})
            data = response.get("data")
            supported_actions = (
                data.get("supported_actions") if isinstance(data, dict) else None
            )
            if (
                not isinstance(supported_actions, list)
                or not all(isinstance(item, str) for item in supported_actions)
                or not {"look_delta", "look_stop"}.issubset(supported_actions)
            ):
                self._record_error(
                    RuntimeError(
                        "engine input bridge does not advertise held look protocol"
                    ),
                    retryable=False,
                )
                return
        except Exception as exc:
            self._record_error(exc)
        else:
            self._record_success()
        while True:
            with self._condition:
                while (
                    not self._stop_requested
                    and self._pending_dx == 0
                    and self._pending_dy == 0
                    and not self._release_requested
                ):
                    self._condition.wait()
                stop_after_request = self._stop_requested
                release = bool(
                    self._release_requested
                    or (stop_after_request and self._drag_requested)
                )
                if release:
                    self._release_requested = False
                    self._drag_requested = False
                    self._pending_dx = 0
                    self._pending_dy = 0
                    dx = 0
                    dy = 0
                elif stop_after_request:
                    return
                else:
                    dx = self._pending_dx
                    dy = self._pending_dy
                    self._pending_dx = 0
                    self._pending_dy = 0
            if release:
                try:
                    self._request("look_stop", {})
                except Exception as exc:
                    self._record_error(exc)
                else:
                    with self._condition:
                        self.releases_emitted += 1
                    self._record_success()
                if stop_after_request:
                    return
                continue
            try:
                self._request(
                    "look_delta",
                    {"dx": dx, "dy": dy, "button": self._button},
                )
            except Exception as exc:
                self._record_error(exc)
            else:
                with self._condition:
                    self.emitted_batches += 1
                self._record_success()

    def submit(self, dx: int, dy: int) -> bool:
        if (
            isinstance(dx, bool)
            or isinstance(dy, bool)
            or type(dx) is not int
            or type(dy) is not int
        ):
            raise TypeError("engine camera deltas must be integers")
        if dx == 0 and dy == 0:
            return False
        with self._condition:
            if self._thread is None or self._stop_requested:
                self.dropped_batches += 1
                return False
            if self._capability_compatible is False:
                self.dropped_batches += 1
                return False
            if time.monotonic() < self._retry_not_before:
                self.dropped_batches += 1
                return False
            if self._pending_dx or self._pending_dy:
                self.coalesced_batches += 1
            self._drag_requested = True
            self._release_requested = False
            self._pending_dx = int(
                _clamp(
                    self._pending_dx + dx,
                    -self._MAX_PENDING_DELTA,
                    self._MAX_PENDING_DELTA,
                )
            )
            self._pending_dy = int(
                _clamp(
                    self._pending_dy + dy,
                    -self._MAX_PENDING_DELTA,
                    self._MAX_PENDING_DELTA,
                )
            )
            self.submitted_batches += 1
            self._condition.notify()
            return True

    def cancel_pending(self) -> bool:
        with self._condition:
            had_pending = bool(self._pending_dx or self._pending_dy)
            changed = bool(had_pending or self._drag_requested)
            self._pending_dx = 0
            self._pending_dy = 0
            if self._drag_requested:
                self._drag_requested = False
                self._release_requested = True
                self.release_requests += 1
            if had_pending:
                self.dropped_batches += 1
            if changed:
                self._condition.notify()
            return changed

    @property
    def telemetry(self) -> dict[str, object]:
        with self._condition:
            return {
                "configured": True,
                "available": self._available,
                "capability_compatible": self._capability_compatible,
                "status": self._status,
                "transport": "engine-held-relative-look-delta",
                "button": self._button,
                "endpoint": os.fspath(self._endpoint),
                "submitted_batches": self.submitted_batches,
                "coalesced_batches": self.coalesced_batches,
                "emitted_batches": self.emitted_batches,
                "dropped_batches": self.dropped_batches,
                "pending_dx": self._pending_dx,
                "pending_dy": self._pending_dy,
                "drag_requested": self._drag_requested,
                "release_pending": self._release_requested,
                "release_requests": self.release_requests,
                "releases_emitted": self.releases_emitted,
                "last_error": self._last_error,
            }

    def close(self) -> None:
        with self._condition:
            thread = self._thread
            if thread is None:
                return
            self._stop_requested = True
            self._pending_dx = 0
            self._pending_dy = 0
            self._condition.notify_all()
        thread.join(timeout=max(1.0, self._timeout_seconds * 4.0))
        if thread.is_alive():
            raise RuntimeError("engine camera look worker did not stop")
        with self._condition:
            self._thread = None
            self._available = False
            self._status = "stopped"


def keyboard_camera_telemetry(
    worker: EngineCameraLookWorker | None,
    integrator: KeyboardCameraLookIntegrator,
    *,
    arrow_keys_available: bool = True,
    rate_deg_s: float = 120.0,
) -> dict[str, object]:
    if type(arrow_keys_available) is not bool:
        raise TypeError("arrow key availability must be boolean")
    if worker is None:
        bridge: dict[str, object] = {
            "configured": False,
            "available": False,
            "capability_compatible": False,
            "status": "disabled",
            "transport": None,
            "button": None,
            "endpoint": None,
            "submitted_batches": 0,
            "coalesced_batches": 0,
            "emitted_batches": 0,
            "dropped_batches": 0,
            "pending_dx": 0,
            "pending_dy": 0,
            "last_error": "Matrix engine input bridge is not configured",
        }
    else:
        bridge = worker.telemetry
    if not arrow_keys_available:
        bridge = {
            **bridge,
            "available": False,
            "status": "unavailable",
            "last_error": "X11 keyboard map is missing one or more arrow keys",
        }
    return {
        **bridge,
        "arrow_keys_available": arrow_keys_available,
        "rate_deg_s": rate_deg_s,
        "rate_scope": "nominal_input_rate_not_final_pov_angular_velocity",
        "integrator": integrator.telemetry,
    }


def transform_camera_yaw(
    provider_yaw_rad: float, *, sign: int, offset_rad: float
) -> float:
    """Convert a provider yaw into SONIC's normalized command frame."""
    if sign not in {-1, 1}:
        raise ValueError("camera yaw sign must be -1 or 1")
    if not math.isfinite(provider_yaw_rad) or not math.isfinite(offset_rad):
        raise ValueError("camera yaw and offset must be finite")
    return wrap_angle_rad(sign * provider_yaw_rad + offset_rad)


def mirror_sensitivity_mapping(
    camera_yaw_source: str,
    *,
    base_deg_per_unit: float,
    effective_deg_per_unit: float,
) -> dict[str, object]:
    """Describe one source's gain without changing the applied value."""

    if camera_yaw_source in {"x11-mirror", "x11-core-gated"}:
        units = "degrees_per_xi2_raw_unit"
    elif camera_yaw_source == "x11-absolute":
        units = "degrees_per_x11_root_pixel"
    elif camera_yaw_source == "ue-final-pov":
        units = "absolute_degrees_from_player_camera_manager_final_pov"
    else:
        units = "degrees_per_unobserved_input_unit"
    return {
        "source": camera_yaw_source,
        "units": units,
        "base_deg_per_unit": base_deg_per_unit,
        "effective_deg_per_unit": effective_deg_per_unit,
        # Compatibility aliases retained for existing overlay/status readers.
        "base_deg_per_raw_unit": base_deg_per_unit,
        "effective_deg_per_raw_unit": effective_deg_per_unit,
        "base_deg_per_px": base_deg_per_unit,
        "effective_deg_per_px": effective_deg_per_unit,
    }


def camera_yaw_telemetry(
    source: str,
    *,
    provider_yaw_rad: float,
    sonic_yaw_rad: float,
) -> dict[str, object]:
    """Expose provider and transformed yaw without participating in control."""

    if not math.isfinite(provider_yaw_rad) or not math.isfinite(sonic_yaw_rad):
        raise ValueError("telemetry camera yaw must be finite")
    return {
        "source": source,
        "provider_yaw_rad": provider_yaw_rad,
        "provider_yaw_deg": math.degrees(provider_yaw_rad),
        "sonic_yaw_rad": sonic_yaw_rad,
        "sonic_yaw_deg": math.degrees(sonic_yaw_rad),
    }


def camera_source_claim(source: str) -> dict[str, object]:
    """Name an input-side camera claim without implying final-view truth."""

    claims = {
        "fixed": (
            "constant_unobserved",
            "configured_constant_not_final_view",
            "no_button_gate",
        ),
        "x11-mirror": (
            "xinput2_raw_motion_mirror",
            "xi2_raw_input_mirror_not_final_view",
            "xi2_raw_button_edges_same_slave_source",
        ),
        "x11-core-gated": (
            "xinput2_raw_motion_core_button_level_gate",
            "xi2_raw_motion_core_button_gate_not_final_view",
            "xquerypointer_core_button_level_sampled_not_event_ordered",
        ),
        "x11-absolute": (
            "xquerypointer_root_absolute_delta",
            "x11_absolute_pointer_delta_mirror_not_final_view",
            "xquerypointer_core_level_sampled_at_50hz",
        ),
        "ue-final-pov": (
            "ue_player_camera_manager_final_pov_state",
            "player_camera_manager_final_pov",
            "xquerypointer_core_level_or_xi2_raw_button_edges",
        ),
        "carla": (
            "carla_spectator_rpc_write_readback",
            "carla_spectator_not_verified_final_view",
            "not_applicable_carla_rpc",
        ),
    }
    try:
        observation, truth_scope, button_scope = claims[source]
    except KeyError as exc:
        raise ValueError(f"unsupported camera yaw source: {source}") from exc
    return {
        "camera_yaw_source": source,
        "camera_yaw_observation": observation,
        "camera_yaw_truth_scope": truth_scope,
        "button_gate_truth_scope": button_scope,
        "legacy": source == "x11-absolute",
        "experimental": source
        in {"x11-core-gated", "x11-absolute", "ue-final-pov"},
        # The source names UE's final PlayerCameraManager POV, but live visual
        # and cardinal-direction acceptance remains outstanding.
        "visible_follow_camera_verified": False,
    }


def initial_sequence(clock: Callable[[], int] = time.monotonic_ns) -> int:
    """Choose a restart-safe starting sequence on this same Linux host."""
    value = clock()
    if isinstance(value, bool) or not isinstance(value, int):
        raise RuntimeError("monotonic_ns returned a non-integer sequence")
    if not 0 <= value <= (2**63 - 1):
        raise RuntimeError("monotonic_ns is outside the input protocol range")
    return value


class CameraYawReader(Protocol):
    def read(self, now: float) -> float | None: ...


@dataclass(frozen=True)
class UeFinalPovObservation:
    """One fail-closed final-POV observation used by the input loop.

    ``angles_changed`` is diagnostic only.  A centered third-person camera can
    rotate with the robot even when the operator is not touching the mouse, so
    final-POV motion alone is not evidence of an active drag.  The X11/XI2
    button-boundary observer owns the locomotion interlock.
    """

    yaw_rad: float | None
    error: str | None
    angles_changed: bool = False
    max_angle_delta_deg: float = 0.0
    sequence: int | None = None
    sample_age_ms: float | None = None
    pitch_deg: float | None = None
    roll_deg: float | None = None
    cache_timestamp_s: float | None = None


def ue_final_pov_telemetry(
    observation: UeFinalPovObservation | None,
) -> dict[str, object]:
    """Expose live probe health without feeding diagnostics back into control."""

    if observation is None:
        return {
            "available": False,
            "error": "not_sampled",
            "sequence": None,
            "sample_age_ms": None,
            "provider_yaw_deg": None,
            "pitch_deg": None,
            "roll_deg": None,
            "cache_timestamp_s": None,
            "angles_changed": False,
            "max_angle_delta_deg": 0.0,
        }
    return {
        "available": observation.yaw_rad is not None,
        "error": observation.error,
        "sequence": observation.sequence,
        "sample_age_ms": observation.sample_age_ms,
        "provider_yaw_deg": (
            math.degrees(observation.yaw_rad)
            if observation.yaw_rad is not None
            else None
        ),
        "pitch_deg": observation.pitch_deg,
        "roll_deg": observation.roll_deg,
        "cache_timestamp_s": observation.cache_timestamp_s,
        "angles_changed": observation.angles_changed,
        "max_angle_delta_deg": observation.max_angle_delta_deg,
    }


class UeFinalPovYawReader:
    """Adapt the supervised UE final-POV state into a safe yaw observation.

    ``CameraStateReader`` owns file integrity, freshness, sequence and exact UE
    PID validation.  This adapter deliberately does not infer mouse-button
    state from camera motion: robot-follow rotation changes the final POV too.
    Missing/stale state still fails closed through ``camera_available=False``;
    actual press/drag/release boundaries are observed independently by XI2.
    """

    def __init__(
        self,
        state_file: Path,
        *,
        expected_ue_pid: int,
        reader: Any | None = None,
    ) -> None:
        if reader is None:
            module = importlib.import_module("matrix_ue_camera_probe")
            reader = module.CameraStateReader(
                state_file,
                expected_ue_pid=expected_ue_pid,
            )
        self._reader = reader

    @property
    def last_error(self) -> str | None:
        value = getattr(self._reader, "last_error", None)
        return value if isinstance(value, str) else None

    def read(self, now: float) -> UeFinalPovObservation:
        if not math.isfinite(now) or now < 0.0:
            raise ValueError("final-POV read time must be finite and non-negative")
        # CameraStateReader owns the read/clock linearization point.  Passing
        # this input frame's earlier timestamp creates a TOCTOU race when the
        # supervisor publishes a valid state between the X11 poll and pread.
        state = self._reader.read()
        if state is None:
            return UeFinalPovObservation(
                yaw_rad=None,
                error=self.last_error,
            )
        yaw_deg = float(state.yaw_deg)
        if not math.isfinite(yaw_deg):
            return UeFinalPovObservation(
                yaw_rad=None,
                error="non_finite_yaw",
            )
        max_angle_delta_deg = float(
            getattr(self._reader, "max_angle_delta_deg", 0.0)
        )
        if not math.isfinite(max_angle_delta_deg) or max_angle_delta_deg < 0.0:
            max_angle_delta_deg = 0.0
        state_monotonic_ns = getattr(state, "monotonic_ns", None)
        sample_age_ms: float | None = None
        now_ns = int(now * 1_000_000_000)
        if (
            isinstance(state_monotonic_ns, int)
            and not isinstance(state_monotonic_ns, bool)
            and 0 < state_monotonic_ns <= now_ns
        ):
            sample_age_ms = (now_ns - state_monotonic_ns) / 1_000_000.0

        def finite_optional(name: str) -> float | None:
            value = getattr(state, name, None)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                return None
            result = float(value)
            return result if math.isfinite(result) else None

        sequence = getattr(state, "sequence", None)
        if (
            isinstance(sequence, bool)
            or not isinstance(sequence, int)
            or sequence <= 0
        ):
            sequence = None
        return UeFinalPovObservation(
            yaw_rad=math.radians(yaw_deg),
            error=None,
            angles_changed=bool(getattr(self._reader, "angles_changed", False)),
            max_angle_delta_deg=max_angle_delta_deg,
            sequence=sequence,
            sample_age_ms=sample_age_ms,
            pitch_deg=finite_optional("pitch_deg"),
            roll_deg=finite_optional("roll_deg"),
            cache_timestamp_s=finite_optional("cache_timestamp_s"),
        )


class CarlaSpectatorYawReader:
    """Read and, when requested, rotate a CARLA spectator camera.

    Packaged Matrix maps do not all couple the visible follow camera to CARLA's
    spectator.  ``--camera-yaw-source carla`` is therefore safe-by-default:
    connection, write, or immediate read-back failure marks snapshots unfocused
    and the core stops.  Coupling to the rendered camera must still be proven by
    the runtime camera probe before acceptance.
    """

    def __init__(
        self,
        host: str,
        port: int,
        *,
        timeout_seconds: float = 0.1,
        retry_seconds: float = 1.0,
        poll_seconds: float = 0.02,
        look_yaw_rate_rad_s: float = math.radians(120.0),
        look_pitch_rate_rad_s: float = math.radians(90.0),
        look_deadzone: float = 0.12,
        minimum_pitch_rad: float = math.radians(-80.0),
        maximum_pitch_rad: float = math.radians(60.0),
        write_readback_tolerance_rad: float = (
            DEFAULT_CARLA_WRITE_READBACK_TOLERANCE_RAD
        ),
    ) -> None:
        for name, value in (
            ("timeout_seconds", timeout_seconds),
            ("retry_seconds", retry_seconds),
            ("poll_seconds", poll_seconds),
            ("look_yaw_rate_rad_s", look_yaw_rate_rad_s),
            ("look_pitch_rate_rad_s", look_pitch_rate_rad_s),
            ("write_readback_tolerance_rad", write_readback_tolerance_rad),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")
        if not math.isfinite(look_deadzone) or not 0.0 <= look_deadzone < 1.0:
            raise ValueError("look_deadzone must be finite and in [0, 1)")
        if (
            not math.isfinite(minimum_pitch_rad)
            or not math.isfinite(maximum_pitch_rad)
            or minimum_pitch_rad >= maximum_pitch_rad
        ):
            raise ValueError("camera pitch limits must be finite and ordered")
        self._host = host
        self._port = port
        self._timeout = timeout_seconds
        self._retry = retry_seconds
        self._poll = poll_seconds
        self._look_yaw_rate = look_yaw_rate_rad_s
        self._look_pitch_rate = look_pitch_rate_rad_s
        self._look_deadzone = look_deadzone
        self._minimum_pitch = minimum_pitch_rad
        self._maximum_pitch = maximum_pitch_rad
        self._write_readback_tolerance = write_readback_tolerance_rad
        self._client: Any | None = None
        self._world: Any | None = None
        self._next_connect = 0.0
        self._next_poll = 0.0
        self._last_yaw: float | None = None

    def _connect(self) -> None:
        carla = importlib.import_module("carla")
        client = carla.Client(self._host, self._port)
        client.set_timeout(self._timeout)
        self._world = client.get_world()
        self._client = client

    def _disconnect(self, now: float) -> None:
        self._client = None
        self._world = None
        self._next_connect = now + self._retry
        self._last_yaw = None

    def _ensure_connected(self, now: float) -> bool:
        if self._world is None and now >= self._next_connect:
            try:
                self._connect()
            except Exception:
                self._disconnect(now)
        return self._world is not None

    def _record_transform(self, transform: Any) -> float:
        yaw_degrees = float(transform.rotation.yaw)
        if not math.isfinite(yaw_degrees):
            raise ValueError("CARLA returned non-finite camera yaw")
        self._last_yaw = wrap_angle_rad(math.radians(yaw_degrees))
        return self._last_yaw

    def read(self, now: float) -> float | None:
        if not self._ensure_connected(now):
            return None
        if self._world is None or now < self._next_poll:
            return self._last_yaw
        self._next_poll = now + self._poll
        try:
            transform = self._world.get_spectator().get_transform()
            self._record_transform(transform)
        except Exception:
            self._disconnect(now)
        return self._last_yaw

    def drive(
        self,
        *,
        now: float,
        dt: float,
        look_yaw: float,
        look_pitch: float,
    ) -> float | None:
        """Apply right-stick yaw/pitch and return an immediate yaw read-back.

        A zero look vector is a read-only poll.  A non-zero vector is written to
        CARLA's spectator and then queried again; the commanded angle itself is
        never accepted as camera truth.
        """

        if not math.isfinite(now) or not math.isfinite(dt) or dt < 0.0:
            raise ValueError("camera drive time values must be finite and non-negative")
        if not math.isfinite(look_yaw) or not math.isfinite(look_pitch):
            raise ValueError("camera look axes must be finite")
        yaw_axis, pitch_axis = apply_radial_deadzone(
            right=_clamp(look_yaw, -1.0, 1.0),
            forward=_clamp(look_pitch, -1.0, 1.0),
            deadzone=self._look_deadzone,
        )
        if math.hypot(yaw_axis, pitch_axis) <= 1e-12:
            return self.read(now)
        if not self._ensure_connected(now):
            return None
        assert self._world is not None
        try:
            spectator = self._world.get_spectator()
            transform = spectator.get_transform()
            current_yaw = float(transform.rotation.yaw)
            current_pitch = float(transform.rotation.pitch)
            if not math.isfinite(current_yaw) or not math.isfinite(current_pitch):
                raise ValueError("CARLA returned a non-finite camera rotation")
            transform.rotation.yaw = current_yaw + math.degrees(
                yaw_axis * self._look_yaw_rate * dt
            )
            next_pitch = math.radians(current_pitch) + (
                pitch_axis * self._look_pitch_rate * dt
            )
            transform.rotation.pitch = math.degrees(
                _clamp(next_pitch, self._minimum_pitch, self._maximum_pitch)
            )
            target_yaw = wrap_angle_rad(math.radians(transform.rotation.yaw))
            target_pitch = math.radians(transform.rotation.pitch)
            spectator.set_transform(transform)
            # Read back from CARLA after every write.  If the RPC endpoint rejects
            # or fails to retain the transform, this frame disarms locomotion.
            observed = spectator.get_transform()
            observed_yaw_degrees = float(observed.rotation.yaw)
            observed_pitch_degrees = float(observed.rotation.pitch)
            if not math.isfinite(observed_yaw_degrees) or not math.isfinite(
                observed_pitch_degrees
            ):
                raise ValueError("CARLA returned a non-finite camera rotation")
            observed_yaw = wrap_angle_rad(math.radians(observed_yaw_degrees))
            observed_pitch = math.radians(observed_pitch_degrees)
            if (
                abs(wrap_angle_rad(observed_yaw - target_yaw))
                > self._write_readback_tolerance
                or abs(observed_pitch - target_pitch)
                > self._write_readback_tolerance
            ):
                raise RuntimeError("CARLA spectator did not retain camera rotation")
            yaw = self._record_transform(observed)
            self._next_poll = now + self._poll
            return yaw
        except Exception:
            self._disconnect(now)
            return None


_X11_GENERIC_EVENT = 35
_XI_ALL_DEVICES = 0
_XI_ALL_MASTER_DEVICES = 1
_XI_MASTER_POINTER = 1
_XI_HIERARCHY_CHANGED = 11
_XI_RAW_BUTTON_PRESS = 15
_XI_RAW_BUTTON_RELEASE = 16
_XI_RAW_MOTION = 17
_MAX_XI2_EVENTS_PER_POLL = 4096


class _XGenericEventCookie(ctypes.Structure):
    _fields_ = (
        ("type", ctypes.c_int),
        ("serial", ctypes.c_ulong),
        ("send_event", ctypes.c_int),
        ("display", ctypes.c_void_p),
        ("extension", ctypes.c_int),
        ("evtype", ctypes.c_int),
        ("cookie", ctypes.c_uint),
        ("data", ctypes.c_void_p),
    )


class _XEvent(ctypes.Union):
    # Xlib guarantees that XEvent is 24 longs on every supported ABI.
    _fields_ = (
        ("type", ctypes.c_int),
        ("xkey", _XKeyEvent),
        ("pad", ctypes.c_long * 24),
    )


class _XIEventMask(ctypes.Structure):
    _fields_ = (
        ("deviceid", ctypes.c_int),
        ("mask_len", ctypes.c_int),
        ("mask", ctypes.POINTER(ctypes.c_ubyte)),
    )


class _XIDeviceInfo(ctypes.Structure):
    # Public XInput2 ABI from XInput2.h.  ``classes`` is opaque here because
    # master selection only needs the fixed fields which precede it.
    _fields_ = (
        ("deviceid", ctypes.c_int),
        ("name", ctypes.c_char_p),
        ("use", ctypes.c_int),
        ("attachment", ctypes.c_int),
        ("enabled", ctypes.c_int),
        ("num_classes", ctypes.c_int),
        ("classes", ctypes.c_void_p),
    )


class _XIValuatorState(ctypes.Structure):
    _fields_ = (
        ("mask_len", ctypes.c_int),
        ("mask", ctypes.POINTER(ctypes.c_ubyte)),
        ("values", ctypes.POINTER(ctypes.c_double)),
    )


class _XIRawEvent(ctypes.Structure):
    _fields_ = (
        ("type", ctypes.c_int),
        ("serial", ctypes.c_ulong),
        ("send_event", ctypes.c_int),
        ("display", ctypes.c_void_p),
        ("extension", ctypes.c_int),
        ("evtype", ctypes.c_int),
        ("time", ctypes.c_ulong),
        ("deviceid", ctypes.c_int),
        ("sourceid", ctypes.c_int),
        ("detail", ctypes.c_int),
        ("flags", ctypes.c_int),
        ("valuators", _XIValuatorState),
        ("raw_values", ctypes.POINTER(ctypes.c_double)),
    )


@dataclass(frozen=True)
class XInput2RawEvent:
    evtype: int
    deviceid: int = 0
    sourceid: int = 0
    detail: int = 0
    dx: float = 0.0
    dy: float = 0.0


def decode_xinput2_xy(mask: bytes, values: tuple[float, ...]) -> tuple[float, float]:
    """Decode XInput2's packed valuators, retaining only raw X/Y axes."""

    if not isinstance(mask, bytes) or not mask:
        raise RuntimeError("XI2 raw motion has an invalid valuator mask")
    expected_values = sum(byte.bit_count() for byte in mask)
    if expected_values != len(values):
        raise RuntimeError("XI2 raw motion valuator mask/value count differs")
    x = 0.0
    y = 0.0
    packed_index = 0
    for axis in range(len(mask) * 8):
        if not mask[axis >> 3] & (1 << (axis & 7)):
            continue
        value = float(values[packed_index])
        packed_index += 1
        if not math.isfinite(value):
            raise RuntimeError("XI2 raw motion contains a non-finite valuator")
        if axis == 0:
            x = value
        elif axis == 1:
            y = value
    return (x, y)


class XInput2DragAccumulator:
    """Attribute raw motion to one fresh, same-source look-button hold."""

    def __init__(self, look_button_detail: int) -> None:
        if look_button_detail not in {1, 2, 3}:
            raise ValueError("XI2 look button detail must be 1, 2, or 3")
        self._look_button_detail = look_button_detail
        self._pressed_sourceid: int | None = None
        self._requires_release = True
        self.button_state_resyncs = 0
        self.last_drop_reason: str | None = None

    def disarm(self) -> None:
        """Require a core-button release and a subsequent fresh raw press."""

        self._pressed_sourceid = None
        self._requires_release = True

    def update(
        self,
        events: tuple[XInput2RawEvent, ...],
        *,
        current_look_pressed: bool,
    ) -> tuple[float, float, bool]:
        if type(current_look_pressed) is not bool:
            raise ValueError("current XI2 look-button state must be boolean")
        self.last_drop_reason = None
        if self._requires_release:
            # Startup, focus loss, and topology changes all lose a trustworthy
            # per-source button boundary.  Never infer one from the master
            # pointer mask.  A captured raw press is a fresh boundary; without
            # one, first observe the combined core button released.
            fresh_press_index = next(
                (
                    index
                    for index, event in enumerate(events)
                    if event.evtype == _XI_RAW_BUTTON_PRESS
                    and event.detail == self._look_button_detail
                ),
                None,
            )
            if fresh_press_index is None:
                if not current_look_pressed:
                    self._requires_release = False
                elif any(event.evtype == _XI_RAW_MOTION for event in events):
                    self.last_drop_reason = "awaiting_xi2_release_or_fresh_press"
                return (0.0, 0.0, False)
            self._requires_release = False
            events = events[fresh_press_index:]

        dx = 0.0
        dy = 0.0
        drag_observed = self._pressed_sourceid is not None
        for event in events:
            if not isinstance(event, XInput2RawEvent):
                raise TypeError("XI2 event must be XInput2RawEvent")
            if event.evtype in {
                _XI_RAW_BUTTON_PRESS,
                _XI_RAW_BUTTON_RELEASE,
                _XI_RAW_MOTION,
            } and (event.deviceid <= 0 or event.sourceid <= 0):
                raise RuntimeError("XI2 raw event has an invalid device identity")
            if (
                event.evtype == _XI_RAW_BUTTON_PRESS
                and event.detail == self._look_button_detail
            ):
                if self._pressed_sourceid not in {None, event.sourceid}:
                    raise RuntimeError("XI2 look button crossed input sources")
                self._pressed_sourceid = event.sourceid
                drag_observed = True
            elif (
                event.evtype == _XI_RAW_BUTTON_RELEASE
                and event.detail == self._look_button_detail
            ):
                if self._pressed_sourceid not in {None, event.sourceid}:
                    raise RuntimeError("XI2 look-button release crossed input sources")
                if self._pressed_sourceid == event.sourceid:
                    self._pressed_sourceid = None
            elif event.evtype == _XI_RAW_MOTION and self._pressed_sourceid is not None:
                if event.sourceid != self._pressed_sourceid:
                    raise RuntimeError("XI2 drag motion crossed input sources")
                if not math.isfinite(event.dx) or not math.isfinite(event.dy):
                    raise RuntimeError("XI2 raw motion contains a non-finite delta")
                dx += event.dx
                dy += event.dy

        if (self._pressed_sourceid is not None) != current_look_pressed:
            self.button_state_resyncs += 1
            self.last_drop_reason = "xi2_button_state_resync"
            self.disarm()
            return (0.0, 0.0, True)
        return (dx, dy, drag_observed)


class XInput2CoreGatedAccumulator:
    """Gate XI2 raw deltas with stable XQueryPointer core-button levels.

    Only a poll interval whose previous and current core levels are both held
    may contribute yaw.  Press/release boundary batches are deliberately
    dropped and still report a drag interlock.  This experimental attribution
    loses at most the boundary portions of a drag, but it never treats raw
    motion observed while the core button is released as camera yaw.
    """

    def __init__(self, look_button_detail: int) -> None:
        if look_button_detail not in {1, 2, 3}:
            raise ValueError("XI2 look button detail must be 1, 2, or 3")
        self._look_button_detail = look_button_detail
        self._previous_core_pressed = False
        self._requires_release = True
        self._bound_sourceid: int | None = None
        self.button_state_resyncs = 0
        self.last_drop_reason: str | None = None
        self.drop_reason_counts: dict[str, int] = {}
        self.ambiguous_raw_motion_events = 0
        self.ambiguous_raw_dx_total = 0.0
        self.ambiguous_raw_dy_total = 0.0
        self.source_bindings = 0
        self.source_rejections = 0

    def disarm(self) -> None:
        self._previous_core_pressed = False
        self._requires_release = True
        self._bound_sourceid = None

    @property
    def bound_sourceid(self) -> int | None:
        return self._bound_sourceid

    def _bind_or_reject(
        self,
        sourceids: set[int],
        motion_events: tuple[XInput2RawEvent, ...],
    ) -> bool:
        """Bind one slave source for a fresh hold; reject any source change."""

        if len(sourceids) > 1:
            self.source_rejections += 1
            self._drop("multiple_slave_sources", motion_events)
            self.disarm()
            return False
        if not sourceids:
            return True
        sourceid = next(iter(sourceids))
        if self._bound_sourceid is None:
            self._bound_sourceid = sourceid
            self.source_bindings += 1
            return True
        if sourceid != self._bound_sourceid:
            self.source_rejections += 1
            self._drop("slave_source_changed", motion_events)
            self.disarm()
            return False
        return True

    def _drop(
        self,
        reason: str,
        motion_events: tuple[XInput2RawEvent, ...],
    ) -> None:
        self.last_drop_reason = reason
        self.drop_reason_counts[reason] = self.drop_reason_counts.get(reason, 0) + 1
        self.ambiguous_raw_motion_events += len(motion_events)
        self.ambiguous_raw_dx_total += sum(event.dx for event in motion_events)
        self.ambiguous_raw_dy_total += sum(event.dy for event in motion_events)

    def update(
        self,
        events: tuple[XInput2RawEvent, ...],
        *,
        current_look_pressed: bool,
    ) -> tuple[float, float, bool]:
        if type(current_look_pressed) is not bool:
            raise ValueError("current core look-button state must be boolean")
        self.last_drop_reason = None
        for event in events:
            if not isinstance(event, XInput2RawEvent):
                raise TypeError("XI2 event must be XInput2RawEvent")
            if event.evtype in {
                _XI_RAW_BUTTON_PRESS,
                _XI_RAW_BUTTON_RELEASE,
                _XI_RAW_MOTION,
            } and (event.deviceid <= 0 or event.sourceid <= 0):
                raise RuntimeError("XI2 raw event has an invalid device identity")
            if event.evtype == _XI_RAW_MOTION and (
                not math.isfinite(event.dx) or not math.isfinite(event.dy)
            ):
                raise RuntimeError("XI2 raw motion contains a non-finite delta")

        motion_events = tuple(
            event for event in events if event.evtype == _XI_RAW_MOTION
        )
        look_edges = tuple(
            event
            for event in events
            if event.evtype in {_XI_RAW_BUTTON_PRESS, _XI_RAW_BUTTON_RELEASE}
            and event.detail == self._look_button_detail
        )
        raw_dx = sum(event.dx for event in motion_events)
        raw_dy = sum(event.dy for event in motion_events)
        batch_sourceids = {
            event.sourceid for event in (*motion_events, *look_edges)
        }

        if self._requires_release:
            if current_look_pressed:
                if motion_events or look_edges:
                    self._drop("awaiting_core_release", motion_events)
                return (0.0, 0.0, True)
            self._requires_release = False
            self._previous_core_pressed = False
            if look_edges:
                self._drop("quick_drag_while_rearming", motion_events)
                return (0.0, 0.0, True)
            if motion_events:
                self._drop("core_released", motion_events)
            return (0.0, 0.0, False)

        previous_pressed = self._previous_core_pressed
        self._previous_core_pressed = current_look_pressed
        if previous_pressed and current_look_pressed:
            if look_edges:
                if not self._bind_or_reject(batch_sourceids, motion_events):
                    return (0.0, 0.0, True)
                self._drop("raw_button_edge_inside_stable_core_hold", motion_events)
                return (0.0, 0.0, True)
            if not self._bind_or_reject(batch_sourceids, motion_events):
                return (0.0, 0.0, True)
            return (raw_dx, raw_dy, True)
        if not previous_pressed and current_look_pressed:
            if not self._bind_or_reject(batch_sourceids, motion_events):
                return (0.0, 0.0, True)
            if motion_events:
                self._drop("core_press_boundary", motion_events)
            return (0.0, 0.0, True)
        if previous_pressed and not current_look_pressed:
            if (
                self._bound_sourceid is not None
                and batch_sourceids
                and batch_sourceids != {self._bound_sourceid}
            ):
                self.source_rejections += 1
                self._drop("slave_source_changed_on_release", motion_events)
                self.disarm()
                return (0.0, 0.0, True)
            if motion_events:
                self._drop("core_release_boundary", motion_events)
            self._bound_sourceid = None
            return (0.0, 0.0, True)
        if look_edges:
            self._drop("quick_press_drag_release", motion_events)
            return (0.0, 0.0, True)
        if motion_events:
            self._drop("core_released", motion_events)
        return (0.0, 0.0, False)


class XInput2RawMotion:
    """Mirror XI_RawMotion commonly used by SDL relative mouse mode.

    This is an input-side observation, not a readback of the final rendered
    UE camera.  To avoid attributing one operator's movement to another
    master pointer, capture is supported only while the X server exposes
    exactly one master pointer.
    """

    _BUTTON_DETAIL = {"left": 1, "middle": 2, "right": 3}

    def __init__(
        self,
        *,
        display_name: str | None,
        look_button: str,
        button_gate: str = "xi2-events",
        x11_library: Any | None = None,
        xi_library: Any | None = None,
    ) -> None:
        self._display: Any | None = None
        if look_button not in self._BUTTON_DETAIL:
            raise ValueError(f"unsupported XI2 look button: {look_button}")
        if button_gate not in {"xi2-events", "x11-core-level"}:
            raise ValueError(f"unsupported XI2 button gate: {button_gate}")
        if x11_library is None:
            x11_name = ctypes.util.find_library("X11")
            if not x11_name:
                raise RuntimeError("libX11 was not found for XI2 raw motion")
            x11_library = ctypes.CDLL(x11_name)
        if xi_library is None:
            xi_name = ctypes.util.find_library("Xi")
            if not xi_name:
                raise RuntimeError("libXi was not found for XI2 raw motion")
            xi_library = ctypes.CDLL(xi_name)
        self._x11 = x11_library
        self._xi = xi_library
        self._configure_signatures()
        encoded_display = display_name.encode() if display_name else None
        self._display = self._x11.XOpenDisplay(encoded_display)
        if not self._display:
            label = display_name or os.environ.get("DISPLAY", "<unset>")
            raise RuntimeError(f"cannot open XI2 raw-motion display {label}")
        try:
            opcode = ctypes.c_int()
            first_event = ctypes.c_int()
            first_error = ctypes.c_int()
            if not self._x11.XQueryExtension(
                self._display,
                b"XInputExtension",
                ctypes.byref(opcode),
                ctypes.byref(first_event),
                ctypes.byref(first_error),
            ):
                raise RuntimeError("XInputExtension is unavailable")
            major = ctypes.c_int(2)
            minor = ctypes.c_int(0)
            if self._xi.XIQueryVersion(
                self._display, ctypes.byref(major), ctypes.byref(minor)
            ) != 0 or (major.value, minor.value) < (2, 0):
                raise RuntimeError("XInput2 2.0 or newer is required")
            self._extension_opcode = opcode.value
            self._negotiated_version = (major.value, minor.value)
            self._root = int(self._x11.XDefaultRootWindow(self._display))
            self._raw_mask_buffer = self._mask_buffer(
                _XI_RAW_BUTTON_PRESS,
                _XI_RAW_BUTTON_RELEASE,
                _XI_RAW_MOTION,
            )
            self._hierarchy_mask_buffer = self._mask_buffer(
                _XI_HIERARCHY_CHANGED
            )
            self._master_deviceid = self._single_master_pointer_deviceid()
            self._subscribe_raw_masters()
            self._subscribe_hierarchy()
            self._x11.XFlush(self._display)
        except Exception:
            self.close()
            raise
        self._button_gate = button_gate
        accumulator_type = (
            XInput2CoreGatedAccumulator
            if button_gate == "x11-core-level"
            else XInput2DragAccumulator
        )
        self._accumulator = accumulator_type(self._BUTTON_DETAIL[look_button])
        self.events_consumed = 0
        self.raw_motion_events = 0
        self.hierarchy_events = 0
        self.foreign_master_events = 0
        self.master_device_changes = 0
        self.accepted_dx_total = 0.0
        self.accepted_dy_total = 0.0
        self.last_accepted_dx = 0.0
        self.last_accepted_dy = 0.0
        self.drag_batches = 0
        self.accepted_drag_batches = 0
        self.dropped_batches = 0
        self.dropped_motion_events = 0
        self.dropped_dx_total = 0.0
        self.dropped_dy_total = 0.0
        self.drop_reason_counts: dict[str, int] = {}
        self.last_drop_reasons: tuple[str, ...] = ()

    def _ensure_telemetry_counters(self) -> None:
        """Initialize counters for legacy unit-test fakes made via __new__."""

        defaults: dict[str, object] = {
            "accepted_dx_total": 0.0,
            "accepted_dy_total": 0.0,
            "last_accepted_dx": 0.0,
            "last_accepted_dy": 0.0,
            "drag_batches": 0,
            "accepted_drag_batches": 0,
            "dropped_batches": 0,
            "dropped_motion_events": 0,
            "dropped_dx_total": 0.0,
            "dropped_dy_total": 0.0,
            "drop_reason_counts": {},
            "last_drop_reasons": (),
            "_button_gate": "xi2-events",
        }
        for name, value in defaults.items():
            if not hasattr(self, name):
                setattr(self, name, value.copy() if isinstance(value, dict) else value)

    def _record_drop(
        self,
        *reasons: str,
        motion_events: tuple[XInput2RawEvent, ...] = (),
    ) -> None:
        self._ensure_telemetry_counters()
        unique_reasons = tuple(dict.fromkeys(reason for reason in reasons if reason))
        if not unique_reasons:
            return
        self.dropped_batches += 1
        self.dropped_motion_events += len(motion_events)
        self.dropped_dx_total += sum(event.dx for event in motion_events)
        self.dropped_dy_total += sum(event.dy for event in motion_events)
        self.last_drop_reasons = unique_reasons
        for reason in unique_reasons:
            self.drop_reason_counts[reason] = self.drop_reason_counts.get(reason, 0) + 1

    def _record_result(
        self,
        dx: float,
        dy: float,
        drag_observed: bool,
        *,
        motion_events: tuple[XInput2RawEvent, ...],
    ) -> None:
        self._ensure_telemetry_counters()
        accumulator_reason = getattr(self._accumulator, "last_drop_reason", None)
        if drag_observed:
            self.drag_batches += 1
        if accumulator_reason is not None:
            self._record_drop(
                accumulator_reason,
                motion_events=motion_events,
            )
            return
        self.last_drop_reasons = ()
        if drag_observed:
            self.accepted_drag_batches += 1
            self.accepted_dx_total += dx
            self.accepted_dy_total += dy
            self.last_accepted_dx = dx
            self.last_accepted_dy = dy

    @staticmethod
    def _mask_buffer(*event_types: int) -> Any:
        mask_length = (max(event_types) >> 3) + 1
        buffer = (ctypes.c_ubyte * mask_length)()
        for event_type in event_types:
            buffer[event_type >> 3] |= 1 << (event_type & 7)
        return buffer

    def _select_mask(self, *, deviceid: int, buffer: Any) -> None:
        mask = _XIEventMask(
            deviceid=deviceid,
            mask_len=len(buffer),
            mask=ctypes.cast(buffer, ctypes.POINTER(ctypes.c_ubyte)),
        )
        if self._xi.XISelectEvents(
            self._display, self._root, ctypes.byref(mask), 1
        ) != 0:
            raise RuntimeError("XISelectEvents rejected raw-motion subscription")

    def _single_master_pointer_deviceid(self) -> int:
        count = ctypes.c_int()
        devices = self._xi.XIQueryDevice(
            self._display,
            _XI_ALL_MASTER_DEVICES,
            ctypes.byref(count),
        )
        try:
            if count.value < 0 or count.value > 256:
                raise RuntimeError("XIQueryDevice returned an invalid device count")
            if count.value and not devices:
                raise RuntimeError("XIQueryDevice omitted its device array")
            masters = tuple(
                devices[index]
                for index in range(count.value)
                if int(devices[index].use) == _XI_MASTER_POINTER
            )
            if len(masters) != 1:
                raise RuntimeError(
                    "XI2 raw capture requires exactly one master pointer"
                )
            master = masters[0]
            if int(master.deviceid) <= 1 or not bool(master.enabled):
                raise RuntimeError(
                    "XI2 raw capture requires one enabled master pointer"
                )
            return int(master.deviceid)
        finally:
            if devices:
                self._xi.XIFreeDeviceInfo(devices)

    def _subscribe_raw_masters(self) -> None:
        self._select_mask(
            deviceid=_XI_ALL_MASTER_DEVICES,
            buffer=self._raw_mask_buffer,
        )

    def _subscribe_hierarchy(self) -> None:
        self._select_mask(
            deviceid=_XI_ALL_DEVICES,
            buffer=self._hierarchy_mask_buffer,
        )

    def _configure_signatures(self) -> None:
        signatures = {
            "XOpenDisplay": ([ctypes.c_char_p], ctypes.c_void_p),
            "XDefaultRootWindow": ([ctypes.c_void_p], ctypes.c_ulong),
            "XQueryExtension": (
                [
                    ctypes.c_void_p,
                    ctypes.c_char_p,
                    ctypes.POINTER(ctypes.c_int),
                    ctypes.POINTER(ctypes.c_int),
                    ctypes.POINTER(ctypes.c_int),
                ],
                ctypes.c_int,
            ),
            "XPending": ([ctypes.c_void_p], ctypes.c_int),
            "XNextEvent": (
                [ctypes.c_void_p, ctypes.POINTER(_XEvent)],
                ctypes.c_int,
            ),
            "XGetEventData": (
                [ctypes.c_void_p, ctypes.POINTER(_XGenericEventCookie)],
                ctypes.c_int,
            ),
            "XFreeEventData": (
                [ctypes.c_void_p, ctypes.POINTER(_XGenericEventCookie)],
                None,
            ),
            "XFlush": ([ctypes.c_void_p], ctypes.c_int),
            "XCloseDisplay": ([ctypes.c_void_p], ctypes.c_int),
        }
        for name, (argtypes, restype) in signatures.items():
            function = getattr(self._x11, name)
            function.argtypes = argtypes
            function.restype = restype
        self._xi.XIQueryVersion.argtypes = (
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_int),
        )
        self._xi.XIQueryVersion.restype = ctypes.c_int
        self._xi.XIQueryDevice.argtypes = (
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_int),
        )
        self._xi.XIQueryDevice.restype = ctypes.POINTER(_XIDeviceInfo)
        self._xi.XIFreeDeviceInfo.argtypes = (
            ctypes.POINTER(_XIDeviceInfo),
        )
        self._xi.XIFreeDeviceInfo.restype = None
        self._xi.XISelectEvents.argtypes = (
            ctypes.c_void_p,
            ctypes.c_ulong,
            ctypes.POINTER(_XIEventMask),
            ctypes.c_int,
        )
        self._xi.XISelectEvents.restype = ctypes.c_int

    @staticmethod
    def _motion_event(raw: _XIRawEvent) -> XInput2RawEvent:
        mask_length = int(raw.valuators.mask_len)
        if not 1 <= mask_length <= 64 or not raw.valuators.mask:
            raise RuntimeError("XI2 raw motion has an invalid valuator mask")
        mask = bytes(raw.valuators.mask[index] for index in range(mask_length))
        value_count = sum(byte.bit_count() for byte in mask)
        if value_count and not raw.raw_values:
            raise RuntimeError("XI2 raw motion omitted packed valuator values")
        values = tuple(float(raw.raw_values[index]) for index in range(value_count))
        dx, dy = decode_xinput2_xy(mask, values)
        return XInput2RawEvent(
            evtype=_XI_RAW_MOTION,
            deviceid=int(raw.deviceid),
            sourceid=int(raw.sourceid),
            dx=dx,
            dy=dy,
        )

    def _read_event(self) -> XInput2RawEvent | None:
        event = _XEvent()
        self._x11.XNextEvent(self._display, ctypes.byref(event))
        cookie = ctypes.cast(
            ctypes.byref(event), ctypes.POINTER(_XGenericEventCookie)
        ).contents
        if (
            cookie.type != _X11_GENERIC_EVENT
            or cookie.extension != self._extension_opcode
            or cookie.evtype
            not in {
                _XI_HIERARCHY_CHANGED,
                _XI_RAW_BUTTON_PRESS,
                _XI_RAW_BUTTON_RELEASE,
                _XI_RAW_MOTION,
            }
        ):
            return None
        if cookie.evtype == _XI_HIERARCHY_CHANGED:
            return XInput2RawEvent(evtype=_XI_HIERARCHY_CHANGED)
        if not self._x11.XGetEventData(self._display, ctypes.byref(cookie)):
            raise RuntimeError("XGetEventData rejected an XI2 raw event")
        try:
            if not cookie.data:
                raise RuntimeError("XI2 raw event omitted cookie data")
            raw = ctypes.cast(
                cookie.data, ctypes.POINTER(_XIRawEvent)
            ).contents
            if cookie.evtype == _XI_RAW_MOTION:
                return self._motion_event(raw)
            return XInput2RawEvent(
                evtype=cookie.evtype,
                deviceid=int(raw.deviceid),
                sourceid=int(raw.sourceid),
                detail=int(raw.detail),
            )
        finally:
            self._x11.XFreeEventData(self._display, ctypes.byref(cookie))

    def poll(
        self,
        *,
        current_look_pressed: bool,
        focused: bool,
    ) -> tuple[float, float, bool]:
        if type(focused) is not bool:
            raise ValueError("XI2 focus state must be boolean")
        self._ensure_telemetry_counters()
        topology_changed = False
        events: list[XInput2RawEvent] = []
        processed_this_poll = 0
        while self._x11.XPending(self._display):
            if processed_this_poll >= _MAX_XI2_EVENTS_PER_POLL:
                raise RuntimeError("XI2 raw-motion backlog exceeded the safe limit")
            event = self._read_event()
            processed_this_poll += 1
            self.events_consumed += 1
            if event is not None:
                events.append(event)
                if event.evtype == _XI_RAW_MOTION:
                    self.raw_motion_events += 1
                elif event.evtype == _XI_HIERARCHY_CHANGED:
                    self.hierarchy_events += 1
        hierarchy_changed = any(
            event.evtype == _XI_HIERARCHY_CHANGED for event in events
        )
        if hierarchy_changed:
            observed_master = self._single_master_pointer_deviceid()
            if observed_master != self._master_deviceid:
                self.master_device_changes += 1
                self._master_deviceid = observed_master
                topology_changed = True
        foreign_master_event_count = sum(
            event.evtype
            in {_XI_RAW_BUTTON_PRESS, _XI_RAW_BUTTON_RELEASE, _XI_RAW_MOTION}
            and event.deviceid != self._master_deviceid
            for event in events
        )
        foreign_master = foreign_master_event_count > 0
        if foreign_master:
            self.foreign_master_events += foreign_master_event_count
        if topology_changed or hierarchy_changed or foreign_master or not focused:
            # Topology/focus boundaries make per-source button attribution
            # ambiguous.  Drop the complete batch and require release followed
            # by a new raw press before any yaw delta can be accepted.
            self._accumulator.disarm()
            reasons = []
            if not focused:
                reasons.append("focus_or_pointer_invalid")
            if hierarchy_changed:
                reasons.append("hierarchy_changed")
            if topology_changed:
                reasons.append("master_device_changed")
            if foreign_master:
                reasons.append("foreign_master_event")
            if events or current_look_pressed or reasons[1:]:
                self._record_drop(
                    *reasons,
                    motion_events=tuple(
                        event
                        for event in events
                        if event.evtype == _XI_RAW_MOTION
                    ),
                )
            raw_look_edge = any(
                event.evtype in {_XI_RAW_BUTTON_PRESS, _XI_RAW_BUTTON_RELEASE}
                and event.detail
                == getattr(self._accumulator, "_look_button_detail", 0)
                for event in events
            )
            if self._button_gate == "x11-core-level":
                drag_observed = bool(
                    current_look_pressed
                    or raw_look_edge
                    or topology_changed
                    or hierarchy_changed
                    or foreign_master
                )
            else:
                # Preserve the existing x11-mirror interlock semantics.
                drag_observed = bool(
                    topology_changed or hierarchy_changed or foreign_master
                )
            if drag_observed:
                self.drag_batches += 1
            return (
                0.0,
                0.0,
                drag_observed,
            )
        result = self._accumulator.update(
            tuple(events), current_look_pressed=current_look_pressed
        )
        self._record_result(
            *result,
            motion_events=tuple(
                event for event in events if event.evtype == _XI_RAW_MOTION
            ),
        )
        return result

    @property
    def telemetry(self) -> dict[str, object]:
        self._ensure_telemetry_counters()
        return {
            "motion_source": (
                "xi2-raw-x11-core-gated"
                if self._button_gate == "x11-core-level"
                else "xi2-raw"
            ),
            "button_gate": self._button_gate,
            "negotiated_version": list(self._negotiated_version),
            "events_consumed": self.events_consumed,
            "raw_motion_events": self.raw_motion_events,
            "hierarchy_events": self.hierarchy_events,
            "master_deviceid": self._master_deviceid,
            "master_pointer_policy": "exactly-one",
            "master_device_changes": self.master_device_changes,
            "foreign_master_events": self.foreign_master_events,
            "button_state_resyncs": self._accumulator.button_state_resyncs,
            "accepted_dx_total": self.accepted_dx_total,
            "accepted_dy_total": self.accepted_dy_total,
            "last_accepted_dx": self.last_accepted_dx,
            "last_accepted_dy": self.last_accepted_dy,
            "drag_batches": self.drag_batches,
            "accepted_drag_batches": self.accepted_drag_batches,
            "dropped_batches": self.dropped_batches,
            "dropped_motion_events": self.dropped_motion_events,
            "dropped_dx_total": self.dropped_dx_total,
            "dropped_dy_total": self.dropped_dy_total,
            "drop_reason_counts": dict(self.drop_reason_counts),
            "last_drop_reasons": list(self.last_drop_reasons),
            "ambiguous_raw_motion_events": getattr(
                self._accumulator, "ambiguous_raw_motion_events", 0
            ),
            "ambiguous_raw_dx_total": getattr(
                self._accumulator, "ambiguous_raw_dx_total", 0.0
            ),
            "ambiguous_raw_dy_total": getattr(
                self._accumulator, "ambiguous_raw_dy_total", 0.0
            ),
            "bound_sourceid": getattr(
                self._accumulator, "bound_sourceid", None
            ),
            "source_bindings": getattr(
                self._accumulator, "source_bindings", 0
            ),
            "source_rejections": getattr(
                self._accumulator, "source_rejections", 0
            ),
            "maximum_events_per_poll": _MAX_XI2_EVENTS_PER_POLL,
        }

    def close(self) -> None:
        if self._display:
            self._x11.XCloseDisplay(self._display)
            self._display = None


class X11AbsoluteDragAccumulator:
    """Mirror held-drag root-pointer deltas with fail-closed boundaries."""

    def __init__(self, maximum_mouse_delta: float) -> None:
        if (
            not math.isfinite(maximum_mouse_delta)
            or maximum_mouse_delta <= 0.0
        ):
            raise ValueError("maximum absolute mouse delta must be positive and finite")
        self._maximum_mouse_delta = float(maximum_mouse_delta)
        self._previous_pointer: tuple[int, int] | None = None
        self._previous_look_pressed = False
        self._requires_release = True
        self.teleport_rejections = 0
        self.last_teleport_delta: tuple[int, int] | None = None
        self.accepted_dx_total = 0.0
        self.accepted_dy_total = 0.0
        self.last_accepted_dx = 0.0
        self.last_accepted_dy = 0.0
        self.drag_batches = 0
        self.accepted_drag_batches = 0
        self.dropped_batches = 0
        self.dropped_motion_events = 0
        self.dropped_dx_total = 0.0
        self.dropped_dy_total = 0.0
        self.drop_reason_counts: dict[str, int] = {}
        self.last_drop_reasons: tuple[str, ...] = ()

    def _drop(
        self,
        reason: str,
        *,
        dropped_dx: float = 0.0,
        dropped_dy: float = 0.0,
        motion_event: bool = False,
    ) -> None:
        self.dropped_batches += 1
        if motion_event:
            self.dropped_motion_events += 1
            self.dropped_dx_total += dropped_dx
            self.dropped_dy_total += dropped_dy
        self.drop_reason_counts[reason] = self.drop_reason_counts.get(reason, 0) + 1
        self.last_drop_reasons = (reason,)

    def disarm(self) -> None:
        self._previous_pointer = None
        self._previous_look_pressed = False
        self._requires_release = True

    def update(
        self,
        *,
        pointer: tuple[int, int] | None,
        current_look_pressed: bool,
        focused: bool,
    ) -> tuple[float, float, bool]:
        if type(current_look_pressed) is not bool or type(focused) is not bool:
            raise ValueError("absolute pointer button/focus states must be boolean")
        if pointer is not None and (
            len(pointer) != 2
            or any(type(coordinate) is not int for coordinate in pointer)
        ):
            raise ValueError("absolute pointer must be an integer root coordinate")
        self.last_drop_reasons = ()

        if pointer is None or not focused:
            drag_observed = bool(
                current_look_pressed or self._previous_look_pressed
            )
            if drag_observed:
                self.drag_batches += 1
                dropped_dx = 0.0
                dropped_dy = 0.0
                motion_event = False
                if pointer is not None and self._previous_pointer is not None:
                    dropped_dx = float(pointer[0] - self._previous_pointer[0])
                    dropped_dy = float(pointer[1] - self._previous_pointer[1])
                    motion_event = True
                self._drop(
                    "pointer_unavailable" if pointer is None else "focus_lost",
                    dropped_dx=dropped_dx,
                    dropped_dy=dropped_dy,
                    motion_event=motion_event,
                )
            self.disarm()
            return (0.0, 0.0, drag_observed)

        if self._requires_release:
            previous_pointer = self._previous_pointer
            self._previous_pointer = pointer
            self._previous_look_pressed = False
            if current_look_pressed:
                self.drag_batches += 1
                dropped_dx = 0.0
                dropped_dy = 0.0
                motion_event = previous_pointer is not None
                if previous_pointer is not None:
                    dropped_dx = float(pointer[0] - previous_pointer[0])
                    dropped_dy = float(pointer[1] - previous_pointer[1])
                self._drop(
                    "awaiting_release_before_fresh_press",
                    dropped_dx=dropped_dx,
                    dropped_dy=dropped_dy,
                    motion_event=motion_event,
                )
                return (0.0, 0.0, True)
            self._requires_release = False
            return (0.0, 0.0, False)

        previous_pointer = self._previous_pointer
        previous_pressed = self._previous_look_pressed
        self._previous_pointer = pointer
        self._previous_look_pressed = current_look_pressed

        if not previous_pressed and current_look_pressed:
            self.drag_batches += 1
            return (0.0, 0.0, True)
        if not previous_pressed:
            return (0.0, 0.0, False)

        # Attribute the complete interval to its held state at the beginning.
        # This deliberately preserves the final delta sampled on release, and
        # the returned drag flag hard-stops movement for that release frame.
        self.drag_batches += 1
        if previous_pointer is None:
            self._drop("missing_previous_pointer")
            return (0.0, 0.0, True)
        raw_dx = pointer[0] - previous_pointer[0]
        raw_dy = pointer[1] - previous_pointer[1]
        if max(abs(raw_dx), abs(raw_dy)) > self._maximum_mouse_delta:
            self.teleport_rejections += 1
            self.last_teleport_delta = (raw_dx, raw_dy)
            self._drop(
                "teleport_rejected",
                dropped_dx=float(raw_dx),
                dropped_dy=float(raw_dy),
                motion_event=True,
            )
            return (0.0, 0.0, True)
        dx = float(raw_dx)
        dy = float(raw_dy)
        self.accepted_drag_batches += 1
        self.accepted_dx_total += dx
        self.accepted_dy_total += dy
        self.last_accepted_dx = dx
        self.last_accepted_dy = dy
        return (dx, dy, True)

    @property
    def telemetry(self) -> dict[str, object]:
        return {
            "motion_source": "x11-absolute-root-delta",
            "button_gate": "xquerypointer-core-level",
            "teleport_rejections": self.teleport_rejections,
            "last_teleport_delta": list(self.last_teleport_delta)
            if self.last_teleport_delta is not None
            else None,
            "maximum_mouse_delta_px": self._maximum_mouse_delta,
            "accepted_dx_total": self.accepted_dx_total,
            "accepted_dy_total": self.accepted_dy_total,
            "last_accepted_dx": self.last_accepted_dx,
            "last_accepted_dy": self.last_accepted_dy,
            "drag_batches": self.drag_batches,
            "accepted_drag_batches": self.accepted_drag_batches,
            "dropped_batches": self.dropped_batches,
            "dropped_motion_events": self.dropped_motion_events,
            "dropped_dx_total": self.dropped_dx_total,
            "dropped_dy_total": self.dropped_dy_total,
            "drop_reason_counts": dict(self.drop_reason_counts),
            "last_drop_reasons": list(self.last_drop_reasons),
        }


class X11KeyboardMouse:
    """Poll global keyboard/pointer state without grabbing it from Matrix UE."""

    _BUTTON_MASK = {"left": 1 << 8, "middle": 1 << 9, "right": 1 << 10}
    _ARROW_KEY_NAMES = frozenset(
        {"arrow_left", "arrow_up", "arrow_right", "arrow_down"}
    )
    _KEYSYMS = {
        "w": 0x0077,
        "a": 0x0061,
        "s": 0x0073,
        "d": 0x0064,
        "q": 0x0071,
        "e": 0x0065,
        "v": 0x0076,
        "x": 0x0078,
        "j": 0x006A,
        "k": 0x006B,
        "l": 0x006C,
        "u": 0x0075,
        "i": 0x0069,
        "o": 0x006F,
        "arrow_left": 0xFF51,
        "arrow_up": 0xFF52,
        "arrow_right": 0xFF53,
        "arrow_down": 0xFF54,
        "ctrl_left": 0xFFE3,
        "ctrl_right": 0xFFE4,
        "alt_left": 0xFFE9,
        "alt_right": 0xFFEA,
        "shift_left": 0xFFE1,
        "shift_right": 0xFFE2,
        "escape": 0xFF1B,
        "mouse_mode": 0x006D,
        "mouse_speed_down": 0x002D,
        "mouse_speed_up": 0x003D,
        "apply_restart": 0xFFC6,
        "movement_mode_cycle": 0xFFC3,
        "apply_return": 0xFF0D,
    }

    def __init__(
        self,
        *,
        display_name: str | None,
        focus_title_pattern: str | None,
        expected_ue_pid: int | None,
        look_button: str,
        capture_raw_motion: bool = False,
        capture_absolute_motion: bool = False,
        raw_button_gate: str = "xi2-events",
        maximum_mouse_delta: float = 200.0,
        grab_ui_keys: bool = False,
        library: Any | None = None,
        xi_library: Any | None = None,
    ) -> None:
        if capture_raw_motion and capture_absolute_motion:
            raise ValueError("raw and absolute mouse capture are mutually exclusive")
        if library is None:
            library_name = ctypes.util.find_library("X11")
            if not library_name:
                raise RuntimeError("libX11 was not found")
            library = ctypes.CDLL(library_name)
        self._x11 = library
        self._configure_signatures()
        encoded_display = display_name.encode() if display_name else None
        self._display = self._x11.XOpenDisplay(encoded_display)
        if not self._display:
            label = display_name or os.environ.get("DISPLAY", "<unset>")
            raise RuntimeError(f"cannot open X11 display {label}")
        self._root = int(self._x11.XDefaultRootWindow(self._display))
        self._keycodes = {
            name: int(self._x11.XKeysymToKeycode(self._display, keysym))
            for name, keysym in self._KEYSYMS.items()
        }
        self._grab_ui_keys = bool(grab_ui_keys)
        self._grabbed_key_modifiers: dict[str, tuple[int, ...]] = {}
        self._grabbed_escape_down = False
        self._grabbed_escape_press_pending = False
        self._grabbed_ui_events = 0
        if any(
            code <= 0
            for name, code in self._keycodes.items()
            if name not in self._ARROW_KEY_NAMES
        ):
            self.close()
            raise RuntimeError("X11 keyboard map is missing a required key")
        self._focus_pattern = (
            re.compile(focus_title_pattern, re.IGNORECASE)
            if focus_title_pattern
            else None
        )
        if expected_ue_pid is not None and expected_ue_pid <= 1:
            self.close()
            raise ValueError("expected UE PID must be greater than 1")
        self._expected_ue_pid = expected_ue_pid
        self._pid_atom = int(
            self._x11.XInternAtom(self._display, b"_NET_WM_PID", 0)
        )
        self._look_mask = self._BUTTON_MASK[look_button]
        self._previous_pointer: tuple[int, int] | None = None
        self._previous_look_pressed = False
        self._maximum_mouse_delta = maximum_mouse_delta
        self._teleport_rejections = 0
        self._last_teleport_delta: tuple[int, int] | None = None
        self._absolute_motion: X11AbsoluteDragAccumulator | None = (
            X11AbsoluteDragAccumulator(maximum_mouse_delta)
            if capture_absolute_motion
            else None
        )
        self._focus_badwindow_recoveries = 0
        self._last_focus_badwindow_resource: int | None = None
        self._active_focus_error_scope: _X11FocusErrorScope | None = None
        self._previous_x_error_handler: int | None = None
        # XSetErrorHandler stores this process-global function pointer.  Keep
        # the ctypes callback alive for the complete backend lifetime even
        # though it is installed only inside a short, XSync-bounded scope.
        self._x_error_handler_callback = _X11_ERROR_HANDLER(
            self._handle_x_error
        )
        if self._grab_ui_keys:
            try:
                self._install_ui_key_grabs()
            except Exception:
                self.close()
                raise
        self._raw_motion: XInput2RawMotion | None = None
        if capture_raw_motion:
            try:
                self._raw_motion = XInput2RawMotion(
                    display_name=display_name,
                    look_button=look_button,
                    button_gate=raw_button_gate,
                    x11_library=self._x11,
                    xi_library=xi_library,
                )
            except Exception:
                self.close()
                raise

    @property
    def arrow_keys_available(self) -> bool:
        return all(
            self._keycodes.get(name, 0) > 0 for name in self._ARROW_KEY_NAMES
        )

    @property
    def pointer_telemetry(self) -> dict[str, object]:
        telemetry = {
            "arrow_keys_available": self.arrow_keys_available,
            "teleport_rejections": self._teleport_rejections,
            "last_teleport_delta": list(self._last_teleport_delta)
            if self._last_teleport_delta is not None
            else None,
            "maximum_mouse_delta_px": self._maximum_mouse_delta,
            "focus_badwindow_recoveries": getattr(
                self, "_focus_badwindow_recoveries", 0
            ),
            "last_focus_badwindow_resource": getattr(
                self, "_last_focus_badwindow_resource", None
            ),
            "ui_keys_grabbed": all(
                bool(getattr(self, "_grabbed_key_modifiers", {}).get(name))
                for name in _X11_UI_GRAB_KEY_NAMES
            ),
            "turn_keys_grabbed": all(
                bool(getattr(self, "_grabbed_key_modifiers", {}).get(name))
                for name in ("q", "e")
            ),
            "grabbed_ui_events": getattr(self, "_grabbed_ui_events", 0),
        }
        raw_motion = getattr(self, "_raw_motion", None)
        if raw_motion is not None:
            telemetry.update(raw_motion.telemetry)
        absolute_motion = getattr(self, "_absolute_motion", None)
        if absolute_motion is not None:
            telemetry.update(absolute_motion.telemetry)
        return telemetry

    def _configure_signatures(self) -> None:
        signatures = {
            "XOpenDisplay": ([ctypes.c_char_p], ctypes.c_void_p),
            "XDefaultRootWindow": ([ctypes.c_void_p], ctypes.c_ulong),
            "XKeysymToKeycode": ([ctypes.c_void_p, ctypes.c_ulong], ctypes.c_uint),
            "XQueryKeymap": ([ctypes.c_void_p, ctypes.c_void_p], ctypes.c_int),
            "XPending": ([ctypes.c_void_p], ctypes.c_int),
            "XNextEvent": (
                [ctypes.c_void_p, ctypes.POINTER(_XEvent)],
                ctypes.c_int,
            ),
            "XGrabKey": (
                [
                    ctypes.c_void_p,
                    ctypes.c_int,
                    ctypes.c_uint,
                    ctypes.c_ulong,
                    ctypes.c_int,
                    ctypes.c_int,
                    ctypes.c_int,
                ],
                ctypes.c_int,
            ),
            "XUngrabKey": (
                [ctypes.c_void_p, ctypes.c_int, ctypes.c_uint, ctypes.c_ulong],
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
            "XGetInputFocus": (
                [ctypes.c_void_p, ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_int)],
                ctypes.c_int,
            ),
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
            "XFetchName": (
                [ctypes.c_void_p, ctypes.c_ulong, ctypes.POINTER(ctypes.c_char_p)],
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
            "XSync": (
                [ctypes.c_void_p, ctypes.c_int],
                ctypes.c_int,
            ),
            "XFlush": ([ctypes.c_void_p], ctypes.c_int),
            # The callback type itself remains process-global in Xlib.  Use a
            # void pointer at the ABI boundary so the previous handler can be
            # restored verbatim, including Xlib's null/default sentinel.
            "XSetErrorHandler": ([ctypes.c_void_p], ctypes.c_void_p),
            "XFree": ([ctypes.c_void_p], ctypes.c_int),
            "XCloseDisplay": ([ctypes.c_void_p], ctypes.c_int),
        }
        optional = {"XPending", "XNextEvent", "XGrabKey", "XUngrabKey", "XFlush"}
        for name, (argtypes, restype) in signatures.items():
            try:
                function = getattr(self._x11, name)
            except AttributeError:
                if name in optional:
                    continue
                raise
            try:
                function.argtypes = argtypes
                function.restype = restype
            except (AttributeError, TypeError):
                # Simple fake callables used by unit tests need not expose
                # ctypes' signature attributes.
                pass

    @staticmethod
    def _pressed(keymap: bytes, keycode: int) -> bool:
        return bool(keymap[keycode >> 3] & (1 << (keycode & 7)))

    @staticmethod
    def _pointer_value(value: object) -> int:
        if isinstance(value, ctypes.c_void_p):
            return int(value.value or 0)
        return int(value or 0)

    def _handle_x_error(
        self,
        display: int | None,
        event_pointer: ctypes.POINTER(_XErrorEvent),
    ) -> int:
        """Suppress only a tracked focus window disappearing mid-query."""

        scope = self._active_focus_error_scope
        if scope is None or not event_pointer:
            return 0
        event = event_pointer.contents
        resource = int(event.resourceid)
        if (
            self._pointer_value(display) == self._pointer_value(self._display)
            and int(event.error_code) == _X11_BAD_WINDOW
            and resource in scope.windows
        ):
            scope.stale_window = resource
            return 0

        previous = self._previous_x_error_handler
        if previous:
            return int(
                _X11_ERROR_HANDLER(previous)(display, event_pointer)
            )
        # A null previous handler means Xlib's default handler.  A ctypes
        # callback cannot raise across the C boundary, so retain the complete
        # identity and surface it immediately after the trailing XSync.
        scope.unexpected_error = (
            int(event.error_code),
            int(event.request_code),
            int(event.minor_code),
            resource,
        )
        return 0

    @contextmanager
    def _x11_error_scope(self, label: str) -> Iterator[_X11FocusErrorScope]:
        """Bound asynchronous X errors to one synchronous Xlib operation.

        Xlib error handlers are process-global while protocol errors are
        asynchronous.  The leading XSync drains older requests under the
        caller's handler; the trailing XSync delivers only errors generated by
        this scope before the exact previous handler is restored.
        """

        with _X11_ERROR_HANDLER_LOCK:
            if self._active_focus_error_scope is not None:
                raise RuntimeError("nested X11 error scope")
            self._x11.XSync(self._display, 0)
            scope = _X11FocusErrorScope(windows=set(), label=label)
            self._active_focus_error_scope = scope
            callback = ctypes.cast(
                self._x_error_handler_callback, ctypes.c_void_p
            )
            previous_raw = self._x11.XSetErrorHandler(callback)
            previous = self._pointer_value(previous_raw)
            self._previous_x_error_handler = previous or None
            try:
                yield scope
            finally:
                try:
                    self._x11.XSync(self._display, 0)
                finally:
                    self._x11.XSetErrorHandler(
                        ctypes.c_void_p(previous) if previous else None
                    )
                    self._active_focus_error_scope = None
                    self._previous_x_error_handler = None
            if scope.unexpected_error is not None:
                error_code, request_code, minor_code, resource = (
                    scope.unexpected_error
                )
                raise RuntimeError(
                    f"unexpected X11 error during {scope.label}: "
                    f"code={error_code} request={request_code} "
                    f"minor={minor_code} resource={resource}"
                )

    @contextmanager
    def _focus_window_error_scope(self) -> Iterator[_X11FocusErrorScope]:
        """Bound asynchronous BadWindow handling to one focus-chain read."""

        with self._x11_error_scope("focus query") as scope:
            yield scope

    def _install_ui_key_grabs(self) -> None:
        """Consume Matrix UI keys before packaged UE can treat them as global commands.

        XQueryKeymap still exposes the physical key level to this provider, so
        ESC/Q/E remain usable by Matrix while Q cannot reach the cooked UE quit
        path.
        """

        missing = [
            name
            for name in ("XGrabKey", "XUngrabKey", "XPending", "XNextEvent", "XFlush")
            if not hasattr(self._x11, name)
        ]
        if missing:
            raise RuntimeError(
                "X11 UI key isolation is unavailable: missing "
                + ", ".join(missing)
            )
        grabbed_by_name: dict[str, tuple[int, ...]] = {}
        try:
            with self._x11_error_scope("Matrix UI passive grab"):
                for name in _X11_UI_GRAB_KEY_NAMES:
                    keycode = int(self._keycodes.get(name, 0))
                    if keycode <= 0:
                        raise RuntimeError(f"X11 keyboard map is missing {name}")
                    grabbed: list[int] = []
                    for modifiers in _X11_UI_GRAB_MODIFIERS:
                        self._x11.XGrabKey(
                            self._display,
                            keycode,
                            modifiers,
                            self._root,
                            0,  # owner_events=False: consume before UE/SDL sees it.
                            _X11_GRAB_MODE_ASYNC,
                            _X11_GRAB_MODE_ASYNC,
                        )
                        grabbed.append(modifiers)
                    grabbed_by_name[name] = tuple(grabbed)
                self._x11.XSync(self._display, 0)
        except RuntimeError as exc:
            raise RuntimeError(f"cannot grab Matrix UI keys: {exc}") from exc
        self._grabbed_key_modifiers = grabbed_by_name

    def _ungrab_ui_key_grabs(self) -> None:
        display = getattr(self, "_display", None)
        if not display:
            return
        grabbed_by_name = dict(getattr(self, "_grabbed_key_modifiers", {}))
        for name, modifiers in grabbed_by_name.items():
            keycode = int(getattr(self, "_keycodes", {}).get(name, 0))
            if keycode <= 0:
                continue
            for modifier in modifiers:
                self._x11.XUngrabKey(display, keycode, modifier, self._root)
        if grabbed_by_name:
            self._x11.XFlush(display)
        self._grabbed_key_modifiers = {}
        self._grabbed_escape_down = False
        self._grabbed_escape_press_pending = False

    def _drain_grabbed_ui_events(self) -> bool:
        """Drain passive-grab key events and return a one-frame Escape sample."""

        if not getattr(self, "_grabbed_key_modifiers", {}):
            return False
        escape_keycode = int(self._keycodes.get("escape", 0))
        processed = 0
        while self._x11.XPending(self._display):
            if processed >= _MAX_X11_GRABBED_UI_EVENTS_PER_POLL:
                raise RuntimeError(
                    "grabbed Matrix UI key event backlog exceeded the safe limit"
                )
            event = _XEvent()
            self._x11.XNextEvent(self._display, ctypes.byref(event))
            processed += 1
            if event.type not in {_X11_KEY_PRESS, _X11_KEY_RELEASE}:
                continue
            if int(event.xkey.keycode) != escape_keycode:
                continue
            self._grabbed_ui_events = (
                getattr(self, "_grabbed_ui_events", 0) + 1
            )
            if event.type == _X11_KEY_PRESS:
                self._grabbed_escape_down = True
                self._grabbed_escape_press_pending = True
            else:
                self._grabbed_escape_down = False
        escape_pressed = bool(
            getattr(self, "_grabbed_escape_down", False)
            or getattr(self, "_grabbed_escape_press_pending", False)
        )
        self._grabbed_escape_press_pending = False
        return escape_pressed

    def _fetch_name(self, window: int) -> str | None:
        name = ctypes.c_char_p()
        if not self._x11.XFetchName(self._display, window, ctypes.byref(name)):
            return None
        try:
            return name.value.decode("utf-8", errors="replace") if name.value else None
        finally:
            if name:
                self._x11.XFree(name)

    def _parent(self, window: int) -> int | None:
        root = ctypes.c_ulong()
        parent = ctypes.c_ulong()
        children = ctypes.POINTER(ctypes.c_ulong)()
        child_count = ctypes.c_uint()
        ok = self._x11.XQueryTree(
            self._display,
            window,
            ctypes.byref(root),
            ctypes.byref(parent),
            ctypes.byref(children),
            ctypes.byref(child_count),
        )
        if children:
            self._x11.XFree(children)
        if not ok or parent.value in {0, window}:
            return None
        return int(parent.value)

    def _window_pid(self, window: int) -> int | None:
        if self._pid_atom == 0:
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

    def _focus_identity(self) -> tuple[bool, str | None, frozenset[int]]:
        """Read validity, title, and PIDs from one X11 focus ancestry chain."""

        result: tuple[bool, str | None, frozenset[int]] = (
            False,
            None,
            frozenset(),
        )
        with self._focus_window_error_scope() as error_scope:
            focus = ctypes.c_ulong()
            revert = ctypes.c_int()
            if self._x11.XGetInputFocus(
                self._display, ctypes.byref(focus), ctypes.byref(revert)
            ):
                window = int(focus.value)
                if window > 1:  # X11 None and PointerRoot sentinels
                    title = None
                    process_ids: set[int] = set()
                    for _ in range(12):
                        error_scope.windows.add(window)
                        if title is None:
                            title = self._fetch_name(window)
                        candidate_pid = self._window_pid(window)
                        if candidate_pid is not None:
                            process_ids.add(candidate_pid)
                        parent = self._parent(window)
                        if parent is None or parent == self._root:
                            break
                        window = parent
                    result = (True, title, frozenset(process_ids))
        if error_scope.stale_window is not None:
            self._focus_badwindow_recoveries = (
                getattr(self, "_focus_badwindow_recoveries", 0) + 1
            )
            self._last_focus_badwindow_resource = error_scope.stale_window
            return (False, None, frozenset())
        return result

    def poll(self) -> KeyboardMouseSample:
        grabbed_escape_pressed = self._drain_grabbed_ui_events()
        key_buffer = ctypes.create_string_buffer(32)
        if not self._x11.XQueryKeymap(self._display, key_buffer):
            raise RuntimeError("XQueryKeymap failed")
        keymap = key_buffer.raw

        root_return = ctypes.c_ulong()
        child_return = ctypes.c_ulong()
        root_x = ctypes.c_int()
        root_y = ctypes.c_int()
        win_x = ctypes.c_int()
        win_y = ctypes.c_int()
        mask = ctypes.c_uint()
        pointer_ok = self._x11.XQueryPointer(
            self._display,
            self._root,
            ctypes.byref(root_return),
            ctypes.byref(child_return),
            ctypes.byref(root_x),
            ctypes.byref(root_y),
            ctypes.byref(win_x),
            ctypes.byref(win_y),
            ctypes.byref(mask),
        )
        pointer = (root_x.value, root_y.value) if pointer_ok else None
        look_pressed = bool(pointer_ok and mask.value & self._look_mask)
        has_application_focus, focus_title, focus_pids = self._focus_identity()
        focus_pid = (
            self._expected_ue_pid
            if self._expected_ue_pid in focus_pids
            else min(focus_pids, default=None)
        )
        # Pointer state is part of the safety interlock: without it we cannot
        # know whether the native look button is held, so movement must stop.
        focused = bool(pointer_ok and has_application_focus)
        if self._focus_pattern is not None:
            focused = bool(
                focused and focus_title and self._focus_pattern.search(focus_title)
            )
        if self._expected_ue_pid is not None:
            focused = bool(focused and self._expected_ue_pid in focus_pids)

        mouse_dx = 0.0
        mouse_dy = 0.0
        raw_drag_observed = False
        absolute_drag_observed = False
        raw_motion = getattr(self, "_raw_motion", None)
        if raw_motion is not None:
            # XI_RawMotion is commonly used by SDL relative mode, which the
            # launcher requests.  Mirror it so the current MouseLock's
            # absolute pyautogui/XTEST recenter cannot cancel the outward raw
            # drag inside one 50 Hz XQueryPointer interval.  Packaged-UE
            # consumption remains a separate live black-box qualification.
            mouse_dx, mouse_dy, raw_drag_observed = raw_motion.poll(
                current_look_pressed=look_pressed,
                focused=focused,
            )
        else:
            absolute_motion = getattr(self, "_absolute_motion", None)
            if absolute_motion is not None:
                (
                    mouse_dx,
                    mouse_dy,
                    absolute_drag_observed,
                ) = absolute_motion.update(
                    pointer=pointer,
                    current_look_pressed=look_pressed,
                    focused=focused,
                )
        self._previous_pointer = pointer
        self._previous_look_pressed = look_pressed

        if not focused:
            mouse_dx = 0.0
            mouse_dy = 0.0
        pressed = {
            name: self._pressed(keymap, code) for name, code in self._keycodes.items()
        }
        return KeyboardMouseSample(
            **{
                name: pressed.get(name, False)
                for name in (
                    "w",
                    "a",
                    "s",
                    "d",
                    "q",
                    "e",
                    "v",
                    "x",
                    "j",
                    "k",
                    "l",
                    "u",
                    "i",
                    "o",
                    "arrow_left",
                    "arrow_up",
                    "arrow_right",
                    "arrow_down",
                )
            },
            ctrl=pressed.get("ctrl_left", False)
            or pressed.get("ctrl_right", False),
            alt=pressed.get("alt_left", False)
            or pressed.get("alt_right", False),
            shift=pressed.get("shift_left", False)
            or pressed.get("shift_right", False),
            escape=grabbed_escape_pressed or pressed.get("escape", False),
            mouse_mode=pressed.get("mouse_mode", False),
            mouse_speed_down=pressed.get("mouse_speed_down", False),
            mouse_speed_up=pressed.get("mouse_speed_up", False),
            apply_restart=pressed.get("apply_restart", False),
            movement_mode_cycle=pressed.get("movement_mode_cycle", False),
            apply_return=pressed.get("apply_return", False),
            mouse_dx=mouse_dx,
            mouse_dy=mouse_dy,
            camera_dragging=focused
            and (look_pressed or raw_drag_observed or absolute_drag_observed),
            focused=focused,
            focus_title=focus_title,
            focus_pid=focus_pid,
        )

    def close(self) -> None:
        self._ungrab_ui_key_grabs()
        raw_motion = getattr(self, "_raw_motion", None)
        if raw_motion is not None:
            raw_motion.close()
            self._raw_motion = None
        if getattr(self, "_display", None):
            self._x11.XCloseDisplay(self._display)
            self._display = None


class LinuxJoystick:
    """Non-blocking standard-library reader for Linux's ``js`` API."""

    def __init__(
        self,
        device: str | None,
        *,
        left_x_axis: int,
        left_y_axis: int,
        right_x_axis: int,
        right_y_axis: int,
        opener: Callable[..., int] = os.open,
        reader: Callable[[int, int], bytes] = os.read,
        closer: Callable[[int], None] = os.close,
    ) -> None:
        self._configured_device = device
        self._left_x = left_x_axis
        self._left_y = left_y_axis
        self._right_x = right_x_axis
        self._right_y = right_y_axis
        self._opener = opener
        self._reader = reader
        self._closer = closer
        self._fd: int | None = None
        self._path: str | None = None
        self._axes: dict[int, float] = {}
        self._next_open = 0.0

    @property
    def path(self) -> str | None:
        return self._path

    def _candidate(self) -> str | None:
        if self._configured_device:
            return self._configured_device
        candidates = sorted(glob.glob("/dev/input/js*"))
        return candidates[0] if candidates else None

    def _open_if_due(self, now: float) -> None:
        if self._fd is not None or now < self._next_open:
            return
        path = self._candidate()
        if path is None:
            self._next_open = now + 1.0
            return
        try:
            self._fd = self._opener(path, os.O_RDONLY | os.O_NONBLOCK)
            self._path = path
            self._axes.clear()
        except OSError:
            self._fd = None
            self._path = None
            self._next_open = now + 1.0

    def _disconnect(self, now: float) -> None:
        if self._fd is not None:
            try:
                self._closer(self._fd)
            except OSError:
                pass
        self._fd = None
        self._path = None
        self._axes.clear()
        self._next_open = now + 1.0

    def poll(self, now: float) -> GamepadSample:
        self._open_if_due(now)
        if self._fd is None:
            return GamepadSample()
        while True:
            try:
                payload = self._reader(self._fd, _JS_EVENT.size)
            except BlockingIOError:
                break
            except OSError as exc:
                if exc.errno in {errno.EAGAIN, errno.EWOULDBLOCK}:
                    break
                self._disconnect(now)
                return GamepadSample()
            if not payload or len(payload) != _JS_EVENT.size:
                self._disconnect(now)
                return GamepadSample()
            _milliseconds, value, event_type, number = _JS_EVENT.unpack(payload)
            event_type &= ~_JS_EVENT_INIT
            if event_type == _JS_EVENT_AXIS:
                self._axes[number] = _clamp(value / 32767.0, -1.0, 1.0)
            elif event_type == _JS_EVENT_BUTTON:
                continue
        return GamepadSample(
            forward=-self._axes.get(self._left_y, 0.0),
            right=self._axes.get(self._left_x, 0.0),
            look_yaw=self._axes.get(self._right_x, 0.0),
            look_pitch=-self._axes.get(self._right_y, 0.0),
            connected=True,
        )

    def close(self) -> None:
        self._disconnect(0.0)


class UnixSeqpacketPublisher:
    """Reconnectable client for the core's authenticated local socket."""

    def __init__(
        self,
        path: str | os.PathLike[str],
        *,
        reconnect_seconds: float = 0.2,
        io_timeout_seconds: float = 0.01,
        socket_factory: Callable[..., socket.socket] = socket.socket,
    ) -> None:
        self.path = Path(path)
        self._reconnect_seconds = reconnect_seconds
        if not math.isfinite(io_timeout_seconds) or io_timeout_seconds <= 0.0:
            raise ValueError("io_timeout_seconds must be positive and finite")
        self._io_timeout_seconds = io_timeout_seconds
        self._socket_factory = socket_factory
        self._socket: socket.socket | None = None
        self._next_connect = 0.0

    @property
    def connected(self) -> bool:
        return self._socket is not None

    def _connect(self, now: float) -> bool:
        if self._socket is not None:
            return True
        if now < self._next_connect:
            return False
        socket_type = getattr(socket, "SOCK_SEQPACKET", None)
        if socket_type is None:
            raise RuntimeError("SOCK_SEQPACKET is unavailable on this platform")
        candidate = self._socket_factory(socket.AF_UNIX, socket_type)
        # A stale server with a full one-peer backlog must not freeze the input
        # sampler and defeat its deadman semantics.
        candidate.settimeout(self._io_timeout_seconds)
        try:
            candidate.connect(os.fspath(self.path))
        except OSError:
            candidate.close()
            self._next_connect = now + self._reconnect_seconds
            return False
        self._socket = candidate
        return True

    def send(self, snapshot: InputSnapshot, *, now: float) -> bool:
        payload = encode_input_packet(snapshot)
        if len(payload) > MAX_PACKET_BYTES:
            raise RuntimeError("encoded input snapshot exceeded protocol limit")
        if not self._connect(now):
            return False
        assert self._socket is not None
        try:
            # One send must correspond to one SOCK_SEQPACKET record.  sendall()
            # is stream-oriented and could turn an exceptional partial write
            # into multiple protocol packets.
            sent = self._socket.send(payload)
            if sent != len(payload):
                raise OSError(
                    f"partial input packet write: sent {sent} of {len(payload)} bytes"
                )
        except OSError:
            self._socket.close()
            self._socket = None
            self._next_connect = now + self._reconnect_seconds
            return False
        return True

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close()
            self._socket = None


def build_snapshot(
    *,
    sequence: int,
    timestamp_monotonic_s: float,
    keyboard: KeyboardMouseSample,
    gamepad: GamepadSample,
    input_source: str,
    camera_yaw_rad: float,
    camera_available: bool,
    input_available: bool = True,
) -> InputSnapshot:
    keys, move_stick, _look_yaw = select_physical_inputs(
        keyboard, gamepad, source=input_source
    )
    return InputSnapshot(
        sequence=sequence,
        timestamp_monotonic_s=timestamp_monotonic_s,
        # Missing actual camera yaw is a safety condition, not permission to
        # keep walking using the last direction.
        # The current Matrix launcher reads the actual UE PlayerCameraManager
        # final POV, so mouse/arrow camera movement can safely coexist with
        # held WASD.  Focus/camera/input availability remain hard gates.
        focused=keyboard.focused and camera_available and input_available,
        camera_yaw_rad=camera_yaw_rad,
        keys=keys,
        move_stick=move_stick,
    )


def _atomic_json(path: Path | None, payload: dict[str, object]) -> None:
    if path is None:
        return
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


def _read_json_object(path: Path) -> dict[str, object] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, UnicodeError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


@dataclass(frozen=True)
class OverlayIntent:
    """One authenticated action emitted by the supervised overlay child."""

    kind: str
    action: str | None = None
    command: str | None = None
    active: bool | None = None
    font_size: int | None = None
    pause_target: str | None = None
    expected_epoch: int | None = None


def function_library_mapping(
    path: Path | None,
    *,
    open_count: int = 0,
    open_error: str | None = None,
) -> dict[str, object]:
    if path is None:
        return {
            "directory": None,
            "available": False,
            "files": [],
            "open_available": False,
            "open_count": open_count,
            "open_error": open_error,
        }
    files: list[str] = []
    available = path.is_dir()
    if available:
        try:
            root = path.resolve()
            for item in sorted(root.rglob("*.mcfunction")):
                if item.is_file():
                    files.append(item.relative_to(root).with_suffix("").as_posix())
                if len(files) >= 64:
                    break
        except OSError as exc:
            available = False
            files = []
            open_error = str(exc)
    return {
        "directory": str(path),
        "available": available,
        "files": files,
        "open_available": bool(shutil.which("xdg-open") or shutil.which("gio")),
        "open_count": open_count,
        "open_error": open_error,
    }


_CELESTIAL_WEATHER_STATE = {
    "cloudiness": 0.0,
    "precipitation": 0.0,
    "precipitation_deposits": 0.0,
    "wind_intensity": 0.0,
    "sun_azimuth_angle": 45.0,
    "sun_altitude_angle": 45.0,
    "fog_density": 0.0,
    "fog_distance": 100_000.0,
    "fog_falloff": 0.0,
    "wetness": 0.0,
    "scattering_intensity": 1.0,
    "mie_scattering_scale": 1.0,
    "rayleigh_scattering_scale": 1.0,
    "dust_storm": 0.0,
}


def _celestial_project_root(project_root: Path | None) -> Path:
    if project_root is not None:
        return project_root
    return Path(os.environ.get("MATRIX_PROJECT_ROOT", Path(__file__).resolve().parents[1]))


def _celestial_scene_assets_available(
    project_root: Path,
    target: Mapping[str, object],
) -> bool:
    scene_xml = target.get("scene_xml")
    destination_id = target.get("destination_id")
    if not isinstance(scene_xml, str) or not isinstance(destination_id, str):
        return False
    required = [
        project_root / "src/robot_mujoco/zsibot_robots/xgb" / scene_xml,
        project_root
        / "src/UeSim/Linux/zsibot_mujoco_ue/Content/model/xgb"
        / scene_xml,
    ]
    if destination_id == "moon":
        required.append(project_root / "dynamicmaps/moonworld.bin")
    return all(path.is_file() for path in required)


def _celestial_target_vector(
    target: Mapping[str, object],
    field: str,
) -> list[float]:
    value = target[field]
    assert isinstance(value, list)
    return [float(component) for component in value]


def celestial_navigation_mapping(
    build_info: Mapping[str, object],
    *,
    project_root: Path | None = None,
) -> dict[str, object]:
    """Publish the safe scene-reload destinations understood by the ESC overlay."""

    root = _celestial_project_root(project_root)
    raw_scene_id = build_info.get("scene_id")
    current_destination = "moon" if raw_scene_id == 15 else "town10"
    ready: dict[str, bool] = {}
    for destination_id, target in WORLD_SCENE_TARGETS.items():
        ready[destination_id] = _celestial_scene_assets_available(root, target)
    ready[current_destination] = True

    moon_ready = ready.get("moon", False)
    town_ready = ready.get("town10", False)
    current_body_id = (
        WORLD_SCENE_TARGETS[current_destination]["body_id"]
        if current_destination in WORLD_SCENE_TARGETS
        else "earth"
    )
    assert isinstance(current_body_id, str)
    current_atmosphere = (
        WORLD_SCENE_TARGETS[current_destination]["atmosphere"]
        if current_destination in WORLD_SCENE_TARGETS
        else "terrestrial"
    )
    assert isinstance(current_atmosphere, str)
    destinations: list[dict[str, object]] = []
    for destination_id in ("town10", "moon"):
        target = WORLD_SCENE_TARGETS[destination_id]
        is_ready = ready.get(destination_id, False)
        destinations.append(
            {
                "id": destination_id,
                "body_id": target["body_id"],
                "body_name": target["body_name"],
                "display_name": target["display_name"],
                "teleport_tag": target["teleport_tag"],
                "runtime_status": "active" if is_ready else "planned",
                "status": "ready" if is_ready else "world_unavailable",
                "enabled": is_ready,
                "surface_coordinates_deg_m": _celestial_target_vector(
                    target, "surface_coordinates_deg_m"
                ),
                "surface_heading_deg": float(target["surface_heading_deg"]),
                "local_position_m": (
                    _celestial_target_vector(target, "local_position_m")
                    if is_ready
                    else None
                ),
                "site_universe_position_m": _celestial_target_vector(
                    target, "site_universe_position_m"
                ),
                "universe_position_m": (
                    _celestial_target_vector(target, "universe_position_m")
                    if is_ready
                    else None
                ),
                "gravity_m_s2": float(target["gravity_m_s2"]),
                "atmosphere": target["atmosphere"],
            }
        )

    return {
        "version": 2,
        "available": True,
        "status": "ready",
        "universe_id": "sol_2080",
        "display_name": "SOL 星体导航",
        "reference_epoch_utc": "2080-01-01T00:00:00Z",
        "time_scale": "TAI",
        "frame": "matrix_mj_world",
        "ephemeris": {
            "provider": "static-sol-v1",
            "accuracy_class": "static-demo",
            "upgrade_target": "spice-v2",
        },
        "simulation_time": {
            "elapsed_tai_ns": 0,
            "scenario_tai_ns": 0,
            "scenario_utc": "2080-01-01T00:00:00Z",
            "rate_numerator": 1,
            "rate_denominator": 1,
            "utc_assumption": "static_demo",
        },
        "origin_rebasing": True,
        "simulation_local_bound_m": 100_000.0,
        "current_body_id": current_body_id,
        "bodies": [
            {
                "id": "sun",
                "display_name": "太阳",
                "naif_id": 10,
                "runtime_status": "reference",
                "center_inertial_m": [0.0, 0.0, 0.0],
                "solar_distance_m": 0.0,
            },
            {
                "id": "earth",
                "display_name": "地球",
                "naif_id": 399,
                "runtime_status": "active" if town_ready else "planned",
                "center_inertial_m": [149_597_870_700.0, 0.0, 0.0],
                "solar_distance_m": 149_597_870_700.0,
            },
            {
                "id": "moon",
                "display_name": "月球",
                "naif_id": 301,
                "runtime_status": "active" if moon_ready else "planned",
                "center_inertial_m": [149_982_270_700.0, 0.0, 0.0],
                "solar_distance_m": 149_982_270_700.0,
            },
        ],
        "lighting": {
            "body_id": current_body_id,
            "atmosphere": current_atmosphere,
            "sun_direction_local": [0.70710678, 0.0, -0.70710678],
            "directional_light_direction_local": [-0.70710678, -0.0, 0.70710678],
            "sun_altitude_deg": 45.0,
            "sun_azimuth_deg": 45.0,
            "solar_distance_m": (
                149_982_270_700.0 if current_body_id == "moon" else 149_597_870_700.0
            ),
            "solar_irradiance_w_m2": 1361.0,
            "sun_angular_radius_deg": 0.2666,
            "eclipse_fraction": 0.0,
            "eclipse_occluder_id": None,
            "starfield_visibility": 1.0 if current_body_id == "moon" else 0.15,
            "visual_profile": {
                "schema": "matrix-celestial-visual-profile/v1",
                "id": f"{current_body_id}-static-v1",
                "sha256": "0" * 64,
                "display_name": (
                    "Moon static visual" if current_body_id == "moon" else "Earth static visual"
                ),
                "body_id": current_body_id,
                "atmosphere": current_atmosphere,
                "renderer": "carla-weather-v1",
                "weather_parameters": dict(_CELESTIAL_WEATHER_STATE),
            },
            "render_authority": "state-only",
            "render_status": "not-applied",
            "render_error": None,
            "visible_camera_verified": False,
        },
        "destinations": destinations,
    }


def open_function_directory(path: Path | None) -> tuple[bool, str | None]:
    if path is None:
        return False, "function directory is not configured"
    try:
        path.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        return False, f"cannot create function directory: {exc}"
    opener = shutil.which("xdg-open")
    command = [opener, str(path)] if opener else None
    if command is None:
        gio = shutil.which("gio")
        if gio is not None:
            command = [gio, "open", str(path)]
    if command is None:
        return False, "xdg-open/gio is not available"
    try:
        subprocess.Popen(
            command,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
    except OSError as exc:
        return False, f"cannot open function directory: {exc}"
    return True, None


class GameCommandClient:
    """Send one typed MC command at a time over an inherited socketpair.

    Raw command text terminates here: :func:`parse_mc_command` produces the
    typed AST carried by ``matrix-game-command/v1``.  A successfully sent
    request remains authoritative until its exact response arrives.  The
    client never reconnects, resends, or converts a timeout into a retry.
    """

    def __init__(self, file_descriptor: int | None) -> None:
        self._connection: socket.socket | None = None
        self._session = os.urandom(16).hex()
        self._sequence = 0
        self._result_revision = 0
        self._pending: GameCommandRequest | None = None
        self._pending_warning: str | None = None
        self._outcome_unknown = False
        self.editing = False
        self._escape_release_required = False
        self.status = "unavailable" if file_descriptor is None else "idle"
        self.ok: bool | None = None
        self.code: str | None = None
        self.message: str | None = (
            "Game commands are unavailable for this run"
            if file_descriptor is None
            else None
        )
        self.warning: str | None = None
        self.restart_required = False
        self.data: dict[str, object] | None = None
        self.last_request_id: str | None = None
        self._runtime_pause = self._runtime_pause_unavailable()
        if file_descriptor is None:
            return
        if (
            isinstance(file_descriptor, bool)
            or not isinstance(file_descriptor, int)
            or file_descriptor < 0
        ):
            raise ValueError("game command file descriptor must be non-negative")
        connection: socket.socket | None = None
        try:
            connection = socket.socket(fileno=file_descriptor)
            if connection.family != socket.AF_UNIX:
                raise ValueError("game command channel must be an AF_UNIX socket")
            if (
                connection.getsockopt(socket.SOL_SOCKET, socket.SO_TYPE)
                != socket.SOCK_SEQPACKET
            ):
                raise ValueError("game command channel must use SOCK_SEQPACKET")
            connection.setblocking(False)
        except Exception:
            if connection is not None:
                connection.close()
            raise
        self._connection = connection
        self._runtime_pause = self._runtime_pause_running(epoch=0)

    @staticmethod
    def _runtime_pause_unavailable() -> dict[str, object]:
        return {
            "state": "unavailable",
            "epoch": 0,
            "can_pause": False,
            "can_resume": False,
            "last_error": None,
        }

    @staticmethod
    def _runtime_pause_running(*, epoch: int) -> dict[str, object]:
        return {
            "state": "running",
            "epoch": epoch,
            "can_pause": True,
            "can_resume": False,
            "last_error": None,
        }

    @staticmethod
    def _runtime_pause_pending(target: str, *, epoch: int) -> dict[str, object]:
        return {
            "state": "pausing" if target == "paused" else "resuming",
            "epoch": epoch,
            "can_pause": False,
            "can_resume": False,
            "last_error": None,
        }

    @staticmethod
    def _coerce_runtime_pause(value: object) -> dict[str, object] | None:
        if not isinstance(value, dict) or set(value) != {
            "state",
            "epoch",
            "can_pause",
            "can_resume",
            "last_error",
        }:
            return None
        state = value.get("state")
        epoch = value.get("epoch")
        can_pause = value.get("can_pause")
        can_resume = value.get("can_resume")
        last_error = value.get("last_error")
        if (
            state
            not in {"running", "paused", "pausing", "resuming", "busy", "fault", "unavailable"}
            or type(epoch) is not int
            or not 0 <= epoch <= MAX_RUNTIME_PAUSE_EPOCH
            or type(can_pause) is not bool
            or type(can_resume) is not bool
            or (last_error is not None and not isinstance(last_error, str))
        ):
            return None
        return {
            "state": state,
            "epoch": epoch,
            "can_pause": can_pause,
            "can_resume": can_resume,
            "last_error": last_error,
        }

    @classmethod
    def _coerce_runtime_pause_from_response_data(
        cls,
        value: object,
    ) -> dict[str, object] | None:
        """Return the latest runtime-pause state from direct or function data."""

        if not isinstance(value, dict):
            return None
        latest = cls._coerce_runtime_pause(value.get("runtime_pause"))
        steps = value.get("steps")
        if isinstance(steps, list):
            for step in steps:
                if not isinstance(step, dict):
                    continue
                step_data = step.get("data")
                if not isinstance(step_data, dict):
                    continue
                nested = cls._coerce_runtime_pause(step_data.get("runtime_pause"))
                if nested is not None:
                    latest = nested
        return latest

    @property
    def available(self) -> bool:
        return self._connection is not None

    @property
    def in_flight(self) -> bool:
        return self._pending is not None

    @property
    def outcome_unknown(self) -> bool:
        return self._outcome_unknown

    def _local_error(self, code: str, message: str) -> None:
        self._result_revision += 1
        self._outcome_unknown = False
        self.status = "error"
        self.ok = False
        self.code = code
        self.message = message
        self.warning = None
        self.restart_required = False
        self.data = None
        self.last_request_id = None

    def _mark_runtime_pause_fault(self, message: str) -> None:
        current = self._runtime_pause
        epoch = current.get("epoch") if isinstance(current, dict) else 0
        self._runtime_pause = {
            "state": "fault",
            "epoch": epoch if type(epoch) is int else 0,
            "can_pause": False,
            "can_resume": False,
            "last_error": message[:256],
        }

    def _close_channel(self) -> None:
        connection = self._connection
        self._connection = None
        if connection is not None:
            connection.close()

    def _protocol_failure(self, message: str) -> None:
        pending = self._pending
        self._pending = None
        self._pending_warning = None
        self._close_channel()
        if pending is not None:
            # A full SOCK_SEQPACKET record was accepted before this failure.
            # The runtime may already have committed the world-state mutation,
            # so presenting an ordinary failure would invite a duplicate
            # summon if the operator retried.  Preserve the correlation id and
            # make the ambiguity terminal for this provider generation.
            detail = message[:256]
            self._result_revision += 1
            self._outcome_unknown = True
            self.status = "error"
            self.ok = None
            self.code = "E_COMMAND_OUTCOME_UNKNOWN"
            self.message = (
                f"Command outcome unknown ({detail}); do not retry blindly. "
                "Restart Matrix and inspect the persisted world state"
            )
            self.warning = None
            self.restart_required = False
            self.data = None
            self.last_request_id = pending.request_id
            return
        self._local_error("E_COMMAND_PROTOCOL", message)

    def set_editing(
        self,
        editing: bool,
        *,
        panel_active: bool,
        restart_requested: bool,
    ) -> bool:
        """Apply an overlay editor-level intent without weakening panel gates."""

        if type(editing) is not bool:
            raise TypeError("command editing state must be boolean")
        if editing and (
            self.restart_required
            or self._outcome_unknown
            or not self.available
            or not panel_active
            or restart_requested
            or self.in_flight
        ):
            return False
        changed = self.editing != editing
        self.editing = editing
        if changed and self.code is None:
            self.status = "editing" if editing and self.available else (
                "idle" if self.available else "unavailable"
            )
        return changed

    def panel_closed(self) -> bool:
        """Clear editor state after a legitimate panel exit."""

        if self.in_flight or self.restart_required or self.outcome_unknown:
            return False
        changed = self.editing
        self.editing = False
        self._escape_release_required = False
        if self.code is None:
            self.status = "idle" if self.available else "unavailable"
        return changed

    def panel_escape_pressed(
        self, pressed: bool, *, editor_owned_this_frame: bool = False
    ) -> bool:
        """Return the Escape level visible to the outer calibration toggle.

        The first Escape while editing belongs to the editor.  Even after the
        overlay publishes ``command_edit(false)``, the held level stays masked
        until a physical release, so it cannot become a synthetic panel-close
        edge on the following provider frame.  Pending commands likewise keep
        the safe panel open.
        """

        if type(pressed) is not bool:
            raise TypeError("Escape level must be boolean")
        if type(editor_owned_this_frame) is not bool:
            raise TypeError("editor-owned Escape flag must be boolean")
        editor_owned_this_frame = bool(editor_owned_this_frame and self.available)
        if not pressed:
            self._escape_release_required = False
            return False
        if (
            self.editing
            or self.in_flight
            or self.restart_required
            or self._outcome_unknown
            or editor_owned_this_frame
        ):
            self._escape_release_required = True
            return False
        if self._escape_release_required:
            return False
        return True

    def submit(
        self,
        command_text: object,
        *,
        calibration_active: bool,
        neutral_frame_ready: bool,
        restart_requested: bool,
        require_editing: bool = True,
    ) -> bool:
        """Parse and atomically send one request when every ESC gate is true."""

        if self.in_flight or self.restart_required or self._outcome_unknown:
            return False
        if not calibration_active:
            self._local_error("E_NOT_PAUSED", "Open the ESC panel before commands")
            return False
        if not neutral_frame_ready:
            self._local_error(
                "E_NEUTRAL_REQUIRED",
                "Wait for the ESC panel to deliver a neutral frame",
            )
            return False
        if require_editing and not self.editing:
            self._local_error(
                "E_COMMAND_EDIT_REQUIRED", "Activate the command input first"
            )
            return False
        if restart_requested:
            self._local_error(
                "E_RESTART_PENDING", "A whole-runtime restart is already pending"
            )
            return False
        connection = self._connection
        if connection is None:
            self._local_error(
                "E_COMMAND_UNAVAILABLE", "Game commands are unavailable for this run"
            )
            return False
        try:
            parsed = parse_mc_command(command_text)
        except CommandParseError as exc:
            message = exc.message
            if exc.column is not None:
                message = f"{message} (column {exc.column})"
            self._local_error(exc.code, message)
            return False
        self._sequence += 1
        request = GameCommandRequest(
            session=self._session,
            sequence=self._sequence,
            request_id=f"cmd-{os.urandom(16).hex()}",
            command=parsed.command,
        )
        payload = encode_command_request(request)
        try:
            sent = connection.send(payload)
        except BlockingIOError as exc:
            # SOCK_SEQPACKET writes are atomic.  No bytes were accepted on
            # BlockingIOError, but retrying automatically would make execution
            # ambiguous if the failure mode ever changes.
            self._local_error("E_COMMAND_SEND", f"Could not send command: {exc}")
            return False
        except OSError as exc:
            self._close_channel()
            self._local_error("E_COMMAND_SEND", f"Could not send command: {exc}")
            return False
        if sent != len(payload):
            self._close_channel()
            self._local_error(
                "E_COMMAND_SEND",
                f"Partial command packet write: sent {sent}/{len(payload)}",
            )
            return False
        self._pending = request
        self._pending_warning = parsed.warning
        self._result_revision += 1
        self.status = "pending"
        self.ok = None
        self.code = None
        self.message = "Command submitted; waiting for the runtime"
        self.warning = parsed.warning
        self.restart_required = False
        self.data = None
        self.last_request_id = request.request_id
        return True

    def set_movement_mode(self, movement_mode: object) -> bool:
        """Send one hot movement-mode request without entering text edit mode."""

        if self.in_flight or self.restart_required or self._outcome_unknown:
            return False
        connection = self._connection
        if connection is None:
            self._local_error(
                "E_COMMAND_UNAVAILABLE", "Game commands are unavailable for this run"
            )
            return False
        try:
            command = MovementModeSet(movement_mode)
        except CommandParseError as exc:
            self._local_error(exc.code, exc.message)
            return False
        self._sequence += 1
        request = GameCommandRequest(
            session=self._session,
            sequence=self._sequence,
            request_id=f"cmd-{os.urandom(16).hex()}",
            command=command,
        )
        payload = encode_command_request(request)
        try:
            sent = connection.send(payload)
        except BlockingIOError as exc:
            self._local_error(
                "E_COMMAND_SEND", f"Could not send movement mode: {exc}"
            )
            return False
        except OSError as exc:
            self._close_channel()
            self._local_error(
                "E_COMMAND_SEND", f"Could not send movement mode: {exc}"
            )
            return False
        if sent != len(payload):
            self._close_channel()
            self._local_error(
                "E_COMMAND_SEND",
                f"Partial movement-mode packet write: sent {sent}/{len(payload)}",
            )
            return False
        self._pending = request
        self._pending_warning = None
        self._result_revision += 1
        self.status = "pending"
        self.ok = None
        self.code = None
        self.message = f"Switching movement mode to {command.movement_mode}"
        self.warning = None
        self.restart_required = False
        self.data = None
        self.last_request_id = request.request_id
        return True

    def set_motion_setting(self, path: object, value: object) -> bool:
        """Send one hot motion-setting request without entering text edit mode."""

        if self.in_flight or self.restart_required or self._outcome_unknown:
            return False
        connection = self._connection
        if connection is None:
            self._local_error(
                "E_COMMAND_UNAVAILABLE", "Game commands are unavailable for this run"
            )
            return False
        try:
            command = MotionSettingSet(path, value)
        except CommandParseError as exc:
            self._local_error(exc.code, exc.message)
            return False
        self._sequence += 1
        request = GameCommandRequest(
            session=self._session,
            sequence=self._sequence,
            request_id=f"cmd-{os.urandom(16).hex()}",
            command=command,
        )
        payload = encode_command_request(request)
        try:
            sent = connection.send(payload)
        except BlockingIOError as exc:
            self._local_error(
                "E_COMMAND_SEND", f"Could not send motion setting: {exc}"
            )
            return False
        except OSError as exc:
            self._close_channel()
            self._local_error(
                "E_COMMAND_SEND", f"Could not send motion setting: {exc}"
            )
            return False
        if sent != len(payload):
            self._close_channel()
            self._local_error(
                "E_COMMAND_SEND",
                f"Partial motion-setting packet write: sent {sent}/{len(payload)}",
            )
            return False
        self._pending = request
        self._pending_warning = None
        self._result_revision += 1
        self.status = "pending"
        self.ok = None
        self.code = None
        self.message = f"Applying motion setting {command.path}"
        self.warning = None
        self.restart_required = False
        self.data = None
        self.last_request_id = request.request_id
        return True

    def set_runtime_pause(self, target: object, *, expected_epoch: object) -> bool:
        """Send one runtime-confirmed control pause/resume request."""

        if self.in_flight or self.restart_required or self._outcome_unknown:
            return False
        connection = self._connection
        if connection is None:
            self._local_error(
                "E_COMMAND_UNAVAILABLE", "Game commands are unavailable for this run"
            )
            self._runtime_pause = self._runtime_pause_unavailable()
            return False
        try:
            command = RuntimePauseSet(target, expected_epoch)
        except CommandParseError as exc:
            self._local_error(exc.code, exc.message)
            self._mark_runtime_pause_fault(exc.message)
            return False
        self._sequence += 1
        request = GameCommandRequest(
            session=self._session,
            sequence=self._sequence,
            request_id=f"cmd-{os.urandom(16).hex()}",
            command=command,
        )
        payload = encode_command_request(request)
        try:
            sent = connection.send(payload)
        except BlockingIOError as exc:
            self._local_error("E_COMMAND_SEND", f"Could not send pause command: {exc}")
            self._mark_runtime_pause_fault(str(exc))
            return False
        except OSError as exc:
            self._close_channel()
            self._local_error("E_COMMAND_SEND", f"Could not send pause command: {exc}")
            self._mark_runtime_pause_fault(str(exc))
            return False
        if sent != len(payload):
            self._close_channel()
            self._local_error(
                "E_COMMAND_SEND",
                f"Partial pause command packet write: sent {sent}/{len(payload)}",
            )
            self._mark_runtime_pause_fault("partial pause command packet write")
            return False
        self._pending = request
        self._pending_warning = None
        self._result_revision += 1
        self.status = "pending"
        self.ok = None
        self.code = None
        self.message = (
            "Pausing Matrix runtime controls"
            if command.target == "paused"
            else "Resuming Matrix runtime controls"
        )
        self.warning = None
        self.restart_required = False
        self.data = None
        self.last_request_id = request.request_id
        assert command.expected_epoch is not None
        self._runtime_pause = self._runtime_pause_pending(
            command.target,
            epoch=command.expected_epoch,
        )
        return True

    def poll(self) -> bool:
        """Receive at most one exact response; never resend a pending request."""

        connection = self._connection
        if connection is None:
            return False
        try:
            payload = connection.recv(MAX_COMMAND_PACKET_BYTES + 1)
        except BlockingIOError:
            return False
        except OSError as exc:
            self._protocol_failure(f"Game command channel failed: {exc}")
            return True
        if not payload:
            if self.restart_required:
                self._close_channel()
                return True
            self._protocol_failure("Game command runtime closed its channel")
            return True
        try:
            response = decode_command_response(payload)
        except CommandProtocolError as exc:
            self._protocol_failure(f"Invalid game command response: {exc}")
            return True
        pending = self._pending
        if pending is None:
            self._protocol_failure("Received an unsolicited game command response")
            return True
        if (
            response.session != pending.session
            or response.sequence != pending.sequence
            or response.request_id != pending.request_id
        ):
            self._protocol_failure("Game command response identity did not match")
            return True
        self._pending = None
        warning = self._pending_warning
        self._pending_warning = None
        self._result_revision += 1
        self._outcome_unknown = False
        self.ok = response.ok
        self.code = response.code
        self.message = response.message
        self.warning = warning
        self.restart_required = response.restart_required
        self.data = dict(response.data) if response.data is not None else None
        self.last_request_id = response.request_id
        coerced_pause = self._coerce_runtime_pause_from_response_data(self.data)
        if coerced_pause is not None:
            self._runtime_pause = coerced_pause
        elif isinstance(pending.command, RuntimePauseSet):
            self._mark_runtime_pause_fault(response.message)
        if response.ok and response.restart_required:
            self.status = "restarting"
        else:
            self.status = "success" if response.ok else "error"
        return True

    def mapping(self) -> dict[str, object]:
        return {
            "available": self.available,
            "editing": self.editing,
            "in_flight": self.in_flight,
            "status": self.status,
            "request_id": self.last_request_id,
            "sequence": self._sequence,
            "result_revision": self._result_revision,
            "ok": self.ok,
            "code": self.code,
            "message": self.message,
            "warning": self.warning,
            "restart_required": self.restart_required,
            "outcome_unknown": self.outcome_unknown,
            "data": self.data,
            "runtime_pause": dict(self._runtime_pause),
        }

    def close(self) -> None:
        # The provider can receive SIGTERM after the runtime closed its side
        # of the socket but before the next frame polls EOF.  First drain one
        # already-buffered response; if the request is still unresolved, keep
        # its correlation id and report a terminal outcome-unknown instead of
        # silently erasing the in-flight command during cleanup.
        if self._pending is not None:
            self.poll()
        if self._pending is not None:
            self._protocol_failure(
                "Game command provider stopped before the runtime response"
            )
            return
        self._pending_warning = None
        self._close_channel()


class CalibrationOverlaySupervisor:
    """Own the X11 overlay and its private pointer-intent socket."""

    _MAX_INTENT_PACKET_BYTES = 2048
    _ALLOWED_ACTIONS = frozenset(
        {
            "profile_local",
            "profile_remote",
            "speed_down",
            "speed_up",
            "apply_return",
            "functions_open_dir",
            "navigation_refresh",
        }
        | _MOVEMENT_MODE_ACTIONS
        | set(_MOTION_PANEL_ACTIONS)
        | set(_VIDEO_PANEL_ACTIONS)
        | _UI_PANEL_ACTIONS
    )

    def __init__(
        self,
        *,
        state_file: Path,
        display_name: str | None,
        expected_ue_pid: int,
        script: Path | None = None,
        python: str = sys.executable,
        startup_timeout_s: float = 3.0,
    ) -> None:
        self.state_file = state_file
        self.ready_file = state_file.with_name(f".{state_file.name}.overlay-status.json")
        self.display_name = display_name
        self.expected_ue_pid = expected_ue_pid
        self.script = script or Path(__file__).with_name(
            "matrix_calibration_overlay.py"
        )
        self.python = python
        self.startup_timeout_s = startup_timeout_s
        self.process: subprocess.Popen[bytes] | None = None
        self._action_socket: socket.socket | None = None
        self._action_session = os.urandom(16).hex()
        self._last_action_sequence = 0

    def start(self, initial_state: dict[str, object] | None = None) -> None:
        if not self.script.is_file():
            raise RuntimeError(f"calibration overlay is missing: {self.script}")
        self.state_file.parent.mkdir(parents=True, exist_ok=True)
        for stale in (self.state_file, self.ready_file):
            try:
                stale.unlink()
            except FileNotFoundError:
                pass
        _atomic_json(
            self.state_file,
            {"active": False, **(initial_state or {}), "version": 1},
        )
        parent_socket, child_socket = socket.socketpair(
            socket.AF_UNIX,
            socket.SOCK_SEQPACKET,
        )
        parent_socket.setblocking(False)
        self._action_socket = parent_socket
        command = [
            self.python,
            # -I ignores PYTHON* environment variables, including the
            # launcher's PYTHONDONTWRITEBYTECODE guard.  Keep isolation, but
            # make the no-bytecode contract an interpreter option so the
            # overlay cannot contaminate the locked runtime venv and block a
            # subsequent F9 generation.
            "-B",
            "-I",
            "-u",
            os.fspath(self.script),
            "--state-file",
            os.fspath(self.state_file),
            "--status-file",
            os.fspath(self.ready_file),
            "--expected-ue-pid",
            str(self.expected_ue_pid),
            "--expected-parent-pid",
            str(os.getpid()),
            "--action-fd",
            str(child_socket.fileno()),
            "--action-session",
            self._action_session,
        ]
        if self.display_name:
            command.extend(("--display", self.display_name))
        try:
            try:
                self.process = subprocess.Popen(
                    command,
                    cwd=self.script.parent.parent,
                    stdin=subprocess.DEVNULL,
                    pass_fds=(child_socket.fileno(),),
                )
            except Exception:
                parent_socket.close()
                self._action_socket = None
                raise
        finally:
            child_socket.close()
        try:
            deadline = time.monotonic() + self.startup_timeout_s
            while time.monotonic() < deadline:
                code = self.process.poll()
                if code is not None:
                    raise RuntimeError(
                        "calibration overlay exited during startup "
                        f"with code {code}"
                    )
                status = _read_json_object(self.ready_file)
                if status is not None and status.get("ready") is True:
                    return
                time.sleep(0.02)
            raise RuntimeError("calibration overlay did not become ready in time")
        except Exception:
            self.close()
            raise

    def publish(self, payload: dict[str, object]) -> None:
        self.ensure_running()
        _atomic_json(self.state_file, {"version": 1, **payload})

    def ensure_running(self) -> None:
        if self.process is None:
            raise RuntimeError("calibration overlay was not started")
        code = self.process.poll()
        if code is not None:
            raise RuntimeError(f"calibration overlay exited with code {code}")

    @staticmethod
    def _strict_intent_object(
        pairs: list[tuple[str, object]],
    ) -> dict[str, object]:
        result: dict[str, object] = {}
        for key, value in pairs:
            if key in result:
                raise RuntimeError(f"duplicate overlay intent field {key!r}")
            result[key] = value
        return result

    def drain_intents(self) -> tuple[OverlayIntent, ...]:
        """Drain bounded, versioned intents from the known overlay child."""

        connection = self._action_socket
        if connection is None:
            raise RuntimeError("calibration overlay action channel is unavailable")
        intents: list[OverlayIntent] = []
        for _ in range(32):
            try:
                payload = connection.recv(self._MAX_INTENT_PACKET_BYTES + 1)
            except BlockingIOError:
                break
            if not payload:
                self.ensure_running()
                raise RuntimeError("calibration overlay action channel closed")
            if len(payload) > self._MAX_INTENT_PACKET_BYTES:
                raise RuntimeError("calibration overlay intent packet is oversized")
            try:
                value = json.loads(
                    payload.decode("utf-8"),
                    object_pairs_hook=self._strict_intent_object,
                    parse_constant=lambda token: (_ for _ in ()).throw(
                        RuntimeError(f"invalid overlay JSON constant {token}")
                    ),
                )
            except (UnicodeDecodeError, json.JSONDecodeError, RuntimeError) as exc:
                raise RuntimeError("invalid calibration overlay intent packet") from exc
            if not isinstance(value, dict):
                raise RuntimeError("invalid calibration overlay intent schema")
            sequence = value.get("sequence")
            if (
                value.get("version") != 1
                or value.get("session") != self._action_session
                or type(sequence) is not int
                or sequence <= self._last_action_sequence
            ):
                raise RuntimeError("invalid calibration overlay intent identity")
            kind = value.get("kind")
            if kind == "action":
                if set(value) != {
                    "version",
                    "session",
                    "sequence",
                    "kind",
                    "action",
                } or value.get("action") not in self._ALLOWED_ACTIONS:
                    raise RuntimeError("invalid calibration overlay action intent")
                intent = OverlayIntent(kind="action", action=value["action"])
            elif kind == "command_edit":
                if set(value) != {
                    "version",
                    "session",
                    "sequence",
                    "kind",
                    "active",
                } or type(value.get("active")) is not bool:
                    raise RuntimeError("invalid calibration overlay command-edit intent")
                intent = OverlayIntent(kind="command_edit", active=value["active"])
            elif kind == "command_submit":
                command = value.get("command")
                if (
                    set(value)
                    != {
                        "version",
                        "session",
                        "sequence",
                        "kind",
                        "command",
                    }
                    or not isinstance(command, str)
                    or len(command) > MAX_COMMAND_CHARS
                ):
                    raise RuntimeError("invalid calibration overlay command-submit intent")
                intent = OverlayIntent(kind="command_submit", command=command)
            elif kind == "command_quick_submit":
                command = value.get("command")
                if (
                    set(value)
                    != {
                        "version",
                        "session",
                        "sequence",
                        "kind",
                        "command",
                    }
                    or not isinstance(command, str)
                    or len(command) > MAX_COMMAND_CHARS
                ):
                    raise RuntimeError(
                        "invalid calibration overlay command-quick-submit intent"
                    )
                intent = OverlayIntent(kind="command_quick_submit", command=command)
            elif kind == "font_size":
                raw_font_size = value.get("font_size")
                if set(value) != {
                    "version",
                    "session",
                    "sequence",
                    "kind",
                    "font_size",
                }:
                    raise RuntimeError("invalid calibration overlay font-size intent")
                try:
                    font_size = canonical_font_size(raw_font_size)
                except ValueError as exc:
                    raise RuntimeError(
                        "invalid calibration overlay font-size intent"
                    ) from exc
                intent = OverlayIntent(kind="font_size", font_size=font_size)
            elif kind == "runtime_pause":
                pause_target = value.get("pause_target")
                expected_epoch = value.get("expected_epoch")
                if (
                    set(value)
                    != {
                        "version",
                        "session",
                        "sequence",
                        "kind",
                        "pause_target",
                        "expected_epoch",
                    }
                    or pause_target not in {"paused", "running"}
                    or type(expected_epoch) is not int
                    or not 0 <= expected_epoch <= MAX_RUNTIME_PAUSE_EPOCH
                ):
                    raise RuntimeError("invalid calibration overlay runtime-pause intent")
                intent = OverlayIntent(
                    kind="runtime_pause",
                    pause_target=pause_target,
                    expected_epoch=expected_epoch,
                )
            else:
                raise RuntimeError("invalid calibration overlay intent kind")
            self._last_action_sequence = sequence
            intents.append(intent)
        return tuple(intents)

    def close(self) -> None:
        process = self.process
        self.process = None
        action_socket = self._action_socket
        self._action_socket = None
        if process is None:
            if action_socket is not None:
                action_socket.close()
            return
        try:
            current = _read_json_object(self.state_file) or {}
            _atomic_json(self.state_file, {**current, "active": False})
        except OSError:
            pass
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)
        if action_socket is not None:
            action_socket.close()


def _wait_until_frame(
    now: float,
    deadline: float,
    *,
    keep_running: Callable[[], bool],
    sleeper: Callable[[float], None] = time.sleep,
    clock: Callable[[], float] = time.monotonic,
) -> tuple[bool, float]:
    """Wait for one frame deadline and re-check shutdown after the wait."""
    if now < deadline:
        sleeper(deadline - now)
        now = clock()
    return keep_running(), now


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", type=Path, default=DEFAULT_SOCKET)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument(
        "--input-source", choices=("auto", "keyboard", "gamepad"), default="auto"
    )
    parser.add_argument("--display", default=os.environ.get("DISPLAY"))
    parser.add_argument(
        "--expected-ue-pid",
        type=int,
        help="Require X11 focus to belong to this supervised UE process",
    )
    parser.add_argument(
        "--focus-title",
        default=r"(zsibot|matrix|unreal)",
        help="Case-insensitive title regex; UE PID binding is always enforced",
    )
    parser.add_argument(
        "--allow-any-focus",
        action="store_true",
        help="Disable only the title regex; exact UE PID binding remains active",
    )
    parser.add_argument(
        "--grab-ui-keys",
        action="store_true",
        help=(
            "Passively grab ESC/Q/E on X11 so Matrix consumes them before "
            "the cooked UE window can treat Q as an application quit key"
        ),
    )
    parser.add_argument(
        "--look-button",
        choices=("left", "middle", "right"),
        default="left",
        help="Native Matrix documents left-drag; used by X11 yaw sources",
    )
    parser.add_argument("--gamepad", default=None, help="Linux js device; auto if omitted")
    parser.add_argument("--gamepad-left-x-axis", type=int, default=0)
    parser.add_argument("--gamepad-left-y-axis", type=int, default=1)
    parser.add_argument("--gamepad-right-x-axis", type=int, default=3)
    parser.add_argument("--gamepad-right-y-axis", type=int, default=4)
    parser.add_argument(
        "--camera-yaw-source",
        choices=(
            "x11-mirror",
            "x11-core-gated",
            "x11-absolute",
            "ue-final-pov",
            "carla",
            "fixed",
        ),
        default="fixed",
        help=(
            "fixed is safe until runtime probing succeeds; x11-mirror requires "
            "XI2 raw button edges; x11-core-gated experimentally gates XI2 raw "
            "motion with the X11 core button; x11-absolute mirrors root-pointer "
            "deltas; ue-final-pov reads the supervised PlayerCameraManager "
            "final POV; the X11 sources do not read back the visible camera"
        ),
    )
    parser.add_argument(
        "--ue-camera-state-file",
        type=Path,
        help="Supervised fresh PlayerCameraManager final-POV state",
    )
    parser.add_argument(
        "--initial-camera-yaw-deg",
        type=float,
        default=0.0,
        help="Initial provider/UE yaw before provider-to-SONIC sign and offset",
    )
    parser.add_argument("--mouse-sensitivity-deg", type=float, default=0.12)
    parser.add_argument(
        "--camera-yaw-sign",
        type=int,
        choices=(-1, 1),
        default=-1,
        help="Provider-to-SONIC yaw sign, determined by the direction probe",
    )
    parser.add_argument(
        "--camera-yaw-offset-deg",
        type=float,
        default=0.0,
        help="Provider-to-SONIC zero-frame offset, determined by calibration",
    )
    parser.add_argument("--carla-host", default="127.0.0.1")
    parser.add_argument("--carla-port", type=int, default=2000)
    parser.add_argument(
        "--gamepad-look-yaw-rate-deg-s",
        type=float,
        default=120.0,
        help="CARLA spectator yaw rate at full right-stick deflection",
    )
    parser.add_argument(
        "--gamepad-look-pitch-rate-deg-s",
        type=float,
        default=90.0,
        help="CARLA spectator pitch rate at full right-stick deflection",
    )
    parser.add_argument("--gamepad-look-deadzone", type=float, default=0.12)
    parser.add_argument("--gamepad-look-min-pitch-deg", type=float, default=-80.0)
    parser.add_argument("--gamepad-look-max-pitch-deg", type=float, default=60.0)
    parser.add_argument("--keyboard-camera-look-rate-deg-s", type=float, default=120.0)
    parser.add_argument("--status-file", type=Path)
    parser.add_argument(
        "--calibration-state-file",
        type=Path,
        help=(
            "Live ESC calibration/overlay state; defaults beside --socket in "
            "the launcher's private runtime directory"
        ),
    )
    parser.add_argument(
        "--mouse-settings-file",
        type=Path,
        default=default_settings_file(),
    )
    parser.add_argument("--ui-settings-file", type=Path)
    parser.add_argument("--motion-settings-file", type=Path)
    parser.add_argument("--video-settings-file", type=Path)
    parser.add_argument(
        "--function-directory",
        type=Path,
        help="Directory containing newline-based Matrix .mcfunction files",
    )
    parser.add_argument(
        "--applied-video-settings-json",
        help="Launcher-applied video settings JSON for pending-restart display",
    )
    parser.add_argument(
        "--applied-mouse-profile",
        choices=(PROFILE_LOCAL, PROFILE_REMOTE),
        default=PROFILE_LOCAL,
    )
    parser.add_argument("--applied-mouse-speed-scale", type=float, default=1.0)
    parser.add_argument("--restart-request-file", type=Path)
    parser.add_argument("--restart-capability-file", type=Path)
    parser.add_argument("--restart-launcher-pid", type=int)
    parser.add_argument(
        "--game-command-fd",
        type=int,
        help="Inherited private SOCK_SEQPACKET channel for typed ESC commands",
    )
    engine_input_socket = os.environ.get("MATRIX_ENGINE_INPUT_SOCKET")
    engine_input_capability = os.environ.get("MATRIX_ENGINE_INPUT_CAPABILITY_FILE")
    parser.add_argument(
        "--engine-input-socket",
        type=Path,
        default=Path(engine_input_socket) if engine_input_socket else None,
        help="Private Matrix engine-input socket used for arrow-key camera look",
    )
    parser.add_argument(
        "--engine-input-capability-file",
        type=Path,
        default=Path(engine_input_capability) if engine_input_capability else None,
        help="Private Matrix engine-input capability for arrow-key camera look",
    )
    parser.add_argument("--max-seconds", type=float, default=0.0)
    parser.add_argument(
        "--dry-run", action="store_true", help="Print canonical packets; do not connect"
    )
    return parser.parse_args()


def _validate_args(args: argparse.Namespace) -> None:
    if not args.socket.is_absolute():
        raise SystemExit("--socket must be an absolute path")
    if not args.socket.parent.is_dir():
        raise SystemExit(f"--socket parent does not exist: {args.socket.parent}")
    if not math.isfinite(args.rate_hz) or not 1.0 <= args.rate_hz <= 200.0:
        raise SystemExit("--rate-hz must be finite and in [1, 200]")
    for name in (
        "gamepad_left_x_axis",
        "gamepad_left_y_axis",
        "gamepad_right_x_axis",
        "gamepad_right_y_axis",
    ):
        if not 0 <= getattr(args, name) <= 255:
            raise SystemExit(f"--{name.replace('_', '-')} must be in [0, 255]")
    for name in (
        "initial_camera_yaw_deg",
        "mouse_sensitivity_deg",
        "camera_yaw_offset_deg",
        "gamepad_look_yaw_rate_deg_s",
        "gamepad_look_pitch_rate_deg_s",
        "gamepad_look_min_pitch_deg",
        "gamepad_look_max_pitch_deg",
        "keyboard_camera_look_rate_deg_s",
    ):
        if not math.isfinite(getattr(args, name)):
            raise SystemExit(f"--{name.replace('_', '-')} must be finite")
    for name in (
        "gamepad_look_yaw_rate_deg_s",
        "gamepad_look_pitch_rate_deg_s",
        "keyboard_camera_look_rate_deg_s",
    ):
        if getattr(args, name) <= 0.0:
            raise SystemExit(f"--{name.replace('_', '-')} must be positive")
    if (
        not math.isfinite(args.gamepad_look_deadzone)
        or not 0.0 <= args.gamepad_look_deadzone < 1.0
    ):
        raise SystemExit("--gamepad-look-deadzone must be finite and in [0, 1)")
    if args.gamepad_look_min_pitch_deg >= args.gamepad_look_max_pitch_deg:
        raise SystemExit("gamepad camera pitch limits must be ordered")
    if args.max_seconds < 0.0 or not math.isfinite(args.max_seconds):
        raise SystemExit("--max-seconds must be finite and non-negative")
    if not 1 <= args.carla_port <= 65535:
        raise SystemExit("--carla-port must be in [1, 65535]")
    if args.expected_ue_pid is not None and args.expected_ue_pid <= 1:
        raise SystemExit("--expected-ue-pid must be greater than 1")
    if args.expected_ue_pid is None and not args.dry_run:
        raise SystemExit("--expected-ue-pid is required outside --dry-run")
    if args.camera_yaw_source == "ue-final-pov":
        if args.expected_ue_pid is None:
            raise SystemExit("--expected-ue-pid is required for ue-final-pov")
        if args.ue_camera_state_file is None:
            raise SystemExit("--ue-camera-state-file is required for ue-final-pov")
        if not args.ue_camera_state_file.is_absolute():
            raise SystemExit("--ue-camera-state-file must be absolute")
    if args.calibration_state_file is not None:
        if not args.calibration_state_file.is_absolute():
            raise SystemExit("--calibration-state-file must be an absolute path")
        if not args.calibration_state_file.parent.is_dir():
            raise SystemExit(
                "--calibration-state-file parent does not exist: "
                f"{args.calibration_state_file.parent}"
            )
    if not args.mouse_settings_file.is_absolute():
        raise SystemExit("--mouse-settings-file must be absolute")
    for name in (
        "ui_settings_file",
        "motion_settings_file",
        "video_settings_file",
    ):
        path = getattr(args, name)
        if path is not None and not path.is_absolute():
            raise SystemExit(f"--{name.replace('_', '-')} must be absolute")
    if args.applied_video_settings_json is not None:
        try:
            value = json.loads(args.applied_video_settings_json)
        except json.JSONDecodeError as exc:
            raise SystemExit("--applied-video-settings-json must be JSON") from exc
        if not isinstance(value, dict):
            raise SystemExit("--applied-video-settings-json must be a JSON object")
    try:
        AppliedMouseSettings(
            profile=args.applied_mouse_profile,
            effective_scale=args.applied_mouse_speed_scale,
        )
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    restart_values = (
        args.restart_request_file,
        args.restart_capability_file,
        args.restart_launcher_pid,
    )
    if any(value is not None for value in restart_values) and not all(
        value is not None for value in restart_values
    ):
        raise SystemExit("restart request file, capability, and launcher PID are all-or-none")
    for name in ("restart_request_file", "restart_capability_file"):
        path = getattr(args, name)
        if path is not None and not path.is_absolute():
            raise SystemExit(f"--{name.replace('_', '-')} must be absolute")
    if args.function_directory is not None and not args.function_directory.is_absolute():
        raise SystemExit("--function-directory must be absolute")
    if args.restart_launcher_pid is not None and args.restart_launcher_pid <= 1:
        raise SystemExit("--restart-launcher-pid must be greater than one")
    game_command_fd = getattr(args, "game_command_fd", None)
    if game_command_fd is not None:
        if game_command_fd < 0:
            raise SystemExit("--game-command-fd must be non-negative")
        try:
            os.fstat(game_command_fd)
        except OSError as exc:
            raise SystemExit(f"--game-command-fd is not open: {exc}") from exc
    engine_values = (args.engine_input_socket, args.engine_input_capability_file)
    if any(value is not None for value in engine_values):
        if not all(value is not None for value in engine_values):
            raise SystemExit(
                "--engine-input-socket and --engine-input-capability-file are all-or-none"
            )
        assert args.engine_input_socket is not None
        assert args.engine_input_capability_file is not None
        if not args.engine_input_socket.is_absolute():
            raise SystemExit("--engine-input-socket must be absolute")
        if not args.engine_input_capability_file.is_absolute():
            raise SystemExit("--engine-input-capability-file must be absolute")


def main() -> int:
    args = _parse_args()
    _validate_args(args)
    function_directory = (
        args.function_directory.resolve()
        if args.function_directory is not None
        else None
    )
    raw_build_info = os.environ.get("MATRIX_BUILD_INFO_JSON")
    try:
        build_info = parse_build_info_json(raw_build_info or "")
    except BuildInfoError as exc:
        build_info = unavailable_build_info(
            profile="local",
            scene_id=0,
            control_source="game",
            error=f"Launch provenance unavailable: {exc}",
            launch_available=False,
        )
    applied_mouse = AppliedMouseSettings(
        profile=args.applied_mouse_profile,
        effective_scale=args.applied_mouse_speed_scale,
    )
    loaded_mouse = load_settings(args.mouse_settings_file)
    mouse_settings = MouseSettingsController(
        path=args.mouse_settings_file,
        desired=loaded_mouse.settings,
        load_status=loaded_mouse.status,
        load_error=loaded_mouse.error,
    )
    if args.ui_settings_file is None:
        ui_settings = UiSettingsController(
            path=None,
            desired=UiSettings(),
            load_status="disabled",
            load_error=None,
        )
    else:
        loaded_ui = load_ui_settings(args.ui_settings_file)
        ui_settings = UiSettingsController(
            path=args.ui_settings_file,
            desired=loaded_ui.settings,
            load_status=loaded_ui.status,
            load_error=loaded_ui.error,
        )
    motion_store: MotionSettingsStore | None = None
    applied_motion: MotionSettings | None = None
    motion_settings_change_count = 0
    motion_settings_error: str | None = None
    if args.motion_settings_file is not None:
        try:
            motion_store = MotionSettingsStore(args.motion_settings_file)
            applied_motion = motion_store.settings
        except (
            MotionSettingsError,
            MotionSettingsPersistenceError,
            OSError,
            ValueError,
        ) as exc:
            motion_settings_error = str(exc)
    video_store: VideoSettingsStore | None = None
    applied_video_runtime: dict[str, object] | None = None
    video_settings_change_count = 0
    video_settings_error: str | None = None
    if args.video_settings_file is not None:
        try:
            video_store = VideoSettingsStore(args.video_settings_file)
            if args.applied_video_settings_json is not None:
                decoded_video = json.loads(args.applied_video_settings_json)
                if isinstance(decoded_video, dict):
                    applied_video_runtime = dict(decoded_video)
            if applied_video_runtime is None:
                applied_video_runtime = video_store.settings.runtime_mapping()
        except (
            VideoSettingsError,
            VideoSettingsPersistenceError,
            OSError,
            ValueError,
            json.JSONDecodeError,
        ) as exc:
            video_settings_error = str(exc)
    restart_requester = RuntimeRestartRequester(
        request_file=args.restart_request_file,
        capability_file=args.restart_capability_file,
        launcher_pid=args.restart_launcher_pid,
    )
    apply_restart_key = ApplyRestartKey()
    apply_return = ApplyReturnController()
    try:
        input_source = effective_input_source(
            args.input_source, args.camera_yaw_source
        )
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    focus_pattern = None if args.allow_any_focus else args.focus_title
    try:
        x11 = X11KeyboardMouse(
            display_name=args.display,
            focus_title_pattern=focus_pattern,
            expected_ue_pid=args.expected_ue_pid,
            look_button=args.look_button,
            capture_raw_motion=captures_xi2_drag_boundaries(
                args.camera_yaw_source
            ),
            capture_absolute_motion=args.camera_yaw_source == "x11-absolute",
            raw_button_gate=(
                "x11-core-level"
                if args.camera_yaw_source == "x11-core-gated"
                else "xi2-events"
            ),
            grab_ui_keys=args.grab_ui_keys,
        )
    except (OSError, RuntimeError, re.error) as exc:
        raise SystemExit(f"Matrix game-control input cannot initialize X11: {exc}") from exc
    overlay: CalibrationOverlaySupervisor | None = None
    if args.expected_ue_pid is not None:
        calibration_state_file = args.calibration_state_file or args.socket.with_name(
            f"{args.socket.name}.calibration.json"
        )
        overlay = CalibrationOverlaySupervisor(
            state_file=calibration_state_file,
            display_name=args.display,
            expected_ue_pid=args.expected_ue_pid,
        )
    gamepad = LinuxJoystick(
        args.gamepad,
        left_x_axis=args.gamepad_left_x_axis,
        left_y_axis=args.gamepad_left_y_axis,
        right_x_axis=args.gamepad_right_x_axis,
        right_y_axis=args.gamepad_right_y_axis,
    )
    tracker = CameraYawTracker(
        math.radians(args.initial_camera_yaw_deg),
        mouse_radians_per_pixel=math.radians(
            args.mouse_sensitivity_deg * applied_mouse.effective_scale
        ),
        # Right-stick look is applied only by the CARLA driver below and comes
        # back as an absolute observed yaw.  The tracker never integrates an
        # unobserved gamepad angle.
        gamepad_radians_per_second=0.0,
    )
    carla_reader: CarlaSpectatorYawReader | None = None
    if args.camera_yaw_source == "carla":
        carla_reader = CarlaSpectatorYawReader(
            args.carla_host,
            args.carla_port,
            look_yaw_rate_rad_s=math.radians(args.gamepad_look_yaw_rate_deg_s),
            look_pitch_rate_rad_s=math.radians(
                args.gamepad_look_pitch_rate_deg_s
            ),
            look_deadzone=args.gamepad_look_deadzone,
            minimum_pitch_rad=math.radians(args.gamepad_look_min_pitch_deg),
            maximum_pitch_rad=math.radians(args.gamepad_look_max_pitch_deg),
        )
    ue_final_pov_reader: UeFinalPovYawReader | None = None
    if args.camera_yaw_source == "ue-final-pov":
        assert args.ue_camera_state_file is not None
        assert args.expected_ue_pid is not None
        try:
            ue_final_pov_reader = UeFinalPovYawReader(
                args.ue_camera_state_file,
                expected_ue_pid=args.expected_ue_pid,
            )
        except (ImportError, OSError, RuntimeError, ValueError) as exc:
            raise SystemExit(
                f"Matrix game-control input cannot initialize UE final-POV reader: {exc}"
            ) from exc
    publisher = None if args.dry_run else UnixSeqpacketPublisher(args.socket)
    try:
        game_command_client = GameCommandClient(getattr(args, "game_command_fd", None))
    except (OSError, ValueError) as exc:
        raise SystemExit(
            f"Matrix game-control input cannot initialize command channel: {exc}"
        ) from exc
    keyboard_camera_integrator = KeyboardCameraLookIntegrator()
    engine_camera_worker: EngineCameraLookWorker | None = None
    if args.engine_input_socket is not None:
        assert args.engine_input_capability_file is not None
        engine_camera_worker = EngineCameraLookWorker(
            args.engine_input_socket,
            args.engine_input_capability_file,
            button=args.look_button,
        )
        engine_camera_worker.start()
    calibration = CalibrationModeController()
    shortcut_arming = StartupShortcutArming()
    movement_mode_cycle_key = MovementModeCycleKey()

    running = True

    def stop(_signum: int, _frame: object) -> None:
        nonlocal running
        running = False

    previous_handlers = {
        signum: signal.signal(signum, stop) for signum in (signal.SIGINT, signal.SIGTERM)
    }
    started = time.monotonic()
    previous_frame = started
    next_frame = started
    # A fresh client must not start again at zero while a still-running core
    # remembers the preceding peer's sequence.  Host monotonic nanoseconds are
    # below the signed 63-bit protocol ceiling for centuries of uptime.
    sequence = initial_sequence()
    sampled_frames = 0
    sent_frames = 0
    last_snapshot: InputSnapshot | None = None
    last_keyboard: KeyboardMouseSample | None = None
    exit_reason = "unknown"
    return_code = 0
    previous_gamepad_connected: bool | None = None
    next_overlay_heartbeat = started
    last_teleport_rejections = 0
    calibration_neutral_frames = 0
    final_pov_observation: UeFinalPovObservation | None = None
    current_movement_mode = (
        motion_store.settings.movement_mode
        if motion_store is not None
        else DEFAULT_MOVEMENT_MODE
    )
    exception_detail: dict[str, object] | None = None
    overlay_error: str | None = None
    overlay_failures = 0
    function_directory_open_count = 0
    function_directory_open_error: str | None = None
    provider_yaw = tracker.yaw
    camera_yaw = transform_camera_yaw(
        provider_yaw,
        sign=args.camera_yaw_sign,
        offset_rad=math.radians(args.camera_yaw_offset_deg),
    )
    effective_mouse_sensitivity = (
        args.mouse_sensitivity_deg * applied_mouse.effective_scale
    )
    sensitivity_telemetry = mirror_sensitivity_mapping(
        args.camera_yaw_source,
        base_deg_per_unit=args.mouse_sensitivity_deg,
        effective_deg_per_unit=effective_mouse_sensitivity,
    )
    source_claim = camera_source_claim(args.camera_yaw_source)

    def overlay_runtime_mapping() -> dict[str, object]:
        return {
            "configured": args.expected_ue_pid is not None,
            "available": overlay is not None,
            "failures": overlay_failures,
            "last_error": overlay_error,
        }

    def live_function_library_mapping() -> dict[str, object]:
        return function_library_mapping(
            function_directory,
            open_count=function_directory_open_count,
            open_error=function_directory_open_error,
        )

    def disable_overlay(context: str, exc: Exception) -> bool:
        """Disable the ESC overlay without killing the movement input provider."""

        nonlocal overlay, overlay_error, overlay_failures, calibration_neutral_frames
        overlay_failures += 1
        overlay_error = f"{context}: {type(exc).__name__}: {exc}"
        print(
            "matrix-game-control-input WARN disabling calibration overlay: "
            f"{overlay_error}",
            file=sys.stderr,
            flush=True,
        )
        if overlay is not None:
            try:
                overlay.close()
            except Exception as close_exc:
                print(
                    "matrix-game-control-input WARN calibration overlay close "
                    f"failed: {type(close_exc).__name__}: {close_exc}",
                    file=sys.stderr,
                    flush=True,
                )
            overlay = None
        left_calibration = calibration.exit()
        if left_calibration:
            calibration_neutral_frames = 0
        return bool(game_command_client.panel_closed() or left_calibration)

    try:
        if overlay is not None:
            try:
                overlay.start(
                    {
                        **source_claim,
                        "mouse_settings": mouse_settings.live_mapping(applied_mouse),
                        "ui_settings": ui_settings.live_mapping(),
                        "motion_settings": motion_settings_live_mapping(
                            motion_store,
                            applied=applied_motion,
                            change_count=motion_settings_change_count,
                            persistence_error=motion_settings_error,
                        ),
                        "video_settings": video_settings_live_mapping(
                            video_store,
                            applied_runtime=applied_video_runtime,
                            change_count=video_settings_change_count,
                            persistence_error=video_settings_error,
                        ),
                        "restart": restart_requester.mapping(),
                        "apply_return": apply_return.mapping(),
                        "command_console": game_command_client.mapping(),
                        "function_library": live_function_library_mapping(),
                        "build_info": build_info,
                        "celestial_navigation": celestial_navigation_mapping(build_info),
                        "strategy_loadout": locked_sonic_strategy_loadout(),
                        "movement_mode": current_movement_mode,
                        "mirror_sensitivity": sensitivity_telemetry,
                        "pointer": x11.pointer_telemetry,
                        "camera_yaw": camera_yaw_telemetry(
                            args.camera_yaw_source,
                            provider_yaw_rad=provider_yaw,
                            sonic_yaw_rad=camera_yaw,
                        ),
                        "ue_final_pov": ue_final_pov_telemetry(None),
                        "keyboard_camera": keyboard_camera_telemetry(
                            engine_camera_worker,
                            keyboard_camera_integrator,
                            arrow_keys_available=x11.arrow_keys_available,
                            rate_deg_s=args.keyboard_camera_look_rate_deg_s,
                        ),
                        "calibration_overlay": overlay_runtime_mapping(),
                    }
                )
            except (OSError, RuntimeError) as exc:
                disable_overlay("startup", exc)
        while running:
            now = time.monotonic()
            if args.max_seconds > 0.0 and now - started >= args.max_seconds:
                exit_reason = "max_seconds"
                break
            still_running, now = _wait_until_frame(
                now,
                next_frame,
                keep_running=lambda: running,
            )
            if not still_running:
                exit_reason = "signal"
                break
            dt = _clamp(now - previous_frame, 0.0, 0.25)
            previous_frame = now
            next_frame = max(next_frame + 1.0 / args.rate_hz, now)

            command_state_changed = game_command_client.poll()
            if (
                command_state_changed
                and game_command_client.ok is True
                and isinstance(game_command_client.code, str)
                and game_command_client.code.startswith("OK_MOTION_SETTING")
                and motion_store is not None
            ):
                try:
                    motion_store.reload_if_changed()
                    applied_motion = motion_store.settings
                    motion_settings_error = None
                except (MotionSettingsError, OSError, ValueError) as exc:
                    motion_settings_error = str(exc)
            if (
                command_state_changed
                and game_command_client.ok is True
                and game_command_client.data is not None
                and isinstance(game_command_client.data.get("movement_mode"), str)
            ):
                try:
                    (
                        current_movement_mode,
                        applied_motion,
                        movement_mode_synced,
                        movement_mode_sync_error,
                    ) = sync_confirmed_movement_mode(
                        game_command_client.data["movement_mode"],
                        store=motion_store,
                        applied=applied_motion,
                    )
                    if movement_mode_synced:
                        motion_settings_change_count += 1
                    motion_settings_error = movement_mode_sync_error
                except ValueError:
                    pass
            raw_keyboard = x11.poll()
            last_keyboard = raw_keyboard
            raw_pad = gamepad.poll(now)
            panel_was_active = calibration.active
            overlay_state_changed = False
            if overlay is not None:
                try:
                    panel_intents = overlay.drain_intents()
                except RuntimeError as exc:
                    overlay_state_changed = disable_overlay("drain_intents", exc)
                    panel_intents = ()
            else:
                panel_intents = ()
            shortcuts_armed = shortcut_arming.update(
                escape_pressed=raw_keyboard.escape,
                restart_pressed=raw_keyboard.apply_restart,
            )
            command_state_changed = bool(command_state_changed or overlay_state_changed)
            panel_escape = game_command_client.panel_escape_pressed(
                raw_keyboard.escape if shortcuts_armed else False,
                # A begin/end pair can both arrive inside one 20 ms provider
                # frame.  The overlay still owned that physical Escape even if
                # the provider had not published the intermediate edit state.
                editor_owned_this_frame=any(
                    intent.kind == "command_edit" for intent in panel_intents
                )
                and game_command_client.available,
            )
            calibration_toggled = calibration.update(
                escape_pressed=panel_escape,
                ue_focused=raw_keyboard.focused,
            )
            if calibration_toggled or not calibration.active:
                calibration_neutral_frames = 0
            if panel_was_active and not calibration.active:
                command_state_changed = bool(
                    game_command_client.panel_closed() or command_state_changed
                )
            neutral_frame_ready = bool(
                calibration_neutral_frames >= 1
                and (publisher is None or publisher.connected)
            )
            panel_actions: list[str] = []
            panel_font_sizes: list[int] = []
            for intent in panel_intents:
                if intent.kind == "action":
                    assert intent.action is not None
                    panel_actions.append(intent.action)
                    continue
                if intent.kind == "font_size":
                    assert intent.font_size is not None
                    panel_font_sizes.append(intent.font_size)
                    continue
                if intent.kind == "runtime_pause":
                    assert intent.pause_target is not None
                    assert intent.expected_epoch is not None
                    command_state_changed = bool(
                        game_command_client.set_runtime_pause(
                            intent.pause_target,
                            expected_epoch=intent.expected_epoch,
                        )
                        or command_state_changed
                    )
                    apply_return.cancel_pending()
                    continue
                if intent.kind == "command_edit":
                    assert intent.active is not None
                    command_state_changed = bool(
                        game_command_client.set_editing(
                            intent.active,
                            panel_active=calibration.active,
                            restart_requested=restart_requester.requested,
                        )
                        or command_state_changed
                    )
                    if intent.active:
                        apply_return.cancel_pending()
                    continue
                assert intent.kind in {"command_submit", "command_quick_submit"}
                assert intent.command is not None
                command_submitted = game_command_client.submit(
                    intent.command,
                    calibration_active=calibration.active,
                    neutral_frame_ready=neutral_frame_ready,
                    restart_requested=restart_requester.requested,
                    require_editing=(intent.kind == "command_submit"),
                )
                # Local parse/gate failures also change the visible result.
                command_state_changed = True
                if command_submitted:
                    apply_return.cancel_pending()
            command_controls_blocked = bool(
                game_command_client.editing
                or game_command_client.in_flight
                or game_command_client.restart_required
                or game_command_client.outcome_unknown
                # A command-edit transition owns this entire sampled frame.
                # XQueryKeymap was polled before the intent drain, so allowing
                # settings shortcuts immediately after command_edit(false)
                # would turn a same-frame M/-/+/Enter/F9 press into a fresh
                # settings edge even though the overlay still owned it.
                or any(
                    intent.kind
                    in {
                        "command_edit",
                        "command_submit",
                        "command_quick_submit",
                        "runtime_pause",
                    }
                    for intent in panel_intents
                )
            )
            keyboard_panel_active = bool(
                calibration.active
                and raw_keyboard.focused
                and not restart_requester.requested
                and not command_controls_blocked
            )
            mouse_settings_changed = mouse_settings.update(
                active=keyboard_panel_active,
                mode_pressed=raw_keyboard.mouse_mode,
                slower_pressed=raw_keyboard.mouse_speed_down,
                faster_pressed=raw_keyboard.mouse_speed_up,
            )
            ui_settings_changed = False
            motion_settings_changed = False
            video_settings_changed = False
            function_library_changed = False
            settings_action_active = bool(
                calibration.active
                and not restart_requester.requested
                and not command_controls_blocked
            )
            for panel_action in panel_actions:
                if panel_action in {
                    "profile_local",
                    "profile_remote",
                    "speed_down",
                    "speed_up",
                }:
                    mouse_settings_changed = bool(
                        mouse_settings.apply_panel_action(
                            panel_action,
                            active=settings_action_active,
                        )
                        or mouse_settings_changed
                    )
                elif panel_action in _UI_PANEL_ACTIONS:
                    ui_settings_changed = bool(
                        ui_settings.apply_panel_action(
                            panel_action,
                            active=settings_action_active,
                        )
                        or ui_settings_changed
                    )
                elif panel_action in _MOTION_PANEL_ACTIONS:
                    if settings_action_active and motion_store is not None:
                        path, direction = _MOTION_PANEL_ACTIONS[panel_action]
                        try:
                            modification = motion_store.step(path, direction)
                            motion_settings_error = None
                            if modification.changed:
                                motion_settings_change_count += 1
                                if game_command_client.set_motion_setting(
                                    modification.path,
                                    modification.value,
                                ):
                                    applied_motion = modification.settings
                                    command_state_changed = True
                                    apply_return.cancel_pending()
                            motion_settings_changed = True
                        except (
                            MotionSettingsError,
                            MotionSettingsPersistenceError,
                            OSError,
                            ValueError,
                        ) as exc:
                            motion_settings_error = str(exc)
                            motion_settings_changed = True
                elif panel_action in _VIDEO_PANEL_ACTIONS:
                    if settings_action_active and video_store is not None:
                        field, direction = _VIDEO_PANEL_ACTIONS[panel_action]
                        try:
                            modification = video_store.step(field, direction)
                            video_settings_error = None
                            if modification.changed:
                                video_settings_change_count += 1
                            video_settings_changed = True
                        except (
                            VideoSettingsError,
                            VideoSettingsPersistenceError,
                            OSError,
                            ValueError,
                        ) as exc:
                            video_settings_error = str(exc)
                            video_settings_changed = True
                elif panel_action == "functions_open_dir":
                    if settings_action_active:
                        opened, open_error = open_function_directory(
                            function_directory
                        )
                        if opened:
                            function_directory_open_count += 1
                        function_directory_open_error = open_error
                        function_library_changed = True
                elif panel_action in _MOVEMENT_MODE_ACTIONS:
                    if settings_action_active:
                        movement_mode = panel_action.removeprefix("movement_mode_")
                        try:
                            movement_mode = validate_movement_mode(movement_mode)
                        except ValueError:
                            continue
                        command_state_changed = bool(
                            game_command_client.set_movement_mode(movement_mode)
                            or command_state_changed
                        )
                        if game_command_client.in_flight:
                            current_movement_mode = movement_mode
                        apply_return.cancel_pending()
            for font_size in panel_font_sizes:
                ui_settings_changed = bool(
                    ui_settings.apply_font_size(
                        font_size,
                        active=settings_action_active,
                    )
                    or ui_settings_changed
                )
            motion_pending_restart = bool(
                motion_store is not None
                and applied_motion is not None
                and motion_store.settings != applied_motion
            )
            video_pending_restart = bool(
                video_store is not None
                and applied_video_runtime is not None
                and video_store.settings.runtime_mapping() != applied_video_runtime
            )
            settings_pending_restart = bool(
                mouse_settings.pending_restart(applied_mouse)
                or motion_pending_restart
                or video_pending_restart
            )
            settings_persistence_error = first_settings_error(
                mouse_settings.persistence_error,
                motion_settings_error,
                video_settings_error,
            )
            restart_requested = apply_restart_key.update(
                pressed=raw_keyboard.apply_restart,
                calibration_active=keyboard_panel_active,
                neutral_frame_ready=neutral_frame_ready,
                pending_restart=settings_pending_restart,
                persistence_ok=settings_persistence_error is None,
                requester=restart_requester,
            )
            left_calibration, ui_restart_requested = apply_return.update(
                enter_pressed=raw_keyboard.apply_return,
                clicked=(
                    "apply_return" in panel_actions
                    and not command_controls_blocked
                ),
                ue_focused=raw_keyboard.focused and not command_controls_blocked,
                panel_was_active=panel_was_active,
                calibration=calibration,
                neutral_frame_ready=(
                    neutral_frame_ready
                    and not command_controls_blocked
                ),
                pending_restart=settings_pending_restart,
                persistence_error=settings_persistence_error,
                requester=restart_requester,
            )
            restart_requested = restart_requested or ui_restart_requested
            if left_calibration:
                calibration_neutral_frames = 0
                command_state_changed = bool(
                    game_command_client.panel_closed() or command_state_changed
                )
            calibration_interlock_active = calibration_interlock_required(
                panel_was_active=panel_was_active,
                panel_active=calibration.active,
            )
            keyboard, pad = apply_calibration_interlock(
                raw_keyboard,
                raw_pad,
                # The ButtonRelease/Enter exit frame can still carry the final
                # held-pointer delta sampled before the UI intent was drained.
                # Keep that whole frame neutral; normal input resumes next frame.
                active=calibration_interlock_active,
            )
            movement_inputs_neutral = bool(
                not any(
                    (
                        raw_keyboard.w,
                        raw_keyboard.a,
                        raw_keyboard.s,
                        raw_keyboard.d,
                        raw_keyboard.q,
                        raw_keyboard.e,
                    )
                )
                and math.hypot(raw_pad.right, raw_pad.forward) <= 0.15
            )
            movement_mode_cycle_edge = movement_mode_cycle_key.update(
                raw_keyboard.movement_mode_cycle,
                enabled=bool(
                    raw_keyboard.focused
                    and not calibration.active
                    and not command_controls_blocked
                    and movement_inputs_neutral
                    and game_command_client.available
                ),
            )
            if movement_mode_cycle_edge:
                next_mode = next_movement_mode(current_movement_mode)
                command_state_changed = bool(
                    game_command_client.set_movement_mode(next_mode)
                    or command_state_changed
                )
                if game_command_client.in_flight:
                    current_movement_mode = next_mode
            camera_arrows_active = keyboard_camera_arrow_active(keyboard)
            camera_dx, camera_dy = keyboard_camera_integrator.update(
                keyboard,
                dt=dt,
                rate_deg_s=args.keyboard_camera_look_rate_deg_s,
                degrees_per_pixel=effective_mouse_sensitivity,
                enabled=bool(
                    engine_camera_worker is not None
                    and keyboard.focused
                    and x11.arrow_keys_available
                    and not calibration.active
                ),
            )
            if engine_camera_worker is not None and (camera_dx or camera_dy):
                engine_camera_worker.submit(camera_dx, camera_dy)
            elif engine_camera_worker is not None and (
                not keyboard.focused or not camera_arrows_active
            ):
                engine_camera_worker.cancel_pending()
            pointer_telemetry = x11.pointer_telemetry
            teleport_rejections = int(pointer_telemetry["teleport_rejections"])
            input_available = gamepad_input_available(
                input_source,
                connected=pad.connected,
                previous_connected=previous_gamepad_connected,
            )
            previous_gamepad_connected = pad.connected
            drive_gamepad_camera = bool(
                carla_reader is not None
                and keyboard.focused
                and input_available
                and pad.connected
                and input_source in {"auto", "gamepad"}
            )
            observed_yaw = (
                carla_reader.drive(
                    now=now,
                    dt=dt,
                    look_yaw=pad.look_yaw if drive_gamepad_camera else 0.0,
                    look_pitch=pad.look_pitch if drive_gamepad_camera else 0.0,
                )
                if carla_reader is not None
                else None
            )
            final_pov_observation = (
                ue_final_pov_reader.read(now)
                if ue_final_pov_reader is not None
                else None
            )
            if final_pov_observation is not None:
                observed_yaw = final_pov_observation.yaw_rad
            camera_available = (
                args.camera_yaw_source not in {"carla", "ue-final-pov"}
                or observed_yaw is not None
            )
            provider_yaw = tracker.update(
                dt=dt,
                mouse_dx=(
                    keyboard.mouse_dx
                    if args.camera_yaw_source
                    in {"x11-mirror", "x11-core-gated", "x11-absolute"}
                    and args.input_source != "gamepad"
                    else 0.0
                ),
                gamepad_look_yaw=0.0,
                observed_yaw_rad=observed_yaw,
            )
            camera_yaw = transform_camera_yaw(
                provider_yaw,
                sign=args.camera_yaw_sign,
                offset_rad=math.radians(args.camera_yaw_offset_deg),
            )
            # Publish input counters and the yaw produced from that exact same
            # poll.  Telemetry stays downstream of every safety decision and
            # never feeds the tracker or snapshot interlocks.
            if overlay is not None:
                try:
                    overlay.ensure_running()
                    if (
                        calibration_toggled
                        or left_calibration
                        or bool(panel_intents)
                        or command_state_changed
                        or mouse_settings_changed
                        or ui_settings_changed
                        or motion_settings_changed
                        or video_settings_changed
                        or function_library_changed
                        or restart_requested
                        or teleport_rejections != last_teleport_rejections
                        or now >= next_overlay_heartbeat
                    ):
                        overlay.publish(
                            {
                                **source_claim,
                                "active": calibration.active,
                                "toggle_count": calibration.toggle_count,
                                "updated_monotonic_s": now,
                                "expected_ue_pid": args.expected_ue_pid,
                                "raw_ue_focused": raw_keyboard.focused,
                                "snapshot_forced_unfocused": calibration_interlock_active,
                                "shortcuts_armed": shortcuts_armed,
                                "neutral_frames": calibration_neutral_frames,
                                "mouse_settings": mouse_settings.live_mapping(
                                    applied_mouse
                                ),
                                "ui_settings": ui_settings.live_mapping(),
                                "motion_settings": motion_settings_live_mapping(
                                    motion_store,
                                    applied=applied_motion,
                                    change_count=motion_settings_change_count,
                                    persistence_error=motion_settings_error,
                                ),
                                "video_settings": video_settings_live_mapping(
                                    video_store,
                                    applied_runtime=applied_video_runtime,
                                    change_count=video_settings_change_count,
                                    persistence_error=video_settings_error,
                                ),
                                "restart": restart_requester.mapping(),
                                "apply_return": apply_return.mapping(),
                                "command_console": game_command_client.mapping(),
                                "function_library": live_function_library_mapping(),
                                "build_info": build_info,
                                "celestial_navigation": celestial_navigation_mapping(build_info),
                                "strategy_loadout": locked_sonic_strategy_loadout(),
                                "movement_mode": current_movement_mode,
                                "mirror_sensitivity": sensitivity_telemetry,
                                "camera_yaw": camera_yaw_telemetry(
                                    args.camera_yaw_source,
                                    provider_yaw_rad=provider_yaw,
                                    sonic_yaw_rad=camera_yaw,
                                ),
                                "ue_final_pov": ue_final_pov_telemetry(
                                    final_pov_observation
                                ),
                                "keyboard_camera": keyboard_camera_telemetry(
                                    engine_camera_worker,
                                    keyboard_camera_integrator,
                                    arrow_keys_available=x11.arrow_keys_available,
                                    rate_deg_s=args.keyboard_camera_look_rate_deg_s,
                                ),
                                "calibration_overlay": overlay_runtime_mapping(),
                                "pointer": pointer_telemetry,
                            }
                        )
                        next_overlay_heartbeat = now + 1.0
                except RuntimeError as exc:
                    disable_overlay("publish", exc)
            last_teleport_rejections = teleport_rejections
            snapshot = build_snapshot(
                sequence=sequence,
                timestamp_monotonic_s=now,
                keyboard=keyboard,
                gamepad=pad,
                input_source=input_source,
                camera_yaw_rad=camera_yaw,
                camera_available=camera_available,
                input_available=input_available,
            )
            last_snapshot = snapshot
            neutral_delivered = False
            if publisher is None:
                print(encode_input_packet(snapshot).decode("ascii"), flush=True)
                sent_frames += 1
                neutral_delivered = True
            elif publisher.send(snapshot, now=now):
                sent_frames += 1
                neutral_delivered = True
            if calibration.active:
                calibration_neutral_frames = (
                    calibration_neutral_frames + 1 if neutral_delivered else 0
                )
            sequence += 1
            sampled_frames += 1
        if exit_reason == "unknown":
            exit_reason = "signal"
    except Exception as exc:
        exception_detail = {
            "type": type(exc).__name__,
            "message": str(exc),
            "traceback": traceback.format_exc(),
        }
        exit_reason = f"error:{type(exc).__name__}:{exc}"
        print(f"matrix-game-control-input ERROR {exc}", file=sys.stderr, flush=True)
        traceback.print_exc(file=sys.stderr)
        return_code = 1
    finally:
        # A focused=false release is immediate; the core's independent 0.15 s
        # deadman threshold remains authoritative if the connection is gone.
        if publisher is not None and last_snapshot is not None:
            release = InputSnapshot(
                sequence=sequence,
                timestamp_monotonic_s=time.monotonic(),
                focused=False,
                camera_yaw_rad=last_snapshot.camera_yaw_rad,
                keys=KeySnapshot(
                    w=False,
                    a=False,
                    s=False,
                    d=False,
                    q=False,
                    e=False,
                    v=False,
                    x=False,
                    ctrl=False,
                    alt=False,
                    shift=False,
                ),
                move_stick=MoveStickSnapshot(0.0, 0.0),
            )
            publisher.send(release, now=time.monotonic())
        # Resolve a response already queued at the shutdown boundary, or mark
        # a successfully sent but unacknowledged command outcome-unknown.  This
        # must happen before the final status snapshot is serialized.
        game_command_client.close()
        if engine_camera_worker is not None:
            try:
                engine_camera_worker.close()
            except RuntimeError as exc:
                print(
                    f"matrix-game-control-input WARN {exc}",
                    file=sys.stderr,
                    flush=True,
                )
        _atomic_json(
            args.status_file,
            {
                **source_claim,
                "completed": return_code == 0,
                "exit_reason": exit_reason,
                "sampled_frames": sampled_frames,
                "sent_frames": sent_frames,
                "socket": os.fspath(args.socket),
                "requested_input_source": args.input_source,
                "effective_input_source": input_source,
                "mouse_settings": mouse_settings.live_mapping(applied_mouse),
                "ui_settings": ui_settings.live_mapping(),
                "motion_settings": motion_settings_live_mapping(
                    motion_store,
                    applied=applied_motion,
                    change_count=motion_settings_change_count,
                    persistence_error=motion_settings_error,
                ),
                "video_settings": video_settings_live_mapping(
                    video_store,
                    applied_runtime=applied_video_runtime,
                    change_count=video_settings_change_count,
                    persistence_error=video_settings_error,
                ),
                "mirror_sensitivity": sensitivity_telemetry,
                "camera_yaw": camera_yaw_telemetry(
                    args.camera_yaw_source,
                    provider_yaw_rad=provider_yaw,
                    sonic_yaw_rad=camera_yaw,
                ),
                "ue_final_pov": ue_final_pov_telemetry(
                    final_pov_observation
                ),
                "keyboard_camera": keyboard_camera_telemetry(
                    engine_camera_worker,
                    keyboard_camera_integrator,
                    arrow_keys_available=x11.arrow_keys_available,
                    rate_deg_s=args.keyboard_camera_look_rate_deg_s,
                ),
                "calibration_overlay": overlay_runtime_mapping(),
                "exception": exception_detail,
                "restart": restart_requester.mapping(),
                "apply_return": apply_return.mapping(),
                "command_console": game_command_client.mapping(),
                "function_library": live_function_library_mapping(),
                "build_info": build_info,
                "celestial_navigation": celestial_navigation_mapping(build_info),
                "strategy_loadout": locked_sonic_strategy_loadout(),
                "gamepad_camera": {
                    "driver": "carla-spectator"
                    if args.camera_yaw_source == "carla"
                    else None,
                    "yaw_rate_deg_s": args.gamepad_look_yaw_rate_deg_s,
                    "pitch_rate_deg_s": args.gamepad_look_pitch_rate_deg_s,
                    "deadzone": args.gamepad_look_deadzone,
                    "minimum_pitch_deg": args.gamepad_look_min_pitch_deg,
                    "maximum_pitch_deg": args.gamepad_look_max_pitch_deg,
                    "write_readback_tolerance_deg": math.degrees(
                        DEFAULT_CARLA_WRITE_READBACK_TOLERANCE_RAD
                    ),
                },
                "gamepad": gamepad.path,
                "focus": {
                    "expected_ue_pid": args.expected_ue_pid,
                    "raw_ue_focused": last_keyboard.focused
                    if last_keyboard is not None
                    else False,
                    "actual_pid": last_keyboard.focus_pid
                    if last_keyboard is not None
                    else None,
                    "title": last_keyboard.focus_title
                    if last_keyboard is not None
                    else None,
                },
                "calibration": {
                    "active": calibration.active,
                    "toggle_count": calibration.toggle_count,
                    "snapshot_forced_unfocused": calibration.active,
                    "state_file": os.fspath(overlay.state_file)
                    if overlay is not None
                    else None,
                },
                "pointer": x11.pointer_telemetry,
                "last_snapshot": last_snapshot.to_mapping()
                if last_snapshot is not None
                else None,
            },
        )
        gamepad.close()
        if overlay is not None:
            overlay.close()
        x11.close()
        if publisher is not None:
            publisher.close()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
