#!/usr/bin/env python3
"""Capture local Matrix UI input and publish strict game-control snapshots.

This is the operator-side adapter for :mod:`matrix_game_control`.  It does not
publish SONIC planner messages: the physics runtime remains the only owner of
that native wire.  Complete input snapshots instead travel over a local Linux
``AF_UNIX/SOCK_SEQPACKET`` connection, using the schema and encoder owned by the
control core.

The default backend polls X11 with ``libX11`` and Linux ``/dev/input/js*``
directly, so no pygame, evdev, or Python Xlib package is required.  A CARLA
spectator yaw reader is optional and imported only when explicitly selected.
"""

from __future__ import annotations

import argparse
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
import socket
import struct
import subprocess
import sys
import tempfile
import time
from typing import Any, Callable, Protocol

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


DEFAULT_SOCKET = Path(
    os.environ.get(
        "MATRIX_GAME_INPUT_SOCKET",
        os.fspath(
            Path(os.environ.get("XDG_RUNTIME_DIR", tempfile.gettempdir()))
            / f"matrix-game-control-{os.getuid()}.sock"
        ),
    )
)
_JS_EVENT = struct.Struct("IhBB")
_JS_EVENT_BUTTON = 0x01
_JS_EVENT_AXIS = 0x02
_JS_EVENT_INIT = 0x80
DEFAULT_CARLA_WRITE_READBACK_TOLERANCE_RAD = math.radians(0.5)


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
    ctrl: bool = False
    shift: bool = False
    escape: bool = False
    mouse_mode: bool = False
    mouse_speed_down: bool = False
    mouse_speed_up: bool = False
    apply_restart: bool = False
    apply_return: bool = False
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
            ctrl=self.ctrl and movement_enabled,
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
    explicit gamepad request fails instead of silently diverging.
    """
    if requested not in {"auto", "keyboard", "gamepad"}:
        raise ValueError(f"unsupported input source: {requested}")
    if camera_yaw_source not in {
        "fixed",
        "x11-mirror",
        "x11-core-gated",
        "x11-absolute",
        "carla",
    }:
        raise ValueError(f"unsupported camera yaw source: {camera_yaw_source}")
    if camera_yaw_source == "carla":
        return requested
    if requested == "gamepad":
        raise ValueError("gamepad input requires an observed CARLA camera yaw")
    return "keyboard" if requested == "auto" else requested


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
        "experimental": source in {"x11-core-gated", "x11-absolute"},
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
    _fields_ = (("type", ctypes.c_int), ("pad", ctypes.c_long * 24))


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
    _KEYSYMS = {
        "w": 0x0077,
        "a": 0x0061,
        "s": 0x0073,
        "d": 0x0064,
        "q": 0x0071,
        "e": 0x0065,
        "v": 0x0076,
        "ctrl_left": 0xFFE3,
        "ctrl_right": 0xFFE4,
        "shift_left": 0xFFE1,
        "shift_right": 0xFFE2,
        "escape": 0xFF1B,
        "mouse_mode": 0x006D,
        "mouse_speed_down": 0x002D,
        "mouse_speed_up": 0x003D,
        "apply_restart": 0xFFC6,
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
        if any(code <= 0 for code in self._keycodes.values()):
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
    def pointer_telemetry(self) -> dict[str, object]:
        telemetry = {
            "teleport_rejections": self._teleport_rejections,
            "last_teleport_delta": list(self._last_teleport_delta)
            if self._last_teleport_delta is not None
            else None,
            "maximum_mouse_delta_px": self._maximum_mouse_delta,
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
            "XFree": ([ctypes.c_void_p], ctypes.c_int),
            "XCloseDisplay": ([ctypes.c_void_p], ctypes.c_int),
        }
        for name, (argtypes, restype) in signatures.items():
            function = getattr(self._x11, name)
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

        focus = ctypes.c_ulong()
        revert = ctypes.c_int()
        if not self._x11.XGetInputFocus(
            self._display, ctypes.byref(focus), ctypes.byref(revert)
        ):
            return (False, None, frozenset())
        window = int(focus.value)
        if window <= 1:  # X11 None and PointerRoot sentinels
            return (False, None, frozenset())
        title = None
        process_ids: set[int] = set()
        for _ in range(12):
            if title is None:
                title = self._fetch_name(window)
            candidate_pid = self._window_pid(window)
            if candidate_pid is not None:
                process_ids.add(candidate_pid)
            parent = self._parent(window)
            if parent is None or parent == self._root:
                break
            window = parent
        return (True, title, frozenset(process_ids))

    def poll(self) -> KeyboardMouseSample:
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
                name: pressed[name]
                for name in ("w", "a", "s", "d", "q", "e", "v")
            },
            ctrl=pressed.get("ctrl_left", False)
            or pressed.get("ctrl_right", False),
            shift=pressed.get("shift_left", False)
            or pressed.get("shift_right", False),
            escape=pressed.get("escape", False),
            mouse_mode=pressed.get("mouse_mode", False),
            mouse_speed_down=pressed.get("mouse_speed_down", False),
            mouse_speed_up=pressed.get("mouse_speed_up", False),
            apply_restart=pressed.get("apply_restart", False),
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
        # Native Matrix documents held mouse-drag as temporary free camera.
        # Treat it like a focus interlock so camera-WASD cannot also walk G1.
        focused=(
            keyboard.focused
            and not keyboard.camera_dragging
            and camera_available
            and input_available
        ),
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


class CalibrationOverlaySupervisor:
    """Own the X11 overlay and its private pointer-intent socket."""

    _ALLOWED_ACTIONS = frozenset(
        {
            "profile_local",
            "profile_remote",
            "speed_down",
            "speed_up",
            "apply_return",
        }
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

    def drain_actions(self) -> tuple[str, ...]:
        """Drain bounded, versioned pointer intents from the known child."""

        connection = self._action_socket
        if connection is None:
            raise RuntimeError("calibration overlay action channel is unavailable")
        actions: list[str] = []
        for _ in range(32):
            try:
                payload = connection.recv(1024)
            except BlockingIOError:
                break
            if not payload:
                self.ensure_running()
                raise RuntimeError("calibration overlay action channel closed")
            if len(payload) >= 1024:
                raise RuntimeError("calibration overlay action packet is oversized")
            try:
                value = json.loads(payload.decode("ascii"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise RuntimeError("invalid calibration overlay action packet") from exc
            if not isinstance(value, dict) or set(value) != {
                "version",
                "session",
                "sequence",
                "action",
            }:
                raise RuntimeError("invalid calibration overlay action schema")
            sequence = value.get("sequence")
            action = value.get("action")
            if (
                value.get("version") != 1
                or value.get("session") != self._action_session
                or type(sequence) is not int
                or sequence <= self._last_action_sequence
                or action not in self._ALLOWED_ACTIONS
            ):
                raise RuntimeError("invalid calibration overlay action identity")
            self._last_action_sequence = sequence
            actions.append(action)
        return tuple(actions)

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
            "carla",
            "fixed",
        ),
        default="fixed",
        help=(
            "fixed is safe until runtime probing succeeds; x11-mirror requires "
            "XI2 raw button edges; x11-core-gated experimentally gates XI2 raw "
            "motion with the X11 core button; x11-absolute mirrors root-pointer "
            "deltas; none reads back or drives the visible camera"
        ),
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
    parser.add_argument(
        "--applied-mouse-profile",
        choices=(PROFILE_LOCAL, PROFILE_REMOTE),
        default=PROFILE_LOCAL,
    )
    parser.add_argument("--applied-mouse-speed-scale", type=float, default=1.0)
    parser.add_argument("--restart-request-file", type=Path)
    parser.add_argument("--restart-capability-file", type=Path)
    parser.add_argument("--restart-launcher-pid", type=int)
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
    ):
        if not math.isfinite(getattr(args, name)):
            raise SystemExit(f"--{name.replace('_', '-')} must be finite")
    for name in (
        "gamepad_look_yaw_rate_deg_s",
        "gamepad_look_pitch_rate_deg_s",
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
    if args.restart_launcher_pid is not None and args.restart_launcher_pid <= 1:
        raise SystemExit("--restart-launcher-pid must be greater than one")


def main() -> int:
    args = _parse_args()
    _validate_args(args)
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
            capture_raw_motion=args.camera_yaw_source
            in {"x11-mirror", "x11-core-gated"},
            capture_absolute_motion=args.camera_yaw_source == "x11-absolute",
            raw_button_gate=(
                "x11-core-level"
                if args.camera_yaw_source == "x11-core-gated"
                else "xi2-events"
            ),
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
    publisher = None if args.dry_run else UnixSeqpacketPublisher(args.socket)
    calibration = CalibrationModeController()
    shortcut_arming = StartupShortcutArming()

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
    try:
        if overlay is not None:
            overlay.start(
                {
                    **source_claim,
                    "mouse_settings": mouse_settings.live_mapping(applied_mouse),
                    "restart": restart_requester.mapping(),
                    "apply_return": apply_return.mapping(),
                    "mirror_sensitivity": sensitivity_telemetry,
                    "pointer": x11.pointer_telemetry,
                    "camera_yaw": camera_yaw_telemetry(
                        args.camera_yaw_source,
                        provider_yaw_rad=provider_yaw,
                        sonic_yaw_rad=camera_yaw,
                    ),
                }
            )
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

            raw_keyboard = x11.poll()
            last_keyboard = raw_keyboard
            raw_pad = gamepad.poll(now)
            shortcuts_armed = shortcut_arming.update(
                escape_pressed=raw_keyboard.escape,
                restart_pressed=raw_keyboard.apply_restart,
            )
            panel_was_active = calibration.active
            calibration_toggled = calibration.update(
                escape_pressed=raw_keyboard.escape if shortcuts_armed else False,
                ue_focused=raw_keyboard.focused,
            )
            if calibration_toggled or not calibration.active:
                calibration_neutral_frames = 0
            panel_actions = overlay.drain_actions() if overlay is not None else ()
            keyboard_panel_active = bool(
                calibration.active
                and raw_keyboard.focused
                and not restart_requester.requested
            )
            mouse_settings_changed = mouse_settings.update(
                active=keyboard_panel_active,
                mode_pressed=raw_keyboard.mouse_mode,
                slower_pressed=raw_keyboard.mouse_speed_down,
                faster_pressed=raw_keyboard.mouse_speed_up,
            )
            for panel_action in panel_actions:
                mouse_settings_changed = bool(
                    mouse_settings.apply_panel_action(
                        panel_action,
                        active=calibration.active and not restart_requester.requested,
                    )
                    or mouse_settings_changed
                )
            restart_requested = apply_restart_key.update(
                pressed=raw_keyboard.apply_restart,
                calibration_active=keyboard_panel_active,
                neutral_frame_ready=calibration_neutral_frames >= 1,
                pending_restart=mouse_settings.pending_restart(applied_mouse),
                persistence_ok=mouse_settings.persistence_error is None,
                requester=restart_requester,
            )
            left_calibration, ui_restart_requested = apply_return.update(
                enter_pressed=raw_keyboard.apply_return,
                clicked="apply_return" in panel_actions,
                ue_focused=raw_keyboard.focused,
                panel_was_active=panel_was_active,
                calibration=calibration,
                neutral_frame_ready=calibration_neutral_frames >= 1,
                pending_restart=mouse_settings.pending_restart(applied_mouse),
                persistence_error=mouse_settings.persistence_error,
                requester=restart_requester,
            )
            restart_requested = restart_requested or ui_restart_requested
            if left_calibration:
                calibration_neutral_frames = 0
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
            camera_available = args.camera_yaw_source != "carla" or observed_yaw is not None
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
                overlay.ensure_running()
                if (
                    calibration_toggled
                    or left_calibration
                    or bool(panel_actions)
                    or mouse_settings_changed
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
                            "restart": restart_requester.mapping(),
                            "apply_return": apply_return.mapping(),
                            "mirror_sensitivity": sensitivity_telemetry,
                            "camera_yaw": camera_yaw_telemetry(
                                args.camera_yaw_source,
                                provider_yaw_rad=provider_yaw,
                                sonic_yaw_rad=camera_yaw,
                            ),
                            "pointer": pointer_telemetry,
                        }
                    )
                    next_overlay_heartbeat = now + 1.0
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
            if calibration.active and neutral_delivered:
                calibration_neutral_frames += 1
            sequence += 1
            sampled_frames += 1
        if exit_reason == "unknown":
            exit_reason = "signal"
    except Exception as exc:
        exit_reason = f"error:{type(exc).__name__}"
        print(f"matrix-game-control-input ERROR {exc}", file=sys.stderr, flush=True)
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
                keys=KeySnapshot(False, False, False, False, False, False, False),
                move_stick=MoveStickSnapshot(0.0, 0.0),
            )
            publisher.send(release, now=time.monotonic())
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
                "mirror_sensitivity": sensitivity_telemetry,
                "camera_yaw": camera_yaw_telemetry(
                    args.camera_yaw_source,
                    provider_yaw_rad=provider_yaw,
                    sonic_yaw_rad=camera_yaw,
                ),
                "restart": restart_requester.mapping(),
                "apply_return": apply_return.mapping(),
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
