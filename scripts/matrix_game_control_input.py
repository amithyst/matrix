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

    With ``fixed`` or ``x11-mirror`` the adapter cannot observe any native UE
    right-stick camera response.  Auto therefore degrades to keyboard-only,
    while an explicit gamepad request fails instead of silently diverging.
    """
    if requested not in {"auto", "keyboard", "gamepad"}:
        raise ValueError(f"unsupported input source: {requested}")
    if camera_yaw_source not in {"fixed", "x11-mirror", "carla"}:
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

    This is only a mirror of the packaged UI: it is truthful when the same
    pointer delta is consumed by UE and its sensitivity has been calibrated.
    It does not itself rotate the visible camera.
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
        maximum_mouse_delta: float = 200.0,
        library: Any | None = None,
    ) -> None:
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

    @property
    def pointer_telemetry(self) -> dict[str, object]:
        return {
            "teleport_rejections": self._teleport_rejections,
            "last_teleport_delta": list(self._last_teleport_delta)
            if self._last_teleport_delta is not None
            else None,
            "maximum_mouse_delta_px": self._maximum_mouse_delta,
        }

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
        mouse_dx = 0.0
        mouse_dy = 0.0
        # Attribute the interval to the state at its beginning.  On the release
        # sample, movement since the preceding held sample was still consumed
        # by UE before the button-up event and must not disappear from the yaw
        # mirror. The first press remains a fresh baseline because pre-press
        # pointer motion cannot be separated from drag motion by polling alone.
        if self._previous_look_pressed and pointer is not None:
            if self._previous_pointer is not None:
                raw_dx = pointer[0] - self._previous_pointer[0]
                raw_dy = pointer[1] - self._previous_pointer[1]
                # Relative-mode UE windows commonly warp the server cursor to
                # their centre.  Absolute-coordinate remote desktops can then
                # reassert the client position, producing a teleport loop.
                # Saturating that jump (the old behaviour) still injected as
                # much as 200 px into the mirrored camera yaw.  Reject the
                # whole discontinuity and use the new position as the next
                # baseline instead; ordinary in-range motion is unchanged.
                if max(abs(raw_dx), abs(raw_dy)) > self._maximum_mouse_delta:
                    self._teleport_rejections += 1
                    self._last_teleport_delta = (raw_dx, raw_dy)
                else:
                    mouse_dx = float(raw_dx)
                    mouse_dy = float(raw_dy)
        self._previous_pointer = pointer
        self._previous_look_pressed = look_pressed

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
            camera_dragging=look_pressed and focused,
            focused=focused,
            focus_title=focus_title,
            focus_pid=focus_pid,
        )

    def close(self) -> None:
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
        help="Native Matrix documents left-drag; only used by x11-mirror",
    )
    parser.add_argument("--gamepad", default=None, help="Linux js device; auto if omitted")
    parser.add_argument("--gamepad-left-x-axis", type=int, default=0)
    parser.add_argument("--gamepad-left-y-axis", type=int, default=1)
    parser.add_argument("--gamepad-right-x-axis", type=int, default=3)
    parser.add_argument("--gamepad-right-y-axis", type=int, default=4)
    parser.add_argument(
        "--camera-yaw-source",
        choices=("x11-mirror", "carla", "fixed"),
        default="fixed",
        help=(
            "fixed is safe until runtime probing succeeds; x11-mirror requires "
            "measured UE mouse sensitivity and does not drive the visible camera"
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
    try:
        if overlay is not None:
            overlay.start(
                {
                    "mouse_settings": mouse_settings.live_mapping(applied_mouse),
                    "restart": restart_requester.mapping(),
                    "apply_return": apply_return.mapping(),
                    "mirror_sensitivity": {
                        "base_deg_per_px": args.mouse_sensitivity_deg,
                        "effective_deg_per_px": (
                            args.mouse_sensitivity_deg
                            * applied_mouse.effective_scale
                        ),
                    },
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
                            "mirror_sensitivity": {
                                "base_deg_per_px": args.mouse_sensitivity_deg,
                                "effective_deg_per_px": (
                                    args.mouse_sensitivity_deg
                                    * applied_mouse.effective_scale
                                ),
                            },
                            "pointer": pointer_telemetry,
                        }
                    )
                    next_overlay_heartbeat = now + 1.0
            last_teleport_rejections = teleport_rejections
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
                    if args.camera_yaw_source == "x11-mirror"
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
                "completed": return_code == 0,
                "exit_reason": exit_reason,
                "sampled_frames": sampled_frames,
                "sent_frames": sent_frames,
                "socket": os.fspath(args.socket),
                "requested_input_source": args.input_source,
                "effective_input_source": input_source,
                "camera_yaw_source": args.camera_yaw_source,
                "mouse_settings": mouse_settings.live_mapping(applied_mouse),
                "mirror_sensitivity": {
                    "base_deg_per_px": args.mouse_sensitivity_deg,
                    "effective_deg_per_px": (
                        args.mouse_sensitivity_deg
                        * applied_mouse.effective_scale
                    ),
                },
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
