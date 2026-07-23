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
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
from typing import Any, Callable, Iterator, Mapping, Protocol

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
from matrix_ui_settings import (
    UiSettings,
    atomic_save_settings as atomic_save_ui_settings,
    default_settings_file as default_ui_settings_file,
    load_settings as load_ui_settings,
    step_font_scale,
)
from matrix_motion_settings import MotionSettings, MotionSettingsError
from matrix_video_settings import (
    VideoSettings,
    VideoSettingsError,
    VideoSettingsPersistenceError,
    VideoSettingsStore,
    default_settings_file as default_video_settings_file,
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
from matrix_external_control import (
    ExternalCommand,
    ExternalControlBroker,
    ExternalInputState,
    ExternalInputToken,
    ProviderGateTelemetry,
)
from matrix_celestial_navigation import (
    CelestialCatalog,
    CelestialNavigationError,
    DEFAULT_ASSET_MANIFEST_PATH,
    DEFAULT_CATALOG_PATH,
    load_catalog,
    probes_from_response,
)
from matrix_celestial_ephemeris import (
    CelestialEphemerisError,
    PersistentSimulationClock,
)
from matrix_celestial_visuals import (
    CarlaWeatherSample,
    CelestialVisualCatalog,
    CelestialVisualError,
    DEFAULT_VISUAL_CATALOG_PATH,
    load_visual_catalog,
)
from matrix_mc_commands import (
    CommandParseError,
    CommandProtocolError,
    CreativeSpawnItem,
    DataModifyInput,
    DataModifyNumber,
    GameCommandRequest,
    MAX_COMMAND_CHARS,
    MAX_COMMAND_PACKET_BYTES,
    PolicySlotAssignment,
    TeleportList,
    TeleportSelector,
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
_JS_EVENT = struct.Struct("IhBB")
_JS_EVENT_BUTTON = 0x01
_JS_EVENT_AXIS = 0x02
_JS_EVENT_INIT = 0x80
DEFAULT_CARLA_WRITE_READBACK_TOLERANCE_RAD = math.radians(0.5)


_X11_BAD_WINDOW = 3
_X11_ERROR_HANDLER_LOCK = threading.RLock()


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


_X11_ERROR_HANDLER = ctypes.CFUNCTYPE(
    ctypes.c_int,
    ctypes.c_void_p,
    ctypes.POINTER(_XErrorEvent),
)


@dataclass
class _X11FocusErrorScope:
    """Errors and window IDs owned by one synchronous focus-chain query."""

    windows: set[int]
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
    ctrl: bool = False
    alt: bool = False
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
            alt=self.alt and movement_enabled,
            shift=self.shift and movement_enabled,
        )


@dataclass(frozen=True)
class GamepadSample:
    forward: float = 0.0
    right: float = 0.0
    look_yaw: float = 0.0
    look_pitch: float = 0.0
    buttons_pressed: bool = False
    connected: bool = False


def physical_external_override_reason(
    keyboard: KeyboardMouseSample,
    gamepad: GamepadSample,
    *,
    move_deadzone: float = 0.15,
    look_deadzone: float = 0.12,
) -> str | None:
    """Return the local safety event that must revoke external authority."""

    for name, value in (
        ("move_deadzone", move_deadzone),
        ("look_deadzone", look_deadzone),
    ):
        if not math.isfinite(value) or not 0.0 <= value < 1.0:
            raise ValueError(f"{name} must be finite and in [0, 1)")

    if not keyboard.focused:
        return "focus_lost"
    if keyboard.escape:
        return "physical_escape"
    if any(
        bool(getattr(keyboard, name))
        for name in (
            "w",
            "a",
            "s",
            "d",
            "q",
            "e",
            "v",
            "ctrl",
            "alt",
            "shift",
            "mouse_mode",
            "mouse_speed_down",
            "mouse_speed_up",
            "apply_restart",
            "apply_return",
        )
    ):
        return "physical_keyboard"
    if (
        keyboard.camera_dragging
        or abs(keyboard.mouse_dx) > 1e-12
        or abs(keyboard.mouse_dy) > 1e-12
    ):
        return "physical_mouse"
    if gamepad.connected and (
        gamepad.buttons_pressed
        or abs(gamepad.forward) > move_deadzone
        or abs(gamepad.right) > move_deadzone
        or abs(gamepad.look_yaw) > look_deadzone
        or abs(gamepad.look_pitch) > look_deadzone
    ):
        return "physical_gamepad"
    return None


def external_input_samples(
    state: ExternalInputState,
    *,
    focus: KeyboardMouseSample,
    look_button: str,
) -> tuple[KeyboardMouseSample, GamepadSample]:
    """Convert a validated virtual full-state snapshot into provider samples."""

    if not isinstance(state, ExternalInputState):
        raise TypeError("external state must be ExternalInputState")
    if look_button not in {"left", "middle", "right"}:
        raise ValueError("external look button is invalid")
    keys = state.keyboard
    keyboard = KeyboardMouseSample(
        **{name: keys[name] for name in (
            "w", "a", "s", "d", "q", "e", "v", "ctrl", "alt", "shift"
        )},
        escape=keys["escape"],
        mouse_mode=keys["mouse_mode"],
        mouse_speed_down=keys["mouse_speed_down"],
        mouse_speed_up=keys["mouse_speed_up"],
        apply_restart=keys["apply_restart"],
        apply_return=keys["apply_return"],
        mouse_dx=state.mouse_dx,
        mouse_dy=state.mouse_dy,
        camera_dragging=state.mouse_buttons[look_button],
        focused=focus.focused,
        focus_title=focus.focus_title,
        focus_pid=focus.focus_pid,
    )
    axes = state.gamepad_axes
    gamepad = GamepadSample(
        forward=axes["forward"],
        right=axes["right"],
        look_yaw=axes["look_yaw"],
        look_pitch=axes["look_pitch"],
        buttons_pressed=any(state.gamepad_buttons.values()),
        connected=state.gamepad_connected,
    )
    return keyboard, gamepad


def external_active_input_device(state: ExternalInputState) -> str | None:
    """Classify the virtual device claim used by gate and final arbitration."""

    if not isinstance(state, ExternalInputState):
        raise TypeError("external input state is invalid")
    keyboard_active = bool(
        any(state.keyboard.values())
        or any(state.mouse_buttons.values())
        or abs(state.mouse_dx) > 1e-12
        or abs(state.mouse_dy) > 1e-12
    )
    gamepad_active = bool(
        state.gamepad_connected
        and (
            any(abs(value) > 1e-12 for value in state.gamepad_axes.values())
            or any(state.gamepad_buttons.values())
        )
    )
    if keyboard_active and gamepad_active:
        return "mixed"
    if keyboard_active:
        # A merely connected, neutral pad is warmup intent only when the
        # keyboard/mouse side is also neutral.
        return "keyboard"
    if gamepad_active or state.gamepad_connected:
        return "gamepad"
    return None


def external_frame_input_source(
    state: ExternalInputState,
    *,
    configured_source: str,
) -> str:
    """Choose a virtual device without bypassing the configured source gate."""

    if configured_source not in {"auto", "keyboard", "gamepad"}:
        raise ValueError("configured external input source is invalid")
    if configured_source != "auto":
        return configured_source

    device = external_active_input_device(state)
    if device in {"keyboard", "gamepad"}:
        return device
    # Mixed input is rejected by the provider source gate before publish.  Do
    # not pick a winner here and accidentally hide one half of the request.
    return configured_source


@dataclass(frozen=True)
class ExternalProviderGateFrame:
    token: ExternalInputToken
    requested_neutral: bool
    requested_device: str | None
    locomotion_admitted: bool


class ExternalLocomotionProviderGate:
    """Qualify exact external revisions before exposing locomotion intent."""

    def __init__(self, broker: ExternalControlBroker) -> None:
        if not isinstance(broker, ExternalControlBroker):
            raise TypeError("external provider gate requires its broker")
        self.broker = broker

    def prepare(
        self,
        state: ExternalInputState,
        token: ExternalInputToken | None,
    ) -> tuple[ExternalInputState, ExternalProviderGateFrame | None]:
        if token is None:
            return state.without_locomotion(), None
        telemetry = self.broker.provider_gate
        exact_ready = bool(
            telemetry.ready and telemetry.input_token == token
        )
        requested_neutral = state.locomotion_neutral
        requested_device = external_active_input_device(state)
        source_admitted = requested_device != "mixed"
        effective = (
            state
            if source_admitted and (requested_neutral or exact_ready)
            else state.without_locomotion()
        )
        return effective, ExternalProviderGateFrame(
            token=token,
            requested_neutral=requested_neutral,
            requested_device=requested_device,
            locomotion_admitted=bool(
                source_admitted and (requested_neutral or exact_ready)
            ),
        )

    def observe_published(
        self,
        frame: ExternalProviderGateFrame,
        *,
        sequence: int,
        published: bool,
        interlock_reason: str | None,
    ) -> bool:
        if not isinstance(frame, ExternalProviderGateFrame):
            raise TypeError("provider gate frame is invalid")
        if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence <= 0:
            raise ValueError("provider gate sequence is invalid")
        if type(published) is not bool:
            raise ValueError("provider gate publish result must be boolean")
        if interlock_reason is not None and (
            not isinstance(interlock_reason, str) or not interlock_reason
        ):
            raise ValueError("provider gate interlock reason is invalid")

        current_token = self.broker.input_token
        current = self.broker.provider_gate
        same_authority_current_or_predecessor = bool(
            current_token is not None
            and frame.token.lease_id == current_token.lease_id
            and frame.token.authority_epoch == current_token.authority_epoch
            and frame.token.input_revision <= current_token.input_revision
        )
        if (
            interlock_reason == "physical_focus_lost"
            and same_authority_current_or_predecessor
        ):
            self.broker.latch_fatal_authority(interlock_reason)
            current_token = self.broker.input_token
            current = self.broker.provider_gate
        # A command/data-modify request may bump the revision after this frame
        # was sampled but before its socket write completed.  Never let that
        # stale success qualify the replacement revision.  A stale failure or
        # final-publish interlock from the same authority is different: the
        # successor may have inherited the old proof, so it must be invalidated
        # fail-closed even though the callback names the predecessor revision.
        if frame.token != current_token or current.input_token != frame.token:
            strict_same_authority_successor = bool(
                current_token is not None
                and current.input_token == current_token
                and frame.token.lease_id == current_token.lease_id
                and frame.token.authority_epoch == current_token.authority_epoch
                and frame.token.input_revision < current_token.input_revision
            )
            stale_frame_invalidates_successor = bool(
                strict_same_authority_successor
                and (not published or interlock_reason is not None)
            )
            if stale_frame_invalidates_successor:
                # Never weaken an already-sticky gate because an older frame
                # completed later.  It is already in the required fail-closed
                # state and its newer diagnostic remains authoritative.
                if current.phase == "interlocked":
                    return False
                sticky_interlock = bool(
                    interlock_reason is not None
                    and interlock_reason != "gamepad_connected_edge"
                )
                reason = (
                    interlock_reason
                    if interlock_reason is not None
                    else "publisher_send_failed"
                )
                last_sequence = current.last_sequence
                if published and (
                    last_sequence is None or sequence > last_sequence
                ):
                    last_sequence = sequence
                assert current_token is not None
                return self.broker.update_provider_gate(
                    ProviderGateTelemetry(
                        authority_epoch=current_token.authority_epoch,
                        lease_id=current_token.lease_id,
                        input_revision=current_token.input_revision,
                        phase=(
                            "interlocked"
                            if sticky_interlock
                            else "awaiting_neutral"
                        ),
                        ready=False,
                        neutral_sent_count=0,
                        last_interlock_reason=reason,
                        last_sequence=last_sequence,
                    )
                )
            return False
        # Consecutive means distinct, monotonically published provider frames.
        # A duplicate callback or regressed sequence must not increment the
        # neutral proof counter a second time.
        if current.last_sequence is not None and sequence <= current.last_sequence:
            return False

        last_sequence = sequence if published else current.last_sequence
        if current.phase == "interlocked":
            return self.broker.update_provider_gate(
                ProviderGateTelemetry(
                    authority_epoch=frame.token.authority_epoch,
                    lease_id=frame.token.lease_id,
                    input_revision=frame.token.input_revision,
                    phase="interlocked",
                    ready=False,
                    neutral_sent_count=0,
                    last_interlock_reason=(
                        current.last_interlock_reason or "input_interlock"
                    ),
                    last_sequence=last_sequence,
                )
            )

        if not published:
            # A failed socket write proves nothing about what the core saw.
            # Drop the consecutive-frame count and require a fresh neutral
            # sequence, while allowing a later neutral frame to rearm.
            return self.broker.update_provider_gate(
                ProviderGateTelemetry(
                    authority_epoch=frame.token.authority_epoch,
                    lease_id=frame.token.lease_id,
                    input_revision=frame.token.input_revision,
                    phase="awaiting_neutral",
                    ready=False,
                    neutral_sent_count=0,
                    last_interlock_reason="publisher_send_failed",
                    last_sequence=current.last_sequence,
                )
            )

        if interlock_reason is not None:
            expected_connect_edge = interlock_reason == "gamepad_connected_edge"
            return self.broker.update_provider_gate(
                ProviderGateTelemetry(
                    authority_epoch=frame.token.authority_epoch,
                    lease_id=frame.token.lease_id,
                    input_revision=frame.token.input_revision,
                    phase=(
                        "awaiting_neutral"
                        if expected_connect_edge
                        else "interlocked"
                    ),
                    ready=False,
                    neutral_sent_count=0,
                    last_interlock_reason=interlock_reason,
                    last_sequence=last_sequence,
                )
            )

        if current.ready:
            assert current.qualified_from_revision is not None
            return self.broker.update_provider_gate(
                ProviderGateTelemetry(
                    authority_epoch=frame.token.authority_epoch,
                    lease_id=frame.token.lease_id,
                    input_revision=frame.token.input_revision,
                    phase="ready",
                    ready=True,
                    neutral_sent_count=current.neutral_sent_count,
                    qualified_from_revision=current.qualified_from_revision,
                    last_interlock_reason=None,
                    last_sequence=last_sequence,
                )
            )

        if not frame.requested_neutral:
            return self.broker.update_provider_gate(
                ProviderGateTelemetry(
                    authority_epoch=frame.token.authority_epoch,
                    lease_id=frame.token.lease_id,
                    input_revision=frame.token.input_revision,
                    phase="awaiting_neutral",
                    ready=False,
                    neutral_sent_count=0,
                    last_interlock_reason="locomotion_requested_before_ready",
                    last_sequence=last_sequence,
                )
            )

        neutral_count = current.neutral_sent_count + 1
        ready = neutral_count >= current.required_neutral_frames
        return self.broker.update_provider_gate(
            ProviderGateTelemetry(
                authority_epoch=frame.token.authority_epoch,
                lease_id=frame.token.lease_id,
                input_revision=frame.token.input_revision,
                phase="ready" if ready else "awaiting_neutral",
                ready=ready,
                neutral_sent_count=neutral_count,
                qualified_from_revision=(
                    frame.token.input_revision if ready else None
                ),
                last_interlock_reason=(
                    None if ready else current.last_interlock_reason
                ),
                last_sequence=last_sequence,
            )
        )


def external_provider_source_interlock_reason(
    frame: ExternalProviderGateFrame,
    *,
    configured_source: str,
) -> str | None:
    """Reject a virtual device that final source arbitration would discard."""

    if not isinstance(frame, ExternalProviderGateFrame):
        raise TypeError("external provider gate frame is invalid")
    if configured_source not in {"auto", "keyboard", "gamepad"}:
        raise ValueError("configured external input source is invalid")
    if frame.requested_device == "mixed":
        return "input_source_mixed"
    if configured_source == "auto" or frame.requested_device is None:
        return None
    if configured_source == "keyboard" and frame.requested_device in {
        "gamepad",
        "mixed",
    }:
        return "input_source_rejects_gamepad"
    if configured_source == "gamepad" and frame.requested_device in {
        "keyboard",
        "mixed",
    }:
        return "input_source_rejects_keyboard"
    return None


def apply_external_source_gate(
    state: ExternalInputState,
    frame: ExternalProviderGateFrame,
    *,
    configured_source: str,
) -> tuple[ExternalInputState, str | None]:
    """Remove all virtual side effects before a rejected source is sampled."""

    if not isinstance(state, ExternalInputState):
        raise TypeError("external input state is invalid")
    reason = external_provider_source_interlock_reason(
        frame,
        configured_source=configured_source,
    )
    return (
        ExternalInputState.neutral() if reason is not None else state,
        reason,
    )


def external_provider_interlock_reason(
    *,
    physical_focused: bool,
    camera_dragging: bool,
    camera_available: bool,
    input_available: bool,
    gamepad_connected_edge: bool,
    calibration_interlock_active: bool,
) -> str | None:
    """Name the final publish precondition that invalidates provider proof."""

    flags = (
        physical_focused,
        camera_dragging,
        camera_available,
        input_available,
        gamepad_connected_edge,
        calibration_interlock_active,
    )
    if any(type(value) is not bool for value in flags):
        raise TypeError("provider interlock flags must be boolean")
    if not physical_focused:
        return "physical_focus_lost"
    if calibration_interlock_active:
        return "calibration_interlock"
    if camera_dragging:
        return "camera_dragging"
    if not camera_available:
        return "camera_unavailable"
    if gamepad_connected_edge:
        return "gamepad_connected_edge"
    if not input_available:
        return "input_unavailable"
    return None


def external_provider_publish_interlock_reason(
    frame: ExternalProviderGateFrame,
    *,
    configured_source: str,
    physical_focused: bool,
    camera_dragging: bool,
    camera_available: bool,
    input_available: bool,
    gamepad_connected_edge: bool,
    calibration_interlock_active: bool,
) -> str | None:
    """Combine source and final-snapshot gates without hiding fatal focus."""

    if not isinstance(frame, ExternalProviderGateFrame):
        raise TypeError("external provider gate frame is invalid")
    if configured_source not in {"auto", "keyboard", "gamepad"}:
        raise ValueError("configured external input source is invalid")
    final_reason = external_provider_interlock_reason(
        physical_focused=physical_focused,
        camera_dragging=camera_dragging,
        camera_available=camera_available,
        input_available=input_available,
        gamepad_connected_edge=gamepad_connected_edge,
        calibration_interlock_active=calibration_interlock_active,
    )
    if final_reason == "physical_focus_lost":
        return final_reason
    return (
        external_provider_source_interlock_reason(
            frame,
            configured_source=configured_source,
        )
        or final_reason
    )


class KeyboardDoubleTapDetector:
    """Derive one held same-key double-tap boost from sampled WASD levels."""

    _DIRECTIONS = ("w", "a", "s", "d")

    def __init__(self, window_s: float = 0.30) -> None:
        window = float(window_s)
        if not math.isfinite(window) or not 0.15 <= window <= 0.50:
            raise ValueError("keyboard double-tap window must be in [0.15, 0.50]s")
        self.window_s = window
        self._previous = {name: False for name in self._DIRECTIONS}
        self._first_press_at: dict[str, float] = {}
        self._released: set[str] = set()
        self._boost_key: str | None = None
        self._source_id: str | None = None
        self._tier: str | None = None
        self.activations = 0
        self.resets = 0
        self.last_reset_reason: str | None = None

    @staticmethod
    def _speed_tier(keyboard: KeyboardMouseSample) -> str:
        # Keep this precedence identical to GameControlCore: either precision
        # modifier wins over Shift, while unmodified WASD selects walk.
        if keyboard.ctrl or keyboard.alt:
            return "slow"
        if keyboard.shift:
            return "run"
        return "walk"

    def _reset(
        self,
        current: dict[str, bool],
        *,
        reason: str,
        source_id: str | None,
        tier: str,
    ) -> None:
        self._previous = dict(current)
        self._first_press_at.clear()
        self._released.clear()
        self._boost_key = None
        self._source_id = source_id
        self._tier = tier
        self.resets += 1
        self.last_reset_reason = reason

    def update(
        self,
        keyboard: KeyboardMouseSample,
        *,
        now_s: float,
        enabled: bool,
        source_id: str = "physical",
    ) -> bool:
        now = float(now_s)
        if not math.isfinite(now) or now < 0.0:
            raise ValueError("double-tap monotonic time must be finite and nonnegative")
        if not isinstance(source_id, str) or not source_id:
            raise ValueError("double-tap source_id must be non-empty")
        current = {
            name: bool(getattr(keyboard, name)) for name in self._DIRECTIONS
        }
        tier = self._speed_tier(keyboard)
        if self._source_id is not None and source_id != self._source_id:
            self._reset(
                current,
                reason="source_changed",
                source_id=source_id,
                tier=tier,
            )
            return False
        self._source_id = source_id
        if not enabled:
            self._reset(
                current,
                reason="input_interlock",
                source_id=source_id,
                tier=tier,
            )
            return False
        if self._tier is None:
            self._tier = tier
        elif tier != self._tier:
            self._reset(
                current,
                reason="tier_changed",
                source_id=source_id,
                tier=tier,
            )
            return False
        if (current["w"] and current["s"]) or (
            current["a"] and current["d"]
        ):
            self._reset(
                current,
                reason="opposing_directions",
                source_id=source_id,
                tier=tier,
            )
            return False

        if self._boost_key is not None:
            if current[self._boost_key]:
                self._previous = dict(current)
                return True
            self._boost_key = None

        for name in self._DIRECTIONS:
            first_at = self._first_press_at.get(name)
            if first_at is not None and now - first_at > self.window_s:
                self._first_press_at.pop(name, None)
                self._released.discard(name)

            rising = current[name] and not self._previous[name]
            falling = not current[name] and self._previous[name]
            if rising:
                first_at = self._first_press_at.get(name)
                if (
                    first_at is not None
                    and name in self._released
                    and now - first_at <= self.window_s
                ):
                    self._boost_key = name
                    self._first_press_at.clear()
                    self._released.clear()
                    self.activations += 1
                    break
                self._first_press_at[name] = now
                self._released.discard(name)
            elif falling:
                first_at = self._first_press_at.get(name)
                if first_at is not None and now - first_at <= self.window_s:
                    self._released.add(name)
                else:
                    self._first_press_at.pop(name, None)
                    self._released.discard(name)

        self._previous = dict(current)
        return self._boost_key is not None and current[self._boost_key]

    @property
    def telemetry(self) -> dict[str, object]:
        return {
            "window_s": self.window_s,
            "active": self._boost_key is not None,
            "boost_key": self._boost_key,
            "activations": self.activations,
            "resets": self.resets,
            "last_reset_reason": self.last_reset_reason,
            "source_id": self._source_id,
        }


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
    """Persist operator-interface choices and apply them live to the overlay."""

    def __init__(
        self,
        *,
        path: Path,
        desired: UiSettings,
        load_status: str,
        load_error: str | None,
    ) -> None:
        self.path = path
        self.desired = desired
        self.load_status = load_status
        self.persistence_error = load_error
        self.change_count = 0

    def apply_panel_action(self, action: str, *, active: bool) -> bool:
        if not active or action not in {"font_down", "font_up"}:
            return False
        direction = -1 if action == "font_down" else 1
        replacement = UiSettings(
            font_scale=step_font_scale(self.desired.font_scale, direction)
        )
        if replacement == self.desired:
            return False
        self.desired = replacement
        self.change_count += 1
        try:
            atomic_save_ui_settings(self.path, replacement)
            self.persistence_error = None
            self.load_status = "saved"
        except (OSError, ValueError) as exc:
            self.persistence_error = str(exc)
        return True

    def live_mapping(self) -> dict[str, object]:
        return {
            "settings_file": os.fspath(self.path),
            "font_scale": self.desired.font_scale,
            "load_status": self.load_status,
            "persistence_error": self.persistence_error,
            "change_count": self.change_count,
        }


class VideoSettingsController:
    """Persist fixed video presets; the running UE keeps its applied snapshot."""

    def __init__(
        self,
        *,
        store: VideoSettingsStore,
        applied: VideoSettings,
    ) -> None:
        self.store = store
        self.applied = applied
        self.persistence_error = store.load_error
        self.change_count = 0

    @staticmethod
    def _values(settings: VideoSettings) -> dict[str, object]:
        mapping = settings.to_mapping()
        return {
            key: value
            for key, value in mapping.items()
            if key not in {"version", "revision"}
        }

    def apply_intent(
        self,
        field: str,
        value: object,
        *,
        expected_revision: int,
        active: bool,
    ) -> bool:
        if not active:
            return False
        try:
            modification = self.store.modify(
                field,
                value,
                expected_revision=expected_revision,
            )
        except VideoSettingsError as exc:
            if exc.code == "E_VIDEO_REVISION_CONFLICT":
                try:
                    self.store.reload()
                except (VideoSettingsPersistenceError, OSError) as reload_exc:
                    self.persistence_error = str(reload_exc)
                    return False
                # A fast duplicate click or another authenticated writer may
                # legitimately advance the revision before this intent is
                # handled. Reconcile to the durable snapshot and let the
                # overlay retry from its next revision instead of deadlocking
                # the restart gate on a stale intent.
                self.persistence_error = None
                return False
            self.persistence_error = str(exc)
            return False
        except (VideoSettingsPersistenceError, OSError) as exc:
            self.persistence_error = str(exc)
            return False
        self.persistence_error = None
        if modification.changed:
            self.change_count += 1
        return modification.changed

    def pending_restart(self) -> bool:
        return self._values(self.store.settings) != self._values(self.applied)

    def live_mapping(self) -> dict[str, object]:
        desired = self.store.settings
        return {
            "available": True,
            "settings_file": os.fspath(self.store.path),
            "revision": desired.revision,
            "current": self._values(self.applied),
            "next_launch": self._values(desired),
            "pending_restart": self.pending_restart(),
            "load_status": self.store.load_status,
            "persistence_error": self.persistence_error,
            "change_count": self.change_count,
            "apply_mode": "whole_runtime_restart",
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


class CarlaCelestialLightingBridge:
    """Apply a complete, versioned visual profile through CARLA weather RPC."""

    def __init__(
        self,
        host: str,
        port: int,
        *,
        timeout_seconds: float = 0.2,
        retry_seconds: float = 1.0,
        apply_interval_seconds: float = 0.5,
        readback_tolerance_deg: float = 0.5,
        weather_readback_tolerance: float = 1e-4,
        carla_module: Any | None = None,
    ) -> None:
        for name, value in (
            ("timeout_seconds", timeout_seconds),
            ("retry_seconds", retry_seconds),
            ("apply_interval_seconds", apply_interval_seconds),
            ("readback_tolerance_deg", readback_tolerance_deg),
            ("weather_readback_tolerance", weather_readback_tolerance),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")
        if not 1 <= port <= 65535:
            raise ValueError("CARLA lighting port must be in [1, 65535]")
        self._host = host
        self._port = port
        self._timeout = timeout_seconds
        self._retry = retry_seconds
        self._interval = apply_interval_seconds
        self._tolerance = readback_tolerance_deg
        self._weather_tolerance = weather_readback_tolerance
        self._carla_module = carla_module
        self._client: Any | None = None
        self._world: Any | None = None
        self._state_lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._latest_sample: CarlaWeatherSample | None = None
        self._applied_sample: CarlaWeatherSample | None = None
        self._generation = 0
        self._next_attempt = 0.0
        self._status = "pending"
        self._error: str | None = None

    def _connect(self) -> None:
        carla = self._carla_module or importlib.import_module("carla")
        client = carla.Client(self._host, self._port)
        client.set_timeout(self._timeout)
        self._world = client.get_world()
        self._client = client

    def _disconnect(self) -> None:
        self._client = None
        self._world = None

    @staticmethod
    def _angle_delta_deg(left: float, right: float) -> float:
        return abs((left - right + 180.0) % 360.0 - 180.0)

    def _parameter_matches(self, name: str, expected: float, actual: float) -> bool:
        if name in {"sun_altitude_angle", "sun_azimuth_angle"}:
            if name == "sun_azimuth_angle":
                return self._angle_delta_deg(actual, expected) <= self._tolerance
            return abs(actual - expected) <= self._tolerance
        return math.isclose(
            actual,
            expected,
            rel_tol=self._weather_tolerance,
            abs_tol=self._weather_tolerance,
        )

    def _samples_match(
        self,
        expected: CarlaWeatherSample,
        applied: CarlaWeatherSample,
    ) -> bool:
        if expected.profile_sha256 != applied.profile_sha256:
            return False
        return all(
            expected_name == applied_name
            and self._parameter_matches(expected_name, expected_value, applied_value)
            for (expected_name, expected_value), (applied_name, applied_value) in zip(
                expected.parameters,
                applied.parameters,
                strict=True,
            )
        )

    def _apply_weather(self, sample: CarlaWeatherSample) -> None:
        if self._world is None:
            self._connect()
        assert self._world is not None
        weather = self._world.get_weather()
        for name, value in sample.parameters:
            if not hasattr(weather, name):
                raise RuntimeError(f"CARLA weather does not expose {name}")
            setattr(weather, name, value)
        self._world.set_weather(weather)
        readback = self._world.get_weather()
        for name, expected in sample.parameters:
            if not hasattr(readback, name):
                raise RuntimeError(f"CARLA weather readback does not expose {name}")
            actual = float(getattr(readback, name))
            if not math.isfinite(actual) or not self._parameter_matches(
                name, expected, actual
            ):
                raise RuntimeError(f"CARLA weather readback did not match {name}")

    def _worker(self) -> None:
        while not self._stop.is_set():
            self._wake.wait(timeout=0.25)
            if self._stop.is_set():
                return
            with self._state_lock:
                sample = self._latest_sample
                generation = self._generation
                next_attempt = self._next_attempt
            if sample is None:
                self._wake.clear()
                continue
            delay = max(0.0, next_attempt - time.monotonic())
            if delay > 0.0 and self._stop.wait(delay):
                return
            try:
                self._apply_weather(sample)
            except Exception:
                self._disconnect()
                with self._state_lock:
                    self._status = "unavailable"
                    self._error = "carla-weather-unavailable"
                    self._next_attempt = time.monotonic() + self._retry
                # Keep the wake flag set so the latest sample is retried.
                continue
            with self._state_lock:
                self._status = "applied"
                self._error = None
                self._applied_sample = sample
                self._next_attempt = time.monotonic() + self._interval
                if generation == self._generation:
                    self._wake.clear()

    def _ensure_worker(self) -> None:
        if self._thread is not None:
            return
        with self._state_lock:
            if self._thread is None:
                self._thread = threading.Thread(
                    target=self._worker,
                    name="matrix-celestial-lighting",
                    daemon=True,
                )
                self._thread.start()

    def apply(
        self,
        lighting: Mapping[str, object],
        sample: CarlaWeatherSample,
        *,
        now: float | None = None,
    ) -> dict[str, object]:
        if now is not None and (not math.isfinite(now) or now < 0.0):
            raise ValueError("CARLA lighting time must be finite and non-negative")
        if not isinstance(sample, CarlaWeatherSample):
            raise ValueError("CARLA lighting requires a typed weather sample")
        parameters = sample.parameters_mapping()
        altitude = lighting.get("sun_altitude_deg")
        azimuth = lighting.get("sun_azimuth_deg")
        if (
            isinstance(altitude, bool)
            or not isinstance(altitude, (int, float))
            or isinstance(azimuth, bool)
            or not isinstance(azimuth, (int, float))
            or not self._parameter_matches(
                "sun_altitude_angle",
                float(altitude),
                parameters["sun_altitude_angle"],
            )
            or not self._parameter_matches(
                "sun_azimuth_angle",
                float(azimuth),
                parameters["sun_azimuth_angle"],
            )
        ):
            raise ValueError("visual profile Sun angles do not match celestial truth")
        with self._state_lock:
            self._latest_sample = sample
            self._generation += 1
            if self._status == "applied" and self._applied_sample is not None:
                if not self._samples_match(sample, self._applied_sample):
                    self._status = "pending"
                    self._error = None
            status = self._status
            error = self._error
        self._ensure_worker()
        self._wake.set()
        result = dict(lighting)
        result["render_authority"] = (
            "carla-weather" if status == "applied" else "state-only"
        )
        result["render_status"] = status
        result["render_error"] = error
        return result

    def close(self) -> None:
        self._stop.set()
        self._wake.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=max(1.0, self._timeout + 0.5))
        writer_alive = thread is not None and thread.is_alive()
        self._disconnect()
        if writer_alive:
            raise RuntimeError("CARLA celestial lighting worker did not stop")


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
        "alt_left": 0xFFE9,
        "alt_right": 0xFFEA,
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
            "focus_badwindow_recoveries": getattr(
                self, "_focus_badwindow_recoveries", 0
            ),
            "last_focus_badwindow_resource": getattr(
                self, "_last_focus_badwindow_resource", None
            ),
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
            "XSync": (
                [ctypes.c_void_p, ctypes.c_int],
                ctypes.c_int,
            ),
            # The callback type itself remains process-global in Xlib.  Use a
            # void pointer at the ABI boundary so the previous handler can be
            # restored verbatim, including Xlib's null/default sentinel.
            "XSetErrorHandler": ([ctypes.c_void_p], ctypes.c_void_p),
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
    def _focus_window_error_scope(self) -> Iterator[_X11FocusErrorScope]:
        """Bound asynchronous BadWindow handling to one focus-chain read.

        Xlib error handlers are process-global while protocol errors are
        asynchronous.  The leading XSync drains older requests under the
        caller's handler; the trailing XSync delivers only errors generated by
        this scope before the exact previous handler is restored.
        """

        with _X11_ERROR_HANDLER_LOCK:
            if self._active_focus_error_scope is not None:
                raise RuntimeError("nested X11 focus error scope")
            self._x11.XSync(self._display, 0)
            scope = _X11FocusErrorScope(windows=set())
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
                    "unexpected X11 error during focus query: "
                    f"code={error_code} request={request_code} "
                    f"minor={minor_code} resource={resource}"
                )

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
            alt=pressed.get("alt_left", False)
            or pressed.get("alt_right", False),
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
        try:
            if raw_motion is not None:
                raw_motion.close()
        finally:
            self._raw_motion = None
            display = getattr(self, "_display", None)
            self._display = None
            if display:
                self._x11.XCloseDisplay(display)


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
        self._buttons: dict[int, bool] = {}
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
            self._buttons.clear()
        except OSError:
            self._fd = None
            self._path = None
            self._next_open = now + 1.0

    def _disconnect(self, now: float) -> None:
        descriptor = self._fd
        self._fd = None
        self._path = None
        self._axes.clear()
        self._buttons.clear()
        self._next_open = now + 1.0
        if descriptor is not None:
            try:
                self._closer(descriptor)
            except OSError:
                pass

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
                self._buttons[number] = bool(value)
        return GamepadSample(
            forward=-self._axes.get(self._left_y, 0.0),
            right=self._axes.get(self._left_x, 0.0),
            look_yaw=self._axes.get(self._right_x, 0.0),
            look_pitch=-self._axes.get(self._right_y, 0.0),
            buttons_pressed=any(self._buttons.values()),
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
        connection = self._socket
        self._socket = None
        if connection is not None:
            connection.close()


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
    keyboard_boost: bool = False,
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
        keyboard_boost=bool(keyboard_boost),
        keys=keys,
        move_stick=move_stick,
    )


def fail_closed_external_snapshot(snapshot: InputSnapshot) -> InputSnapshot:
    """Preserve frame identity while removing every externally authored input."""

    if not isinstance(snapshot, InputSnapshot):
        raise TypeError("external publish snapshot is invalid")
    return InputSnapshot(
        sequence=snapshot.sequence,
        timestamp_monotonic_s=snapshot.timestamp_monotonic_s,
        focused=False,
        camera_yaw_rad=snapshot.camera_yaw_rad,
        keyboard_boost=False,
        keys=KeySnapshot(
            w=False,
            a=False,
            s=False,
            d=False,
            q=False,
            e=False,
            v=False,
            ctrl=False,
            alt=False,
            shift=False,
        ),
        move_stick=MoveStickSnapshot(right=0.0, forward=0.0),
        protocol=snapshot.protocol,
    )


def apply_external_publish_interlock(
    snapshot: InputSnapshot,
    frame: ExternalProviderGateFrame | None,
    interlock_reason: str | None,
) -> InputSnapshot:
    """Guarantee that an interlocked external frame writes no side effects."""

    if not isinstance(snapshot, InputSnapshot):
        raise TypeError("external publish snapshot is invalid")
    if frame is not None and not isinstance(frame, ExternalProviderGateFrame):
        raise TypeError("external provider gate frame is invalid")
    if interlock_reason is not None and (
        not isinstance(interlock_reason, str) or not interlock_reason
    ):
        raise ValueError("external input interlock reason is invalid")
    if frame is None or interlock_reason is None:
        return snapshot
    return fail_closed_external_snapshot(snapshot)


@dataclass(frozen=True)
class ExternalPublishBoundary:
    snapshot: InputSnapshot
    current_token: ExternalInputToken | None
    exact_revision: bool


def external_publish_boundary(
    broker: ExternalControlBroker,
    frame: ExternalProviderGateFrame | None,
    snapshot: InputSnapshot,
    *,
    now: float,
) -> ExternalPublishBoundary:
    """Freeze the final authority decision immediately before socket send."""

    if not isinstance(broker, ExternalControlBroker):
        raise TypeError("external publish boundary requires its broker")
    if frame is not None and not isinstance(frame, ExternalProviderGateFrame):
        raise TypeError("external provider gate frame is invalid")
    if not isinstance(snapshot, InputSnapshot):
        raise TypeError("external publish snapshot is invalid")
    current_token = broker.publish_boundary_token(now=now)
    exact_revision = bool(frame is not None and frame.token == current_token)
    return ExternalPublishBoundary(
        snapshot=(
            snapshot
            if frame is None or exact_revision
            else fail_closed_external_snapshot(snapshot)
        ),
        current_token=current_token,
        exact_revision=exact_revision,
    )


class _CleanupCoordinator:
    """Run every provider cleanup step even when earlier steps fail."""

    def __init__(self) -> None:
        self.failures: list[dict[str, str]] = []

    def run(
        self,
        label: str,
        operation: Callable[[], Any],
        *,
        default: Any = None,
    ) -> Any:
        try:
            return operation()
        except BaseException as exc:
            failure = {
                "step": label,
                "type": type(exc).__name__,
                "message": str(exc) or type(exc).__name__,
            }
            self.failures.append(failure)
            try:
                print(
                    "matrix-game-control-input cleanup ERROR "
                    f"{label}: {failure['type']}: {failure['message']}",
                    file=sys.stderr,
                    flush=True,
                )
            except BaseException:
                # stderr itself can be closed or broken during teardown.  A
                # diagnostic sink must never become a cleanup dependency.
                pass
            return default


def _close_provider_resources(
    cleanup: _CleanupCoordinator,
    *,
    gamepad: Any,
    overlay: Any | None,
    x11: Any,
    publisher: Any | None,
    external_control: Any | None,
    previous_handlers: dict[int, Any],
) -> None:
    """Attempt every owned-resource close and every handler restoration."""

    cleanup.run("gamepad_close", gamepad.close)
    if overlay is not None:
        cleanup.run("overlay_close", overlay.close)
    cleanup.run("x11_close", x11.close)
    if publisher is not None:
        cleanup.run("publisher_close", publisher.close)
    if external_control is not None:
        cleanup.run("external_control_close", external_control.close)
    for signum, handler in previous_handlers.items():
        cleanup.run(
            f"signal_restore_{signal.Signals(signum).name}",
            lambda signum=signum, handler=handler: signal.signal(
                signum, handler
            ),
        )


def _cleanup_outcome(
    cleanup: _CleanupCoordinator,
    *,
    return_code: int,
    exit_reason: str,
) -> tuple[int, str]:
    if not cleanup.failures:
        return return_code, exit_reason
    if not exit_reason.startswith("error:"):
        exit_reason = f"cleanup_error:{cleanup.failures[0]['step']}"
    return 1, exit_reason


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


def decode_applied_video_settings(value: str) -> VideoSettings:
    """Decode the launcher's structured runtime mapping without free-form text."""

    def strict_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
        result: dict[str, object] = {}
        for key, item in pairs:
            if key in result:
                raise ValueError(f"duplicate applied video setting {key!r}")
            result[key] = item
        return result

    try:
        raw = json.loads(
            value,
            object_pairs_hook=strict_object,
            parse_constant=lambda token: (_ for _ in ()).throw(
                ValueError(f"invalid applied video constant {token}")
            ),
        )
        if not isinstance(raw, dict):
            raise ValueError("applied video settings must be an object")
        expected = {
            "revision",
            "resolution",
            "resolution_width",
            "resolution_height",
            "window_mode",
            "fps_limit",
            "quality",
            "camera_smoothing",
        }
        if set(raw) != expected:
            raise ValueError("applied video settings have an invalid schema")
        settings = VideoSettings(
            revision=raw.get("revision"),
            resolution=raw.get("resolution"),
            window_mode=raw.get("window_mode"),
            fps_limit=raw.get("fps_limit"),
            quality=raw.get("quality"),
            camera_smoothing=raw.get("camera_smoothing"),
        )
        if settings.runtime_mapping() != raw:
            raise ValueError("applied video settings runtime fields disagree")
        return settings
    except (
        json.JSONDecodeError,
        TypeError,
        ValueError,
        VideoSettingsError,
    ) as exc:
        raise ValueError(f"invalid applied video settings: {exc}") from exc


@dataclass(frozen=True)
class OverlayIntent:
    """One authenticated action emitted by the supervised overlay child."""

    kind: str
    action: str | None = None
    command: str | None = None
    active: bool | None = None
    slot: str | None = None
    policy_id: str | None = None
    item_id: str | None = None
    destination_id: str | None = None
    video_field: str | None = None
    video_value: object = None
    expected_revision: int | None = None


@dataclass(frozen=True)
class PendingExternalInputPublish:
    """One data-modify mutation waiting for its exact provider-frame write."""

    token: ExternalInputToken
    path: str
    warning: str | None
    data: dict[str, object] | None


class GameCommandClient:
    """Send one typed MC command at a time over an inherited socketpair.

    Raw command text terminates here: :func:`parse_mc_command` produces the
    typed AST carried by ``matrix-game-command/v1``.  A successfully sent
    request remains authoritative until its exact response arrives.  The
    client never reconnects, resends, or converts a timeout into a retry.
    """

    def __init__(
        self,
        file_descriptor: int | None,
        *,
        initial_strategy_loadout: object = None,
        initial_creative_inventory: object = None,
        initial_motion_settings: object = None,
        celestial_catalog: CelestialCatalog | None = None,
        celestial_clock: PersistentSimulationClock | None = None,
        celestial_visual_catalog: CelestialVisualCatalog | None = None,
        celestial_visual_profile: str = "auto",
        celestial_lighting_bridge: CarlaCelestialLightingBridge | None = None,
    ) -> None:
        self._connection: socket.socket | None = None
        self._session = os.urandom(16).hex()
        self._sequence = 0
        self._result_revision = 0
        self._pending: GameCommandRequest | None = None
        self._pending_external_input: PendingExternalInputPublish | None = None
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
        self._strategy_loadout = self._validate_strategy_loadout(
            initial_strategy_loadout
        )
        self._creative_inventory = self._validate_creative_inventory(
            initial_creative_inventory
        )
        self._motion_settings = (
            None
            if initial_motion_settings is None
            else validate_motion_settings_telemetry(initial_motion_settings)
        )
        self._celestial_catalog = celestial_catalog
        if celestial_clock is not None and celestial_catalog is None:
            raise ValueError("celestial clock requires a celestial catalog")
        if celestial_visual_catalog is not None and celestial_catalog is None:
            raise ValueError("celestial visuals require a celestial catalog")
        if (
            celestial_lighting_bridge is not None
            and celestial_visual_catalog is None
        ):
            raise ValueError("celestial lighting bridge requires visual profiles")
        self._celestial_clock = (
            celestial_clock
            if celestial_clock is not None
            else (
                celestial_catalog.create_clock()
                if celestial_catalog is not None
                else None
            )
        )
        self._celestial_visual_catalog = celestial_visual_catalog
        self._celestial_visual_profile = celestial_visual_profile
        self._celestial_lighting_bridge = celestial_lighting_bridge
        self._teleport_probes = {}
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

    @staticmethod
    def _validate_strategy_loadout(value: object) -> dict[str, object]:
        if value is None:
            return {
                "version": 1,
                "available": False,
                "status": "unavailable",
                "active_slot": "locomotion",
                "pending": None,
                "slots": [],
                "resident_models": [],
            }
        if not isinstance(value, dict) or value.get("version") != 1:
            raise ValueError("strategy loadout has an invalid version")
        if type(value.get("available")) is not bool:
            raise ValueError("strategy loadout availability is invalid")
        if value.get("status") not in {
            "unavailable",
            "loading",
            "ready",
            "switching",
        }:
            raise ValueError("strategy loadout status is invalid")
        if value.get("active_slot") not in {"locomotion", "recovery"}:
            raise ValueError("strategy loadout active slot is invalid")
        slots = value.get("slots")
        resident_models = value.get("resident_models")
        if not isinstance(slots, list) or not isinstance(resident_models, list):
            raise ValueError("strategy loadout collections are invalid")
        seen_slots: set[str] = set()
        for slot in slots:
            if not isinstance(slot, dict):
                raise ValueError("strategy slot must be an object")
            slot_id = slot.get("slot")
            selected = slot.get("selected_policy_id")
            candidates = slot.get("candidates")
            if (
                slot_id not in {"locomotion", "recovery"}
                or slot_id in seen_slots
                or not isinstance(selected, str)
                or type(slot.get("locked")) is not bool
                or not isinstance(candidates, list)
            ):
                raise ValueError("strategy slot has an invalid schema")
            seen_slots.add(slot_id)
            candidate_ids: set[str] = set()
            for candidate in candidates:
                if not isinstance(candidate, dict):
                    raise ValueError("strategy candidate must be an object")
                try:
                    validated = PolicySlotAssignment(
                        slot=slot_id,
                        policy_id=candidate.get("policy_id"),
                    )
                except CommandParseError as exc:
                    raise ValueError(str(exc)) from exc
                if (
                    validated.policy_id in candidate_ids
                    or type(candidate.get("resident")) is not bool
                    or type(candidate.get("available")) is not bool
                ):
                    raise ValueError("strategy candidate has an invalid schema")
                candidate_ids.add(validated.policy_id)
            if selected not in candidate_ids:
                raise ValueError("strategy slot selection is not a candidate")
        if slots and seen_slots != {"locomotion", "recovery"}:
            raise ValueError("strategy loadout must define both slots")
        try:
            cloned = json.loads(json.dumps(value, allow_nan=False))
        except (TypeError, ValueError) as exc:
            raise ValueError("strategy loadout is not strict JSON") from exc
        assert isinstance(cloned, dict)
        return cloned

    def strategy_loadout_mapping(self) -> dict[str, object]:
        return json.loads(json.dumps(self._strategy_loadout, allow_nan=False))

    @staticmethod
    def _validate_creative_inventory(value: object) -> dict[str, object]:
        if value is None:
            return {
                "version": 1,
                "available": False,
                "spawn_count": 0,
                "items": [],
            }
        if not isinstance(value, dict) or value.get("version") != 1:
            raise ValueError("creative inventory has an invalid version")
        if type(value.get("available")) is not bool:
            raise ValueError("creative inventory availability is invalid")
        spawn_count = value.get("spawn_count")
        items = value.get("items")
        if (
            type(spawn_count) is not int
            or spawn_count < 0
            or not isinstance(items, list)
            or len(items) > 16
        ):
            raise ValueError("creative inventory counters are invalid")
        seen: set[str] = set()
        for item in items:
            if not isinstance(item, dict) or set(item) != {
                "item_id",
                "label",
                "pool_size",
                "remaining",
            }:
                raise ValueError("creative inventory item has an invalid schema")
            item_id = item.get("item_id")
            label = item.get("label")
            pool_size = item.get("pool_size")
            remaining = item.get("remaining")
            try:
                CreativeSpawnItem(item_id=item_id)
            except CommandParseError as exc:
                raise ValueError(str(exc)) from exc
            if (
                item_id in seen
                or not isinstance(label, str)
                or not label
                or len(label) > 40
                or type(pool_size) is not int
                or type(remaining) is not int
                or not 0 <= remaining <= pool_size <= 32
            ):
                raise ValueError("creative inventory item values are invalid")
            seen.add(item_id)
        try:
            cloned = json.loads(json.dumps(value, allow_nan=False))
        except (TypeError, ValueError) as exc:
            raise ValueError("creative inventory is not strict JSON") from exc
        assert isinstance(cloned, dict)
        return cloned

    def creative_inventory_mapping(self) -> dict[str, object]:
        return json.loads(json.dumps(self._creative_inventory, allow_nan=False))

    def motion_settings_mapping(self) -> dict[str, object] | None:
        return (
            json.loads(json.dumps(self._motion_settings, allow_nan=False))
            if self._motion_settings is not None
            else None
        )

    def celestial_navigation_mapping(self) -> dict[str, object]:
        catalog = self._celestial_catalog
        clock = self._celestial_clock
        if catalog is None or clock is None:
            return {
                "version": 2,
                "available": False,
                "status": "unavailable",
                "universe_id": "unavailable",
                "display_name": "Universe unavailable",
                "reference_epoch_utc": None,
                "time_scale": None,
                "frame": None,
                "ephemeris": None,
                "simulation_time": None,
                "origin_rebasing": True,
                "simulation_local_bound_m": 100000.0,
                "current_body_id": None,
                "bodies": [],
                "lighting": None,
                "destinations": [],
            }
        snapshot = clock.snapshot()
        mapping = catalog.navigation_mapping(
            self._teleport_probes,
            command_available=self.available,
            in_flight=self.in_flight,
            restart_required=self.restart_required,
            outcome_unknown=self.outcome_unknown,
            simulation_time=snapshot,
        )
        lighting = mapping.get("lighting")
        visual_catalog = self._celestial_visual_catalog
        if visual_catalog is not None and isinstance(lighting, dict):
            sample = visual_catalog.sample(
                lighting,
                profile_id=self._celestial_visual_profile,
            )
            profiled_lighting = dict(lighting)
            profiled_lighting["visual_profile"] = sample.profile_mapping()
            if self._celestial_lighting_bridge is not None:
                profiled_lighting = self._celestial_lighting_bridge.apply(
                    profiled_lighting,
                    sample,
                )
            mapping["lighting"] = profiled_lighting
        clock.checkpoint()
        return mapping

    def checkpoint_celestial_clock(self) -> bool:
        clock = self._celestial_clock
        return clock.checkpoint() if clock is not None else False

    @property
    def available(self) -> bool:
        return self._connection is not None

    @property
    def in_flight(self) -> bool:
        return self._pending is not None or self._pending_external_input is not None

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
        if restart_requested:
            self._local_error(
                "E_RESTART_PENDING", "A whole-runtime restart is already pending"
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
        if isinstance(parsed.command, DataModifyInput):
            self._local_error(
                "E_EXTERNAL_API_REQUIRED",
                "control.input data modify requires an active external-control lease",
            )
            return False
        if not self.editing and not isinstance(parsed.command, DataModifyNumber):
            self._local_error(
                "E_COMMAND_EDIT_REQUIRED", "Activate the command input first"
            )
            return False
        return self._send_typed_command(
            parsed.command,
            warning=parsed.warning,
            pending_message="Command submitted; waiting for the runtime",
        )

    def submit_external(
        self,
        command_text: object,
        *,
        calibration_active: bool,
        neutral_frame_ready: bool,
        restart_requested: bool,
        input_modifier: Callable[
            [DataModifyInput],
            tuple[ExternalInputToken, dict[str, object] | None],
        ],
    ) -> bool:
        """Submit one capability-authenticated API command.

        Input data modifications stay provider-side and can drive normal
        gameplay.  World/policy/settings commands retain the ESC neutral-frame
        gate, while the visual text-editor gate is unnecessary for an already
        authenticated external client.
        """

        if self.in_flight or self.restart_required or self._outcome_unknown:
            return False
        if restart_requested:
            self._local_error(
                "E_RESTART_PENDING", "A whole-runtime restart is already pending"
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
        if isinstance(parsed.command, DataModifyInput):
            try:
                modified = input_modifier(parsed.command)
            except Exception as exc:
                code = getattr(exc, "code", "E_EXTERNAL_INPUT")
                message = getattr(exc, "message", str(exc) or type(exc).__name__)
                self._local_error(str(code), str(message))
                return False
            if (
                not isinstance(modified, tuple)
                or len(modified) != 2
                or not isinstance(modified[0], ExternalInputToken)
                or (modified[1] is not None and not isinstance(modified[1], dict))
            ):
                self._local_error(
                    "E_EXTERNAL_INPUT",
                    "input modifier did not return an exact input token",
                )
                return False
            token, data = modified
            self._pending_external_input = PendingExternalInputPublish(
                token=token,
                path=parsed.command.path,
                warning=parsed.warning,
                data=data,
            )
            self._result_revision += 1
            self._outcome_unknown = False
            self.status = "pending"
            self.ok = None
            self.code = None
            self.message = (
                "Input modified; waiting for its exact provider-frame publish"
            )
            self.warning = parsed.warning
            self.restart_required = False
            self.data = data
            self.last_request_id = None
            return True
        if not calibration_active:
            self._local_error("E_NOT_PAUSED", "Open the ESC panel before commands")
            return False
        if not neutral_frame_ready:
            self._local_error(
                "E_NEUTRAL_REQUIRED",
                "Wait for the ESC panel to deliver a neutral frame",
            )
            return False
        return self._send_typed_command(
            parsed.command,
            warning=parsed.warning,
            pending_message="External command submitted; waiting for the runtime",
        )

    def resolve_external_input_publish(
        self,
        *,
        sampled_token: ExternalInputToken | None,
        current_token: ExternalInputToken | None,
        authority_active: bool,
        published: bool,
        locomotion_admitted: bool,
        interlock_reason: str | None,
        data: dict[str, object] | None = None,
    ) -> bool:
        """Resolve a provider-side input command from one final frame outcome."""

        pending = self._pending_external_input
        if pending is None:
            return False
        if (
            type(authority_active) is not bool
            or type(published) is not bool
            or type(locomotion_admitted) is not bool
        ):
            raise TypeError("external input publish outcome flags must be boolean")
        if sampled_token is not None and not isinstance(
            sampled_token, ExternalInputToken
        ):
            raise TypeError("sampled external input token is invalid")
        if current_token is not None and not isinstance(
            current_token, ExternalInputToken
        ):
            raise TypeError("current external input token is invalid")
        if interlock_reason is not None and (
            not isinstance(interlock_reason, str) or not interlock_reason
        ):
            raise ValueError("external input interlock reason is invalid")
        if data is not None and not isinstance(data, dict):
            raise TypeError("external input result data must be a mapping")

        if not authority_active or current_token is None:
            return self._finish_external_input_publish(
                ok=False,
                code="E_AUTHORITY_REVOKED",
                message="External input authority was revoked before publish",
                data=data,
            )
        if current_token != pending.token:
            return self._finish_external_input_publish(
                ok=False,
                code="E_INPUT_SUPERSEDED",
                message="External input revision changed before publish",
                data=data,
            )
        # The provider may have sampled the predecessor immediately before the
        # command was admitted in this same loop iteration.  That frame cannot
        # prove or reject the pending mutation; wait for the exact revision.
        if sampled_token != pending.token:
            return False
        if interlock_reason is not None:
            return self._finish_external_input_publish(
                ok=False,
                code="E_INPUT_INTERLOCK",
                message=f"External input publish interlocked: {interlock_reason}",
                data=data,
            )
        if not published:
            return self._finish_external_input_publish(
                ok=False,
                code="E_INPUT_PUBLISH_FAILED",
                message="External input provider frame was not published",
                data=data,
            )
        if not locomotion_admitted:
            return self._finish_external_input_publish(
                ok=False,
                code="E_INPUT_INTERLOCK",
                message="External input provider gate did not admit locomotion",
                data=data,
            )
        return self._finish_external_input_publish(
            ok=True,
            code="OK_DATA_INPUT_MODIFIED",
            message=f"Set {pending.path}",
            data=data,
        )

    def _finish_external_input_publish(
        self,
        *,
        ok: bool,
        code: str,
        message: str,
        data: dict[str, object] | None,
    ) -> bool:
        pending = self._pending_external_input
        if pending is None:
            return False
        self._pending_external_input = None
        self._result_revision += 1
        self._outcome_unknown = False
        self.status = "success" if ok else "error"
        self.ok = ok
        self.code = code
        self.message = message
        self.warning = pending.warning
        self.restart_required = False
        self.data = pending.data if data is None else data
        self.last_request_id = None
        return True

    def _send_typed_command(
        self,
        command: object,
        *,
        warning: str | None,
        pending_message: str,
    ) -> bool:
        connection = self._connection
        if connection is None:
            self._local_error(
                "E_COMMAND_UNAVAILABLE", "Game commands are unavailable for this run"
            )
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
        self._pending_warning = warning
        self._result_revision += 1
        self.status = "pending"
        self.ok = None
        self.code = None
        self.message = pending_message
        self.warning = warning
        self.restart_required = False
        self.data = None
        self.last_request_id = request.request_id
        return True

    def select_policy(
        self,
        slot: object,
        policy_id: object,
        *,
        calibration_active: bool,
        neutral_frame_ready: bool,
        restart_requested: bool,
    ) -> bool:
        """Send one strategy-slot transaction without entering text editing."""

        if self.in_flight or self.restart_required or self._outcome_unknown:
            return False
        if not calibration_active:
            self._local_error("E_NOT_PAUSED", "Open the ESC panel before switching")
            return False
        if not neutral_frame_ready:
            self._local_error(
                "E_NEUTRAL_REQUIRED",
                "Wait for the ESC panel to deliver a neutral frame",
            )
            return False
        if restart_requested:
            self._local_error(
                "E_RESTART_PENDING", "A whole-runtime restart is already pending"
            )
            return False
        try:
            command = PolicySlotAssignment(slot=slot, policy_id=policy_id)
        except CommandParseError as exc:
            self._local_error(exc.code, exc.message)
            return False
        return self._send_typed_command(
            command,
            warning=None,
            pending_message="Switching resident policy; waiting for writer ACK",
        )

    def spawn_creative_item(
        self,
        item_id: object,
        *,
        calibration_active: bool,
        neutral_frame_ready: bool,
        restart_requested: bool,
    ) -> bool:
        if self.in_flight or self.restart_required or self.outcome_unknown:
            return False
        if not calibration_active or not neutral_frame_ready:
            self._local_error(
                "E_NEUTRAL_REQUIRED",
                "Open ESC and wait for a neutral frame before taking an item",
            )
            return False
        if restart_requested:
            self._local_error(
                "E_RESTART_PENDING", "A whole-runtime restart is already pending"
            )
            return False
        try:
            command = CreativeSpawnItem(item_id=item_id)
        except CommandParseError as exc:
            self._local_error(exc.code, exc.message)
            return False
        return self._send_typed_command(
            command,
            warning=None,
            pending_message="Taking item from creative inventory",
        )

    def refresh_celestial_navigation(
        self,
        *,
        calibration_active: bool,
        neutral_frame_ready: bool,
        restart_requested: bool,
    ) -> bool:
        """Query only catalog tags over the existing typed command channel."""

        if self.in_flight or self.restart_required or self._outcome_unknown:
            return False
        catalog = self._celestial_catalog
        if catalog is None:
            self._local_error(
                "E_NAVIGATION_UNAVAILABLE", "Celestial navigation is unavailable"
            )
            return False
        if not calibration_active:
            self._local_error("E_NOT_PAUSED", "Open the ESC panel before refreshing")
            return False
        if not neutral_frame_ready:
            self._local_error(
                "E_NEUTRAL_REQUIRED",
                "Wait for the ESC panel to deliver a neutral frame",
            )
            return False
        if restart_requested:
            self._local_error(
                "E_RESTART_PENDING", "A whole-runtime restart is already pending"
            )
            return False
        command = TeleportList(
            tuple(destination.teleport_tag for destination in catalog.destinations)
        )
        sent = self._send_typed_command(
            command,
            warning=None,
            pending_message="Refreshing celestial teleport points",
        )
        if sent:
            self._teleport_probes = {}
        return sent

    def select_celestial_destination(
        self,
        destination_id: object,
        *,
        calibration_active: bool,
        neutral_frame_ready: bool,
        restart_requested: bool,
    ) -> bool:
        """Resolve one catalog destination to a typed teleport selector."""

        if self.in_flight or self.restart_required or self._outcome_unknown:
            return False
        if not calibration_active:
            self._local_error("E_NOT_PAUSED", "Open the ESC panel before teleporting")
            return False
        if not neutral_frame_ready:
            self._local_error(
                "E_NEUTRAL_REQUIRED",
                "Wait for the ESC panel to deliver a neutral frame",
            )
            return False
        if restart_requested:
            self._local_error(
                "E_RESTART_PENDING", "A whole-runtime restart is already pending"
            )
            return False
        catalog = self._celestial_catalog
        if catalog is None:
            self._local_error(
                "E_NAVIGATION_UNAVAILABLE", "Celestial navigation is unavailable"
            )
            return False
        if not isinstance(destination_id, str):
            self._local_error(
                "E_DESTINATION_INVALID", "Celestial destination id is invalid"
            )
            return False
        try:
            destination = catalog.destination(destination_id)
            body = catalog.body(destination.body_id)
        except CelestialNavigationError as exc:
            self._local_error("E_DESTINATION_INVALID", str(exc))
            return False
        probe = self._teleport_probes.get(destination.teleport_tag)
        if not body.runtime_ready:
            self._local_error(
                "E_WORLD_UNAVAILABLE",
                f"{body.display_name} runtime is not deployed on this build",
            )
            return False
        if probe is None or not probe.found:
            self._local_error(
                "E_SELECTOR_NO_TARGET",
                f"Teleport point {destination.teleport_tag!r} is not discovered",
            )
            return False
        return self._send_typed_command(
            TeleportSelector(tag=destination.teleport_tag),
            warning=None,
            pending_message=f"Routing to {destination.display_name}",
        )

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
        response_data = dict(response.data) if response.data is not None else None
        validated_loadout: dict[str, object] | None = None
        validated_inventory: dict[str, object] | None = None
        validated_motion_settings: dict[str, object] | None = None
        if response_data is not None and "strategy_loadout" in response_data:
            try:
                validated_loadout = self._validate_strategy_loadout(
                    response_data["strategy_loadout"]
                )
            except ValueError as exc:
                self._protocol_failure(
                    f"Invalid strategy loadout in command response: {exc}"
                )
                return True
        if response_data is not None and "creative_inventory" in response_data:
            try:
                validated_inventory = self._validate_creative_inventory(
                    response_data["creative_inventory"]
                )
            except ValueError as exc:
                self._protocol_failure(
                    f"Invalid creative inventory in command response: {exc}"
                )
                return True
        if response_data is not None and "motion_settings" in response_data:
            try:
                validated_motion_settings = validate_motion_settings_telemetry(
                    response_data["motion_settings"]
                )
            except ValueError as exc:
                self._protocol_failure(
                    f"Invalid motion settings in command response: {exc}"
                )
                return True
        teleport_probes = None
        if (
            response.ok
            and isinstance(pending.command, TeleportList)
            and self._celestial_catalog is not None
        ):
            try:
                teleport_probes = probes_from_response(
                    response_data,
                    catalog=self._celestial_catalog,
                )
            except CelestialNavigationError as exc:
                self._protocol_failure(
                    f"Invalid celestial navigation response: {exc}"
                )
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
        self.data = response_data
        if validated_loadout is not None:
            self._strategy_loadout = validated_loadout
        if validated_inventory is not None:
            self._creative_inventory = validated_inventory
        if validated_motion_settings is not None:
            self._motion_settings = validated_motion_settings
        if teleport_probes is not None:
            self._teleport_probes = teleport_probes
        self.last_request_id = response.request_id
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
        pending_external = self._pending_external_input
        if pending_external is not None:
            self._pending_external_input = None
            self._result_revision += 1
            self._outcome_unknown = True
            self.status = "error"
            self.ok = None
            self.code = "E_COMMAND_OUTCOME_UNKNOWN"
            self.message = (
                "External input publish outcome unknown; do not retry blindly"
            )
            self.warning = pending_external.warning
            self.restart_required = False
            self.data = pending_external.data
            self.last_request_id = None
        self._pending_warning = None
        self._close_channel()

    def finalize_celestial_resources(self) -> None:
        """Close providers after the final status mapping has been serialized."""

        first_error: BaseException | None = None
        operations = (
            (
                self._celestial_clock.close
                if self._celestial_clock is not None
                else None
            ),
            (
                self._celestial_lighting_bridge.close
                if self._celestial_lighting_bridge is not None
                else None
            ),
            (
                self._celestial_catalog.ephemeris.close
                if self._celestial_catalog is not None
                else None
            ),
        )
        for operation in operations:
            if operation is None:
                continue
            try:
                operation()
            except BaseException as exc:
                if first_error is None:
                    first_error = exc
        if first_error is not None:
            raise first_error


def validate_motion_settings_telemetry(value: object) -> dict[str, object]:
    """Validate and clone one runtime-owned motion-settings status snapshot."""

    if not isinstance(value, dict) or set(value) != {
        "settings_file",
        "load_status",
        "load_error",
        "settings",
    }:
        raise ValueError("motion settings telemetry has an invalid schema")
    settings_file = value.get("settings_file")
    if not isinstance(settings_file, str) or not Path(settings_file).is_absolute():
        raise ValueError("motion settings telemetry file must be absolute")
    if value.get("load_status") not in {
        "loaded",
        "missing",
        "invalid",
        "provided",
        "saved",
    }:
        raise ValueError("motion settings telemetry load status is invalid")
    load_error = value.get("load_error")
    if load_error is not None and not isinstance(load_error, str):
        raise ValueError("motion settings telemetry load error is invalid")
    try:
        MotionSettings.from_mapping(value.get("settings"))
    except (MotionSettingsError, TypeError, ValueError) as exc:
        raise ValueError(f"motion settings telemetry values are invalid: {exc}") from exc
    return json.loads(json.dumps(value, allow_nan=False))


def live_motion_settings_telemetry(
    initial: dict[str, object] | None,
    command_client: GameCommandClient,
) -> dict[str, object] | None:
    """Prefer the latest acknowledged settings, retaining the launch snapshot."""

    candidate: object = command_client.motion_settings_mapping()
    if candidate is None:
        candidate = initial
    if candidate is None:
        return None
    return validate_motion_settings_telemetry(candidate)


class CalibrationOverlaySupervisor:
    """Own the X11 overlay and its private pointer-intent socket."""

    _MAX_INTENT_PACKET_BYTES = 2048
    _ALLOWED_ACTIONS = frozenset(
        {
            "profile_local",
            "profile_remote",
            "speed_down",
            "speed_up",
            "font_down",
            "font_up",
            "apply_return",
        }
    )

    def __init__(
        self,
        *,
        state_file: Path,
        display_name: str | None,
        expected_ue_pid: int,
        font_scale: float = 1.0,
        script: Path | None = None,
        python: str = sys.executable,
        startup_timeout_s: float = 3.0,
    ) -> None:
        self.state_file = state_file
        self.ready_file = state_file.with_name(f".{state_file.name}.overlay-status.json")
        self.display_name = display_name
        self.expected_ue_pid = expected_ue_pid
        self.font_scale = UiSettings(font_scale=font_scale).font_scale
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
            "--font-scale",
            f"{self.font_scale:.2f}",
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
            elif kind == "strategy_select":
                if set(value) != {
                    "version",
                    "session",
                    "sequence",
                    "kind",
                    "slot",
                    "policy_id",
                }:
                    raise RuntimeError("invalid strategy-selection intent schema")
                try:
                    assignment = PolicySlotAssignment(
                        slot=value.get("slot"),
                        policy_id=value.get("policy_id"),
                    )
                except CommandParseError as exc:
                    raise RuntimeError(
                        "invalid strategy-selection intent"
                    ) from exc
                intent = OverlayIntent(
                    kind="strategy_select",
                    slot=assignment.slot,
                    policy_id=assignment.policy_id,
                )
            elif kind == "creative_spawn":
                if set(value) != {
                    "version",
                    "session",
                    "sequence",
                    "kind",
                    "item_id",
                }:
                    raise RuntimeError("invalid creative-spawn intent schema")
                try:
                    item = CreativeSpawnItem(item_id=value.get("item_id"))
                except CommandParseError as exc:
                    raise RuntimeError("invalid creative-spawn intent") from exc
                intent = OverlayIntent(
                    kind="creative_spawn",
                    item_id=item.item_id,
                )
            elif kind == "navigation_refresh":
                if set(value) != {
                    "version",
                    "session",
                    "sequence",
                    "kind",
                }:
                    raise RuntimeError("invalid navigation-refresh intent schema")
                intent = OverlayIntent(kind="navigation_refresh")
            elif kind == "navigation_select":
                destination_id = value.get("destination_id")
                if (
                    set(value)
                    != {
                        "version",
                        "session",
                        "sequence",
                        "kind",
                        "destination_id",
                    }
                    or not isinstance(destination_id, str)
                    or re.fullmatch(r"[a-z0-9][a-z0-9._-]{0,63}", destination_id)
                    is None
                ):
                    raise RuntimeError("invalid navigation-selection intent schema")
                intent = OverlayIntent(
                    kind="navigation_select",
                    destination_id=destination_id,
                )
            elif kind == "video_setting":
                if set(value) != {
                    "version",
                    "session",
                    "sequence",
                    "kind",
                    "field",
                    "value",
                    "expected_revision",
                }:
                    raise RuntimeError("invalid video-setting intent schema")
                expected_revision = value.get("expected_revision")
                if (
                    type(expected_revision) is not int
                    or not 0 <= expected_revision < 2**63
                ):
                    raise RuntimeError("invalid video-setting intent revision")
                field = value.get("field")
                try:
                    VideoSettings().with_patch({field: value.get("value")})
                except (TypeError, ValueError, VideoSettingsError) as exc:
                    raise RuntimeError("invalid video-setting intent value") from exc
                intent = OverlayIntent(
                    kind="video_setting",
                    video_field=field,
                    video_value=value.get("value"),
                    expected_revision=expected_revision,
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
        try:
            if process is None:
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
        finally:
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
        "--keyboard-double-tap-window-s",
        type=float,
        default=0.30,
        help="Same-key press-release-press boost window in seconds",
    )
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
    parser.add_argument("--gamepad-move-deadzone", type=float, default=0.15)
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
        "--video-settings-file",
        type=Path,
        default=None,
        help="Host-scoped next-launch video settings",
    )
    parser.add_argument(
        "--applied-video-settings-json",
        help="Validated video settings snapshot frozen by the launcher",
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
    parser.add_argument(
        "--strategy-loadout-json",
        help="Initial resident strategy-slot state supplied by the physics runtime",
    )
    parser.add_argument(
        "--creative-inventory-json",
        help="Initial creative inventory state supplied by the physics runtime",
    )
    parser.add_argument(
        "--motion-settings-json",
        help="Initial runtime-owned six-gear settings telemetry",
    )
    parser.add_argument(
        "--external-control-socket",
        type=Path,
        help="Same-UID authenticated AF_UNIX automation endpoint",
    )
    parser.add_argument(
        "--external-control-capability-file",
        type=Path,
        help="Private 32-byte capability file for external-control clients",
    )
    parser.add_argument(
        "--external-control-deadman-seconds",
        type=float,
        default=0.15,
    )
    parser.add_argument(
        "--celestial-catalog",
        type=Path,
        default=DEFAULT_CATALOG_PATH,
        help="Strict SOL universe/body/destination catalog",
    )
    parser.add_argument(
        "--celestial-clock-state-file",
        type=Path,
        help="Optional persistent TAI clock state shared across cold reloads",
    )
    parser.add_argument(
        "--celestial-assets-manifest",
        type=Path,
        default=DEFAULT_ASSET_MANIFEST_PATH,
    )
    parser.add_argument("--celestial-de440s-kernel", type=Path)
    parser.add_argument("--celestial-jplephem-wheel", type=Path)
    parser.add_argument(
        "--celestial-lighting-bridge",
        choices=("state-only", "carla-weather"),
        default="state-only",
        help="Optional readback-verified CARLA visual-profile bridge",
    )
    parser.add_argument(
        "--celestial-visual-catalog",
        type=Path,
        default=DEFAULT_VISUAL_CATALOG_PATH,
    )
    parser.add_argument("--celestial-visual-profile", default="auto")
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
    if (
        not math.isfinite(args.gamepad_move_deadzone)
        or not 0.0 <= args.gamepad_move_deadzone < 1.0
    ):
        raise SystemExit("--gamepad-move-deadzone must be finite and in [0, 1)")
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
    if args.video_settings_file is not None and not args.video_settings_file.is_absolute():
        raise SystemExit("--video-settings-file must be absolute")
    if args.applied_video_settings_json is not None:
        try:
            decode_applied_video_settings(args.applied_video_settings_json)
        except ValueError as exc:
            raise SystemExit(str(exc)) from exc
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
    game_command_fd = getattr(args, "game_command_fd", None)
    if game_command_fd is not None:
        if game_command_fd < 0:
            raise SystemExit("--game-command-fd must be non-negative")
        try:
            os.fstat(game_command_fd)
        except OSError as exc:
            raise SystemExit(f"--game-command-fd is not open: {exc}") from exc
    external_values = (
        args.external_control_socket,
        args.external_control_capability_file,
    )
    if any(value is not None for value in external_values) and not all(
        value is not None for value in external_values
    ):
        raise SystemExit("external control socket and capability file are all-or-none")
    for name in ("external_control_socket", "external_control_capability_file"):
        path = getattr(args, name)
        if path is not None and not path.is_absolute():
            raise SystemExit(f"--{name.replace('_', '-')} must be absolute")
    if args.external_control_socket is not None:
        assert args.external_control_capability_file is not None
        external_paths = {
            args.external_control_socket.resolve(strict=False),
            args.external_control_capability_file.resolve(strict=False),
        }
        if len(external_paths) != 2:
            raise SystemExit("external control socket and capability must be distinct")
        calibration_path = args.calibration_state_file or args.socket.with_name(
            f"{args.socket.name}.calibration.json"
        )
        reserved_paths = (
            args.socket,
            calibration_path,
            args.mouse_settings_file,
            args.status_file,
            args.restart_request_file,
            args.restart_capability_file,
            args.ue_camera_state_file,
        )
        for reserved in reserved_paths:
            if (
                reserved is not None
                and reserved.resolve(strict=False) in external_paths
            ):
                raise SystemExit(
                    "external control paths must be distinct from provider IPC/state paths"
                )
    if (
        not math.isfinite(args.external_control_deadman_seconds)
        or not 0.01 <= args.external_control_deadman_seconds <= 0.15
    ):
        raise SystemExit("--external-control-deadman-seconds must be in [0.01, 0.15]")
    if not args.celestial_catalog.is_absolute():
        raise SystemExit("--celestial-catalog must be an absolute path")
    if not args.celestial_catalog.is_file() or args.celestial_catalog.is_symlink():
        raise SystemExit(
            f"--celestial-catalog must be a regular file: {args.celestial_catalog}"
        )
    if args.celestial_clock_state_file is not None:
        if not args.celestial_clock_state_file.is_absolute():
            raise SystemExit("--celestial-clock-state-file must be an absolute path")
        if (
            args.celestial_clock_state_file.exists()
            and (
                args.celestial_clock_state_file.is_symlink()
                or not args.celestial_clock_state_file.is_file()
            )
        ):
            raise SystemExit(
                "--celestial-clock-state-file must be a regular file when present"
            )
    if (
        not args.celestial_assets_manifest.is_absolute()
        or args.celestial_assets_manifest.is_symlink()
        or not args.celestial_assets_manifest.is_file()
    ):
        raise SystemExit("--celestial-assets-manifest must be an absolute regular file")
    if (
        not args.celestial_visual_catalog.is_absolute()
        or args.celestial_visual_catalog.is_symlink()
        or not args.celestial_visual_catalog.is_file()
    ):
        raise SystemExit("--celestial-visual-catalog must be an absolute regular file")
    ephemeris_assets = (
        args.celestial_de440s_kernel,
        args.celestial_jplephem_wheel,
    )
    if any(value is not None for value in ephemeris_assets) and not all(
        value is not None for value in ephemeris_assets
    ):
        raise SystemExit("DE440s kernel and jplephem wheel are all-or-none")
    for name in ("celestial_de440s_kernel", "celestial_jplephem_wheel"):
        path = getattr(args, name)
        if path is not None and (
            not path.is_absolute() or path.is_symlink() or not path.is_file()
        ):
            raise SystemExit(
                f"--{name.replace('_', '-')} must be an absolute regular file"
            )


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
    ui_settings_path = default_ui_settings_file()
    loaded_ui = load_ui_settings(ui_settings_path)
    ui_settings = UiSettingsController(
        path=ui_settings_path,
        desired=loaded_ui.settings,
        load_status=loaded_ui.status,
        load_error=loaded_ui.error,
    )
    video_settings_path = args.video_settings_file or default_video_settings_file(
        os.environ.get("MATRIX_HOST_PROFILE", "local")
    )
    video_store = VideoSettingsStore(video_settings_path)
    applied_video = (
        decode_applied_video_settings(args.applied_video_settings_json)
        if args.applied_video_settings_json is not None
        else video_store.settings
    )
    video_settings = VideoSettingsController(
        store=video_store,
        applied=applied_video,
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
            capture_raw_motion=captures_xi2_drag_boundaries(
                args.camera_yaw_source
            ),
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
            font_scale=ui_settings.desired.font_scale,
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
    initial_strategy_loadout: object = None
    initial_creative_inventory: object = None
    if args.strategy_loadout_json is not None:
        try:
            initial_strategy_loadout = json.loads(args.strategy_loadout_json)
        except json.JSONDecodeError as exc:
            raise SystemExit(
                f"Matrix game-control input received invalid strategy loadout: {exc}"
            ) from exc
    if args.creative_inventory_json is not None:
        try:
            initial_creative_inventory = json.loads(args.creative_inventory_json)
        except json.JSONDecodeError as exc:
            raise SystemExit(
                f"Matrix game-control input received invalid creative inventory: {exc}"
            ) from exc
    initial_motion_settings: dict[str, object] | None = None
    if args.motion_settings_json is not None:
        try:
            decoded_motion_settings = json.loads(args.motion_settings_json)
            initial_motion_settings = validate_motion_settings_telemetry(
                decoded_motion_settings
            )
        except (json.JSONDecodeError, ValueError) as exc:
            raise SystemExit(
                f"Matrix game-control input received invalid motion settings: {exc}"
            ) from exc
    try:
        celestial_catalog = load_catalog(
            args.celestial_catalog,
            de440s_kernel=args.celestial_de440s_kernel,
            jplephem_wheel=args.celestial_jplephem_wheel,
            asset_manifest=args.celestial_assets_manifest,
        )
        celestial_clock = celestial_catalog.create_clock(
            args.celestial_clock_state_file
        )
        celestial_visual_catalog = load_visual_catalog(
            args.celestial_visual_catalog
        )
        if args.celestial_visual_profile != "auto":
            celestial_visual_catalog.profile(args.celestial_visual_profile)
        celestial_lighting_bridge = (
            CarlaCelestialLightingBridge(args.carla_host, args.carla_port)
            if args.celestial_lighting_bridge == "carla-weather"
            else None
        )
    except (
        CelestialNavigationError,
        CelestialEphemerisError,
        CelestialVisualError,
    ) as exc:
        raise SystemExit(
            f"Matrix game-control input cannot load celestial catalog: {exc}"
        ) from exc
    try:
        game_command_client = GameCommandClient(
            getattr(args, "game_command_fd", None),
            initial_strategy_loadout=initial_strategy_loadout,
            initial_creative_inventory=initial_creative_inventory,
            initial_motion_settings=initial_motion_settings,
            celestial_catalog=celestial_catalog,
            celestial_clock=celestial_clock,
            celestial_visual_catalog=celestial_visual_catalog,
            celestial_visual_profile=args.celestial_visual_profile,
            celestial_lighting_bridge=celestial_lighting_bridge,
        )
    except (OSError, ValueError) as exc:
        raise SystemExit(
            f"Matrix game-control input cannot initialize command channel: {exc}"
        ) from exc
    calibration = CalibrationModeController()
    shortcut_arming = StartupShortcutArming()
    double_tap = KeyboardDoubleTapDetector(args.keyboard_double_tap_window_s)
    external_control: ExternalControlBroker | None = None
    external_provider_gate: ExternalLocomotionProviderGate | None = None
    external_inflight_command: ExternalCommand | None = None
    if args.external_control_socket is not None:
        assert args.external_control_capability_file is not None
        external_control = ExternalControlBroker(
            args.external_control_socket,
            args.external_control_capability_file,
            deadman_seconds=args.external_control_deadman_seconds,
        )
        external_provider_gate = ExternalLocomotionProviderGate(external_control)

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
    external_telemetry = (
        external_control.telemetry(now=started)
        if external_control is not None
        else None
    )
    external_telemetry_signature: tuple[object, ...] | None = None
    calibration_neutral_frames = 0
    final_pov_observation: UeFinalPovObservation | None = None
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
        if external_control is not None:
            external_control.open()
        if overlay is not None:
            overlay.start(
                {
                    **source_claim,
                    "mouse_settings": mouse_settings.live_mapping(applied_mouse),
                    "ui_settings": ui_settings.live_mapping(),
                    "video_settings": video_settings.live_mapping(),
                    "restart": restart_requester.mapping(),
                    "apply_return": apply_return.mapping(),
                    "command_console": game_command_client.mapping(),
                    "motion_settings": live_motion_settings_telemetry(
                        initial_motion_settings, game_command_client
                    ),
                    "strategy_loadout": game_command_client.strategy_loadout_mapping(),
                    "celestial_navigation": (
                        game_command_client.celestial_navigation_mapping()
                    ),
                    "mirror_sensitivity": sensitivity_telemetry,
                    "pointer": x11.pointer_telemetry,
                    "camera_yaw": camera_yaw_telemetry(
                        args.camera_yaw_source,
                        provider_yaw_rad=provider_yaw,
                        sonic_yaw_rad=camera_yaw,
                    ),
                    "ue_final_pov": ue_final_pov_telemetry(None),
                    "external_control": external_telemetry,
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

            command_state_changed = False
            video_settings_changed = False
            game_command_client.checkpoint_celestial_clock()
            physical_keyboard = x11.poll()
            last_keyboard = physical_keyboard
            physical_pad = gamepad.poll(now)
            panel_intents = overlay.drain_intents() if overlay is not None else ()
            external_control_changed = False
            frame_input_source = input_source
            input_source_id = "physical"
            raw_keyboard = physical_keyboard
            raw_pad = physical_pad
            external_gate_frame: ExternalProviderGateFrame | None = None
            if external_control is not None:
                external_control.poll(now=now)
                override_reason = physical_external_override_reason(
                    physical_keyboard,
                    physical_pad,
                    move_deadzone=args.gamepad_move_deadzone,
                    look_deadzone=args.gamepad_look_deadzone,
                )
                if override_reason is None and panel_intents:
                    override_reason = "physical_panel"
                # Focus is one of the provider's final snapshot interlocks.  It
                # keeps the lease alive just long enough for the client to read
                # typed gate telemetry; actual physical key/mouse/gamepad input
                # still revokes external authority immediately.
                if (
                    override_reason is not None
                    and override_reason != "focus_lost"
                    and external_control.lease_active
                ):
                    external_control.local_override(override_reason)
                    external_control_changed = True
                if external_control.lease_active:
                    external_state, external_token = (
                        external_control.sample_with_token(now=now)
                    )
                    assert external_provider_gate is not None
                    external_state, external_gate_frame = (
                        external_provider_gate.prepare(
                            external_state,
                            external_token,
                        )
                    )
                    if external_gate_frame is not None:
                        external_state, _source_interlock = (
                            apply_external_source_gate(
                                external_state,
                                external_gate_frame,
                                configured_source=input_source,
                            )
                        )
                    raw_keyboard, raw_pad = external_input_samples(
                        external_state,
                        focus=physical_keyboard,
                        look_button=args.look_button,
                    )
                    frame_input_source = external_frame_input_source(
                        external_state,
                        configured_source=input_source,
                    )
                    input_source_id = "external"
            command_state_changed = game_command_client.poll()
            if (
                external_control is not None
                and external_inflight_command is not None
                and command_state_changed
                and not game_command_client.in_flight
            ):
                external_control.complete_command(
                    external_inflight_command,
                    game_command_client.mapping(),
                )
                external_inflight_command = None
                external_control_changed = True
            shortcuts_armed = shortcut_arming.update(
                escape_pressed=raw_keyboard.escape,
                restart_pressed=raw_keyboard.apply_restart,
            )
            panel_was_active = calibration.active
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
            for intent in panel_intents:
                if intent.kind == "action":
                    assert intent.action is not None
                    panel_actions.append(intent.action)
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
                if intent.kind == "strategy_select":
                    assert intent.slot is not None
                    assert intent.policy_id is not None
                    game_command_client.select_policy(
                        intent.slot,
                        intent.policy_id,
                        calibration_active=calibration.active,
                        neutral_frame_ready=neutral_frame_ready,
                        restart_requested=restart_requester.requested,
                    )
                    command_state_changed = True
                    apply_return.cancel_pending()
                    continue
                if intent.kind == "creative_spawn":
                    assert intent.item_id is not None
                    game_command_client.spawn_creative_item(
                        intent.item_id,
                        calibration_active=calibration.active,
                        neutral_frame_ready=neutral_frame_ready,
                        restart_requested=restart_requester.requested,
                    )
                    command_state_changed = True
                    apply_return.cancel_pending()
                    continue
                if intent.kind == "navigation_refresh":
                    game_command_client.refresh_celestial_navigation(
                        calibration_active=calibration.active,
                        neutral_frame_ready=neutral_frame_ready,
                        restart_requested=restart_requester.requested,
                    )
                    command_state_changed = True
                    apply_return.cancel_pending()
                    continue
                if intent.kind == "navigation_select":
                    assert intent.destination_id is not None
                    game_command_client.select_celestial_destination(
                        intent.destination_id,
                        calibration_active=calibration.active,
                        neutral_frame_ready=neutral_frame_ready,
                        restart_requested=restart_requester.requested,
                    )
                    command_state_changed = True
                    apply_return.cancel_pending()
                    continue
                if intent.kind == "video_setting":
                    assert intent.video_field is not None
                    assert intent.expected_revision is not None
                    video_settings_changed = bool(
                        video_settings.apply_intent(
                            intent.video_field,
                            intent.video_value,
                            expected_revision=intent.expected_revision,
                            active=(
                                calibration.active
                                and not restart_requester.requested
                                and not game_command_client.editing
                                and not game_command_client.in_flight
                                and not game_command_client.restart_required
                                and not game_command_client.outcome_unknown
                            ),
                        )
                        or video_settings_changed
                    )
                    apply_return.cancel_pending()
                    continue
                assert intent.kind == "command_submit"
                assert intent.command is not None
                command_submitted = game_command_client.submit(
                    intent.command,
                    calibration_active=calibration.active,
                    neutral_frame_ready=neutral_frame_ready,
                    restart_requested=restart_requester.requested,
                )
                # Local parse/gate failures also change the visible result.
                command_state_changed = True
                if command_submitted:
                    apply_return.cancel_pending()
            if (
                external_control is not None
                and external_control.lease_active
                and not game_command_client.in_flight
                and not game_command_client.restart_required
                and not game_command_client.outcome_unknown
            ):
                external_commands = external_control.drain_commands(limit=1)
                if external_commands:
                    external_command = external_commands[0]

                    def modify_external_input(
                        command: DataModifyInput,
                    ) -> tuple[ExternalInputToken, dict[str, object] | None]:
                        assert external_control is not None
                        modified_token = external_control.apply_data_modify(
                            command.path,
                            command.value,
                            now=now,
                        )
                        return (
                            modified_token,
                            {
                                "request_sequence": external_command.request_sequence,
                                "peer_pid": external_command.peer_pid,
                                "external_control": external_control.telemetry(now=now),
                            },
                        )

                    external_submitted = game_command_client.submit_external(
                        external_command.command,
                        calibration_active=calibration.active,
                        neutral_frame_ready=neutral_frame_ready,
                        restart_requested=restart_requester.requested,
                        input_modifier=modify_external_input,
                    )
                    command_state_changed = True
                    external_control_changed = True
                    if external_submitted:
                        apply_return.cancel_pending()
                    if game_command_client.in_flight:
                        external_inflight_command = external_command
                    else:
                        external_control.complete_command(
                            external_command,
                            game_command_client.mapping(),
                        )
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
                        "strategy_select",
                        "creative_spawn",
                        "navigation_refresh",
                        "navigation_select",
                        "video_setting",
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
            for panel_action in panel_actions:
                mouse_settings_changed = bool(
                    mouse_settings.apply_panel_action(
                        panel_action,
                        active=(
                            calibration.active
                            and not restart_requester.requested
                            and not command_controls_blocked
                        ),
                    )
                    or mouse_settings_changed
                )
                ui_settings_changed = bool(
                    ui_settings.apply_panel_action(
                        panel_action,
                        active=(
                            calibration.active
                            and not restart_requester.requested
                            and not command_controls_blocked
                        ),
                    )
                    or ui_settings_changed
                )
            restart_requested = apply_restart_key.update(
                pressed=raw_keyboard.apply_restart,
                calibration_active=keyboard_panel_active,
                neutral_frame_ready=neutral_frame_ready,
                pending_restart=bool(
                    mouse_settings.pending_restart(applied_mouse)
                    or video_settings.pending_restart()
                ),
                persistence_ok=bool(
                    mouse_settings.persistence_error is None
                    and video_settings.persistence_error is None
                ),
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
                pending_restart=bool(
                    mouse_settings.pending_restart(applied_mouse)
                    or video_settings.pending_restart()
                ),
                persistence_error=(
                    mouse_settings.persistence_error
                    or video_settings.persistence_error
                ),
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
            pointer_telemetry = x11.pointer_telemetry
            teleport_rejections = int(pointer_telemetry["teleport_rejections"])
            gamepad_connected_edge = bool(
                previous_gamepad_connected is not None
                and pad.connected != previous_gamepad_connected
            )
            input_available = gamepad_input_available(
                frame_input_source,
                connected=pad.connected,
                previous_connected=previous_gamepad_connected,
            )
            previous_gamepad_connected = pad.connected
            drive_gamepad_camera = bool(
                carla_reader is not None
                and keyboard.focused
                and input_available
                and pad.connected
                and frame_input_source in {"auto", "gamepad"}
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
            if external_gate_frame is not None:
                external_gate_interlock_reason = (
                    external_provider_publish_interlock_reason(
                        external_gate_frame,
                        configured_source=input_source,
                        physical_focused=physical_keyboard.focused,
                        camera_dragging=keyboard.camera_dragging,
                        camera_available=camera_available,
                        input_available=input_available,
                        gamepad_connected_edge=gamepad_connected_edge,
                        calibration_interlock_active=calibration_interlock_active,
                    )
                )
            else:
                external_gate_interlock_reason = None
            keyboard_boost = double_tap.update(
                keyboard,
                now_s=now,
                enabled=bool(
                    frame_input_source in {"auto", "keyboard"}
                    and keyboard.focused
                    and not keyboard.camera_dragging
                    and camera_available
                    and input_available
                ),
                source_id=input_source_id,
            )
            provider_yaw = tracker.update(
                dt=dt,
                mouse_dx=(
                    keyboard.mouse_dx
                    if args.camera_yaw_source
                    in {"x11-mirror", "x11-core-gated", "x11-absolute"}
                    and frame_input_source != "gamepad"
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
            if external_control is not None:
                external_telemetry = external_control.telemetry(now=now)
                next_external_signature = (
                    external_telemetry["connected_clients"],
                    external_telemetry["lease_active"],
                    external_telemetry["lease_owner_pid"],
                    external_telemetry["command_queue_depth"],
                    external_telemetry["deadman_stops"],
                    external_telemetry["local_overrides"],
                    external_telemetry["commands_queued"],
                    external_telemetry["last_override_reason"],
                    json.dumps(
                        external_telemetry["input_token"],
                        sort_keys=True,
                    ),
                    json.dumps(
                        external_telemetry["provider_gate"],
                        sort_keys=True,
                    ),
                )
                if next_external_signature != external_telemetry_signature:
                    external_control_changed = True
                    external_telemetry_signature = next_external_signature
            # Publish input counters and the yaw produced from that exact same
            # poll.  Telemetry stays downstream of every safety decision and
            # never feeds the tracker or snapshot interlocks.
            if overlay is not None:
                overlay.ensure_running()
                if (
                    calibration_toggled
                    or left_calibration
                    or bool(panel_intents)
                    or command_state_changed
                    or mouse_settings_changed
                    or ui_settings_changed
                    or video_settings_changed
                    or restart_requested
                    or external_control_changed
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
                            "video_settings": video_settings.live_mapping(),
                            "restart": restart_requester.mapping(),
                            "apply_return": apply_return.mapping(),
                            "command_console": game_command_client.mapping(),
                            "motion_settings": live_motion_settings_telemetry(
                                initial_motion_settings, game_command_client
                            ),
                            "strategy_loadout": (
                                game_command_client.strategy_loadout_mapping()
                            ),
                            "creative_inventory": (
                                game_command_client.creative_inventory_mapping()
                            ),
                            "celestial_navigation": (
                                game_command_client.celestial_navigation_mapping()
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
                            "pointer": pointer_telemetry,
                            "keyboard_double_tap": double_tap.telemetry,
                            "external_control": external_telemetry,
                        }
                    )
                    next_overlay_heartbeat = now + 1.0
            last_teleport_rejections = teleport_rejections
            snapshot = build_snapshot(
                sequence=sequence,
                timestamp_monotonic_s=now,
                keyboard=keyboard,
                gamepad=pad,
                input_source=frame_input_source,
                camera_yaw_rad=camera_yaw,
                camera_available=camera_available,
                input_available=input_available,
                keyboard_boost=keyboard_boost,
            )
            snapshot = apply_external_publish_interlock(
                snapshot,
                external_gate_frame,
                external_gate_interlock_reason,
            )
            publish_now = time.monotonic()
            publish_boundary_token: ExternalInputToken | None = None
            publish_boundary_exact = False
            if external_control is not None:
                boundary = external_publish_boundary(
                    external_control,
                    external_gate_frame,
                    snapshot,
                    now=publish_now,
                )
                snapshot = boundary.snapshot
                publish_boundary_token = boundary.current_token
                publish_boundary_exact = boundary.exact_revision
            last_snapshot = snapshot
            neutral_delivered = False
            if publisher is None:
                print(encode_input_packet(snapshot).decode("ascii"), flush=True)
                sent_frames += 1
                neutral_delivered = True
            elif publisher.send(snapshot, now=publish_now):
                sent_frames += 1
                neutral_delivered = True
            provider_packet_published = bool(
                publisher is not None and neutral_delivered
            )
            exact_external_published = bool(
                provider_packet_published and publish_boundary_exact
            )
            if external_gate_frame is not None:
                assert external_provider_gate is not None
                external_provider_gate.observe_published(
                    external_gate_frame,
                    sequence=snapshot.sequence,
                    # Printing a dry-run packet is useful diagnostics, but it
                    # is not proof that the core accepted a provider frame.
                    # A successfully sent boundary-neutral packet also counts
                    # as a stale-frame send success here, so same-loop R1->R2
                    # cannot invalidate R2's inherited proof.  It still cannot
                    # resolve the pending R2 command below.
                    published=provider_packet_published,
                    interlock_reason=external_gate_interlock_reason,
                )
            if (
                external_control is not None
                and external_inflight_command is not None
            ):
                persistent_gate_interlock = (
                    external_control.provider_gate.last_interlock_reason
                    if external_control.provider_gate.phase == "interlocked"
                    else None
                )
                publish_resolved = game_command_client.resolve_external_input_publish(
                    sampled_token=(
                        external_gate_frame.token
                        if external_gate_frame is not None
                        else None
                    ),
                    current_token=publish_boundary_token,
                    authority_active=publish_boundary_token is not None,
                    published=exact_external_published,
                    locomotion_admitted=bool(
                        external_gate_frame is not None
                        and external_gate_frame.locomotion_admitted
                    ),
                    interlock_reason=(
                        external_gate_interlock_reason
                        or persistent_gate_interlock
                    ),
                    data={
                        "request_sequence": (
                            external_inflight_command.request_sequence
                        ),
                        "peer_pid": external_inflight_command.peer_pid,
                        "external_control": external_control.telemetry(
                            now=publish_now
                        ),
                    },
                )
                if publish_resolved:
                    external_control.complete_command(
                        external_inflight_command,
                        game_command_client.mapping(),
                    )
                    external_inflight_command = None
                    command_state_changed = True
                    external_control_changed = True
            if calibration.active:
                calibration_neutral_frames = (
                    calibration_neutral_frames + 1 if neutral_delivered else 0
                )
            sequence += 1
            sampled_frames += 1
        if exit_reason == "unknown":
            exit_reason = "signal"
    except Exception as exc:
        exit_reason = f"error:{type(exc).__name__}"
        print(f"matrix-game-control-input ERROR {exc}", file=sys.stderr, flush=True)
        return_code = 1
    finally:
        cleanup = _CleanupCoordinator()

        # A focused=false release is immediate; the core's independent 0.15 s
        # deadman threshold remains authoritative if the connection is gone.
        if publisher is not None and last_snapshot is not None:
            def publish_release() -> None:
                release = InputSnapshot(
                    sequence=sequence,
                    timestamp_monotonic_s=time.monotonic(),
                    focused=False,
                    camera_yaw_rad=last_snapshot.camera_yaw_rad,
                    keyboard_boost=False,
                    keys=KeySnapshot(
                        False, False, False, False, False, False, False
                    ),
                    move_stick=MoveStickSnapshot(0.0, 0.0),
                )
                publisher.send(release, now=time.monotonic())

            cleanup.run("publisher_release", publish_release)
        # Resolve a response already queued at the shutdown boundary, or mark
        # a successfully sent but unacknowledged command outcome-unknown.  This
        # must happen before the final status snapshot is serialized.
        cleanup.run("command_receipt", game_command_client.close)
        if external_control is not None and external_inflight_command is not None:
            cleanup.run(
                "external_command_receipt",
                lambda: external_control.complete_command(
                    external_inflight_command,
                    game_command_client.mapping(),
                ),
            )
            external_inflight_command = None
            external_telemetry = cleanup.run(
                "external_telemetry",
                external_control.telemetry,
                default=external_telemetry,
            )

        def build_final_status() -> dict[str, object]:
            return {
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
                "video_settings": video_settings.live_mapping(),
                "mirror_sensitivity": sensitivity_telemetry,
                "camera_yaw": camera_yaw_telemetry(
                    args.camera_yaw_source,
                    provider_yaw_rad=provider_yaw,
                    sonic_yaw_rad=camera_yaw,
                ),
                "ue_final_pov": ue_final_pov_telemetry(
                    final_pov_observation
                ),
                "restart": restart_requester.mapping(),
                "apply_return": apply_return.mapping(),
                "command_console": game_command_client.mapping(),
                "motion_settings": live_motion_settings_telemetry(
                    initial_motion_settings, game_command_client
                ),
                "strategy_loadout": game_command_client.strategy_loadout_mapping(),
                "celestial_navigation": (
                    game_command_client.celestial_navigation_mapping()
                ),
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
                "keyboard_double_tap": double_tap.telemetry,
                "external_control": external_telemetry,
                "last_snapshot": last_snapshot.to_mapping()
                if last_snapshot is not None
                else None,
            }

        # Capture telemetry while the underlying resources are still intact.
        # Serialization is deliberately delayed until every close and signal
        # restoration attempt has run, so a status write failure cannot strand
        # provider-owned resources.
        final_status = cleanup.run(
            "status_prepare",
            build_final_status,
            default=None,
        )
        cleanup.run(
            "celestial_resources",
            game_command_client.finalize_celestial_resources,
        )
        _close_provider_resources(
            cleanup,
            gamepad=gamepad,
            overlay=overlay,
            x11=x11,
            publisher=publisher,
            external_control=external_control,
            previous_handlers=previous_handlers,
        )
        return_code, exit_reason = _cleanup_outcome(
            cleanup,
            return_code=return_code,
            exit_reason=exit_reason,
        )
        if final_status is not None:
            final_status["completed"] = return_code == 0
            final_status["exit_reason"] = exit_reason
            final_status["cleanup_errors"] = list(cleanup.failures)
            failures_before_status = len(cleanup.failures)
            cleanup.run(
                "status_write",
                lambda: _atomic_json(args.status_file, final_status),
            )
            if len(cleanup.failures) != failures_before_status:
                return_code, exit_reason = _cleanup_outcome(
                    cleanup,
                    return_code=return_code,
                    exit_reason=exit_reason,
                )
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
