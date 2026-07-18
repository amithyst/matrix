#!/usr/bin/env python3
"""Camera-relative third-person control primitives for Matrix + SONIC.

This module deliberately does not capture keyboard/mouse events and does not
talk to SONIC.  A Matrix UI input adapter publishes strict snapshots, this
module turns those snapshots into a small, deterministic motion command, and a
separate runtime adapter sends the command through SONIC's native planner wire.

The local input protocol uses an AF_UNIX/SOCK_SEQPACKET socket instead of a UDP
port.  That gives ordered, reliable, message-bounded delivery without exposing
an input listener on the network.  The server additionally restricts the
socket to mode 0600 and verifies Linux SO_PEERCRED credentials.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import socket
import stat
import struct
from typing import Any, Mapping, Sequence


PROTOCOL_NAME = "matrix-game-input/v2"
MAX_PACKET_BYTES = 4096
_UNSET_SOCKET_TIMEOUT = object()
_KEY_NAMES = frozenset(("w", "a", "s", "d", "q", "e", "v", "ctrl", "shift"))
_STICK_NAMES = frozenset(("right", "forward"))
_TOP_LEVEL_NAMES = frozenset(
    (
        "protocol",
        "sequence",
        "timestamp_monotonic_s",
        "focused",
        "camera_yaw_rad",
        "keys",
        "move_stick",
    )
)


class InputProtocolError(ValueError):
    """The input packet is malformed or violates the protocol schema."""


class InputRejectedError(ValueError):
    """A valid snapshot is unsafe to apply (stale, future, or replayed)."""


def _finite_number(value: Any, *, name: str, nonnegative: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise InputProtocolError(f"{name} must be a number")
    try:
        result = float(value)
    except (OverflowError, ValueError) as exc:
        raise InputProtocolError(f"{name} must be a finite number") from exc
    if not math.isfinite(result):
        raise InputProtocolError(f"{name} must be finite")
    if nonnegative and result < 0.0:
        raise InputProtocolError(f"{name} must be nonnegative")
    return result


def _strict_mapping(value: Any, *, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise InputProtocolError(f"{name} must be an object")
    if not all(isinstance(key, str) for key in value):
        raise InputProtocolError(f"{name} keys must be strings")
    return value


def _require_exact_keys(
    value: Mapping[str, Any], expected: frozenset[str], *, name: str
) -> None:
    actual = frozenset(value)
    missing = sorted(expected - actual)
    unknown = sorted(actual - expected)
    if missing:
        raise InputProtocolError(f"{name} is missing fields: {', '.join(missing)}")
    if unknown:
        raise InputProtocolError(f"{name} has unknown fields: {', '.join(unknown)}")


@dataclass(frozen=True)
class KeySnapshot:
    """Current key state, including keyboard-only speed modifiers.

    Q/E are carried for future actions but never contribute to locomotion.
    Ctrl/Shift select a digital-WASD speed tier; they do not quantize gamepad
    input, whose magnitude remains continuous.
    """

    w: bool
    a: bool
    s: bool
    d: bool
    q: bool
    e: bool
    v: bool
    ctrl: bool = False
    shift: bool = False

    def __post_init__(self) -> None:
        for name in _KEY_NAMES:
            if type(getattr(self, name)) is not bool:
                raise InputProtocolError(f"keys.{name} must be a boolean")

    @classmethod
    def from_mapping(cls, value: Any) -> "KeySnapshot":
        mapping = _strict_mapping(value, name="keys")
        _require_exact_keys(mapping, _KEY_NAMES, name="keys")
        return cls(**{name: mapping[name] for name in _KEY_NAMES})

    def to_mapping(self) -> dict[str, bool]:
        return {name: getattr(self, name) for name in sorted(_KEY_NAMES)}


@dataclass(frozen=True)
class MoveStickSnapshot:
    """Platform-neutral left stick: right and forward are both in [-1, 1]."""

    right: float
    forward: float

    def __post_init__(self) -> None:
        right = _finite_number(self.right, name="move_stick.right")
        forward = _finite_number(self.forward, name="move_stick.forward")
        if not -1.0 <= right <= 1.0:
            raise InputProtocolError("move_stick.right must be in [-1, 1]")
        if not -1.0 <= forward <= 1.0:
            raise InputProtocolError("move_stick.forward must be in [-1, 1]")
        object.__setattr__(self, "right", right)
        object.__setattr__(self, "forward", forward)

    @classmethod
    def from_mapping(cls, value: Any) -> "MoveStickSnapshot":
        mapping = _strict_mapping(value, name="move_stick")
        _require_exact_keys(mapping, _STICK_NAMES, name="move_stick")
        return cls(right=mapping["right"], forward=mapping["forward"])

    def to_mapping(self) -> dict[str, float]:
        return {"right": self.right, "forward": self.forward}


@dataclass(frozen=True)
class InputSnapshot:
    """One complete input state sampled against the host monotonic clock."""

    sequence: int
    timestamp_monotonic_s: float
    focused: bool
    camera_yaw_rad: float
    keys: KeySnapshot
    move_stick: MoveStickSnapshot
    protocol: str = PROTOCOL_NAME

    def __post_init__(self) -> None:
        if self.protocol != PROTOCOL_NAME:
            raise InputProtocolError(
                f"protocol must be {PROTOCOL_NAME!r}, got {self.protocol!r}"
            )
        if isinstance(self.sequence, bool) or not isinstance(self.sequence, int):
            raise InputProtocolError("sequence must be an integer")
        if not 0 <= self.sequence <= (2**63 - 1):
            raise InputProtocolError("sequence must be in [0, 2^63 - 1]")
        timestamp = _finite_number(
            self.timestamp_monotonic_s,
            name="timestamp_monotonic_s",
            nonnegative=True,
        )
        if type(self.focused) is not bool:
            raise InputProtocolError("focused must be a boolean")
        yaw = _finite_number(self.camera_yaw_rad, name="camera_yaw_rad")
        if not isinstance(self.keys, KeySnapshot):
            raise InputProtocolError("keys must be a KeySnapshot")
        if not isinstance(self.move_stick, MoveStickSnapshot):
            raise InputProtocolError("move_stick must be a MoveStickSnapshot")
        object.__setattr__(self, "timestamp_monotonic_s", timestamp)
        object.__setattr__(self, "camera_yaw_rad", yaw)

    @classmethod
    def from_mapping(cls, value: Any) -> "InputSnapshot":
        mapping = _strict_mapping(value, name="input snapshot")
        _require_exact_keys(mapping, _TOP_LEVEL_NAMES, name="input snapshot")
        return cls(
            protocol=mapping["protocol"],
            sequence=mapping["sequence"],
            timestamp_monotonic_s=mapping["timestamp_monotonic_s"],
            focused=mapping["focused"],
            camera_yaw_rad=mapping["camera_yaw_rad"],
            keys=KeySnapshot.from_mapping(mapping["keys"]),
            move_stick=MoveStickSnapshot.from_mapping(mapping["move_stick"]),
        )

    def to_mapping(self) -> dict[str, Any]:
        return {
            "protocol": self.protocol,
            "sequence": self.sequence,
            "timestamp_monotonic_s": self.timestamp_monotonic_s,
            "focused": self.focused,
            "camera_yaw_rad": self.camera_yaw_rad,
            "keys": self.keys.to_mapping(),
            "move_stick": self.move_stick.to_mapping(),
        }


def _reject_json_constant(value: str) -> None:
    raise InputProtocolError(f"JSON constant {value!r} is not allowed")


def _object_without_duplicate_keys(pairs: Sequence[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise InputProtocolError(f"duplicate JSON field: {key}")
        result[key] = value
    return result


def decode_input_packet(payload: bytes) -> InputSnapshot:
    """Decode one bounded, UTF-8, duplicate-free JSON snapshot packet."""

    if not isinstance(payload, bytes):
        raise InputProtocolError("input packet must be bytes")
    if not payload:
        raise InputProtocolError("input packet is empty")
    if len(payload) > MAX_PACKET_BYTES:
        raise InputProtocolError(
            f"input packet exceeds {MAX_PACKET_BYTES} byte limit"
        )
    try:
        text = payload.decode("utf-8", errors="strict")
    except UnicodeDecodeError as exc:
        raise InputProtocolError("input packet is not valid UTF-8") from exc
    try:
        value = json.loads(
            text,
            object_pairs_hook=_object_without_duplicate_keys,
            parse_constant=_reject_json_constant,
        )
    except InputProtocolError:
        raise
    except (json.JSONDecodeError, TypeError, RecursionError) as exc:
        raise InputProtocolError(f"input packet is not valid JSON: {exc}") from exc
    return InputSnapshot.from_mapping(value)


def encode_input_packet(snapshot: InputSnapshot) -> bytes:
    """Encode a validated snapshot into the canonical compact JSON form."""

    if not isinstance(snapshot, InputSnapshot):
        raise InputProtocolError("snapshot must be an InputSnapshot")
    payload = json.dumps(
        snapshot.to_mapping(),
        allow_nan=False,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("ascii")
    if len(payload) > MAX_PACKET_BYTES:
        raise InputProtocolError(
            f"encoded input packet exceeds {MAX_PACKET_BYTES} byte limit"
        )
    return payload


def wrap_angle_rad(value: float) -> float:
    """Wrap a finite angle to [-pi, pi]."""

    value = _finite_number(value, name="angle")
    return math.atan2(math.sin(value), math.cos(value))


def camera_relative_to_world(
    *, right: float, forward: float, camera_yaw_rad: float
) -> tuple[float, float]:
    """Project a camera-local horizontal vector into Matrix world XY.

    Camera pitch and roll are intentionally absent from this interface.  Yaw
    zero means camera-forward is world +X.  SONIC uses a right-handed
    x-forward/y-left frame, so camera-right is world -Y at zero yaw.
    """

    right = _finite_number(right, name="right")
    forward = _finite_number(forward, name="forward")
    yaw = wrap_angle_rad(camera_yaw_rad)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        (forward * cosine) + (right * sine),
        (forward * sine) - (right * cosine),
    )


def apply_radial_deadzone(
    *, right: float, forward: float, deadzone: float
) -> tuple[float, float]:
    """Apply a circular deadzone and remap the remaining stick range to [0, 1]."""

    right = _finite_number(right, name="right")
    forward = _finite_number(forward, name="forward")
    deadzone = _finite_number(deadzone, name="deadzone", nonnegative=True)
    if deadzone >= 1.0:
        raise ValueError("deadzone must be less than 1")
    magnitude = math.hypot(right, forward)
    if magnitude <= deadzone or magnitude <= 1e-12:
        return (0.0, 0.0)
    unit_right = right / magnitude
    unit_forward = forward / magnitude
    clamped_magnitude = min(magnitude, 1.0)
    remapped_magnitude = (clamped_magnitude - deadzone) / (1.0 - deadzone)
    return (unit_right * remapped_magnitude, unit_forward * remapped_magnitude)


def _move_toward(current: float, target: float, maximum_delta: float) -> float:
    if target > current:
        return min(target, current + maximum_delta)
    return max(target, current - maximum_delta)


@dataclass(frozen=True)
class ControlConfig:
    """Tuning and safety limits for :class:`GameControlCore`."""

    max_speed_mps: float = 0.30
    max_acceleration_mps2: float = 1.20
    max_deceleration_mps2: float = 2.40
    max_turn_rate_rad_s: float = 2.50
    min_gait_speed_mps: float = 0.10
    gait_start_speed_mps: float = 0.10
    gait_stop_speed_mps: float = 0.08
    gait_start_heading_error_rad: float = math.radians(15.0)
    gait_stop_heading_error_rad: float = math.radians(30.0)
    stick_deadzone: float = 0.15
    input_timeout_s: float = 0.15
    max_snapshot_age_s: float = 0.15
    max_future_skew_s: float = 0.05
    max_step_s: float = 0.10
    speed_epsilon_mps: float = 1e-4

    def __post_init__(self) -> None:
        positive = (
            "max_speed_mps",
            "max_acceleration_mps2",
            "max_deceleration_mps2",
            "max_turn_rate_rad_s",
            "min_gait_speed_mps",
            "gait_start_speed_mps",
            "gait_stop_speed_mps",
            "gait_start_heading_error_rad",
            "gait_stop_heading_error_rad",
            "input_timeout_s",
            "max_snapshot_age_s",
            "max_step_s",
            "speed_epsilon_mps",
        )
        for name in positive:
            value = _finite_number(getattr(self, name), name=name)
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")
            object.__setattr__(self, name, value)
        future_skew = _finite_number(
            self.max_future_skew_s, name="max_future_skew_s", nonnegative=True
        )
        deadzone = _finite_number(
            self.stick_deadzone, name="stick_deadzone", nonnegative=True
        )
        if deadzone >= 1.0:
            raise ValueError("stick_deadzone must be less than 1")
        if not (
            self.gait_stop_speed_mps < self.min_gait_speed_mps
            and self.min_gait_speed_mps == self.gait_start_speed_mps
            and self.gait_start_speed_mps <= self.max_speed_mps
        ):
            raise ValueError(
                "gait speeds must satisfy stop < minimum == start <= maximum"
            )
        if not (
            self.gait_start_heading_error_rad
            < self.gait_stop_heading_error_rad
            <= (math.pi / 2.0)
        ):
            raise ValueError(
                "gait heading errors must satisfy start < stop <= pi/2"
            )
        slow_speed_at_start_edge = self.min_gait_speed_mps * math.cos(
            self.gait_start_heading_error_rad
        )
        if self.gait_stop_speed_mps > slow_speed_at_start_edge:
            raise ValueError(
                "gait stop speed must not overlap the slow-tier heading start edge"
            )
        object.__setattr__(self, "max_future_skew_s", future_skew)
        object.__setattr__(self, "stick_deadzone", deadzone)


@dataclass(frozen=True)
class RobotMotionCommand:
    """SONIC-friendly direction/facing plus a scalar speed."""

    sequence: int | None
    movement: tuple[float, float, float]
    facing: tuple[float, float, float]
    speed_mps: float
    mode: str
    safe_stop: bool
    reason: str | None


class GameControlCore:
    """Stateful camera-relative locomotion with deterministic safety stops."""

    def __init__(
        self,
        config: ControlConfig | None = None,
        *,
        initial_heading_rad: float = 0.0,
    ) -> None:
        self.config = config or ControlConfig()
        self._command_heading_rad = wrap_angle_rad(initial_heading_rad)
        self._measured_heading_rad: float | None = None
        self._speed_mps = 0.0
        self._gait_active = False
        self._snapshot: InputSnapshot | None = None
        self._last_received_at_s: float | None = None
        self._last_sequence: int | None = None
        self._free_camera = False
        self._v_was_down = False
        self._turn_sign = 1.0
        self._requires_neutral_rearm = True
        self._invalid_reason: str | None = None

    @property
    def free_camera(self) -> bool:
        return self._free_camera

    @property
    def heading_rad(self) -> float:
        """Latest rate-limited facing command in the normalized SONIC frame."""

        return self._command_heading_rad

    @property
    def measured_heading_rad(self) -> float | None:
        """Latest physical base heading, or ``None`` before runtime feedback."""

        return self._measured_heading_rad

    def synchronize_heading(self, heading_rad: float) -> None:
        """Update physical yaw feedback without overwriting the facing target."""

        self._measured_heading_rad = wrap_angle_rad(heading_rad)

    def invalidate_input(self, reason: str = "input_invalidated") -> None:
        """Stop immediately and require neutral input before re-arming."""

        if not isinstance(reason, str) or not reason:
            raise ValueError("input invalidation reason must be a non-empty string")
        self._snapshot = None
        self._last_received_at_s = None
        self._speed_mps = 0.0
        self._gait_active = False
        self._requires_neutral_rearm = True
        self._invalid_reason = reason

    def accept_snapshot(
        self, snapshot: InputSnapshot, *, received_at_s: float
    ) -> None:
        """Validate freshness/order and atomically install an input snapshot.

        Rejected snapshots never refresh the deadman timer.
        """

        if not isinstance(snapshot, InputSnapshot):
            raise InputRejectedError("snapshot must be an InputSnapshot")
        try:
            received_at = _finite_number(
                received_at_s, name="received_at_s", nonnegative=True
            )
        except InputProtocolError as exc:
            raise InputRejectedError(str(exc)) from exc
        if self._last_sequence is not None and snapshot.sequence <= self._last_sequence:
            raise InputRejectedError(
                f"sequence must increase: got {snapshot.sequence} after "
                f"{self._last_sequence}"
            )
        age = received_at - snapshot.timestamp_monotonic_s
        if age > self.config.max_snapshot_age_s:
            raise InputRejectedError(
                f"snapshot is stale by {age:.6f}s "
                f"(limit {self.config.max_snapshot_age_s:.6f}s)"
            )
        if age < -self.config.max_future_skew_s:
            raise InputRejectedError(
                f"snapshot is {-age:.6f}s in the future "
                f"(limit {self.config.max_future_skew_s:.6f}s)"
            )

        # V is an edge-triggered mode toggle.  Focus loss never creates an
        # edge, and holding V across refocus cannot toggle the mode by accident.
        if snapshot.focused and snapshot.keys.v and not self._v_was_down:
            self._free_camera = not self._free_camera
            self._requires_neutral_rearm = True
        self._v_was_down = snapshot.keys.v
        if not snapshot.focused:
            self._requires_neutral_rearm = True
        digital_neutral = not any(
            (snapshot.keys.w, snapshot.keys.a, snapshot.keys.s, snapshot.keys.d)
        )
        analog_neutral = math.hypot(
            snapshot.move_stick.right, snapshot.move_stick.forward
        ) <= self.config.stick_deadzone
        if (
            snapshot.focused
            and not self._free_camera
            and digital_neutral
            and analog_neutral
        ):
            self._requires_neutral_rearm = False
        self._snapshot = snapshot
        self._last_received_at_s = received_at
        self._last_sequence = snapshot.sequence
        self._invalid_reason = None

    def _safe_stop(self, *, reason: str, deadman: bool) -> RobotMotionCommand:
        # Safety conditions bypass smoothing.  In particular, stale input,
        # focus loss, and free-camera mode can never leave residual velocity.
        self._speed_mps = 0.0
        self._gait_active = False
        # ``facing`` remains an active orientation target even in SONIC IDLE.
        # If the rate-limited command is ahead of the physical body, preserving
        # it here would let the robot keep turning after focus loss, EOF, or a
        # deadman timeout.  Absorb fresh runtime feedback so a safety stop holds
        # the body's current heading instead of completing a stale turn.
        if self._measured_heading_rad is not None:
            self._command_heading_rad = self._measured_heading_rad
        facing = (
            math.cos(self._command_heading_rad),
            math.sin(self._command_heading_rad),
            0.0,
        )
        return RobotMotionCommand(
            sequence=self._last_sequence,
            movement=(0.0, 0.0, 0.0),
            facing=facing,
            speed_mps=0.0,
            mode="deadman" if deadman else "free_camera",
            safe_stop=True,
            reason=reason,
        )

    def _safety_reason(self, now_s: float) -> tuple[str, bool] | None:
        if self._snapshot is None or self._last_received_at_s is None:
            return (self._invalid_reason or "no_input", True)
        if now_s < self._last_received_at_s:
            self._requires_neutral_rearm = True
            return ("clock_regression", True)
        if (
            now_s - self._last_received_at_s >= self.config.input_timeout_s
            or now_s - self._snapshot.timestamp_monotonic_s
            >= self.config.input_timeout_s
        ):
            self._requires_neutral_rearm = True
            return ("input_timeout", True)
        if not self._snapshot.focused:
            self._requires_neutral_rearm = True
            return ("focus_lost", True)
        if self._free_camera:
            return ("free_camera", False)
        if self._requires_neutral_rearm:
            return ("awaiting_neutral", True)
        return None

    def _local_movement(self) -> tuple[float, float]:
        assert self._snapshot is not None
        keys = self._snapshot.keys
        right = float(keys.d) - float(keys.a)
        forward = float(keys.w) - float(keys.s)
        magnitude = math.hypot(right, forward)
        if any((keys.w, keys.a, keys.s, keys.d)):
            # Digital input wins while held.  Normalize diagonals so W+D is no
            # faster than W. Opposing keys intentionally resolve to neutral
            # instead of exposing a simultaneously held analog stick. Q/E are
            # deliberately absent from this equation.
            if magnitude <= 1e-12:
                return (0.0, 0.0)
            scale = 1.0 / max(1.0, magnitude)
            return (right * scale, forward * scale)
        return apply_radial_deadzone(
            right=self._snapshot.move_stick.right,
            forward=self._snapshot.move_stick.forward,
            deadzone=self.config.stick_deadzone,
        )

    def _requested_speed(self, input_magnitude: float) -> float:
        """Map digital tiers or analog stick travel onto native SLOW_WALK.

        Keyboard movement follows the usual third-person convention: Ctrl is
        held for a precise slow walk, unmodified WASD is ordinary walking, and
        Shift is held to run.  All three remain speed targets inside SONIC's
        native SLOW_WALK manifold.  Ctrl wins a Ctrl+Shift conflict so an
        accidental overlap can only reduce speed.  Gamepad magnitude remains
        continuous and is never quantized into these keyboard tiers.
        """

        assert self._snapshot is not None
        keys = self._snapshot.keys
        digital_movement = any((keys.w, keys.a, keys.s, keys.d))
        if digital_movement:
            if keys.ctrl:
                return self.config.min_gait_speed_mps
            if keys.shift:
                return self.config.max_speed_mps
            return (
                self.config.min_gait_speed_mps + self.config.max_speed_mps
            ) / 2.0
        # Treat the deadzone-remapped stick magnitude like a native analog
        # gait command: the first non-zero intent starts at SONIC's minimum
        # feasible gait, then the rest of the stick travel spans the full
        # remaining speed range.
        return self.config.min_gait_speed_mps + (
            self.config.max_speed_mps - self.config.min_gait_speed_mps
        ) * input_magnitude

    def command(self, *, now_s: float, dt_s: float) -> RobotMotionCommand:
        """Advance smoothing by ``dt_s`` and return the current robot command."""

        try:
            now = _finite_number(now_s, name="now_s", nonnegative=True)
            dt = _finite_number(dt_s, name="dt_s", nonnegative=True)
        except InputProtocolError as exc:
            raise ValueError(str(exc)) from exc
        dt = min(dt, self.config.max_step_s)

        safety = self._safety_reason(now)
        if safety is not None:
            reason, deadman = safety
            return self._safe_stop(reason=reason, deadman=deadman)

        assert self._snapshot is not None
        local_right, local_forward = self._local_movement()
        input_magnitude = min(1.0, math.hypot(local_right, local_forward))
        alignment = 0.0
        requested_speed = 0.0

        if input_magnitude > 1e-12:
            world_x, world_y = camera_relative_to_world(
                right=local_right,
                forward=local_forward,
                camera_yaw_rad=self._snapshot.camera_yaw_rad,
            )
            desired_heading = math.atan2(world_y, world_x)
            heading_error = wrap_angle_rad(
                desired_heading - self._command_heading_rad
            )
            # At the antipode, tiny camera-yaw noise can represent the same
            # direction as either +pi or -pi.  Latch the prior turn side so an
            # exact reversal never chatters between left and right.
            if abs(abs(heading_error) - math.pi) <= 0.05:
                heading_error = self._turn_sign * abs(heading_error)
            elif abs(heading_error) > 1e-6:
                self._turn_sign = math.copysign(1.0, heading_error)
            max_heading_delta = self.config.max_turn_rate_rad_s * dt
            heading_delta = max(-max_heading_delta, min(max_heading_delta, heading_error))
            self._command_heading_rad = wrap_angle_rad(
                self._command_heading_rad + heading_delta
            )

            # Reduce translation while making a large turn.  This produces a
            # natural turn-in-place for reversals rather than walking sideways.
            # Native SONIC replans IDLE when facing changes, so a zero target
            # speed still preserves this rate-limited orientation command.  If
            # physics feedback is available, translation waits for the body --
            # not merely the command target -- to align with the request.
            command_error = wrap_angle_rad(
                desired_heading - self._command_heading_rad
            )
            command_alignment = max(0.0, math.cos(command_error))
            if self._measured_heading_rad is None:
                alignment = command_alignment
            else:
                measured_error = wrap_angle_rad(
                    desired_heading - self._measured_heading_rad
                )
                measured_alignment = max(0.0, math.cos(measured_error))
                # A mid-turn retarget can make the body already face the new
                # request while the rate-limited planner target still points in
                # the old direction (or vice versa).  Translation must wait for
                # both frames; otherwise movement/facing would be published in
                # a direction that the alignment gate did not actually check.
                alignment = min(command_alignment, measured_alignment)
            # Digital WASD uses Ctrl/walk/Shift speed tiers.  Analog stick
            # travel stays continuous; ``max(minimum, maximum * magnitude)``
            # would flatten roughly the first third of a 0.30 m/s stick into a
            # single 0.10 m/s speed and make gentle control feel digital.
            requested_speed = self._requested_speed(input_magnitude)
            target_speed = requested_speed * alignment
        else:
            target_speed = 0.0

        rate = (
            self.config.max_acceleration_mps2
            if target_speed > self._speed_mps
            else self.config.max_deceleration_mps2
        )
        self._speed_mps = _move_toward(self._speed_mps, target_speed, rate * dt)
        if self._speed_mps < self.config.speed_epsilon_mps:
            self._speed_mps = 0.0

        # SONIC's native SLOW_WALK manifold starts at 0.10 m/s.  Keep distinct
        # start/stop thresholds around that floor so measured-heading noise
        # cannot switch native motion modes every control frame.  A deliberate
        # input release still follows the internal deceleration ramp to zero.
        if (
            input_magnitude > 1e-12
            and self._gait_active
            and (
                target_speed + self.config.speed_epsilon_mps
                < self.config.gait_stop_speed_mps
                or alignment
                < math.cos(self.config.gait_stop_heading_error_rad)
            )
        ):
            # Keep turning in native IDLE until physical alignment can support
            # the minimum gait; never hold a 0.10 m/s floor in a wrong heading.
            self._gait_active = False
        elif (
            input_magnitude > 1e-12
            and not self._gait_active
            and requested_speed + self.config.speed_epsilon_mps
            >= self.config.gait_start_speed_mps
            and alignment + self.config.speed_epsilon_mps
            >= math.cos(self.config.gait_start_heading_error_rad)
            and self._speed_mps + self.config.speed_epsilon_mps
            >= (
                self.config.min_gait_speed_mps
                * math.cos(self.config.gait_start_heading_error_rad)
            )
        ):
            self._gait_active = True
            # The native manifold cannot publish a speed below its gait floor.
            # Snap the hidden ramp back to that exact floor on entry so the
            # first non-zero frame is 0.10 m/s, then subsequent frames obey the
            # configured acceleration from that boundary.
            self._speed_mps = self.config.gait_start_speed_mps
        if self._speed_mps == 0.0:
            self._gait_active = False
        output_speed = (
            max(self.config.min_gait_speed_mps, self._speed_mps)
            if self._gait_active
            else 0.0
        )

        direction = (
            math.cos(self._command_heading_rad),
            math.sin(self._command_heading_rad),
            0.0,
        )
        moving = output_speed > 0.0
        return RobotMotionCommand(
            sequence=self._last_sequence,
            movement=direction if moving else (0.0, 0.0, 0.0),
            facing=direction,
            speed_mps=output_speed,
            mode="move" if moving else "idle",
            safe_stop=False,
            reason=None,
        )


@dataclass(frozen=True)
class PeerCredentials:
    pid: int
    uid: int
    gid: int


class UnixInputConnection:
    """An authenticated, reliable stream of message-bounded snapshots."""

    def __init__(
        self,
        connection: socket.socket,
        credentials: PeerCredentials,
        *,
        max_packet_bytes: int,
    ) -> None:
        self._connection = connection
        self.credentials = credentials
        self._max_packet_bytes = max_packet_bytes

    def receive(self, *, timeout_s: float | None = None) -> InputSnapshot:
        self._connection.settimeout(timeout_s)
        payload = self._connection.recv(self._max_packet_bytes + 1)
        if not payload:
            raise EOFError("input peer closed the socket")
        if len(payload) > self._max_packet_bytes:
            raise InputProtocolError(
                f"input packet exceeds {self._max_packet_bytes} byte limit"
            )
        return decode_input_packet(payload)

    def close(self) -> None:
        self._connection.close()

    def __enter__(self) -> "UnixInputConnection":
        return self

    def __exit__(self, *_args: Any) -> None:
        self.close()


class UnixSeqpacketInputServer:
    """Linux local-only input receiver with filesystem and UID allowlists."""

    def __init__(
        self,
        path: str | os.PathLike[str],
        *,
        allowed_uids: set[int] | frozenset[int] | None = None,
        max_packet_bytes: int = MAX_PACKET_BYTES,
        backlog: int = 1,
    ) -> None:
        self.path = Path(path)
        self.allowed_uids = frozenset(
            {os.getuid()} if allowed_uids is None else allowed_uids
        )
        if not self.allowed_uids or any(
            isinstance(uid, bool) or not isinstance(uid, int) or uid < 0
            for uid in self.allowed_uids
        ):
            raise ValueError("allowed_uids must contain nonnegative integer UIDs")
        if not isinstance(max_packet_bytes, int) or not 1 <= max_packet_bytes <= 65536:
            raise ValueError("max_packet_bytes must be in [1, 65536]")
        if not isinstance(backlog, int) or backlog < 1:
            raise ValueError("backlog must be a positive integer")
        self.max_packet_bytes = max_packet_bytes
        self.backlog = backlog
        self._socket: socket.socket | None = None
        self._socket_identity: tuple[int, int] | None = None
        self._accept_timeout: float | None | object = _UNSET_SOCKET_TIMEOUT

    def open(self) -> None:
        if self._socket is not None:
            raise RuntimeError("input server is already open")
        if self.path.exists() or self.path.is_symlink():
            raise FileExistsError(f"refusing to replace existing socket path: {self.path}")
        if not self.path.parent.is_dir():
            raise FileNotFoundError(f"socket parent directory does not exist: {self.path.parent}")
        socket_type = getattr(socket, "SOCK_SEQPACKET", None)
        if socket_type is None:
            raise RuntimeError("SOCK_SEQPACKET is unavailable on this platform")

        server = socket.socket(socket.AF_UNIX, socket_type)
        owned_identity: tuple[int, int] | None = None
        try:
            server.bind(os.fspath(self.path))
            socket_stat = self.path.stat()
            owned_identity = (socket_stat.st_dev, socket_stat.st_ino)
            os.chmod(self.path, 0o600)
            server.listen(self.backlog)
        except Exception:
            server.close()
            try:
                socket_stat = self.path.stat()
                current_identity = (socket_stat.st_dev, socket_stat.st_ino)
                if (
                    owned_identity is not None
                    and stat.S_ISSOCK(socket_stat.st_mode)
                    and current_identity == owned_identity
                ):
                    self.path.unlink()
            except (FileNotFoundError, OSError):
                pass
            raise
        self._socket = server
        self._socket_identity = owned_identity
        self._accept_timeout = _UNSET_SOCKET_TIMEOUT

    @staticmethod
    def _peer_credentials(connection: socket.socket) -> PeerCredentials:
        if not hasattr(socket, "SO_PEERCRED"):
            raise RuntimeError("SO_PEERCRED is unavailable on this platform")
        raw = connection.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12)
        pid, uid, gid = struct.unpack("3i", raw)
        return PeerCredentials(pid=pid, uid=uid, gid=gid)

    def accept(self, *, timeout_s: float | None = None) -> UnixInputConnection:
        if self._socket is None:
            raise RuntimeError("input server is not open")
        if (
            self._accept_timeout is _UNSET_SOCKET_TIMEOUT
            or timeout_s != self._accept_timeout
        ):
            self._socket.settimeout(timeout_s)
            self._accept_timeout = timeout_s
        connection, _address = self._socket.accept()
        try:
            credentials = self._peer_credentials(connection)
            if credentials.uid not in self.allowed_uids:
                raise PermissionError(
                    f"input peer UID {credentials.uid} is not allowed"
                )
        except Exception:
            connection.close()
            raise
        return UnixInputConnection(
            connection,
            credentials,
            max_packet_bytes=self.max_packet_bytes,
        )

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close()
            self._socket = None
        try:
            socket_stat = self.path.stat()
            identity = (socket_stat.st_dev, socket_stat.st_ino)
            if stat.S_ISSOCK(socket_stat.st_mode) and identity == self._socket_identity:
                self.path.unlink()
        except FileNotFoundError:
            pass
        finally:
            self._socket_identity = None
            self._accept_timeout = _UNSET_SOCKET_TIMEOUT

    def __enter__(self) -> "UnixSeqpacketInputServer":
        self.open()
        return self

    def __exit__(self, *_args: Any) -> None:
        self.close()
