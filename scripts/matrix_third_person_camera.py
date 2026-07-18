#!/usr/bin/env python3
"""Reference state machine and geometry for a future UE orbit camera bridge.

This module deliberately does not claim to move the camera in the Matrix 0.1.2
cooked build.  That build exposes neither project sources nor a verified
runtime camera endpoint.  The classes below define the fail-closed capability
gate and the deterministic orbit/collision behaviour that a cooked UE plugin
must implement and report before ``orbit-follow`` can be selected.

The reference frame is right-handed Z-up.  At yaw/pitch zero the camera is on
the negative X side of the pivot and looks toward positive X.  Positive yaw
orbits counter-clockwise in XY; positive pitch raises the camera.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Callable, Mapping


CAMERA_BRIDGE_PROTOCOL = "matrix-third-person-camera/v1"
DEFAULT_CAMERA_FRAME_MAX_AGE_NS = 100_000_000
DEFAULT_CAMERA_REQUEST_TIMEOUT_NS = 500_000_000
CAMERA_GEOMETRY_TOLERANCE_M = 0.005
CAMERA_HANDOFF_PIVOT_TOLERANCE_M = 0.25
CAMERA_HANDOFF_ARM_TOLERANCE_M = 0.02
CAMERA_HANDOFF_DIRECTION_TOLERANCE_RAD = math.radians(1.0)
CAMERA_RELATIVE_LOCK_TOLERANCE_M = 0.02
MODE_NATIVE_FOLLOW = "native-follow"
MODE_NATIVE_FREE = "native-free"
MODE_RELATIVE_LOCK = "relative-lock"
MODE_ORBIT_FOLLOW = "orbit-follow"
CAMERA_MODES = frozenset(
    (
        MODE_NATIVE_FOLLOW,
        MODE_NATIVE_FREE,
        MODE_RELATIVE_LOCK,
        MODE_ORBIT_FOLLOW,
    )
)
FOLLOW_MODES = frozenset(
    (MODE_NATIVE_FOLLOW, MODE_RELATIVE_LOCK, MODE_ORBIT_FOLLOW)
)


class CameraModeUnavailable(RuntimeError):
    """The requested visible-camera mode lacks authoritative UE capability."""


class CameraCollisionUnavailable(RuntimeError):
    """A trustworthy, non-penetrating camera collision result is unavailable."""


def _finite(value: object, *, name: str, nonnegative: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    if nonnegative and result < 0.0:
        raise ValueError(f"{name} must be nonnegative")
    return result


def wrap_angle_rad(value: float) -> float:
    value = _finite(value, name="angle")
    return math.atan2(math.sin(value), math.cos(value))


@dataclass(frozen=True)
class Vec3:
    x: float
    y: float
    z: float

    def __post_init__(self) -> None:
        for name in ("x", "y", "z"):
            object.__setattr__(
                self,
                name,
                _finite(getattr(self, name), name=f"vector.{name}"),
            )

    def __add__(self, other: "Vec3") -> "Vec3":
        if not isinstance(other, Vec3):
            return NotImplemented
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "Vec3") -> "Vec3":
        if not isinstance(other, Vec3):
            return NotImplemented
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def scaled(self, scale: float) -> "Vec3":
        scale = _finite(scale, name="scale")
        return Vec3(self.x * scale, self.y * scale, self.z * scale)

    @property
    def length(self) -> float:
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    @classmethod
    def from_sequence(cls, value: object, *, name: str) -> "Vec3":
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            raise ValueError(f"{name} must be a three-number array")
        return cls(*value)

    def as_list(self) -> list[float]:
        return [self.x, self.y, self.z]


def _direction_angle_rad(left: Vec3, right: Vec3) -> float:
    left_length = left.length
    right_length = right.length
    if left_length <= 1e-9 or right_length <= 1e-9:
        raise CameraModeUnavailable("camera handoff direction is undefined")
    dot = (
        left.x * right.x + left.y * right.y + left.z * right.z
    ) / (left_length * right_length)
    return math.acos(max(-1.0, min(1.0, dot)))


def _normalized(vector: Vec3, *, name: str) -> Vec3:
    length = vector.length
    if length <= 1e-9:
        raise CameraModeUnavailable(f"{name} direction is undefined")
    return vector.scaled(1.0 / length)


def _cross(left: Vec3, right: Vec3) -> Vec3:
    return Vec3(
        left.y * right.z - left.z * right.y,
        left.z * right.x - left.x * right.z,
        left.x * right.y - left.y * right.x,
    )


def _slerp_direction(start: Vec3, end: Vec3, progress: float) -> Vec3:
    amount = _finite(progress, name="direction progress", nonnegative=True)
    if amount > 1.0:
        raise ValueError("direction progress must not exceed one")
    start_unit = _normalized(start, name="source view")
    end_unit = _normalized(end, name="target view")
    if amount <= 0.0:
        return start_unit
    if amount >= 1.0:
        return end_unit
    dot = max(
        -1.0,
        min(
            1.0,
            start_unit.x * end_unit.x
            + start_unit.y * end_unit.y
            + start_unit.z * end_unit.z,
        ),
    )
    if dot > 0.9995:
        blended = start_unit.scaled(1.0 - amount) + end_unit.scaled(amount)
        return _normalized(blended, name="interpolated view")
    if dot < -0.9995:
        basis = (
            Vec3(0.0, 0.0, 1.0)
            if abs(start_unit.z) < 0.9
            else Vec3(0.0, 1.0, 0.0)
        )
        orthogonal = _normalized(
            _cross(start_unit, basis), name="antipodal view"
        )
        angle = math.pi * amount
        return start_unit.scaled(math.cos(angle)) + orthogonal.scaled(
            math.sin(angle)
        )
    angle = math.acos(dot)
    denominator = math.sin(angle)
    return start_unit.scaled(math.sin((1.0 - amount) * angle) / denominator) + (
        end_unit.scaled(math.sin(amount * angle) / denominator)
    )


@dataclass(frozen=True)
class CameraBridgeCapabilities:
    """Authoritative capabilities reported by the cooked UE camera bridge."""

    protocol: str = CAMERA_BRIDGE_PROTOCOL
    authoritative_robot_pivot: bool = False
    final_view_readback: bool = False
    orbit_control: bool = False
    sphere_sweep: bool = False
    input_mode_readback: bool = False
    relative_pose_handoff: bool = False
    relative_lock_control: bool = False

    def __post_init__(self) -> None:
        if self.protocol != CAMERA_BRIDGE_PROTOCOL:
            raise ValueError(
                f"camera bridge protocol must be {CAMERA_BRIDGE_PROTOCOL!r}"
            )
        for name in (
            "authoritative_robot_pivot",
            "final_view_readback",
            "orbit_control",
            "sphere_sweep",
            "input_mode_readback",
            "relative_pose_handoff",
            "relative_lock_control",
        ):
            if type(getattr(self, name)) is not bool:
                raise ValueError(f"camera capability {name} must be boolean")

    @classmethod
    def from_mapping(cls, value: object) -> "CameraBridgeCapabilities":
        if not isinstance(value, Mapping) or not all(
            isinstance(key, str) for key in value
        ):
            raise ValueError("camera capabilities must be an object")
        expected = frozenset(
            (
                "protocol",
                "authoritative_robot_pivot",
                "final_view_readback",
                "orbit_control",
                "sphere_sweep",
                "input_mode_readback",
                "relative_pose_handoff",
                "relative_lock_control",
            )
        )
        actual = frozenset(value)
        if actual != expected:
            missing = sorted(expected - actual)
            unknown = sorted(actual - expected)
            detail = []
            if missing:
                detail.append(f"missing={missing}")
            if unknown:
                detail.append(f"unknown={unknown}")
            raise ValueError("invalid camera capabilities: " + " ".join(detail))
        return cls(**{name: value[name] for name in expected})

    @property
    def orbit_ready(self) -> bool:
        return bool(
            self.authoritative_robot_pivot
            and self.final_view_readback
            and self.orbit_control
            and self.sphere_sweep
            and self.input_mode_readback
            and self.relative_pose_handoff
        )

    def require_orbit(self) -> None:
        if self.orbit_ready:
            return
        missing = [
            name
            for name in (
                "authoritative_robot_pivot",
                "final_view_readback",
                "orbit_control",
                "sphere_sweep",
                "input_mode_readback",
                "relative_pose_handoff",
            )
            if not getattr(self, name)
        ]
        raise CameraModeUnavailable(
            "orbit-follow requires authoritative cooked-UE capabilities: "
            + ", ".join(missing)
        )

    @property
    def relative_lock_ready(self) -> bool:
        return bool(
            self.authoritative_robot_pivot
            and self.final_view_readback
            and self.input_mode_readback
            and self.relative_pose_handoff
            and self.relative_lock_control
        )

    def require_relative_lock(self) -> None:
        if self.relative_lock_ready:
            return
        missing = [
            name
            for name in (
                "authoritative_robot_pivot",
                "final_view_readback",
                "input_mode_readback",
                "relative_pose_handoff",
                "relative_lock_control",
            )
            if not getattr(self, name)
        ]
        raise CameraModeUnavailable(
            "relative-lock requires authoritative cooked-UE capabilities: "
            + ", ".join(missing)
        )


def _nonnegative_integer(value: object, *, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a nonnegative integer")
    return value


def _bridge_identifier(value: object, *, name: str) -> str:
    if not isinstance(value, str) or not 1 <= len(value) <= 64:
        raise ValueError(f"{name} must be a 1..64 character string")
    if not all(character.isalnum() or character in "-_." for character in value):
        raise ValueError(f"{name} contains unsupported characters")
    return value


@dataclass(frozen=True)
class CameraBridgeFrame:
    """Same-render-frame view and pivot read back from the future UE bridge.

    Non-orbit modes have no bridge-owned arm result and therefore report null
    for the three collision fields.  ``orbit-follow`` must report the actual view
    *after* PlayerCameraManager and collision resolution, not a requested pose.
    Protocol v1 defines ``look_at`` in the world-Z-up, zero-roll camera frame;
    a runtime with camera roll or lens shift needs a newer protocol schema.
    """

    session_id: str
    sequence: int
    produced_monotonic_ns: int
    applied_request_id: int | None
    render_frame_id: int
    mode: str
    robot_pivot: Vec3
    camera_position: Vec3
    look_at: Vec3
    input_captured: bool
    desired_arm_m: float | None = None
    actual_arm_m: float | None = None
    collision_limited: bool | None = None

    def __post_init__(self) -> None:
        _bridge_identifier(self.session_id, name="camera session_id")
        _nonnegative_integer(self.sequence, name="camera frame sequence")
        _nonnegative_integer(
            self.produced_monotonic_ns, name="produced_monotonic_ns"
        )
        _nonnegative_integer(self.render_frame_id, name="render_frame_id")
        if self.applied_request_id is not None:
            request_id = _nonnegative_integer(
                self.applied_request_id, name="applied_request_id"
            )
            if request_id <= 0:
                raise ValueError("applied_request_id must be positive")
        if self.mode not in CAMERA_MODES:
            raise ValueError(f"unsupported camera mode: {self.mode}")
        for name in ("robot_pivot", "camera_position", "look_at"):
            if not isinstance(getattr(self, name), Vec3):
                raise ValueError(f"camera frame {name} must be Vec3")
        if type(self.input_captured) is not bool:
            raise ValueError("camera frame input_captured must be boolean")

        arm_values = (
            self.desired_arm_m,
            self.actual_arm_m,
            self.collision_limited,
        )
        if self.mode != MODE_ORBIT_FOLLOW:
            if arm_values != (None, None, None):
                raise ValueError(
                    "native camera frames must not claim bridge collision state"
                )
            return
        if self.desired_arm_m is None or self.actual_arm_m is None:
            raise ValueError("orbit-follow frame requires desired and actual arm")
        desired = _finite(
            self.desired_arm_m, name="desired_arm_m", nonnegative=True
        )
        actual = _finite(self.actual_arm_m, name="actual_arm_m", nonnegative=True)
        if desired <= 0.0 or actual <= 0.0 or actual > desired + 1e-9:
            raise ValueError("orbit-follow frame has invalid arm lengths")
        if type(self.collision_limited) is not bool:
            raise ValueError("orbit-follow frame collision_limited must be boolean")
        if self.collision_limited != (actual < desired - 1e-9):
            raise ValueError("orbit-follow collision flag disagrees with arm lengths")
        measured_arm = (self.camera_position - self.robot_pivot).length
        if abs(measured_arm - actual) > CAMERA_GEOMETRY_TOLERANCE_M:
            raise ValueError(
                "orbit-follow camera distance disagrees with actual_arm_m"
            )
        look_at_error = (self.look_at - self.robot_pivot).length
        if look_at_error > CAMERA_GEOMETRY_TOLERANCE_M:
            raise ValueError("orbit-follow look_at must equal the robot pivot")
        object.__setattr__(self, "desired_arm_m", desired)
        object.__setattr__(self, "actual_arm_m", actual)

    def require_fresh(
        self,
        *,
        now_monotonic_ns: int,
        max_age_ns: int = DEFAULT_CAMERA_FRAME_MAX_AGE_NS,
    ) -> None:
        now = _nonnegative_integer(now_monotonic_ns, name="now_monotonic_ns")
        max_age = _nonnegative_integer(max_age_ns, name="max_age_ns")
        age = now - self.produced_monotonic_ns
        if age < 0:
            raise CameraModeUnavailable("camera bridge frame is from the future")
        if age > max_age:
            raise CameraModeUnavailable("camera bridge frame is stale")

    @classmethod
    def from_mapping(cls, value: object) -> "CameraBridgeFrame":
        if not isinstance(value, Mapping) or not all(
            isinstance(key, str) for key in value
        ):
            raise ValueError("camera bridge frame must be an object")
        expected = frozenset(
            (
                "protocol",
                "session_id",
                "sequence",
                "produced_monotonic_ns",
                "applied_request_id",
                "render_frame_id",
                "mode",
                "robot_pivot_m",
                "camera_position_m",
                "look_at_m",
                "input_captured",
                "desired_arm_m",
                "actual_arm_m",
                "collision_limited",
            )
        )
        actual = frozenset(value)
        if actual != expected:
            missing = sorted(expected - actual)
            unknown = sorted(actual - expected)
            detail = []
            if missing:
                detail.append(f"missing={missing}")
            if unknown:
                detail.append(f"unknown={unknown}")
            raise ValueError("invalid camera bridge frame: " + " ".join(detail))
        if value["protocol"] != CAMERA_BRIDGE_PROTOCOL:
            raise ValueError(
                f"camera bridge protocol must be {CAMERA_BRIDGE_PROTOCOL!r}"
            )
        return cls(
            session_id=value["session_id"],
            sequence=value["sequence"],
            produced_monotonic_ns=value["produced_monotonic_ns"],
            applied_request_id=value["applied_request_id"],
            render_frame_id=value["render_frame_id"],
            mode=value["mode"],
            robot_pivot=Vec3.from_sequence(
                value["robot_pivot_m"], name="robot_pivot_m"
            ),
            camera_position=Vec3.from_sequence(
                value["camera_position_m"], name="camera_position_m"
            ),
            look_at=Vec3.from_sequence(value["look_at_m"], name="look_at_m"),
            input_captured=value["input_captured"],
            desired_arm_m=value["desired_arm_m"],
            actual_arm_m=value["actual_arm_m"],
            collision_limited=value["collision_limited"],
        )


@dataclass(frozen=True)
class RelativeCameraPose:
    """A final native-camera source view expressed relative to robot pivot.

    Translating both offsets by a later authoritative pivot implements the
    requested free -> locked handoff without leaving an invisible world-space
    anchor behind.  Orbit follow uses the camera offset as its yaw/pitch/arm
    seed but deliberately changes ``look_at`` to the robot pivot.
    """

    source_sequence: int
    source_produced_monotonic_ns: int
    source_render_frame_id: int
    source_pivot: Vec3
    camera_offset: Vec3
    look_at_offset: Vec3

    def __post_init__(self) -> None:
        _nonnegative_integer(self.source_sequence, name="source_sequence")
        _nonnegative_integer(
            self.source_produced_monotonic_ns,
            name="source_produced_monotonic_ns",
        )
        _nonnegative_integer(
            self.source_render_frame_id, name="source_render_frame_id"
        )
        for name in ("source_pivot", "camera_offset", "look_at_offset"):
            if not isinstance(getattr(self, name), Vec3):
                raise ValueError(f"relative camera {name} must be Vec3")
        if self.camera_offset.length <= 1e-9:
            raise CameraModeUnavailable(
                "free-camera view is coincident with the robot pivot"
            )
        if (self.look_at_offset - self.camera_offset).length <= 1e-9:
            raise CameraModeUnavailable("free-camera view direction is undefined")

    @classmethod
    def capture(cls, frame: CameraBridgeFrame) -> "RelativeCameraPose":
        if not isinstance(frame, CameraBridgeFrame):
            raise ValueError("free-camera handoff requires CameraBridgeFrame")
        if frame.mode != MODE_NATIVE_FREE:
            raise CameraModeUnavailable(
                "relative pose may be captured only from native-free readback"
            )
        return cls._from_frame(frame)

    @classmethod
    def capture_transition_source(
        cls, frame: CameraBridgeFrame
    ) -> "RelativeCameraPose":
        if not isinstance(frame, CameraBridgeFrame):
            raise ValueError("camera handoff requires CameraBridgeFrame")
        return cls._from_frame(frame)

    @classmethod
    def _from_frame(cls, frame: CameraBridgeFrame) -> "RelativeCameraPose":
        return cls(
            source_sequence=frame.sequence,
            source_produced_monotonic_ns=frame.produced_monotonic_ns,
            source_render_frame_id=frame.render_frame_id,
            source_pivot=frame.robot_pivot,
            camera_offset=frame.camera_position - frame.robot_pivot,
            look_at_offset=frame.look_at - frame.robot_pivot,
        )

    @property
    def arm_m(self) -> float:
        return self.camera_offset.length

    @property
    def yaw_rad(self) -> float:
        return wrap_angle_rad(
            math.atan2(-self.camera_offset.y, -self.camera_offset.x)
        )

    @property
    def pitch_rad(self) -> float:
        horizontal = math.hypot(self.camera_offset.x, self.camera_offset.y)
        return math.atan2(self.camera_offset.z, horizontal)

    def translated_view(self, robot_pivot: Vec3) -> tuple[Vec3, Vec3]:
        """Return camera/look-at translated by exactly the pivot displacement."""

        if not isinstance(robot_pivot, Vec3):
            raise ValueError("robot_pivot must be Vec3")
        return (
            robot_pivot + self.camera_offset,
            robot_pivot + self.look_at_offset,
        )


class _CameraRequestLease:
    def __init__(self) -> None:
        self.status = "pending"


@dataclass(frozen=True)
class CameraModeRequest:
    """A correlated request awaiting a newer authoritative UE frame."""

    session_id: str
    request_id: int
    requested_monotonic_ns: int
    source_frame: CameraBridgeFrame
    target_mode: str
    relative_pose: RelativeCameraPose | None = None
    _provenance: object = field(
        default_factory=object, repr=False, compare=False
    )
    _lease: _CameraRequestLease = field(
        default_factory=_CameraRequestLease, repr=False, compare=False
    )

    def __post_init__(self) -> None:
        _bridge_identifier(self.session_id, name="camera request session_id")
        request_id = _nonnegative_integer(
            self.request_id, name="camera request_id"
        )
        if request_id <= 0:
            raise ValueError("camera request_id must be positive")
        requested = _nonnegative_integer(
            self.requested_monotonic_ns, name="requested_monotonic_ns"
        )
        if not isinstance(self.source_frame, CameraBridgeFrame):
            raise ValueError("camera request source must be CameraBridgeFrame")
        if self.source_frame.session_id != self.session_id:
            raise ValueError("camera request/source session mismatch")
        if requested < self.source_frame.produced_monotonic_ns:
            raise ValueError("camera request predates its source frame")
        if self.target_mode not in CAMERA_MODES:
            raise ValueError(f"unsupported camera mode: {self.target_mode}")
        if self.target_mode == self.source_frame.mode:
            raise ValueError("camera request target must differ from source mode")
        if self.target_mode in (MODE_RELATIVE_LOCK, MODE_ORBIT_FOLLOW):
            if not isinstance(self.relative_pose, RelativeCameraPose):
                raise ValueError(
                    "locked camera request requires a relative source pose"
                )
            if (
                self.relative_pose.source_sequence
                != self.source_frame.sequence
                or self.relative_pose.source_render_frame_id
                != self.source_frame.render_frame_id
                or self.relative_pose.source_produced_monotonic_ns
                != self.source_frame.produced_monotonic_ns
            ):
                raise ValueError("locked request pose/source frame mismatch")
            expected_camera_offset = (
                self.source_frame.camera_position - self.source_frame.robot_pivot
            )
            expected_look_at_offset = (
                self.source_frame.look_at - self.source_frame.robot_pivot
            )
            if (
                self.relative_pose.source_pivot != self.source_frame.robot_pivot
                or self.relative_pose.camera_offset != expected_camera_offset
                or self.relative_pose.look_at_offset != expected_look_at_offset
            ):
                raise ValueError("locked request pose geometry/source mismatch")
        elif self.relative_pose is not None:
            raise ValueError("native camera request must not carry orbit pose")

    @property
    def active(self) -> bool:
        return self._lease.status == "pending"

    @property
    def cancelled(self) -> bool:
        return self._lease.status == "cancelled"

    def _mark_confirmed(self) -> None:
        self._lease.status = "confirmed"

    def _cancel(self) -> None:
        self._lease.status = "cancelled"


@dataclass(frozen=True)
class RelativeLockState:
    """Executable relative-lock view for an authoritative robot pivot."""

    pivot: Vec3
    camera_position: Vec3
    look_at: Vec3
    _request_provenance: object = field(repr=False, compare=False)

    def __post_init__(self) -> None:
        for name in ("pivot", "camera_position", "look_at"):
            if not isinstance(getattr(self, name), Vec3):
                raise ValueError(f"relative-lock {name} must be Vec3")


class RelativeLockController:
    """Keep captured camera/look-at offsets while the robot pivot translates."""

    def __init__(self, request: CameraModeRequest) -> None:
        if not isinstance(request, CameraModeRequest):
            raise ValueError("relative-lock requires CameraModeRequest")
        if request.target_mode != MODE_RELATIVE_LOCK:
            raise CameraModeUnavailable(
                "camera request does not target relative-lock"
            )
        if not request.active:
            raise CameraModeUnavailable(
                "relative-lock request is no longer pending"
            )
        assert request.relative_pose is not None
        self.request = request
        self.relative_pose = request.relative_pose
        self._state: RelativeLockState | None = None

    @property
    def state(self) -> RelativeLockState | None:
        return self._state

    def step(self, *, robot_pivot: Vec3) -> RelativeLockState:
        if self.request.cancelled:
            raise CameraModeUnavailable("relative-lock request was cancelled")
        if not isinstance(robot_pivot, Vec3):
            raise ValueError("relative-lock robot_pivot must be Vec3")
        camera_position, look_at = self.relative_pose.translated_view(robot_pivot)
        state = RelativeLockState(
            pivot=robot_pivot,
            camera_position=camera_position,
            look_at=look_at,
            _request_provenance=self.request._provenance,
        )
        self._state = state
        return state


class CameraModeController:
    """Requested/confirmed camera-mode policy for C, V, and the ESC UI.

    ``mode`` is only the last UE-confirmed mode.  Every key/UI action creates a
    correlated request and leaves ``mode`` unchanged until a newer final render
    frame echoes the exact session/request pair.  No helper directly mutates a
    confirmed mode, including the initial native-follow -> orbit transition.
    """

    def __init__(self, *, session_id: str = "matrix-camera-reference") -> None:
        self.session_id = _bridge_identifier(
            session_id, name="camera controller session_id"
        )
        self.mode = MODE_NATIVE_FOLLOW
        self.last_follow_mode = MODE_NATIVE_FOLLOW
        self.pending_request: CameraModeRequest | None = None
        self.confirmed_frame: CameraBridgeFrame | None = None
        self.bridge_faulted = False
        self._next_request_id = 1
        self._last_consumed_sequence = -1
        self.transition_count = 0
        self.neutral_rearm_required = True

    @property
    def pending_mode(self) -> str | None:
        if self.pending_request is None:
            return None
        return self.pending_request.target_mode

    @property
    def pending_relative_pose(self) -> RelativeCameraPose | None:
        if self.pending_request is None:
            return None
        return self.pending_request.relative_pose

    def _validate_source_frame(
        self,
        *,
        source_frame: CameraBridgeFrame,
        now_monotonic_ns: int,
        max_frame_age_ns: int,
    ) -> None:
        if not isinstance(source_frame, CameraBridgeFrame):
            raise ValueError("camera transition source must be CameraBridgeFrame")
        source_frame.require_fresh(
            now_monotonic_ns=now_monotonic_ns,
            max_age_ns=max_frame_age_ns,
        )
        if source_frame.session_id != self.session_id:
            raise CameraModeUnavailable("camera transition source session mismatch")
        if source_frame.mode != self.mode:
            raise CameraModeUnavailable(
                "camera transition source disagrees with confirmed mode"
            )
        if self.confirmed_frame is None:
            if source_frame.applied_request_id is not None:
                raise CameraModeUnavailable(
                    "initial camera source unexpectedly claims a request"
                )
        elif (
            source_frame.applied_request_id
            != self.confirmed_frame.applied_request_id
        ):
            raise CameraModeUnavailable(
                "camera transition source request lineage mismatch"
            )
        if source_frame.input_captured:
            raise CameraModeUnavailable(
                "camera input must be released before a mode request"
            )
        if source_frame.sequence <= self._last_consumed_sequence:
            raise CameraModeUnavailable(
                "camera transition source sequence was already consumed"
            )

    def request_mode(
        self,
        target_mode: str,
        *,
        capabilities: CameraBridgeCapabilities,
        source_frame: CameraBridgeFrame,
        now_monotonic_ns: int,
        max_frame_age_ns: int = DEFAULT_CAMERA_FRAME_MAX_AGE_NS,
    ) -> bool:
        """Create a correlated request without changing confirmed ``mode``."""

        if target_mode not in CAMERA_MODES:
            raise ValueError(f"unsupported camera mode: {target_mode}")
        if self.pending_request is not None:
            return False
        if target_mode == self.mode:
            return False
        if target_mode == MODE_ORBIT_FOLLOW:
            capabilities.require_orbit()
        elif target_mode == MODE_RELATIVE_LOCK:
            capabilities.require_relative_lock()
        self._validate_source_frame(
            source_frame=source_frame,
            now_monotonic_ns=now_monotonic_ns,
            max_frame_age_ns=max_frame_age_ns,
        )
        relative_pose = None
        if target_mode in (MODE_RELATIVE_LOCK, MODE_ORBIT_FOLLOW):
            relative_pose = RelativeCameraPose.capture_transition_source(
                source_frame
            )
        request = CameraModeRequest(
            session_id=self.session_id,
            request_id=self._next_request_id,
            requested_monotonic_ns=now_monotonic_ns,
            source_frame=source_frame,
            target_mode=target_mode,
            relative_pose=relative_pose,
        )
        self._next_request_id += 1
        self._last_consumed_sequence = source_frame.sequence
        self.pending_request = request
        self.neutral_rearm_required = True
        return True

    def select_follow(
        self,
        mode: str,
        *,
        capabilities: CameraBridgeCapabilities,
        source_frame: CameraBridgeFrame,
        now_monotonic_ns: int,
        max_frame_age_ns: int = DEFAULT_CAMERA_FRAME_MAX_AGE_NS,
    ) -> bool:
        if mode not in FOLLOW_MODES:
            raise ValueError("select_follow accepts only a follow-camera mode")
        return self.request_mode(
            mode,
            capabilities=capabilities,
            source_frame=source_frame,
            now_monotonic_ns=now_monotonic_ns,
            max_frame_age_ns=max_frame_age_ns,
        )

    def on_orbit_toggle(
        self,
        *,
        capabilities: CameraBridgeCapabilities,
        source_frame: CameraBridgeFrame,
        now_monotonic_ns: int,
        max_frame_age_ns: int = DEFAULT_CAMERA_FRAME_MAX_AGE_NS,
    ) -> bool:
        """Request C toggle; C remains ignored in confirmed native-free mode."""

        if self.mode == MODE_NATIVE_FREE or self.pending_request is not None:
            return False
        target = (
            MODE_NATIVE_FOLLOW
            if self.mode == MODE_ORBIT_FOLLOW
            else MODE_ORBIT_FOLLOW
        )
        return self.select_follow(
            target,
            capabilities=capabilities,
            source_frame=source_frame,
            now_monotonic_ns=now_monotonic_ns,
            max_frame_age_ns=max_frame_age_ns,
        )

    def on_v_edge(
        self,
        *,
        capabilities: CameraBridgeCapabilities,
        source_frame: CameraBridgeFrame,
        now_monotonic_ns: int,
        max_frame_age_ns: int = DEFAULT_CAMERA_FRAME_MAX_AGE_NS,
    ) -> bool:
        """Request native-free <-> last-follow without assuming UE success."""

        if self.pending_request is not None:
            return False
        target = (
            self.last_follow_mode
            if self.mode == MODE_NATIVE_FREE
            else MODE_NATIVE_FREE
        )
        return self.request_mode(
            target,
            capabilities=capabilities,
            source_frame=source_frame,
            now_monotonic_ns=now_monotonic_ns,
            max_frame_age_ns=max_frame_age_ns,
        )

    def _validate_orbit_completion(
        self,
        request: CameraModeRequest,
        orbit_frame: CameraBridgeFrame,
    ) -> None:
        assert request.relative_pose is not None
        pose = request.relative_pose
        if (
            orbit_frame.robot_pivot - pose.source_pivot
        ).length > CAMERA_HANDOFF_PIVOT_TOLERANCE_M:
            raise CameraModeUnavailable(
                "orbit completion pivot is unrelated to the request source"
            )
        assert orbit_frame.desired_arm_m is not None
        if (
            abs(orbit_frame.desired_arm_m - pose.arm_m)
            > CAMERA_HANDOFF_ARM_TOLERANCE_M
        ):
            raise CameraModeUnavailable(
                "orbit completion desired arm disagrees with the request pose"
            )
        actual_direction = orbit_frame.camera_position - orbit_frame.robot_pivot
        direction_error = _direction_angle_rad(
            actual_direction, pose.camera_offset
        )
        if direction_error > CAMERA_HANDOFF_DIRECTION_TOLERANCE_RAD:
            raise CameraModeUnavailable(
                "orbit completion direction disagrees with the request pose"
            )

    def _validate_relative_lock_completion(
        self,
        request: CameraModeRequest,
        relative_frame: CameraBridgeFrame,
    ) -> None:
        assert request.relative_pose is not None
        pose = request.relative_pose
        camera_offset = relative_frame.camera_position - relative_frame.robot_pivot
        look_at_offset = relative_frame.look_at - relative_frame.robot_pivot
        if (
            camera_offset - pose.camera_offset
        ).length > CAMERA_RELATIVE_LOCK_TOLERANCE_M:
            raise CameraModeUnavailable(
                "relative-lock camera offset disagrees with the request pose"
            )
        if (
            look_at_offset - pose.look_at_offset
        ).length > CAMERA_RELATIVE_LOCK_TOLERANCE_M:
            raise CameraModeUnavailable(
                "relative-lock look-at offset disagrees with the request pose"
            )

    def complete_mode_transition(
        self,
        *,
        capabilities: CameraBridgeCapabilities,
        confirmed_frame: CameraBridgeFrame,
        now_monotonic_ns: int,
        max_frame_age_ns: int = DEFAULT_CAMERA_FRAME_MAX_AGE_NS,
        max_request_age_ns: int = DEFAULT_CAMERA_REQUEST_TIMEOUT_NS,
    ) -> bool:
        """Commit only the exact pending request's newer final render frame."""

        if self.expire_pending_request(
            now_monotonic_ns=now_monotonic_ns,
            max_request_age_ns=max_request_age_ns,
        ):
            raise CameraModeUnavailable("camera mode request timed out")
        request = self.pending_request
        if request is None:
            raise CameraModeUnavailable("no camera mode request is pending")
        if not isinstance(confirmed_frame, CameraBridgeFrame):
            raise ValueError("camera completion requires CameraBridgeFrame")
        confirmed_frame.require_fresh(
            now_monotonic_ns=now_monotonic_ns,
            max_age_ns=max_frame_age_ns,
        )
        if confirmed_frame.session_id != request.session_id:
            raise CameraModeUnavailable("camera completion session mismatch")
        if confirmed_frame.applied_request_id != request.request_id:
            raise CameraModeUnavailable("camera completion request id mismatch")
        if confirmed_frame.mode != request.target_mode:
            raise CameraModeUnavailable("camera completion mode mismatch")
        if confirmed_frame.input_captured:
            raise CameraModeUnavailable(
                "camera input must remain released during mode completion"
            )
        source = request.source_frame
        if confirmed_frame.sequence <= source.sequence:
            raise CameraModeUnavailable(
                "camera completion requires a newer frame sequence"
            )
        if confirmed_frame.render_frame_id <= source.render_frame_id:
            raise CameraModeUnavailable(
                "camera completion requires a newer render frame"
            )
        if confirmed_frame.produced_monotonic_ns <= source.produced_monotonic_ns:
            raise CameraModeUnavailable(
                "camera completion timestamp must follow its request source"
            )
        if confirmed_frame.produced_monotonic_ns <= request.requested_monotonic_ns:
            raise CameraModeUnavailable(
                "camera completion timestamp must follow its request"
            )
        if (
            confirmed_frame.robot_pivot - source.robot_pivot
        ).length > CAMERA_HANDOFF_PIVOT_TOLERANCE_M:
            raise CameraModeUnavailable(
                "camera completion pivot is unrelated to the request source"
            )
        if request.target_mode == MODE_ORBIT_FOLLOW:
            capabilities.require_orbit()
            self._validate_orbit_completion(request, confirmed_frame)
        elif request.target_mode == MODE_RELATIVE_LOCK:
            capabilities.require_relative_lock()
            self._validate_relative_lock_completion(request, confirmed_frame)

        self.mode = request.target_mode
        if self.mode in FOLLOW_MODES:
            self.last_follow_mode = self.mode
        self.confirmed_frame = confirmed_frame
        self._last_consumed_sequence = confirmed_frame.sequence
        request._mark_confirmed()
        self.pending_request = None
        self.bridge_faulted = False
        self.transition_count += 1
        self.neutral_rearm_required = True
        return True

    def complete_orbit_handoff(
        self,
        *,
        capabilities: CameraBridgeCapabilities,
        orbit_frame: CameraBridgeFrame,
        now_monotonic_ns: int,
        max_frame_age_ns: int = DEFAULT_CAMERA_FRAME_MAX_AGE_NS,
        max_request_age_ns: int = DEFAULT_CAMERA_REQUEST_TIMEOUT_NS,
    ) -> bool:
        if self.pending_mode != MODE_ORBIT_FOLLOW:
            raise CameraModeUnavailable("no orbit mode request is pending")
        return self.complete_mode_transition(
            capabilities=capabilities,
            confirmed_frame=orbit_frame,
            now_monotonic_ns=now_monotonic_ns,
            max_frame_age_ns=max_frame_age_ns,
            max_request_age_ns=max_request_age_ns,
        )

    def expire_pending_request(
        self,
        *,
        now_monotonic_ns: int,
        max_request_age_ns: int = DEFAULT_CAMERA_REQUEST_TIMEOUT_NS,
    ) -> bool:
        """Cancel an over-age request while leaving confirmed mode unchanged."""

        request = self.pending_request
        if request is None:
            return False
        now = _nonnegative_integer(now_monotonic_ns, name="now_monotonic_ns")
        max_age = _nonnegative_integer(
            max_request_age_ns, name="max_request_age_ns"
        )
        age = now - request.requested_monotonic_ns
        if age < 0:
            raise CameraModeUnavailable("request timeout clock moved backwards")
        if age <= max_age:
            return False
        request._cancel()
        self.pending_request = None
        self.neutral_rearm_required = True
        return True

    def reconcile_capabilities(
        self, capabilities: CameraBridgeCapabilities
    ) -> bool:
        """Disarm on loss without inventing an unconfirmed visual transition."""

        orbit_unavailable = not capabilities.orbit_ready
        relative_unavailable = not capabilities.relative_lock_ready
        changed = False
        if (
            self.last_follow_mode == MODE_ORBIT_FOLLOW
            and orbit_unavailable
        ) or (
            self.last_follow_mode == MODE_RELATIVE_LOCK
            and relative_unavailable
        ):
            self.last_follow_mode = MODE_NATIVE_FOLLOW
            changed = True
        pending_unavailable = (
            self.pending_mode == MODE_ORBIT_FOLLOW and orbit_unavailable
        ) or (
            self.pending_mode == MODE_RELATIVE_LOCK and relative_unavailable
        )
        if pending_unavailable:
            assert self.pending_request is not None
            self.pending_request._cancel()
            self.pending_request = None
            changed = True
        confirmed_unavailable = (
            self.mode == MODE_ORBIT_FOLLOW and orbit_unavailable
        ) or (
            self.mode == MODE_RELATIVE_LOCK and relative_unavailable
        )
        if confirmed_unavailable and not self.bridge_faulted:
            self.bridge_faulted = True
            changed = True
        if changed:
            self.neutral_rearm_required = True
        return changed

    def acknowledge_neutral_rearm(self) -> None:
        if self.pending_request is not None or self.bridge_faulted:
            raise CameraModeUnavailable(
                "neutral re-arm cannot complete during camera handoff/fault"
            )
        self.neutral_rearm_required = False


@dataclass(frozen=True)
class OrbitCameraConfig:
    pivot_height_m: float = 1.15
    desired_arm_m: float = 3.20
    maximum_desired_arm_m: float = 8.00
    probe_radius_m: float = 0.20
    collision_padding_m: float = 0.08
    minimum_operational_arm_m: float = 0.05
    recovery_speed_mps: float = 2.50
    minimum_pitch_rad: float = math.radians(-25.0)
    maximum_pitch_rad: float = math.radians(65.0)
    initial_yaw_rad: float = 0.0
    initial_pitch_rad: float = math.radians(15.0)
    maximum_step_s: float = 0.25

    def __post_init__(self) -> None:
        nonnegative = (
            "pivot_height_m",
            "desired_arm_m",
            "maximum_desired_arm_m",
            "probe_radius_m",
            "collision_padding_m",
            "minimum_operational_arm_m",
            "recovery_speed_mps",
            "maximum_step_s",
        )
        for name in nonnegative:
            object.__setattr__(
                self,
                name,
                _finite(getattr(self, name), name=name, nonnegative=True),
            )
        for name in (
            "minimum_pitch_rad",
            "maximum_pitch_rad",
            "initial_yaw_rad",
            "initial_pitch_rad",
        ):
            object.__setattr__(
                self, name, _finite(getattr(self, name), name=name)
            )
        if self.desired_arm_m <= 0.0:
            raise ValueError("desired_arm_m must be positive")
        if self.maximum_desired_arm_m <= 0.0:
            raise ValueError("maximum_desired_arm_m must be positive")
        if self.probe_radius_m <= 0.0:
            raise ValueError("probe_radius_m must be positive")
        if self.recovery_speed_mps <= 0.0:
            raise ValueError("recovery_speed_mps must be positive")
        if self.maximum_step_s <= 0.0:
            raise ValueError("maximum_step_s must be positive")
        if not 0.0 <= self.minimum_operational_arm_m < self.desired_arm_m:
            raise ValueError(
                "minimum_operational_arm_m must be in [0, desired_arm_m)"
            )
        if self.desired_arm_m > self.maximum_desired_arm_m:
            raise ValueError("desired_arm_m must not exceed maximum_desired_arm_m")
        if self.minimum_pitch_rad >= self.maximum_pitch_rad:
            raise ValueError("camera pitch limits must be ordered")
        if not (
            self.minimum_pitch_rad
            <= self.initial_pitch_rad
            <= self.maximum_pitch_rad
        ):
            raise ValueError("initial camera pitch must be inside its limits")


@dataclass(frozen=True)
class SphereSweepHit:
    """First blocking result along pivot -> desired camera, excluding the robot."""

    blocking: bool
    distance_m: float | None = None
    started_penetrating: bool = False

    def __post_init__(self) -> None:
        if type(self.blocking) is not bool:
            raise ValueError("sphere sweep blocking must be boolean")
        if type(self.started_penetrating) is not bool:
            raise ValueError("sphere sweep started_penetrating must be boolean")
        if self.blocking:
            if self.distance_m is None:
                raise ValueError("a blocking sphere sweep requires distance_m")
            object.__setattr__(
                self,
                "distance_m",
                _finite(
                    self.distance_m,
                    name="sphere_sweep.distance_m",
                    nonnegative=True,
                ),
            )
        elif self.distance_m is not None:
            raise ValueError("a clear sphere sweep must not carry distance_m")


@dataclass(frozen=True)
class OrbitCameraState:
    pivot: Vec3
    camera_position: Vec3
    look_at: Vec3
    yaw_rad: float
    pitch_rad: float
    desired_arm_m: float
    actual_arm_m: float
    collision_limited: bool
    _handoff_provenance: object | None = field(
        default=None, repr=False, compare=False
    )

    @property
    def offset(self) -> Vec3:
        return self.camera_position - self.pivot


@dataclass(frozen=True)
class LookAtHandoffState:
    """A safe-stop transition view that is not yet an orbit-follow frame."""

    target_orbit: OrbitCameraState
    look_at: Vec3
    progress: float
    complete: bool

    def __post_init__(self) -> None:
        if not isinstance(self.target_orbit, OrbitCameraState):
            raise ValueError("look-at handoff target must be OrbitCameraState")
        if not isinstance(self.look_at, Vec3):
            raise ValueError("look-at handoff target must be Vec3")
        progress = _finite(self.progress, name="handoff progress", nonnegative=True)
        if progress > 1.0:
            raise ValueError("look-at handoff progress must not exceed one")
        if type(self.complete) is not bool:
            raise ValueError("look-at handoff complete must be boolean")
        if self.complete != (progress >= 1.0):
            raise ValueError("look-at handoff completion disagrees with progress")
        object.__setattr__(self, "progress", progress)

    @property
    def pivot(self) -> Vec3:
        return self.target_orbit.pivot

    @property
    def camera_position(self) -> Vec3:
        return self.target_orbit.camera_position


class LookAtHandoffController:
    """Blend free view direction to the pivot without claiming orbit early.

    While ``complete`` is false, production must retain the request's confirmed
    source mode and keep robot control stopped.  ``target_orbit`` comes from the
    collision-resolved orbit controller, so smoothing view direction never
    weakens the wall/floor arm bound.  Only the complete target may be
    acknowledged as a strict ``orbit-follow`` frame.
    """

    def __init__(
        self,
        request: CameraModeRequest,
        *,
        duration_s: float = 0.20,
    ) -> None:
        if not isinstance(request, CameraModeRequest):
            raise ValueError("look-at handoff requires CameraModeRequest")
        if request.target_mode != MODE_ORBIT_FOLLOW:
            raise CameraModeUnavailable(
                "look-at handoff requires a pending orbit request"
            )
        if not request.active:
            raise CameraModeUnavailable("look-at handoff request is no longer pending")
        assert request.relative_pose is not None
        self.request = request
        self.relative_pose = request.relative_pose
        self.duration_s = _finite(
            duration_s, name="look-at handoff duration_s", nonnegative=True
        )
        if self.duration_s <= 0.0:
            raise ValueError("look-at handoff duration_s must be positive")
        self._elapsed_s = 0.0
        self._state: LookAtHandoffState | None = None

    @property
    def state(self) -> LookAtHandoffState | None:
        return self._state

    def step(
        self, *, target_orbit: OrbitCameraState, dt_s: float
    ) -> LookAtHandoffState:
        if not isinstance(target_orbit, OrbitCameraState):
            raise ValueError("target_orbit must be OrbitCameraState")
        if not self.request.active:
            raise CameraModeUnavailable("look-at handoff request is no longer pending")
        if target_orbit._handoff_provenance is not self.request._provenance:
            raise CameraModeUnavailable(
                "look-at target was not derived from this camera request"
            )
        dt = _finite(dt_s, name="dt_s", nonnegative=True)
        next_elapsed = min(self.duration_s, self._elapsed_s + dt)
        linear = next_elapsed / self.duration_s
        # Smoothstep has zero velocity at both ends and is deterministic across
        # UE and Python conformance tests when driven by the same elapsed time.
        progress = linear * linear * (3.0 - 2.0 * linear)
        complete = next_elapsed >= self.duration_s
        if complete:
            progress = 1.0
            look_at = target_orbit.pivot
        else:
            source_direction = (
                self.relative_pose.look_at_offset
                - self.relative_pose.camera_offset
            )
            target_direction = target_orbit.pivot - target_orbit.camera_position
            direction = _slerp_direction(
                source_direction, target_direction, progress
            )
            focus_distance = (
                source_direction.length * (1.0 - progress)
                + target_direction.length * progress
            )
            look_at = target_orbit.camera_position + direction.scaled(
                focus_distance
            )
        state = LookAtHandoffState(
            target_orbit=target_orbit,
            look_at=look_at,
            progress=progress,
            complete=complete,
        )
        self._elapsed_s = next_elapsed
        self._state = state
        return state


SphereSweep = Callable[[Vec3, Vec3, float], SphereSweepHit]


def orbit_offset(*, yaw_rad: float, pitch_rad: float, arm_m: float) -> Vec3:
    """Return pivot-to-camera offset for the documented orbit convention."""

    yaw = _finite(yaw_rad, name="yaw_rad")
    pitch = _finite(pitch_rad, name="pitch_rad")
    arm = _finite(arm_m, name="arm_m", nonnegative=True)
    horizontal = arm * math.cos(pitch)
    return Vec3(
        -horizontal * math.cos(yaw),
        -horizontal * math.sin(yaw),
        arm * math.sin(pitch),
    )


class OrbitCameraController:
    """Deterministic conformance reference for a cooked UE orbit-camera rig.

    Pivot translation is exact, so robot motion preserves the camera offset in
    an unobstructed scene.  A newly closer collision contracts the arm in the
    same step (never easing through a wall); clearance recovery is rate-limited.
    ``sphere_sweep`` is mandatory on every step and must represent the UE
    collision world, including walls and ground on the Camera channel.
    """

    def __init__(self, config: OrbitCameraConfig | None = None) -> None:
        self.config = config or OrbitCameraConfig()
        self._yaw = wrap_angle_rad(self.config.initial_yaw_rad)
        self._pitch = self.config.initial_pitch_rad
        self._desired_arm = self.config.desired_arm_m
        self._actual_arm = self.config.desired_arm_m
        self._handoff_provenance: object | None = None
        self._state: OrbitCameraState | None = None

    @property
    def state(self) -> OrbitCameraState | None:
        return self._state

    def step(
        self,
        *,
        robot_position: Vec3,
        yaw_delta_rad: float,
        pitch_delta_rad: float,
        dt_s: float,
        sphere_sweep: SphereSweep,
    ) -> OrbitCameraState:
        if not isinstance(robot_position, Vec3):
            raise ValueError("robot_position must be Vec3")
        yaw_delta = _finite(yaw_delta_rad, name="yaw_delta_rad")
        pitch_delta = _finite(pitch_delta_rad, name="pitch_delta_rad")
        dt = min(
            _finite(dt_s, name="dt_s", nonnegative=True),
            self.config.maximum_step_s,
        )

        next_yaw = wrap_angle_rad(self._yaw + yaw_delta)
        next_pitch = max(
            self.config.minimum_pitch_rad,
            min(self.config.maximum_pitch_rad, self._pitch + pitch_delta),
        )
        pivot = robot_position + Vec3(0.0, 0.0, self.config.pivot_height_m)
        state = self._resolve_state(
            pivot=pivot,
            yaw_rad=next_yaw,
            pitch_rad=next_pitch,
            desired_arm_m=self._desired_arm,
            previous_arm_m=self._actual_arm,
            dt_s=dt,
            sphere_sweep=sphere_sweep,
            handoff_provenance=self._handoff_provenance,
        )
        self._commit(state)
        return state

    def relock_from_free(
        self,
        *,
        frame: CameraBridgeFrame,
        dt_s: float,
        sphere_sweep: SphereSweep,
    ) -> OrbitCameraState:
        """Math-only free-view relock helper without transition provenance.

        A valid clear result preserves camera position and targets the pivot;
        collision may shorten the arm immediately.  This method cannot produce
        a state accepted by ``LookAtHandoffController``.  Production transition
        tests must use ``relock_from_request`` instead.
        """

        pose = RelativeCameraPose.capture(frame)
        return self._relock_from_pose(
            frame=frame,
            pose=pose,
            dt_s=dt_s,
            sphere_sweep=sphere_sweep,
            handoff_provenance=None,
        )

    def relock_from_request(
        self,
        *,
        request: CameraModeRequest,
        dt_s: float,
        sphere_sweep: SphereSweep,
    ) -> OrbitCameraState:
        """Build the only transition-authorized orbit target for a request."""

        if not isinstance(request, CameraModeRequest):
            raise ValueError("orbit relock requires CameraModeRequest")
        if request.target_mode != MODE_ORBIT_FOLLOW:
            raise CameraModeUnavailable("camera request does not target orbit")
        if not request.active:
            raise CameraModeUnavailable("orbit camera request is no longer pending")
        assert request.relative_pose is not None
        return self._relock_from_pose(
            frame=request.source_frame,
            pose=request.relative_pose,
            dt_s=dt_s,
            sphere_sweep=sphere_sweep,
            handoff_provenance=request._provenance,
        )

    def _relock_from_pose(
        self,
        *,
        frame: CameraBridgeFrame,
        pose: RelativeCameraPose,
        dt_s: float,
        sphere_sweep: SphereSweep,
        handoff_provenance: object | None,
    ) -> OrbitCameraState:
        if frame.input_captured:
            raise CameraModeUnavailable(
                "camera input must be released before relative pose handoff"
            )
        if not (
            self.config.minimum_pitch_rad
            <= pose.pitch_rad
            <= self.config.maximum_pitch_rad
        ):
            raise CameraModeUnavailable(
                "free-camera pitch lies outside orbit-follow limits"
            )
        if not (
            self.config.minimum_operational_arm_m
            <= pose.arm_m
            <= self.config.maximum_desired_arm_m
        ):
            raise CameraModeUnavailable(
                "free-camera arm lies outside orbit-follow limits"
            )
        dt = min(
            _finite(dt_s, name="dt_s", nonnegative=True),
            self.config.maximum_step_s,
        )
        state = self._resolve_state(
            pivot=frame.robot_pivot,
            yaw_rad=pose.yaw_rad,
            pitch_rad=pose.pitch_rad,
            desired_arm_m=pose.arm_m,
            previous_arm_m=pose.arm_m,
            dt_s=dt,
            sphere_sweep=sphere_sweep,
            handoff_provenance=handoff_provenance,
        )
        self._commit(state)
        return state

    def _resolve_state(
        self,
        *,
        pivot: Vec3,
        yaw_rad: float,
        pitch_rad: float,
        desired_arm_m: float,
        previous_arm_m: float,
        dt_s: float,
        sphere_sweep: SphereSweep,
        handoff_provenance: object | None,
    ) -> OrbitCameraState:
        if not callable(sphere_sweep):
            raise CameraCollisionUnavailable("a UE sphere sweep callback is required")
        desired_position = pivot + orbit_offset(
            yaw_rad=yaw_rad,
            pitch_rad=pitch_rad,
            arm_m=desired_arm_m,
        )
        try:
            hit = sphere_sweep(pivot, desired_position, self.config.probe_radius_m)
        except CameraCollisionUnavailable:
            raise
        except Exception as exc:
            raise CameraCollisionUnavailable(
                f"UE camera sphere sweep failed: {type(exc).__name__}: {exc}"
            ) from exc
        if not isinstance(hit, SphereSweepHit):
            raise CameraCollisionUnavailable(
                "UE camera sphere sweep returned an invalid result"
            )
        if hit.started_penetrating:
            raise CameraCollisionUnavailable(
                "camera pivot started inside blocking geometry"
            )

        safe_arm = desired_arm_m
        if hit.blocking:
            assert hit.distance_m is not None
            if hit.distance_m > desired_arm_m + 1e-9:
                raise CameraCollisionUnavailable(
                    "camera sphere sweep hit lies beyond its requested segment"
                )
            safe_arm = max(0.0, hit.distance_m - self.config.collision_padding_m)
            if safe_arm < self.config.minimum_operational_arm_m:
                raise CameraCollisionUnavailable(
                    "camera collision leaves no operational arm clearance"
                )

        if previous_arm_m > safe_arm:
            # Collision contraction is immediate: smoothing may never put the
            # camera beyond the first verified blocking clearance.
            next_arm = safe_arm
        else:
            next_arm = min(
                safe_arm,
                previous_arm_m + self.config.recovery_speed_mps * dt_s,
            )
        # This flag means the final view is still arm-constrained, including
        # rate-limited recovery after the blocking sweep has become clear.  It
        # must agree with the strict bridge-frame invariant actual < desired.
        collision_limited = next_arm < desired_arm_m - 1e-9
        camera_position = pivot + orbit_offset(
            yaw_rad=yaw_rad,
            pitch_rad=pitch_rad,
            arm_m=next_arm,
        )
        state = OrbitCameraState(
            pivot=pivot,
            camera_position=camera_position,
            look_at=pivot,
            yaw_rad=yaw_rad,
            pitch_rad=pitch_rad,
            desired_arm_m=desired_arm_m,
            actual_arm_m=next_arm,
            collision_limited=collision_limited,
            _handoff_provenance=handoff_provenance,
        )
        return state

    def _commit(self, state: OrbitCameraState) -> None:
        # Commit only after a complete authoritative collision result.  A
        # failed query leaves the last valid orbit state untouched.
        self._yaw = state.yaw_rad
        self._pitch = state.pitch_rad
        self._desired_arm = state.desired_arm_m
        self._actual_arm = state.actual_arm_m
        self._handoff_provenance = state._handoff_provenance
        self._state = state
