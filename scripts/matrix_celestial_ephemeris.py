#!/usr/bin/env python3
"""Deterministic, origin-rebased celestial frames for Matrix.

The built-in provider is intentionally a visual/navigation ephemeris.  Its
public API mirrors the state that a future NAIF SPICE provider must return, so
physics and UI code do not depend on one ephemeris implementation.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
from typing import Callable, Mapping


NANOSECONDS_PER_SECOND = 1_000_000_000
SECONDS_PER_DAY = 86_400.0
DAYS_PER_JULIAN_CENTURY = 36_525.0
ASTRONOMICAL_UNIT_M = 149_597_870_700.0
SOLAR_CONSTANT_W_M2 = 1_361.0
J2000_UTC = datetime(2000, 1, 1, 12, 0, 0, tzinfo=timezone.utc)
J2000_OBLIQUITY_RAD = math.radians(23.43928)
CLOCK_SCHEMA = "matrix-celestial-clock/v1"
EPHEMERIS_PROVIDER = "matrix-analytical-v1"
EPHEMERIS_ACCURACY = "visual-navigation"
JPL_EPHEMERIS_PROVIDER = "jpl-de440s-v1"
JPL_EPHEMERIS_ACCURACY = "de440-position-visual-rotation"
MAX_ABSOLUTE_ELAPSED_NS = 10_000 * 366 * 86_400 * NANOSECONDS_PER_SECOND
ASSET_MANIFEST_SCHEMA = "matrix-celestial-assets/v1"

Vector3 = tuple[float, float, float]
Matrix3 = tuple[Vector3, Vector3, Vector3]


class CelestialEphemerisError(ValueError):
    """Raised when celestial time, frames, or ephemeris data are invalid."""


def verify_locked_ephemeris_assets(
    manifest_path: Path,
    *,
    kernel_path: Path,
    jplephem_wheel: Path,
) -> None:
    """Verify the exact offline wheel and DE440s bytes before importing either."""

    if (
        not manifest_path.is_absolute()
        or manifest_path.is_symlink()
        or not manifest_path.is_file()
    ):
        raise CelestialEphemerisError(
            "celestial asset manifest must be an absolute regular file"
        )
    try:
        value = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise CelestialEphemerisError(
            f"cannot load celestial asset manifest: {exc}"
        ) from exc
    if not isinstance(value, dict) or set(value) != {
        "schema",
        "provider",
        "coverage",
        "assets",
    }:
        raise CelestialEphemerisError("celestial asset manifest schema is invalid")
    if (
        value.get("schema") != ASSET_MANIFEST_SCHEMA
        or value.get("provider") != JPL_EPHEMERIS_PROVIDER
    ):
        raise CelestialEphemerisError("celestial asset provider is unsupported")
    assets = value.get("assets")
    if not isinstance(assets, list):
        raise CelestialEphemerisError("celestial asset list is invalid")
    expected_paths = {
        "de440s_spk": kernel_path,
        "jplephem_wheel": jplephem_wheel,
    }
    seen: set[str] = set()
    for item in assets:
        if not isinstance(item, dict) or set(item) != {
            "role",
            "filename",
            "size",
            "sha256",
            "urls",
        }:
            raise CelestialEphemerisError("celestial asset entry is invalid")
        role = item.get("role")
        if role not in expected_paths or role in seen:
            raise CelestialEphemerisError("celestial asset role is invalid")
        path = expected_paths[role]
        if (
            not path.is_absolute()
            or path.is_symlink()
            or not path.is_file()
            or path.name != item.get("filename")
        ):
            raise CelestialEphemerisError(f"celestial asset {role} path is invalid")
        size = item.get("size")
        digest = item.get("sha256")
        if (
            isinstance(size, bool)
            or not isinstance(size, int)
            or size <= 0
            or not isinstance(digest, str)
            or len(digest) != 64
            or any(character not in "0123456789abcdef" for character in digest)
            or path.stat().st_size != size
        ):
            raise CelestialEphemerisError(f"celestial asset {role} lock is invalid")
        hasher = hashlib.sha256()
        try:
            with path.open("rb") as stream:
                for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                    hasher.update(chunk)
        except OSError as exc:
            raise CelestialEphemerisError(
                f"cannot read celestial asset {role}: {exc}"
            ) from exc
        if hasher.hexdigest() != digest:
            raise CelestialEphemerisError(
                f"celestial asset {role} SHA256 does not match"
            )
        seen.add(role)
    if seen != set(expected_paths):
        raise CelestialEphemerisError("celestial asset manifest is incomplete")


def _finite(value: object, *, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CelestialEphemerisError(f"{label} must be finite")
    try:
        result = float(value)
    except (OverflowError, ValueError) as exc:
        raise CelestialEphemerisError(f"{label} must be finite") from exc
    if not math.isfinite(result):
        raise CelestialEphemerisError(f"{label} must be finite")
    return result


def _integer(value: object, *, label: str, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise CelestialEphemerisError(f"{label} must be an integer")
    if not minimum <= value <= maximum:
        raise CelestialEphemerisError(f"{label} is outside its allowed range")
    return value


def _dot(left: Vector3, right: Vector3) -> float:
    return sum(left[index] * right[index] for index in range(3))


def _cross(left: Vector3, right: Vector3) -> Vector3:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _norm(value: Vector3) -> float:
    return math.sqrt(_dot(value, value))


def _unit(value: Vector3, *, label: str) -> Vector3:
    length = _norm(value)
    if not math.isfinite(length) or length <= 1e-12:
        raise CelestialEphemerisError(f"{label} has no direction")
    return tuple(component / length for component in value)  # type: ignore[return-value]


def _add(left: Vector3, right: Vector3) -> Vector3:
    return tuple(left[index] + right[index] for index in range(3))  # type: ignore[return-value]


def _subtract(left: Vector3, right: Vector3) -> Vector3:
    return tuple(left[index] - right[index] for index in range(3))  # type: ignore[return-value]


def _scale(value: Vector3, factor: float) -> Vector3:
    return tuple(component * factor for component in value)  # type: ignore[return-value]


def _matvec(matrix: Matrix3, value: Vector3) -> Vector3:
    return tuple(_dot(row, value) for row in matrix)  # type: ignore[return-value]


def _transpose(matrix: Matrix3) -> Matrix3:
    return tuple(
        tuple(matrix[row][column] for row in range(3))
        for column in range(3)
    )  # type: ignore[return-value]


def _columns(first: Vector3, second: Vector3, third: Vector3) -> Matrix3:
    return tuple(
        (first[row], second[row], third[row]) for row in range(3)
    )  # type: ignore[return-value]


def _rotate_about_axis(value: Vector3, axis: Vector3, angle_rad: float) -> Vector3:
    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)
    return _add(
        _add(_scale(value, cosine), _scale(_cross(axis, value), sine)),
        _scale(axis, _dot(axis, value) * (1.0 - cosine)),
    )


def _wrap_degrees(value: float) -> float:
    return value % 360.0


def _canonical_epoch(value: str) -> datetime:
    try:
        parsed = datetime.strptime(value, "%Y-%m-%dT%H:%M:%SZ").replace(
            tzinfo=timezone.utc
        )
    except (TypeError, ValueError) as exc:
        raise CelestialEphemerisError(
            "reference epoch must be canonical UTC"
        ) from exc
    return parsed


@dataclass(frozen=True)
class KeplerOrbit:
    """Mean orbital elements and linear rates in Julian centuries."""

    parent_id: str
    semi_major_axis_m: float
    semi_major_axis_rate_m_per_century: float
    eccentricity: float
    eccentricity_rate_per_century: float
    inclination_deg: float
    inclination_rate_deg_per_century: float
    mean_longitude_deg: float
    mean_longitude_rate_deg_per_century: float
    longitude_periapsis_deg: float
    longitude_periapsis_rate_deg_per_century: float
    longitude_ascending_node_deg: float
    longitude_ascending_node_rate_deg_per_century: float

    def __post_init__(self) -> None:
        values = {
            field: _finite(getattr(self, field), label=field)
            for field in (
                "semi_major_axis_m",
                "semi_major_axis_rate_m_per_century",
                "eccentricity",
                "eccentricity_rate_per_century",
                "inclination_deg",
                "inclination_rate_deg_per_century",
                "mean_longitude_deg",
                "mean_longitude_rate_deg_per_century",
                "longitude_periapsis_deg",
                "longitude_periapsis_rate_deg_per_century",
                "longitude_ascending_node_deg",
                "longitude_ascending_node_rate_deg_per_century",
            )
        }
        if not self.parent_id:
            raise CelestialEphemerisError("orbit parent id is required")
        if values["semi_major_axis_m"] <= 0.0:
            raise CelestialEphemerisError("semi-major axis must be positive")
        if not 0.0 <= values["eccentricity"] < 0.99:
            raise CelestialEphemerisError("orbit eccentricity is invalid")

    def relative_position_m(self, julian_centuries: float) -> Vector3:
        centuries = _finite(julian_centuries, label="julian_centuries")
        semi_major = self.semi_major_axis_m + (
            self.semi_major_axis_rate_m_per_century * centuries
        )
        eccentricity = self.eccentricity + (
            self.eccentricity_rate_per_century * centuries
        )
        if semi_major <= 0.0 or not 0.0 <= eccentricity < 0.99:
            raise CelestialEphemerisError("orbit left its supported element range")
        inclination = math.radians(
            self.inclination_deg
            + self.inclination_rate_deg_per_century * centuries
        )
        mean_longitude = self.mean_longitude_deg + (
            self.mean_longitude_rate_deg_per_century * centuries
        )
        longitude_periapsis = self.longitude_periapsis_deg + (
            self.longitude_periapsis_rate_deg_per_century * centuries
        )
        longitude_node = self.longitude_ascending_node_deg + (
            self.longitude_ascending_node_rate_deg_per_century * centuries
        )
        mean_anomaly = math.radians(
            ((_wrap_degrees(mean_longitude - longitude_periapsis) + 180.0) % 360.0)
            - 180.0
        )
        eccentric_anomaly = mean_anomaly
        for _iteration in range(16):
            residual = (
                eccentric_anomaly
                - eccentricity * math.sin(eccentric_anomaly)
                - mean_anomaly
            )
            derivative = 1.0 - eccentricity * math.cos(eccentric_anomaly)
            delta = residual / derivative
            eccentric_anomaly -= delta
            if abs(delta) <= 1e-14:
                break
        orbital_x = semi_major * (math.cos(eccentric_anomaly) - eccentricity)
        orbital_y = semi_major * math.sqrt(1.0 - eccentricity * eccentricity) * (
            math.sin(eccentric_anomaly)
        )
        argument_periapsis = math.radians(longitude_periapsis - longitude_node)
        node = math.radians(longitude_node)
        cos_argument = math.cos(argument_periapsis)
        sin_argument = math.sin(argument_periapsis)
        cos_node = math.cos(node)
        sin_node = math.sin(node)
        cos_inclination = math.cos(inclination)
        sin_inclination = math.sin(inclination)
        ecliptic = (
            (
                cos_argument * cos_node
                - sin_argument * sin_node * cos_inclination
            )
            * orbital_x
            + (
                -sin_argument * cos_node
                - cos_argument * sin_node * cos_inclination
            )
            * orbital_y,
            (
                cos_argument * sin_node
                + sin_argument * cos_node * cos_inclination
            )
            * orbital_x
            + (
                -sin_argument * sin_node
                + cos_argument * cos_node * cos_inclination
            )
            * orbital_y,
            sin_argument * sin_inclination * orbital_x
            + cos_argument * sin_inclination * orbital_y,
        )
        cosine = math.cos(J2000_OBLIQUITY_RAD)
        sine = math.sin(J2000_OBLIQUITY_RAD)
        return (
            ecliptic[0],
            ecliptic[1] * cosine - ecliptic[2] * sine,
            ecliptic[1] * sine + ecliptic[2] * cosine,
        )


@dataclass(frozen=True)
class UniformRotation:
    pole_right_ascension_deg: float
    pole_right_ascension_rate_deg_per_century: float
    pole_declination_deg: float
    pole_declination_rate_deg_per_century: float
    prime_meridian_deg: float
    spin_rate_deg_per_day: float

    def body_to_inertial(self, julian_days: float) -> Matrix3:
        days = _finite(julian_days, label="julian_days")
        centuries = days / DAYS_PER_JULIAN_CENTURY
        right_ascension = math.radians(
            self.pole_right_ascension_deg
            + self.pole_right_ascension_rate_deg_per_century * centuries
        )
        declination = math.radians(
            self.pole_declination_deg
            + self.pole_declination_rate_deg_per_century * centuries
        )
        pole = _unit(
            (
                math.cos(declination) * math.cos(right_ascension),
                math.cos(declination) * math.sin(right_ascension),
                math.sin(declination),
            ),
            label="body pole",
        )
        zero_meridian = (
            -math.sin(right_ascension),
            math.cos(right_ascension),
            0.0,
        )
        meridian_angle = math.radians(
            _wrap_degrees(
                self.prime_meridian_deg + self.spin_rate_deg_per_day * days
            )
        )
        body_x = _unit(
            _rotate_about_axis(zero_meridian, pole, meridian_angle),
            label="body prime meridian",
        )
        body_y = _unit(_cross(pole, body_x), label="body east axis")
        return _columns(body_x, body_y, pole)


@dataclass(frozen=True)
class CelestialBodyDefinition:
    body_id: str
    display_name: str
    naif_id: int
    ellipsoid_radii_m: Vector3
    gravity_m_s2: float
    atmosphere: str
    runtime_status: str
    rotation: UniformRotation
    orbit: KeplerOrbit | None
    barycenter_companion_id: str | None = None
    primary_to_companion_mass_ratio: float | None = None

    def __post_init__(self) -> None:
        radii = tuple(
            _finite(value, label=f"{self.body_id}.ellipsoid_radii_m")
            for value in self.ellipsoid_radii_m
        )
        if len(radii) != 3 or any(value <= 0.0 for value in radii):
            raise CelestialEphemerisError("body ellipsoid radii are invalid")
        if max(radii) > 1_000_000_000.0:
            raise CelestialEphemerisError("body ellipsoid is outside supported range")
        gravity = _finite(self.gravity_m_s2, label=f"{self.body_id}.gravity_m_s2")
        if gravity <= 0.0:
            raise CelestialEphemerisError("body gravity must be positive")
        if (self.barycenter_companion_id is None) != (
            self.primary_to_companion_mass_ratio is None
        ):
            raise CelestialEphemerisError("barycenter correction is incomplete")
        if self.primary_to_companion_mass_ratio is not None and (
            _finite(
                self.primary_to_companion_mass_ratio,
                label="primary_to_companion_mass_ratio",
            )
            <= 0.0
        ):
            raise CelestialEphemerisError("barycenter mass ratio must be positive")


@dataclass(frozen=True)
class BodyState:
    body_id: str
    center_inertial_m: Vector3
    body_to_inertial: Matrix3

    def body_fixed_to_inertial(self, position_m: Vector3) -> Vector3:
        return _add(self.center_inertial_m, _matvec(self.body_to_inertial, position_m))

    def inertial_direction_to_body_fixed(self, direction: Vector3) -> Vector3:
        return _matvec(_transpose(self.body_to_inertial), direction)


@dataclass(frozen=True)
class SurfaceAnchor:
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    heading_deg: float

    def __post_init__(self) -> None:
        latitude = _finite(self.latitude_deg, label="surface latitude")
        longitude = _finite(self.longitude_deg, label="surface longitude")
        altitude = _finite(self.altitude_m, label="surface altitude")
        _finite(self.heading_deg, label="surface heading")
        if not -90.0 <= latitude <= 90.0:
            raise CelestialEphemerisError("surface latitude is invalid")
        if not -180.0 <= longitude <= 180.0:
            raise CelestialEphemerisError("surface longitude is invalid")
        if not -100_000.0 <= altitude <= 10_000_000.0:
            raise CelestialEphemerisError("surface altitude is invalid")

    def body_fixed_position(self, radii_m: Vector3) -> Vector3:
        semi_major_x, semi_major_y, semi_minor = radii_m
        if not math.isclose(semi_major_x, semi_major_y, rel_tol=0.0, abs_tol=1e-3):
            raise CelestialEphemerisError("triaxial geodetic anchors are unsupported")
        latitude = math.radians(self.latitude_deg)
        longitude = math.radians(self.longitude_deg)
        eccentricity_squared = 1.0 - (
            semi_minor * semi_minor / (semi_major_x * semi_major_x)
        )
        normal_radius = semi_major_x / math.sqrt(
            1.0 - eccentricity_squared * math.sin(latitude) ** 2
        )
        return (
            (normal_radius + self.altitude_m)
            * math.cos(latitude)
            * math.cos(longitude),
            (normal_radius + self.altitude_m)
            * math.cos(latitude)
            * math.sin(longitude),
            (
                normal_radius * (1.0 - eccentricity_squared) + self.altitude_m
            )
            * math.sin(latitude),
        )

    def local_to_body_fixed_basis(self) -> Matrix3:
        latitude = math.radians(self.latitude_deg)
        longitude = math.radians(self.longitude_deg)
        heading = math.radians(self.heading_deg)
        east = (-math.sin(longitude), math.cos(longitude), 0.0)
        north = (
            -math.sin(latitude) * math.cos(longitude),
            -math.sin(latitude) * math.sin(longitude),
            math.cos(latitude),
        )
        up = (
            math.cos(latitude) * math.cos(longitude),
            math.cos(latitude) * math.sin(longitude),
            math.sin(latitude),
        )
        forward = _add(_scale(east, math.sin(heading)), _scale(north, math.cos(heading)))
        left = _add(_scale(east, -math.cos(heading)), _scale(north, math.sin(heading)))
        return _columns(forward, left, up)

    def local_position_to_body_fixed(
        self, radii_m: Vector3, local_position_m: Vector3
    ) -> Vector3:
        return _add(
            self.body_fixed_position(radii_m),
            _matvec(self.local_to_body_fixed_basis(), local_position_m),
        )


class AnalyticalEphemeris:
    """Low-cost heliocentric ICRF provider for visual/navigation state."""

    def __init__(
        self,
        bodies: tuple[CelestialBodyDefinition, ...],
        *,
        reference_epoch_utc: str,
        tai_minus_utc_at_epoch_s: int,
    ) -> None:
        self._bodies = {body.body_id: body for body in bodies}
        if len(self._bodies) != len(bodies):
            raise CelestialEphemerisError("body ids must be unique")
        if "sun" not in self._bodies or self._bodies["sun"].orbit is not None:
            raise CelestialEphemerisError("analytical ephemeris requires a fixed Sun")
        self._reference_epoch = _canonical_epoch(reference_epoch_utc)
        self._tai_minus_utc = _integer(
            tai_minus_utc_at_epoch_s,
            label="tai_minus_utc_at_epoch_s",
            minimum=0,
            maximum=1000,
        )
        self._reference_days = (
            self._reference_epoch - J2000_UTC
        ).total_seconds() / SECONDS_PER_DAY + (
            self._tai_minus_utc + 32.184
        ) / SECONDS_PER_DAY
        for body in bodies:
            if body.orbit is not None and body.orbit.parent_id not in self._bodies:
                raise CelestialEphemerisError(
                    f"{body.body_id} orbit parent does not exist"
                )
            if (
                body.barycenter_companion_id is not None
                and body.barycenter_companion_id not in self._bodies
            ):
                raise CelestialEphemerisError(
                    f"{body.body_id} barycenter companion does not exist"
                )

    @property
    def reference_epoch_utc(self) -> str:
        return self._reference_epoch.strftime("%Y-%m-%dT%H:%M:%SZ")

    def julian_days(self, elapsed_tai_ns: int) -> float:
        elapsed = _integer(
            elapsed_tai_ns,
            label="elapsed_tai_ns",
            minimum=-MAX_ABSOLUTE_ELAPSED_NS,
            maximum=MAX_ABSOLUTE_ELAPSED_NS,
        )
        return self._reference_days + elapsed / NANOSECONDS_PER_SECOND / SECONDS_PER_DAY

    def _raw_relative(self, body_id: str, centuries: float) -> Vector3:
        orbit = self._bodies[body_id].orbit
        if orbit is None:
            return (0.0, 0.0, 0.0)
        return orbit.relative_position_m(centuries)

    def states(self, elapsed_tai_ns: int) -> Mapping[str, BodyState]:
        days = self.julian_days(elapsed_tai_ns)
        centuries = days / DAYS_PER_JULIAN_CENTURY
        centers: dict[str, Vector3] = {"sun": (0.0, 0.0, 0.0)}
        visiting: set[str] = set()

        def center(body_id: str) -> Vector3:
            cached = centers.get(body_id)
            if cached is not None:
                return cached
            if body_id in visiting:
                raise CelestialEphemerisError("orbit parent graph contains a cycle")
            visiting.add(body_id)
            body = self._bodies[body_id]
            if body.orbit is None:
                result = (0.0, 0.0, 0.0)
            else:
                result = _add(
                    center(body.orbit.parent_id),
                    self._raw_relative(body_id, centuries),
                )
            if body.barycenter_companion_id is not None:
                companion = self._bodies[body.barycenter_companion_id]
                if companion.orbit is None or companion.orbit.parent_id != body_id:
                    raise CelestialEphemerisError(
                        "barycenter companion must orbit its corrected primary"
                    )
                ratio = body.primary_to_companion_mass_ratio
                assert ratio is not None
                result = _subtract(
                    result,
                    _scale(
                        self._raw_relative(companion.body_id, centuries),
                        1.0 / (ratio + 1.0),
                    ),
                )
            centers[body_id] = result
            visiting.remove(body_id)
            return result

        result: dict[str, BodyState] = {}
        for body_id, body in self._bodies.items():
            result[body_id] = BodyState(
                body_id=body_id,
                center_inertial_m=center(body_id),
                body_to_inertial=body.rotation.body_to_inertial(days),
            )
        return result

    def close(self) -> None:
        return None


class JplSpkEphemeris(AnalyticalEphemeris):
    """DE440s center positions with the same deterministic rotation contract."""

    def __init__(
        self,
        bodies: tuple[CelestialBodyDefinition, ...],
        *,
        reference_epoch_utc: str,
        tai_minus_utc_at_epoch_s: int,
        kernel_path: Path,
        jplephem_wheel: Path,
    ) -> None:
        super().__init__(
            bodies,
            reference_epoch_utc=reference_epoch_utc,
            tai_minus_utc_at_epoch_s=tai_minus_utc_at_epoch_s,
        )
        for path, label in (
            (kernel_path, "DE440s kernel"),
            (jplephem_wheel, "jplephem wheel"),
        ):
            if not path.is_absolute() or path.is_symlink() or not path.is_file():
                raise CelestialEphemerisError(f"{label} must be an absolute regular file")
        wheel_text = os.fspath(jplephem_wheel)
        if wheel_text not in sys.path:
            sys.path.insert(0, wheel_text)
        try:
            spk_module = importlib.import_module("jplephem.spk")
            module_path = os.fspath(getattr(spk_module, "__file__", ""))
            if not module_path.startswith(wheel_text + os.sep):
                raise ImportError("jplephem was not imported from the locked wheel")
            self._kernel = spk_module.SPK.open(os.fspath(kernel_path))
            self._closed = False
        except (ImportError, OSError, ValueError) as exc:
            raise CelestialEphemerisError(f"cannot open DE440s kernel: {exc}") from exc
        required_segments = {
            (0, 3),
            (0, 4),
            (0, 10),
            (3, 301),
            (3, 399),
        }
        available_segments = {
            (int(segment.center), int(segment.target))
            for segment in self._kernel.segments
        }
        missing = sorted(required_segments - available_segments)
        if missing:
            raise CelestialEphemerisError(
                f"DE440s kernel is missing required segments: {missing}"
            )
        try:
            self.states(0)
        except Exception as exc:
            raise CelestialEphemerisError(
                f"DE440s does not cover the scenario epoch: {exc}"
            ) from exc

    @staticmethod
    def _vector_from_array(value: object) -> Vector3:
        try:
            components = tuple(float(value[index]) for index in range(3))  # type: ignore[index]
        except (IndexError, TypeError, ValueError, OverflowError) as exc:
            raise CelestialEphemerisError("DE440s returned an invalid vector") from exc
        if any(not math.isfinite(component) for component in components):
            raise CelestialEphemerisError("DE440s returned a non-finite vector")
        return components  # type: ignore[return-value]

    def _spk_position_km(self, center: int, target: int, julian_date: float) -> Vector3:
        try:
            value = self._kernel[center, target].compute(julian_date)
        except Exception as exc:
            raise CelestialEphemerisError(
                f"DE440s position query failed for {center}->{target}: {exc}"
            ) from exc
        return self._vector_from_array(value)

    def states(self, elapsed_tai_ns: int) -> Mapping[str, BodyState]:
        if self._closed:
            raise CelestialEphemerisError("DE440s provider is closed")
        days = self.julian_days(elapsed_tai_ns)
        julian_date = 2_451_545.0 + days
        sun_barycentric = self._spk_position_km(0, 10, julian_date)
        earth_moon_barycenter = self._spk_position_km(0, 3, julian_date)
        barycentric_km = {
            "sun": sun_barycentric,
            "earth": _add(
                earth_moon_barycenter,
                self._spk_position_km(3, 399, julian_date),
            ),
            "moon": _add(
                earth_moon_barycenter,
                self._spk_position_km(3, 301, julian_date),
            ),
            "mars": self._spk_position_km(0, 4, julian_date),
        }
        unsupported = sorted(set(self._bodies) - set(barycentric_km))
        if unsupported:
            raise CelestialEphemerisError(
                f"DE440s adapter does not map configured bodies: {unsupported}"
            )
        result: dict[str, BodyState] = {}
        for body_id, body in self._bodies.items():
            heliocentric_km = _subtract(barycentric_km[body_id], sun_barycentric)
            result[body_id] = BodyState(
                body_id=body_id,
                center_inertial_m=_scale(heliocentric_km, 1000.0),
                body_to_inertial=body.rotation.body_to_inertial(days),
            )
        return result

    def close(self) -> None:
        if not getattr(self, "_closed", True):
            self._kernel.close()
            self._closed = True

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass


def _circle_overlap_fraction(
    *, sun_radius_rad: float, occluder_radius_rad: float, separation_rad: float
) -> float:
    sun_radius = max(0.0, sun_radius_rad)
    occluder_radius = max(0.0, occluder_radius_rad)
    separation = max(0.0, separation_rad)
    if sun_radius <= 0.0 or occluder_radius <= 0.0:
        return 0.0
    if separation >= sun_radius + occluder_radius:
        return 0.0
    if separation <= abs(occluder_radius - sun_radius):
        if occluder_radius >= sun_radius:
            return 1.0
        return (occluder_radius * occluder_radius) / (sun_radius * sun_radius)
    left = math.acos(
        max(
            -1.0,
            min(
                1.0,
                (
                    separation * separation
                    + sun_radius * sun_radius
                    - occluder_radius * occluder_radius
                )
                / (2.0 * separation * sun_radius),
            ),
        )
    )
    right = math.acos(
        max(
            -1.0,
            min(
                1.0,
                (
                    separation * separation
                    + occluder_radius * occluder_radius
                    - sun_radius * sun_radius
                )
                / (2.0 * separation * occluder_radius),
            ),
        )
    )
    area = (
        sun_radius * sun_radius * left
        + occluder_radius * occluder_radius * right
        - 0.5
        * math.sqrt(
            max(
                0.0,
                (-separation + sun_radius + occluder_radius)
                * (separation + sun_radius - occluder_radius)
                * (separation - sun_radius + occluder_radius)
                * (separation + sun_radius + occluder_radius),
            )
        )
    )
    return max(0.0, min(1.0, area / (math.pi * sun_radius * sun_radius)))


def solar_lighting_state(
    *,
    observer_body: CelestialBodyDefinition,
    anchor: SurfaceAnchor,
    local_position_m: Vector3,
    bodies: Mapping[str, CelestialBodyDefinition],
    states: Mapping[str, BodyState],
) -> dict[str, object]:
    body_state = states[observer_body.body_id]
    observer_body_fixed = anchor.local_position_to_body_fixed(
        observer_body.ellipsoid_radii_m,
        local_position_m,
    )
    observer_inertial = body_state.body_fixed_to_inertial(observer_body_fixed)
    sun_vector = _subtract(states["sun"].center_inertial_m, observer_inertial)
    solar_distance = _norm(sun_vector)
    sun_direction_inertial = _unit(sun_vector, label="observer-to-Sun vector")
    sun_direction_body = body_state.inertial_direction_to_body_fixed(
        sun_direction_inertial
    )
    local_basis = anchor.local_to_body_fixed_basis()
    sun_direction_local = _matvec(_transpose(local_basis), sun_direction_body)

    latitude = math.radians(anchor.latitude_deg)
    longitude = math.radians(anchor.longitude_deg)
    east = (-math.sin(longitude), math.cos(longitude), 0.0)
    north = (
        -math.sin(latitude) * math.cos(longitude),
        -math.sin(latitude) * math.sin(longitude),
        math.cos(latitude),
    )
    up = (
        math.cos(latitude) * math.cos(longitude),
        math.cos(latitude) * math.sin(longitude),
        math.sin(latitude),
    )
    east_component = _dot(sun_direction_body, east)
    north_component = _dot(sun_direction_body, north)
    up_component = max(-1.0, min(1.0, _dot(sun_direction_body, up)))
    altitude_deg = math.degrees(math.asin(up_component))
    azimuth_deg = _wrap_degrees(
        math.degrees(math.atan2(east_component, north_component))
    )

    sun_radius = max(bodies["sun"].ellipsoid_radii_m)
    sun_angular_radius = math.asin(min(1.0, sun_radius / solar_distance))
    eclipse_fraction = 0.0
    occluder_id: str | None = None
    for candidate_id, candidate in bodies.items():
        if candidate_id in {"sun", observer_body.body_id}:
            continue
        candidate_vector = _subtract(
            states[candidate_id].center_inertial_m,
            observer_inertial,
        )
        candidate_distance = _norm(candidate_vector)
        if candidate_distance <= 0.0 or candidate_distance >= solar_distance:
            continue
        candidate_direction = _unit(candidate_vector, label="eclipse occluder vector")
        alignment = max(-1.0, min(1.0, _dot(sun_direction_inertial, candidate_direction)))
        separation = math.acos(alignment)
        candidate_radius = max(candidate.ellipsoid_radii_m)
        candidate_angular_radius = math.asin(
            min(1.0, candidate_radius / candidate_distance)
        )
        overlap = _circle_overlap_fraction(
            sun_radius_rad=sun_angular_radius,
            occluder_radius_rad=candidate_angular_radius,
            separation_rad=separation,
        )
        if overlap > eclipse_fraction:
            eclipse_fraction = overlap
            occluder_id = candidate_id
    irradiance = SOLAR_CONSTANT_W_M2 * (
        ASTRONOMICAL_UNIT_M / solar_distance
    ) ** 2
    if altitude_deg >= 0.0:
        starfield_visibility = 0.0
    elif altitude_deg <= -18.0:
        starfield_visibility = 1.0
    else:
        starfield_visibility = max(0.0, min(1.0, -altitude_deg / 18.0))
    return {
        "body_id": observer_body.body_id,
        "atmosphere": observer_body.atmosphere,
        "sun_direction_local": list(sun_direction_local),
        "directional_light_direction_local": [
            -component for component in sun_direction_local
        ],
        "sun_altitude_deg": altitude_deg,
        "sun_azimuth_deg": azimuth_deg,
        "solar_distance_m": solar_distance,
        "solar_irradiance_w_m2": irradiance,
        "sun_angular_radius_deg": math.degrees(sun_angular_radius),
        "eclipse_fraction": eclipse_fraction,
        "eclipse_occluder_id": occluder_id,
        "starfield_visibility": starfield_visibility,
        "render_authority": "state-only",
        "render_status": "not-applied",
        "render_error": None,
        "visible_camera_verified": False,
    }


@dataclass(frozen=True)
class SimulationTimeSnapshot:
    elapsed_tai_ns: int
    scenario_tai_ns: int
    scenario_utc: str
    rate_numerator: int
    rate_denominator: int
    utc_assumption: str

    def mapping(self) -> dict[str, object]:
        return {
            "elapsed_tai_ns": self.elapsed_tai_ns,
            "scenario_tai_ns": self.scenario_tai_ns,
            "scenario_utc": self.scenario_utc,
            "rate_numerator": self.rate_numerator,
            "rate_denominator": self.rate_denominator,
            "utc_assumption": self.utc_assumption,
        }


class PersistentSimulationClock:
    """Integer monotonic scenario clock with bounded atomic checkpoints."""

    def __init__(
        self,
        *,
        universe_id: str,
        reference_epoch_utc: str,
        tai_minus_utc_at_epoch_s: int,
        rate_numerator: int = 1,
        rate_denominator: int = 1,
        state_path: Path | None = None,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        self.universe_id = universe_id
        self.reference_epoch_utc = reference_epoch_utc
        self._reference_epoch = _canonical_epoch(reference_epoch_utc)
        self._tai_minus_utc = _integer(
            tai_minus_utc_at_epoch_s,
            label="tai_minus_utc_at_epoch_s",
            minimum=0,
            maximum=1000,
        )
        self.rate_numerator = _integer(
            rate_numerator,
            label="clock rate numerator",
            minimum=0,
            maximum=1_000_000,
        )
        self.rate_denominator = _integer(
            rate_denominator,
            label="clock rate denominator",
            minimum=1,
            maximum=1_000_000,
        )
        self._state_path = state_path
        self._monotonic_ns = monotonic_ns
        self._base_elapsed_ns = 0
        self._base_monotonic_ns = self._read_monotonic()
        self._last_checkpoint_monotonic_ns = self._base_monotonic_ns
        self._writer_condition = threading.Condition()
        self._pending_payload: dict[str, object] | None = None
        self._writer_writing = False
        self._writer_error: Exception | None = None
        self._writer_stopping = False
        self._closed = False
        self._writer_thread: threading.Thread | None = None
        if state_path is not None:
            self._base_elapsed_ns = self._load_state(state_path)
            self._writer_thread = threading.Thread(
                target=self._writer_loop,
                name="matrix-celestial-clock",
                daemon=True,
            )
            self._writer_thread.start()

    def _read_monotonic(self) -> int:
        value = self._monotonic_ns()
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise CelestialEphemerisError("monotonic clock returned an invalid value")
        return value

    def _load_state(self, path: Path) -> int:
        if path.exists() and (path.is_symlink() or not path.is_file()):
            raise CelestialEphemerisError("celestial clock state is not a regular file")
        try:
            raw = json.loads(path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return 0
        except (OSError, UnicodeError, json.JSONDecodeError) as exc:
            raise CelestialEphemerisError(
                f"cannot load celestial clock state: {exc}"
            ) from exc
        expected = {
            "schema",
            "universe_id",
            "reference_epoch_utc",
            "elapsed_tai_ns",
            "rate_numerator",
            "rate_denominator",
        }
        if not isinstance(raw, dict) or set(raw) != expected:
            raise CelestialEphemerisError("celestial clock state schema is invalid")
        if (
            raw.get("schema") != CLOCK_SCHEMA
            or raw.get("universe_id") != self.universe_id
            or raw.get("reference_epoch_utc") != self.reference_epoch_utc
            or raw.get("rate_numerator") != self.rate_numerator
            or raw.get("rate_denominator") != self.rate_denominator
        ):
            raise CelestialEphemerisError("celestial clock state identity does not match")
        return _integer(
            raw.get("elapsed_tai_ns"),
            label="elapsed_tai_ns",
            minimum=-MAX_ABSOLUTE_ELAPSED_NS,
            maximum=MAX_ABSOLUTE_ELAPSED_NS,
        )

    def elapsed_tai_ns(self, now_monotonic_ns: int | None = None) -> int:
        now = self._read_monotonic() if now_monotonic_ns is None else _integer(
            now_monotonic_ns,
            label="now_monotonic_ns",
            minimum=0,
            maximum=(1 << 63) - 1,
        )
        if now < self._base_monotonic_ns:
            raise CelestialEphemerisError("monotonic clock moved backwards")
        scaled = (
            (now - self._base_monotonic_ns) * self.rate_numerator
        ) // self.rate_denominator
        result = self._base_elapsed_ns + scaled
        return _integer(
            result,
            label="elapsed_tai_ns",
            minimum=-MAX_ABSOLUTE_ELAPSED_NS,
            maximum=MAX_ABSOLUTE_ELAPSED_NS,
        )

    def snapshot(self, now_monotonic_ns: int | None = None) -> SimulationTimeSnapshot:
        elapsed = self.elapsed_tai_ns(now_monotonic_ns)
        whole_seconds, remainder_ns = divmod(elapsed, NANOSECONDS_PER_SECOND)
        try:
            scenario_utc_datetime = self._reference_epoch + timedelta(
                seconds=whole_seconds
            )
        except OverflowError as exc:
            raise CelestialEphemerisError(
                "scenario time is outside the supported calendar range"
            ) from exc
        scenario_utc = scenario_utc_datetime.strftime("%Y-%m-%dT%H:%M:%S")
        if remainder_ns:
            scenario_utc += f".{remainder_ns:09d}".rstrip("0")
        scenario_utc += "Z"
        unix_seconds = int(self._reference_epoch.timestamp()) + whole_seconds
        scenario_tai_ns = (
            (unix_seconds + self._tai_minus_utc) * NANOSECONDS_PER_SECOND
            + remainder_ns
        )
        return SimulationTimeSnapshot(
            elapsed_tai_ns=elapsed,
            scenario_tai_ns=scenario_tai_ns,
            scenario_utc=scenario_utc,
            rate_numerator=self.rate_numerator,
            rate_denominator=self.rate_denominator,
            utc_assumption="frozen-tai-minus-utc-at-scenario-epoch",
        )

    def _write_payload(self, payload: dict[str, object]) -> None:
        assert self._state_path is not None
        path = self._state_path
        path.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
        )
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                json.dump(payload, stream, sort_keys=True, separators=(",", ":"))
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary_name, path)
            directory_descriptor = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory_descriptor)
            finally:
                os.close(directory_descriptor)
        except Exception:
            try:
                os.unlink(temporary_name)
            except FileNotFoundError:
                pass
            raise

    def _writer_loop(self) -> None:
        while True:
            with self._writer_condition:
                while self._pending_payload is None and not self._writer_stopping:
                    self._writer_condition.wait()
                if self._pending_payload is None and self._writer_stopping:
                    return
                payload = self._pending_payload
                self._pending_payload = None
                self._writer_writing = True
            assert payload is not None
            try:
                self._write_payload(payload)
            except Exception as exc:
                with self._writer_condition:
                    self._writer_error = exc
                    self._writer_writing = False
                    self._writer_stopping = True
                    self._writer_condition.notify_all()
                return
            with self._writer_condition:
                self._writer_writing = False
                self._writer_condition.notify_all()

    def _raise_writer_error(self) -> None:
        error = self._writer_error
        if error is not None:
            raise CelestialEphemerisError(
                f"celestial clock checkpoint failed: {error}"
            ) from error

    def checkpoint(self, *, force: bool = False) -> bool:
        if self._state_path is None:
            return False
        with self._writer_condition:
            self._raise_writer_error()
            if self._closed:
                raise CelestialEphemerisError("celestial clock is closed")
        now = self._read_monotonic()
        if (
            not force
            and now - self._last_checkpoint_monotonic_ns < NANOSECONDS_PER_SECOND
        ):
            return False
        elapsed = self.elapsed_tai_ns(now)
        payload: dict[str, object] = {
            "schema": CLOCK_SCHEMA,
            "universe_id": self.universe_id,
            "reference_epoch_utc": self.reference_epoch_utc,
            "elapsed_tai_ns": elapsed,
            "rate_numerator": self.rate_numerator,
            "rate_denominator": self.rate_denominator,
        }
        self._base_elapsed_ns = elapsed
        self._base_monotonic_ns = now
        self._last_checkpoint_monotonic_ns = now
        with self._writer_condition:
            self._raise_writer_error()
            self._pending_payload = payload
            self._writer_condition.notify_all()
            if force:
                while (
                    self._pending_payload is not None or self._writer_writing
                ) and self._writer_error is None:
                    self._writer_condition.wait()
                self._raise_writer_error()
        return True

    def close(self) -> None:
        if self._state_path is None or self._closed:
            return
        checkpoint_error: Exception | None = None
        try:
            self.checkpoint(force=True)
        except Exception as exc:
            checkpoint_error = exc
        with self._writer_condition:
            self._writer_stopping = True
            self._closed = True
            self._writer_condition.notify_all()
        thread = self._writer_thread
        if thread is not None:
            thread.join(timeout=5.0)
            if thread.is_alive():
                stop_error = CelestialEphemerisError(
                    "celestial clock writer did not stop"
                )
                if checkpoint_error is not None:
                    raise stop_error from checkpoint_error
                raise stop_error
        if checkpoint_error is not None:
            raise checkpoint_error


__all__ = [
    "ASTRONOMICAL_UNIT_M",
    "ASSET_MANIFEST_SCHEMA",
    "AnalyticalEphemeris",
    "BodyState",
    "CLOCK_SCHEMA",
    "CelestialBodyDefinition",
    "CelestialEphemerisError",
    "EPHEMERIS_ACCURACY",
    "EPHEMERIS_PROVIDER",
    "JPL_EPHEMERIS_ACCURACY",
    "JPL_EPHEMERIS_PROVIDER",
    "JplSpkEphemeris",
    "KeplerOrbit",
    "PersistentSimulationClock",
    "SimulationTimeSnapshot",
    "SurfaceAnchor",
    "UniformRotation",
    "solar_lighting_state",
    "verify_locked_ephemeris_assets",
]
