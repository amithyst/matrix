#!/usr/bin/env python3
"""Strict dynamic celestial catalog and origin-rebased Matrix navigation."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re
import sys
from typing import Mapping

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from matrix_world_state import (
    WorldPose,
    WorldStateError,
    validate_tag,
    validate_world_id,
)
from matrix_celestial_ephemeris import (
    AnalyticalEphemeris,
    CelestialBodyDefinition,
    CelestialEphemerisError,
    EPHEMERIS_ACCURACY,
    EPHEMERIS_PROVIDER,
    JPL_EPHEMERIS_ACCURACY,
    JPL_EPHEMERIS_PROVIDER,
    JplSpkEphemeris,
    KeplerOrbit,
    PersistentSimulationClock,
    SimulationTimeSnapshot,
    SurfaceAnchor,
    UniformRotation,
    solar_lighting_state,
    verify_locked_ephemeris_assets,
)


CATALOG_SCHEMA = "matrix-celestial-universe/v2"
DEFAULT_CATALOG_PATH = (
    Path(__file__).resolve().parents[1] / "config/universe/sol-2080.json"
)
DEFAULT_ASSET_MANIFEST_PATH = (
    Path(__file__).resolve().parents[1]
    / "config/universe/de440s-2080.lock.json"
)
MAX_CELESTIAL_BODIES = 16
MAX_CELESTIAL_DESTINATIONS = 8
_ID_RE = re.compile(r"[a-z0-9][a-z0-9._-]{0,63}\Z")
_ENTITY_ID_RE = re.compile(r"tp-[0-9a-f]{32}\Z")
_FRAME_RE = re.compile(r"[a-z0-9][a-z0-9._-]{0,95}\Z")
_EPOCH_RE = re.compile(r"[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}Z\Z")
_RUNTIME_STATUSES = frozenset({"reference", "active", "planned"})
_NAVIGATION_STATUSES = frozenset(
    {"unavailable", "refreshing", "unknown", "undiscovered", "world_unavailable", "ready"}
)


class CelestialNavigationError(ValueError):
    """Raised when a celestial catalog or runtime navigation payload is invalid."""


def _finite_number(value: object, *, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CelestialNavigationError(f"{label} must be a finite number")
    try:
        result = float(value)
    except (OverflowError, ValueError) as exc:
        raise CelestialNavigationError(f"{label} must be a finite number") from exc
    if not math.isfinite(result):
        raise CelestialNavigationError(f"{label} must be a finite number")
    return result


def _bounded_integer(
    value: object, *, label: str, minimum: int, maximum: int
) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise CelestialNavigationError(f"{label} must be an integer")
    if not minimum <= value <= maximum:
        raise CelestialNavigationError(f"{label} is outside its allowed range")
    return value


def _bounded_text(value: object, *, label: str, maximum: int = 96) -> str:
    if (
        not isinstance(value, str)
        or not value
        or len(value) > maximum
        or any(ord(character) < 0x20 or ord(character) == 0x7F for character in value)
    ):
        raise CelestialNavigationError(f"{label} must be bounded display text")
    return value


def _safe_id(value: object, *, label: str) -> str:
    if not isinstance(value, str) or _ID_RE.fullmatch(value) is None:
        raise CelestialNavigationError(f"{label} is invalid")
    return value


def _vector3(value: object, *, label: str) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise CelestialNavigationError(f"{label} must contain three numbers")
    return tuple(
        _finite_number(component, label=f"{label}[{index}]")
        for index, component in enumerate(value)
    )  # type: ignore[return-value]


@dataclass(frozen=True)
class CelestialBody:
    body_id: str
    display_name: str
    naif_id: int
    ellipsoid_radii_m: tuple[float, float, float]
    gravity_m_s2: float
    atmosphere: str
    runtime_status: str
    rotation: UniformRotation
    orbit: KeplerOrbit | None
    barycenter_companion_id: str | None
    primary_to_companion_mass_ratio: float | None

    @property
    def runtime_ready(self) -> bool:
        return self.runtime_status == "active"

    def definition(self) -> CelestialBodyDefinition:
        return CelestialBodyDefinition(
            body_id=self.body_id,
            display_name=self.display_name,
            naif_id=self.naif_id,
            ellipsoid_radii_m=self.ellipsoid_radii_m,
            gravity_m_s2=self.gravity_m_s2,
            atmosphere=self.atmosphere,
            runtime_status=self.runtime_status,
            rotation=self.rotation,
            orbit=self.orbit,
            barycenter_companion_id=self.barycenter_companion_id,
            primary_to_companion_mass_ratio=self.primary_to_companion_mass_ratio,
        )


@dataclass(frozen=True)
class CelestialDestination:
    destination_id: str
    body_id: str
    display_name: str
    teleport_tag: str
    surface_anchor: SurfaceAnchor
    launch_route: "CelestialLaunchRoute | None" = None


@dataclass(frozen=True)
class CelestialLaunchRoute:
    scene_id: int
    world_id: str
    entry_pose: WorldPose
    required_assets: tuple[str, ...]

    def missing_assets(self, project_root: Path) -> tuple[str, ...]:
        """Return catalog-relative assets that are not installed locally."""

        root = Path(project_root)
        return tuple(
            relative
            for relative in self.required_assets
            if not (root / relative).is_file()
        )

    def synthetic_entity_id(self, *, destination_id: str, teleport_tag: str) -> str:
        digest = hashlib.sha256(
            f"{destination_id}\0{teleport_tag}\0{self.scene_id}".encode("utf-8")
        ).hexdigest()
        return f"tp-{digest[:32]}"

    def to_mapping(self, *, destination_id: str, teleport_tag: str) -> dict[str, object]:
        return {
            "schema": "matrix-celestial-launch-route/v1",
            "destination_id": destination_id,
            "teleport_tag": teleport_tag,
            "target_scene_id": self.scene_id,
            "target_world_id": self.world_id,
            "entry_pose": self.entry_pose.to_mapping(),
            "required_assets": list(self.required_assets),
        }


@dataclass(frozen=True)
class TeleportProbe:
    tag: str
    found: bool
    entity_id: str | None = None
    pose: WorldPose | None = None

    def __post_init__(self) -> None:
        try:
            tag = validate_tag(self.tag)
        except WorldStateError as exc:
            raise CelestialNavigationError(str(exc)) from exc
        if type(self.found) is not bool:
            raise CelestialNavigationError("teleport probe found flag is invalid")
        if self.entity_id is not None and (
            not isinstance(self.entity_id, str)
            or _ENTITY_ID_RE.fullmatch(self.entity_id) is None
        ):
            raise CelestialNavigationError("teleport probe entity id is invalid")
        if self.pose is not None and not isinstance(self.pose, WorldPose):
            raise CelestialNavigationError("teleport probe pose is invalid")
        if self.found != (self.entity_id is not None and self.pose is not None):
            raise CelestialNavigationError("teleport probe payload is inconsistent")
        object.__setattr__(self, "tag", tag)


@dataclass(frozen=True)
class CelestialCatalog:
    universe_id: str
    display_name: str
    reference_epoch_utc: str
    time_scale: str
    tai_minus_utc_at_epoch_s: int
    clock_rate_numerator: int
    clock_rate_denominator: int
    frame: str
    ephemeris_provider: str
    ephemeris_accuracy: str
    ephemeris_upgrade_target: str
    origin_rebasing: bool
    simulation_local_bound_m: float
    default_body_id: str
    bodies: tuple[CelestialBody, ...]
    destinations: tuple[CelestialDestination, ...]
    ephemeris: AnalyticalEphemeris

    def body(self, body_id: str) -> CelestialBody:
        for body in self.bodies:
            if body.body_id == body_id:
                return body
        raise CelestialNavigationError(f"unknown celestial body {body_id!r}")

    def destination(self, destination_id: str) -> CelestialDestination:
        for destination in self.destinations:
            if destination.destination_id == destination_id:
                return destination
        raise CelestialNavigationError(
            f"unknown celestial destination {destination_id!r}"
        )

    def list_command(self) -> str:
        tags = " ".join(destination.teleport_tag for destination in self.destinations)
        return f"/teleport list {tags}"

    def teleport_command(self, destination_id: str) -> str:
        destination = self.destination(destination_id)
        return (
            "/tp @s @e[type=matrix:teleport_point,"
            f"tag={destination.teleport_tag},limit=1,sort=nearest]"
        )

    def create_clock(self, state_path: Path | None = None) -> PersistentSimulationClock:
        return PersistentSimulationClock(
            universe_id=self.universe_id,
            reference_epoch_utc=self.reference_epoch_utc,
            tai_minus_utc_at_epoch_s=self.tai_minus_utc_at_epoch_s,
            rate_numerator=self.clock_rate_numerator,
            rate_denominator=self.clock_rate_denominator,
            state_path=state_path,
        )

    def _reference_time(self) -> SimulationTimeSnapshot:
        clock = PersistentSimulationClock(
            universe_id=self.universe_id,
            reference_epoch_utc=self.reference_epoch_utc,
            tai_minus_utc_at_epoch_s=self.tai_minus_utc_at_epoch_s,
            rate_numerator=self.clock_rate_numerator,
            rate_denominator=self.clock_rate_denominator,
            monotonic_ns=lambda: 0,
        )
        return clock.snapshot(0)

    def navigation_mapping(
        self,
        probes: Mapping[str, TeleportProbe],
        *,
        command_available: bool,
        in_flight: bool,
        restart_required: bool,
        outcome_unknown: bool,
        simulation_time: SimulationTimeSnapshot | None = None,
    ) -> dict[str, object]:
        current_time = simulation_time or self._reference_time()
        if (
            current_time.rate_numerator != self.clock_rate_numerator
            or current_time.rate_denominator != self.clock_rate_denominator
        ):
            raise CelestialNavigationError("simulation clock rate does not match catalog")
        definitions = {body.body_id: body.definition() for body in self.bodies}
        try:
            states = self.ephemeris.states(current_time.elapsed_tai_ns)
        except CelestialEphemerisError as exc:
            raise CelestialNavigationError(str(exc)) from exc
        navigation_available = bool(
            command_available and not restart_required and not outcome_unknown
        )
        destinations: list[dict[str, object]] = []
        current_anchor: SurfaceAnchor | None = None
        current_local_position = (0.0, 0.0, 0.0)
        for destination in self.destinations:
            body = self.body(destination.body_id)
            body_state = states[body.body_id]
            probe = probes.get(destination.teleport_tag)
            if not navigation_available:
                status = "unavailable"
            elif body.runtime_status != "active":
                status = "world_unavailable"
            elif probe is None:
                status = "unknown"
            elif not probe.found:
                status = "undiscovered"
            else:
                status = "ready"
            if status not in _NAVIGATION_STATUSES:
                raise AssertionError(status)
            local_position = (
                [probe.pose.x, probe.pose.y, probe.pose.z]
                if probe is not None and probe.pose is not None
                else None
            )
            if local_position is not None and any(
                abs(component) > self.simulation_local_bound_m
                for component in local_position
            ):
                raise CelestialNavigationError(
                    "teleport point exceeds the simulation-local coordinate bound"
                )
            site_body_fixed = destination.surface_anchor.body_fixed_position(
                body.ellipsoid_radii_m
            )
            site_universe_position = body_state.body_fixed_to_inertial(
                site_body_fixed
            )
            universe_position = (
                list(
                    body_state.body_fixed_to_inertial(
                        destination.surface_anchor.local_position_to_body_fixed(
                            body.ellipsoid_radii_m,
                            (
                                probe.pose.x,
                                probe.pose.y,
                                probe.pose.z,
                            ),
                        )
                    )
                )
                if probe is not None and probe.pose is not None
                else None
            )
            if body.body_id == self.default_body_id and current_anchor is None:
                current_anchor = destination.surface_anchor
                if probe is not None and probe.pose is not None:
                    current_local_position = (
                        probe.pose.x,
                        probe.pose.y,
                        probe.pose.z,
                    )
            destinations.append(
                {
                    "id": destination.destination_id,
                    "body_id": body.body_id,
                    "body_name": body.display_name,
                    "display_name": destination.display_name,
                    "teleport_tag": destination.teleport_tag,
                    "runtime_status": body.runtime_status,
                    "status": status,
                    "enabled": bool(
                        status == "ready"
                        and not in_flight
                    ),
                    "surface_coordinates_deg_m": [
                        destination.surface_anchor.latitude_deg,
                        destination.surface_anchor.longitude_deg,
                        destination.surface_anchor.altitude_m,
                    ],
                    "surface_heading_deg": destination.surface_anchor.heading_deg,
                    "local_position_m": local_position,
                    "site_universe_position_m": list(site_universe_position),
                    "universe_position_m": universe_position,
                    "gravity_m_s2": body.gravity_m_s2,
                    "atmosphere": body.atmosphere,
                }
            )
        if current_anchor is None:
            raise CelestialNavigationError(
                "default celestial body has no configured surface destination"
            )
        try:
            lighting = solar_lighting_state(
                observer_body=definitions[self.default_body_id],
                anchor=current_anchor,
                local_position_m=current_local_position,
                bodies=definitions,
                states=states,
            )
        except CelestialEphemerisError as exc:
            raise CelestialNavigationError(str(exc)) from exc
        body_states = []
        sun_center = states["sun"].center_inertial_m
        for body in self.bodies:
            center = states[body.body_id].center_inertial_m
            body_states.append(
                {
                    "id": body.body_id,
                    "display_name": body.display_name,
                    "naif_id": body.naif_id,
                    "runtime_status": body.runtime_status,
                    "center_inertial_m": list(center),
                    "solar_distance_m": math.sqrt(
                        sum(
                            (center[index] - sun_center[index]) ** 2
                            for index in range(3)
                        )
                    ),
                }
            )
        return {
            "version": 2,
            "available": navigation_available,
            "status": (
                "unavailable"
                if not navigation_available
                else ("refreshing" if in_flight else "ready")
            ),
            "universe_id": self.universe_id,
            "display_name": self.display_name,
            "reference_epoch_utc": self.reference_epoch_utc,
            "time_scale": self.time_scale,
            "frame": self.frame,
            "ephemeris": {
                "provider": self.ephemeris_provider,
                "accuracy_class": self.ephemeris_accuracy,
                "upgrade_target": self.ephemeris_upgrade_target,
            },
            "simulation_time": current_time.mapping(),
            "origin_rebasing": self.origin_rebasing,
            "simulation_local_bound_m": self.simulation_local_bound_m,
            "current_body_id": self.default_body_id,
            "bodies": body_states,
            "lighting": lighting,
            "destinations": destinations,
        }


def _load_strict_json(path: Path) -> dict[str, object]:
    try:
        value = json.loads(
            path.read_text(encoding="utf-8"),
            object_pairs_hook=lambda pairs: _strict_object(pairs),
            parse_constant=lambda token: (_ for _ in ()).throw(
                CelestialNavigationError(f"invalid catalog constant {token}")
            ),
        )
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise CelestialNavigationError(f"cannot load celestial catalog: {exc}") from exc
    if not isinstance(value, dict):
        raise CelestialNavigationError("celestial catalog root must be an object")
    return value


def _strict_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise CelestialNavigationError(f"duplicate celestial catalog field {key!r}")
        result[key] = value
    return result


def _parse_rotation(value: object, *, label: str) -> UniformRotation:
    expected = {
        "model",
        "pole_right_ascension_deg",
        "pole_right_ascension_rate_deg_per_century",
        "pole_declination_deg",
        "pole_declination_rate_deg_per_century",
        "prime_meridian_deg",
        "spin_rate_deg_per_day",
    }
    if not isinstance(value, dict) or set(value) != expected:
        raise CelestialNavigationError(f"{label} has an invalid schema")
    if value.get("model") != "iau-uniform-v1":
        raise CelestialNavigationError(f"{label}.model is unsupported")
    try:
        return UniformRotation(
            pole_right_ascension_deg=_finite_number(
                value.get("pole_right_ascension_deg"),
                label=f"{label}.pole_right_ascension_deg",
            ),
            pole_right_ascension_rate_deg_per_century=_finite_number(
                value.get("pole_right_ascension_rate_deg_per_century"),
                label=f"{label}.pole_right_ascension_rate_deg_per_century",
            ),
            pole_declination_deg=_finite_number(
                value.get("pole_declination_deg"),
                label=f"{label}.pole_declination_deg",
            ),
            pole_declination_rate_deg_per_century=_finite_number(
                value.get("pole_declination_rate_deg_per_century"),
                label=f"{label}.pole_declination_rate_deg_per_century",
            ),
            prime_meridian_deg=_finite_number(
                value.get("prime_meridian_deg"),
                label=f"{label}.prime_meridian_deg",
            ),
            spin_rate_deg_per_day=_finite_number(
                value.get("spin_rate_deg_per_day"),
                label=f"{label}.spin_rate_deg_per_day",
            ),
        )
    except CelestialEphemerisError as exc:
        raise CelestialNavigationError(str(exc)) from exc


def _parse_orbit(value: object, *, label: str) -> KeplerOrbit | None:
    if value is None:
        return None
    expected = {
        "model",
        "parent_id",
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
    }
    if not isinstance(value, dict) or set(value) != expected:
        raise CelestialNavigationError(f"{label} has an invalid schema")
    if value.get("model") != "jpl-mean-elements-v1":
        raise CelestialNavigationError(f"{label}.model is unsupported")
    try:
        return KeplerOrbit(
            parent_id=_safe_id(value.get("parent_id"), label=f"{label}.parent_id"),
            semi_major_axis_m=_finite_number(
                value.get("semi_major_axis_m"), label=f"{label}.semi_major_axis_m"
            ),
            semi_major_axis_rate_m_per_century=_finite_number(
                value.get("semi_major_axis_rate_m_per_century"),
                label=f"{label}.semi_major_axis_rate_m_per_century",
            ),
            eccentricity=_finite_number(
                value.get("eccentricity"), label=f"{label}.eccentricity"
            ),
            eccentricity_rate_per_century=_finite_number(
                value.get("eccentricity_rate_per_century"),
                label=f"{label}.eccentricity_rate_per_century",
            ),
            inclination_deg=_finite_number(
                value.get("inclination_deg"), label=f"{label}.inclination_deg"
            ),
            inclination_rate_deg_per_century=_finite_number(
                value.get("inclination_rate_deg_per_century"),
                label=f"{label}.inclination_rate_deg_per_century",
            ),
            mean_longitude_deg=_finite_number(
                value.get("mean_longitude_deg"),
                label=f"{label}.mean_longitude_deg",
            ),
            mean_longitude_rate_deg_per_century=_finite_number(
                value.get("mean_longitude_rate_deg_per_century"),
                label=f"{label}.mean_longitude_rate_deg_per_century",
            ),
            longitude_periapsis_deg=_finite_number(
                value.get("longitude_periapsis_deg"),
                label=f"{label}.longitude_periapsis_deg",
            ),
            longitude_periapsis_rate_deg_per_century=_finite_number(
                value.get("longitude_periapsis_rate_deg_per_century"),
                label=f"{label}.longitude_periapsis_rate_deg_per_century",
            ),
            longitude_ascending_node_deg=_finite_number(
                value.get("longitude_ascending_node_deg"),
                label=f"{label}.longitude_ascending_node_deg",
            ),
            longitude_ascending_node_rate_deg_per_century=_finite_number(
                value.get("longitude_ascending_node_rate_deg_per_century"),
                label=f"{label}.longitude_ascending_node_rate_deg_per_century",
            ),
        )
    except CelestialEphemerisError as exc:
        raise CelestialNavigationError(str(exc)) from exc


def _parse_surface_anchor(value: object, *, label: str) -> SurfaceAnchor:
    expected = {"latitude_deg", "longitude_deg", "altitude_m", "heading_deg"}
    if not isinstance(value, dict) or set(value) != expected:
        raise CelestialNavigationError(f"{label} has an invalid schema")
    try:
        return SurfaceAnchor(
            latitude_deg=_finite_number(
                value.get("latitude_deg"), label=f"{label}.latitude_deg"
            ),
            longitude_deg=_finite_number(
                value.get("longitude_deg"), label=f"{label}.longitude_deg"
            ),
            altitude_m=_finite_number(
                value.get("altitude_m"), label=f"{label}.altitude_m"
            ),
            heading_deg=_finite_number(
                value.get("heading_deg"), label=f"{label}.heading_deg"
            ),
        )
    except CelestialEphemerisError as exc:
        raise CelestialNavigationError(str(exc)) from exc


def _parse_launch_route(value: object, *, label: str) -> CelestialLaunchRoute | None:
    if value is None:
        return None
    expected = {
        "schema",
        "target_scene_id",
        "target_world_id",
        "entry_pose",
        "required_assets",
    }
    if not isinstance(value, dict) or set(value) != expected:
        raise CelestialNavigationError(f"{label} has an invalid schema")
    if value.get("schema") != "matrix-celestial-launch-route/v1":
        raise CelestialNavigationError(f"{label}.schema is unsupported")
    scene_id = _bounded_integer(
        value.get("target_scene_id"),
        label=f"{label}.target_scene_id",
        minimum=0,
        maximum=99,
    )
    try:
        world_id = validate_world_id(value.get("target_world_id"))
        entry_pose = WorldPose.from_mapping(
            value.get("entry_pose"),
            label=f"{label}.entry_pose",
        )
    except WorldStateError as exc:
        raise CelestialNavigationError(str(exc)) from exc
    raw_assets = value.get("required_assets")
    if not isinstance(raw_assets, list) or not 1 <= len(raw_assets) <= 8:
        raise CelestialNavigationError(f"{label}.required_assets is invalid")
    assets: list[str] = []
    for index, asset in enumerate(raw_assets):
        relative = _bounded_text(
            asset,
            label=f"{label}.required_assets[{index}]",
            maximum=160,
        )
        asset_path = Path(relative)
        if (
            asset_path.is_absolute()
            or not asset_path.parts
            or any(part in {"", ".", ".."} for part in asset_path.parts)
        ):
            raise CelestialNavigationError(
                f"{label}.required_assets[{index}] must be a safe relative path"
            )
        assets.append(relative)
    if len(assets) != len(set(assets)):
        raise CelestialNavigationError(f"{label}.required_assets must be unique")
    return CelestialLaunchRoute(
        scene_id=scene_id,
        world_id=world_id,
        entry_pose=entry_pose,
        required_assets=tuple(assets),
    )


def load_catalog(
    path: Path = DEFAULT_CATALOG_PATH,
    *,
    de440s_kernel: Path | None = None,
    jplephem_wheel: Path | None = None,
    asset_manifest: Path = DEFAULT_ASSET_MANIFEST_PATH,
) -> CelestialCatalog:
    root = _load_strict_json(path)
    if set(root) != {"schema", "universe", "default_body_id", "bodies", "destinations"}:
        raise CelestialNavigationError("celestial catalog has an invalid schema")
    if root.get("schema") != CATALOG_SCHEMA:
        raise CelestialNavigationError("celestial catalog version is unsupported")
    universe = root.get("universe")
    expected_universe = {
        "id",
        "display_name",
        "reference_epoch_utc",
        "time_scale",
        "tai_minus_utc_at_epoch_s",
        "clock_rate",
        "frame",
        "ephemeris",
        "units",
        "origin_rebasing",
        "simulation_local_bound_m",
    }
    if not isinstance(universe, dict) or set(universe) != expected_universe:
        raise CelestialNavigationError("universe coordinate contract is invalid")
    if universe.get("units") != "m" or universe.get("origin_rebasing") is not True:
        raise CelestialNavigationError(
            "celestial universe requires metre units and origin rebasing"
        )
    if universe.get("time_scale") != "TAI":
        raise CelestialNavigationError("celestial universe requires TAI time")
    epoch = universe.get("reference_epoch_utc")
    if not isinstance(epoch, str) or _EPOCH_RE.fullmatch(epoch) is None:
        raise CelestialNavigationError("reference epoch must be canonical UTC")
    frame = universe.get("frame")
    if not isinstance(frame, str) or _FRAME_RE.fullmatch(frame) is None:
        raise CelestialNavigationError("universe frame is invalid")
    local_bound = _finite_number(
        universe.get("simulation_local_bound_m"),
        label="simulation_local_bound_m",
    )
    if not 1.0 <= local_bound <= 100_000.0:
        raise CelestialNavigationError("simulation local bound must be in [1, 100000]")
    tai_minus_utc = _bounded_integer(
        universe.get("tai_minus_utc_at_epoch_s"),
        label="tai_minus_utc_at_epoch_s",
        minimum=0,
        maximum=1000,
    )
    clock_rate = universe.get("clock_rate")
    if not isinstance(clock_rate, dict) or set(clock_rate) != {
        "numerator",
        "denominator",
    }:
        raise CelestialNavigationError("universe clock rate is invalid")
    rate_numerator = _bounded_integer(
        clock_rate.get("numerator"),
        label="clock_rate.numerator",
        minimum=0,
        maximum=1_000_000,
    )
    rate_denominator = _bounded_integer(
        clock_rate.get("denominator"),
        label="clock_rate.denominator",
        minimum=1,
        maximum=1_000_000,
    )
    ephemeris_contract = universe.get("ephemeris")
    if not isinstance(ephemeris_contract, dict) or set(ephemeris_contract) != {
        "provider",
        "accuracy_class",
        "upgrade_target",
    }:
        raise CelestialNavigationError("universe ephemeris contract is invalid")
    if (
        ephemeris_contract.get("provider") != EPHEMERIS_PROVIDER
        or ephemeris_contract.get("accuracy_class") != EPHEMERIS_ACCURACY
    ):
        raise CelestialNavigationError("universe ephemeris provider is unsupported")
    upgrade_target = _safe_id(
        ephemeris_contract.get("upgrade_target"),
        label="ephemeris.upgrade_target",
    )

    raw_bodies = root.get("bodies")
    if not isinstance(raw_bodies, list) or not 2 <= len(raw_bodies) <= MAX_CELESTIAL_BODIES:
        raise CelestialNavigationError("celestial bodies collection is invalid")
    bodies: list[CelestialBody] = []
    expected_body = {
        "id",
        "display_name",
        "naif_id",
        "ellipsoid_radii_m",
        "gravity_m_s2",
        "atmosphere",
        "runtime_status",
        "rotation",
        "orbit",
        "barycenter_companion_id",
        "primary_to_companion_mass_ratio",
    }
    for index, value in enumerate(raw_bodies):
        if not isinstance(value, dict) or set(value) != expected_body:
            raise CelestialNavigationError(f"bodies[{index}] has an invalid schema")
        body_id = _safe_id(value.get("id"), label=f"bodies[{index}].id")
        runtime_status = value.get("runtime_status")
        if runtime_status not in _RUNTIME_STATUSES:
            raise CelestialNavigationError(f"bodies[{index}].runtime_status is invalid")
        gravity = _finite_number(
            value.get("gravity_m_s2"), label=f"bodies[{index}].gravity_m_s2"
        )
        if not 0.0 < gravity < 1_000.0:
            raise CelestialNavigationError(f"bodies[{index}].gravity_m_s2 is invalid")
        companion_value = value.get("barycenter_companion_id")
        companion_id = (
            _safe_id(companion_value, label=f"bodies[{index}].barycenter_companion_id")
            if companion_value is not None
            else None
        )
        mass_ratio_value = value.get("primary_to_companion_mass_ratio")
        mass_ratio = (
            _finite_number(
                mass_ratio_value,
                label=f"bodies[{index}].primary_to_companion_mass_ratio",
            )
            if mass_ratio_value is not None
            else None
        )
        body = CelestialBody(
            body_id=body_id,
            display_name=_bounded_text(
                value.get("display_name"), label=f"bodies[{index}].display_name"
            ),
            naif_id=_bounded_integer(
                value.get("naif_id"),
                label=f"bodies[{index}].naif_id",
                minimum=0,
                maximum=1_000_000_000,
            ),
            ellipsoid_radii_m=_vector3(
                value.get("ellipsoid_radii_m"),
                label=f"bodies[{index}].ellipsoid_radii_m",
            ),
            gravity_m_s2=gravity,
            atmosphere=_safe_id(
                value.get("atmosphere"), label=f"bodies[{index}].atmosphere"
            ),
            runtime_status=runtime_status,
            rotation=_parse_rotation(
                value.get("rotation"), label=f"bodies[{index}].rotation"
            ),
            orbit=_parse_orbit(
                value.get("orbit"), label=f"bodies[{index}].orbit"
            ),
            barycenter_companion_id=companion_id,
            primary_to_companion_mass_ratio=mass_ratio,
        )
        try:
            body.definition()
        except CelestialEphemerisError as exc:
            raise CelestialNavigationError(str(exc)) from exc
        bodies.append(body)
    body_ids = [body.body_id for body in bodies]
    naif_ids = [body.naif_id for body in bodies]
    if len(body_ids) != len(set(body_ids)):
        raise CelestialNavigationError("celestial body ids must be unique")
    if len(naif_ids) != len(set(naif_ids)):
        raise CelestialNavigationError("celestial NAIF ids must be unique")
    if (
        "sun" not in body_ids
        or next(body for body in bodies if body.body_id == "sun").runtime_status
        != "reference"
    ):
        raise CelestialNavigationError("celestial catalog requires a reference Sun")
    default_body_id = _safe_id(root.get("default_body_id"), label="default_body_id")
    if default_body_id not in body_ids:
        raise CelestialNavigationError("default celestial body does not exist")
    if next(body for body in bodies if body.body_id == default_body_id).runtime_status != "active":
        raise CelestialNavigationError("default celestial body must be active")
    if (de440s_kernel is None) != (jplephem_wheel is None):
        raise CelestialNavigationError(
            "DE440s kernel and jplephem wheel must be configured together"
        )
    ephemeris_provider = EPHEMERIS_PROVIDER
    ephemeris_accuracy = EPHEMERIS_ACCURACY
    try:
        if de440s_kernel is None:
            ephemeris = AnalyticalEphemeris(
                tuple(body.definition() for body in bodies),
                reference_epoch_utc=epoch,
                tai_minus_utc_at_epoch_s=tai_minus_utc,
            )
        else:
            assert jplephem_wheel is not None
            verify_locked_ephemeris_assets(
                asset_manifest,
                kernel_path=de440s_kernel,
                jplephem_wheel=jplephem_wheel,
            )
            ephemeris = JplSpkEphemeris(
                tuple(body.definition() for body in bodies),
                reference_epoch_utc=epoch,
                tai_minus_utc_at_epoch_s=tai_minus_utc,
                kernel_path=de440s_kernel,
                jplephem_wheel=jplephem_wheel,
            )
            ephemeris_provider = JPL_EPHEMERIS_PROVIDER
            ephemeris_accuracy = JPL_EPHEMERIS_ACCURACY
        ephemeris.states(0)
    except CelestialEphemerisError as exc:
        raise CelestialNavigationError(str(exc)) from exc

    raw_destinations = root.get("destinations")
    if (
        not isinstance(raw_destinations, list)
        or not 1 <= len(raw_destinations) <= MAX_CELESTIAL_DESTINATIONS
    ):
        raise CelestialNavigationError("celestial destinations collection is invalid")
    destinations: list[CelestialDestination] = []
    base_destination_fields = {
        "id",
        "body_id",
        "display_name",
        "teleport_tag",
        "surface_anchor",
    }
    routed_destination_fields = base_destination_fields | {"launch_route"}
    for index, value in enumerate(raw_destinations):
        fields = frozenset(value) if isinstance(value, dict) else frozenset()
        if not isinstance(value, dict) or fields not in {
            frozenset(base_destination_fields),
            frozenset(routed_destination_fields),
        }:
            raise CelestialNavigationError(
                f"destinations[{index}] has an invalid schema"
            )
        if fields == base_destination_fields:
            launch_route = None
        else:
            launch_route = _parse_launch_route(
                value.get("launch_route"),
                label=f"destinations[{index}].launch_route",
            )
        body_id = _safe_id(
            value.get("body_id"), label=f"destinations[{index}].body_id"
        )
        if body_id not in body_ids:
            raise CelestialNavigationError(
                f"destinations[{index}] references an unknown body"
            )
        if next(body for body in bodies if body.body_id == body_id).runtime_status == "reference":
            raise CelestialNavigationError(
                f"destinations[{index}] cannot target a reference body"
            )
        try:
            teleport_tag = validate_tag(value.get("teleport_tag"))
        except WorldStateError as exc:
            raise CelestialNavigationError(str(exc)) from exc
        destinations.append(
            CelestialDestination(
                destination_id=_safe_id(
                    value.get("id"), label=f"destinations[{index}].id"
                ),
                body_id=body_id,
                display_name=_bounded_text(
                    value.get("display_name"),
                    label=f"destinations[{index}].display_name",
                ),
                teleport_tag=teleport_tag,
                surface_anchor=_parse_surface_anchor(
                    value.get("surface_anchor"),
                    label=f"destinations[{index}].surface_anchor",
                ),
                launch_route=launch_route,
            )
        )
    destination_ids = [destination.destination_id for destination in destinations]
    destination_tags = [destination.teleport_tag for destination in destinations]
    if len(destination_ids) != len(set(destination_ids)):
        raise CelestialNavigationError("celestial destination ids must be unique")
    if len(destination_tags) != len(set(destination_tags)):
        raise CelestialNavigationError("celestial destination tags must be unique")
    if not any(destination.body_id == default_body_id for destination in destinations):
        raise CelestialNavigationError("default celestial body requires a destination")
    return CelestialCatalog(
        universe_id=_safe_id(universe.get("id"), label="universe.id"),
        display_name=_bounded_text(
            universe.get("display_name"), label="universe.display_name"
        ),
        reference_epoch_utc=epoch,
        time_scale="TAI",
        tai_minus_utc_at_epoch_s=tai_minus_utc,
        clock_rate_numerator=rate_numerator,
        clock_rate_denominator=rate_denominator,
        frame=frame,
        ephemeris_provider=ephemeris_provider,
        ephemeris_accuracy=ephemeris_accuracy,
        ephemeris_upgrade_target=upgrade_target,
        origin_rebasing=True,
        simulation_local_bound_m=local_bound,
        default_body_id=default_body_id,
        bodies=tuple(bodies),
        destinations=tuple(destinations),
        ephemeris=ephemeris,
    )


def probes_from_response(
    value: object, *, catalog: CelestialCatalog
) -> dict[str, TeleportProbe]:
    if not isinstance(value, dict) or set(value) != {
        "world_id",
        "teleport_points",
    }:
        raise CelestialNavigationError("teleport-list response has an invalid schema")
    try:
        validate_world_id(value.get("world_id"))
    except WorldStateError as exc:
        raise CelestialNavigationError("teleport-list world id is invalid") from exc
    raw_points = value.get("teleport_points")
    if not isinstance(raw_points, list) or len(raw_points) != len(catalog.destinations):
        raise CelestialNavigationError("teleport-list result count is invalid")
    expected_tags = [destination.teleport_tag for destination in catalog.destinations]
    probes: dict[str, TeleportProbe] = {}
    for index, raw in enumerate(raw_points):
        if not isinstance(raw, dict):
            raise CelestialNavigationError(f"teleport_points[{index}] is invalid")
        tag = raw.get("tag")
        if tag != expected_tags[index] or tag in probes:
            raise CelestialNavigationError("teleport-list tags do not match the catalog")
        found = raw.get("found")
        if found is False and set(raw) == {"tag", "found"}:
            probe = TeleportProbe(tag=tag, found=False)
        elif found is True and set(raw) == {
            "tag",
            "found",
            "entity_id",
            "position",
            "yaw_rad",
        }:
            position = raw.get("position")
            if not isinstance(position, list) or len(position) != 3:
                raise CelestialNavigationError("teleport-list position is invalid")
            try:
                coordinates = tuple(
                    _finite_number(
                        component,
                        label=f"teleport-list position[{axis}]",
                    )
                    for axis, component in enumerate(position)
                )
                yaw_rad = _finite_number(
                    raw.get("yaw_rad"),
                    label="teleport-list yaw_rad",
                )
                pose = WorldPose(
                    coordinates[0],
                    coordinates[1],
                    coordinates[2],
                    yaw_rad,
                )
            except (CelestialNavigationError, WorldStateError) as exc:
                raise CelestialNavigationError(str(exc)) from exc
            entity_id = raw.get("entity_id")
            probe = TeleportProbe(
                tag=tag,
                found=True,
                entity_id=entity_id,
                pose=pose,
            )
        else:
            raise CelestialNavigationError(
                f"teleport_points[{index}] has an invalid schema"
            )
        probes[tag] = probe
    return probes


def _parse_cli_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate a locked Matrix celestial provider"
    )
    parser.add_argument("command", choices=("validate",))
    parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG_PATH)
    parser.add_argument(
        "--asset-manifest",
        type=Path,
        default=DEFAULT_ASSET_MANIFEST_PATH,
    )
    parser.add_argument("--de440s-kernel", type=Path)
    parser.add_argument("--jplephem-wheel", type=Path)
    return parser.parse_args()


def main() -> int:
    args = _parse_cli_args()
    try:
        catalog = load_catalog(
            args.catalog,
            de440s_kernel=args.de440s_kernel,
            jplephem_wheel=args.jplephem_wheel,
            asset_manifest=args.asset_manifest,
        )
        states = catalog.ephemeris.states(0)
        output = {
            "schema": CATALOG_SCHEMA,
            "universe_id": catalog.universe_id,
            "frame": catalog.frame,
            "reference_epoch_utc": catalog.reference_epoch_utc,
            "ephemeris_provider": catalog.ephemeris_provider,
            "ephemeris_accuracy": catalog.ephemeris_accuracy,
            "body_ids": list(states),
        }
        print(json.dumps(output, sort_keys=True, separators=(",", ":")))
        catalog.ephemeris.close()
        return 0
    except CelestialNavigationError as exc:
        print(f"matrix-celestial-navigation ERROR {exc}", file=sys.stderr)
        return 2


__all__ = [
    "CATALOG_SCHEMA",
    "CelestialCatalog",
    "CelestialDestination",
    "CelestialLaunchRoute",
    "CelestialNavigationError",
    "DEFAULT_CATALOG_PATH",
    "DEFAULT_ASSET_MANIFEST_PATH",
    "TeleportProbe",
    "load_catalog",
    "probes_from_response",
]


if __name__ == "__main__":
    raise SystemExit(main())
