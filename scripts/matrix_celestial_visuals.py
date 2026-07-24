#!/usr/bin/env python3
"""Versioned, deterministic visual profiles for Matrix celestial lighting.

The profile catalog owns only renderer inputs.  Ephemeris-derived Sun angles
remain authoritative, while MuJoCo gravity, contacts, SONIC observations, and
camera resolution stay outside this module.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re
import sys
from types import MappingProxyType
from typing import Mapping


VISUAL_CATALOG_SCHEMA = "matrix-celestial-visual-profiles/v1"
VISUAL_PROFILE_SCHEMA = "matrix-celestial-visual-profile/v1"
CARLA_RENDERER = "carla-weather-v1"
LOCKED_CARLA_SOURCE = MappingProxyType({
    "project": "carla-simulator/carla",
    "url": "https://github.com/carla-simulator/carla",
    "revision": "d7b45c1e159e6d13296f7a3a4e8b13e6c2d62c18",
    "path": "LibCarla/source/carla/rpc/WeatherParameters.cpp",
    "license": "MIT",
})
DEFAULT_VISUAL_CATALOG_PATH = (
    Path(__file__).resolve().parents[1]
    / "config/universe/celestial-visual-profiles-v1.json"
)
MAX_VISUAL_CATALOG_BYTES = 256 * 1024
MAX_VISUAL_PROFILES = 32

CARLA_WEATHER_FIELDS = (
    "cloudiness",
    "precipitation",
    "precipitation_deposits",
    "wind_intensity",
    "sun_azimuth_angle",
    "sun_altitude_angle",
    "fog_density",
    "fog_distance",
    "fog_falloff",
    "wetness",
    "scattering_intensity",
    "mie_scattering_scale",
    "rayleigh_scattering_scale",
    "dust_storm",
)
CARLA_STATIC_WEATHER_FIELDS = tuple(
    field
    for field in CARLA_WEATHER_FIELDS
    if field not in {"sun_azimuth_angle", "sun_altitude_angle"}
)

_PARAMETER_BOUNDS = {
    "cloudiness": (0.0, 100.0),
    "precipitation": (0.0, 100.0),
    "precipitation_deposits": (0.0, 100.0),
    "wind_intensity": (0.0, 100.0),
    "fog_density": (0.0, 100.0),
    "fog_distance": (0.0, 100_000.0),
    "fog_falloff": (0.0, 10.0),
    "wetness": (0.0, 100.0),
    "scattering_intensity": (0.0, 10.0),
    "mie_scattering_scale": (0.0, 10.0),
    "rayleigh_scattering_scale": (0.0, 10.0),
    "dust_storm": (0.0, 100.0),
}
_SAMPLE_PARAMETER_BOUNDS = {
    **_PARAMETER_BOUNDS,
    "sun_azimuth_angle": (0.0, 360.0),
    "sun_altitude_angle": (-90.0, 90.0),
}
_ID_RE = re.compile(r"[a-z0-9][a-z0-9._-]{0,63}\Z")
_REVISION_RE = re.compile(r"[0-9a-f]{40}\Z")
_SHA256_RE = re.compile(r"[0-9a-f]{64}\Z")


class CelestialVisualError(ValueError):
    """Raised when a visual profile or renderer sample violates its contract."""


def _finite_number(value: object, *, label: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
    ):
        raise CelestialVisualError(f"{label} must be finite")
    return float(value)


def _identifier(value: object, *, label: str) -> str:
    if not isinstance(value, str) or _ID_RE.fullmatch(value) is None:
        raise CelestialVisualError(f"{label} is invalid")
    return value


def _bounded_ascii_text(value: object, *, label: str, maximum: int = 160) -> str:
    if (
        not isinstance(value, str)
        or not value
        or len(value) > maximum
        or any(ord(character) < 0x20 or ord(character) > 0x7E for character in value)
    ):
        raise CelestialVisualError(f"{label} must be bounded printable ASCII")
    return value


def _strict_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise CelestialVisualError(f"duplicate visual catalog field {key!r}")
        result[key] = value
    return result


def _load_strict_json(path: Path) -> dict[str, object]:
    if not path.is_absolute():
        raise CelestialVisualError("visual catalog path must be absolute")
    if path.is_symlink() or not path.is_file():
        raise CelestialVisualError("visual catalog must be a regular non-symlink file")
    try:
        size = path.stat().st_size
        if not 1 <= size <= MAX_VISUAL_CATALOG_BYTES:
            raise CelestialVisualError("visual catalog size is invalid")
        root = json.loads(
            path.read_text(encoding="utf-8"),
            object_pairs_hook=_strict_object,
            parse_constant=lambda token: (_ for _ in ()).throw(
                CelestialVisualError(f"invalid visual catalog constant {token}")
            ),
        )
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise CelestialVisualError(f"cannot load visual catalog: {exc}") from exc
    if not isinstance(root, dict):
        raise CelestialVisualError("visual catalog root must be an object")
    return root


@dataclass(frozen=True)
class VisualSource:
    project: str
    url: str
    revision: str
    path: str
    license: str

    def mapping(self) -> dict[str, str]:
        return {
            "project": self.project,
            "url": self.url,
            "revision": self.revision,
            "path": self.path,
            "license": self.license,
        }


@dataclass(frozen=True)
class CarlaWeatherSample:
    profile_id: str
    profile_sha256: str
    display_name: str
    body_id: str
    atmosphere: str
    renderer: str
    parameters: tuple[tuple[str, float], ...]

    def __post_init__(self) -> None:
        _identifier(self.profile_id, label="visual profile id")
        _bounded_ascii_text(self.display_name, label="visual profile display name")
        _identifier(self.body_id, label="visual profile body id")
        _identifier(self.atmosphere, label="visual profile atmosphere")
        if self.renderer != CARLA_RENDERER:
            raise CelestialVisualError("CARLA weather sample renderer is invalid")
        if _SHA256_RE.fullmatch(self.profile_sha256) is None:
            raise CelestialVisualError("visual profile SHA256 is invalid")
        names = tuple(name for name, _value in self.parameters)
        if names != CARLA_WEATHER_FIELDS:
            raise CelestialVisualError("CARLA weather sample fields are invalid")
        for name, value in self.parameters:
            if not math.isfinite(value):
                raise CelestialVisualError(f"CARLA weather sample {name} is not finite")
            minimum, maximum = _SAMPLE_PARAMETER_BOUNDS[name]
            if not minimum <= value <= maximum:
                raise CelestialVisualError(
                    f"CARLA weather sample {name} is outside its bounds"
                )
            if name == "sun_azimuth_angle" and value >= 360.0:
                raise CelestialVisualError("CARLA Sun azimuth must be below 360")

    def parameters_mapping(self) -> dict[str, float]:
        return dict(self.parameters)

    def profile_mapping(self) -> dict[str, object]:
        return {
            "schema": VISUAL_PROFILE_SCHEMA,
            "id": self.profile_id,
            "sha256": self.profile_sha256,
            "display_name": self.display_name,
            "body_id": self.body_id,
            "atmosphere": self.atmosphere,
            "renderer": self.renderer,
            "weather_parameters": self.parameters_mapping(),
        }


@dataclass(frozen=True)
class CelestialVisualProfile:
    profile_id: str
    display_name: str
    body_id: str
    atmosphere: str
    renderer: str
    basis: str
    derivation: str
    static_parameters: tuple[tuple[str, float], ...]
    profile_sha256: str

    def sample(self, lighting: Mapping[str, object]) -> CarlaWeatherSample:
        body_id = _identifier(lighting.get("body_id"), label="lighting.body_id")
        atmosphere = _identifier(
            lighting.get("atmosphere"), label="lighting.atmosphere"
        )
        if body_id != self.body_id or atmosphere != self.atmosphere:
            raise CelestialVisualError(
                f"visual profile {self.profile_id!r} does not match "
                f"{body_id}/{atmosphere}"
            )
        altitude = _finite_number(
            lighting.get("sun_altitude_deg"), label="lighting.sun_altitude_deg"
        )
        azimuth = _finite_number(
            lighting.get("sun_azimuth_deg"), label="lighting.sun_azimuth_deg"
        )
        if not -90.0 <= altitude <= 90.0:
            raise CelestialVisualError("lighting Sun altitude is outside [-90, 90]")
        if not 0.0 <= azimuth < 360.0:
            raise CelestialVisualError("lighting Sun azimuth is outside [0, 360)")
        static = dict(self.static_parameters)
        parameters = tuple(
            (
                name,
                azimuth
                if name == "sun_azimuth_angle"
                else altitude
                if name == "sun_altitude_angle"
                else static[name],
            )
            for name in CARLA_WEATHER_FIELDS
        )
        return CarlaWeatherSample(
            profile_id=self.profile_id,
            profile_sha256=self.profile_sha256,
            display_name=self.display_name,
            body_id=self.body_id,
            atmosphere=self.atmosphere,
            renderer=self.renderer,
            parameters=parameters,
        )


@dataclass(frozen=True)
class CelestialVisualCatalog:
    source: VisualSource
    default_profiles: tuple[tuple[str, str], ...]
    profiles: tuple[CelestialVisualProfile, ...]

    def __post_init__(self) -> None:
        profile_ids = tuple(profile.profile_id for profile in self.profiles)
        if len(profile_ids) != len(set(profile_ids)):
            raise CelestialVisualError("visual profile ids must be unique")
        defaults = dict(self.default_profiles)
        if len(defaults) != len(self.default_profiles):
            raise CelestialVisualError("visual default body ids must be unique")
        profile_by_id = {profile.profile_id: profile for profile in self.profiles}
        for body_id, profile_id in self.default_profiles:
            profile = profile_by_id.get(profile_id)
            if profile is None or profile.body_id != body_id:
                raise CelestialVisualError(
                    f"default visual profile for {body_id!r} is invalid"
                )

    def profile(self, profile_id: str) -> CelestialVisualProfile:
        for profile in self.profiles:
            if profile.profile_id == profile_id:
                return profile
        raise CelestialVisualError(f"unknown celestial visual profile {profile_id!r}")

    def sample(
        self,
        lighting: Mapping[str, object],
        *,
        profile_id: str = "auto",
    ) -> CarlaWeatherSample:
        body_id = _identifier(lighting.get("body_id"), label="lighting.body_id")
        if profile_id == "auto":
            selected = dict(self.default_profiles).get(body_id)
            if selected is None:
                raise CelestialVisualError(
                    f"no default visual profile exists for body {body_id!r}"
                )
        else:
            selected = _identifier(profile_id, label="visual profile override")
        return self.profile(selected).sample(lighting)


def _parse_source(value: object) -> VisualSource:
    expected = {"project", "url", "revision", "path", "license"}
    if not isinstance(value, dict) or set(value) != expected:
        raise CelestialVisualError("visual source has an invalid schema")
    project = _bounded_ascii_text(value.get("project"), label="source.project")
    url = _bounded_ascii_text(value.get("url"), label="source.url", maximum=512)
    revision = value.get("revision")
    path = _bounded_ascii_text(value.get("path"), label="source.path", maximum=256)
    license_name = _bounded_ascii_text(value.get("license"), label="source.license")
    if not url.startswith("https://github.com/"):
        raise CelestialVisualError("visual source URL must be a GitHub HTTPS URL")
    if not isinstance(revision, str) or _REVISION_RE.fullmatch(revision) is None:
        raise CelestialVisualError("visual source revision must be a full Git commit")
    source = VisualSource(project, url, revision, path, license_name)
    if source.mapping() != LOCKED_CARLA_SOURCE:
        raise CelestialVisualError("visual source does not match locked CARLA 0.9.15")
    return source


def _parse_weather_parameters(value: object, *, label: str) -> tuple[tuple[str, float], ...]:
    if not isinstance(value, dict) or set(value) != set(CARLA_STATIC_WEATHER_FIELDS):
        raise CelestialVisualError(f"{label} has an invalid CARLA weather schema")
    parameters: list[tuple[str, float]] = []
    for name in CARLA_STATIC_WEATHER_FIELDS:
        number = _finite_number(value.get(name), label=f"{label}.{name}")
        minimum, maximum = _PARAMETER_BOUNDS[name]
        if not minimum <= number <= maximum:
            raise CelestialVisualError(
                f"{label}.{name} must be in [{minimum:g}, {maximum:g}]"
            )
        parameters.append((name, number))
    return tuple(parameters)


def load_visual_catalog(
    path: Path = DEFAULT_VISUAL_CATALOG_PATH,
) -> CelestialVisualCatalog:
    root = _load_strict_json(path)
    expected = {"schema", "source", "default_profiles", "profiles"}
    if set(root) != expected or root.get("schema") != VISUAL_CATALOG_SCHEMA:
        raise CelestialVisualError("celestial visual catalog version is unsupported")
    source = _parse_source(root.get("source"))
    raw_defaults = root.get("default_profiles")
    if not isinstance(raw_defaults, dict) or not raw_defaults:
        raise CelestialVisualError("visual default profiles must be a non-empty object")
    defaults = tuple(
        (
            _identifier(body_id, label="default profile body id"),
            _identifier(profile_id, label=f"default profile for {body_id}"),
        )
        for body_id, profile_id in raw_defaults.items()
    )
    raw_profiles = root.get("profiles")
    if (
        not isinstance(raw_profiles, list)
        or not 1 <= len(raw_profiles) <= MAX_VISUAL_PROFILES
    ):
        raise CelestialVisualError("visual profiles collection is invalid")
    profiles: list[CelestialVisualProfile] = []
    expected_profile = {
        "id",
        "display_name",
        "body_id",
        "atmosphere",
        "renderer",
        "basis",
        "derivation",
        "weather_parameters",
    }
    for index, value in enumerate(raw_profiles):
        label = f"profiles[{index}]"
        if not isinstance(value, dict) or set(value) != expected_profile:
            raise CelestialVisualError(f"{label} has an invalid schema")
        renderer = _identifier(value.get("renderer"), label=f"{label}.renderer")
        if renderer != CARLA_RENDERER:
            raise CelestialVisualError(f"{label}.renderer is unsupported")
        derivation = _identifier(
            value.get("derivation"), label=f"{label}.derivation"
        )
        if derivation not in {"upstream-preset", "physical-adapter"}:
            raise CelestialVisualError(f"{label}.derivation is unsupported")
        canonical = {
            "schema": VISUAL_PROFILE_SCHEMA,
            "source": source.mapping(),
            **value,
        }
        profile_sha256 = hashlib.sha256(
            json.dumps(
                canonical,
                sort_keys=True,
                separators=(",", ":"),
                ensure_ascii=True,
                allow_nan=False,
            ).encode("ascii")
        ).hexdigest()
        profiles.append(
            CelestialVisualProfile(
                profile_id=_identifier(value.get("id"), label=f"{label}.id"),
                display_name=_bounded_ascii_text(
                    value.get("display_name"), label=f"{label}.display_name"
                ),
                body_id=_identifier(
                    value.get("body_id"), label=f"{label}.body_id"
                ),
                atmosphere=_identifier(
                    value.get("atmosphere"), label=f"{label}.atmosphere"
                ),
                renderer=renderer,
                basis=_bounded_ascii_text(
                    value.get("basis"), label=f"{label}.basis"
                ),
                derivation=derivation,
                static_parameters=_parse_weather_parameters(
                    value.get("weather_parameters"),
                    label=f"{label}.weather_parameters",
                ),
                profile_sha256=profile_sha256,
            )
        )
    return CelestialVisualCatalog(source, defaults, tuple(profiles))


def _parse_cli_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate deterministic Matrix celestial visual profiles"
    )
    parser.add_argument("command", choices=("validate",))
    parser.add_argument("--catalog", type=Path, default=DEFAULT_VISUAL_CATALOG_PATH)
    parser.add_argument("--profile", default="auto")
    return parser.parse_args()


def main() -> int:
    args = _parse_cli_args()
    try:
        catalog = load_visual_catalog(args.catalog)
        if args.profile != "auto":
            catalog.profile(_identifier(args.profile, label="visual profile override"))
        print(
            json.dumps(
                {
                    "schema": VISUAL_CATALOG_SCHEMA,
                    "source": catalog.source.mapping(),
                    "default_profiles": dict(catalog.default_profiles),
                    "profile_ids": [profile.profile_id for profile in catalog.profiles],
                    "selected_profile": args.profile,
                },
                sort_keys=True,
                separators=(",", ":"),
            )
        )
        return 0
    except CelestialVisualError as exc:
        print(f"matrix-celestial-visuals ERROR {exc}", file=sys.stderr)
        return 2


__all__ = [
    "CARLA_RENDERER",
    "CARLA_STATIC_WEATHER_FIELDS",
    "CARLA_WEATHER_FIELDS",
    "CarlaWeatherSample",
    "CelestialVisualCatalog",
    "CelestialVisualError",
    "CelestialVisualProfile",
    "DEFAULT_VISUAL_CATALOG_PATH",
    "LOCKED_CARLA_SOURCE",
    "VISUAL_CATALOG_SCHEMA",
    "VISUAL_PROFILE_SCHEMA",
    "load_visual_catalog",
]


if __name__ == "__main__":
    raise SystemExit(main())
