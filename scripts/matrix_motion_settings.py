#!/usr/bin/env python3
"""Strict, host-scoped persistence for Matrix keyboard motion speeds.

The runtime is expected to be the sole writer of this file.  UI, typed-command,
and external-API adapters can all use :class:`MotionSettingsStore` so validation,
compare-and-swap semantics, persistence, and revision handling stay identical.
This module intentionally uses only the Python standard library.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import json
import math
import os
from pathlib import Path
import re
import tempfile
import threading
from typing import Callable, Mapping


SETTINGS_VERSION = 1
MAX_REVISION = (2**63) - 1

GEAR_SLOW = "slow"
GEAR_WALK = "walk"
GEAR_RUN = "run"
GEARS = (GEAR_SLOW, GEAR_WALK, GEAR_RUN)

SPEED_FIELD = "speed_mps"
DOUBLE_TAP_SPEED_FIELD = "double_tap_speed_mps"
SPEED_FIELDS = (SPEED_FIELD, DOUBLE_TAP_SPEED_FIELD)

GEAR_SPEED_RANGES_MPS: Mapping[str, tuple[float, float]] = {
    GEAR_SLOW: (0.10, 0.80),
    GEAR_WALK: (0.80, 2.50),
    GEAR_RUN: (2.50, 7.50),
}

DEFAULT_GEAR_SPEEDS_MPS: Mapping[str, tuple[float, float]] = {
    GEAR_SLOW: (0.10, 0.20),
    GEAR_WALK: (0.80, 1.00),
    GEAR_RUN: (2.50, 2.75),
}

GEAR_STEP_MPS: Mapping[str, float] = {
    GEAR_SLOW: 0.05,
    GEAR_WALK: 0.10,
    GEAR_RUN: 0.25,
}

_PATH_PREFIX = "control.motion.gears"
MOTION_SETTING_PATHS = frozenset(
    f"{_PATH_PREFIX}.{gear}.{field}"
    for gear in GEARS
    for field in SPEED_FIELDS
)
_PATH_PARTS: Mapping[str, tuple[str, str]] = {
    f"{_PATH_PREFIX}.{gear}.{field}": (gear, field)
    for gear in GEARS
    for field in SPEED_FIELDS
}
_FIELD_NAMES: Mapping[tuple[str, str], str] = {
    (GEAR_SLOW, SPEED_FIELD): "slow_speed_mps",
    (GEAR_SLOW, DOUBLE_TAP_SPEED_FIELD): "slow_double_tap_speed_mps",
    (GEAR_WALK, SPEED_FIELD): "walk_speed_mps",
    (GEAR_WALK, DOUBLE_TAP_SPEED_FIELD): "walk_double_tap_speed_mps",
    (GEAR_RUN, SPEED_FIELD): "run_speed_mps",
    (GEAR_RUN, DOUBLE_TAP_SPEED_FIELD): "run_double_tap_speed_mps",
}
_PROFILE_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,63}\Z")


class MotionSettingsError(ValueError):
    """A typed validation or compare-and-swap error."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


class MotionSettingsPersistenceError(RuntimeError):
    """The candidate was valid but could not be durably stored."""

    code = "E_DATA_PERSIST"

    def __init__(self, message: str) -> None:
        super().__init__(message)
        self.message = message


def canonical_host_profile(value: object) -> str:
    """Return one bounded path-safe host profile."""

    if not isinstance(value, str) or _PROFILE_RE.fullmatch(value) is None:
        raise MotionSettingsError(
            "E_HOST_PROFILE",
            "host profile must use 1-64 ASCII letters, digits, dot, underscore, or dash",
        )
    return value


def default_settings_file(
    profile: object | None = None,
    *,
    config_home: Path | str | None = None,
) -> Path:
    """Return ``~/.config/matrix/hosts/<profile>/motion-control.json``.

    ``MATRIX_HOST_PROFILE`` supplies the profile when the caller omits it.
    ``XDG_CONFIG_HOME`` is honored in the usual way and is injectable for tests.
    """

    selected = os.environ.get("MATRIX_HOST_PROFILE") if profile is None else profile
    if selected is None:
        raise MotionSettingsError(
            "E_HOST_PROFILE", "host profile is required for motion settings"
        )
    host_profile = canonical_host_profile(selected)
    if config_home is None:
        configured = os.environ.get("XDG_CONFIG_HOME")
        root = Path(configured) if configured else Path.home() / ".config"
    else:
        root = Path(config_home)
    return root.expanduser() / "matrix" / "hosts" / host_profile / "motion-control.json"


def _finite_speed(value: object, *, name: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
    ):
        raise MotionSettingsError(
            "E_DATA_TYPE", f"{name} must be a finite JSON number"
        )
    return float(value)


def _revision(value: object) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not 0 <= value <= MAX_REVISION
    ):
        raise MotionSettingsError(
            "E_DATA_REVISION", f"revision must be an integer in [0, {MAX_REVISION}]"
        )
    return value


def _path_parts(path: object) -> tuple[str, str]:
    if not isinstance(path, str) or path not in _PATH_PARTS:
        raise MotionSettingsError(
            "E_DATA_PATH_UNKNOWN", f"unsupported motion settings path: {path!r}"
        )
    return _PATH_PARTS[path]


@dataclass(frozen=True)
class MotionSettings:
    """One validated, revisioned snapshot of all keyboard motion speeds."""

    revision: int = 0
    slow_speed_mps: float = DEFAULT_GEAR_SPEEDS_MPS[GEAR_SLOW][0]
    slow_double_tap_speed_mps: float = DEFAULT_GEAR_SPEEDS_MPS[GEAR_SLOW][1]
    walk_speed_mps: float = DEFAULT_GEAR_SPEEDS_MPS[GEAR_WALK][0]
    walk_double_tap_speed_mps: float = DEFAULT_GEAR_SPEEDS_MPS[GEAR_WALK][1]
    run_speed_mps: float = DEFAULT_GEAR_SPEEDS_MPS[GEAR_RUN][0]
    run_double_tap_speed_mps: float = DEFAULT_GEAR_SPEEDS_MPS[GEAR_RUN][1]

    def __post_init__(self) -> None:
        object.__setattr__(self, "revision", _revision(self.revision))
        for gear in GEARS:
            minimum, maximum = GEAR_SPEED_RANGES_MPS[gear]
            base_name = _FIELD_NAMES[(gear, SPEED_FIELD)]
            boost_name = _FIELD_NAMES[(gear, DOUBLE_TAP_SPEED_FIELD)]
            base = _finite_speed(getattr(self, base_name), name=base_name)
            boost = _finite_speed(getattr(self, boost_name), name=boost_name)
            if not minimum <= base <= maximum:
                raise MotionSettingsError(
                    "E_DATA_RANGE",
                    f"{base_name} must be in [{minimum:.2f}, {maximum:.2f}]",
                )
            if not minimum <= boost <= maximum:
                raise MotionSettingsError(
                    "E_DATA_RANGE",
                    f"{boost_name} must be in [{minimum:.2f}, {maximum:.2f}]",
                )
            if boost <= base:
                raise MotionSettingsError(
                    "E_DATA_CONSTRAINT",
                    f"{boost_name} must be greater than {base_name}",
                )
            object.__setattr__(self, base_name, base)
            object.__setattr__(self, boost_name, boost)

    def value_for_path(self, path: object) -> float:
        gear, field = _path_parts(path)
        return getattr(self, _FIELD_NAMES[(gear, field)])

    def with_value(
        self,
        path: object,
        value: object,
        *,
        revision: int | None = None,
    ) -> "MotionSettings":
        gear, field = _path_parts(path)
        field_name = _FIELD_NAMES[(gear, field)]
        speed = _finite_speed(value, name=field_name)
        next_revision = self.revision if revision is None else _revision(revision)
        return replace(self, revision=next_revision, **{field_name: speed})

    def to_mapping(self) -> dict[str, object]:
        return {
            "version": SETTINGS_VERSION,
            "revision": self.revision,
            "gears": {
                gear: {
                    SPEED_FIELD: getattr(self, _FIELD_NAMES[(gear, SPEED_FIELD)]),
                    DOUBLE_TAP_SPEED_FIELD: getattr(
                        self, _FIELD_NAMES[(gear, DOUBLE_TAP_SPEED_FIELD)]
                    ),
                }
                for gear in GEARS
            },
        }

    @classmethod
    def from_mapping(cls, value: object) -> "MotionSettings":
        if not isinstance(value, dict) or set(value) != {
            "version",
            "revision",
            "gears",
        }:
            raise MotionSettingsError(
                "E_DATA_SCHEMA", "motion settings must contain version/revision/gears"
            )
        if type(value.get("version")) is not int or value.get("version") != SETTINGS_VERSION:
            raise MotionSettingsError(
                "E_DATA_VERSION", f"motion settings version must be {SETTINGS_VERSION}"
            )
        gears = value.get("gears")
        if not isinstance(gears, dict) or set(gears) != set(GEARS):
            raise MotionSettingsError(
                "E_DATA_SCHEMA", "motion settings must define exactly slow/walk/run"
            )
        fields: dict[str, object] = {"revision": value.get("revision")}
        for gear in GEARS:
            entry = gears.get(gear)
            if not isinstance(entry, dict) or set(entry) != set(SPEED_FIELDS):
                raise MotionSettingsError(
                    "E_DATA_SCHEMA",
                    f"motion settings gear {gear!r} has an invalid schema",
                )
            for field in SPEED_FIELDS:
                fields[_FIELD_NAMES[(gear, field)]] = entry.get(field)
        return cls(**fields)


@dataclass(frozen=True)
class MotionSettingsLoad:
    settings: MotionSettings
    status: str
    error: str | None = None


def _strict_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise MotionSettingsError(
                "E_DATA_SCHEMA", f"duplicate motion settings field {key!r}"
            )
        result[key] = value
    return result


def load_settings(path: Path) -> MotionSettingsLoad:
    """Load one strict snapshot; missing or invalid state safely uses defaults."""

    try:
        text = path.read_text(encoding="utf-8")
    except FileNotFoundError:
        return MotionSettingsLoad(MotionSettings(), "missing")
    except (OSError, UnicodeError) as exc:
        return MotionSettingsLoad(
            MotionSettings(), "invalid", f"cannot read motion settings: {exc}"
        )
    try:
        value = json.loads(
            text,
            object_pairs_hook=_strict_object,
            parse_constant=lambda token: (_ for _ in ()).throw(
                MotionSettingsError(
                    "E_DATA_TYPE", f"invalid JSON numeric constant {token}"
                )
            ),
        )
        settings = MotionSettings.from_mapping(value)
    except (json.JSONDecodeError, MotionSettingsError, TypeError, ValueError) as exc:
        return MotionSettingsLoad(
            MotionSettings(), "invalid", f"invalid motion settings: {exc}"
        )
    return MotionSettingsLoad(settings, "loaded")


def atomic_save_settings(path: Path, settings: MotionSettings) -> None:
    """Atomically replace one private settings file and fsync its directory."""

    if not isinstance(path, Path) or not path.is_absolute():
        raise ValueError("motion settings path must be an absolute pathlib.Path")
    if not isinstance(settings, MotionSettings):
        raise TypeError("settings must be MotionSettings")
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            delete=False,
        ) as stream:
            temporary = Path(stream.name)
            os.fchmod(stream.fileno(), 0o600)
            json.dump(
                settings.to_mapping(),
                stream,
                indent=2,
                sort_keys=True,
                allow_nan=False,
            )
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        temporary = None
        directory_fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        if temporary is not None:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass


def _step_presets(gear: str) -> tuple[float, ...]:
    minimum, maximum = GEAR_SPEED_RANGES_MPS[gear]
    step = GEAR_STEP_MPS[gear]
    count = int(round((maximum - minimum) / step))
    return tuple(round(minimum + index * step, 10) for index in range(count + 1))


_GEAR_STEP_PRESETS: Mapping[str, tuple[float, ...]] = {
    gear: _step_presets(gear) for gear in GEARS
}


def step_motion_speed(
    settings: MotionSettings,
    path: object,
    direction: int,
) -> float:
    """Return the adjacent panel preset without violating base/boost ordering."""

    if not isinstance(settings, MotionSettings):
        raise TypeError("settings must be MotionSettings")
    if isinstance(direction, bool) or type(direction) is not int or direction not in {-1, 1}:
        raise MotionSettingsError(
            "E_DATA_STEP", "motion speed step direction must be -1 or 1"
        )
    gear, field = _path_parts(path)
    current = settings.value_for_path(path)
    presets = _GEAR_STEP_PRESETS[gear]
    if direction > 0:
        candidates = tuple(value for value in presets if value > current + 1e-12)
        result = candidates[0] if candidates else presets[-1]
    else:
        candidates = tuple(value for value in presets if value < current - 1e-12)
        result = candidates[-1] if candidates else presets[0]

    if field == SPEED_FIELD:
        boost_path = f"{_PATH_PREFIX}.{gear}.{DOUBLE_TAP_SPEED_FIELD}"
        boost = settings.value_for_path(boost_path)
        allowed = tuple(value for value in presets if value < boost - 1e-12)
        if not allowed:
            return current
        result = min(result, allowed[-1])
    else:
        base_path = f"{_PATH_PREFIX}.{gear}.{SPEED_FIELD}"
        base = settings.value_for_path(base_path)
        allowed = tuple(value for value in presets if value > base + 1e-12)
        if not allowed:
            return current
        result = max(result, allowed[0])
    return result


@dataclass(frozen=True)
class MotionSettingsModification:
    settings: MotionSettings
    path: str
    previous_value: float
    value: float
    changed: bool


SettingsSaver = Callable[[Path, MotionSettings], None]


class MotionSettingsStore:
    """Serialize validated CAS updates through one in-process settings owner."""

    def __init__(
        self,
        path: Path,
        *,
        initial: MotionSettings | None = None,
        fallback: MotionSettings | None = None,
        saver: SettingsSaver = atomic_save_settings,
    ) -> None:
        if not isinstance(path, Path) or not path.is_absolute():
            raise ValueError("motion settings store path must be an absolute pathlib.Path")
        if not callable(saver):
            raise TypeError("motion settings saver must be callable")
        if initial is not None and fallback is not None:
            raise ValueError("initial and fallback motion settings are mutually exclusive")
        if initial is not None and not isinstance(initial, MotionSettings):
            raise TypeError("initial settings must be MotionSettings")
        if fallback is not None and not isinstance(fallback, MotionSettings):
            raise TypeError("fallback settings must be MotionSettings")
        loaded = load_settings(path) if initial is None else None
        self.path = path
        self._settings = (
            initial
            if initial is not None
            else (
                loaded.settings
                if loaded.status == "loaded" or fallback is None
                else fallback
            )
        )
        self.load_status = loaded.status if loaded is not None else "provided"
        self.load_error = loaded.error if loaded is not None else None
        self._saver = saver
        self._lock = threading.RLock()

    @property
    def settings(self) -> MotionSettings:
        with self._lock:
            return self._settings

    def modify(
        self,
        path: object,
        value: object,
        *,
        expected_revision: int | None = None,
    ) -> MotionSettingsModification:
        canonical_path = path if isinstance(path, str) else path
        _path_parts(canonical_path)
        with self._lock:
            current = self._settings
            if expected_revision is not None:
                expected = _revision(expected_revision)
                if expected != current.revision:
                    raise MotionSettingsError(
                        "E_DATA_REVISION_CONFLICT",
                        f"expected revision {expected}, current revision is {current.revision}",
                    )
            previous = current.value_for_path(canonical_path)
            # with_value performs type/range/cross-field validation before any
            # filesystem mutation.  An unchanged set is idempotent and does not
            # consume a revision or rewrite the file.
            validated = current.with_value(canonical_path, value)
            replacement_value = validated.value_for_path(canonical_path)
            if replacement_value == previous:
                return MotionSettingsModification(
                    current, canonical_path, previous, previous, False
                )
            if current.revision >= MAX_REVISION:
                raise MotionSettingsError(
                    "E_DATA_REVISION", "motion settings revision is exhausted"
                )
            candidate = current.with_value(
                canonical_path,
                replacement_value,
                revision=current.revision + 1,
            )
            try:
                self._saver(self.path, candidate)
            except (OSError, UnicodeError, ValueError) as exc:
                raise MotionSettingsPersistenceError(
                    f"could not persist motion settings: {exc}"
                ) from exc
            self._settings = candidate
            self.load_status = "saved"
            self.load_error = None
            return MotionSettingsModification(
                candidate,
                canonical_path,
                previous,
                replacement_value,
                True,
            )

    def step(
        self,
        path: object,
        direction: int,
        *,
        expected_revision: int | None = None,
    ) -> MotionSettingsModification:
        with self._lock:
            next_value = step_motion_speed(self._settings, path, direction)
            return self.modify(
                path,
                next_value,
                expected_revision=expected_revision,
            )

    def mapping(self) -> dict[str, object]:
        with self._lock:
            return {
                "settings_file": os.fspath(self.path),
                "load_status": self.load_status,
                "load_error": self.load_error,
                "settings": self._settings.to_mapping(),
            }


__all__ = [
    "DEFAULT_GEAR_SPEEDS_MPS",
    "DOUBLE_TAP_SPEED_FIELD",
    "GEARS",
    "GEAR_SPEED_RANGES_MPS",
    "GEAR_STEP_MPS",
    "MAX_REVISION",
    "MOTION_SETTING_PATHS",
    "MotionSettings",
    "MotionSettingsError",
    "MotionSettingsLoad",
    "MotionSettingsModification",
    "MotionSettingsPersistenceError",
    "MotionSettingsStore",
    "SETTINGS_VERSION",
    "SPEED_FIELD",
    "atomic_save_settings",
    "canonical_host_profile",
    "default_settings_file",
    "load_settings",
    "step_motion_speed",
]
