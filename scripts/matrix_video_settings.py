#!/usr/bin/env python3
"""Strict, host-scoped persistence for Matrix video and camera settings.

The module is intentionally independent from the overlay and launcher.  Both can
use :class:`VideoSettingsStore` so validation, compare-and-swap semantics, and
durable persistence remain identical.  Every runtime-facing value is selected
from a fixed preset; arbitrary console commands or shell fragments can never
enter the settings file through this API.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
import fcntl
import json
import os
from pathlib import Path
import re
import secrets
import stat
import sys
import threading
from typing import Callable, Mapping, Sequence


SETTINGS_VERSION = 1
MAX_REVISION = (2**63) - 1
MAX_SETTINGS_BYTES = 64 * 1024

RESOLUTION_FIELD = "resolution"
WINDOW_MODE_FIELD = "window_mode"
FPS_LIMIT_FIELD = "fps_limit"
QUALITY_FIELD = "quality"
CAMERA_SMOOTHING_FIELD = "camera_smoothing"
VIDEO_SETTING_FIELDS = frozenset(
    {
        RESOLUTION_FIELD,
        WINDOW_MODE_FIELD,
        FPS_LIMIT_FIELD,
        QUALITY_FIELD,
        CAMERA_SMOOTHING_FIELD,
    }
)

# Values are literal, bounded display modes rather than free-form strings that
# could be interpreted as an Unreal console command.
RESOLUTION_PRESETS = (
    "1280x720",
    "1600x900",
    "1920x1080",
    "2560x1440",
)
WINDOW_MODE_PRESETS = ("windowed", "borderless", "fullscreen")
FPS_LIMIT_PRESETS = (30, 60, 90, 120)
QUALITY_PRESETS = ("low", "medium", "high", "epic")
CAMERA_SMOOTHING_PRESETS = ("off", "low", "medium", "high")

DEFAULT_RESOLUTION = "1920x1080"
DEFAULT_WINDOW_MODE = "borderless"
DEFAULT_FPS_LIMIT = 60
DEFAULT_QUALITY = "high"
DEFAULT_CAMERA_SMOOTHING = "medium"

_PROFILE_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,63}\Z")
_DIRECTORY_OPEN_FLAGS = (
    os.O_RDONLY
    | getattr(os, "O_DIRECTORY", 0)
    | getattr(os, "O_CLOEXEC", 0)
    | getattr(os, "O_NOFOLLOW", 0)
)


class VideoSettingsError(ValueError):
    """A typed schema, validation, path, or compare-and-swap error."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


class VideoSettingsPersistenceError(RuntimeError):
    """The candidate was valid but could not be durably stored."""

    code = "E_VIDEO_PERSIST"

    def __init__(self, message: str) -> None:
        super().__init__(message)
        self.message = message


def canonical_host_profile(value: object) -> str:
    """Return one bounded, path-safe host profile name."""

    if not isinstance(value, str) or _PROFILE_RE.fullmatch(value) is None:
        raise VideoSettingsError(
            "E_HOST_PROFILE",
            "host profile must use 1-64 ASCII letters, digits, dot, underscore, or dash",
        )
    return value


def default_settings_file(
    profile: object | None = None,
    *,
    config_home: Path | str | None = None,
) -> Path:
    """Return ``~/.config/matrix/hosts/<profile>/video-settings.json``.

    ``MATRIX_HOST_PROFILE`` supplies the profile when the caller omits it.
    ``XDG_CONFIG_HOME`` is honored in the usual way and is injectable for tests.
    """

    selected = os.environ.get("MATRIX_HOST_PROFILE") if profile is None else profile
    if selected is None:
        raise VideoSettingsError(
            "E_HOST_PROFILE", "host profile is required for video settings"
        )
    host_profile = canonical_host_profile(selected)
    if config_home is None:
        configured = os.environ.get("XDG_CONFIG_HOME")
        root = Path(configured) if configured else Path.home() / ".config"
    else:
        root = Path(config_home)
    return (
        root.expanduser()
        / "matrix"
        / "hosts"
        / host_profile
        / "video-settings.json"
    )


def _revision(value: object) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not 0 <= value <= MAX_REVISION
    ):
        raise VideoSettingsError(
            "E_VIDEO_REVISION",
            f"revision must be an integer in [0, {MAX_REVISION}]",
        )
    return value


def _preset(
    value: object,
    *,
    name: str,
    choices: Sequence[object],
) -> object:
    if type(value) is not type(choices[0]) or value not in choices:
        rendered = ", ".join(repr(choice) for choice in choices)
        raise VideoSettingsError(
            "E_VIDEO_PRESET",
            f"{name} must be one of: {rendered}",
        )
    return value


def _field(value: object) -> str:
    if not isinstance(value, str) or value not in VIDEO_SETTING_FIELDS:
        raise VideoSettingsError(
            "E_VIDEO_FIELD", f"unsupported video settings field: {value!r}"
        )
    return value


@dataclass(frozen=True)
class VideoSettings:
    """One validated, revisioned snapshot of all host video settings."""

    revision: int = 0
    resolution: str = DEFAULT_RESOLUTION
    window_mode: str = DEFAULT_WINDOW_MODE
    fps_limit: int = DEFAULT_FPS_LIMIT
    quality: str = DEFAULT_QUALITY
    camera_smoothing: str = DEFAULT_CAMERA_SMOOTHING

    def __post_init__(self) -> None:
        object.__setattr__(self, "revision", _revision(self.revision))
        object.__setattr__(
            self,
            RESOLUTION_FIELD,
            _preset(
                self.resolution,
                name=RESOLUTION_FIELD,
                choices=RESOLUTION_PRESETS,
            ),
        )
        object.__setattr__(
            self,
            WINDOW_MODE_FIELD,
            _preset(
                self.window_mode,
                name=WINDOW_MODE_FIELD,
                choices=WINDOW_MODE_PRESETS,
            ),
        )
        object.__setattr__(
            self,
            FPS_LIMIT_FIELD,
            _preset(
                self.fps_limit,
                name=FPS_LIMIT_FIELD,
                choices=FPS_LIMIT_PRESETS,
            ),
        )
        object.__setattr__(
            self,
            QUALITY_FIELD,
            _preset(
                self.quality,
                name=QUALITY_FIELD,
                choices=QUALITY_PRESETS,
            ),
        )
        object.__setattr__(
            self,
            CAMERA_SMOOTHING_FIELD,
            _preset(
                self.camera_smoothing,
                name=CAMERA_SMOOTHING_FIELD,
                choices=CAMERA_SMOOTHING_PRESETS,
            ),
        )

    def value_for_field(self, field: object) -> str | int:
        return getattr(self, _field(field))

    def with_patch(
        self,
        changes: Mapping[str, object],
        *,
        revision: int | None = None,
    ) -> "VideoSettings":
        """Return one fully validated snapshot with ``changes`` applied."""

        if not isinstance(changes, Mapping):
            raise VideoSettingsError(
                "E_VIDEO_PATCH", "video settings patch must be a mapping"
            )
        fields: dict[str, object] = {}
        for key, value in changes.items():
            fields[_field(key)] = value
        next_revision = self.revision if revision is None else _revision(revision)
        return replace(self, revision=next_revision, **fields)

    def to_mapping(self) -> dict[str, object]:
        return {
            "version": SETTINGS_VERSION,
            "revision": self.revision,
            RESOLUTION_FIELD: self.resolution,
            WINDOW_MODE_FIELD: self.window_mode,
            FPS_LIMIT_FIELD: self.fps_limit,
            QUALITY_FIELD: self.quality,
            CAMERA_SMOOTHING_FIELD: self.camera_smoothing,
        }

    def runtime_mapping(self) -> dict[str, object]:
        """Return structured launcher values with no command text.

        Callers can map the normalized preset names onto engine-specific
        settings without parsing arbitrary user input.
        """

        width_text, height_text = self.resolution.split("x", 1)
        width = int(width_text)
        height = int(height_text)
        return {
            "revision": self.revision,
            RESOLUTION_FIELD: self.resolution,
            "resolution_width": width,
            "resolution_height": height,
            WINDOW_MODE_FIELD: self.window_mode,
            FPS_LIMIT_FIELD: self.fps_limit,
            QUALITY_FIELD: self.quality,
            CAMERA_SMOOTHING_FIELD: self.camera_smoothing,
        }

    @classmethod
    def from_mapping(cls, value: object) -> "VideoSettings":
        expected = {"version", "revision", *VIDEO_SETTING_FIELDS}
        if not isinstance(value, dict) or set(value) != expected:
            raise VideoSettingsError(
                "E_VIDEO_SCHEMA",
                "video settings must contain exactly "
                "version/revision/resolution/window_mode/fps_limit/quality/"
                "camera_smoothing",
            )
        if type(value.get("version")) is not int or value.get("version") != SETTINGS_VERSION:
            raise VideoSettingsError(
                "E_VIDEO_VERSION",
                f"video settings version must be {SETTINGS_VERSION}",
            )
        return cls(
            revision=value.get("revision"),
            resolution=value.get(RESOLUTION_FIELD),
            window_mode=value.get(WINDOW_MODE_FIELD),
            fps_limit=value.get(FPS_LIMIT_FIELD),
            quality=value.get(QUALITY_FIELD),
            camera_smoothing=value.get(CAMERA_SMOOTHING_FIELD),
        )


@dataclass(frozen=True)
class LoadedVideoSettings:
    settings: VideoSettings
    status: str
    error: str | None = None


# Compatibility spelling for callers that follow the older mouse/motion store
# naming convention.
VideoSettingsLoad = LoadedVideoSettings


def _strict_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise VideoSettingsError(
                "E_VIDEO_SCHEMA", f"duplicate video settings field {key!r}"
            )
        result[key] = value
    return result


def _absolute_path(path: Path, *, label: str) -> Path:
    if not isinstance(path, Path) or not path.is_absolute():
        raise ValueError(f"{label} must be an absolute pathlib.Path")
    if len(path.parts) < 2 or any(part in {".", ".."} for part in path.parts[1:]):
        raise ValueError(f"{label} must not contain dot path components")
    return path


def _open_parent_directory(path: Path, *, create: bool) -> int:
    """Open a stable parent directory fd without following any symlink."""

    path = _absolute_path(path, label="video settings path")
    current_fd = os.open("/", _DIRECTORY_OPEN_FLAGS)
    try:
        for component in path.parts[1:-1]:
            if create:
                try:
                    os.mkdir(component, mode=0o700, dir_fd=current_fd)
                except FileExistsError:
                    pass
            next_fd = os.open(component, _DIRECTORY_OPEN_FLAGS, dir_fd=current_fd)
            os.close(current_fd)
            current_fd = next_fd
        return current_fd
    except BaseException:
        os.close(current_fd)
        raise


def _read_regular_file(path: Path) -> bytes:
    parent_fd = _open_parent_directory(path, create=False)
    file_fd: int | None = None
    try:
        file_fd = os.open(
            path.name,
            os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0),
            dir_fd=parent_fd,
        )
        metadata = os.fstat(file_fd)
        if not stat.S_ISREG(metadata.st_mode):
            raise OSError(f"video settings must be a regular non-symlink file: {path}")
        chunks: list[bytes] = []
        total = 0
        while True:
            chunk = os.read(file_fd, min(8192, MAX_SETTINGS_BYTES + 1 - total))
            if not chunk:
                break
            chunks.append(chunk)
            total += len(chunk)
            if total > MAX_SETTINGS_BYTES:
                raise OSError(
                    f"video settings exceed {MAX_SETTINGS_BYTES} bytes: {path}"
                )
        return b"".join(chunks)
    finally:
        if file_fd is not None:
            os.close(file_fd)
        os.close(parent_fd)


def load_settings(path: Path) -> LoadedVideoSettings:
    """Load one strict snapshot; missing or invalid state safely uses defaults."""

    _absolute_path(path, label="video settings path")
    try:
        payload = _read_regular_file(path)
    except FileNotFoundError:
        return LoadedVideoSettings(VideoSettings(), "missing")
    except OSError as exc:
        return LoadedVideoSettings(
            VideoSettings(), "invalid", f"cannot read video settings: {exc}"
        )
    try:
        text = payload.decode("utf-8")
        value = json.loads(
            text,
            object_pairs_hook=_strict_object,
            parse_constant=lambda token: (_ for _ in ()).throw(
                VideoSettingsError(
                    "E_VIDEO_PRESET", f"invalid JSON numeric constant {token}"
                )
            ),
        )
        settings = VideoSettings.from_mapping(value)
    except (
        UnicodeError,
        json.JSONDecodeError,
        VideoSettingsError,
        TypeError,
        ValueError,
    ) as exc:
        return LoadedVideoSettings(
            VideoSettings(), "invalid", f"invalid video settings: {exc}"
        )
    return LoadedVideoSettings(settings, "loaded")


def _encoded_settings(settings: VideoSettings) -> bytes:
    return (
        json.dumps(
            settings.to_mapping(),
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")


def atomic_save_settings(
    path: Path,
    settings: VideoSettings,
    expected_revision: int | None = None,
) -> None:
    """Atomically replace one private file without following path symlinks."""

    _absolute_path(path, label="video settings path")
    if not isinstance(settings, VideoSettings):
        raise TypeError("settings must be VideoSettings")
    expected = None if expected_revision is None else _revision(expected_revision)
    payload = _encoded_settings(settings)
    parent_fd: int | None = None
    temporary_name: str | None = None
    try:
        parent_fd = _open_parent_directory(path, create=True)
        # A stable directory-inode lock makes expected_revision a real CAS
        # among all writers that use this API, even though the settings inode is
        # replaced on every successful save.
        fcntl.flock(parent_fd, fcntl.LOCK_EX)
        try:
            existing = os.stat(path.name, dir_fd=parent_fd, follow_symlinks=False)
        except FileNotFoundError:
            existing = None
        if existing is not None and (
            stat.S_ISLNK(existing.st_mode) or not stat.S_ISREG(existing.st_mode)
        ):
            raise VideoSettingsPersistenceError(
                f"refusing non-regular or symlink video settings path: {path}"
            )
        if expected is not None:
            current = load_settings(path)
            if current.status == "invalid":
                raise VideoSettingsPersistenceError(
                    "refusing revision-guarded write over invalid video settings"
                )
            current_revision = (
                current.settings.revision if current.status == "loaded" else 0
            )
            if current_revision != expected:
                raise VideoSettingsError(
                    "E_VIDEO_REVISION_CONFLICT",
                    f"expected revision {expected}, current revision is "
                    f"{current_revision}",
                )

        for _attempt in range(16):
            candidate = f".{path.name}.{os.getpid()}.{secrets.token_hex(8)}"
            try:
                file_fd = os.open(
                    candidate,
                    os.O_WRONLY
                    | os.O_CREAT
                    | os.O_EXCL
                    | getattr(os, "O_CLOEXEC", 0)
                    | getattr(os, "O_NOFOLLOW", 0),
                    0o600,
                    dir_fd=parent_fd,
                )
            except FileExistsError:
                continue
            temporary_name = candidate
            break
        else:
            raise VideoSettingsPersistenceError(
                f"could not allocate video settings temporary file in {path.parent}"
            )

        try:
            os.fchmod(file_fd, 0o600)
            offset = 0
            while offset < len(payload):
                written = os.write(file_fd, payload[offset:])
                if written <= 0:
                    raise OSError("short write while persisting video settings")
                offset += written
            os.fsync(file_fd)
        finally:
            os.close(file_fd)

        os.replace(
            temporary_name,
            path.name,
            src_dir_fd=parent_fd,
            dst_dir_fd=parent_fd,
        )
        temporary_name = None
        os.fsync(parent_fd)
    except (VideoSettingsError, VideoSettingsPersistenceError):
        raise
    except (OSError, UnicodeError, ValueError) as exc:
        raise VideoSettingsPersistenceError(
            f"could not persist video settings: {exc}"
        ) from exc
    finally:
        if temporary_name is not None and parent_fd is not None:
            try:
                os.unlink(temporary_name, dir_fd=parent_fd)
            except FileNotFoundError:
                pass
        if parent_fd is not None:
            os.close(parent_fd)


@dataclass(frozen=True)
class VideoSettingsModification:
    settings: VideoSettings
    previous_settings: VideoSettings
    changed_fields: tuple[str, ...]
    changed: bool


SettingsSaver = Callable[[Path, VideoSettings], None]


def step_setting(
    settings: VideoSettings,
    field: object,
    direction: int,
) -> str | int:
    """Return the adjacent fixed preset for one panel setting."""

    if not isinstance(settings, VideoSettings):
        raise TypeError("settings must be VideoSettings")
    if type(direction) is not int or direction not in {-1, 1}:
        raise VideoSettingsError(
            "E_VIDEO_STEP", "video setting step direction must be -1 or 1"
        )
    canonical_field = _field(field)
    presets: Sequence[str | int]
    if canonical_field == RESOLUTION_FIELD:
        presets = RESOLUTION_PRESETS
    elif canonical_field == WINDOW_MODE_FIELD:
        presets = WINDOW_MODE_PRESETS
    elif canonical_field == FPS_LIMIT_FIELD:
        presets = FPS_LIMIT_PRESETS
    elif canonical_field == QUALITY_FIELD:
        presets = QUALITY_PRESETS
    else:
        presets = CAMERA_SMOOTHING_PRESETS
    current = settings.value_for_field(canonical_field)
    index = presets.index(current)
    next_index = max(0, min(len(presets) - 1, index + direction))
    return presets[next_index]


class VideoSettingsStore:
    """Serialize validated multi-field CAS updates through one settings owner."""

    def __init__(
        self,
        path: Path,
        *,
        initial: VideoSettings | None = None,
        fallback: VideoSettings | None = None,
        saver: SettingsSaver = atomic_save_settings,
    ) -> None:
        _absolute_path(path, label="video settings store path")
        if not callable(saver):
            raise TypeError("video settings saver must be callable")
        if initial is not None and fallback is not None:
            raise ValueError("initial and fallback video settings are mutually exclusive")
        if initial is not None and not isinstance(initial, VideoSettings):
            raise TypeError("initial settings must be VideoSettings")
        if fallback is not None and not isinstance(fallback, VideoSettings):
            raise TypeError("fallback settings must be VideoSettings")
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
    def settings(self) -> VideoSettings:
        with self._lock:
            return self._settings

    def reload(self) -> LoadedVideoSettings:
        """Reconcile in-memory state after a compare-and-swap conflict."""

        with self._lock:
            loaded = load_settings(self.path)
            if loaded.status == "invalid":
                raise VideoSettingsPersistenceError(
                    loaded.error or "video settings became invalid"
                )
            self._settings = loaded.settings
            self.load_status = loaded.status
            self.load_error = loaded.error
            return loaded

    def patch(
        self,
        changes: Mapping[str, object],
        *,
        expected_revision: int | None = None,
    ) -> VideoSettingsModification:
        """Validate and atomically persist one multi-field settings patch."""

        if not isinstance(changes, Mapping):
            raise VideoSettingsError(
                "E_VIDEO_PATCH", "video settings patch must be a mapping"
            )
        with self._lock:
            current = self._settings
            if expected_revision is not None:
                expected = _revision(expected_revision)
                if expected != current.revision:
                    raise VideoSettingsError(
                        "E_VIDEO_REVISION_CONFLICT",
                        f"expected revision {expected}, current revision is "
                        f"{current.revision}",
                    )
            validated = current.with_patch(changes)
            changed_fields = tuple(
                sorted(
                    field
                    for field in VIDEO_SETTING_FIELDS
                    if validated.value_for_field(field)
                    != current.value_for_field(field)
                )
            )
            if not changed_fields:
                return VideoSettingsModification(current, current, (), False)
            if current.revision >= MAX_REVISION:
                raise VideoSettingsError(
                    "E_VIDEO_REVISION", "video settings revision is exhausted"
                )
            candidate = validated.with_patch({}, revision=current.revision + 1)
            try:
                if self._saver is atomic_save_settings:
                    atomic_save_settings(
                        self.path,
                        candidate,
                        expected_revision=current.revision,
                    )
                else:
                    self._saver(self.path, candidate)
            except (VideoSettingsError, VideoSettingsPersistenceError):
                raise
            except (OSError, UnicodeError, ValueError) as exc:
                raise VideoSettingsPersistenceError(
                    f"could not persist video settings: {exc}"
                ) from exc
            self._settings = candidate
            self.load_status = "saved"
            self.load_error = None
            return VideoSettingsModification(
                candidate, current, changed_fields, True
            )

    def modify(
        self,
        field: object,
        value: object,
        *,
        expected_revision: int | None = None,
    ) -> VideoSettingsModification:
        """Convenience wrapper for a single-field CAS patch."""

        canonical_field = _field(field)
        return self.patch(
            {canonical_field: value},
            expected_revision=expected_revision,
        )

    def step(
        self,
        field: object,
        direction: int,
        *,
        expected_revision: int | None = None,
    ) -> VideoSettingsModification:
        """Persist one adjacent preset using the same CAS path as ``patch``."""

        with self._lock:
            value = step_setting(self._settings, field, direction)
            return self.modify(
                field,
                value,
                expected_revision=expected_revision,
            )

    def mapping(self) -> dict[str, object]:
        with self._lock:
            return {
                "settings_file": os.fspath(self.path),
                "load_status": self.load_status,
                "load_error": self.load_error,
                "settings": self._settings.to_mapping(),
                "runtime": self._settings.runtime_mapping(),
            }


def _settings_path(arguments: argparse.Namespace) -> Path:
    if arguments.settings_file is not None:
        return _absolute_path(
            arguments.settings_file, label="video settings CLI path"
        )
    return default_settings_file(arguments.profile)


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read or atomically patch host-scoped Matrix video settings"
    )
    parser.add_argument("--profile", help="host profile (or MATRIX_HOST_PROFILE)")
    parser.add_argument(
        "--settings-file",
        type=Path,
        help="explicit absolute settings file (overrides --profile)",
    )
    subparsers = parser.add_subparsers(dest="action", required=True)
    subparsers.add_parser("show", help="print persisted/default and runtime settings")
    launch_json = subparsers.add_parser(
        "launch-json", help="print compact, validated launcher settings JSON"
    )
    launch_json.add_argument(
        "--file",
        type=Path,
        help="explicit absolute settings file (overrides global options)",
    )
    patch = subparsers.add_parser("patch", help="apply one revision-guarded patch")
    patch.add_argument("--expected-revision", type=int, required=True)
    patch.add_argument("--resolution", choices=RESOLUTION_PRESETS)
    patch.add_argument("--window-mode", choices=WINDOW_MODE_PRESETS)
    patch.add_argument("--fps-limit", type=int, choices=FPS_LIMIT_PRESETS)
    patch.add_argument("--quality", choices=QUALITY_PRESETS)
    patch.add_argument("--camera-smoothing", choices=CAMERA_SMOOTHING_PRESETS)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _argument_parser()
    arguments = parser.parse_args(argv)
    try:
        if arguments.action == "launch-json" and arguments.file is not None:
            arguments.settings_file = arguments.file
        path = _settings_path(arguments)
        store = VideoSettingsStore(path)
        if arguments.action == "launch-json":
            json.dump(
                store.settings.runtime_mapping(),
                sys.stdout,
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            )
            sys.stdout.write("\n")
            return 0
        if arguments.action == "show":
            payload: dict[str, object] = store.mapping()
        else:
            changes = {
                field: getattr(arguments, field)
                for field in VIDEO_SETTING_FIELDS
                if getattr(arguments, field) is not None
            }
            if not changes:
                raise VideoSettingsError(
                    "E_VIDEO_PATCH", "patch requires at least one video setting"
                )
            modification = store.patch(
                changes,
                expected_revision=arguments.expected_revision,
            )
            payload = {
                **store.mapping(),
                "changed": modification.changed,
                "changed_fields": list(modification.changed_fields),
            }
        json.dump(payload, sys.stdout, sort_keys=True, allow_nan=False)
        sys.stdout.write("\n")
        return 0
    except (
        VideoSettingsError,
        VideoSettingsPersistenceError,
        OSError,
        ValueError,
    ) as exc:
        error = {
            "error": getattr(exc, "code", "E_VIDEO_SETTINGS"),
            "message": str(exc),
        }
        json.dump(error, sys.stderr, sort_keys=True)
        sys.stderr.write("\n")
        return 2


__all__ = [
    "CAMERA_SMOOTHING_FIELD",
    "CAMERA_SMOOTHING_PRESETS",
    "DEFAULT_CAMERA_SMOOTHING",
    "DEFAULT_FPS_LIMIT",
    "DEFAULT_QUALITY",
    "DEFAULT_RESOLUTION",
    "DEFAULT_WINDOW_MODE",
    "FPS_LIMIT_FIELD",
    "FPS_LIMIT_PRESETS",
    "LoadedVideoSettings",
    "MAX_REVISION",
    "QUALITY_FIELD",
    "QUALITY_PRESETS",
    "RESOLUTION_FIELD",
    "RESOLUTION_PRESETS",
    "SETTINGS_VERSION",
    "VIDEO_SETTING_FIELDS",
    "VideoSettings",
    "VideoSettingsError",
    "VideoSettingsLoad",
    "VideoSettingsModification",
    "VideoSettingsPersistenceError",
    "VideoSettingsStore",
    "WINDOW_MODE_FIELD",
    "WINDOW_MODE_PRESETS",
    "atomic_save_settings",
    "canonical_host_profile",
    "default_settings_file",
    "load_settings",
    "main",
    "step_setting",
]


if __name__ == "__main__":
    raise SystemExit(main())
