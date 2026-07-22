#!/usr/bin/env python3
"""Authenticated provider-side control broker for Matrix automation.

The broker deliberately lives beside the X11/gamepad input provider.  It never
opens the private physics socket and it never publishes DDS.  A local client
can hold one short-lived lease, replace a complete virtual input snapshot, or
queue a command for the provider's existing typed command path.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field, replace
import hmac
import json
import math
import os
from pathlib import Path
import re
import secrets
import socket
import stat
import struct
import tempfile
import time
from typing import Callable


PROTOCOL = "matrix-external-control/v2"
PROVIDER_GATE_SCHEMA = "matrix-external-provider-gate/v1"
PROVIDER_GATE_REQUIRED_NEUTRAL_FRAMES = 2
COMMAND_RECEIPT_SCHEMA = "matrix-external-command-receipt/v1"
MAX_PACKET_BYTES = 16_384
MAX_COMMAND_CHARS = 512
MAX_SEQUENCE = (2**63) - 1
DEFAULT_DEADMAN_SECONDS = 0.15
MAX_DEADMAN_SECONDS = 0.15
MAX_CLIENTS = 4
MAX_COMMAND_QUEUE = 64
MAX_COMMAND_RECEIPTS = 128
AUTHENTICATION_TIMEOUT_SECONDS = 0.25

KEYBOARD_FIELDS = (
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
    "escape",
    "mouse_mode",
    "mouse_speed_down",
    "mouse_speed_up",
    "apply_restart",
    "apply_return",
)
MOUSE_BUTTON_FIELDS = ("left", "middle", "right")
GAMEPAD_AXIS_FIELDS = ("forward", "right", "look_yaw", "look_pitch")
GAMEPAD_BUTTON_FIELDS = (
    "south",
    "east",
    "west",
    "north",
    "left_bumper",
    "right_bumper",
    "select",
    "start",
)


class ExternalControlError(ValueError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


def _strict_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise ExternalControlError("E_JSON_DUPLICATE", f"duplicate field {key!r}")
        result[key] = value
    return result


def _strict_json(payload: bytes) -> dict[str, object]:
    if not payload or len(payload) > MAX_PACKET_BYTES:
        raise ExternalControlError("E_PACKET_SIZE", "packet size is invalid")
    try:
        text = payload.decode("utf-8")
        value = json.loads(
            text,
            object_pairs_hook=_strict_object,
            parse_constant=lambda token: (_ for _ in ()).throw(
                ExternalControlError(
                    "E_JSON_NONFINITE", f"non-finite JSON value {token!r}"
                )
            ),
        )
    except UnicodeDecodeError as exc:
        raise ExternalControlError("E_JSON_UTF8", "packet is not UTF-8") from exc
    except json.JSONDecodeError as exc:
        raise ExternalControlError("E_JSON", "packet is not strict JSON") from exc
    if not isinstance(value, dict):
        raise ExternalControlError("E_SCHEMA", "packet must be a JSON object")
    return value


def _finite(value: object, *, field_name: str, minimum: float, maximum: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ExternalControlError("E_INPUT_TYPE", f"{field_name} must be numeric")
    number = float(value)
    if not math.isfinite(number):
        raise ExternalControlError("E_INPUT_NONFINITE", f"{field_name} must be finite")
    if not minimum <= number <= maximum:
        raise ExternalControlError(
            "E_INPUT_RANGE", f"{field_name} must be in [{minimum:g}, {maximum:g}]"
        )
    return number


def _bool_mapping(value: object, fields: tuple[str, ...], *, label: str) -> dict[str, bool]:
    if not isinstance(value, dict) or set(value) != set(fields):
        raise ExternalControlError(
            "E_INPUT_SCHEMA", f"{label} must define exactly {', '.join(fields)}"
        )
    result: dict[str, bool] = {}
    for name in fields:
        item = value[name]
        if type(item) is not bool:
            raise ExternalControlError("E_INPUT_TYPE", f"{label}.{name} must be boolean")
        result[name] = item
    return result


def neutral_keyboard() -> dict[str, bool]:
    return {name: False for name in KEYBOARD_FIELDS}


def neutral_mouse_buttons() -> dict[str, bool]:
    return {name: False for name in MOUSE_BUTTON_FIELDS}


def neutral_gamepad_buttons() -> dict[str, bool]:
    return {name: False for name in GAMEPAD_BUTTON_FIELDS}


@dataclass(frozen=True)
class ExternalInputState:
    keyboard: dict[str, bool] = field(default_factory=neutral_keyboard)
    mouse_buttons: dict[str, bool] = field(default_factory=neutral_mouse_buttons)
    mouse_dx: float = 0.0
    mouse_dy: float = 0.0
    gamepad_connected: bool = False
    gamepad_axes: dict[str, float] = field(
        default_factory=lambda: {name: 0.0 for name in GAMEPAD_AXIS_FIELDS}
    )
    gamepad_buttons: dict[str, bool] = field(default_factory=neutral_gamepad_buttons)

    @classmethod
    def neutral(cls) -> "ExternalInputState":
        return cls()

    @classmethod
    def from_mapping(cls, value: object) -> "ExternalInputState":
        if not isinstance(value, dict) or set(value) != {
            "keyboard",
            "mouse",
            "gamepad",
        }:
            raise ExternalControlError(
                "E_INPUT_SCHEMA", "input state must define keyboard/mouse/gamepad"
            )
        keyboard = _bool_mapping(value["keyboard"], KEYBOARD_FIELDS, label="keyboard")
        mouse = value["mouse"]
        if not isinstance(mouse, dict) or set(mouse) != {"buttons", "dx", "dy"}:
            raise ExternalControlError(
                "E_INPUT_SCHEMA", "mouse must define buttons/dx/dy"
            )
        mouse_buttons = _bool_mapping(
            mouse["buttons"], MOUSE_BUTTON_FIELDS, label="mouse.buttons"
        )
        gamepad = value["gamepad"]
        if not isinstance(gamepad, dict) or set(gamepad) != {
            "connected",
            "axes",
            "buttons",
        }:
            raise ExternalControlError(
                "E_INPUT_SCHEMA", "gamepad must define connected/axes/buttons"
            )
        if type(gamepad["connected"]) is not bool:
            raise ExternalControlError(
                "E_INPUT_TYPE", "gamepad.connected must be boolean"
            )
        axes = gamepad["axes"]
        if not isinstance(axes, dict) or set(axes) != set(GAMEPAD_AXIS_FIELDS):
            raise ExternalControlError(
                "E_INPUT_SCHEMA", "gamepad.axes has an invalid schema"
            )
        gamepad_axes = {
            name: _finite(
                axes[name], field_name=f"gamepad.axes.{name}", minimum=-1.0, maximum=1.0
            )
            for name in GAMEPAD_AXIS_FIELDS
        }
        gamepad_buttons = _bool_mapping(
            gamepad["buttons"], GAMEPAD_BUTTON_FIELDS, label="gamepad.buttons"
        )
        if not gamepad["connected"] and (
            any(abs(axis) > 1e-12 for axis in gamepad_axes.values())
            or any(gamepad_buttons.values())
        ):
            raise ExternalControlError(
                "E_INPUT_STATE",
                "disconnected gamepad must have neutral axes and buttons",
            )
        return cls(
            keyboard=keyboard,
            mouse_buttons=mouse_buttons,
            mouse_dx=_finite(
                mouse["dx"], field_name="mouse.dx", minimum=-4096.0, maximum=4096.0
            ),
            mouse_dy=_finite(
                mouse["dy"], field_name="mouse.dy", minimum=-4096.0, maximum=4096.0
            ),
            gamepad_connected=gamepad["connected"],
            gamepad_axes=gamepad_axes,
            gamepad_buttons=gamepad_buttons,
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "keyboard": dict(self.keyboard),
            "mouse": {
                "buttons": dict(self.mouse_buttons),
                "dx": self.mouse_dx,
                "dy": self.mouse_dy,
            },
            "gamepad": {
                "connected": self.gamepad_connected,
                "axes": dict(self.gamepad_axes),
                "buttons": dict(self.gamepad_buttons),
            },
        }

    def with_data_modify(self, path: str, value: bool | float) -> "ExternalInputState":
        parts = path.split(".")
        if len(parts) != 4 or parts[:2] != ["control", "input"]:
            raise ExternalControlError("E_DATA_PATH_UNKNOWN", "unsupported input path")
        family, name = parts[2:]
        if family == "keyboard" and name in KEYBOARD_FIELDS and type(value) is bool:
            keyboard = dict(self.keyboard)
            keyboard[name] = value
            return replace(self, keyboard=keyboard)
        if family == "mouse" and name in MOUSE_BUTTON_FIELDS and type(value) is bool:
            buttons = dict(self.mouse_buttons)
            buttons[name] = value
            return replace(self, mouse_buttons=buttons)
        if family == "mouse" and name in {"dx", "dy"}:
            number = _finite(value, field_name=path, minimum=-4096.0, maximum=4096.0)
            return replace(self, **{f"mouse_{name}": number})
        if family == "gamepad" and name in GAMEPAD_AXIS_FIELDS:
            number = _finite(value, field_name=path, minimum=-1.0, maximum=1.0)
            axes = dict(self.gamepad_axes)
            axes[name] = number
            return replace(self, gamepad_axes=axes, gamepad_connected=True)
        raise ExternalControlError("E_DATA_PATH_UNKNOWN", "unsupported input path")

    @property
    def locomotion_neutral(self) -> bool:
        return bool(
            not any(self.keyboard[name] for name in ("w", "a", "s", "d"))
            and abs(self.gamepad_axes["forward"]) <= 1e-12
            and abs(self.gamepad_axes["right"]) <= 1e-12
        )

    def without_locomotion(self) -> "ExternalInputState":
        keyboard = dict(self.keyboard)
        for name in ("w", "a", "s", "d"):
            keyboard[name] = False
        axes = dict(self.gamepad_axes)
        axes["forward"] = 0.0
        axes["right"] = 0.0
        return replace(self, keyboard=keyboard, gamepad_axes=axes)


@dataclass(frozen=True)
class ExternalInputToken:
    """Exact identity of one client-authored external input revision."""

    lease_id: str
    authority_epoch: int
    input_revision: int

    def __post_init__(self) -> None:
        if not isinstance(self.lease_id, str) or re.fullmatch(
            r"[0-9a-f]{32}", self.lease_id
        ) is None:
            raise ValueError("external input token lease id is invalid")
        if (
            isinstance(self.authority_epoch, bool)
            or not isinstance(self.authority_epoch, int)
            or self.authority_epoch <= 0
        ):
            raise ValueError("external input token authority epoch is invalid")
        if (
            isinstance(self.input_revision, bool)
            or not isinstance(self.input_revision, int)
            or self.input_revision < 0
        ):
            raise ValueError("external input token revision is invalid")

    def to_mapping(self) -> dict[str, object]:
        return {
            "lease_id": self.lease_id,
            "authority_epoch": self.authority_epoch,
            "input_revision": self.input_revision,
        }

    @classmethod
    def from_mapping(cls, value: object) -> "ExternalInputToken":
        if not isinstance(value, dict) or set(value) != {
            "lease_id",
            "authority_epoch",
            "input_revision",
        }:
            raise ValueError("external input token schema is invalid")
        return cls(
            lease_id=value["lease_id"],
            authority_epoch=value["authority_epoch"],
            input_revision=value["input_revision"],
        )


@dataclass(frozen=True)
class ProviderGateTelemetry:
    """Typed provider-to-client proof that external locomotion is rearmed."""

    authority_epoch: int
    lease_id: str | None
    input_revision: int | None
    phase: str
    ready: bool
    neutral_sent_count: int
    qualified_from_revision: int | None = None
    last_interlock_reason: str | None = None
    last_sequence: int | None = None
    schema: str = PROVIDER_GATE_SCHEMA
    required_neutral_frames: int = PROVIDER_GATE_REQUIRED_NEUTRAL_FRAMES

    def __post_init__(self) -> None:
        if self.schema != PROVIDER_GATE_SCHEMA:
            raise ValueError("provider gate schema is invalid")
        if (
            isinstance(self.authority_epoch, bool)
            or not isinstance(self.authority_epoch, int)
            or self.authority_epoch < 0
        ):
            raise ValueError("provider gate authority epoch is invalid")
        if self.phase not in {
            "inactive",
            "awaiting_neutral",
            "ready",
            "interlocked",
        }:
            raise ValueError("provider gate phase is invalid")
        active_identity = self.lease_id is not None or self.input_revision is not None
        if self.phase == "inactive":
            if active_identity:
                raise ValueError("inactive provider gate cannot carry an input token")
        else:
            try:
                ExternalInputToken(
                    lease_id=self.lease_id,
                    authority_epoch=self.authority_epoch,
                    input_revision=self.input_revision,
                )
            except (TypeError, ValueError) as exc:
                raise ValueError("active provider gate input token is invalid") from exc
        if type(self.ready) is not bool:
            raise ValueError("provider gate ready flag must be boolean")
        if (
            isinstance(self.neutral_sent_count, bool)
            or not isinstance(self.neutral_sent_count, int)
            or self.neutral_sent_count < 0
        ):
            raise ValueError("provider gate neutral count is invalid")
        if self.required_neutral_frames != PROVIDER_GATE_REQUIRED_NEUTRAL_FRAMES:
            raise ValueError("provider gate neutral-frame contract is invalid")
        if self.last_interlock_reason is not None and (
            not isinstance(self.last_interlock_reason, str)
            or not self.last_interlock_reason
        ):
            raise ValueError("provider gate interlock reason is invalid")
        if self.last_sequence is not None and (
            isinstance(self.last_sequence, bool)
            or not isinstance(self.last_sequence, int)
            or self.last_sequence <= 0
        ):
            raise ValueError("provider gate sequence is invalid")
        if self.ready != (self.phase == "ready"):
            raise ValueError("provider gate ready flag disagrees with phase")
        if self.ready and self.neutral_sent_count < self.required_neutral_frames:
            raise ValueError("ready provider gate lacks neutral-frame proof")
        if self.phase in {"inactive", "interlocked"} and self.neutral_sent_count != 0:
            raise ValueError("inactive/interlocked provider gate must have zero count")
        if self.ready:
            if (
                isinstance(self.qualified_from_revision, bool)
                or not isinstance(self.qualified_from_revision, int)
                or self.qualified_from_revision < 0
                or self.input_revision is None
                or self.qualified_from_revision > self.input_revision
            ):
                raise ValueError("ready provider gate qualification is invalid")
        elif self.qualified_from_revision is not None:
            raise ValueError("unready provider gate cannot carry a qualification")

    @property
    def input_token(self) -> ExternalInputToken | None:
        if self.lease_id is None or self.input_revision is None:
            return None
        return ExternalInputToken(
            lease_id=self.lease_id,
            authority_epoch=self.authority_epoch,
            input_revision=self.input_revision,
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "authority_epoch": self.authority_epoch,
            "lease_id": self.lease_id,
            "input_revision": self.input_revision,
            "phase": self.phase,
            "ready": self.ready,
            "neutral_sent_count": self.neutral_sent_count,
            "qualified_from_revision": self.qualified_from_revision,
            "required_neutral_frames": self.required_neutral_frames,
            "last_interlock_reason": self.last_interlock_reason,
            "last_sequence": self.last_sequence,
        }

    @classmethod
    def from_mapping(cls, value: object) -> "ProviderGateTelemetry":
        if not isinstance(value, dict) or set(value) != {
            "schema",
            "authority_epoch",
            "lease_id",
            "input_revision",
            "phase",
            "ready",
            "neutral_sent_count",
            "qualified_from_revision",
            "required_neutral_frames",
            "last_interlock_reason",
            "last_sequence",
        }:
            raise ValueError("provider gate telemetry schema is invalid")
        return cls(
            schema=value["schema"],
            authority_epoch=value["authority_epoch"],
            lease_id=value["lease_id"],
            input_revision=value["input_revision"],
            phase=value["phase"],
            ready=value["ready"],
            neutral_sent_count=value["neutral_sent_count"],
            qualified_from_revision=value["qualified_from_revision"],
            required_neutral_frames=value["required_neutral_frames"],
            last_interlock_reason=value["last_interlock_reason"],
            last_sequence=value["last_sequence"],
        )


@dataclass(frozen=True)
class ExternalCommand:
    command: str
    request_sequence: int
    peer_pid: int
    command_id: str
    authority_epoch: int


@dataclass
class _CommandReceipt:
    command_id: str
    authority_epoch: int
    peer_pid: int
    state: str
    terminal: bool = False
    authority_revoked: bool = False
    result: dict[str, object] | None = None

    def to_mapping(self) -> dict[str, object]:
        return {
            "command_id": self.command_id,
            "authority_epoch": self.authority_epoch,
            "state": self.state,
            "terminal": self.terminal,
            "authority_revoked": self.authority_revoked,
            "result": self.result,
        }


@dataclass
class _Client:
    connection: socket.socket
    pid: int
    uid: int
    connected_at: float
    authenticated: bool = False
    last_sequence: int = 0


Clock = Callable[[], float]


def _atomic_capability(path: Path, token: str) -> None:
    if not path.is_absolute():
        raise ValueError("capability path must be absolute")
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    if path.is_symlink():
        raise RuntimeError("refusing symlink capability path")
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="ascii",
            dir=path.parent,
            prefix=f".{path.name}.",
            delete=False,
        ) as stream:
            temporary = Path(stream.name)
            os.fchmod(stream.fileno(), 0o600)
            stream.write(token + "\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        temporary = None
    finally:
        if temporary is not None:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass


def _atomic_command_receipt(path: Path, receipt: dict[str, object]) -> None:
    if not path.is_absolute():
        raise ValueError("command receipt path must be absolute")
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    if path.is_symlink():
        raise RuntimeError("refusing symlink command receipt path")
    payload = (
        json.dumps(receipt, separators=(",", ":"), sort_keys=True, allow_nan=False)
        + "\n"
    )
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
            stream.write(payload)
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


class ExternalControlBroker:
    """Nonblocking, same-UID AF_UNIX/SOCK_SEQPACKET control broker."""

    def __init__(
        self,
        path: Path,
        capability_file: Path,
        *,
        deadman_seconds: float = DEFAULT_DEADMAN_SECONDS,
        clock: Clock = time.monotonic,
    ) -> None:
        if not isinstance(path, Path) or not path.is_absolute():
            raise ValueError("external control socket path must be absolute")
        if not isinstance(capability_file, Path) or not capability_file.is_absolute():
            raise ValueError("external capability path must be absolute")
        if path.resolve(strict=False) == capability_file.resolve(strict=False):
            raise ValueError("external socket and capability paths must be distinct")
        deadman = float(deadman_seconds)
        if not math.isfinite(deadman) or not 0.01 <= deadman <= MAX_DEADMAN_SECONDS:
            raise ValueError("external deadman must be in [0.01, 0.15] seconds")
        if not callable(clock):
            raise TypeError("clock must be callable")
        if getattr(socket, "SOCK_SEQPACKET", None) is None:
            raise RuntimeError("external control requires SOCK_SEQPACKET")
        self.path = path
        self.capability_file = capability_file
        self.receipt_directory = capability_file.with_name(
            f"{capability_file.name}.receipts"
        )
        self.deadman_seconds = deadman
        self._clock = clock
        self._token = secrets.token_hex(32)
        self._listener: socket.socket | None = None
        self._owned_socket_identity: tuple[int, int] | None = None
        self._owned_capability_identity: tuple[int, int] | None = None
        self._clients: dict[int, _Client] = {}
        self._lease_client_fd: int | None = None
        self._lease_id: str | None = None
        self._lease_last_refresh: float | None = None
        self._fatal_authority_reason: str | None = None
        self._authority_epoch = 0
        self._input_revision: int | None = None
        self._provider_gate = ProviderGateTelemetry(
            authority_epoch=0,
            lease_id=None,
            input_revision=None,
            phase="inactive",
            ready=False,
            neutral_sent_count=0,
        )
        self._state = ExternalInputState.neutral()
        self._commands: deque[ExternalCommand] = deque(maxlen=MAX_COMMAND_QUEUE)
        self._command_receipts: dict[str, _CommandReceipt] = {}
        self._last_override_reason: str | None = None
        self.accepted_connections = 0
        self.rejected_peers = 0
        self.protocol_errors = 0
        self.stale_lease_rejections = 0
        self.auth_failures = 0
        self.lease_acquisitions = 0
        self.lease_conflicts = 0
        self.deadman_stops = 0
        self.local_overrides = 0
        self.input_replacements = 0
        self.commands_queued = 0
        self.commands_admitted = 0
        self.commands_completed = 0
        self.commands_rejected = 0
        self.commands_cancelled = 0
        self.receipt_persistence_errors = 0
        self.last_receipt_persistence_error: str | None = None

    @property
    def capability(self) -> str:
        return self._token

    @property
    def authority_epoch(self) -> int:
        return self._authority_epoch

    @property
    def provider_gate(self) -> ProviderGateTelemetry:
        return self._provider_gate

    @property
    def input_token(self) -> ExternalInputToken | None:
        if self._lease_id is None or self._input_revision is None:
            return None
        return ExternalInputToken(
            lease_id=self._lease_id,
            authority_epoch=self._authority_epoch,
            input_revision=self._input_revision,
        )

    def update_provider_gate(self, value: ProviderGateTelemetry | dict[str, object]) -> bool:
        """Publish an epoch-bound provider proof without reviving stale authority."""

        telemetry = (
            value
            if isinstance(value, ProviderGateTelemetry)
            else ProviderGateTelemetry.from_mapping(value)
        )
        if telemetry.authority_epoch != self._authority_epoch:
            return False
        if self._lease_id is None:
            if telemetry.phase != "inactive":
                return False
        else:
            current = self.input_token
            if telemetry.phase == "inactive" or telemetry.input_token != current:
                return False
            if self._fatal_authority_reason is not None and (
                telemetry.phase != "interlocked"
                or telemetry.last_interlock_reason
                != self._fatal_authority_reason
            ):
                return False
        self._provider_gate = telemetry
        return True

    def latch_fatal_authority(self, reason: str) -> bool:
        """Zero input and freeze authority until its original deadman expiry."""

        if reason != "physical_focus_lost":
            raise ValueError("unsupported fatal external-authority reason")
        if self._lease_id is None:
            return False
        if self._fatal_authority_reason is not None:
            return False
        token = self.input_token
        assert token is not None
        self._fatal_authority_reason = reason
        self._state = ExternalInputState.neutral()
        self._last_override_reason = reason
        self._provider_gate = ProviderGateTelemetry(
            authority_epoch=token.authority_epoch,
            lease_id=token.lease_id,
            input_revision=token.input_revision,
            phase="interlocked",
            ready=False,
            neutral_sent_count=0,
            last_interlock_reason=reason,
            last_sequence=self._provider_gate.last_sequence,
        )
        return True

    @staticmethod
    def _identity(path: Path) -> tuple[int, int]:
        metadata = path.stat(follow_symlinks=False)
        return (metadata.st_dev, metadata.st_ino)

    def _prepare_endpoint(self) -> None:
        if not self.path.exists() and not self.path.is_symlink():
            return
        if self.path.is_symlink():
            raise RuntimeError(f"refusing symlink external endpoint: {self.path}")
        metadata = self.path.stat(follow_symlinks=False)
        if not stat.S_ISSOCK(metadata.st_mode) or metadata.st_uid != os.getuid():
            raise RuntimeError(f"refusing to replace unowned non-socket endpoint: {self.path}")
        expected_identity = (metadata.st_dev, metadata.st_ino)
        probe = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        probe.settimeout(0.10)
        try:
            probe.connect(os.fspath(self.path))
        except FileNotFoundError:
            return
        except ConnectionRefusedError:
            pass
        except OSError as exc:
            raise RuntimeError(
                f"cannot prove existing external endpoint is stale: {self.path}: {exc}"
            ) from exc
        else:
            raise RuntimeError(f"external control endpoint is already active: {self.path}")
        finally:
            probe.close()
        try:
            current_identity = self._identity(self.path)
        except FileNotFoundError:
            return
        if current_identity != expected_identity:
            raise RuntimeError("external endpoint changed during stale-socket check")
        self.path.unlink()

    def _prepare_capability(self) -> None:
        if not self.capability_file.exists() and not self.capability_file.is_symlink():
            return
        if self.capability_file.is_symlink():
            raise RuntimeError(
                f"refusing symlink external capability: {self.capability_file}"
            )
        metadata = self.capability_file.stat(follow_symlinks=False)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_uid != os.getuid():
            raise RuntimeError(
                f"refusing to replace unowned capability: {self.capability_file}"
            )

    def _prepare_receipt_directory(self) -> None:
        if self.receipt_directory.is_symlink():
            raise RuntimeError(
                f"refusing symlink receipt directory: {self.receipt_directory}"
            )
        try:
            self.receipt_directory.mkdir(mode=0o700)
        except FileExistsError:
            pass
        metadata = self.receipt_directory.stat(follow_symlinks=False)
        if not stat.S_ISDIR(metadata.st_mode) or metadata.st_uid != os.getuid():
            raise RuntimeError(
                f"external receipt path is not an owned directory: "
                f"{self.receipt_directory}"
            )
        os.chmod(self.receipt_directory, 0o700, follow_symlinks=False)

    def open(self) -> None:
        if self._listener is not None:
            return
        self.path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        self._prepare_endpoint()
        self._prepare_capability()
        self._prepare_receipt_directory()
        _atomic_capability(self.capability_file, self._token)
        self._owned_capability_identity = self._identity(self.capability_file)
        listener = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        try:
            listener.bind(os.fspath(self.path))
            os.chmod(self.path, 0o600)
            listener.listen(MAX_CLIENTS)
            listener.setblocking(False)
            self._owned_socket_identity = self._identity(self.path)
        except Exception:
            listener.close()
            self._unlink_owned_paths()
            raise
        self._listener = listener

    def _unlink_owned_paths(self) -> None:
        for path, expected in (
            (self.path, self._owned_socket_identity),
            (self.capability_file, self._owned_capability_identity),
        ):
            if expected is None:
                continue
            try:
                if self._identity(path) == expected:
                    path.unlink()
            except FileNotFoundError:
                pass
        self._owned_socket_identity = None
        self._owned_capability_identity = None

    @staticmethod
    def _peer(connection: socket.socket) -> tuple[int, int]:
        raw = connection.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12)
        pid, uid, _gid = struct.unpack("3i", raw)
        return pid, uid

    def _release_lease(self, *, reason: str, count_deadman: bool = False) -> None:
        if self._lease_id is not None and count_deadman:
            self.deadman_stops += 1
        while self._commands:
            command = self._commands.popleft()
            receipt = self._command_receipts.get(command.command_id)
            if receipt is not None and not receipt.terminal:
                receipt.state = "cancelled"
                receipt.terminal = True
                receipt.authority_revoked = True
                receipt.result = {
                    "ok": False,
                    "code": "E_AUTHORITY_REVOKED",
                    "message": f"external command cancelled: {reason}",
                }
                self.commands_cancelled += 1
                self._persist_receipt(receipt)
        for receipt in self._command_receipts.values():
            if (
                not receipt.terminal
                and receipt.authority_epoch == self._authority_epoch
            ):
                receipt.authority_revoked = True
        self._lease_client_fd = None
        self._lease_id = None
        self._lease_last_refresh = None
        self._fatal_authority_reason = None
        self._input_revision = None
        self._state = ExternalInputState.neutral()
        self._last_override_reason = reason
        self._provider_gate = ProviderGateTelemetry(
            authority_epoch=self._authority_epoch,
            lease_id=None,
            input_revision=None,
            phase="inactive",
            ready=False,
            neutral_sent_count=0,
            last_interlock_reason=reason,
            last_sequence=self._provider_gate.last_sequence,
        )

    def _prune_receipts(self) -> None:
        if len(self._command_receipts) <= MAX_COMMAND_RECEIPTS:
            return
        for command_id, receipt in tuple(self._command_receipts.items()):
            if receipt.terminal:
                self._command_receipts.pop(command_id, None)
                if len(self._command_receipts) <= MAX_COMMAND_RECEIPTS:
                    return

    def _persist_receipt(self, receipt: _CommandReceipt) -> None:
        if not receipt.terminal:
            return
        payload = {
            "schema": COMMAND_RECEIPT_SCHEMA,
            "written_unix_ns": time.time_ns(),
            "receipt": receipt.to_mapping(),
        }
        try:
            _atomic_command_receipt(
                self.receipt_directory / f"{receipt.command_id}.json",
                payload,
            )
        except (OSError, RuntimeError, TypeError, ValueError) as exc:
            self.receipt_persistence_errors += 1
            self.last_receipt_persistence_error = str(exc)
        else:
            self.last_receipt_persistence_error = None

    def _expire(self, now: float) -> None:
        if self._lease_last_refresh is None:
            return
        # Bias the floating-point boundary toward stopping: the deadman may
        # fire a sub-picosecond early but can never retain motion past its
        # configured 150 ms safety contract due to subtraction rounding.
        if now - self._lease_last_refresh >= self.deadman_seconds - 1e-12:
            self._release_lease(reason="deadman", count_deadman=True)

    def _accept(self, now: float) -> None:
        assert self._listener is not None
        for _ in range(MAX_CLIENTS):
            try:
                connection, _ = self._listener.accept()
            except BlockingIOError:
                return
            connection.setblocking(False)
            try:
                pid, uid = self._peer(connection)
            except OSError:
                connection.close()
                self.rejected_peers += 1
                continue
            if len(self._clients) >= MAX_CLIENTS:
                unauthenticated = sorted(
                    (
                        (fd, candidate)
                        for fd, candidate in self._clients.items()
                        if not candidate.authenticated
                    ),
                    key=lambda item: item[1].connected_at,
                )
                if unauthenticated:
                    self._drop_client(unauthenticated[0][0])
            if uid != os.getuid() or len(self._clients) >= MAX_CLIENTS:
                connection.close()
                self.rejected_peers += 1
                continue
            self._clients[connection.fileno()] = _Client(
                connection,
                pid,
                uid,
                connected_at=now,
            )
            self.accepted_connections += 1

    @staticmethod
    def _response(
        sequence: int,
        *,
        ok: bool,
        code: str,
        message: str,
        data: dict[str, object] | None = None,
    ) -> bytes:
        payload = {
            "protocol": PROTOCOL,
            "kind": "response",
            "sequence": sequence,
            "ok": ok,
            "code": code,
            "message": message,
            "data": data,
        }
        return (json.dumps(payload, separators=(",", ":"), allow_nan=False) + "\n").encode(
            "utf-8"
        )

    def _require_lease(self, client_fd: int, payload: dict[str, object]) -> None:
        if "lease_id" not in payload:
            raise ExternalControlError("E_SCHEMA", "lease_id is required")
        lease_id = payload.get("lease_id")
        if (
            not isinstance(lease_id, str)
            or self._lease_id is None
            or client_fd != self._lease_client_fd
            or not hmac.compare_digest(lease_id, self._lease_id)
        ):
            raise ExternalControlError("E_LEASE", "client does not own the active lease")

    def _handle(
        self,
        client_fd: int,
        client: _Client,
        request: dict[str, object],
        now: float,
    ) -> bytes:
        if set(request) != {
            "protocol",
            "kind",
            "sequence",
            "capability",
            "operation",
            "payload",
        }:
            raise ExternalControlError("E_SCHEMA", "request has an invalid schema")
        if request.get("protocol") != PROTOCOL or request.get("kind") != "request":
            raise ExternalControlError("E_PROTOCOL", "request protocol is invalid")
        sequence = request.get("sequence")
        if (
            isinstance(sequence, bool)
            or not isinstance(sequence, int)
            or not 1 <= sequence <= MAX_SEQUENCE
        ):
            raise ExternalControlError("E_SEQUENCE", "sequence must be a positive integer")
        if sequence <= client.last_sequence:
            raise ExternalControlError("E_SEQUENCE", "sequence did not increase")
        client.last_sequence = sequence
        capability = request.get("capability")
        if not isinstance(capability, str) or not hmac.compare_digest(
            capability, self._token
        ):
            self.auth_failures += 1
            raise ExternalControlError("E_AUTH", "capability is invalid")
        client.authenticated = True
        operation = request.get("operation")
        payload = request.get("payload")
        if not isinstance(operation, str) or not isinstance(payload, dict):
            raise ExternalControlError("E_SCHEMA", "operation/payload are invalid")

        if operation == "lease.acquire":
            if payload:
                raise ExternalControlError("E_SCHEMA", "lease.acquire payload must be empty")
            if self._lease_id is not None and self._lease_client_fd != client_fd:
                self.lease_conflicts += 1
                raise ExternalControlError("E_LEASE_BUSY", "another client owns the lease")
            if self._fatal_authority_reason is not None:
                raise ExternalControlError(
                    "E_AUTHORITY_REVOKED",
                    "external authority is latched until its deadman deadline",
                )
            if self._lease_id is None:
                self._lease_client_fd = client_fd
                self._lease_id = secrets.token_hex(16)
                self._authority_epoch += 1
                self._input_revision = 0
                self.lease_acquisitions += 1
                self._fatal_authority_reason = None
                self._provider_gate = ProviderGateTelemetry(
                    authority_epoch=self._authority_epoch,
                    lease_id=self._lease_id,
                    input_revision=self._input_revision,
                    phase="awaiting_neutral",
                    ready=False,
                    neutral_sent_count=0,
                )
            self._lease_last_refresh = now
            self._last_override_reason = None
            return self._response(
                sequence,
                ok=True,
                code="OK_LEASE",
                message="external control lease acquired",
                data={
                    "lease_id": self._lease_id,
                    "deadman_seconds": self.deadman_seconds,
                    "authority_epoch": self._authority_epoch,
                    "input_token": self.input_token.to_mapping(),
                    "provider_gate": self._provider_gate.to_mapping(),
                },
            )

        if operation == "lease.renew":
            if set(payload) != {"lease_id"}:
                raise ExternalControlError("E_SCHEMA", "lease.renew payload is invalid")
            self._require_lease(client_fd, payload)
            if self._fatal_authority_reason is None:
                self._lease_last_refresh = now
            return self._response(
                sequence,
                ok=True,
                code="OK_RENEWED",
                message=(
                    "external control lease renewed"
                    if self._fatal_authority_reason is None
                    else "fatal authority latch observed; deadline unchanged"
                ),
                data={
                    "lease_id": self._lease_id,
                    "authority_epoch": self._authority_epoch,
                    "input_token": self.input_token.to_mapping(),
                    "provider_gate": self._provider_gate.to_mapping(),
                },
            )

        if operation == "lease.release":
            if set(payload) != {"lease_id"}:
                raise ExternalControlError("E_SCHEMA", "lease.release payload is invalid")
            self._require_lease(client_fd, payload)
            self._release_lease(reason="client_release")
            return self._response(
                sequence,
                ok=True,
                code="OK_RELEASED",
                message="external control lease released",
            )

        if operation == "input.replace":
            if set(payload) not in (
                {"lease_id", "state"},
                {"lease_id", "state", "qualified_token"},
            ):
                raise ExternalControlError("E_SCHEMA", "input.replace payload is invalid")
            self._require_lease(client_fd, payload)
            state = ExternalInputState.from_mapping(payload["state"])
            if self._fatal_authority_reason is not None:
                if state != ExternalInputState.neutral():
                    raise ExternalControlError(
                        "E_AUTHORITY_REVOKED",
                        "fatal authority latch only permits full-neutral cleanup",
                    )
                assert self._input_revision is not None
                self._input_revision += 1
                self._state = state
                self.input_replacements += 1
                cleanup_token = self.input_token
                assert cleanup_token is not None
                self._provider_gate = ProviderGateTelemetry(
                    authority_epoch=cleanup_token.authority_epoch,
                    lease_id=cleanup_token.lease_id,
                    input_revision=cleanup_token.input_revision,
                    phase="interlocked",
                    ready=False,
                    neutral_sent_count=0,
                    last_interlock_reason=self._fatal_authority_reason,
                    last_sequence=self._provider_gate.last_sequence,
                )
                return self._response(
                    sequence,
                    ok=True,
                    code="OK_INPUT_REPLACED",
                    message="fatal authority input cleared; deadline unchanged",
                    data={
                        "input_token": cleanup_token.to_mapping(),
                        "provider_gate": self._provider_gate.to_mapping(),
                    },
                )
            current_token = self.input_token
            assert current_token is not None
            qualified_from_revision: int | None = None
            if not state.locomotion_neutral:
                raw_proof = payload.get("qualified_token")
                if raw_proof is None:
                    raise ExternalControlError(
                        "E_INPUT_NOT_READY",
                        "non-neutral locomotion requires a provider gate proof",
                    )
                try:
                    proof = ExternalInputToken.from_mapping(raw_proof)
                except ValueError as exc:
                    raise ExternalControlError(
                        "E_INPUT_SUPERSEDED",
                        "provider gate proof token is malformed or stale",
                    ) from exc
                if proof != current_token:
                    raise ExternalControlError(
                        "E_INPUT_SUPERSEDED",
                        "provider gate proof no longer names the current input",
                    )
                if (
                    not self._provider_gate.ready
                    or self._provider_gate.input_token != current_token
                ):
                    raise ExternalControlError(
                        "E_INPUT_NOT_READY",
                        "provider has not qualified the current neutral input",
                    )
                qualified_from_revision = (
                    self._provider_gate.qualified_from_revision
                )
                assert qualified_from_revision is not None
            assert self._input_revision is not None
            self._input_revision += 1
            self._state = state
            self._lease_last_refresh = now
            self.input_replacements += 1
            next_token = self.input_token
            assert next_token is not None
            if state.locomotion_neutral:
                self._provider_gate = ProviderGateTelemetry(
                    authority_epoch=self._authority_epoch,
                    lease_id=next_token.lease_id,
                    input_revision=next_token.input_revision,
                    phase="awaiting_neutral",
                    ready=False,
                    neutral_sent_count=0,
                    last_interlock_reason="input_revision_changed",
                    last_sequence=self._provider_gate.last_sequence,
                )
            else:
                self._provider_gate = ProviderGateTelemetry(
                    authority_epoch=self._authority_epoch,
                    lease_id=next_token.lease_id,
                    input_revision=next_token.input_revision,
                    phase="ready",
                    ready=True,
                    neutral_sent_count=self._provider_gate.neutral_sent_count,
                    qualified_from_revision=qualified_from_revision,
                    # A successful exact-token transfer is the current gate
                    # state.  Do not carry a historical rearm diagnostic into
                    # a ready revision where clients could mistake it for a
                    # live interlock.
                    last_interlock_reason=None,
                    last_sequence=self._provider_gate.last_sequence,
                )
            return self._response(
                sequence,
                ok=True,
                code="OK_INPUT_REPLACED",
                message="virtual input state replaced",
                data={
                    "input_token": next_token.to_mapping(),
                    "provider_gate": self._provider_gate.to_mapping(),
                },
            )

        if operation == "command.submit":
            if set(payload) != {"lease_id", "command"}:
                raise ExternalControlError("E_SCHEMA", "command.submit payload is invalid")
            self._require_lease(client_fd, payload)
            if self._fatal_authority_reason is not None:
                raise ExternalControlError(
                    "E_AUTHORITY_REVOKED",
                    "fatal authority latch rejects new commands",
                )
            command = payload.get("command")
            if (
                not isinstance(command, str)
                or not command
                or len(command) > MAX_COMMAND_CHARS
                or any(ord(character) < 0x20 or ord(character) == 0x7F for character in command)
            ):
                raise ExternalControlError("E_COMMAND", "command must be bounded printable text")
            if len(self._commands) >= MAX_COMMAND_QUEUE:
                raise ExternalControlError("E_COMMAND_QUEUE", "command queue is full")
            command_id = secrets.token_hex(16)
            queued = ExternalCommand(
                command,
                sequence,
                client.pid,
                command_id,
                self._authority_epoch,
            )
            self._commands.append(queued)
            self._command_receipts[command_id] = _CommandReceipt(
                command_id=command_id,
                authority_epoch=self._authority_epoch,
                peer_pid=client.pid,
                state="queued",
            )
            self._prune_receipts()
            self._lease_last_refresh = now
            self.commands_queued += 1
            return self._response(
                sequence,
                ok=True,
                code="OK_COMMAND_QUEUED",
                message="command queued for the Matrix provider",
                data={
                    "command_id": command_id,
                    "authority_epoch": self._authority_epoch,
                },
            )

        if operation == "command.result":
            if set(payload) != {"command_id"}:
                raise ExternalControlError("E_SCHEMA", "command.result payload is invalid")
            command_id = payload.get("command_id")
            if not isinstance(command_id, str):
                raise ExternalControlError("E_SCHEMA", "command_id is invalid")
            receipt = self._command_receipts.get(command_id)
            if receipt is None or receipt.peer_pid != client.pid:
                raise ExternalControlError("E_COMMAND_UNKNOWN", "command receipt is unknown")
            return self._response(
                sequence,
                ok=True,
                code="OK_COMMAND_RESULT",
                message="external command receipt",
                data=receipt.to_mapping(),
            )

        if operation == "status.get":
            if payload:
                raise ExternalControlError("E_SCHEMA", "status.get payload must be empty")
            return self._response(
                sequence,
                ok=True,
                code="OK_STATUS",
                message="external control status",
                data=self.telemetry(now=now),
            )
        raise ExternalControlError("E_OPERATION", f"unsupported operation {operation!r}")

    def _drop_client(self, client_fd: int) -> None:
        client = self._clients.pop(client_fd, None)
        if client is not None:
            client.connection.close()
        if self._lease_client_fd == client_fd:
            self._release_lease(reason="disconnect")

    def poll(self, *, now: float | None = None) -> None:
        if self._listener is None:
            raise RuntimeError("external control broker is not open")
        current = self._clock() if now is None else float(now)
        if not math.isfinite(current) or current < 0.0:
            raise ValueError("external control time must be finite and nonnegative")
        self._expire(current)
        for client_fd, client in tuple(self._clients.items()):
            if (
                not client.authenticated
                and current - client.connected_at >= AUTHENTICATION_TIMEOUT_SECONDS
            ):
                self._drop_client(client_fd)
        self._accept(current)
        for client_fd, client in tuple(self._clients.items()):
            for _ in range(16):
                try:
                    packet = client.connection.recv(MAX_PACKET_BYTES + 1)
                except BlockingIOError:
                    break
                except OSError:
                    self._drop_client(client_fd)
                    break
                if not packet:
                    self._drop_client(client_fd)
                    break
                sequence = 0
                try:
                    request = _strict_json(packet)
                    candidate = request.get("sequence")
                    if isinstance(candidate, int) and not isinstance(candidate, bool):
                        sequence = candidate
                    response = self._handle(client_fd, client, request, current)
                except ExternalControlError as exc:
                    if exc.code == "E_LEASE":
                        self.stale_lease_rejections += 1
                    elif exc.code not in {
                        "E_AUTHORITY_REVOKED",
                        "E_INPUT_NOT_READY",
                        "E_INPUT_SUPERSEDED",
                    }:
                        self.protocol_errors += 1
                    response = self._response(
                        sequence,
                        ok=False,
                        code=exc.code,
                        message=exc.message,
                    )
                try:
                    sent = client.connection.send(response)
                    if sent != len(response):
                        raise OSError("partial external control response")
                except (BlockingIOError, OSError):
                    self._drop_client(client_fd)
                    break

    @property
    def lease_active(self) -> bool:
        self._expire(self._clock())
        return self._lease_id is not None

    def sample(self, *, now: float | None = None) -> ExternalInputState:
        state, _token = self.sample_with_token(now=now)
        return state

    def publish_boundary_token(
        self,
        *,
        now: float | None = None,
    ) -> ExternalInputToken | None:
        """Revalidate authority immediately before a provider socket write.

        This deliberately does not sample or consume one-shot mouse deltas.  A
        caller can therefore compare its earlier sampled token against this
        boundary token using one frozen monotonic timestamp.
        """

        current = self._clock() if now is None else float(now)
        if not math.isfinite(current) or current < 0.0:
            raise ValueError("external control time must be finite and nonnegative")
        self._expire(current)
        return self.input_token

    def sample_with_token(
        self,
        *,
        now: float | None = None,
    ) -> tuple[ExternalInputState, ExternalInputToken | None]:
        """Atomically sample state and the exact client revision that authored it."""

        current = self._clock() if now is None else float(now)
        self._expire(current)
        if self._lease_id is None:
            return ExternalInputState.neutral(), None
        state = self._state
        token = self.input_token
        assert token is not None
        if state.mouse_dx != 0.0 or state.mouse_dy != 0.0:
            self._state = replace(state, mouse_dx=0.0, mouse_dy=0.0)
        return state, token

    def apply_data_modify(
        self,
        path: str,
        value: bool | float,
        *,
        now: float | None = None,
    ) -> ExternalInputToken:
        current = self._clock() if now is None else float(now)
        self._expire(current)
        if self._lease_id is None:
            raise ExternalControlError("E_LEASE", "an active external lease is required")
        if self._fatal_authority_reason is not None:
            raise ExternalControlError(
                "E_AUTHORITY_REVOKED",
                "fatal authority latch rejects input modification",
            )
        replacement = self._state.with_data_modify(path, value)
        current_token = self.input_token
        assert current_token is not None
        qualified_from_revision: int | None = None
        if not replacement.locomotion_neutral:
            if (
                not self._provider_gate.ready
                or self._provider_gate.input_token != current_token
            ):
                raise ExternalControlError(
                    "E_INPUT_NOT_READY",
                    "data modify cannot create locomotion before provider rearm",
                )
            qualified_from_revision = self._provider_gate.qualified_from_revision
            assert qualified_from_revision is not None
        assert self._input_revision is not None
        self._input_revision += 1
        self._state = replacement
        self._lease_last_refresh = current
        self.input_replacements += 1
        next_token = self.input_token
        assert next_token is not None
        if replacement.locomotion_neutral:
            self._provider_gate = ProviderGateTelemetry(
                authority_epoch=self._authority_epoch,
                lease_id=next_token.lease_id,
                input_revision=next_token.input_revision,
                phase="awaiting_neutral",
                ready=False,
                neutral_sent_count=0,
                last_interlock_reason="input_revision_changed",
                last_sequence=self._provider_gate.last_sequence,
            )
        else:
            self._provider_gate = ProviderGateTelemetry(
                authority_epoch=self._authority_epoch,
                lease_id=next_token.lease_id,
                input_revision=next_token.input_revision,
                phase="ready",
                ready=True,
                neutral_sent_count=self._provider_gate.neutral_sent_count,
                qualified_from_revision=qualified_from_revision,
                last_interlock_reason=None,
                last_sequence=self._provider_gate.last_sequence,
            )
        return next_token

    def local_override(self, reason: str) -> None:
        if not isinstance(reason, str) or not reason:
            raise ValueError("local override reason must be non-empty")
        if self._lease_id is not None:
            self.local_overrides += 1
        self._release_lease(reason=reason)

    def drain_commands(self, *, limit: int = 16) -> tuple[ExternalCommand, ...]:
        if isinstance(limit, bool) or not isinstance(limit, int) or not 1 <= limit <= 64:
            raise ValueError("command drain limit must be in [1, 64]")
        if self._fatal_authority_reason is not None:
            return ()
        result = []
        while self._commands and len(result) < limit:
            command = self._commands.popleft()
            receipt = self._command_receipts.get(command.command_id)
            if (
                receipt is None
                or receipt.terminal
                or self._lease_id is None
                or command.authority_epoch != self._authority_epoch
            ):
                if receipt is not None and not receipt.terminal:
                    receipt.state = "cancelled"
                    receipt.terminal = True
                    receipt.authority_revoked = True
                    receipt.result = {
                        "ok": False,
                        "code": "E_AUTHORITY_REVOKED",
                        "message": "external command authority expired before admission",
                    }
                    self.commands_cancelled += 1
                    self._persist_receipt(receipt)
                continue
            receipt.state = "admitted"
            self.commands_admitted += 1
            result.append(command)
        return tuple(result)

    def complete_command(
        self,
        command: ExternalCommand,
        result: dict[str, object],
    ) -> None:
        if not isinstance(command, ExternalCommand):
            raise TypeError("external command completion requires ExternalCommand")
        if not isinstance(result, dict):
            raise TypeError("external command result must be a mapping")
        receipt = self._command_receipts.get(command.command_id)
        if receipt is None or receipt.terminal:
            return
        ok = result.get("ok")
        outcome_unknown = result.get("outcome_unknown")
        if type(outcome_unknown) is not bool or (
            not outcome_unknown and type(ok) is not bool
        ):
            raise ValueError("external command result lacks typed outcome fields")
        receipt.authority_revoked = bool(
            receipt.authority_revoked
            or command.authority_epoch != self._authority_epoch
            or self._lease_id is None
        )
        receipt.result = json.loads(json.dumps(result, allow_nan=False))
        receipt.terminal = True
        if outcome_unknown:
            receipt.state = "outcome_unknown"
        elif ok:
            receipt.state = "completed"
            self.commands_completed += 1
        else:
            receipt.state = "rejected"
            self.commands_rejected += 1
        self._persist_receipt(receipt)
        self._prune_receipts()

    def telemetry(self, *, now: float | None = None) -> dict[str, object]:
        current = self._clock() if now is None else float(now)
        self._expire(current)
        age = (
            max(0.0, current - self._lease_last_refresh)
            if self._lease_last_refresh is not None
            else None
        )
        owner = self._clients.get(self._lease_client_fd or -1)
        return {
            "protocol": PROTOCOL,
            "socket": os.fspath(self.path),
            "capability_file": os.fspath(self.capability_file),
            "receipt_directory": os.fspath(self.receipt_directory),
            "deadman_seconds": self.deadman_seconds,
            "connected_clients": len(self._clients),
            "lease_active": self._lease_id is not None,
            "lease_owner_pid": owner.pid if owner is not None else None,
            "authority_epoch": self._authority_epoch,
            "input_token": (
                self.input_token.to_mapping()
                if self.input_token is not None
                else None
            ),
            "provider_gate": self._provider_gate.to_mapping(),
            "lease_age_seconds": age,
            "fatal_authority_reason": self._fatal_authority_reason,
            "command_queue_depth": len(self._commands),
            "accepted_connections": self.accepted_connections,
            "rejected_peers": self.rejected_peers,
            "protocol_errors": self.protocol_errors,
            "stale_lease_rejections": self.stale_lease_rejections,
            "auth_failures": self.auth_failures,
            "lease_acquisitions": self.lease_acquisitions,
            "lease_conflicts": self.lease_conflicts,
            "deadman_stops": self.deadman_stops,
            "local_overrides": self.local_overrides,
            "input_replacements": self.input_replacements,
            "commands_queued": self.commands_queued,
            "commands_admitted": self.commands_admitted,
            "commands_completed": self.commands_completed,
            "commands_rejected": self.commands_rejected,
            "commands_cancelled": self.commands_cancelled,
            "command_receipts": len(self._command_receipts),
            "receipt_persistence_errors": self.receipt_persistence_errors,
            "last_receipt_persistence_error": self.last_receipt_persistence_error,
            "last_override_reason": self._last_override_reason,
        }

    def close(self) -> None:
        for client_fd in tuple(self._clients):
            self._drop_client(client_fd)
        if self._listener is not None:
            self._listener.close()
            self._listener = None
        self._release_lease(reason="closed")
        self._unlink_owned_paths()

    def __enter__(self) -> "ExternalControlBroker":
        self.open()
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()


__all__ = [
    "COMMAND_RECEIPT_SCHEMA",
    "DEFAULT_DEADMAN_SECONDS",
    "ExternalCommand",
    "ExternalControlBroker",
    "ExternalControlError",
    "ExternalInputToken",
    "ExternalInputState",
    "GAMEPAD_AXIS_FIELDS",
    "GAMEPAD_BUTTON_FIELDS",
    "KEYBOARD_FIELDS",
    "MAX_PACKET_BYTES",
    "MOUSE_BUTTON_FIELDS",
    "PROTOCOL",
    "PROVIDER_GATE_REQUIRED_NEUTRAL_FRAMES",
    "PROVIDER_GATE_SCHEMA",
    "ProviderGateTelemetry",
]
