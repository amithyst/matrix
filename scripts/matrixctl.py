#!/usr/bin/env python3
"""Small authenticated client for Matrix's provider-side external control API."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import re
import socket
import stat
import sys
import tempfile
import time

from matrix_external_control import (
    COMMAND_RECEIPT_SCHEMA,
    ExternalInputState,
    GAMEPAD_AXIS_FIELDS,
    KEYBOARD_FIELDS,
    MAX_PACKET_BYTES,
    PROTOCOL,
)


_CAPABILITY_RE = re.compile(r"[0-9a-f]{64}\Z")
# The provider publishes at 50 Hz and deliberately requires locomotion-neutral
# frames when authority changes to the external source.  Keep the new lease in
# that state across several provider frames so source-change and neutral-rearm
# interlocks cannot consume the first requested input.  Keyboard actions retain
# their modifiers during this interval so the double-tap detector establishes
# the requested speed tier before it sees the first WASD edge.
_NEUTRAL_WARMUP_SECONDS = 0.12


def _read_capability(path: Path) -> str:
    """Read one private capability without following the final path component."""

    flags = os.O_RDONLY | os.O_NONBLOCK | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(path, flags)
    try:
        metadata = os.fstat(descriptor)
        if (
            not stat.S_ISREG(metadata.st_mode)
            or metadata.st_uid != os.getuid()
            or metadata.st_mode & 0o077
            or not 1 <= metadata.st_size <= 128
        ):
            raise PermissionError(
                "external-control capability must be a private owned regular file"
            )
        raw = os.read(descriptor, 129)
    finally:
        os.close(descriptor)
    if not raw or len(raw) > 128:
        raise RuntimeError("external-control capability file size is invalid")
    try:
        capability = raw.decode("ascii").strip()
    except UnicodeDecodeError as exc:
        raise RuntimeError(
            "external-control capability file is malformed"
        ) from exc
    if _CAPABILITY_RE.fullmatch(capability) is None:
        raise RuntimeError("external-control capability file is malformed")
    return capability


class MatrixControlResponseError(RuntimeError):
    """Typed negative response from the authenticated control endpoint."""

    def __init__(self, code: str, message: str) -> None:
        self.code = code
        self.message = message
        super().__init__(f"{code}: {message}")


class MatrixCommandOutcomeUnknownError(RuntimeError):
    """Admitted command lacks a terminal receipt and may have lost authority."""

    def __init__(self, message: str, *, lease_available: bool) -> None:
        if type(lease_available) is not bool:
            raise TypeError("lease availability must be boolean")
        self.lease_available = lease_available
        super().__init__(message)


def default_endpoint(profile: str) -> tuple[Path, Path]:
    if not isinstance(profile, str) or not re.fullmatch(r"[A-Za-z0-9_.-]{1,64}", profile):
        raise ValueError("profile must contain only letters, digits, dot, underscore, or dash")
    runtime_root = Path(
        os.environ.get("XDG_RUNTIME_DIR", tempfile.gettempdir())
    ) / f"matrix-external-control-{os.getuid()}"
    return runtime_root / f"{profile}.sock", runtime_root / f"{profile}.cap"


def _finite(value: object, *, name: str, minimum: float, maximum: float) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be numeric")
    number = float(value)
    if not math.isfinite(number) or not minimum <= number <= maximum:
        raise ValueError(f"{name} must be finite and in [{minimum:g}, {maximum:g}]")
    return number


class MatrixControlClient:
    def __init__(
        self,
        endpoint: Path,
        capability_file: Path,
        *,
        timeout_seconds: float = 1.0,
    ) -> None:
        if not endpoint.is_absolute() or not capability_file.is_absolute():
            raise ValueError("endpoint and capability file must be absolute")
        self.endpoint = endpoint
        self.capability_file = capability_file
        self.receipt_directory = capability_file.with_name(
            f"{capability_file.name}.receipts"
        )
        self.timeout_seconds = _finite(
            timeout_seconds,
            name="timeout_seconds",
            minimum=0.05,
            maximum=10.0,
        )
        self._socket: socket.socket | None = None
        self._sequence = 0
        self._capability: str | None = None
        self._authority_epoch: int | None = None

    def connect(self) -> None:
        if self._socket is not None:
            return
        capability = _read_capability(self.capability_file)
        connection = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        connection.settimeout(self.timeout_seconds)
        try:
            connection.connect(os.fspath(self.endpoint))
        except Exception:
            connection.close()
            raise
        self._capability = capability
        self._socket = connection

    def request(
        self,
        operation: str,
        payload: dict[str, object],
    ) -> dict[str, object]:
        if self._socket is None or self._capability is None:
            raise RuntimeError("external-control client is not connected")
        self._sequence += 1
        packet = {
            "protocol": PROTOCOL,
            "kind": "request",
            "sequence": self._sequence,
            "capability": self._capability,
            "operation": operation,
            "payload": payload,
        }
        encoded = json.dumps(
            packet,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
        sent = self._socket.send(encoded)
        if sent != len(encoded):
            raise RuntimeError("partial external-control request")
        raw = self._socket.recv(MAX_PACKET_BYTES + 1)
        if not raw or len(raw) > MAX_PACKET_BYTES:
            raise RuntimeError("external-control response size is invalid")
        try:
            response = json.loads(raw)
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise RuntimeError("external-control response is invalid JSON") from exc
        if (
            not isinstance(response, dict)
            or response.get("protocol") != PROTOCOL
            or response.get("kind") != "response"
            or response.get("sequence") != self._sequence
            or type(response.get("ok")) is not bool
            or not isinstance(response.get("code"), str)
            or not isinstance(response.get("message"), str)
        ):
            raise RuntimeError("external-control response schema is invalid")
        if not response["ok"]:
            raise MatrixControlResponseError(
                response["code"],
                response["message"],
            )
        return response

    def acquire(self) -> tuple[str, float]:
        response = self.request("lease.acquire", {})
        data = response.get("data")
        if not isinstance(data, dict) or not isinstance(data.get("lease_id"), str):
            raise RuntimeError("lease response is malformed")
        deadman = _finite(
            data.get("deadman_seconds"),
            name="deadman_seconds",
            minimum=0.01,
            maximum=0.15,
        )
        authority_epoch = data.get("authority_epoch")
        if (
            isinstance(authority_epoch, bool)
            or not isinstance(authority_epoch, int)
            or authority_epoch <= 0
        ):
            raise RuntimeError("lease response authority epoch is malformed")
        self._authority_epoch = authority_epoch
        return data["lease_id"], deadman

    def refresh(self, lease_id: str) -> None:
        response = self.request("lease.renew", {"lease_id": lease_id})
        data = response.get("data")
        if (
            not isinstance(data, dict)
            or data.get("lease_id") != lease_id
            or data.get("authority_epoch") != self._authority_epoch
        ):
            raise RuntimeError("lease renewal changed external authority")

    def replace(self, lease_id: str, state: ExternalInputState) -> None:
        self.request(
            "input.replace",
            {"lease_id": lease_id, "state": state.to_mapping()},
        )

    def command(self, lease_id: str, command: str) -> dict[str, object]:
        return self.request(
            "command.submit",
            {"lease_id": lease_id, "command": command},
        )

    def command_result(self, command_id: str) -> dict[str, object]:
        response = self.request("command.result", {"command_id": command_id})
        data = response.get("data")
        if (
            not isinstance(data, dict)
            or data.get("command_id") != command_id
            or type(data.get("terminal")) is not bool
            or type(data.get("authority_revoked")) is not bool
            or not isinstance(data.get("state"), str)
        ):
            raise RuntimeError("external command receipt is malformed")
        return data

    def persistent_command_result(
        self,
        command_id: str,
    ) -> dict[str, object] | None:
        if re.fullmatch(r"[0-9a-f]{32}", command_id) is None:
            raise ValueError("command_id is invalid")
        path = self.receipt_directory / f"{command_id}.json"
        if path.is_symlink() or not path.is_file():
            return None
        metadata = path.stat(follow_symlinks=False)
        if metadata.st_uid != os.getuid() or metadata.st_mode & 0o077:
            raise RuntimeError("persistent command receipt is not private")
        raw = path.read_bytes()
        if not raw or len(raw) > MAX_PACKET_BYTES:
            raise RuntimeError("persistent command receipt size is invalid")
        try:
            payload = json.loads(raw)
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise RuntimeError("persistent command receipt is invalid JSON") from exc
        if (
            not isinstance(payload, dict)
            or payload.get("schema") != COMMAND_RECEIPT_SCHEMA
            or isinstance(payload.get("written_unix_ns"), bool)
            or not isinstance(payload.get("written_unix_ns"), int)
        ):
            raise RuntimeError("persistent command receipt schema is invalid")
        receipt = payload.get("receipt")
        if (
            not isinstance(receipt, dict)
            or receipt.get("command_id") != command_id
            or receipt.get("terminal") is not True
            or type(receipt.get("authority_revoked")) is not bool
            or not isinstance(receipt.get("state"), str)
        ):
            raise RuntimeError("persistent command receipt identity is invalid")
        return receipt

    def release(self, lease_id: str) -> None:
        self.request("lease.release", {"lease_id": lease_id})

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close()
            self._socket = None
        self._capability = None
        self._authority_epoch = None

    def __enter__(self) -> "MatrixControlClient":
        self.connect()
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()


def _state_with_keyboard(
    key: str | None,
    modifiers: tuple[str, ...],
) -> ExternalInputState:
    mapping = ExternalInputState.neutral().to_mapping()
    if key is not None:
        mapping["keyboard"][key] = True
    for modifier in modifiers:
        mapping["keyboard"][modifier] = True
    return ExternalInputState.from_mapping(mapping)


def _state_with_mouse(dx: float, dy: float, button: str | None) -> ExternalInputState:
    mapping = ExternalInputState.neutral().to_mapping()
    mapping["mouse"]["dx"] = dx
    mapping["mouse"]["dy"] = dy
    if button is not None:
        mapping["mouse"]["buttons"][button] = True
    return ExternalInputState.from_mapping(mapping)


def _state_with_gamepad(args: argparse.Namespace) -> ExternalInputState:
    mapping = ExternalInputState.neutral().to_mapping()
    mapping["gamepad"]["connected"] = True
    for name in GAMEPAD_AXIS_FIELDS:
        mapping["gamepad"]["axes"][name] = getattr(args, name)
    return ExternalInputState.from_mapping(mapping)


def _wait_with_lease_refresh(
    client: MatrixControlClient,
    lease_id: str,
    *,
    seconds: float,
    refresh_seconds: float,
    clock=time.monotonic,
    sleeper=time.sleep,
) -> None:
    deadline = clock() + seconds
    next_refresh = clock() + refresh_seconds
    while True:
        now = clock()
        remaining = deadline - now
        if remaining <= 0.0:
            return
        sleeper(max(0.0, min(next_refresh - now, remaining)))
        now = clock()
        if now >= deadline:
            return
        client.refresh(lease_id)
        next_refresh += refresh_seconds
        if next_refresh <= now:
            next_refresh = now + refresh_seconds


def _hold_state(
    client: MatrixControlClient,
    lease_id: str,
    state: ExternalInputState,
    *,
    seconds: float,
    refresh_seconds: float,
    clock=time.monotonic,
    sleeper=time.sleep,
) -> None:
    # A held input is immutable until the next explicit segment.  Replacing the
    # full state every refresh needlessly amplifies provider/UI telemetry I/O;
    # renew the lease instead and keep the configured deadman authoritative.
    client.replace(lease_id, state)
    _wait_with_lease_refresh(
        client,
        lease_id,
        seconds=seconds,
        refresh_seconds=refresh_seconds,
        clock=clock,
        sleeper=sleeper,
    )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--profile",
        default=os.environ.get("MATRIX_PROFILE", "local"),
    )
    parser.add_argument("--socket", type=Path)
    parser.add_argument("--capability-file", type=Path)
    parser.add_argument("--timeout", type=float, default=1.0)
    subparsers = parser.add_subparsers(dest="action", required=True)
    subparsers.add_parser("status", help="show broker status")

    key = subparsers.add_parser("key", help="hold or double-tap a virtual key")
    key.add_argument("key", choices=KEYBOARD_FIELDS)
    key.add_argument(
        "--modifier",
        action="append",
        choices=("ctrl", "alt", "shift"),
        default=[],
    )
    key.add_argument("--seconds", type=float, default=0.50)
    key.add_argument("--double", action="store_true")
    key.add_argument("--tap-gap", type=float, default=0.08)

    mouse = subparsers.add_parser("mouse", help="send a virtual mouse delta/button")
    mouse.add_argument("--dx", type=float, default=0.0)
    mouse.add_argument("--dy", type=float, default=0.0)
    mouse.add_argument("--button", choices=("left", "middle", "right"))
    mouse.add_argument("--seconds", type=float, default=0.08)

    gamepad = subparsers.add_parser("gamepad", help="hold virtual gamepad axes")
    for name in GAMEPAD_AXIS_FIELDS:
        gamepad.add_argument(f"--{name.replace('_', '-')}", type=float, default=0.0)
    gamepad.add_argument("--seconds", type=float, default=0.50)

    command = subparsers.add_parser("command", help="queue an MC-style command")
    command.add_argument("command")
    command.add_argument("--hold-seconds", type=float, default=2.0)
    return parser.parse_args()


def _resolved_paths(args: argparse.Namespace) -> tuple[Path, Path]:
    default_socket, default_capability = default_endpoint(args.profile)
    endpoint = args.socket or Path(
        os.environ.get("MATRIX_GAME_EXTERNAL_CONTROL_SOCKET")
        or os.environ.get("MATRIX_EXTERNAL_CONTROL_SOCKET")
        or default_socket
    )
    capability = args.capability_file or Path(
        os.environ.get("MATRIX_GAME_EXTERNAL_CONTROL_CAPABILITY_FILE")
        or os.environ.get("MATRIX_EXTERNAL_CONTROL_CAPABILITY_FILE")
        or default_capability
    )
    return endpoint, capability


def _persistent_command_result_until(
    client: MatrixControlClient,
    command_id: str,
    *,
    wait_seconds: float,
    clock=time.monotonic,
    sleeper=time.sleep,
) -> dict[str, object] | None:
    """Wait briefly for an already-terminal durable command receipt."""

    deadline = clock() + wait_seconds
    while True:
        receipt = client.persistent_command_result(command_id)
        if receipt is not None:
            return receipt
        remaining = deadline - clock()
        if remaining <= 0.0:
            return None
        sleeper(min(0.02, remaining))


def _wait_for_command_terminal(
    client: MatrixControlClient,
    lease_id: str,
    command_id: str,
    *,
    hold_seconds: float,
    refresh_seconds: float,
    clock=time.monotonic,
    sleeper=time.sleep,
) -> tuple[dict[str, object], bool]:
    """Resolve one admitted command without confusing authority with outcome.

    A local override or deadman may revoke the input lease after the runtime has
    admitted the command.  The command's terminal receipt remains authoritative,
    so lease-renewal failure switches this loop to receipt-only polling instead
    of turning a possibly-completed command into a retryable-looking exception.
    """

    deadline = clock() + hold_seconds
    refresh_lease = True
    poll_failure: BaseException | None = None
    while clock() < deadline:
        try:
            receipt = client.command_result(command_id)
        except (OSError, RuntimeError) as exc:
            poll_failure = exc
            refresh_lease = False
            break
        if receipt["terminal"]:
            lease_available = bool(
                refresh_lease and receipt.get("authority_revoked") is not True
            )
            return receipt, lease_available
        if receipt.get("authority_revoked") is True:
            refresh_lease = False

        remaining = max(0.0, deadline - clock())
        if remaining <= 0.0:
            break
        sleeper(min(refresh_seconds, remaining))
        if refresh_lease:
            try:
                client.refresh(lease_id)
            except OSError as exc:
                poll_failure = exc
                refresh_lease = False
                break
            except RuntimeError as exc:
                # E_LEASE is expected after local override/deadman.  Other
                # renewal protocol failures receive the same conservative
                # treatment: stop exercising authority and retain only the
                # admitted command's receipt channel.
                poll_failure = exc
                refresh_lease = False

    try:
        persistent = _persistent_command_result_until(
            client,
            command_id,
            wait_seconds=0.50 if poll_failure is not None else 0.0,
            clock=clock,
            sleeper=sleeper,
        )
    except (OSError, RuntimeError) as exc:
        persistent = None
        if poll_failure is None:
            poll_failure = exc
        refresh_lease = False
    if persistent is not None and persistent["terminal"]:
        lease_available = bool(
            poll_failure is None
            and refresh_lease
            and persistent.get("authority_revoked") is not True
        )
        return persistent, lease_available

    detail = (
        "endpoint unavailable before terminal receipt"
        if poll_failure is not None
        else "no terminal receipt"
    )
    raise MatrixCommandOutcomeUnknownError(
        f"E_COMMAND_OUTCOME_UNKNOWN: {detail} for {command_id}",
        lease_available=bool(poll_failure is None and refresh_lease),
    ) from poll_failure


def main() -> int:
    args = _parse_args()
    endpoint, capability = _resolved_paths(args)
    with MatrixControlClient(endpoint, capability, timeout_seconds=args.timeout) as client:
        if args.action == "status":
            print(json.dumps(client.request("status.get", {}), indent=2, sort_keys=True))
            return 0

        lease_id, deadman = client.acquire()
        refresh_seconds = min(0.05, deadman / 3.0)
        neutral = ExternalInputState.neutral()
        response: dict[str, object] | None = None
        lease_available = True
        try:
            warmup_state = (
                _state_with_keyboard(None, tuple(args.modifier))
                if args.action == "key"
                else neutral
            )
            _hold_state(
                client,
                lease_id,
                warmup_state,
                seconds=_NEUTRAL_WARMUP_SECONDS,
                refresh_seconds=refresh_seconds,
            )
            if args.action == "key":
                seconds = _finite(args.seconds, name="seconds", minimum=0.01, maximum=3600.0)
                state = _state_with_keyboard(args.key, tuple(args.modifier))
                if args.double:
                    gap = _finite(args.tap_gap, name="tap_gap", minimum=0.04, maximum=0.10)
                    _hold_state(
                        client,
                        lease_id,
                        state,
                        seconds=0.04,
                        refresh_seconds=refresh_seconds,
                    )
                    modifier_only = _state_with_keyboard(
                        None,
                        tuple(args.modifier),
                    )
                    _hold_state(
                        client,
                        lease_id,
                        modifier_only,
                        seconds=gap,
                        refresh_seconds=refresh_seconds,
                    )
                _hold_state(
                    client,
                    lease_id,
                    state,
                    seconds=seconds,
                    refresh_seconds=refresh_seconds,
                )
            elif args.action == "mouse":
                dx = _finite(args.dx, name="dx", minimum=-4096.0, maximum=4096.0)
                dy = _finite(args.dy, name="dy", minimum=-4096.0, maximum=4096.0)
                seconds = _finite(args.seconds, name="seconds", minimum=0.02, maximum=10.0)
                state = _state_with_mouse(dx, dy, args.button)
                client.replace(lease_id, state)
                # Keep the one-shot delta visible for at least one 50 Hz
                # provider frame.  Renew only the lease: replacing this state
                # would apply the relative mouse delta more than once.
                visibility_seconds = min(0.04, seconds)
                _wait_with_lease_refresh(
                    client,
                    lease_id,
                    seconds=visibility_seconds,
                    refresh_seconds=refresh_seconds,
                )
                held = _state_with_mouse(0.0, 0.0, args.button)
                _hold_state(
                    client,
                    lease_id,
                    held,
                    seconds=max(0.0, seconds - visibility_seconds),
                    refresh_seconds=refresh_seconds,
                )
            elif args.action == "gamepad":
                seconds = _finite(args.seconds, name="seconds", minimum=0.01, maximum=3600.0)
                state = _state_with_gamepad(args)
                _hold_state(
                    client,
                    lease_id,
                    state,
                    seconds=seconds,
                    refresh_seconds=refresh_seconds,
                )
            else:
                assert args.action == "command"
                hold = _finite(
                    args.hold_seconds,
                    name="hold_seconds",
                    minimum=0.05,
                    maximum=30.0,
                )
                response = client.command(lease_id, args.command)
                response_data = response.get("data")
                if not isinstance(response_data, dict) or not isinstance(
                    response_data.get("command_id"), str
                ):
                    raise RuntimeError("queued command has no command_id")
                command_id = response_data["command_id"]
                receipt, lease_available = _wait_for_command_terminal(
                    client,
                    lease_id,
                    command_id,
                    hold_seconds=hold,
                    refresh_seconds=refresh_seconds,
                )
                terminal_result = receipt.get("result")
                if not isinstance(terminal_result, dict):
                    raise RuntimeError("terminal command receipt has no result")
                response = {
                    "protocol": PROTOCOL,
                    "kind": "command_terminal",
                    "command_id": command_id,
                    "receipt": receipt,
                }
                if terminal_result.get("ok") is not True:
                    code = terminal_result.get("code", "E_COMMAND_REJECTED")
                    message = terminal_result.get("message", "external command rejected")
                    raise RuntimeError(f"{code}: {message}")
            if lease_available:
                try:
                    client.replace(lease_id, neutral)
                    client.release(lease_id)
                except (
                    BrokenPipeError,
                    ConnectionError,
                    FileNotFoundError,
                    RuntimeError,
                    socket.timeout,
                ):
                    # A command may revoke authority or deliberately replace
                    # the provider endpoint after publishing its terminal
                    # receipt.  Cleanup failure cannot overwrite that outcome.
                    pass
        except BaseException as exc:
            # A deadman/local override has already zeroed state and revoked the
            # lease.  Sending neutral/release with that stale identity merely
            # creates a second E_LEASE and cannot improve safety.
            lease_lost = bool(
                not lease_available
                or (
                    isinstance(exc, MatrixCommandOutcomeUnknownError)
                    and not exc.lease_available
                )
                or (
                    isinstance(exc, MatrixControlResponseError)
                    and exc.code == "E_LEASE"
                )
            )
            if not lease_lost:
                try:
                    client.replace(lease_id, neutral)
                    client.release(lease_id)
                except Exception:
                    pass
            raise
        if response is not None:
            print(json.dumps(response, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"matrixctl ERROR {exc}", file=sys.stderr)
        raise SystemExit(2) from exc
