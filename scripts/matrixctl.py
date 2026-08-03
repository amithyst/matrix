#!/usr/bin/env python3
"""Small authenticated client for Matrix's engine-input bridge.

This c242 backport intentionally keeps only the engine-input client needed by
the keyboard arrow camera-look path.  The newer mainline ``matrixctl`` also
contains provider-side external-control helpers, but those depend on a broader
runtime surface that is deliberately not part of this stable SONIC branch.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import re
import socket
import stat
import tempfile


_CAPABILITY_RE = re.compile(r"[0-9a-f]{64}\Z")
ENGINE_INPUT_PROTOCOL = "matrix-engine-input/v1"
ENGINE_INPUT_MAX_PACKET_BYTES = 4096


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
                "engine-input capability must be a private owned regular file"
            )
        raw = os.read(descriptor, 129)
    finally:
        os.close(descriptor)
    if not raw or len(raw) > 128:
        raise RuntimeError("engine-input capability file size is invalid")
    try:
        capability = raw.decode("ascii").strip()
    except UnicodeDecodeError as exc:
        raise RuntimeError(
            "engine-input capability file is malformed"
        ) from exc
    if _CAPABILITY_RE.fullmatch(capability) is None:
        raise RuntimeError("engine-input capability file is malformed")
    return capability


def default_engine_endpoint(profile: str) -> tuple[Path, Path]:
    if not isinstance(profile, str) or not re.fullmatch(
        r"[A-Za-z0-9_.-]{1,64}",
        profile,
    ):
        raise ValueError(
            "profile must contain only letters, digits, dot, underscore, or dash"
        )
    runtime_root = Path(
        os.environ.get("XDG_RUNTIME_DIR", tempfile.gettempdir())
    ) / f"matrix-engine-input-{os.getuid()}"
    return runtime_root / f"{profile}.sock", runtime_root / f"{profile}.cap"


def _finite(value: object, *, name: str, minimum: float, maximum: float) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be numeric")
    number = float(value)
    if not math.isfinite(number) or not minimum <= number <= maximum:
        raise ValueError(f"{name} must be finite and in [{minimum:g}, {maximum:g}]")
    return number


class MatrixEngineInputClient:
    """One-request client for the pre-UE uinput/XTEST bridge."""

    def __init__(
        self,
        endpoint: Path,
        capability_file: Path,
        *,
        timeout_seconds: float = 10.0,
    ) -> None:
        if not endpoint.is_absolute() or not capability_file.is_absolute():
            raise ValueError("engine endpoint and capability must be absolute")
        self.endpoint = endpoint
        self.capability_file = capability_file
        self.timeout_seconds = _finite(
            timeout_seconds,
            name="timeout_seconds",
            minimum=0.05,
            maximum=30.0,
        )
        self._socket: socket.socket | None = None
        self._capability: str | None = None
        self._sequence = 0

    def connect(self) -> None:
        if self._socket is not None:
            return
        capability = _read_capability(self.capability_file)
        connection = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        connection.settimeout(self.timeout_seconds)
        try:
            connection.connect(os.fspath(self.endpoint))
        except BaseException:
            connection.close()
            raise
        self._socket = connection
        self._capability = capability

    def request(
        self,
        action: str,
        payload: dict[str, object],
    ) -> dict[str, object]:
        if self._socket is None or self._capability is None:
            raise RuntimeError("engine-input client is not connected")
        self._sequence += 1
        encoded = json.dumps(
            {
                "protocol": ENGINE_INPUT_PROTOCOL,
                "sequence": self._sequence,
                "capability": self._capability,
                "action": action,
                "payload": payload,
            },
            separators=(",", ":"),
            sort_keys=True,
            allow_nan=False,
        ).encode("utf-8")
        if len(encoded) > ENGINE_INPUT_MAX_PACKET_BYTES:
            raise ValueError("engine-input request is too large")
        sent = self._socket.send(encoded)
        if sent != len(encoded):
            raise RuntimeError("partial engine-input request")
        raw = self._socket.recv(ENGINE_INPUT_MAX_PACKET_BYTES + 1)
        if not raw or len(raw) > ENGINE_INPUT_MAX_PACKET_BYTES:
            raise RuntimeError("engine-input response size is invalid")
        try:
            response = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise RuntimeError("engine-input response is malformed") from exc
        if (
            not isinstance(response, dict)
            or set(response)
            != {
                "protocol",
                "sequence",
                "ok",
                "code",
                "message",
                "data",
            }
            or response.get("protocol") != ENGINE_INPUT_PROTOCOL
            or response.get("sequence") != self._sequence
            or type(response.get("ok")) is not bool
            or not isinstance(response.get("code"), str)
            or not isinstance(response.get("message"), str)
            or not isinstance(response.get("data"), dict)
        ):
            raise RuntimeError("engine-input response schema is invalid")
        if response["ok"] is not True:
            raise RuntimeError(f"{response['code']}: {response['message']}")
        return response

    def close(self) -> None:
        connection = self._socket
        self._socket = None
        self._capability = None
        if connection is not None:
            connection.close()

    def __enter__(self) -> "MatrixEngineInputClient":
        self.connect()
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()
