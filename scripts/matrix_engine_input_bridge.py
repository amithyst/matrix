#!/usr/bin/env python3
"""Capability-gated Linux uinput bridge for Matrix engine automation.

The launcher starts this helper before UE so SDL enumerates the synthetic
pointer/keyboard and gamepad during engine startup.  Opening ``/dev/uinput`` is
the only privileged operation.  The process immediately drops to the Matrix
user before binding its private Unix socket, reading requests, or emitting any
input events.
"""

from __future__ import annotations

import argparse
import ctypes
import errno
import fcntl
import hmac
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import stat
import struct
import time
from typing import Any


PROTOCOL = "matrix-engine-input/v1"
MAX_PACKET_BYTES = 4096
MAX_SECONDS = 10.0
MAX_MOUSE_DELTA = 4096
MOUSE_PRESS_LEAD_SECONDS = 0.02
_CAPABILITY_RE = re.compile(r"[0-9a-f]{64}\Z")

_UINPUT_TYPE = ord("U")
_IOC_WRITE = 1


def _ioc(direction: int, number: int, size: int = 0) -> int:
    return (
        (direction << 30)
        | (_UINPUT_TYPE << 8)
        | number
        | (size << 16)
    )


def _iow(number: int, size: int = 4) -> int:
    return _ioc(_IOC_WRITE, number, size)


_UINPUT_SETUP = struct.Struct("@HHHH80sI")
_UINPUT_ABS_SETUP = struct.Struct("@H2xiiiiii")
_INPUT_EVENT = struct.Struct("@llHHi")

UI_DEV_CREATE = _ioc(0, 1)
UI_DEV_DESTROY = _ioc(0, 2)
UI_DEV_SETUP = _iow(3, _UINPUT_SETUP.size)
UI_ABS_SETUP = _iow(4, _UINPUT_ABS_SETUP.size)
UI_SET_EVBIT = _iow(100)
UI_SET_KEYBIT = _iow(101)
UI_SET_RELBIT = _iow(102)
UI_SET_ABSBIT = _iow(103)

BUS_USB = 0x03
EV_SYN = 0x00
EV_KEY = 0x01
EV_REL = 0x02
EV_ABS = 0x03
SYN_REPORT = 0

REL_X = 0x00
REL_Y = 0x01
REL_WHEEL = 0x08
ABS_X = 0x00
ABS_Y = 0x01
ABS_RX = 0x03
ABS_RY = 0x04

BTN_LEFT = 0x110
BTN_RIGHT = 0x111
BTN_MIDDLE = 0x112
BTN_SOUTH = 0x130
BTN_EAST = 0x131
BTN_NORTH = 0x133
BTN_WEST = 0x134
BTN_TL = 0x136
BTN_TR = 0x137
BTN_SELECT = 0x13A
BTN_START = 0x13B

KEY_ESC = 1
KEY_Q = 16
KEY_W = 17
KEY_E = 18
KEY_LEFTCTRL = 29
KEY_A = 30
KEY_S = 31
KEY_D = 32
KEY_LEFTSHIFT = 42
KEY_V = 47
KEY_LEFTALT = 56

POINTER_KEY_CODES = {
    "w": KEY_W,
    "a": KEY_A,
    "s": KEY_S,
    "d": KEY_D,
    "q": KEY_Q,
    "e": KEY_E,
    "v": KEY_V,
    "ctrl": KEY_LEFTCTRL,
    "alt": KEY_LEFTALT,
    "shift": KEY_LEFTSHIFT,
    "escape": KEY_ESC,
}
MODIFIER_NAMES = frozenset(("ctrl", "alt", "shift"))
MOUSE_BUTTON_CODES = {
    "left": BTN_LEFT,
    "middle": BTN_MIDDLE,
    "right": BTN_RIGHT,
}
GAMEPAD_AXIS_CODES = {
    "right": ABS_X,
    "forward": ABS_Y,
    "look_yaw": ABS_RX,
    "look_pitch": ABS_RY,
}
GAMEPAD_BUTTON_CODES = {
    "south": BTN_SOUTH,
    "east": BTN_EAST,
    "west": BTN_WEST,
    "north": BTN_NORTH,
    "left_bumper": BTN_TL,
    "right_bumper": BTN_TR,
    "select": BTN_SELECT,
    "start": BTN_START,
}


class EngineInputError(ValueError):
    def __init__(self, code: str, message: str) -> None:
        self.code = code
        self.message = message
        super().__init__(message)


def _strict_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise EngineInputError(
                "E_JSON_DUPLICATE",
                f"duplicate field {key!r}",
            )
        result[key] = value
    return result


def _decode_packet(raw: bytes) -> dict[str, object]:
    if not raw or len(raw) > MAX_PACKET_BYTES:
        raise EngineInputError("E_PACKET_SIZE", "packet size is invalid")
    try:
        value = json.loads(
            raw.decode("utf-8"),
            object_pairs_hook=_strict_object,
            parse_constant=lambda token: (_ for _ in ()).throw(
                EngineInputError(
                    "E_JSON_NONFINITE",
                    f"invalid JSON constant {token}",
                )
            ),
        )
    except UnicodeDecodeError as exc:
        raise EngineInputError("E_JSON_ENCODING", "packet is not UTF-8") from exc
    except json.JSONDecodeError as exc:
        raise EngineInputError("E_JSON_INVALID", "packet is invalid JSON") from exc
    if not isinstance(value, dict):
        raise EngineInputError("E_SCHEMA", "packet must be an object")
    if set(value) != {
        "protocol",
        "sequence",
        "capability",
        "action",
        "payload",
    }:
        raise EngineInputError("E_SCHEMA", "packet fields are invalid")
    if value["protocol"] != PROTOCOL:
        raise EngineInputError("E_PROTOCOL", "protocol is unsupported")
    sequence = value["sequence"]
    if (
        isinstance(sequence, bool)
        or not isinstance(sequence, int)
        or not 1 <= sequence < 2**63
    ):
        raise EngineInputError("E_SEQUENCE", "sequence is invalid")
    if not isinstance(value["capability"], str):
        raise EngineInputError("E_AUTH", "capability is invalid")
    if not isinstance(value["action"], str):
        raise EngineInputError("E_ACTION", "action is invalid")
    if not isinstance(value["payload"], dict):
        raise EngineInputError("E_SCHEMA", "payload must be an object")
    return value


def _finite(
    value: object,
    *,
    name: str,
    minimum: float,
    maximum: float,
) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise EngineInputError("E_VALUE", f"{name} must be numeric")
    number = float(value)
    if not math.isfinite(number) or not minimum <= number <= maximum:
        raise EngineInputError(
            "E_VALUE",
            f"{name} must be finite and in [{minimum:g}, {maximum:g}]",
        )
    return number


def _exact(payload: dict[str, object], fields: set[str]) -> None:
    if set(payload) != fields:
        raise EngineInputError("E_SCHEMA", "payload fields are invalid")


def _private_capability(path: Path, expected_uid: int) -> str:
    parent = path.parent.stat(follow_symlinks=False)
    if (
        not stat.S_ISDIR(parent.st_mode)
        or parent.st_uid != expected_uid
        or parent.st_mode & 0o077
    ):
        raise PermissionError("capability directory is not private/user-owned")
    flags = os.O_RDONLY | os.O_NONBLOCK | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(path, flags)
    try:
        metadata = os.fstat(descriptor)
        if (
            not stat.S_ISREG(metadata.st_mode)
            or metadata.st_uid != expected_uid
            or metadata.st_mode & 0o077
            or not 1 <= metadata.st_size <= 128
        ):
            raise PermissionError("capability is not a private owned file")
        raw = os.read(descriptor, 129)
    finally:
        os.close(descriptor)
    try:
        capability = raw.decode("ascii").strip()
    except UnicodeDecodeError as exc:
        raise ValueError("capability is malformed") from exc
    if _CAPABILITY_RE.fullmatch(capability) is None:
        raise ValueError("capability is malformed")
    return capability


class UInputDevice:
    def __init__(
        self,
        path: Path,
        *,
        name: str,
        product: int,
        key_codes: tuple[int, ...] = (),
        rel_codes: tuple[int, ...] = (),
        abs_codes: tuple[int, ...] = (),
    ) -> None:
        flags = os.O_WRONLY | os.O_NONBLOCK | getattr(os, "O_CLOEXEC", 0)
        self._fd = os.open(path, flags)
        self._created = False
        try:
            if key_codes:
                fcntl.ioctl(self._fd, UI_SET_EVBIT, EV_KEY)
                for code in key_codes:
                    fcntl.ioctl(self._fd, UI_SET_KEYBIT, code)
            if rel_codes:
                fcntl.ioctl(self._fd, UI_SET_EVBIT, EV_REL)
                for code in rel_codes:
                    fcntl.ioctl(self._fd, UI_SET_RELBIT, code)
            if abs_codes:
                fcntl.ioctl(self._fd, UI_SET_EVBIT, EV_ABS)
                for code in abs_codes:
                    fcntl.ioctl(self._fd, UI_SET_ABSBIT, code)
                    fcntl.ioctl(
                        self._fd,
                        UI_ABS_SETUP,
                        _UINPUT_ABS_SETUP.pack(
                            code,
                            0,
                            -32767,
                            32767,
                            16,
                            4096,
                            0,
                        ),
                    )
            encoded_name = name.encode("ascii")
            if len(encoded_name) >= 80:
                raise ValueError("uinput device name is too long")
            setup = _UINPUT_SETUP.pack(
                BUS_USB,
                0x1209,
                product,
                1,
                encoded_name,
                0,
            )
            fcntl.ioctl(self._fd, UI_DEV_SETUP, setup)
            fcntl.ioctl(self._fd, UI_DEV_CREATE)
            self._created = True
        except BaseException:
            os.close(self._fd)
            raise

    def emit(self, event_type: int, code: int, value: int) -> None:
        payload = _INPUT_EVENT.pack(0, 0, event_type, code, int(value))
        written = os.write(self._fd, payload)
        if written != len(payload):
            raise OSError(errno.EIO, "short uinput event write")

    def sync(self) -> None:
        self.emit(EV_SYN, SYN_REPORT, 0)

    def close(self) -> None:
        if self._fd < 0:
            return
        try:
            if self._created:
                fcntl.ioctl(self._fd, UI_DEV_DESTROY)
        finally:
            os.close(self._fd)
            self._fd = -1
            self._created = False


class EngineInputController:
    def __init__(self, uinput_path: Path) -> None:
        pointer_keys = tuple(
            sorted(
                {
                    *POINTER_KEY_CODES.values(),
                    *MOUSE_BUTTON_CODES.values(),
                }
            )
        )
        self._pointer_device = UInputDevice(
            uinput_path,
            name="Matrix Engine Pointer Keyboard",
            product=0x7429,
            key_codes=pointer_keys,
            rel_codes=(REL_X, REL_Y, REL_WHEEL),
        )
        try:
            self._gamepad_device = UInputDevice(
                uinput_path,
                name="Matrix Engine Gamepad",
                product=0x7430,
                key_codes=tuple(sorted(GAMEPAD_BUTTON_CODES.values())),
                abs_codes=tuple(sorted(GAMEPAD_AXIS_CODES.values())),
            )
        except BaseException:
            self._pointer_device.close()
            raise
        self._pointer_pressed: set[int] = set()
        self._gamepad_pressed: set[int] = set()
        self._axis_values = {
            code: 0 for code in GAMEPAD_AXIS_CODES.values()
        }
        self.actions = 0
        self.errors = 0
        self._stop_requested = False

    def request_stop(self) -> None:
        self._stop_requested = True

    def _sleep(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while True:
            if self._stop_requested:
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return
            time.sleep(min(remaining, 0.05))

    def _pointer_key(self, code: int, pressed: bool) -> None:
        if pressed and code not in self._pointer_pressed:
            self._pointer_device.emit(EV_KEY, code, 1)
            self._pointer_pressed.add(code)
        elif not pressed and code in self._pointer_pressed:
            self._pointer_device.emit(EV_KEY, code, 0)
            self._pointer_pressed.remove(code)

    def _gamepad_key(self, code: int, pressed: bool) -> None:
        if pressed and code not in self._gamepad_pressed:
            self._gamepad_device.emit(EV_KEY, code, 1)
            self._gamepad_pressed.add(code)
        elif not pressed and code in self._gamepad_pressed:
            self._gamepad_device.emit(EV_KEY, code, 0)
            self._gamepad_pressed.remove(code)

    def neutral(self) -> None:
        for code in tuple(self._pointer_pressed):
            self._pointer_device.emit(EV_KEY, code, 0)
        self._pointer_pressed.clear()
        self._pointer_device.sync()
        for code in tuple(self._gamepad_pressed):
            self._gamepad_device.emit(EV_KEY, code, 0)
        self._gamepad_pressed.clear()
        for code, value in tuple(self._axis_values.items()):
            if value != 0:
                self._gamepad_device.emit(EV_ABS, code, 0)
                self._axis_values[code] = 0
        self._gamepad_device.sync()

    def mouse(
        self,
        *,
        dx: int,
        dy: int,
        button: str | None,
        seconds: float,
    ) -> None:
        code = MOUSE_BUTTON_CODES.get(button) if button is not None else None
        try:
            if code is not None:
                self._pointer_key(code, True)
                # Keep the button transition in its own input report.  Xorg can
                # otherwise dispatch relative motion before the button press,
                # so UE never observes the configured look-button chord.
                self._pointer_device.sync()
                self._sleep(MOUSE_PRESS_LEAD_SECONDS)
            if dx:
                self._pointer_device.emit(EV_REL, REL_X, dx)
            if dy:
                self._pointer_device.emit(EV_REL, REL_Y, dy)
            self._pointer_device.sync()
            self._sleep(seconds)
        finally:
            if code is not None:
                self._pointer_key(code, False)
                self._pointer_device.sync()
        self.actions += 1

    def key(
        self,
        *,
        key: str,
        modifiers: tuple[str, ...],
        seconds: float,
        double: bool,
        tap_gap: float,
    ) -> None:
        key_code = POINTER_KEY_CODES[key]
        modifier_codes = tuple(POINTER_KEY_CODES[name] for name in modifiers)
        try:
            for code in modifier_codes:
                self._pointer_key(code, True)
            if double:
                self._pointer_key(key_code, True)
                self._pointer_device.sync()
                self._sleep(0.04)
                self._pointer_key(key_code, False)
                self._pointer_device.sync()
                self._sleep(tap_gap)
            self._pointer_key(key_code, True)
            self._pointer_device.sync()
            self._sleep(seconds)
        finally:
            self._pointer_key(key_code, False)
            for code in reversed(modifier_codes):
                self._pointer_key(code, False)
            self._pointer_device.sync()
        self.actions += 1

    def gamepad(
        self,
        *,
        axes: dict[str, float],
        buttons: tuple[str, ...],
        seconds: float,
    ) -> None:
        try:
            for name, code in GAMEPAD_AXIS_CODES.items():
                value = axes[name]
                if name in {"forward", "look_pitch"}:
                    value = -value
                raw = int(round(value * 32767.0))
                self._gamepad_device.emit(EV_ABS, code, raw)
                self._axis_values[code] = raw
            for name, code in GAMEPAD_BUTTON_CODES.items():
                self._gamepad_key(code, name in buttons)
            self._gamepad_device.sync()
            self._sleep(seconds)
        finally:
            for code in tuple(self._gamepad_pressed):
                self._gamepad_key(code, False)
            for code in tuple(self._axis_values):
                self._gamepad_device.emit(EV_ABS, code, 0)
                self._axis_values[code] = 0
            self._gamepad_device.sync()
        self.actions += 1

    def close(self) -> None:
        try:
            self.neutral()
        finally:
            try:
                self._gamepad_device.close()
            finally:
                self._pointer_device.close()


def _validate_mouse(payload: dict[str, object]) -> dict[str, object]:
    _exact(payload, {"dx", "dy", "button", "seconds"})
    dx_number = _finite(
        payload["dx"],
        name="dx",
        minimum=-MAX_MOUSE_DELTA,
        maximum=MAX_MOUSE_DELTA,
    )
    dy_number = _finite(
        payload["dy"],
        name="dy",
        minimum=-MAX_MOUSE_DELTA,
        maximum=MAX_MOUSE_DELTA,
    )
    button = payload["button"]
    if button is not None and button not in MOUSE_BUTTON_CODES:
        raise EngineInputError("E_VALUE", "mouse button is invalid")
    seconds = _finite(
        payload["seconds"],
        name="seconds",
        minimum=0.02,
        maximum=MAX_SECONDS,
    )
    return {
        "dx": int(round(dx_number)),
        "dy": int(round(dy_number)),
        "button": button,
        "seconds": seconds,
    }


def _validate_key(payload: dict[str, object]) -> dict[str, object]:
    _exact(
        payload,
        {"key", "modifiers", "seconds", "double", "tap_gap"},
    )
    key = payload["key"]
    if not isinstance(key, str) or key not in POINTER_KEY_CODES:
        raise EngineInputError("E_VALUE", "key is invalid")
    raw_modifiers = payload["modifiers"]
    if not isinstance(raw_modifiers, list):
        raise EngineInputError("E_VALUE", "modifiers must be a list")
    modifiers: list[str] = []
    for value in raw_modifiers:
        if (
            not isinstance(value, str)
            or value not in MODIFIER_NAMES
            or value in modifiers
        ):
            raise EngineInputError("E_VALUE", "modifier list is invalid")
        modifiers.append(value)
    seconds = _finite(
        payload["seconds"],
        name="seconds",
        minimum=0.01,
        maximum=MAX_SECONDS,
    )
    double = payload["double"]
    if type(double) is not bool:
        raise EngineInputError("E_VALUE", "double must be boolean")
    tap_gap = _finite(
        payload["tap_gap"],
        name="tap_gap",
        minimum=0.04,
        maximum=0.10,
    )
    return {
        "key": key,
        "modifiers": tuple(modifiers),
        "seconds": seconds,
        "double": double,
        "tap_gap": tap_gap,
    }


def _validate_gamepad(payload: dict[str, object]) -> dict[str, object]:
    _exact(payload, {"axes", "buttons", "seconds"})
    raw_axes = payload["axes"]
    if not isinstance(raw_axes, dict) or set(raw_axes) != set(
        GAMEPAD_AXIS_CODES
    ):
        raise EngineInputError("E_VALUE", "gamepad axes are invalid")
    axes = {
        name: _finite(
            raw_axes[name],
            name=name,
            minimum=-1.0,
            maximum=1.0,
        )
        for name in GAMEPAD_AXIS_CODES
    }
    raw_buttons = payload["buttons"]
    if not isinstance(raw_buttons, list):
        raise EngineInputError("E_VALUE", "gamepad buttons must be a list")
    buttons: list[str] = []
    for value in raw_buttons:
        if (
            not isinstance(value, str)
            or value not in GAMEPAD_BUTTON_CODES
            or value in buttons
        ):
            raise EngineInputError("E_VALUE", "gamepad button list is invalid")
        buttons.append(value)
    seconds = _finite(
        payload["seconds"],
        name="seconds",
        minimum=0.01,
        maximum=MAX_SECONDS,
    )
    return {"axes": axes, "buttons": tuple(buttons), "seconds": seconds}


def _peer_uid(connection: socket.socket) -> int:
    raw = connection.getsockopt(
        socket.SOL_SOCKET,
        socket.SO_PEERCRED,
        struct.calcsize("3i"),
    )
    _pid, uid, _gid = struct.unpack("3i", raw)
    return uid


def _response(
    *,
    sequence: int,
    ok: bool,
    code: str,
    message: str,
    data: dict[str, object] | None = None,
) -> bytes:
    return json.dumps(
        {
            "protocol": PROTOCOL,
            "sequence": sequence,
            "ok": ok,
            "code": code,
            "message": message,
            "data": data or {},
        },
        separators=(",", ":"),
        sort_keys=True,
        allow_nan=False,
    ).encode("utf-8")


def _write_ready(path: Path, socket_path: Path) -> None:
    payload = (
        json.dumps(
            {
                "protocol": PROTOCOL,
                "pid": os.getpid(),
                "socket": os.fspath(socket_path),
                "uid": os.getuid(),
            },
            separators=(",", ":"),
            sort_keys=True,
        )
        + "\n"
    ).encode("utf-8")
    descriptor = os.open(
        path,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0),
        0o600,
    )
    try:
        if os.write(descriptor, payload) != len(payload):
            raise OSError(errno.EIO, "short ready-file write")
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _drop_privileges(uid: int, gid: int) -> None:
    if os.geteuid() != 0:
        raise PermissionError("engine input bridge must start as root")
    os.setgroups([])
    os.setgid(gid)
    os.setuid(uid)
    libc = ctypes.CDLL(None, use_errno=True)
    # PR_SET_NO_NEW_PRIVS
    if libc.prctl(38, 1, 0, 0, 0) != 0:
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error))


def _set_parent_death_signal() -> None:
    libc = ctypes.CDLL(None, use_errno=True)
    # PR_SET_PDEATHSIG
    if libc.prctl(1, signal.SIGTERM, 0, 0, 0) != 0:
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error))
    if os.getppid() == 1:
        raise RuntimeError("launcher parent exited during bridge startup")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--socket", type=Path, required=True)
    parser.add_argument("--capability-file", type=Path, required=True)
    parser.add_argument("--ready-file", type=Path, required=True)
    parser.add_argument("--target-uid", type=int, required=True)
    parser.add_argument("--target-gid", type=int, required=True)
    parser.add_argument("--uinput", type=Path, default=Path("/dev/uinput"))
    parser.add_argument("--enumeration-delay-seconds", type=float, default=2.0)
    args = parser.parse_args()
    for path in (
        args.socket,
        args.capability_file,
        args.ready_file,
        args.uinput,
    ):
        if not path.is_absolute():
            parser.error("all paths must be absolute")
    if args.target_uid < 0 or args.target_gid < 0:
        parser.error("target uid/gid must be nonnegative")
    if (
        not math.isfinite(args.enumeration_delay_seconds)
        or not 0.5 <= args.enumeration_delay_seconds <= 10.0
    ):
        parser.error("enumeration delay must be in [0.5, 10]")
    if len({args.socket, args.capability_file, args.ready_file}) != 3:
        parser.error("socket, capability, and ready paths must be distinct")
    return args


def main() -> int:
    args = parse_args()
    os.umask(0o077)
    _set_parent_death_signal()
    controller = EngineInputController(args.uinput)
    listener: socket.socket | None = None
    ready_written = False
    running = True
    rejected_peers = 0
    auth_failures = 0

    def stop(_signum: int, _frame: Any) -> None:
        nonlocal running
        running = False
        controller.request_stop()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGHUP, stop)
    try:
        _drop_privileges(args.target_uid, args.target_gid)
        capability = _private_capability(
            args.capability_file,
            args.target_uid,
        )
        parent = args.socket.parent.stat(follow_symlinks=False)
        if (
            not stat.S_ISDIR(parent.st_mode)
            or parent.st_uid != args.target_uid
            or parent.st_mode & 0o077
        ):
            raise PermissionError("socket directory is not private/user-owned")
        if args.socket.exists() or args.socket.is_symlink():
            raise FileExistsError(args.socket)
        listener = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        listener.settimeout(0.2)
        listener.bind(os.fspath(args.socket))
        os.chmod(args.socket, 0o600, follow_symlinks=False)
        listener.listen(4)
        time.sleep(args.enumeration_delay_seconds)
        _write_ready(args.ready_file, args.socket)
        ready_written = True
        print(
            f"matrix-engine-input ready pid={os.getpid()} uid={os.getuid()}",
            flush=True,
        )
        while running:
            try:
                connection, _ = listener.accept()
            except socket.timeout:
                continue
            except InterruptedError:
                continue
            with connection:
                connection.settimeout(0.5)
                sequence = 0
                try:
                    if _peer_uid(connection) != args.target_uid:
                        rejected_peers += 1
                        raise EngineInputError(
                            "E_PEER",
                            "peer uid is not authorized",
                        )
                    packet = _decode_packet(
                        connection.recv(MAX_PACKET_BYTES + 1)
                    )
                    sequence = int(packet["sequence"])
                    if not hmac.compare_digest(
                        packet["capability"],
                        capability,
                    ):
                        auth_failures += 1
                        raise EngineInputError(
                            "E_AUTH",
                            "capability is invalid",
                        )
                    action = packet["action"]
                    payload = packet["payload"]
                    assert isinstance(action, str)
                    assert isinstance(payload, dict)
                    if action == "status":
                        _exact(payload, set())
                        data = {
                            "pid": os.getpid(),
                            "uid": os.getuid(),
                            "actions": controller.actions,
                            "errors": controller.errors,
                            "auth_failures": auth_failures,
                            "rejected_peers": rejected_peers,
                            "pointer_device": "Matrix Engine Pointer Keyboard",
                            "gamepad_device": "Matrix Engine Gamepad",
                        }
                        result = _response(
                            sequence=sequence,
                            ok=True,
                            code="OK_STATUS",
                            message="engine input bridge is ready",
                            data=data,
                        )
                    elif action == "mouse":
                        values = _validate_mouse(payload)
                        controller.mouse(**values)
                        result = _response(
                            sequence=sequence,
                            ok=True,
                            code="OK_MOUSE",
                            message="engine mouse input completed",
                        )
                    elif action == "key":
                        values = _validate_key(payload)
                        controller.key(**values)
                        result = _response(
                            sequence=sequence,
                            ok=True,
                            code="OK_KEY",
                            message="engine key input completed",
                        )
                    elif action == "gamepad":
                        values = _validate_gamepad(payload)
                        controller.gamepad(**values)
                        result = _response(
                            sequence=sequence,
                            ok=True,
                            code="OK_GAMEPAD",
                            message="engine gamepad input completed",
                        )
                    else:
                        raise EngineInputError(
                            "E_ACTION",
                            "action is unsupported",
                        )
                except EngineInputError as exc:
                    controller.errors += 1
                    result = _response(
                        sequence=sequence,
                        ok=False,
                        code=exc.code,
                        message=exc.message,
                    )
                except (
                    BrokenPipeError,
                    ConnectionError,
                    OSError,
                    socket.timeout,
                ) as exc:
                    controller.errors += 1
                    try:
                        controller.neutral()
                    except OSError:
                        running = False
                    if isinstance(exc, OSError) and exc.errno in {
                        errno.ENODEV,
                        errno.EBADF,
                        errno.EIO,
                    }:
                        running = False
                    continue
                try:
                    connection.send(result)
                except (BrokenPipeError, ConnectionError, OSError):
                    pass
    finally:
        if listener is not None:
            listener.close()
        try:
            controller.close()
        finally:
            for path in (
                args.ready_file if ready_written else None,
                args.socket,
            ):
                if path is None:
                    continue
                try:
                    path.unlink()
                except FileNotFoundError:
                    pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
