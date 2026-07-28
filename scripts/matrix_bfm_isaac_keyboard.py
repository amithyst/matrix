#!/usr/bin/env python3
"""Forward focused Matrix X11 key events to Leo's Unix keyboard adapter."""

from __future__ import annotations

import argparse
import ctypes
import ctypes.util
import json
import math
import os
from pathlib import Path
import queue
import re
import signal
import socket
import subprocess
import threading
import time
from typing import Iterable, Iterator


KEY_SYMBOLS = {
    "w": "W",
    "s": "S",
    "a": "A",
    "d": "D",
    "q": "Q",
    "e": "E",
    "r": "R",
    "c": "C",
    "space": "SPACE",
    "BackSpace": "BACKSPACE",
    "Escape": "ESCAPE",
    "Shift_L": "LEFT_SHIFT",
    "Shift_R": "RIGHT_SHIFT",
    "Left": "LEFT",
    "Right": "RIGHT",
    "Up": "UP",
    "Down": "DOWN",
}

ARROW_KEYS = frozenset(("LEFT", "RIGHT", "UP", "DOWN"))


class ArrowLookIntegrator:
    """Convert held arrow keys into smooth integer X11 pointer deltas."""

    def __init__(self) -> None:
        self._residual_x = 0.0
        self._residual_y = 0.0

    def reset(self) -> None:
        self._residual_x = 0.0
        self._residual_y = 0.0

    def update(
        self, held: set[str], *, dt_s: float, pixels_per_second: float
    ) -> tuple[int, int]:
        if not math.isfinite(dt_s) or dt_s < 0.0:
            raise ValueError("camera look dt must be finite and non-negative")
        if not math.isfinite(pixels_per_second) or pixels_per_second <= 0.0:
            raise ValueError("camera look speed must be positive and finite")
        yaw = int("RIGHT" in held) - int("LEFT" in held)
        pitch = int("DOWN" in held) - int("UP" in held)
        if yaw == 0 and pitch == 0:
            self.reset()
            return (0, 0)
        pixels = min(dt_s, 0.05) * pixels_per_second
        if yaw:
            self._residual_x += yaw * pixels
        else:
            self._residual_x = 0.0
        if pitch:
            self._residual_y += pitch * pixels
        else:
            self._residual_y = 0.0
        dx = math.trunc(self._residual_x)
        dy = math.trunc(self._residual_y)
        self._residual_x -= dx
        self._residual_y -= dy
        return (dx, dy)


class XTestCameraDrag:
    """Drive Matrix's native left-button camera drag through XTEST."""

    def __init__(self, display: str) -> None:
        x11_name = ctypes.util.find_library("X11") or "libX11.so.6"
        xtst_name = ctypes.util.find_library("Xtst") or "libXtst.so.6"
        self._x11 = ctypes.CDLL(x11_name)
        self._xtst = ctypes.CDLL(xtst_name)
        self._x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        self._x11.XOpenDisplay.restype = ctypes.c_void_p
        self._x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
        self._x11.XCloseDisplay.restype = ctypes.c_int
        self._x11.XFlush.argtypes = [ctypes.c_void_p]
        self._x11.XFlush.restype = ctypes.c_int
        self._xtst.XTestQueryExtension.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_int),
        ]
        self._xtst.XTestQueryExtension.restype = ctypes.c_int
        self._xtst.XTestFakeButtonEvent.argtypes = [
            ctypes.c_void_p,
            ctypes.c_uint,
            ctypes.c_int,
            ctypes.c_ulong,
        ]
        self._xtst.XTestFakeButtonEvent.restype = ctypes.c_int
        self._xtst.XTestFakeRelativeMotionEvent.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_ulong,
        ]
        self._xtst.XTestFakeRelativeMotionEvent.restype = ctypes.c_int
        self._display = self._x11.XOpenDisplay(display.encode("utf-8"))
        if not self._display:
            raise RuntimeError(f"XTEST could not open display {display}")
        values = [ctypes.c_int() for _ in range(4)]
        if not self._xtst.XTestQueryExtension(
            self._display, *(ctypes.byref(value) for value in values)
        ):
            self.close()
            raise RuntimeError("XTEST extension is unavailable")
        self._pressed = False

    def set_pressed(self, pressed: bool) -> None:
        if pressed == self._pressed:
            return
        if not self._xtst.XTestFakeButtonEvent(
            self._display, 1, int(pressed), 0
        ):
            raise RuntimeError("XTEST camera button injection failed")
        self._x11.XFlush(self._display)
        self._pressed = pressed

    def move(self, dx: int, dy: int) -> None:
        if not dx and not dy:
            return
        if not self._xtst.XTestFakeRelativeMotionEvent(
            self._display, int(dx), int(dy), 0
        ):
            raise RuntimeError("XTEST camera motion injection failed")
        self._x11.XFlush(self._display)

    def close(self) -> None:
        display = getattr(self, "_display", None)
        if display:
            if getattr(self, "_pressed", False):
                self._xtst.XTestFakeButtonEvent(display, 1, 0, 0)
                self._x11.XFlush(display)
            self._x11.XCloseDisplay(display)
            self._display = None
            self._pressed = False


def parse_xmodmap(
    lines: Iterable[str], *, include_escape: bool = True
) -> dict[int, str]:
    mapping: dict[int, str] = {}
    pattern = re.compile(r"^keycode\s+(\d+)\s+=\s+(\S+)")
    for line in lines:
        match = pattern.match(line.strip())
        if match is None:
            continue
        key = KEY_SYMBOLS.get(match.group(2))
        if key is not None:
            if key == "ESCAPE" and not include_escape:
                continue
            mapping[int(match.group(1))] = key
    return mapping


def parse_xinput_events(lines: Iterable[str]) -> Iterator[tuple[int, bool]]:
    pressed: bool | None = None
    event_pattern = re.compile(r"\(RawKey(Press|Release)\)")
    detail_pattern = re.compile(r"^detail:\s*(\d+)")
    for line in lines:
        event = event_pattern.search(line)
        if event is not None:
            pressed = event.group(1) == "Press"
            continue
        detail = detail_pattern.match(line.strip())
        if detail is not None and pressed is not None:
            yield int(detail.group(1)), pressed
            pressed = None


def parse_active_window_id(output: str) -> str | None:
    match = re.search(r"window id #\s*(0x[0-9a-fA-F]+)", output)
    if match is None or match.group(1) == "0x0":
        return None
    return match.group(1)


def parse_window_pid(output: str) -> int | None:
    match = re.search(r"=\s*(\d+)\s*$", output.strip())
    return None if match is None else int(match.group(1))


def active_window_pid(environment: dict[str, str]) -> int | None:
    try:
        root = subprocess.run(
            ("xprop", "-root", "_NET_ACTIVE_WINDOW"),
            env=environment,
            check=True,
            capture_output=True,
            text=True,
            timeout=2,
        )
        window_id = parse_active_window_id(root.stdout)
        if window_id is None:
            return None
        window = subprocess.run(
            ("xprop", "-id", window_id, "_NET_WM_PID"),
            env=environment,
            check=True,
            capture_output=True,
            text=True,
            timeout=2,
        )
        return parse_window_pid(window.stdout)
    except (OSError, subprocess.SubprocessError):
        return None


def process_belongs_to_root(
    pid: int, allowed_root: Path, *, proc_root: Path = Path("/proc")
) -> bool:
    allowed = allowed_root.resolve()
    executable_link = proc_root / str(int(pid)) / "exe"
    try:
        executable = executable_link.resolve(strict=True)
    except OSError:
        return False
    return (
        executable.name.startswith("zsibot_mujoco_ue")
        and (executable == allowed or allowed in executable.parents)
    )


def wait_for_socket(path: Path, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if path.is_socket():
            return
        time.sleep(0.05)
    raise TimeoutError(f"keyboard socket did not appear: {path}")


def send_event(sender: socket.socket, path: Path, key: str, pressed: bool) -> None:
    sender.sendto(
        json.dumps(
            {"key": key, "pressed": pressed},
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8"),
        str(path),
    )


def run_bridge(
    socket_path: Path,
    *,
    display: str,
    xauthority: str,
    allowed_process_root: Path,
    wait_s: float,
    camera_look_backend: str,
    camera_look_pixels_per_second: float,
    forward_escape: bool,
) -> int:
    if camera_look_backend not in ("off", "xtest"):
        raise ValueError("camera look backend must be off or xtest")
    if (
        not math.isfinite(camera_look_pixels_per_second)
        or camera_look_pixels_per_second <= 0.0
    ):
        raise ValueError("camera look speed must be positive and finite")
    wait_for_socket(socket_path, wait_s)
    environment = {**os.environ, "DISPLAY": display}
    if xauthority:
        environment["XAUTHORITY"] = xauthority
    keymap_result = subprocess.run(
        ("xmodmap", "-pke"),
        env=environment,
        check=True,
        capture_output=True,
        text=True,
        timeout=10,
    )
    keymap = parse_xmodmap(
        keymap_result.stdout.splitlines(), include_escape=forward_escape
    )
    if not keymap:
        raise RuntimeError("X keymap contains no supported BFM controls")

    camera_drag = (
        XTestCameraDrag(display) if camera_look_backend == "xtest" else None
    )
    if camera_drag is not None:
        print(
            "[INFO] XTEST arrow camera look ready "
            f"pixels_per_second={camera_look_pixels_per_second:g}",
            flush=True,
        )

    sender = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        process = subprocess.Popen(
            ("stdbuf", "-oL", "-eL", "xinput", "test-xi2", "--root"),
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
    except BaseException:
        sender.close()
        if camera_drag is not None:
            camera_drag.close()
        raise
    event_queue: queue.Queue[tuple[int, bool] | None] = queue.Queue()

    def read_events() -> None:
        assert process.stdout is not None
        try:
            for event in parse_xinput_events(process.stdout):
                event_queue.put(event)
        finally:
            event_queue.put(None)

    reader = threading.Thread(
        target=read_events, name="matrix-bfm-isaac-xinput", daemon=True
    )
    reader.start()
    stop_requested = False
    previous_handlers: dict[int, object] = {}

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        previous_handlers[signum] = signal.signal(signum, request_stop)

    pressed_keys: set[str] = set()
    look_integrator = ArrowLookIntegrator()
    look_period_s = 1.0 / 30.0
    next_look_tick = time.monotonic()
    last_look_tick = next_look_tick

    def release_pressed() -> None:
        for key in sorted(pressed_keys):
            send_event(sender, socket_path, key, False)
        pressed_keys.clear()

    print(
        "matrix-bfm-isaac-keyboard ready "
        f"display={display} socket={socket_path} keys={sorted(set(keymap.values()))}",
        flush=True,
    )
    try:
        focus_allowed = False
        next_focus_check = 0.0
        while not stop_requested:
            now = time.monotonic()
            if now >= next_focus_check:
                focused_pid = active_window_pid(environment)
                next_focus_allowed = (
                    focused_pid is not None
                    and process_belongs_to_root(focused_pid, allowed_process_root)
                )
                if focus_allowed and not next_focus_allowed:
                    release_pressed()
                focus_allowed = next_focus_allowed
                next_focus_check = now + 0.10
            if now >= next_look_tick:
                active_arrows = pressed_keys.intersection(ARROW_KEYS)
                enabled = bool(camera_drag is not None and focus_allowed and active_arrows)
                if camera_drag is not None:
                    camera_drag.set_pressed(enabled)
                if enabled:
                    dx, dy = look_integrator.update(
                        pressed_keys,
                        dt_s=max(0.0, now - last_look_tick),
                        pixels_per_second=camera_look_pixels_per_second,
                    )
                    camera_drag.move(dx, dy)
                else:
                    look_integrator.reset()
                last_look_tick = now
                next_look_tick = now + look_period_s
            try:
                event = event_queue.get(
                    timeout=max(
                        0.0,
                        min(next_focus_check, next_look_tick) - time.monotonic(),
                    )
                )
            except queue.Empty:
                if process.poll() is not None:
                    break
                continue
            if event is None:
                break
            keycode, pressed = event
            if not focus_allowed:
                continue
            key = keymap.get(keycode)
            if key is None:
                continue
            if pressed:
                pressed_keys.add(key)
            else:
                pressed_keys.discard(key)
            send_event(sender, socket_path, key, pressed)
    finally:
        try:
            release_pressed()
        except OSError:
            pass
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=3)
        reader.join(timeout=1)
        sender.close()
        if camera_drag is not None:
            camera_drag.close()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
    return int(process.returncode or 0)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", type=Path, required=True)
    parser.add_argument("--display", default=os.environ.get("DISPLAY", ":0"))
    parser.add_argument(
        "--xauthority",
        default=os.environ.get("XAUTHORITY", f"/run/user/{os.getuid()}/gdm/Xauthority"),
    )
    parser.add_argument("--allowed-process-root", type=Path, required=True)
    parser.add_argument("--wait", type=float, default=120.0)
    parser.add_argument(
        "--camera-look-backend", choices=("off", "xtest"), default="xtest"
    )
    parser.add_argument("--camera-look-pixels-per-second", type=float, default=600.0)
    parser.add_argument(
        "--ignore-escape",
        action="store_true",
        help="Do not forward physical Escape key events to the BFM runtime.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    if args.wait <= 0.0:
        parser.error("--wait must be positive")
    return run_bridge(
        args.socket,
        display=args.display,
        xauthority=args.xauthority,
        allowed_process_root=args.allowed_process_root,
        wait_s=args.wait,
        camera_look_backend=args.camera_look_backend,
        camera_look_pixels_per_second=args.camera_look_pixels_per_second,
        forward_escape=not args.ignore_escape,
    )


if __name__ == "__main__":
    raise SystemExit(main())
