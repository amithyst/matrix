#!/usr/bin/env python3
"""Forward PICO controller sticks into Matrix's shared game-input state machine."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import signal
import tempfile
import time

from matrixctl import MatrixEngineInputClient


PROTOCOL = "matrix-pico-gamepad/v1"


def _axis(value: object) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("PICO axis is not finite")
    return max(-1.0, min(1.0, number))


def _pair(value: object, *, name: str) -> tuple[float, float]:
    if not hasattr(value, "__len__") or not hasattr(value, "__getitem__"):
        raise ValueError(f"{name} axis payload is not indexable")
    if len(value) < 2:  # type: ignore[arg-type]
        raise ValueError(f"{name} axis payload has fewer than two values")
    return (_axis(value[0]), _axis(value[1]))  # type: ignore[index]


def _atomic_json(path: Path | None, payload: dict[str, object]) -> None:
    if path is None:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        dir=path.parent,
        prefix=f".{path.name}.",
        delete=False,
    ) as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
        temporary = Path(stream.name)
    os.replace(temporary, path)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--engine-socket", type=Path, required=True)
    parser.add_argument("--capability-file", type=Path, required=True)
    parser.add_argument("--status-file", type=Path)
    parser.add_argument("--rate-hz", type=float, default=30.0)
    parser.add_argument("--hold-seconds", type=float, default=0.03)
    args = parser.parse_args()
    for name in ("engine_socket", "capability_file", "status_file"):
        value = getattr(args, name)
        if value is not None and not value.is_absolute():
            parser.error(f"--{name.replace('_', '-')} must be absolute")
    if not math.isfinite(args.rate_hz) or not 10.0 <= args.rate_hz <= 50.0:
        parser.error("--rate-hz must be finite and in [10, 50]")
    if not math.isfinite(args.hold_seconds) or not 0.01 <= args.hold_seconds <= 0.05:
        parser.error("--hold-seconds must be finite and in [0.01, 0.05]")
    if args.hold_seconds > 1.0 / args.rate_hz:
        parser.error("--hold-seconds must not exceed one bridge frame")
    return args


def main() -> int:
    args = _parse_args()
    import xrobotoolkit_sdk as xrt

    xrt.init()
    running = True

    def stop(_signum: int, _frame: object) -> None:
        nonlocal running
        running = False

    previous_handlers = {
        signum: signal.signal(signum, stop)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    started = time.monotonic()
    next_frame = started
    next_status = started
    frames = 0
    sdk_errors = 0
    engine_errors = 0
    last_error: str | None = None
    left = (0.0, 0.0)
    right = (0.0, 0.0)
    mapped = {
        "right": 0.0,
        "forward": 0.0,
        "look_yaw": 0.0,
        "look_pitch": 0.0,
    }
    try:
        while running:
            now = time.monotonic()
            if now < next_frame:
                time.sleep(min(next_frame - now, 0.01))
                continue
            next_frame = max(next_frame + 1.0 / args.rate_hz, now)
            try:
                left = _pair(xrt.get_left_axis(), name="left")
                right = _pair(xrt.get_right_axis(), name="right")
                last_error = None
            except Exception as exc:
                sdk_errors += 1
                left = (0.0, 0.0)
                right = (0.0, 0.0)
                last_error = f"sdk:{type(exc).__name__}:{exc}"
            mapped = {
                "right": -left[0],
                "forward": left[1],
                "look_yaw": right[0],
                "look_pitch": right[1],
            }
            client = MatrixEngineInputClient(
                args.engine_socket,
                args.capability_file,
                timeout_seconds=0.2,
            )
            try:
                client.connect()
                client.request(
                    "gamepad",
                    {
                        "axes": mapped,
                        "buttons": [],
                        "seconds": args.hold_seconds,
                    },
                )
                frames += 1
            except Exception as exc:
                engine_errors += 1
                last_error = f"engine:{type(exc).__name__}:{exc}"
                time.sleep(0.05)
            finally:
                client.close()
            now = time.monotonic()
            if now >= next_status:
                next_status = now + 0.2
                _atomic_json(
                    args.status_file,
                    {
                        "protocol": PROTOCOL,
                        "pid": os.getpid(),
                        "frames": frames,
                        "sdk_errors": sdk_errors,
                        "engine_errors": engine_errors,
                        "last_error": last_error,
                        "left_axis": list(left),
                        "right_axis": list(right),
                        "mapped_axes": mapped,
                        "elapsed_s": round(now - started, 3),
                    },
                )
    finally:
        try:
            xrt.close()
        except Exception:
            pass
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
