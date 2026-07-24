#!/usr/bin/env python3
"""Own and supervise the Matrix UE process without polling a reusable PID."""

from __future__ import annotations

import argparse
from functools import partial
import importlib
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import tempfile
import time
from typing import Any, BinaryIO, Callable

from exec_with_parent_death_signal import _arm_parent_death_signal


UNKNOWN_EXIT_CODE = 255
SPAWN_FAILURE_EXIT_CODE = 127
CAMERA_SAMPLE_INTERVAL_SECONDS = 0.02
CAMERA_BIND_TIMEOUT_SECONDS = 15.0


class _CameraProbeRuntime:
    """Own one final-POV probe and its single-writer state file.

    Probe and state failures are control-data failures, not permission to kill
    UE.  Every service call therefore either writes the sampled final POV or a
    fresh invalid record and returns to the supervisor loop.
    """

    def __init__(
        self,
        module: Any,
        probe: Any | None,
        writer: Any,
        *,
        initialization_error: str | None = None,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        self._module = module
        self._probe = probe
        self._writer = writer
        self._monotonic_ns = monotonic_ns
        self.bound = False
        self.closed = False
        self.initialization_error = initialization_error
        self._published_valid_sample = False
        self._last_diagnostic: str | None = None
        if initialization_error is not None:
            self._diagnose(initialization_error)

    @classmethod
    def open(cls, state_file: Path, layout_file: Path) -> "_CameraProbeRuntime":
        module = importlib.import_module("matrix_ue_camera_probe")
        writer = module.CameraStateWriter(state_file)
        try:
            layout = module.load_layout(layout_file)
            probe = module.UECameraProbe(layout)
        except Exception as exc:
            return cls(
                module,
                None,
                writer,
                initialization_error=(
                    f"probe_initialization_failed:{type(exc).__name__}:{exc}"
                ),
            )
        return cls(module, probe, writer)

    @property
    def probe_ready(self) -> bool:
        return self._probe is not None

    def _diagnose(self, message: str) -> None:
        if message == self._last_diagnostic:
            return
        self._last_diagnostic = message
        try:
            print(
                f"matrix-ue-supervisor CAMERA {message}",
                file=sys.stderr,
                flush=True,
            )
        except Exception:
            pass

    def _invalid_observation(self, pid: int, error_code: Any) -> Any:
        return self._module.CameraProbeObservation(
            ue_pid=pid,
            monotonic_ns=max(1, int(self._monotonic_ns())),
            valid=False,
            error_code=error_code,
        )

    def _write(self, observation: Any) -> bool:
        if self.closed:
            return False
        try:
            self._writer.write(observation)
        except Exception as exc:
            self._diagnose(f"state_write_failed:{type(exc).__name__}:{exc}")
            return False
        if bool(getattr(observation, "valid", False)):
            self._published_valid_sample = True
        return True

    def invalidate(self, pid: int, *, identity: bool = False) -> bool:
        error_code = (
            self._module.ProbeError.IDENTITY_MISMATCH
            if identity
            else self._module.ProbeError.INTERNAL
        )
        return self._write(self._invalid_observation(pid, error_code))

    def try_bind(self, pid: int) -> bool:
        if self.bound:
            return True
        if self._probe is None:
            self.invalidate(pid)
            return False
        try:
            duration_ns = self._probe.bind(pid)
        except Exception as exc:
            # The direct child initially executes env/bash before the UE script
            # execs the packaged binary under the same PID.  Identity mismatch
            # is therefore retried by the bounded supervisor startup state.
            self._diagnose(f"bind_pending:{type(exc).__name__}:{exc}")
            self.invalidate(pid, identity=True)
            return False
        self.bound = True
        self._last_diagnostic = None
        if isinstance(duration_ns, int) and duration_ns >= 0:
            self._diagnose(f"bound:identity_verification_ns={duration_ns}")
        else:
            self._diagnose("bound")
        return True

    def sample(self, pid: int) -> bool:
        if self._probe is None:
            return self.invalidate(pid)
        if not self.bound:
            return self.invalidate(pid, identity=True)
        try:
            observation = self._probe.sample(pid)
        except Exception as exc:
            self._diagnose(f"sample_failed:{type(exc).__name__}:{exc}")
            observation = self._invalid_observation(
                pid, self._module.ProbeError.INTERNAL
            )
        # A TORN_CAMERA_CACHE observation means UE updated its final POV
        # between the probe's two full-cache reads.  Once one verified sample
        # has been published, retain that record rather than replacing it with
        # a one-frame invalid value.  Its monotonic timestamp is deliberately
        # left unchanged, so CameraStateReader's freshness deadline still
        # fails closed if valid sampling does not resume promptly.
        if (
            self._published_valid_sample
            and not bool(getattr(observation, "valid", False))
            and getattr(observation, "error_code", None)
            == self._module.ProbeError.TORN_CAMERA_CACHE
        ):
            return True
        written = self._write(observation)
        if not written and bool(getattr(observation, "valid", False)):
            # If publishing a valid sample itself failed, make one immediate
            # best-effort attempt to replace it with an explicit invalid state.
            return self.invalidate(pid)
        return written

    def service_failed(self, pid: int, exc: Exception) -> None:
        self._diagnose(f"service_failed:{type(exc).__name__}:{exc}")
        self.invalidate(pid)

    def close(self, pid: int | None) -> None:
        if self.closed:
            return
        if pid is not None and pid > 0:
            self.invalidate(pid)
        try:
            self._writer.close()
        except Exception as exc:
            self._diagnose(f"state_close_failed:{type(exc).__name__}:{exc}")
        self.closed = True


def _service_camera_probe(
    runtime: _CameraProbeRuntime,
    *,
    ue_pid: int,
    now: float,
    bind_deadline: float,
) -> None:
    """Run one 50 Hz probe action without coupling failures to UE lifetime."""

    if not runtime.probe_ready:
        runtime.invalidate(ue_pid)
    elif runtime.bound:
        runtime.sample(ue_pid)
    elif now < bind_deadline:
        if runtime.try_bind(ue_pid):
            # The first post-bind sample may still be NOT_READY while UE proves
            # CameraCachePrivate.Timestamp is advancing.
            runtime.sample(ue_pid)
    else:
        runtime.invalidate(ue_pid, identity=True)


def _normalized_returncode(returncode: int) -> int:
    if returncode >= 0:
        return min(returncode, 255)
    return min(128 + abs(returncode), 255)


def _atomic_text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        dir=path.parent,
        prefix=f".{path.name}.",
        delete=False,
    ) as stream:
        stream.write(value)
        temporary = Path(stream.name)
    os.replace(temporary, path)


def _publish_failure(path: Path, returncode: int) -> None:
    payload = {
        "name": "ue",
        "exit_code": _normalized_returncode(returncode),
    }
    _atomic_text(path, json.dumps(payload, separators=(",", ":")) + "\n")


def _peek_returncode(process: subprocess.Popen[bytes]) -> int | None:
    """Observe a direct child without reaping it, keeping its PID/PGID reserved."""
    flags = os.WEXITED | os.WNOHANG | os.WNOWAIT
    result = os.waitid(os.P_PID, process.pid, flags)
    if result is None:
        return None
    if result.si_code == os.CLD_EXITED:
        return int(result.si_status)
    return -int(result.si_status)


def _signal_process_group(process: subprocess.Popen[bytes], signum: int) -> None:
    try:
        os.killpg(process.pid, signum)
    except ProcessLookupError:
        pass


def _reap_process_group(
    process: subprocess.Popen[bytes],
    *,
    returncode: int | None,
    term_grace_seconds: float,
) -> int:
    """Stop residual group members while the unreaped leader reserves its PGID."""
    _signal_process_group(process, signal.SIGTERM)
    deadline = time.monotonic() + term_grace_seconds
    observed = returncode
    while observed is None and time.monotonic() < deadline:
        time.sleep(0.02)
        observed = _peek_returncode(process)
    # The wrapper normally execs UE. If it left descendants behind, kill them
    # before reaping the group leader so its numeric PGID cannot be reused.
    _signal_process_group(process, signal.SIGKILL)
    waited = process.wait()
    return observed if observed is not None else waited


def _arm_ue_parent_death_signal(expected_parent_pid: int) -> None:
    # SIGKILL closes the last orphan window if the supervisor itself is killed.
    # Recheck after prctl to close the fork-to-prctl parent-death race.
    _arm_parent_death_signal(signal.SIGKILL)
    if os.getppid() != expected_parent_pid:
        os._exit(125)


def _control_event(stream: BinaryIO, timeout: float) -> str | None:
    readable, _, _ = select.select([stream], [], [], timeout)
    if not readable:
        return None
    data = os.read(stream.fileno(), 4096)
    if not data:
        return "eof"
    if b"stop" in data.split():
        return "stop"
    return None


def supervise(args: argparse.Namespace) -> int:
    stop_requested = False
    camera_runtime: _CameraProbeRuntime | None = None
    process: subprocess.Popen[bytes] | None = None

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    for signum in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
        signal.signal(signum, request_stop)

    _arm_parent_death_signal(signal.SIGTERM)
    if os.getppid() != args.expected_parent_pid:
        _publish_failure(args.failure_file, UNKNOWN_EXIT_CODE)
        return UNKNOWN_EXIT_CODE

    try:
        log_stream = args.log.open("ab", buffering=0)
    except OSError as exc:
        print(f"matrix-ue-supervisor ERROR opening UE log: {exc}", file=sys.stderr)
        _publish_failure(args.failure_file, SPAWN_FAILURE_EXIT_CODE)
        return SPAWN_FAILURE_EXIT_CODE

    try:
        if args.camera_state_file is not None:
            assert args.camera_layout is not None
            try:
                camera_runtime = _CameraProbeRuntime.open(
                    args.camera_state_file,
                    args.camera_layout,
                )
            except Exception as exc:
                try:
                    print(
                        "matrix-ue-supervisor ERROR initializing final-POV probe: "
                        f"{exc}",
                        file=sys.stderr,
                        flush=True,
                    )
                except Exception:
                    pass
                # Camera observation is fail-closed data, not UE lifecycle
                # authority.  A missing helper/writer leaves the state absent
                # and the provider stopped, while UE still starts normally.
                camera_runtime = None
        try:
            supervisor_pid = os.getpid()
            process = subprocess.Popen(
                args.command,
                stdin=subprocess.DEVNULL,
                stdout=log_stream,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                preexec_fn=partial(
                    _arm_ue_parent_death_signal, supervisor_pid
                ),
            )
        except (OSError, ValueError, subprocess.SubprocessError) as exc:
            print(f"matrix-ue-supervisor ERROR starting UE: {exc}", file=sys.stderr)
            _atomic_text(args.pid_file, "0\n")
            _publish_failure(args.failure_file, SPAWN_FAILURE_EXIT_CODE)
            while not stop_requested:
                event = _control_event(sys.stdin.buffer, 0.05)
                stop_requested = event in {"stop", "eof"}
            return SPAWN_FAILURE_EXIT_CODE

        _atomic_text(args.pid_file, f"{process.pid}\n")
        next_camera_sample = time.monotonic()
        camera_bind_deadline = (
            next_camera_sample + CAMERA_BIND_TIMEOUT_SECONDS
        )
        while True:
            returncode = _peek_returncode(process)
            if returncode is not None:
                if camera_runtime is not None:
                    camera_runtime.close(process.pid)
                _publish_failure(args.failure_file, returncode)
                final_code = _reap_process_group(
                    process,
                    returncode=returncode,
                    term_grace_seconds=0.2,
                )
                while not stop_requested:
                    event = _control_event(sys.stdin.buffer, 0.05)
                    stop_requested = event in {"stop", "eof"}
                return _normalized_returncode(final_code)

            now = time.monotonic()
            if camera_runtime is not None and now >= next_camera_sample:
                try:
                    _service_camera_probe(
                        camera_runtime,
                        ue_pid=process.pid,
                        now=now,
                        bind_deadline=camera_bind_deadline,
                    )
                except Exception as exc:
                    # A defensive outer boundary keeps any adapter defect from
                    # changing UE ownership/lifetime semantics.
                    camera_runtime.service_failed(process.pid, exc)
                next_camera_sample = (
                    time.monotonic() + CAMERA_SAMPLE_INTERVAL_SECONDS
                )
            control_timeout = 0.02
            if camera_runtime is not None:
                control_timeout = min(
                    control_timeout,
                    max(0.0, next_camera_sample - time.monotonic()),
                )
            control_event = _control_event(sys.stdin.buffer, control_timeout)
            if (
                control_event == "eof"
                and not stop_requested
                and os.getppid() == args.expected_parent_pid
            ):
                # Losing the control writer while the launcher is still alive is
                # a supervisor failure, not an authorized clean shutdown.
                if camera_runtime is not None:
                    camera_runtime.close(process.pid)
                _publish_failure(args.failure_file, UNKNOWN_EXIT_CODE)
                _reap_process_group(
                    process,
                    returncode=None,
                    term_grace_seconds=args.term_grace_seconds,
                )
                return UNKNOWN_EXIT_CODE
            if stop_requested or control_event in {"stop", "eof"}:
                # Recheck after observing the stop request. A child that exited
                # before this boundary is an unexpected exit, even exit code 0.
                returncode = _peek_returncode(process)
                if returncode is not None:
                    if camera_runtime is not None:
                        camera_runtime.close(process.pid)
                    _publish_failure(args.failure_file, returncode)
                    final_code = _reap_process_group(
                        process,
                        returncode=returncode,
                        term_grace_seconds=0.2,
                    )
                    return _normalized_returncode(final_code)
                if camera_runtime is not None:
                    camera_runtime.close(process.pid)
                _reap_process_group(
                    process,
                    returncode=None,
                    term_grace_seconds=args.term_grace_seconds,
                )
                return 0
    finally:
        if camera_runtime is not None:
            camera_runtime.close(process.pid if process is not None else None)
        log_stream.close()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pid-file", type=Path, required=True)
    parser.add_argument("--failure-file", type=Path, required=True)
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--expected-parent-pid", type=int, required=True)
    parser.add_argument("--term-grace-seconds", type=float, default=3.0)
    parser.add_argument(
        "--camera-state-file",
        type=Path,
        help="Private final-POV state file written for this UE lifetime",
    )
    parser.add_argument(
        "--camera-layout",
        type=Path,
        help="Build-pinned PlayerCameraManager memory layout",
    )
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if args.command[:1] == ["--"]:
        args.command = args.command[1:]
    if not args.command:
        parser.error("a UE command is required after --")
    if args.expected_parent_pid <= 1:
        parser.error("--expected-parent-pid must identify the launcher")
    if args.term_grace_seconds < 0.0:
        parser.error("--term-grace-seconds must be non-negative")
    camera_values = (args.camera_state_file, args.camera_layout)
    if any(value is not None for value in camera_values) and not all(
        value is not None for value in camera_values
    ):
        parser.error("--camera-state-file and --camera-layout are all-or-none")
    for name in ("camera_state_file", "camera_layout"):
        path = getattr(args, name)
        if path is not None and not path.is_absolute():
            parser.error(f"--{name.replace('_', '-')} must be absolute")
    return args


def main() -> int:
    return supervise(_parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
