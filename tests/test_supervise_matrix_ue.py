from __future__ import annotations

import importlib
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "supervise_matrix_ue.py"
if os.fspath(SCRIPT_PATH.parent) not in sys.path:
    sys.path.insert(0, os.fspath(SCRIPT_PATH.parent))
SPEC = importlib.util.spec_from_file_location("supervise_matrix_ue", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class FakeProbeError:
    NONE = 0
    IDENTITY_MISMATCH = 1
    TORN_CAMERA_CACHE = 6
    INTERNAL = 9


class FakeModule:
    ProbeError = FakeProbeError

    @staticmethod
    def CameraProbeObservation(**values):
        return SimpleNamespace(**values)


class FakeProbe:
    def __init__(self, *, bind_results=(), sample_results=()) -> None:
        self.bind_results = iter(bind_results)
        self.sample_results = iter(sample_results)
        self.bind_calls = []
        self.sample_calls = []

    def bind(self, pid: int):
        self.bind_calls.append(pid)
        result = next(self.bind_results)
        if isinstance(result, Exception):
            raise result
        return result

    def sample(self, pid: int):
        self.sample_calls.append(pid)
        result = next(self.sample_results)
        if isinstance(result, Exception):
            raise result
        return result


class FakeWriter:
    def __init__(self, *, failures: int = 0) -> None:
        self.failures = failures
        self.observations = []
        self.close_calls = 0

    def write(self, observation) -> None:
        self.observations.append(observation)
        if self.failures:
            self.failures -= 1
            raise OSError("busy")

    def close(self) -> None:
        self.close_calls += 1


def valid_observation(pid: int, yaw: float = 30.0):
    return SimpleNamespace(
        ue_pid=pid,
        monotonic_ns=1_000_000_000,
        valid=True,
        error_code=FakeProbeError.NONE,
        yaw_deg=yaw,
    )


def invalid_observation(pid: int, error_code: int):
    return SimpleNamespace(
        ue_pid=pid,
        monotonic_ns=1_000_000_000,
        valid=False,
        error_code=error_code,
    )


class CameraProbeRuntimeTest(unittest.TestCase):
    @staticmethod
    def runtime(probe: FakeProbe, writer: FakeWriter):
        return MODULE._CameraProbeRuntime(
            FakeModule,
            probe,
            writer,
            monotonic_ns=lambda: 1_000_000_000,
        )

    def test_exec_chain_identity_mismatch_is_retried_before_sampling(self) -> None:
        probe = FakeProbe(
            bind_results=(RuntimeError("still bash"), 1234),
            sample_results=(valid_observation(4242),),
        )
        writer = FakeWriter()
        runtime = self.runtime(probe, writer)

        MODULE._service_camera_probe(
            runtime,
            ue_pid=4242,
            now=1.0,
            bind_deadline=2.0,
        )
        self.assertFalse(runtime.bound)
        self.assertEqual(probe.sample_calls, [])
        self.assertEqual(
            writer.observations[-1].error_code,
            FakeProbeError.IDENTITY_MISMATCH,
        )

        MODULE._service_camera_probe(
            runtime,
            ue_pid=4242,
            now=1.02,
            bind_deadline=2.0,
        )
        self.assertTrue(runtime.bound)
        self.assertEqual(probe.bind_calls, [4242, 4242])
        self.assertEqual(probe.sample_calls, [4242])
        self.assertTrue(writer.observations[-1].valid)

    def test_layout_failure_keeps_writer_and_publishes_internal_invalid(self) -> None:
        writer = FakeWriter()

        class InitializationModule(FakeModule):
            @staticmethod
            def CameraStateWriter(_path):
                return writer

            @staticmethod
            def load_layout(_path):
                raise ValueError("bad layout")

        with mock.patch.object(
            MODULE.importlib,
            "import_module",
            return_value=InitializationModule,
        ):
            runtime = MODULE._CameraProbeRuntime.open(
                Path("/tmp/camera-state.bin"),
                Path("/tmp/camera-layout.json"),
            )

        self.assertFalse(runtime.probe_ready)
        MODULE._service_camera_probe(
            runtime,
            ue_pid=4242,
            now=1.0,
            bind_deadline=2.0,
        )
        self.assertFalse(writer.observations[-1].valid)
        self.assertEqual(
            writer.observations[-1].error_code,
            FakeProbeError.INTERNAL,
        )

    def test_bind_deadline_stays_invalid_without_another_hash_attempt(self) -> None:
        probe = FakeProbe(bind_results=(1234,))
        writer = FakeWriter()
        runtime = self.runtime(probe, writer)

        MODULE._service_camera_probe(
            runtime,
            ue_pid=4242,
            now=2.0,
            bind_deadline=2.0,
        )

        self.assertEqual(probe.bind_calls, [])
        self.assertFalse(runtime.bound)
        self.assertEqual(
            writer.observations[-1].error_code,
            FakeProbeError.IDENTITY_MISMATCH,
        )

    def test_probe_exception_publishes_internal_invalid_without_raising(self) -> None:
        probe = FakeProbe(
            bind_results=(None,),
            sample_results=(RuntimeError("read failed"),),
        )
        writer = FakeWriter()
        runtime = self.runtime(probe, writer)

        self.assertTrue(runtime.try_bind(4242))
        self.assertTrue(runtime.sample(4242))
        self.assertFalse(writer.observations[-1].valid)
        self.assertEqual(
            writer.observations[-1].error_code,
            FakeProbeError.INTERNAL,
        )

    def test_torn_camera_cache_retains_last_verified_record_until_freshness_expires(
        self,
    ) -> None:
        probe = FakeProbe(
            bind_results=(None,),
            sample_results=(
                valid_observation(4242, yaw=30.0),
                invalid_observation(4242, FakeProbeError.TORN_CAMERA_CACHE),
                valid_observation(4242, yaw=31.0),
            ),
        )
        writer = FakeWriter()
        runtime = self.runtime(probe, writer)

        self.assertTrue(runtime.try_bind(4242))
        self.assertTrue(runtime.sample(4242))
        self.assertTrue(runtime.sample(4242))
        self.assertEqual(len(writer.observations), 1)
        self.assertTrue(writer.observations[0].valid)
        self.assertTrue(runtime.sample(4242))
        self.assertEqual(len(writer.observations), 2)
        self.assertEqual(writer.observations[-1].yaw_deg, 31.0)

    def test_initial_torn_camera_cache_remains_fail_closed(self) -> None:
        probe = FakeProbe(
            bind_results=(None,),
            sample_results=(
                invalid_observation(4242, FakeProbeError.TORN_CAMERA_CACHE),
            ),
        )
        writer = FakeWriter()
        runtime = self.runtime(probe, writer)

        self.assertTrue(runtime.try_bind(4242))
        self.assertTrue(runtime.sample(4242))
        self.assertEqual(len(writer.observations), 1)
        self.assertFalse(writer.observations[0].valid)
        self.assertEqual(
            writer.observations[0].error_code,
            FakeProbeError.TORN_CAMERA_CACHE,
        )

    def test_valid_write_failure_attempts_an_invalid_replacement(self) -> None:
        probe = FakeProbe(
            bind_results=(None,),
            sample_results=(valid_observation(4242),),
        )
        writer = FakeWriter(failures=1)
        runtime = self.runtime(probe, writer)

        self.assertTrue(runtime.try_bind(4242))
        self.assertTrue(runtime.sample(4242))
        self.assertEqual(len(writer.observations), 2)
        self.assertTrue(writer.observations[0].valid)
        self.assertFalse(writer.observations[1].valid)
        self.assertEqual(
            writer.observations[1].error_code,
            FakeProbeError.INTERNAL,
        )

    def test_close_publishes_terminal_invalid_and_is_idempotent(self) -> None:
        probe = FakeProbe()
        writer = FakeWriter()
        runtime = self.runtime(probe, writer)

        runtime.close(4242)
        runtime.close(4242)

        self.assertTrue(runtime.closed)
        self.assertEqual(writer.close_calls, 1)
        self.assertEqual(len(writer.observations), 1)
        self.assertFalse(writer.observations[0].valid)
        self.assertEqual(
            writer.observations[0].error_code,
            FakeProbeError.INTERNAL,
        )

    def test_sample_period_is_fifty_hertz(self) -> None:
        self.assertAlmostEqual(MODULE.CAMERA_SAMPLE_INTERVAL_SECONDS, 1.0 / 50.0)


class CameraProbeCliTest(unittest.TestCase):
    def base_argv(self):
        return [
            "supervise_matrix_ue.py",
            "--pid-file",
            "/tmp/ue.pid",
            "--failure-file",
            "/tmp/failure.json",
            "--log",
            "/tmp/ue.log",
            "--expected-parent-pid",
            str(os.getpid()),
        ]

    def test_camera_state_and_layout_are_accepted_as_an_atomic_pair(self) -> None:
        argv = [
            *self.base_argv(),
            "--camera-state-file",
            "/tmp/camera-state.bin",
            "--camera-layout",
            "/tmp/camera-layout.json",
            "--",
            "/bin/true",
        ]
        with mock.patch.object(MODULE.sys, "argv", argv):
            args = MODULE._parse_args()
        self.assertEqual(args.camera_state_file, Path("/tmp/camera-state.bin"))
        self.assertEqual(args.camera_layout, Path("/tmp/camera-layout.json"))
        self.assertEqual(args.command, ["/bin/true"])
        self.assertFalse(hasattr(args, "camera_executable"))

    def test_camera_arguments_are_all_or_none(self) -> None:
        argv = [
            *self.base_argv(),
            "--camera-state-file",
            "/tmp/camera-state.bin",
            "--",
            "/bin/true",
        ]
        with mock.patch.object(MODULE.sys, "argv", argv), self.assertRaises(
            SystemExit
        ):
            MODULE._parse_args()

    def test_camera_executable_argument_is_not_accepted(self) -> None:
        argv = [
            *self.base_argv(),
            "--camera-executable",
            "/tmp/ue",
            "--",
            "/bin/true",
        ]
        with mock.patch.object(MODULE.sys, "argv", argv), self.assertRaises(
            SystemExit
        ):
            MODULE._parse_args()

    def test_writer_initialization_failure_does_not_block_ue_spawn(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            pid_file = root / "ue.pid"
            failure_file = root / "failure.json"
            # The missing parent makes CameraStateWriter initialization fail.
            state_file = root / "missing" / "camera-state.bin"
            process = subprocess.Popen(
                [
                    sys.executable,
                    os.fspath(SCRIPT_PATH),
                    "--pid-file",
                    os.fspath(pid_file),
                    "--failure-file",
                    os.fspath(failure_file),
                    "--log",
                    os.fspath(root / "ue.log"),
                    "--expected-parent-pid",
                    str(os.getpid()),
                    "--camera-state-file",
                    os.fspath(state_file),
                    "--camera-layout",
                    os.fspath(root / "missing-layout.json"),
                    "--",
                    "/bin/sh",
                    "-c",
                    "while :; do sleep 1; done",
                ],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
            try:
                deadline = time.monotonic() + 3.0
                while not pid_file.exists() and time.monotonic() < deadline:
                    time.sleep(0.01)
                self.assertTrue(pid_file.exists(), "supervisor did not spawn UE")
                self.assertGreater(int(pid_file.read_text(encoding="utf-8")), 1)
                assert process.stdin is not None
                process.stdin.write(b"stop\n")
                process.stdin.flush()
                process.stdin.close()
                self.assertEqual(process.wait(timeout=6.0), 0)
                self.assertFalse(failure_file.exists())
                self.assertFalse(state_file.exists())
                assert process.stderr is not None
                diagnostic = process.stderr.read().decode("utf-8", errors="replace")
                process.stderr.close()
                self.assertIn("initializing final-POV probe", diagnostic)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait(timeout=3.0)
                if process.stderr is not None and not process.stderr.closed:
                    process.stderr.close()

    def test_malformed_layout_runs_ue_and_publishes_internal_invalid(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            pid_file = root / "ue.pid"
            failure_file = root / "failure.json"
            state_file = root / "camera-state.bin"
            layout_file = root / "camera-layout.json"
            layout_file.write_text("{}\n", encoding="utf-8")
            process = subprocess.Popen(
                [
                    sys.executable,
                    os.fspath(SCRIPT_PATH),
                    "--pid-file",
                    os.fspath(pid_file),
                    "--failure-file",
                    os.fspath(failure_file),
                    "--log",
                    os.fspath(root / "ue.log"),
                    "--expected-parent-pid",
                    str(os.getpid()),
                    "--camera-state-file",
                    os.fspath(state_file),
                    "--camera-layout",
                    os.fspath(layout_file),
                    "--",
                    "/bin/sh",
                    "-c",
                    "while :; do sleep 1; done",
                ],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
            try:
                deadline = time.monotonic() + 3.0
                while (
                    (not pid_file.exists() or not state_file.exists())
                    and time.monotonic() < deadline
                ):
                    time.sleep(0.01)
                self.assertTrue(pid_file.exists(), "supervisor did not spawn UE")
                ue_pid = int(pid_file.read_text(encoding="utf-8"))
                self.assertGreater(ue_pid, 1)
                self.assertTrue(state_file.exists(), "invalid state was not published")
                self.assertFalse(failure_file.exists())

                assert process.stdin is not None
                process.stdin.write(b"stop\n")
                process.stdin.flush()
                process.stdin.close()
                self.assertEqual(process.wait(timeout=6.0), 0)
                self.assertFalse(failure_file.exists())

                camera = importlib.import_module("matrix_ue_camera_probe")
                reader = camera.CameraStateReader(
                    state_file,
                    expected_ue_pid=ue_pid,
                    max_age_ns=10_000_000_000,
                )
                self.assertIsNone(reader.read())
                self.assertEqual(
                    reader.last_error,
                    f"probe_error_{int(camera.ProbeError.INTERNAL)}",
                )
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait(timeout=3.0)
                if process.stderr is not None and not process.stderr.closed:
                    process.stderr.close()


if __name__ == "__main__":
    unittest.main()
