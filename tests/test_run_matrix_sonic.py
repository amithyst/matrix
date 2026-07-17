from __future__ import annotations

import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "run_matrix_sonic.py"
SPEC = importlib.util.spec_from_file_location("run_matrix_sonic", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class MatrixSonicRuntimeTest(unittest.TestCase):
    @staticmethod
    def snapshot(
        *,
        step_index: int = 0,
        sim_time: float = 0.0,
        qpos_len: int = 36,
        qvel_len: int = 35,
        ctrl_len: int = 29,
        torque_len: int = 29,
        fall_detected: bool = False,
        reset_count: int = 0,
        last_reset_reason: str | None = None,
    ) -> SimpleNamespace:
        return SimpleNamespace(
            step_index=step_index,
            sim_time=sim_time,
            qpos=[0.0] * qpos_len,
            qvel=[0.0] * qvel_len,
            ctrl=[0.0] * ctrl_len,
            applied_torque=[0.0] * torque_len,
            fall_detected=fall_detected,
            reset_count=reset_count,
            last_reset_reason=last_reset_reason,
        )

    @staticmethod
    def process_is_running(pid: int) -> bool:
        try:
            state = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8").split()[2]
        except (FileNotFoundError, IndexError, OSError):
            return False
        return state != "Z"

    def test_planner_endpoint_requires_loopback_tcp(self) -> None:
        self.assertEqual(MODULE._loopback_zmq_port("tcp://127.0.0.1:5556"), 5556)
        self.assertEqual(MODULE._loopback_zmq_port("tcp://[::1]:6000"), 6000)
        for endpoint in (
            "tcp://0.0.0.0:5556",
            "tcp://192.168.1.2:5556",
            "udp://127.0.0.1:5556",
            "tcp://127.0.0.1",
        ):
            with self.subTest(endpoint=endpoint):
                with self.assertRaises(ValueError):
                    MODULE._loopback_zmq_port(endpoint)

    def test_root_up_z_is_one_for_upright_quaternion(self) -> None:
        self.assertAlmostEqual(MODULE._root_up_z([0, 0, 0, 1, 0, 0, 0]), 1.0)

    def test_root_up_z_is_negative_for_upside_down_quaternion(self) -> None:
        self.assertAlmostEqual(MODULE._root_up_z([0, 0, 0, 0, 1, 0, 0]), -1.0)

    def test_acceptance_rejects_fall_and_short_lowcmd(self) -> None:
        failures = MODULE._acceptance_failures(
            unstable=False,
            fall_detected=True,
            fail_on_fall=True,
            active_lowcmd=True,
            active_elapsed_s=12.0,
            min_active_seconds=30.0,
            physics_step_hz=200.0,
            min_physics_hz=195.0,
            rtf=1.0,
            min_rtf=0.95,
        )
        self.assertEqual(failures[0], "fall_detected")
        self.assertTrue(failures[1].startswith("active_lowcmd_too_short:"))

    def test_acceptance_allows_interactive_run_without_minimum(self) -> None:
        self.assertEqual(
            MODULE._acceptance_failures(
                unstable=False,
                fall_detected=False,
                fail_on_fall=True,
                active_lowcmd=False,
                active_elapsed_s=0.0,
                min_active_seconds=0.0,
                physics_step_hz=0.0,
                min_physics_hz=0.0,
                rtf=0.0,
                min_rtf=0.0,
            ),
            [],
        )

    def test_acceptance_rejects_stale_lowcmd_and_slow_physics(self) -> None:
        failures = MODULE._acceptance_failures(
            unstable=False,
            fall_detected=False,
            fail_on_fall=True,
            active_lowcmd=False,
            active_elapsed_s=45.0,
            min_active_seconds=30.0,
            physics_step_hz=190.0,
            min_physics_hz=195.0,
            rtf=0.90,
            min_rtf=0.95,
        )
        self.assertEqual(
            failures,
            [
                "lowcmd_not_fresh_at_exit",
                "physics_hz_too_low:190.000<195.000",
                "rtf_too_low:0.9000<0.9500",
            ],
        )

    def test_child_exit_is_not_misclassified_as_numerical_instability(self) -> None:
        failures = MODULE._acceptance_failures(
            unstable=False,
            fall_detected=False,
            fail_on_fall=False,
            active_lowcmd=False,
            active_elapsed_s=0.0,
            min_active_seconds=0.0,
            physics_step_hz=200.0,
            min_physics_hz=0.0,
            rtf=1.0,
            min_rtf=0.0,
            failed_child=("deploy", 17),
        )
        self.assertEqual(failures, ["native_child_exit:deploy:17"])
        self.assertNotIn("numerical_instability", failures)

    def test_acceptance_enforces_optional_displacement_gate(self) -> None:
        failures = MODULE._acceptance_failures(
            unstable=False,
            fall_detected=False,
            fail_on_fall=False,
            active_lowcmd=False,
            active_elapsed_s=0.0,
            min_active_seconds=0.0,
            physics_step_hz=200.0,
            min_physics_hz=0.0,
            rtf=1.0,
            min_rtf=0.0,
            root_displacement_xy_m=0.49,
            min_displacement_m=0.5,
        )
        self.assertEqual(failures, ["root_displacement_too_small:0.490<0.500"])

    def test_acceptance_enforces_directional_final_x_gate(self) -> None:
        failures = MODULE._acceptance_failures(
            unstable=False,
            fall_detected=False,
            fail_on_fall=False,
            active_lowcmd=True,
            active_elapsed_s=30.0,
            min_active_seconds=30.0,
            physics_step_hz=200.0,
            min_physics_hz=195.0,
            rtf=1.0,
            min_rtf=0.95,
            root_displacement_xy_m=10.0,
            min_displacement_m=0.5,
            root_final_x=114.0,
            min_final_x=128.0,
        )
        self.assertEqual(failures, ["final_x_too_small:114.000<128.000"])

    def test_acceptance_rejects_lateral_distance_without_forward_progress(self) -> None:
        failures = MODULE._acceptance_failures(
            unstable=False,
            fall_detected=False,
            fail_on_fall=False,
            active_lowcmd=True,
            active_elapsed_s=30.0,
            min_active_seconds=30.0,
            physics_step_hz=200.0,
            min_physics_hz=195.0,
            rtf=1.0,
            min_rtf=0.95,
            root_displacement_xy_m=5.0,
            min_displacement_m=0.5,
            root_final_x=128.1,
            min_final_x=128.0,
            root_displacement_x_m=0.1,
            min_forward_x_m=4.0,
        )
        self.assertEqual(failures, ["forward_x_too_small:0.100<4.000"])

    def test_acceptance_rejects_authoritative_reset_count(self) -> None:
        failures = MODULE._acceptance_failures(
            unstable=False,
            fall_detected=False,
            fail_on_fall=True,
            active_lowcmd=True,
            active_elapsed_s=30.0,
            min_active_seconds=30.0,
            physics_step_hz=200.0,
            min_physics_hz=195.0,
            rtf=1.0,
            min_rtf=0.95,
            reset_count=1,
            max_resets=0,
        )
        self.assertEqual(failures, ["reset_count_exceeded:1>0"])

    def test_snapshot_requires_exact_native_dimensions(self) -> None:
        self.assertIsNone(MODULE._snapshot_validation_error(self.snapshot()))
        for field, kwargs, expected in (
            ("qpos", {"qpos_len": 37}, "qpos=37,expected=36"),
            ("qvel", {"qvel_len": 34}, "qvel=34,expected=35"),
            ("ctrl", {"ctrl_len": 30}, "ctrl=30,expected=29"),
            (
                "applied_torque",
                {"torque_len": 28},
                "applied_torque=28,expected=29",
            ),
        ):
            with self.subTest(field=field):
                error = MODULE._snapshot_validation_error(self.snapshot(**kwargs))
                self.assertEqual(error, f"snapshot_dimension:{expected}")

    def test_snapshot_step_must_advance_once_and_time_must_increase(self) -> None:
        previous = self.snapshot(step_index=10, sim_time=1.0)
        self.assertIsNone(
            MODULE._snapshot_validation_error(
                self.snapshot(step_index=11, sim_time=1.005), previous
            )
        )
        self.assertEqual(
            MODULE._snapshot_validation_error(
                self.snapshot(step_index=12, sim_time=1.005), previous
            ),
            "snapshot_step_index_not_sequential:12,expected=11",
        )
        self.assertEqual(
            MODULE._snapshot_validation_error(
                self.snapshot(step_index=11, sim_time=1.0), previous
            ),
            "snapshot_sim_time_not_increasing:1.0,previous=1.0",
        )

    def test_snapshot_rejects_non_finite_physics_vectors(self) -> None:
        for field in ("qpos", "qvel", "ctrl", "applied_torque"):
            with self.subTest(field=field):
                snapshot = self.snapshot()
                getattr(snapshot, field)[2] = math.nan
                self.assertEqual(
                    MODULE._snapshot_validation_error(snapshot),
                    f"snapshot_non_finite:{field}[2]=nan",
                )

    def test_snapshot_validates_authoritative_fall_reset_fields(self) -> None:
        self.assertIsNone(
            MODULE._snapshot_validation_error(
                self.snapshot(
                    fall_detected=True,
                    reset_count=1,
                    last_reset_reason="fall",
                )
            )
        )
        invalid_fall = self.snapshot()
        invalid_fall.fall_detected = 1
        self.assertEqual(
            MODULE._snapshot_validation_error(invalid_fall),
            "snapshot_invalid_fall_detected:1",
        )
        self.assertEqual(
            MODULE._snapshot_validation_error(self.snapshot(reset_count=-1)),
            "snapshot_invalid_reset_count:-1",
        )
        self.assertEqual(
            MODULE._snapshot_validation_error(
                self.snapshot(step_index=1, sim_time=0.005, reset_count=1),
                self.snapshot(step_index=0, sim_time=0.0, reset_count=2),
            ),
            "snapshot_reset_count_decreased:1,previous=2",
        )

    def test_only_normal_bounded_completion_can_pass(self) -> None:
        completed = MODULE._qualification_state(
            max_seconds=120.0,
            termination_reason="max_seconds",
            failures=[],
            runtime_verified=True,
        )
        self.assertTrue(completed["passed"])
        self.assertTrue(completed["completed"])

        bounded_signal = MODULE._qualification_state(
            max_seconds=120.0,
            termination_reason="signal",
            failures=[],
            runtime_verified=True,
        )
        self.assertFalse(bounded_signal["passed"])
        self.assertIn("run_interrupted", bounded_signal["acceptance_failures"])

        interactive_signal = MODULE._qualification_state(
            max_seconds=0.0,
            termination_reason="signal",
            failures=[],
            runtime_verified=False,
        )
        self.assertFalse(interactive_signal["passed"])
        self.assertTrue(interactive_signal["interrupted"])
        self.assertEqual(interactive_signal["acceptance_failures"], [])

        unverified = MODULE._qualification_state(
            max_seconds=30.0,
            termination_reason="max_seconds",
            failures=[],
            runtime_verified=False,
        )
        self.assertFalse(unverified["passed"])
        self.assertIn(
            "runtime_not_verified_for_qualification",
            unverified["acceptance_failures"],
        )

    def test_qualified_runtime_consumes_matching_verifier_receipt(self) -> None:
        lock = REPO_ROOT / "config/runtime/matrix-sonic.lock.json"
        matrix_commit = subprocess.run(
            ["git", "-C", str(REPO_ROOT), "rev-parse", "HEAD"],
            text=True,
            capture_output=True,
            check=True,
        ).stdout.strip()
        with tempfile.TemporaryDirectory() as temporary:
            receipt_path = Path(temporary) / "receipt.json"
            payload = {
                "passed": True,
                "checks": [{"name": "locked runtime", "ok": True}],
                "profile": "trna",
                "lock": str(lock),
                "lock_sha256": MODULE._sha256_file(lock),
                "matrix_root": str(REPO_ROOT),
                "matrix_commit": matrix_commit,
                "sonic_root": "/sonic",
            }
            receipt_path.write_text(json.dumps(payload), encoding="utf-8")
            args = SimpleNamespace(
                qualified_runtime=True,
                runtime_lock_sha256=payload["lock_sha256"],
                matrix_commit=matrix_commit,
                verification_receipt=receipt_path,
                qualification_profile="trna",
                sonic_root=Path("/sonic"),
            )
            MODULE._validate_qualification_receipt(args)
            self.assertEqual(args.verification_receipt, receipt_path.resolve())
            self.assertEqual(
                args.verification_receipt_sha256,
                MODULE._sha256_file(receipt_path),
            )

            payload["passed"] = False
            receipt_path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(SystemExit, "receipt does not match"):
                MODULE._validate_qualification_receipt(args)

    def test_native_config_uses_matrix_model_and_waits_for_lowcmd(self) -> None:
        args = SimpleNamespace(
            dds_interface="lo",
            physics_hz=200.0,
            startup_band=True,
            startup_band_hold=4.0,
            startup_band_fade=3.0,
            low_cmd_fresh_timeout_seconds=0.1,
        )
        kwargs = MODULE._native_config_kwargs(args, Path("/tmp/matrix.xml"))
        self.assertEqual(kwargs["robot_scene"], "/tmp/matrix.xml")
        self.assertEqual(kwargs["interface"], "lo")
        self.assertEqual(kwargs["sim_frequency"], 200)
        self.assertTrue(kwargs["elastic_band_release_enabled"])
        self.assertTrue(kwargs["elastic_band_wait_for_lowcmd"])
        self.assertEqual(kwargs["elastic_band_hold_seconds"], 4.0)
        self.assertEqual(kwargs["elastic_band_fade_seconds"], 3.0)
        self.assertEqual(kwargs["low_cmd_fresh_timeout_seconds"], 0.1)
        self.assertFalse(kwargs["with_hands"])
        self.assertFalse(kwargs["reset_on_fall"])

    def test_disabling_startup_band_requests_immediate_release(self) -> None:
        args = SimpleNamespace(
            dds_interface="lo",
            physics_hz=200.0,
            startup_band=False,
            startup_band_hold=4.0,
            startup_band_fade=3.0,
            low_cmd_fresh_timeout_seconds=0.1,
        )
        kwargs = MODULE._native_config_kwargs(args, Path("/tmp/matrix.xml"))
        self.assertTrue(kwargs["elastic_band_release_enabled"])
        self.assertFalse(kwargs["elastic_band_wait_for_lowcmd"])
        self.assertEqual(kwargs["elastic_band_hold_seconds"], 0.0)
        self.assertEqual(kwargs["elastic_band_fade_seconds"], 0.0)

    def test_native_planner_uses_sonic_wire_builders(self) -> None:
        class FakeSocket:
            def __init__(self) -> None:
                self.bound = None
                self.sent = []

            def setsockopt(self, *_args) -> None:
                pass

            def bind(self, endpoint) -> None:
                self.bound = endpoint

            def send(self, payload) -> None:
                self.sent.append(payload)

            def close(self, **_kwargs) -> None:
                pass

        socket = FakeSocket()

        class FakeContext:
            @classmethod
            def instance(cls):
                return cls()

            def socket(self, _kind):
                return socket

        fake_zmq = SimpleNamespace(Context=FakeContext, PUB=1, LINGER=2)
        commands = []
        planners = []

        def build_command_message(**kwargs):
            commands.append(kwargs)
            return b"command"

        def build_planner_message(**kwargs):
            planners.append(kwargs)
            return b"planner"

        client = MODULE.NativePlannerClient(
            "tcp://127.0.0.1:5556",
            zmq_module=fake_zmq,
            build_command_message=build_command_message,
            build_planner_message=build_planner_message,
        )
        client.send_velocity(
            1.0,
            0.0,
            0.5,
            dt=0.2,
        )

        self.assertEqual(socket.bound, "tcp://127.0.0.1:5556")
        self.assertEqual(socket.sent, [b"command", b"planner"])
        self.assertTrue(commands[0]["start"])
        self.assertEqual(planners[0]["mode"], 2)
        self.assertAlmostEqual(planners[0]["movement"][0], math.cos(0.1))
        self.assertAlmostEqual(planners[0]["movement"][1], math.sin(0.1))
        self.assertAlmostEqual(planners[0]["facing"][0], math.cos(0.1))
        self.assertAlmostEqual(planners[0]["facing"][1], math.sin(0.1))
        self.assertEqual(planners[0]["speed"], 1.0)

        with mock.patch.object(MODULE.time, "sleep") as sleep:
            client.close()
        self.assertTrue(commands[-1]["stop"])
        self.assertEqual(socket.sent[-3:], [b"command"] * 3)
        self.assertEqual(sleep.call_count, 3)

    def test_native_planner_yaw_only_remains_idle(self) -> None:
        class FakeSocket:
            def setsockopt(self, *_args) -> None:
                pass

            def bind(self, _endpoint) -> None:
                pass

            def send(self, _payload) -> None:
                pass

            def close(self, **_kwargs) -> None:
                pass

        socket = FakeSocket()

        class FakeContext:
            @classmethod
            def instance(cls):
                return cls()

            def socket(self, _kind):
                return socket

        planners = []
        client = MODULE.NativePlannerClient(
            "tcp://127.0.0.1:5556",
            zmq_module=SimpleNamespace(Context=FakeContext, PUB=1, LINGER=2),
            build_command_message=lambda **_kwargs: b"command",
            build_planner_message=lambda **kwargs: planners.append(kwargs) or b"planner",
        )
        client.send_velocity(0.0, 0.0, 1.0, dt=0.1)
        self.assertEqual(planners[0]["mode"], 0)
        self.assertEqual(planners[0]["movement"], [0.0, 0.0, 0.0])
        self.assertEqual(planners[0]["speed"], -1.0)
        self.assertAlmostEqual(planners[0]["facing"][0], math.cos(0.1))
        client.close()

    @mock.patch.object(MODULE.subprocess, "Popen")
    def test_native_process_group_runs_locked_binary_directly(self, popen) -> None:
        process = mock.Mock()
        popen.return_value = process
        group = MODULE.NativeProcessGroup(Path("/sonic"), {})
        group.start_deploy(interface="lo", zmq_port=6000)

        guarded_command = popen.call_args.args[0]
        self.assertEqual(guarded_command[0], sys.executable)
        self.assertEqual(
            Path(guarded_command[1]).name,
            "exec_with_parent_death_signal.py",
        )
        command = guarded_command[guarded_command.index("--") + 1 :]
        self.assertEqual(command[0], "/sonic/gear_sonic_deploy/target/release/g1_deploy_onnx_ref")
        self.assertNotIn("deploy.sh", command)
        self.assertEqual(command[1], "lo")
        self.assertIn("--disable-crc-check", command)
        self.assertEqual(command[command.index("--zmq-port") + 1], "6000")
        self.assertEqual(
            group.env["FASTRTPS_DEFAULT_PROFILES_FILE"],
            "/sonic/gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/config/fastrtps_profile.xml",
        )
        self.assertEqual(group.env["ROS_LOCALHOST_ONLY"], "1")
        self.assertEqual(group.env["PYTHONNOUSERSITE"], "1")
        self.assertEqual(group.env["PYTHONPATH"], "/sonic")

    def test_process_group_prepends_sonic_to_existing_pythonpath(self) -> None:
        group = MODULE.NativeProcessGroup(
            Path("/sonic"), {"PYTHONPATH": "/locked/site"}
        )
        self.assertEqual(
            group.env["PYTHONPATH"],
            f"/sonic{MODULE.os.pathsep}/locked/site",
        )

    def test_process_group_passes_the_exact_host_lock_to_guardian(self) -> None:
        with tempfile.TemporaryFile() as lock_stream, mock.patch.object(
            MODULE.subprocess, "Popen"
        ) as popen:
            lock_fd = lock_stream.fileno()
            group = MODULE.NativeProcessGroup(
                Path("/sonic"),
                {"MATRIX_SONIC_HOST_LOCK_FD": str(lock_fd)},
            )
            group.start_deploy(interface="lo", zmq_port=6000)
            self.assertEqual(popen.call_args.kwargs["pass_fds"], (lock_fd,))

    @mock.patch.object(MODULE.time, "sleep")
    def test_native_deploy_gets_a_graceful_stop_window(self, sleep) -> None:
        process = mock.Mock()
        process.poll.side_effect = [None, 0]
        group = MODULE.NativeProcessGroup(Path("/sonic"), {})
        group.children.append(("deploy", process))

        self.assertTrue(group.wait_for_child("deploy", timeout=2.0))
        sleep.assert_called_once()

    @mock.patch.object(MODULE.subprocess, "Popen")
    def test_pico_uses_its_locked_python_and_planner_port(self, popen) -> None:
        popen.return_value = mock.Mock()
        group = MODULE.NativeProcessGroup(Path("/sonic"), {})
        group.start_pico("/pico/bin/python", port=6000)
        guarded_command = popen.call_args.args[0]
        command = guarded_command[guarded_command.index("--") + 1 :]
        self.assertEqual(command[0], "/pico/bin/python")
        self.assertEqual(command[1], "-u")
        self.assertEqual(command[command.index("--port") + 1], "6000")

    def test_parent_death_guardian_kills_native_process_group(self) -> None:
        guardian = REPO_ROOT / "scripts/exec_with_parent_death_signal.py"
        child_code = "\n".join(
            (
                "import os",
                "from pathlib import Path",
                "import subprocess",
                "import sys",
                "import time",
                "grandchild = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])",
                "Path(sys.argv[1]).write_text(f'{os.getpid()} {grandchild.pid}', encoding='utf-8')",
                "time.sleep(60)",
            )
        )
        supervisor_code = "\n".join(
            (
                "import os",
                "import subprocess",
                "import sys",
                "import time",
                "process = subprocess.Popen([sys.executable, sys.argv[1], '--expected-parent', str(os.getpid()), '--', sys.executable, '-c', sys.argv[2], sys.argv[3]], start_new_session=True)",
                "print(process.pid, flush=True)",
                "time.sleep(60)",
            )
        )

        with tempfile.TemporaryDirectory() as temporary:
            pid_file = Path(temporary) / "native-pids"
            supervisor = subprocess.Popen(
                [
                    sys.executable,
                    "-c",
                    supervisor_code,
                    str(guardian),
                    child_code,
                    str(pid_file),
                ],
                stdout=subprocess.PIPE,
                text=True,
            )
            assert supervisor.stdout is not None
            group_id = int(supervisor.stdout.readline().strip())
            try:
                deadline = time.monotonic() + 5.0
                while not pid_file.is_file() and time.monotonic() < deadline:
                    time.sleep(0.02)
                self.assertTrue(pid_file.is_file(), "guarded child did not start")
                native_pids = [
                    int(value) for value in pid_file.read_text(encoding="utf-8").split()
                ]

                os.kill(supervisor.pid, signal.SIGKILL)
                supervisor.wait(timeout=5.0)
                deadline = time.monotonic() + 5.0
                while (
                    any(self.process_is_running(pid) for pid in native_pids)
                    and time.monotonic() < deadline
                ):
                    time.sleep(0.05)
                self.assertFalse(
                    any(self.process_is_running(pid) for pid in native_pids),
                    f"native process group survived supervisor death: {native_pids}",
                )
            finally:
                if supervisor.poll() is None:
                    supervisor.kill()
                    supervisor.wait(timeout=5.0)
                supervisor.stdout.close()
                try:
                    os.killpg(group_id, signal.SIGKILL)
                except ProcessLookupError:
                    pass

    def test_supervisor_receives_signal_when_run_sim_parent_is_sigkilled(self) -> None:
        child_code = "\n".join(
            (
                "import importlib.util",
                "import os",
                "from pathlib import Path",
                "import sys",
                "import time",
                "script = Path(sys.argv[1]).resolve()",
                "sys.path.insert(0, str(script.parent))",
                "spec = importlib.util.spec_from_file_location('guarded_runner', script)",
                "module = importlib.util.module_from_spec(spec)",
                "spec.loader.exec_module(module)",
                "module._arm_supervisor_parent_death(os.getppid())",
                "Path(sys.argv[2]).write_text(str(os.getpid()), encoding='utf-8')",
                "time.sleep(60)",
            )
        )
        parent_code = "\n".join(
            (
                "import subprocess",
                "import sys",
                "import time",
                "child = subprocess.Popen([sys.executable, '-c', sys.argv[1], sys.argv[2], sys.argv[3]])",
                "print(child.pid, flush=True)",
                "time.sleep(60)",
            )
        )

        with tempfile.TemporaryDirectory() as temporary:
            pid_file = Path(temporary) / "supervisor-pid"
            parent = subprocess.Popen(
                [
                    sys.executable,
                    "-c",
                    parent_code,
                    child_code,
                    str(SCRIPT_PATH),
                    str(pid_file),
                ],
                stdout=subprocess.PIPE,
                text=True,
            )
            assert parent.stdout is not None
            supervisor_pid = int(parent.stdout.readline().strip())
            try:
                deadline = time.monotonic() + 5.0
                while not pid_file.is_file() and time.monotonic() < deadline:
                    time.sleep(0.02)
                self.assertTrue(pid_file.is_file(), "supervisor did not arm PDEATHSIG")

                os.kill(parent.pid, signal.SIGKILL)
                parent.wait(timeout=5.0)
                deadline = time.monotonic() + 5.0
                while (
                    self.process_is_running(supervisor_pid)
                    and time.monotonic() < deadline
                ):
                    time.sleep(0.05)
                self.assertFalse(
                    self.process_is_running(supervisor_pid),
                    "supervisor survived run_sim parent death",
                )
            finally:
                if parent.poll() is None:
                    parent.kill()
                    parent.wait(timeout=5.0)
                parent.stdout.close()
                try:
                    os.kill(supervisor_pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass

    @mock.patch.object(MODULE.os, "killpg")
    def test_process_group_close_signals_group_after_leader_exit(self, killpg) -> None:
        process = mock.Mock(pid=4321)
        process.poll.return_value = 7

        def signal_group(_process_group, signum):
            if signum == 0:
                raise ProcessLookupError

        killpg.side_effect = signal_group
        group = MODULE.NativeProcessGroup(Path("/sonic"), {})
        group.children.append(("deploy", process))

        group.close()

        self.assertIn(mock.call(4321, MODULE.signal.SIGTERM), killpg.call_args_list)
        process.wait.assert_called_once_with(timeout=0.0)

    @mock.patch.object(MODULE.os, "killpg")
    def test_process_group_close_reports_survivors(self, _killpg) -> None:
        process = mock.Mock(pid=4321)
        group = MODULE.NativeProcessGroup(Path("/sonic"), {})
        group.children.append(("deploy", process))

        with mock.patch.object(
            group, "_wait_for_groups", side_effect=[{4321}, {4321}]
        ), self.assertRaisesRegex(RuntimeError, "survived SIGKILL"):
            group.close()

    def test_cleanup_failure_invalidates_written_status(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            status = Path(temporary) / "status.json"
            status.write_text(
                json.dumps(
                    {
                        "acceptance_failures": [],
                        "passed": True,
                        "termination_reason": "max_seconds",
                    }
                ),
                encoding="utf-8",
            )

            MODULE._record_cleanup_failure(status, ["native processes: alive"])

            payload = json.loads(status.read_text(encoding="utf-8"))
            self.assertFalse(payload["passed"])
            self.assertIn("cleanup_failure", payload["acceptance_failures"])
            self.assertEqual(payload["termination_reason"], "cleanup_failure")

    @mock.patch.object(MODULE.time, "sleep")
    @mock.patch.object(MODULE.time, "monotonic", return_value=10.0)
    @mock.patch.object(MODULE.os, "killpg")
    def test_process_group_wait_is_bounded(self, killpg, _monotonic, sleep) -> None:
        process = mock.Mock(pid=4321)
        group = MODULE.NativeProcessGroup(Path("/sonic"), {})
        group.children.append(("deploy", process))

        remaining = group._wait_for_groups({4321}, deadline=10.0)

        self.assertEqual(remaining, {4321})
        killpg.assert_called_once_with(4321, 0)
        sleep.assert_not_called()

    def test_startup_failure_closes_simulator_and_started_children(self) -> None:
        events = []
        simulator = mock.Mock()
        simulator.get_state_snapshot.return_value = self.snapshot()
        simulator.close.side_effect = lambda: events.append("simulator-close")

        process_group = mock.Mock()
        process_group.start_pico.side_effect = lambda *_args, **_kwargs: events.append(
            "pico-start"
        )

        def fail_deploy(**_kwargs):
            events.append("deploy-start")
            raise RuntimeError("deploy failed")

        process_group.start_deploy.side_effect = fail_deploy
        process_group.close.side_effect = lambda: events.append("processes-close")

        fake_numpy = ModuleType("numpy")
        fake_numpy.float64 = float
        fake_numpy.asarray = lambda values, dtype=None: list(values)
        fake_zmq = ModuleType("zmq")
        run_sim_loop = ModuleType("gear_sonic.scripts.run_sim_loop")
        run_sim_loop.create_simulator = lambda _config: simulator
        configs = ModuleType("gear_sonic.utils.mujoco_sim.configs")
        configs.SimLoopConfig = lambda **kwargs: kwargs
        planner_sender = ModuleType(
            "gear_sonic.utils.teleop.zmq.zmq_planner_sender"
        )
        planner_sender.build_command_message = lambda **_kwargs: b"command"
        planner_sender.build_planner_message = lambda **_kwargs: b"planner"
        render_protocol = ModuleType("matrix_render_protocol")
        render_protocol.MatrixRenderPublisher = mock.Mock()
        render_protocol.packet_size = lambda **_kwargs: 0

        fake_modules = {
            "numpy": fake_numpy,
            "zmq": fake_zmq,
            "gear_sonic.scripts.run_sim_loop": run_sim_loop,
            "gear_sonic.utils.mujoco_sim.configs": configs,
            "gear_sonic.utils.teleop.zmq.zmq_planner_sender": planner_sender,
            "matrix_render_protocol": render_protocol,
        }
        for package_name in (
            "gear_sonic",
            "gear_sonic.scripts",
            "gear_sonic.utils",
            "gear_sonic.utils.mujoco_sim",
            "gear_sonic.utils.teleop",
            "gear_sonic.utils.teleop.zmq",
        ):
            package = ModuleType(package_name)
            package.__path__ = []
            fake_modules[package_name] = package

        args = SimpleNamespace(
            model=SCRIPT_PATH,
            sonic_root=Path("/sonic"),
            control_source="pico",
            planner_bind="tcp://127.0.0.1:5556",
            pico_python="/pico/bin/python",
            dds_interface="lo",
            render_host="127.0.0.1",
            render_port=9999,
            no_render_sync=True,
            physics_hz=200.0,
            control_hz=50.0,
            max_seconds=1.0,
            fail_on_fall=False,
            min_active_seconds=0.0,
            min_displacement_m=0.0,
            min_final_x=None,
            min_forward_x_m=0.0,
            low_cmd_fresh_timeout_seconds=0.1,
            min_physics_hz=0.0,
            min_rtf=0.0,
            max_resets=0,
            walk_after=-1.0,
            vx=0.3,
            vy=0.0,
            yaw_rate=0.0,
            status_file=None,
            qualified_runtime=False,
            qualification_profile=None,
            runtime_lock_sha256=None,
            scenario_layout_sha256=None,
            matrix_commit=None,
            verification_receipt=None,
            expected_parent_pid=None,
            print_every=2.0,
            startup_band=False,
            startup_band_hold=4.0,
            startup_band_fade=3.0,
        )

        def record_signal(signum, handler):
            events.append(("signal", int(signum), handler))

        with (
            mock.patch.dict(MODULE.sys.modules, fake_modules),
            mock.patch.object(MODULE, "_parse_args", return_value=args),
            mock.patch.object(
                MODULE, "_configure_native_runtime", return_value=Path("/sonic")
            ),
            mock.patch.object(MODULE, "_sonic_commit", return_value="deadbeef"),
            mock.patch.object(
                MODULE, "NativeProcessGroup", return_value=process_group
            ),
            mock.patch.object(MODULE.signal, "getsignal", return_value="previous"),
            mock.patch.object(MODULE.signal, "signal", side_effect=record_signal),
        ):
            with self.assertRaisesRegex(RuntimeError, "deploy failed"):
                MODULE.main()

        self.assertEqual([event[0] for event in events[:2]], ["signal", "signal"])
        self.assertLess(events.index("pico-start"), events.index("deploy-start"))
        self.assertIn("processes-close", events)
        self.assertIn("simulator-close", events)


if __name__ == "__main__":
    unittest.main()
