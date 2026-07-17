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
        low_cmd_fresh: bool = False,
        low_cmd_received: bool = False,
        low_cmd_age_s: float | None = None,
        elastic_band_scale: float = 0.0,
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
            low_cmd_fresh=low_cmd_fresh,
            low_cmd_received=low_cmd_received,
            low_cmd_age_s=low_cmd_age_s,
            elastic_band_scale=elastic_band_scale,
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

    def test_absolute_physics_pacing_compensates_sleep_overshoot(self) -> None:
        with mock.patch.object(
            MODULE.time, "perf_counter", side_effect=[9.996, 10.00025]
        ), mock.patch.object(MODULE.time, "sleep") as sleep:
            next_deadline = MODULE._pace_absolute_deadline(10.0, 0.005)

        sleep.assert_called_once()
        self.assertAlmostEqual(sleep.call_args.args[0], 0.004)
        self.assertAlmostEqual(next_deadline, 10.005)
        self.assertIn(
            "simulator.step_once(rate_limit=False)",
            SCRIPT_PATH.read_text(encoding="utf-8"),
        )

    def test_absolute_physics_pacing_resets_after_sustained_overrun(self) -> None:
        with mock.patch.object(
            MODULE.time, "perf_counter", return_value=10.011
        ), mock.patch.object(MODULE.time, "sleep") as sleep:
            next_deadline = MODULE._pace_absolute_deadline(10.0, 0.005)

        sleep.assert_not_called()
        self.assertAlmostEqual(next_deadline, 10.016)

    def test_absolute_physics_pacing_does_not_accumulate_sleep_overshoot(self) -> None:
        clock = [0.0]

        def sleep_with_overshoot(duration: float) -> None:
            clock[0] += duration + 0.00025

        deadline = 0.005
        with mock.patch.object(
            MODULE.time, "perf_counter", side_effect=lambda: clock[0]
        ), mock.patch.object(MODULE.time, "sleep", side_effect=sleep_with_overshoot):
            for _ in range(200):
                deadline = MODULE._pace_absolute_deadline(deadline, 0.005)

        self.assertAlmostEqual(clock[0], 1.00025)
        self.assertAlmostEqual(deadline, 1.005)

    def test_qualified_acceptance_rejects_weaker_lock_gates(self) -> None:
        lock = json.loads(
            (REPO_ROOT / "config/runtime/matrix-sonic.lock.json").read_text(
                encoding="utf-8"
            )
        )["acceptance"]
        base = {
            "qualified_runtime": True,
            "min_active_seconds": lock["active_lowcmd_seconds_min"],
            "min_displacement_m": lock["root_displacement_xy_min_m"],
            "min_physics_hz": lock["physics_hz_min"],
            "min_rtf": lock["rtf_min"],
            "low_cmd_fresh_timeout_seconds": lock[
                "low_cmd_fresh_timeout_seconds"
            ],
            "max_resets": lock["instability_resets_max"],
            "fail_on_fall": True,
        }
        MODULE._validate_qualified_acceptance(SimpleNamespace(**base))

        weaker_values = {
            "min_active_seconds": 0.0,
            "min_displacement_m": 0.0,
            "min_physics_hz": 0.0,
            "min_rtf": 0.0,
            "low_cmd_fresh_timeout_seconds": 1.0,
            "max_resets": lock["instability_resets_max"] + 1,
            "fail_on_fall": False,
        }
        for argument, weaker in weaker_values.items():
            values = dict(base)
            values[argument] = weaker
            with self.subTest(argument=argument), self.assertRaisesRegex(
                SystemExit, argument
            ):
                MODULE._validate_qualified_acceptance(SimpleNamespace(**values))

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

    def test_fake_ue_exit_42_invalidates_otherwise_passing_status(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fake_ue = root / "fake-ue"
            fake_ue.write_text("#!/usr/bin/env bash\nexit 42\n", encoding="utf-8")
            fake_ue.chmod(0o755)
            ue_result = subprocess.run([str(fake_ue)], check=False)
            self.assertEqual(ue_result.returncode, 42)

            failure_file = root / "failure.json"
            failure_file.write_text(
                json.dumps({"name": "ue", "exit_code": ue_result.returncode}),
                encoding="utf-8",
            )
            failure = MODULE._read_external_failure(failure_file)
            self.assertEqual(failure, ("ue", 42))

            status = root / "status.json"
            status.write_text(
                json.dumps(
                    {
                        "acceptance_failures": [],
                        "completed": True,
                        "passed": True,
                        "termination_reason": "max_seconds",
                    }
                ),
                encoding="utf-8",
            )
            assert failure is not None
            MODULE._record_external_child_failure(status, failure)
            payload = json.loads(status.read_text(encoding="utf-8"))

            self.assertFalse(payload["passed"])
            self.assertFalse(payload["completed"])
            self.assertEqual(payload["failed_child_name"], "ue")
            self.assertEqual(payload["failed_child_exit_code"], 42)
            self.assertEqual(payload["termination_reason"], "child_exit")
            self.assertIn("native_child_exit:ue:42", payload["acceptance_failures"])

    def test_late_ue_failure_creates_missing_final_status(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            status = Path(temporary) / "status.json"

            MODULE._record_external_child_failure(status, ("ue", 23))

            payload = json.loads(status.read_text(encoding="utf-8"))
            self.assertFalse(payload["passed"])
            self.assertFalse(payload["completed"])
            self.assertEqual(payload["failed_child_name"], "ue")
            self.assertEqual(payload["failed_child_exit_code"], 23)
            self.assertEqual(payload["termination_reason"], "child_exit")
            self.assertEqual(
                payload["acceptance_failures"], ["native_child_exit:ue:23"]
            )

    def test_normal_ue_lifecycle_without_failure_record_is_not_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            failure_file = Path(temporary) / "failure.json"
            self.assertIsNone(MODULE._read_external_failure(failure_file))

        run_sim = (REPO_ROOT / "scripts/run_sim.sh").read_text(encoding="utf-8")
        supervisor = (REPO_ROOT / "scripts/supervise_matrix_ue.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("coproc MATRIX_UE_SUPERVISOR", run_sim)
        self.assertIn('wait "$UE_SUPERVISOR_PID"', run_sim)
        self.assertNotIn('kill -0 "$ue_pid"', run_sim)
        self.assertNotIn('PIDS+=("$UE_PID")', run_sim)
        self.assertNotIn("UE_EXPECTED_STOP_FILE", run_sim)
        self.assertIn("os.WNOWAIT", supervisor)
        self.assertIn("start_new_session=True", supervisor)
        self.assertIn("signal.SIGKILL", supervisor)

    def test_ue_supervisor_classifies_unexpected_and_expected_exit(self) -> None:
        supervisor = REPO_ROOT / "scripts/supervise_matrix_ue.py"
        cases = (
            ("unexpected", ["/bin/sh", "-c", "exit 42"], 42, 42),
            ("expected", ["/bin/sh", "-c", "while :; do sleep 1; done"], 0, None),
        )
        for name, command, expected_code, expected_failure in cases:
            with self.subTest(case=name), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                pid_file = root / "ue.pid"
                failure_file = root / "failure.json"
                process = subprocess.Popen(
                    [
                        sys.executable,
                        str(supervisor),
                        "--pid-file",
                        str(pid_file),
                        "--failure-file",
                        str(failure_file),
                        "--log",
                        str(root / "ue.log"),
                        "--expected-parent-pid",
                        str(os.getpid()),
                        "--",
                        *command,
                    ],
                    stdin=subprocess.PIPE,
                )
                try:
                    deadline = time.monotonic() + 3.0
                    while not pid_file.exists() and time.monotonic() < deadline:
                        time.sleep(0.01)
                    self.assertTrue(pid_file.exists(), "supervisor did not publish UE PID")
                    if name == "unexpected":
                        deadline = time.monotonic() + 3.0
                        while not failure_file.exists() and time.monotonic() < deadline:
                            time.sleep(0.01)
                    assert process.stdin is not None
                    process.stdin.write(b"stop\n")
                    process.stdin.flush()
                    process.stdin.close()
                    self.assertEqual(process.wait(timeout=6.0), expected_code)
                    failure = (
                        json.loads(failure_file.read_text(encoding="utf-8"))
                        if failure_file.exists()
                        else None
                    )
                    if expected_failure is None:
                        self.assertIsNone(failure)
                    else:
                        self.assertEqual(
                            failure, {"name": "ue", "exit_code": expected_failure}
                        )
                finally:
                    if process.poll() is None:
                        process.kill()
                        process.wait(timeout=3.0)

    def test_malformed_ue_failure_never_falls_back_to_exit_zero(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            failure_file = Path(temporary) / "failure.json"
            failure_file.write_text('{"name":"ue"}\n', encoding="utf-8")
            self.assertEqual(
                MODULE._read_external_failure(failure_file),
                ("ue", MODULE._UNKNOWN_EXTERNAL_EXIT_CODE),
            )
            self.assertNotEqual(MODULE._UNKNOWN_EXTERNAL_EXIT_CODE, 0)

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
            active_lock = json.loads(lock.read_text(encoding="utf-8"))
            required_checks = [
                "Matrix source commit",
                "Matrix tracked source clean",
                "Matrix ignored source overlays absent",
                "native runtime Python",
                "native runtime Python prefix",
                "native runtime Python isolation",
                "native SONIC source clean",
                "native SONIC ignored source overlays absent",
                "native SONIC Git checkout required",
                "native SONIC commit",
                "native SONIC Python API",
                "gear_sonic import origin",
                "SONIC deploy dependency closure",
                "Matrix UE dependency closure",
                "TensorRT ABI",
            ]
            payload = {
                "passed": True,
                "checks": [
                    {"name": name, "ok": True} for name in required_checks
                ],
                "profile": "trna",
                "lock": str(lock),
                "lock_sha256": MODULE._sha256_file(lock),
                "matrix_root": str(REPO_ROOT),
                "matrix_commit": matrix_commit,
                "sonic_root": "/sonic",
                "runtime_root": "/runtime",
                "python": str((REPO_ROOT / ".venv-audit/bin/python").absolute()),
                "python_prefix": str((REPO_ROOT / ".venv-audit").absolute()),
                "pico_python": None,
                "pico_wheel": None,
                "full_hashes": True,
                "sonic_git_checkout": True,
                "qualification_eligible": True,
                "verification_flags": {
                    "fast": False,
                    "skip_dynamic": False,
                    "skip_installed_assets": False,
                    "require_git_sonic": True,
                },
                "verification_inventory": {
                    "runtime_files_expected": len(active_lock["runtime_files"]),
                    "runtime_files_checked": len(active_lock["runtime_files"]),
                    "runtime_trees_expected": len(active_lock["runtime_trees"]),
                    "runtime_trees_checked": len(active_lock["runtime_trees"]),
                    "installed_files_expected": len(
                        active_lock["matrix_release"]["installed_files"]
                    ),
                    "installed_files_checked": len(
                        active_lock["matrix_release"]["installed_files"]
                    ),
                    "installed_trees_expected": len(
                        active_lock["matrix_release"]["installed_trees"]
                    ),
                    "installed_trees_checked": len(
                        active_lock["matrix_release"]["installed_trees"]
                    ),
                    "dynamic_checks_performed": True,
                },
                "qualification_required_checks": required_checks,
                "missing_qualification_checks": [],
                "launch_roots": MODULE._expected_receipt_roots(
                    "trna", Path("/runtime"), Path("/sonic")
                ),
                "launch_environment": {
                    "ld_library_path": os.environ.get("LD_LIBRARY_PATH", ""),
                    "pythonpath": os.environ.get("PYTHONPATH", ""),
                    "tensorrt_root": os.environ.get("TensorRT_ROOT", ""),
                    "python_pycache_prefix": os.environ.get(
                        "PYTHONPYCACHEPREFIX", ""
                    ),
                    "python_dont_write_bytecode": os.environ.get(
                        "PYTHONDONTWRITEBYTECODE", ""
                    ),
                },
            }
            receipt_path.write_text(json.dumps(payload), encoding="utf-8")
            args = SimpleNamespace(
                qualified_runtime=True,
                runtime_lock_sha256=payload["lock_sha256"],
                matrix_commit=matrix_commit,
                verification_receipt=receipt_path,
                qualification_profile="trna",
                sonic_root=Path("/sonic"),
                control_source="planner",
                pico_python=None,
            )
            def validate_receipt():
                with (
                    mock.patch.object(
                        MODULE.sys,
                        "executable",
                        str(REPO_ROOT / ".venv-audit/bin/python"),
                    ),
                    mock.patch.object(
                        MODULE.sys, "prefix", str(REPO_ROOT / ".venv-audit")
                    ),
                ):
                    return MODULE._validate_qualification_receipt(args)

            self.assertEqual(validate_receipt(), payload)
            self.assertEqual(args.verification_receipt, receipt_path.resolve())
            self.assertEqual(
                args.verification_receipt_sha256,
                MODULE._sha256_file(receipt_path),
            )

            payload["full_hashes"] = False
            receipt_path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(SystemExit, "receipt does not match"):
                validate_receipt()

            payload["full_hashes"] = True
            payload["verification_flags"]["skip_dynamic"] = True
            receipt_path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(SystemExit, "receipt does not match"):
                validate_receipt()

            payload["verification_flags"]["skip_dynamic"] = False
            payload["passed"] = False
            receipt_path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(SystemExit, "receipt does not match"):
                validate_receipt()

    def test_qualified_model_rejects_receipt_model_root_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            model = root / "scene.xml"
            model.write_text("<mujoco/>\n", encoding="utf-8")
            (root / "manifest.json").write_text(
                json.dumps(
                    {
                        "canonical_model": "/different/model.xml",
                        "canonical_meshes": "/different/meshes",
                    }
                ),
                encoding="utf-8",
            )
            args = SimpleNamespace(
                qualified_runtime=True,
                sonic_root=Path("/sonic"),
                scenario_layout_sha256=None,
            )
            receipt = {
                "launch_roots": MODULE._expected_receipt_roots(
                    "trna", Path("/runtime"), Path("/sonic")
                )
            }
            with self.assertRaisesRegex(SystemExit, "canonical path"):
                MODULE._validate_qualified_model(args, model, receipt)

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
    @mock.patch.object(MODULE, "_peek_child_returncode", side_effect=[None, 0])
    def test_native_deploy_gets_a_graceful_stop_window(
        self, _peek, sleep
    ) -> None:
        process = mock.Mock()
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

    @mock.patch.object(MODULE, "_peek_child_returncode", return_value=7)
    @mock.patch.object(MODULE.os, "killpg")
    def test_process_group_close_signals_group_after_leader_exit(
        self, killpg, _peek
    ) -> None:
        process = mock.Mock(pid=4321)

        def signal_group(_process_group, signum):
            if signum == MODULE.signal.SIGKILL:
                return None

        killpg.side_effect = signal_group
        group = MODULE.NativeProcessGroup(Path("/sonic"), {})
        group.children.append(("deploy", process))

        group.close()

        self.assertIn(mock.call(4321, MODULE.signal.SIGTERM), killpg.call_args_list)
        self.assertIn(mock.call(4321, MODULE.signal.SIGKILL), killpg.call_args_list)
        process.wait.assert_called_once_with(timeout=2.0)

    def test_process_group_close_kills_group_before_exact_reap(self) -> None:
        events = []
        observed = iter((None, 0))
        process = mock.Mock(pid=4321)
        process.wait.side_effect = lambda **_kwargs: events.append("wait") or 0
        group = MODULE.NativeProcessGroup(Path("/sonic"), {})
        group.children.append(("deploy", process))

        def peek(_process):
            events.append("peek")
            return next(observed)

        def signal_group(_process_group, signum):
            events.append(
                "term" if signum == MODULE.signal.SIGTERM else "kill"
            )

        with (
            mock.patch.object(MODULE, "_peek_child_returncode", side_effect=peek),
            mock.patch.object(MODULE.os, "killpg", side_effect=signal_group),
        ):
            group.close()

        self.assertEqual(events, ["peek", "term", "peek", "kill", "wait"])

    @mock.patch.object(MODULE.time, "monotonic", side_effect=[0.0, 6.0])
    @mock.patch.object(MODULE, "_peek_child_returncode", return_value=None)
    @mock.patch.object(MODULE.os, "killpg")
    def test_process_group_close_reports_child_after_sigkill(
        self, _killpg, _peek, _monotonic
    ) -> None:
        process = mock.Mock(pid=4321)
        process.wait.side_effect = subprocess.TimeoutExpired("child", 2.0)
        group = MODULE.NativeProcessGroup(Path("/sonic"), {})
        group.children.append(("deploy", process))

        with self.assertRaisesRegex(RuntimeError, "did not exit after SIGKILL"):
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

    def test_process_group_boundary_observes_exit_without_reaping(self) -> None:
        for exit_code in (0, 42):
            with self.subTest(exit_code=exit_code):
                process = subprocess.Popen(
                    [sys.executable, "-c", f"raise SystemExit({exit_code})"],
                    start_new_session=True,
                )
                group = MODULE.NativeProcessGroup(Path("/sonic"), {})
                group.children.append(("deploy", process))
                deadline = time.monotonic() + 5.0
                while MODULE._peek_child_returncode(process) is None:
                    if time.monotonic() >= deadline:
                        self.fail("native child did not exit")
                    time.sleep(0.01)

                self.assertEqual(
                    group.begin_expected_stop(), ("deploy", exit_code)
                )
                self.assertIsNone(process.returncode)
                group.close()

    def test_process_group_boundary_authorizes_later_stop(self) -> None:
        process = subprocess.Popen(
            [sys.executable, "-c", "import time; time.sleep(60)"],
            start_new_session=True,
        )
        group = MODULE.NativeProcessGroup(Path("/sonic"), {})
        group.children.append(("deploy", process))
        try:
            self.assertIsNone(group.begin_expected_stop())
            self.assertIsNone(group.failed_child())
        finally:
            group.close()

    def test_process_group_close_kills_term_ignoring_descendant_before_reap(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            pid_file = Path(temporary) / "grandchild.pid"
            leader_code = "\n".join(
                (
                    "from pathlib import Path",
                    "import subprocess,sys,time",
                    "code='import signal,time; signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(60)'",
                    "child=subprocess.Popen([sys.executable, '-c', code])",
                    "Path(sys.argv[1]).write_text(str(child.pid), encoding='utf-8')",
                    "time.sleep(60)",
                )
            )
            process = subprocess.Popen(
                [sys.executable, "-c", leader_code, str(pid_file)],
                start_new_session=True,
            )
            deadline = time.monotonic() + 5.0
            while not pid_file.is_file():
                if time.monotonic() >= deadline:
                    process.kill()
                    process.wait(timeout=5.0)
                    self.fail("native grandchild pid was not published")
                time.sleep(0.01)
            grandchild_pid = int(pid_file.read_text(encoding="utf-8"))
            group = MODULE.NativeProcessGroup(Path("/sonic"), {})
            group.children.append(("deploy", process))

            self.assertIsNone(group.begin_expected_stop())
            group.close()

            deadline = time.monotonic() + 5.0
            while self.process_is_running(grandchild_pid) and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertFalse(self.process_is_running(grandchild_pid))

    def test_startup_failure_closes_simulator_and_started_children(self) -> None:
        events = []
        simulator = mock.Mock()
        simulator.get_state_snapshot.return_value = self.snapshot()
        simulator.close.side_effect = lambda: events.append("simulator-close")

        process_group = mock.Mock()
        process_group.failed_child.return_value = None
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
            external_failure_file=None,
            ue_pid=None,
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

    def test_preexisting_ue_exit_zero_prevents_native_children(self) -> None:
        class FakeArray(list):
            def copy(self):
                return FakeArray(self)

            def __sub__(self, other):
                return FakeArray(
                    left - right for left, right in zip(self, other, strict=True)
                )

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            failure_file = root / "failure.json"
            failure_file.write_text(
                json.dumps({"name": "ue", "exit_code": 0}),
                encoding="utf-8",
            )
            status_file = root / "status.json"

            simulator = mock.Mock()
            simulator.get_state_snapshot.return_value = self.snapshot()
            process_group = mock.Mock()
            process_group.failed_child.return_value = None
            process_group.begin_expected_stop.return_value = None

            fake_numpy = ModuleType("numpy")
            fake_numpy.float64 = float
            fake_numpy.asarray = lambda values, dtype=None: FakeArray(values)
            fake_numpy.linalg = SimpleNamespace(
                norm=lambda values: math.sqrt(sum(value * value for value in values))
            )
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
                max_seconds=0.0,
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
                status_file=status_file,
                qualified_runtime=False,
                qualification_profile=None,
                runtime_lock_sha256=None,
                scenario_layout_sha256=None,
                matrix_commit=None,
                verification_receipt=None,
                expected_parent_pid=None,
                external_failure_file=failure_file,
                ue_pid=4321,
                print_every=2.0,
                startup_band=False,
                startup_band_hold=4.0,
                startup_band_fade=3.0,
            )

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
                mock.patch.object(MODULE.signal, "signal"),
            ):
                result = MODULE.main()

            self.assertEqual(result, 2)
            process_group.start_pico.assert_not_called()
            process_group.start_deploy.assert_not_called()
            process_group.close.assert_called_once_with()
            simulator.close.assert_called_once_with()
            payload = json.loads(status_file.read_text(encoding="utf-8"))
            self.assertFalse(payload["passed"])
            self.assertFalse(payload["completed"])
            self.assertEqual(payload["failed_child_name"], "ue")
            self.assertEqual(payload["failed_child_exit_code"], 0)
            self.assertEqual(payload["termination_reason"], "child_exit")
            self.assertEqual(payload["ue_pid"], 4321)
            self.assertIn("native_child_exit:ue:0", payload["acceptance_failures"])

            # Reuse the complete main fixture to inject an exit precisely at
            # the authoritative native pre-stop boundary.
            failure_file.unlink()
            status_file.unlink()
            process_group.reset_mock()
            process_group.failed_child.return_value = None
            process_group.begin_expected_stop.return_value = ("deploy", 0)
            args.max_seconds = 1e-9
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
                mock.patch.object(MODULE.signal, "signal"),
            ):
                boundary_result = MODULE.main()

            self.assertEqual(boundary_result, 2)
            boundary_payload = json.loads(status_file.read_text(encoding="utf-8"))
            self.assertFalse(boundary_payload["passed"])
            self.assertFalse(boundary_payload["completed"])
            self.assertEqual(boundary_payload["failed_child_name"], "deploy")
            self.assertEqual(boundary_payload["failed_child_exit_code"], 0)
            self.assertEqual(boundary_payload["termination_reason"], "child_exit")
            self.assertIn(
                "native_child_exit:deploy:0",
                boundary_payload["acceptance_failures"],
            )


if __name__ == "__main__":
    unittest.main()
