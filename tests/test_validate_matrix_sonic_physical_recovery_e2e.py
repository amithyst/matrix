import importlib.util
import sys
import unittest
from pathlib import Path
from unittest import mock


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "validate_matrix_sonic_physical_recovery_e2e.py"
)
SPEC = importlib.util.spec_from_file_location(
    "validate_matrix_sonic_physical_recovery_e2e", SCRIPT
)
assert SPEC is not None and SPEC.loader is not None
validator = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = validator
SPEC.loader.exec_module(validator)


def passing_status():
    return {
        "passed": True,
        "acceptance_failures": [],
        "game_fall_recovery": {
            "mode": "physical",
            "episodes": 1,
            "recoveries": 1,
            "deploy_generation": 2,
            "state": "GAME_SONIC",
            "fail_closed": False,
            "physical_only": True,
            "previous_sonic_writer_revoked": True,
            "simulator_state_mutation": False,
            "latest_completed_recovery_worker_episode_id": 1,
            "worker": {
                "episode_id": 2,
                "first_write": False,
                "stopped": False,
                "completed_episodes": [
                    {
                        "episode_id": 1,
                        "go_sent": True,
                        "first_write": True,
                        "amp_hold_first_write": True,
                        "amp_hold_sent": True,
                        "joint_hold_first_write": False,
                        "joint_hold_sent": False,
                        "hold_kind": "amp",
                        "stop_sent": True,
                        "stopped": True,
                        "command_history": [
                            {"command": "GO"},
                            {"command": "ENTER_AMP_HOLD"},
                            {"command": "STOP"},
                        ],
                        "policy_switch_first_writes": [],
                    }
                ],
            },
            "replacement_sonic_writer_gate": {
                "ready_no_lowcmd_writer": True,
                "shadow_ready_no_lowcmd_writer": True,
                "first_write": True,
                "reentry_alignment_complete": True,
                "reentry_policy_full_control": True,
            },
        },
        "game_input": {"moving_command_frames": 1},
        "active_lowcmd": True,
        "root_xyz": [0.0, 0.0, 0.72],
        "root_up_z": 0.99,
        "current_fall_detected": False,
        "instability_resets": 0,
        "numerical_error": None,
        "failed_child_exit_code": None,
    }


def passing_probe():
    return {
        "force_ticks_applied": 8,
        "force_ticks_required": 8,
        "minimum_root_z_m": 0.10,
        "minimum_root_up_z": 0.95,
        "mutation_counter_scope": (
            "physical_knockdown_probe_direct_operations_only"
        ),
        "qpos_writes": 0,
        "qvel_writes": 0,
        "reset_calls": 0,
        "reload_calls": 0,
        "teleports": 0,
    }


def passing_input_peer():
    return {
        "recovery_observed": True,
        "movement_packets_sent": 1,
        "last_error": None,
    }


def evaluate(
    status,
    probe,
    input_peer,
    *,
    runtime_return_code=0,
):
    return validator._evaluate(
        status,
        probe,
        input_peer,
        runtime_return_code=runtime_return_code,
    )


def passing_native_sonic_status():
    status = passing_status()
    status["game_fall_recovery"] = {
        "mode": "sonic",
        "state": "monitoring",
        "policy_command": "KNEEL_TWO_LEGS_TO_IDLE",
        "episodes": 1,
        "recoveries": 1,
        "timed_out": False,
    }
    return status


def passing_direct_sonic_handoff_status():
    status = passing_status()
    status["game_fall_recovery"]["handoff_mode"] = "sonic"
    status["game_fall_recovery"]["worker"]["completed_episodes"][0][
        "amp_hold_first_write"
    ] = False
    status["game_fall_recovery"]["worker"]["completed_episodes"][0][
        "joint_hold_first_write"
    ] = False
    status["game_fall_recovery"]["worker"]["completed_episodes"][0][
        "amp_hold_sent"
    ] = False
    status["game_fall_recovery"]["worker"]["completed_episodes"][0][
        "joint_hold_sent"
    ] = False
    status["game_fall_recovery"]["worker"]["completed_episodes"][0][
        "hold_kind"
    ] = None
    status["game_fall_recovery"]["worker"]["completed_episodes"][0][
        "command_history"
    ] = [{"command": "GO"}, {"command": "STOP"}]
    return status


class PhysicalRecoveryEvidenceTests(unittest.TestCase):
    @staticmethod
    def game_input_peer():
        return validator.GameInputPeer(
            socket_path=Path("/tmp/matrix-recovery-test.sock"),
            status_path=Path("/tmp/matrix-recovery-test-status.json"),
            move_seconds=1.0,
            applied_hold_seconds=0.5,
            neutral_after_seconds=0.1,
            completion_event=validator.threading.Event(),
        )

    @staticmethod
    def record_sent(peer, *, moving):
        sequence = peer._sequence
        peer._sequence += 1
        peer._record_sent(moving=moving)
        return sequence

    def test_game_input_peer_reads_only_valid_nonnegative_move_frames(self):
        read = validator.GameInputPeer._moving_command_frames
        self.assertEqual(read({"game_input": {"moving_command_frames": 7}}), 7)
        self.assertEqual(read({"game_input": {"moving_command_frames": True}}), 0)
        self.assertEqual(read({"game_input": {"moving_command_frames": -1}}), 0)
        self.assertEqual(read({"game_input": {"moving_command_frames": "7"}}), 0)
        self.assertEqual(read({"game_input": []}), 0)
        self.assertEqual(read(None), 0)

    def test_game_input_peer_completes_success_and_failed_attempts_in_process(self):
        for applied in (False, True):
            peer = self.game_input_peer()
            if applied:
                peer.move_applied_monotonic_s = 10.0

            peer._complete_attempt()

            with self.subTest(applied=applied):
                self.assertTrue(peer.completion_event.is_set())
                self.assertTrue(peer.stop_requested_after_attempt)
                self.assertIs(peer.stop_requested_after_success, applied)

    def test_game_input_peer_protocol_error_completes_failed_attempt(self):
        peer = self.game_input_peer()
        peer._client = object()
        with mock.patch.object(
            peer,
            "_send",
            side_effect=OSError("injected socket failure"),
        ):
            peer._run()

        self.assertEqual(peer.last_error, "OSError: injected socket failure")
        self.assertTrue(peer.completion_event.is_set())
        self.assertTrue(peer.stop_requested_after_attempt)
        self.assertFalse(peer.stop_requested_after_success)

    def test_runtime_deadline_must_be_finite_and_positive(self):
        self.assertEqual(
            validator._runtime_deadline_seconds(["--max-seconds", "120"]),
            120.0,
        )
        for arguments in (
            [],
            ["--max-seconds", "0"],
            ["--max-seconds", "-1"],
            ["--max-seconds", "nan"],
            ["--max-seconds", "inf"],
            ["--max-seconds", "not-a-number"],
        ):
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                validator._runtime_deadline_seconds(arguments)

    def test_game_input_peer_sends_neutral_until_rearm_is_acknowledged(self):
        peer = self.game_input_peer()
        status = passing_status()
        status["game_input"] = {
            "moving_command_frames": 0,
            "mode": "deadman",
            "safe_stop": True,
            "locomotion_mode": 0,
            "speed_mps": 0.0,
            "sequence": 1,
            "stop_reason": "awaiting_neutral",
        }

        self.assertFalse(peer._movement_requested(status, now=10.0))
        self.assertTrue(peer.recovery_observed)
        self.assertIsNone(peer.move_started_monotonic_s)
        first_neutral_sequence = self.record_sent(peer, moving=False)

        self.assertFalse(peer._movement_requested(status, now=10.02))
        self.assertIsNone(peer.move_started_monotonic_s)
        self.record_sent(peer, moving=False)
        self.assertEqual(peer.post_recovery_neutral_packets_sent, 2)

        status["game_input"].update(
            {
                "mode": "idle",
                "safe_stop": False,
                "locomotion_mode": 0,
                "speed_mps": 0.0,
                "sequence": first_neutral_sequence,
                "stop_reason": None,
            }
        )
        self.assertTrue(peer._movement_requested(status, now=10.04))
        self.assertTrue(peer.neutral_handshake_complete)
        self.assertEqual(peer.move_started_monotonic_s, 10.04)
        self.assertEqual(peer.moving_command_frames_at_resume, 0)

    def test_game_input_peer_requires_post_recovery_neutral_even_if_clear(self):
        peer = self.game_input_peer()
        status = passing_status()
        status["game_input"] = {
            "moving_command_frames": 0,
            "mode": "idle",
            "safe_stop": False,
            "locomotion_mode": 0,
            "speed_mps": 0.0,
            "sequence": 1,
            "stop_reason": None,
        }

        self.assertFalse(peer._movement_requested(status, now=20.0))
        self.assertIsNone(peer.move_started_monotonic_s)
        neutral_sequence = self.record_sent(peer, moving=False)
        self.assertFalse(peer._movement_requested(status, now=20.01))
        status["game_input"]["sequence"] = neutral_sequence
        self.assertTrue(peer._movement_requested(status, now=20.02))
        self.assertEqual(peer.move_started_monotonic_s, 20.02)

    def test_game_input_peer_never_sends_w_before_policy_full_control(self):
        peer = self.game_input_peer()
        status = passing_status()
        status["game_fall_recovery"]["replacement_sonic_writer_gate"][
            "reentry_policy_full_control"
        ] = False
        status["game_input"] = {
            "moving_command_frames": 0,
            "mode": "idle",
            "safe_stop": False,
            "locomotion_mode": 0,
            "speed_mps": 0.0,
            "sequence": 1,
            "stop_reason": None,
        }

        self.assertFalse(peer._movement_requested(status, now=25.0))
        self.assertFalse(peer.recovery_observed)
        self.assertIsNone(peer.move_started_monotonic_s)
        self.record_sent(peer, moving=False)

        status["game_fall_recovery"]["replacement_sonic_writer_gate"][
            "reentry_policy_full_control"
        ] = True
        self.assertFalse(peer._movement_requested(status, now=25.02))
        neutral_sequence = self.record_sent(peer, moving=False)
        status["game_input"]["sequence"] = neutral_sequence
        self.assertTrue(peer._movement_requested(status, now=25.04))
        self.assertTrue(peer.recovery_observed)
        self.assertIsNotNone(peer.move_started_monotonic_s)

    def test_game_input_peer_does_not_treat_missing_stop_reason_as_ack(self):
        peer = self.game_input_peer()
        status = passing_status()
        status["game_input"] = {"moving_command_frames": 0}

        self.assertFalse(peer._movement_requested(status, now=30.0))
        self.record_sent(peer, moving=False)
        self.assertFalse(peer._movement_requested(status, now=30.02))
        self.assertIsNone(peer.move_started_monotonic_s)

    def test_game_input_peer_rejects_stale_recovery_stop_as_neutral_ack(self):
        peer = self.game_input_peer()
        status = passing_status()
        status["game_input"] = {
            "moving_command_frames": 0,
            "mode": "deadman",
            "safe_stop": True,
            "locomotion_mode": 0,
            "speed_mps": 0.0,
            "sequence": 1,
            "stop_reason": "physical_fall_recovery",
        }

        self.assertFalse(peer._movement_requested(status, now=40.0))
        neutral_sequence = self.record_sent(peer, moving=False)
        status["game_input"]["sequence"] = neutral_sequence
        self.assertFalse(peer._movement_requested(status, now=40.02))
        self.assertIsNone(peer.move_started_monotonic_s)

    def test_game_input_peer_rearms_after_safe_stop_without_resetting_deadline(self):
        peer = self.game_input_peer()
        status = passing_status()
        status["game_input"] = {
            "moving_command_frames": 0,
            "mode": "deadman",
            "safe_stop": True,
            "locomotion_mode": 0,
            "speed_mps": 0.0,
            "sequence": 1,
            "stop_reason": "awaiting_neutral",
        }

        self.assertFalse(peer._movement_requested(status, now=50.0))
        first_neutral_sequence = self.record_sent(peer, moving=False)
        status["game_input"].update(
            {
                "mode": "idle",
                "safe_stop": False,
                "sequence": first_neutral_sequence,
                "stop_reason": None,
            }
        )
        self.assertTrue(peer._movement_requested(status, now=50.02))
        first_move_started = peer.move_started_monotonic_s
        self.record_sent(peer, moving=True)

        status["game_input"].update(
            {
                "mode": "deadman",
                "safe_stop": True,
                "stop_reason": "awaiting_neutral",
            }
        )
        self.assertFalse(peer._movement_requested(status, now=50.04))
        self.assertFalse(peer.neutral_handshake_complete)
        self.assertEqual(peer.neutral_rearm_count, 1)
        rearm_barrier = peer.neutral_sequence_barrier
        self.assertIsNotNone(rearm_barrier)
        self.record_sent(peer, moving=False)

        status["game_input"].update(
            {
                "mode": "idle",
                "safe_stop": False,
                "sequence": rearm_barrier - 1,
                "stop_reason": None,
            }
        )
        self.assertFalse(peer._movement_requested(status, now=50.06))
        status["game_input"]["sequence"] = rearm_barrier
        self.assertTrue(peer._movement_requested(status, now=50.08))
        self.assertEqual(peer.move_started_monotonic_s, first_move_started)

        status["game_input"].update(
            {
                "mode": "deadman",
                "safe_stop": True,
                "stop_reason": "awaiting_neutral",
            }
        )
        self.assertFalse(peer._movement_requested(status, now=51.03))
        self.assertEqual(peer.move_finished_monotonic_s, 51.03)

    def test_zero_scoped_probe_mutation_counters_pass(self):
        self.assertEqual(
            evaluate(
                passing_status(), passing_probe(), passing_input_peer()
            ),
            [],
        )

    def test_final_evidence_rejects_nonzero_or_missing_runtime_return_code(self):
        for return_code, expected in (
            (1, "runtime_return_code_nonzero"),
            (-9, "runtime_return_code_nonzero"),
            (None, "runtime_return_code_missing_or_invalid"),
            (True, "runtime_return_code_missing_or_invalid"),
        ):
            with self.subTest(return_code=return_code):
                self.assertIn(
                    expected,
                    evaluate(
                        passing_status(),
                        passing_probe(),
                        passing_input_peer(),
                        runtime_return_code=return_code,
                    ),
                )

    def test_final_evidence_requires_strict_runtime_passed_attestation(self):
        for passed in (False, 1, None):
            status = passing_status()
            status["passed"] = passed
            with self.subTest(passed=passed):
                self.assertIn(
                    "runtime_status_not_passed",
                    evaluate(status, passing_probe(), passing_input_peer()),
                )
        missing = passing_status()
        del missing["passed"]
        self.assertIn(
            "runtime_status_not_passed",
            evaluate(missing, passing_probe(), passing_input_peer()),
        )

    def test_final_evidence_rejects_acceptance_failures_and_missing_field(self):
        status = passing_status()
        status["acceptance_failures"] = ["runtime_not_verified"]
        self.assertIn(
            "runtime_acceptance_failures_present",
            evaluate(status, passing_probe(), passing_input_peer()),
        )
        del status["acceptance_failures"]
        self.assertIn(
            "runtime_acceptance_failures_missing_or_invalid",
            evaluate(status, passing_probe(), passing_input_peer()),
        )

    def test_each_nonzero_probe_mutation_counter_fails(self):
        for field in (
            "qpos_writes",
            "qvel_writes",
            "reset_calls",
            "reload_calls",
            "teleports",
        ):
            probe = passing_probe()
            probe[field] = 1
            with self.subTest(field=field):
                self.assertIn(
                    f"probe_{field}_nonzero",
                    evaluate(
                        passing_status(), probe, passing_input_peer()
                    ),
                )

    def test_missing_probe_counter_fails_instead_of_defaulting_to_zero(self):
        probe = passing_probe()
        del probe["qpos_writes"]
        self.assertIn(
            "probe_qpos_writes_nonzero",
            evaluate(passing_status(), probe, passing_input_peer()),
        )

    def test_probe_telemetry_declares_direct_operation_scope(self):
        probe = validator.PhysicalKnockdownProbe(
            direction="forward",
            force_newtons=3400.0,
            duration_s=0.04,
            ready_hold_s=0.5,
        )
        telemetry = probe.telemetry()
        self.assertEqual(
            telemetry["mutation_counter_scope"],
            "physical_knockdown_probe_direct_operations_only",
        )
        self.assertFalse(telemetry["simulator_state_mutation_by_probe"])
        for field in (
            "qpos_writes",
            "qvel_writes",
            "reset_calls",
            "reload_calls",
            "teleports",
        ):
            self.assertEqual(telemetry[field], 0)

    def test_native_sonic_joint_control_recovery_passes_without_handoff_fields(self):
        self.assertEqual(
            evaluate(
                passing_native_sonic_status(),
                passing_probe(),
                passing_input_peer(),
            ),
            [],
        )

    def test_native_sonic_timeout_fails(self):
        status = passing_native_sonic_status()
        status["game_fall_recovery"]["timed_out"] = True
        self.assertIn(
            "native_sonic_recovery_timed_out",
            evaluate(status, passing_probe(), passing_input_peer()),
        )

    def test_physical_getup_can_skip_destabilizing_amp_hold(self):
        self.assertEqual(
            evaluate(
                passing_direct_sonic_handoff_status(),
                passing_probe(),
                passing_input_peer(),
            ),
            [],
        )

    def test_completed_episode_survives_new_standby_worker_telemetry(self):
        status = passing_direct_sonic_handoff_status()
        status["game_fall_recovery"]["worker"] = {
            "episode_id": 2,
            "first_write": False,
            "stopped": False,
            "completed_episodes": [
                {
                    "episode_id": 1,
                    "go_sent": True,
                    "first_write": True,
                    "amp_hold_first_write": False,
                    "amp_hold_sent": False,
                    "joint_hold_first_write": False,
                    "joint_hold_sent": False,
                    "hold_kind": None,
                    "stop_sent": True,
                    "stopped": True,
                    "command_history": [
                        {"command": "GO"},
                        {"command": "STOP"},
                    ],
                    "policy_switch_first_writes": [],
                }
            ],
        }

        self.assertEqual(
            evaluate(status, passing_probe(), passing_input_peer()),
            [],
        )

    def test_old_success_cannot_satisfy_a_new_episode(self):
        status = passing_direct_sonic_handoff_status()
        status["game_fall_recovery"][
            "latest_completed_recovery_worker_episode_id"
        ] = 3

        self.assertIn(
            "completed_recovery_worker_episode_not_found",
            evaluate(status, passing_probe(), passing_input_peer()),
        )


if __name__ == "__main__":
    unittest.main()
