from __future__ import annotations

import importlib.util
import sys
import unittest
from dataclasses import FrozenInstanceError, replace
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "matrix_sonic_recovery.py"
SPEC = importlib.util.spec_from_file_location("matrix_sonic_recovery", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
recovery = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = recovery
SPEC.loader.exec_module(recovery)


def sample(now: float, **changes):
    base = recovery.RecoveryInput(
        now_s=now,
        fall_detected=False,
        root_z_m=0.30,
        root_up_z=0.20,
        root_linear_speed_m_s=0.0,
        root_angular_speed_rad_s=0.0,
        joint_velocity_rms_rad_s=0.0,
        lowcmd_fresh=True,
        lowcmd_age_s=0.01,
        deploy_alive=True,
        deploy_generation=7,
        deploy_process_ready=False,
        deploy_writer_ready=False,
        deploy_writer_created=True,
        deploy_writer_revoked=False,
        deploy_first_write=False,
        deploy_policy_full_control=False,
        deploy_safe_idle_hold=False,
        policy_alive=False,
        policy_ready=False,
        policy_first_write=False,
        policy_hold_first_write=False,
        reset_count=0,
        foot_contact=True,
        grounded_contact=True,
        neutral_confirmed=False,
    )
    return replace(base, **changes)


def drive_to_policy_recovering(machine):
    out = machine.step(sample(0.0))
    assert out.state is recovery.RecoveryState.GAME_SONIC
    assert out.start_policy_process
    out = machine.step(sample(0.1, fall_detected=True))
    assert out.request_sonic_stop
    out = machine.step(
        sample(
            0.2,
            fall_detected=True,
            deploy_alive=False,
            lowcmd_fresh=False,
            lowcmd_age_s=0.11,
        )
    )
    assert out.state is recovery.RecoveryState.SONIC_QUIET
    out = machine.step(
        sample(0.3, deploy_alive=False, lowcmd_fresh=False, lowcmd_age_s=0.2)
    )
    assert not out.start_policy_process
    out = machine.step(
        sample(
            0.46,
            deploy_alive=False,
            lowcmd_fresh=False,
            lowcmd_age_s=0.3,
            policy_alive=True,
            policy_ready=True,
        )
    )
    assert out.authorize_policy_writer
    return out


def stable_policy_sample(now: float, **changes):
    values = {
        "deploy_alive": True,
        "deploy_generation": 8,
        "deploy_writer_ready": False,
        "deploy_writer_created": False,
        "deploy_first_write": False,
        "policy_alive": True,
        "policy_ready": True,
        "policy_first_write": True,
        "lowcmd_fresh": True,
        "lowcmd_age_s": 0.01,
        "root_z_m": 0.70,
        "root_up_z": 0.97,
        "root_linear_speed_m_s": 0.10,
        "root_angular_speed_rad_s": 0.30,
        "joint_velocity_rms_rad_s": 0.40,
        "foot_contact": True,
    }
    values.update(changes)
    return sample(now, **values)


def drive_to_amp_holding(machine):
    drive_to_policy_recovering(machine)
    first_write = stable_policy_sample(0.5, deploy_alive=False)
    out = machine.step(first_write)
    assert out.start_sonic
    assert out.state is recovery.RecoveryState.POLICY_RECOVERING
    stable = stable_policy_sample(0.6)
    out = machine.step(stable)
    assert out.state is recovery.RecoveryState.POLICY_GETUP_STABLE
    out = machine.step(replace(stable, now_s=2.1))
    assert out.request_policy_hold
    assert out.state is recovery.RecoveryState.POLICY_AMP_HOLD_REQUESTED
    holding = replace(
        stable,
        now_s=2.2,
        policy_hold_first_write=True,
    )
    out = machine.step(holding)
    assert out.state is recovery.RecoveryState.POLICY_AMP_HOLDING
    return holding


def drive_to_sonic_stabilizing(machine):
    holding = drive_to_amp_holding(machine)
    out = machine.step(
        replace(holding, now_s=3.7, deploy_writer_ready=True)
    )
    assert out.state is recovery.RecoveryState.POLICY_STOP_REQUESTED
    quiet = replace(
        holding,
        now_s=3.8,
        deploy_writer_ready=True,
        policy_alive=False,
        policy_ready=False,
        policy_first_write=False,
        policy_hold_first_write=False,
        lowcmd_fresh=False,
        lowcmd_age_s=0.11,
    )
    out = machine.step(quiet)
    assert out.state is recovery.RecoveryState.POLICY_QUIET
    out = machine.step(replace(quiet, now_s=3.9, lowcmd_age_s=0.2))
    assert out.state is recovery.RecoveryState.SONIC_RESTARTING
    assert out.authorize_sonic_writer
    first_write = replace(
        quiet,
        now_s=4.0,
        deploy_writer_created=True,
        deploy_first_write=True,
        lowcmd_fresh=True,
        lowcmd_age_s=0.01,
    )
    out = machine.step(first_write)
    assert out.state is recovery.RecoveryState.SONIC_STABILIZING
    return first_write


def resident_sample(now: float, **changes):
    base = recovery.ResidentRecoveryInput(
        now_s=now,
        fall_detected=False,
        root_z_m=0.70,
        root_up_z=0.97,
        root_linear_speed_m_s=0.05,
        root_angular_speed_rad_s=0.10,
        joint_velocity_rms_rad_s=0.20,
        lowcmd_fresh=True,
        lowcmd_age_s=0.01,
        sonic_alive=True,
        sonic_generation=11,
        sonic_resident_ready=True,
        sonic_writer_active=True,
        sonic_writer_paused=False,
        sonic_resume_first_write=True,
        policy_alive=True,
        policy_resident_ready=True,
        policy_writer_active=False,
        policy_writer_paused=True,
        policy_first_write=False,
        reset_count=0,
        foot_contact=True,
        grounded_contact=True,
        neutral_confirmed=False,
    )
    return replace(base, **changes)


class RecoveryFsmTests(unittest.TestCase):
    def test_ready_worker_cannot_go_before_takeover_settle_gate(self):
        machine = recovery.SingleWriterRecoveryFSM()
        machine.step(sample(0.0))
        machine.step(sample(0.1, fall_detected=True))
        machine.step(
            sample(0.2, deploy_alive=False, lowcmd_fresh=False, lowcmd_age_s=0.11)
        )
        machine.step(
            sample(0.3, deploy_alive=False, lowcmd_fresh=False, lowcmd_age_s=0.2)
        )
        ready = sample(
            0.31,
            deploy_alive=False,
            policy_alive=True,
            policy_ready=True,
            lowcmd_fresh=False,
            lowcmd_age_s=0.21,
        )
        early = machine.step(ready)
        self.assertEqual(early.state, recovery.RecoveryState.POLICY_STARTING)
        self.assertFalse(early.authorize_policy_writer)
        moving = machine.step(
            replace(
                ready,
                now_s=0.46,
                root_angular_speed_rad_s=1.5,
                joint_velocity_rms_rad_s=1.2,
            )
        )
        self.assertEqual(moving.state, recovery.RecoveryState.POLICY_STARTING)
        self.assertFalse(moving.authorize_policy_writer)
        airborne = machine.step(
            replace(ready, now_s=0.56, grounded_contact=False)
        )
        self.assertEqual(airborne.state, recovery.RecoveryState.POLICY_STARTING)
        self.assertFalse(airborne.authorize_policy_writer)
        allowed = machine.step(replace(ready, now_s=0.66))
        self.assertEqual(allowed.state, recovery.RecoveryState.POLICY_RECOVERING)
        self.assertTrue(allowed.authorize_policy_writer)

    def test_normal_physical_recovery_chain(self):
        machine = recovery.SingleWriterRecoveryFSM()
        holding = drive_to_amp_holding(machine)
        out = machine.step(
            replace(holding, now_s=3.7, deploy_writer_ready=True)
        )
        self.assertEqual(out.state, recovery.RecoveryState.POLICY_STOP_REQUESTED)
        self.assertTrue(out.request_policy_stop)

        out = machine.step(
            replace(
                holding,
                now_s=3.8,
                deploy_writer_ready=True,
                policy_alive=False,
                policy_ready=False,
                policy_first_write=False,
                policy_hold_first_write=False,
                lowcmd_fresh=False,
                lowcmd_age_s=0.11,
            )
        )
        self.assertEqual(out.state, recovery.RecoveryState.POLICY_QUIET)
        out = machine.step(
            stable_policy_sample(
                3.9,
                deploy_writer_ready=True,
                policy_alive=False,
                policy_ready=False,
                policy_first_write=False,
                policy_hold_first_write=False,
                lowcmd_fresh=False,
                lowcmd_age_s=0.2,
            )
        )
        self.assertTrue(out.authorize_sonic_writer)

    def test_stable_getup_can_handoff_directly_to_writer_free_sonic(self):
        machine = recovery.SingleWriterRecoveryFSM(
            recovery.RecoveryConfig(use_amp_hold=False)
        )
        drive_to_policy_recovering(machine)
        first = stable_policy_sample(0.5, deploy_alive=False)
        self.assertTrue(machine.step(first).start_sonic)
        braking_candidate = stable_policy_sample(
            0.6,
            root_z_m=0.65,
            root_up_z=0.97,
            root_linear_speed_m_s=0.4,
            root_angular_speed_rad_s=0.8,
            joint_velocity_rms_rad_s=0.8,
        )
        self.assertFalse(machine._is_stable(braking_candidate))
        self.assertTrue(machine._is_policy_handoff_ready(braking_candidate))
        self.assertFalse(
            machine._is_policy_handoff_ready(
                replace(braking_candidate, root_z_m=0.59)
            )
        )
        self.assertFalse(
            machine._is_policy_handoff_ready(
                replace(braking_candidate, root_angular_speed_rad_s=1.01)
            )
        )
        # Direct SONIC handoff does not use the looser AMP capture gate.  HoST
        # remains the dynamic balance controller until strict stability holds.
        self.assertEqual(
            machine.step(braking_candidate).state,
            recovery.RecoveryState.POLICY_RECOVERING,
        )

        stable = stable_policy_sample(0.7)
        self.assertTrue(machine._is_stable(stable))
        self.assertEqual(
            machine.step(stable).state,
            recovery.RecoveryState.POLICY_GETUP_STABLE,
        )

        # Direct handoff requires the strict 1.5-second stability window and
        # keeps learned balance active throughout replacement SONIC prewarm.
        waiting = machine.step(replace(stable, now_s=0.79))
        self.assertEqual(waiting.state, recovery.RecoveryState.POLICY_GETUP_STABLE)
        self.assertFalse(waiting.request_policy_stop)
        not_ready = machine.step(replace(stable, now_s=2.21))
        self.assertEqual(
            not_ready.state,
            recovery.RecoveryState.POLICY_GETUP_STABLE,
        )
        self.assertFalse(not_ready.request_policy_stop)
        ready_sample = replace(
            stable,
            now_s=2.22,
            deploy_writer_ready=True,
        )
        ready = machine.step(ready_sample)
        self.assertEqual(
            ready.state, recovery.RecoveryState.POLICY_STOP_REQUESTED
        )
        self.assertTrue(ready.request_policy_stop)
        self.assertFalse(ready.request_policy_hold)

        quieting = machine.step(
            replace(
                ready_sample,
                now_s=2.3,
                deploy_writer_ready=True,
                policy_alive=False,
                policy_ready=False,
                policy_first_write=False,
                policy_hold_first_write=False,
                lowcmd_fresh=False,
                lowcmd_age_s=0.11,
            )
        )
        self.assertEqual(quieting.state, recovery.RecoveryState.POLICY_QUIET)
        authorized = machine.step(
            replace(
                ready_sample,
                now_s=2.4,
                deploy_writer_ready=True,
                policy_alive=False,
                policy_ready=False,
                policy_first_write=False,
                policy_hold_first_write=False,
                lowcmd_fresh=False,
                lowcmd_age_s=0.2,
            )
        )
        self.assertEqual(authorized.state, recovery.RecoveryState.SONIC_RESTARTING)
        self.assertTrue(authorized.authorize_sonic_writer)

        out = machine.step(
            stable_policy_sample(
                4.0,
                deploy_writer_ready=True,
                deploy_writer_created=True,
                deploy_first_write=True,
                policy_alive=False,
                policy_ready=False,
                policy_first_write=False,
                policy_hold_first_write=False,
            )
        )
        self.assertEqual(out.state, recovery.RecoveryState.SONIC_STABILIZING)
        replacement_stable = stable_policy_sample(
            4.1,
            deploy_writer_ready=True,
            deploy_writer_created=True,
            deploy_first_write=True,
            deploy_policy_full_control=True,
            policy_alive=False,
            policy_ready=False,
            policy_first_write=False,
            policy_hold_first_write=False,
        )
        out = machine.step(replacement_stable)
        self.assertEqual(out.state, recovery.RecoveryState.SONIC_STABILIZING)
        out = machine.step(replace(replacement_stable, now_s=5.6))
        self.assertEqual(out.state, recovery.RecoveryState.WAIT_NEUTRAL)
        self.assertTrue(out.inhibit_game_input)
        out = machine.step(
            replace(replacement_stable, now_s=5.7, neutral_confirmed=True)
        )
        self.assertEqual(out.state, recovery.RecoveryState.GAME_SONIC)
        self.assertTrue(out.resume_game)
        self.assertTrue(out.start_policy_process)
        self.assertFalse(out.inhibit_game_input)
        with self.assertRaises(FrozenInstanceError):
            out.start_sonic = True

    def test_safe_idle_hold_waits_for_full_control_before_resuming_game(self):
        machine = recovery.SingleWriterRecoveryFSM()
        first_write = drive_to_sonic_stabilizing(machine)

        waiting = machine.step(replace(first_write, now_s=4.1))
        self.assertEqual(
            waiting.state, recovery.RecoveryState.SONIC_STABILIZING
        )
        self.assertFalse(machine._sonic_policy_full_control_seen)
        self.assertFalse(machine._sonic_safe_idle_hold_seen)

        safe_hold = replace(
            first_write,
            now_s=4.2,
            deploy_safe_idle_hold=True,
        )
        out = machine.step(safe_hold)
        self.assertEqual(out.state, recovery.RecoveryState.SONIC_STABILIZING)
        self.assertTrue(machine._sonic_safe_idle_hold_seen)
        self.assertFalse(machine._sonic_policy_full_control_seen)

        out = machine.step(replace(safe_hold, now_s=5.7))
        self.assertEqual(out.state, recovery.RecoveryState.WAIT_NEUTRAL)
        self.assertTrue(out.inhibit_game_input)
        out = machine.step(replace(safe_hold, now_s=5.8))
        self.assertEqual(out.state, recovery.RecoveryState.WAIT_NEUTRAL)
        out = machine.step(
            replace(safe_hold, now_s=5.9, neutral_confirmed=True)
        )
        self.assertEqual(out.state, recovery.RecoveryState.WAIT_NEUTRAL)
        self.assertFalse(out.resume_game)
        self.assertTrue(out.inhibit_game_input)
        out = machine.step(
            replace(
                safe_hold,
                now_s=5.92,
                deploy_policy_full_control=True,
                neutral_confirmed=True,
            )
        )
        self.assertEqual(out.state, recovery.RecoveryState.GAME_SONIC)
        self.assertTrue(out.resume_game)
        self.assertTrue(out.start_policy_process)
        self.assertFalse(out.inhibit_game_input)

    def test_safe_idle_hold_loss_in_wait_neutral_fails_closed(self):
        machine = recovery.SingleWriterRecoveryFSM()
        first_write = drive_to_sonic_stabilizing(machine)
        safe_hold = replace(
            first_write,
            now_s=4.1,
            deploy_safe_idle_hold=True,
        )
        machine.step(safe_hold)
        out = machine.step(replace(safe_hold, now_s=5.6))
        self.assertEqual(out.state, recovery.RecoveryState.WAIT_NEUTRAL)

        out = machine.step(
            replace(safe_hold, now_s=5.7, deploy_safe_idle_hold=False)
        )
        self.assertEqual(out.state, recovery.RecoveryState.FAILED)
        self.assertEqual(
            out.failure_reason,
            "sonic_policy_control_latch_lost_while_waiting_for_neutral",
        )

    def test_safe_idle_hold_validation_is_strict_and_requires_first_write(self):
        machine = recovery.SingleWriterRecoveryFSM()
        out = machine.step(sample(0.0, deploy_safe_idle_hold=1))
        self.assertEqual(out.state, recovery.RecoveryState.FAILED)
        self.assertEqual(out.failure_reason, "invalid_deploy_safe_idle_hold")

        machine = recovery.SingleWriterRecoveryFSM()
        out = machine.step(sample(0.0, deploy_safe_idle_hold=True))
        self.assertEqual(out.state, recovery.RecoveryState.FAILED)
        self.assertEqual(
            out.failure_reason,
            "sonic_safe_idle_hold_without_first_write",
        )

    def test_two_writer_capable_processes_fail_closed(self):
        machine = recovery.SingleWriterRecoveryFSM()
        drive_to_policy_recovering(machine)
        out = machine.step(
            sample(
                0.5,
                deploy_alive=True,
                policy_alive=True,
                policy_ready=True,
                policy_first_write=True,
            )
        )
        self.assertEqual(out.state, recovery.RecoveryState.FAILED)
        self.assertEqual(out.failure_reason, "multiple_writer_capable_processes_alive")
        self.assertTrue(out.request_sonic_stop)
        self.assertTrue(out.request_policy_stop)
        self.assertTrue(out.fail_closed)

    def test_policy_cannot_start_without_sonic_stale_edge(self):
        config = recovery.RecoveryConfig(quiet_timeout_s=0.2)
        machine = recovery.SingleWriterRecoveryFSM(config)
        machine.step(sample(0.0))
        machine.step(sample(0.1, fall_detected=True))
        # No fresh->stale edge: the last fresh packet remains fresh after death.
        out = machine.step(sample(0.2, deploy_alive=False))
        self.assertEqual(out.state, recovery.RecoveryState.SONIC_QUIET)
        out = machine.step(sample(0.41, deploy_alive=False))
        self.assertEqual(out.state, recovery.RecoveryState.FAILED)
        self.assertEqual(out.failure_reason, "timeout_sonic_quiet")
        self.assertFalse(out.start_policy_process)

    def test_sonic_stale_before_process_death_is_not_lost(self):
        machine = recovery.SingleWriterRecoveryFSM()
        machine.step(sample(0.0))
        machine.step(sample(0.1, fall_detected=True))
        # DDS ages stale while the deploy is still completing its native stop.
        out = machine.step(
            sample(
                0.2,
                fall_detected=True,
                lowcmd_fresh=False,
                lowcmd_age_s=0.11,
            )
        )
        self.assertEqual(out.state, recovery.RecoveryState.SONIC_STOP_REQUESTED)
        # Reaping on the next sample must retain fresh-seen/current-stale.
        out = machine.step(
            sample(
                0.3,
                deploy_alive=False,
                lowcmd_fresh=False,
                lowcmd_age_s=0.21,
            )
        )
        self.assertEqual(out.state, recovery.RecoveryState.SONIC_QUIET)
        out = machine.step(
            sample(
                0.4,
                deploy_alive=False,
                lowcmd_fresh=False,
                lowcmd_age_s=0.31,
            )
        )
        self.assertFalse(out.start_policy_process)

    def test_writer_free_hot_standby_waits_for_sonic_stale_before_go(self):
        machine = recovery.SingleWriterRecoveryFSM()
        standby = sample(
            0.0,
            policy_alive=True,
            policy_ready=True,
            policy_first_write=False,
        )
        out = machine.step(standby)
        self.assertEqual(out.state, recovery.RecoveryState.GAME_SONIC)
        self.assertTrue(out.start_policy_process)
        self.assertFalse(out.authorize_policy_writer)

        out = machine.step(replace(standby, now_s=0.1, fall_detected=True))
        self.assertEqual(out.state, recovery.RecoveryState.SONIC_STOP_REQUESTED)
        self.assertTrue(out.request_sonic_stop)
        self.assertFalse(out.authorize_policy_writer)

        out = machine.step(
            replace(
                standby,
                now_s=0.2,
                fall_detected=True,
                deploy_alive=False,
                lowcmd_fresh=False,
                lowcmd_age_s=0.11,
            )
        )
        self.assertEqual(out.state, recovery.RecoveryState.SONIC_QUIET)
        self.assertFalse(out.authorize_policy_writer)
        out = machine.step(
            replace(
                standby,
                now_s=0.3,
                deploy_alive=False,
                lowcmd_fresh=False,
                lowcmd_age_s=0.2,
            )
        )
        self.assertEqual(out.state, recovery.RecoveryState.POLICY_STARTING)
        self.assertFalse(out.start_policy_process)
        self.assertFalse(out.authorize_policy_writer)
        out = machine.step(
            replace(
                standby,
                now_s=0.46,
                deploy_alive=False,
                lowcmd_fresh=False,
                lowcmd_age_s=0.36,
            )
        )
        self.assertEqual(out.state, recovery.RecoveryState.POLICY_RECOVERING)
        self.assertTrue(out.authorize_policy_writer)

    def test_hard_revoked_old_sonic_may_finish_cleanup_during_host_takeover(self):
        machine = recovery.SingleWriterRecoveryFSM()
        self.assertTrue(machine.step(sample(0.0)).start_policy_process)
        standby = sample(
            0.1,
            fall_detected=True,
            policy_alive=True,
            policy_ready=True,
        )
        self.assertTrue(machine.step(standby).request_sonic_stop)
        revoked = replace(
            standby,
            now_s=0.2,
            deploy_writer_revoked=True,
            lowcmd_fresh=False,
            lowcmd_age_s=0.11,
        )
        out = machine.step(revoked)
        self.assertEqual(out.state, recovery.RecoveryState.SONIC_QUIET)
        out = machine.step(replace(revoked, now_s=0.3, lowcmd_age_s=0.2))
        self.assertEqual(out.state, recovery.RecoveryState.POLICY_STARTING)
        out = machine.step(replace(revoked, now_s=0.46, lowcmd_age_s=0.36))
        self.assertEqual(out.state, recovery.RecoveryState.POLICY_RECOVERING)
        self.assertTrue(out.authorize_policy_writer)

        host_first_write = replace(
            revoked,
            now_s=0.5,
            policy_first_write=True,
            lowcmd_fresh=True,
            lowcmd_age_s=0.01,
        )
        out = machine.step(host_first_write)
        self.assertEqual(out.state, recovery.RecoveryState.POLICY_RECOVERING)
        self.assertFalse(out.start_sonic)
        out = machine.step(
            replace(
                host_first_write,
                now_s=0.6,
                deploy_alive=False,
                deploy_writer_created=False,
                deploy_writer_revoked=True,
            )
        )
        self.assertTrue(out.start_sonic)

    def test_policy_stale_before_process_death_is_not_lost(self):
        machine = recovery.SingleWriterRecoveryFSM()
        holding = drive_to_amp_holding(machine)
        out = machine.step(
            replace(holding, now_s=3.7, deploy_writer_ready=True)
        )
        self.assertTrue(out.request_policy_stop)
        # LowCmd ages stale before the worker process reports dead.
        out = machine.step(
            replace(
                holding,
                now_s=3.8,
                deploy_writer_ready=True,
                lowcmd_fresh=False,
                lowcmd_age_s=0.11,
            )
        )
        self.assertEqual(out.state, recovery.RecoveryState.POLICY_STOP_REQUESTED)
        out = machine.step(
            replace(
                holding,
                now_s=3.9,
                deploy_writer_ready=True,
                policy_alive=False,
                policy_ready=False,
                policy_first_write=False,
                policy_hold_first_write=False,
                lowcmd_fresh=False,
                lowcmd_age_s=0.21,
            )
        )
        self.assertEqual(out.state, recovery.RecoveryState.POLICY_QUIET)
        out = machine.step(
            stable_policy_sample(
                4.0,
                deploy_writer_ready=True,
                policy_alive=False,
                policy_ready=False,
                policy_first_write=False,
                policy_hold_first_write=False,
                lowcmd_fresh=False,
                lowcmd_age_s=0.31,
            )
        )
        self.assertTrue(out.authorize_sonic_writer)

    def test_stage_timeout_fails_closed(self):
        config = recovery.RecoveryConfig(sonic_stop_timeout_s=0.2)
        machine = recovery.SingleWriterRecoveryFSM(config)
        machine.step(sample(0.0))
        machine.step(sample(0.1, fall_detected=True))
        out = machine.step(sample(0.31, fall_detected=True))
        self.assertEqual(out.state, recovery.RecoveryState.FAILED)
        self.assertEqual(out.failure_reason, "timeout_sonic_stop_requested")
        self.assertTrue(out.request_sonic_stop)

    def test_stability_hold_restarts_after_any_interruption(self):
        machine = recovery.SingleWriterRecoveryFSM()
        drive_to_policy_recovering(machine)
        first = stable_policy_sample(0.5, deploy_alive=False)
        self.assertTrue(machine.step(first).start_sonic)
        stable = stable_policy_sample(0.6)
        machine.step(stable)
        out = machine.step(replace(stable, now_s=1.5, foot_contact=False))
        self.assertEqual(out.state, recovery.RecoveryState.POLICY_RECOVERING)
        out = machine.step(replace(stable, now_s=1.6))
        self.assertEqual(out.state, recovery.RecoveryState.POLICY_GETUP_STABLE)
        out = machine.step(replace(stable, now_s=1.8))
        self.assertEqual(out.state, recovery.RecoveryState.POLICY_GETUP_STABLE)
        self.assertFalse(out.request_policy_hold)
        out = machine.step(replace(stable, now_s=1.91))
        self.assertTrue(out.request_policy_hold)

    def test_policy_timeout_is_absolute_from_go_across_stability_oscillation(self):
        machine = recovery.SingleWriterRecoveryFSM(
            recovery.RecoveryConfig(policy_recovery_timeout_s=1.0)
        )
        drive_to_policy_recovering(machine)
        first = stable_policy_sample(0.5, deploy_alive=False)
        self.assertTrue(machine.step(first).start_sonic)
        stable = stable_policy_sample(0.6)
        self.assertEqual(
            machine.step(stable).state,
            recovery.RecoveryState.POLICY_GETUP_STABLE,
        )
        self.assertEqual(
            machine.step(
                replace(stable, now_s=0.9, root_linear_speed_m_s=0.9)
            ).state,
            recovery.RecoveryState.POLICY_RECOVERING,
        )
        self.assertEqual(
            machine.step(replace(stable, now_s=1.2)).state,
            recovery.RecoveryState.POLICY_GETUP_STABLE,
        )
        out = machine.step(
            replace(stable, now_s=1.47, root_linear_speed_m_s=0.9)
        )
        self.assertEqual(out.state, recovery.RecoveryState.FAILED)
        self.assertEqual(out.failure_reason, "timeout_policy_recovery")
        self.assertTrue(out.request_policy_stop)

    def test_first_write_event_is_latched(self):
        machine = recovery.SingleWriterRecoveryFSM()
        drive_to_policy_recovering(machine)
        first_write = stable_policy_sample(0.5, deploy_alive=False)
        self.assertTrue(machine.step(first_write).start_sonic)
        # FIRST_WRITE is an event in the worker protocol, so it need not remain
        # asserted on every subsequent observation.
        stable = stable_policy_sample(0.6, policy_first_write=False)
        out = machine.step(stable)
        self.assertEqual(out.state, recovery.RecoveryState.POLICY_GETUP_STABLE)
        out = machine.step(replace(stable, now_s=2.1))
        self.assertTrue(out.request_policy_hold)

    def test_restart_requires_new_generation_stale_then_fresh(self):
        machine = recovery.SingleWriterRecoveryFSM()
        drive_to_policy_recovering(machine)
        first = stable_policy_sample(0.5, deploy_alive=False)
        self.assertTrue(machine.step(first).start_sonic)
        # Reusing generation 7 is never accepted even while writer-free.
        out = machine.step(
            stable_policy_sample(0.6, deploy_generation=7)
        )
        self.assertEqual(out.state, recovery.RecoveryState.FAILED)
        self.assertEqual(
            out.failure_reason, "sonic_prewarm_did_not_advance_generation"
        )

        # A separate run proves a new generation may not write before the AMP
        # owner has stopped and LowCmd has crossed fresh -> stale.
        machine = recovery.SingleWriterRecoveryFSM()
        drive_to_policy_recovering(machine)
        self.assertTrue(machine.step(first).start_sonic)
        out = machine.step(
            stable_policy_sample(
                0.6,
                deploy_writer_ready=True,
                deploy_writer_created=True,
                deploy_first_write=True,
            )
        )
        self.assertEqual(out.state, recovery.RecoveryState.FAILED)
        self.assertEqual(out.failure_reason, "multiple_writer_capable_processes_alive")

    def test_replacement_sonic_allows_measured_warm_start_latency(self):
        config = recovery.RecoveryConfig()
        self.assertGreaterEqual(config.sonic_prewarm_timeout_s, 35.85619)
        self.assertLessEqual(config.sonic_prewarm_timeout_s, 45.0)
        self.assertGreaterEqual(config.policy_recovery_timeout_s, 58.2)
        self.assertGreaterEqual(config.episode_timeout_s, 60.0)

    def test_amp_hold_transition_requires_first_write_ack(self):
        machine = recovery.SingleWriterRecoveryFSM(
            recovery.RecoveryConfig(policy_hold_start_timeout_s=0.5)
        )
        drive_to_policy_recovering(machine)
        first = stable_policy_sample(0.5, deploy_alive=False)
        self.assertTrue(machine.step(first).start_sonic)
        stable = stable_policy_sample(0.6)
        machine.step(stable)
        requested = machine.step(replace(stable, now_s=2.1))
        self.assertTrue(requested.request_policy_hold)
        waiting = machine.step(replace(stable, now_s=2.2))
        self.assertEqual(
            waiting.state, recovery.RecoveryState.POLICY_AMP_HOLD_REQUESTED
        )
        timed_out = machine.step(replace(stable, now_s=2.61))
        self.assertEqual(timed_out.state, recovery.RecoveryState.FAILED)
        self.assertEqual(
            timed_out.failure_reason, "timeout_policy_amp_hold_requested"
        )

    def test_writer_free_sonic_prewarm_timeout_fails_closed(self):
        machine = recovery.SingleWriterRecoveryFSM(
            recovery.RecoveryConfig(
                policy_recovery_timeout_s=60.0,
                sonic_prewarm_timeout_s=1.0,
            )
        )
        drive_to_policy_recovering(machine)
        first = stable_policy_sample(0.5, deploy_alive=False)
        self.assertTrue(machine.step(first).start_sonic)
        out = machine.step(stable_policy_sample(1.51))
        self.assertEqual(out.state, recovery.RecoveryState.FAILED)
        self.assertEqual(out.failure_reason, "timeout_sonic_prewarm")
        self.assertTrue(out.request_sonic_stop)
        self.assertTrue(out.request_policy_stop)

    def test_process_ready_does_not_require_shadow_ready_while_getup_is_slow(self):
        machine = recovery.SingleWriterRecoveryFSM(
            recovery.RecoveryConfig(
                policy_recovery_timeout_s=60.0,
                sonic_prewarm_timeout_s=1.0,
            )
        )
        drive_to_policy_recovering(machine)
        first = stable_policy_sample(0.5, deploy_alive=False)
        self.assertTrue(machine.step(first).start_sonic)
        out = machine.step(
            stable_policy_sample(
                1.51,
                deploy_process_ready=True,
                deploy_writer_ready=False,
            )
        )
        self.assertNotEqual(out.state, recovery.RecoveryState.FAILED)
        self.assertFalse(out.request_policy_stop)

    def test_replacement_sonic_cannot_resume_while_robot_has_fallen_again(self):
        machine = recovery.SingleWriterRecoveryFSM(
            recovery.RecoveryConfig(sonic_stabilize_timeout_s=1.0)
        )
        holding = drive_to_amp_holding(machine)
        machine.step(
            replace(
                holding,
                now_s=3.7,
                deploy_writer_ready=True,
            )
        )
        machine.step(
            replace(
                holding,
                now_s=3.8,
                deploy_writer_ready=True,
                policy_alive=False,
                policy_ready=False,
                policy_first_write=False,
                policy_hold_first_write=False,
                lowcmd_fresh=False,
                lowcmd_age_s=0.11,
            )
        )
        machine.step(
            stable_policy_sample(
                3.9,
                deploy_writer_ready=True,
                policy_alive=False,
                policy_ready=False,
                policy_first_write=False,
                policy_hold_first_write=False,
                lowcmd_fresh=False,
                lowcmd_age_s=0.2,
            )
        )
        fallen_sonic = stable_policy_sample(
            4.0,
            deploy_writer_ready=True,
            deploy_writer_created=True,
            deploy_first_write=True,
            policy_alive=False,
            policy_ready=False,
            policy_first_write=False,
            policy_hold_first_write=False,
            root_z_m=0.10,
            root_up_z=0.10,
        )
        out = machine.step(fallen_sonic)
        self.assertEqual(out.state, recovery.RecoveryState.SONIC_STABILIZING)
        out = machine.step(replace(fallen_sonic, now_s=4.1))
        self.assertEqual(out.state, recovery.RecoveryState.SONIC_STABILIZING)
        out = machine.step(replace(fallen_sonic, now_s=5.01))
        self.assertEqual(out.state, recovery.RecoveryState.FAILED)
        self.assertEqual(out.failure_reason, "timeout_sonic_stabilizing")

    def test_replacement_sonic_full_control_has_a_short_independent_deadline(self):
        machine = recovery.SingleWriterRecoveryFSM(
            recovery.RecoveryConfig(
                sonic_full_control_timeout_s=0.3,
                sonic_stabilize_timeout_s=10.0,
            )
        )
        holding = drive_to_amp_holding(machine)
        machine.step(replace(holding, now_s=3.7, deploy_writer_ready=True))
        quiet = replace(
            holding,
            now_s=3.8,
            deploy_writer_ready=True,
            policy_alive=False,
            policy_ready=False,
            policy_first_write=False,
            policy_hold_first_write=False,
            lowcmd_fresh=False,
            lowcmd_age_s=0.11,
        )
        machine.step(quiet)
        machine.step(replace(quiet, now_s=3.9, lowcmd_age_s=0.2))
        first_write = replace(
            quiet,
            now_s=4.0,
            deploy_writer_created=True,
            deploy_first_write=True,
            lowcmd_fresh=True,
            lowcmd_age_s=0.01,
        )
        out = machine.step(first_write)
        self.assertEqual(out.state, recovery.RecoveryState.SONIC_STABILIZING)
        out = machine.step(replace(first_write, now_s=4.31))
        self.assertEqual(out.state, recovery.RecoveryState.FAILED)
        self.assertEqual(out.failure_reason, "timeout_sonic_full_control")

    def test_safe_idle_hold_satisfies_short_reentry_deadline(self):
        machine = recovery.SingleWriterRecoveryFSM(
            recovery.RecoveryConfig(
                sonic_full_control_timeout_s=0.3,
                sonic_stabilize_timeout_s=10.0,
            )
        )
        first_write = drive_to_sonic_stabilizing(machine)
        out = machine.step(
            replace(
                first_write,
                now_s=4.31,
                deploy_safe_idle_hold=True,
            )
        )
        self.assertEqual(out.state, recovery.RecoveryState.SONIC_STABILIZING)
        self.assertFalse(out.fail_closed)
        self.assertTrue(machine._sonic_safe_idle_hold_seen)
        self.assertFalse(machine._sonic_policy_full_control_seen)

    def test_wait_neutral_requires_full_control_within_reentry_deadline(self):
        machine = recovery.SingleWriterRecoveryFSM(
            recovery.RecoveryConfig(
                sonic_full_control_timeout_s=0.3,
                sonic_stabilize_timeout_s=10.0,
            )
        )
        first_write = drive_to_sonic_stabilizing(machine)
        safe_hold = replace(
            first_write,
            now_s=4.1,
            deploy_safe_idle_hold=True,
        )
        machine.step(safe_hold)
        waiting = machine.step(replace(safe_hold, now_s=5.6))
        self.assertEqual(waiting.state, recovery.RecoveryState.WAIT_NEUTRAL)
        self.assertTrue(waiting.inhibit_game_input)

        still_waiting = machine.step(
            replace(safe_hold, now_s=5.8, neutral_confirmed=True)
        )
        self.assertEqual(
            still_waiting.state, recovery.RecoveryState.WAIT_NEUTRAL
        )
        self.assertTrue(still_waiting.inhibit_game_input)

        timed_out = machine.step(
            replace(safe_hold, now_s=5.91, neutral_confirmed=True)
        )
        self.assertEqual(timed_out.state, recovery.RecoveryState.FAILED)
        self.assertEqual(timed_out.failure_reason, "timeout_sonic_full_control")

    def test_reset_counter_change_is_forbidden(self):
        machine = recovery.SingleWriterRecoveryFSM()
        machine.step(sample(0.0))
        out = machine.step(sample(0.1, reset_count=1))
        self.assertEqual(out.state, recovery.RecoveryState.FAILED)
        self.assertEqual(out.failure_reason, "reset_count_changed")

    def test_dead_policy_may_leave_sticky_protocol_event_without_being_a_writer(self):
        machine = recovery.SingleWriterRecoveryFSM()
        machine.step(sample(0.0))
        out = machine.step(
            sample(0.1, policy_alive=False, policy_first_write=True)
        )
        self.assertEqual(out.state, recovery.RecoveryState.GAME_SONIC)
        self.assertFalse(out.fail_closed)


class ResidentRecoveryFsmTests(unittest.TestCase):
    def test_recovery_policy_slot_changes_only_while_sonic_owns_control(self):
        machine = recovery.ResidentPolicyRecoveryFSM(
            recovery_policy_id="kungfu"
        )

        self.assertEqual(machine.select_recovery_policy("host"), "kungfu")
        self.assertEqual(machine.recovery_policy_id, "host")
        machine.step(resident_sample(0.0, fall_detected=True))
        with self.assertRaisesRegex(RuntimeError, "GAME_SONIC"):
            machine.select_recovery_policy("amp")

    def test_sonic_kungfu_same_sonic_authority_cycle(self):
        machine = recovery.ResidentPolicyRecoveryFSM()
        outputs = []

        outputs.append(machine.step(resident_sample(0.0)))
        self.assertTrue(outputs[-1].start_policy_process)
        self.assertEqual(
            outputs[-1].state, recovery.ResidentRecoveryState.GAME_SONIC
        )

        outputs.append(machine.step(resident_sample(0.1, fall_detected=True)))
        self.assertTrue(outputs[-1].request_sonic_pause)
        self.assertEqual(
            outputs[-1].state,
            recovery.ResidentRecoveryState.SONIC_PAUSE_REQUESTED,
        )

        paused_sonic = resident_sample(
            0.2,
            fall_detected=True,
            sonic_writer_active=False,
            sonic_writer_paused=True,
            sonic_resume_first_write=False,
        )
        outputs.append(machine.step(paused_sonic))
        self.assertEqual(
            outputs[-1].state, recovery.ResidentRecoveryState.SONIC_QUIET
        )
        outputs.append(
            machine.step(
                replace(
                    paused_sonic,
                    now_s=0.31,
                    lowcmd_fresh=False,
                    lowcmd_age_s=0.11,
                )
            )
        )
        self.assertEqual(
            outputs[-1].state, recovery.ResidentRecoveryState.KUNGFU_STARTING
        )

        quiet = replace(
            paused_sonic,
            now_s=0.5,
            lowcmd_fresh=False,
            lowcmd_age_s=0.30,
        )
        outputs.append(machine.step(quiet))
        self.assertTrue(outputs[-1].authorize_policy_writer)

        kungfu_active = replace(
            quiet,
            now_s=0.52,
            fall_detected=False,
            lowcmd_fresh=True,
            lowcmd_age_s=0.01,
            policy_writer_active=True,
            policy_writer_paused=False,
            policy_first_write=True,
        )
        outputs.append(machine.step(kungfu_active))
        self.assertEqual(
            outputs[-1].state, recovery.ResidentRecoveryState.KUNGFU_RECOVERING
        )
        outputs.append(machine.step(replace(kungfu_active, now_s=0.6)))
        self.assertTrue(outputs[-1].request_policy_pause)
        self.assertEqual(
            outputs[-1].state,
            recovery.ResidentRecoveryState.POLICY_PAUSE_REQUESTED,
        )
        self.assertEqual(outputs[-1].authority_policy_id, "kungfu")
        self.assertEqual(outputs[-1].recovery_policy_id, "kungfu")

        paused_kungfu = replace(
            kungfu_active,
            now_s=2.2,
            policy_writer_active=False,
            policy_writer_paused=True,
        )
        outputs.append(machine.step(paused_kungfu))
        self.assertEqual(
            outputs[-1].state, recovery.ResidentRecoveryState.KUNGFU_QUIET
        )
        outputs.append(
            machine.step(
                replace(
                    paused_kungfu,
                    now_s=2.31,
                    lowcmd_fresh=False,
                    lowcmd_age_s=0.11,
                )
            )
        )
        self.assertTrue(outputs[-1].resume_sonic_writer)
        self.assertEqual(
            outputs[-1].state,
            recovery.ResidentRecoveryState.SONIC_RESUME_REQUESTED,
        )

        sonic_resumed = replace(
            paused_kungfu,
            now_s=2.4,
            sonic_writer_active=True,
            sonic_writer_paused=False,
            sonic_resume_first_write=True,
            lowcmd_fresh=True,
            lowcmd_age_s=0.01,
        )
        outputs.append(machine.step(sonic_resumed))
        self.assertEqual(
            outputs[-1].state,
            recovery.ResidentRecoveryState.SONIC_STABILIZING,
        )
        outputs.append(machine.step(replace(sonic_resumed, now_s=2.5)))
        outputs.append(machine.step(replace(sonic_resumed, now_s=4.01)))
        self.assertEqual(
            outputs[-1].state, recovery.ResidentRecoveryState.WAIT_NEUTRAL
        )
        outputs.append(
            machine.step(
                replace(sonic_resumed, now_s=4.1, neutral_confirmed=True)
            )
        )
        self.assertTrue(outputs[-1].resume_game)
        self.assertEqual(
            outputs[-1].state, recovery.ResidentRecoveryState.GAME_SONIC
        )

        self.assertFalse(any(output.start_sonic for output in outputs))
        self.assertFalse(any(output.request_sonic_stop for output in outputs))
        self.assertFalse(any(output.request_policy_stop for output in outputs))
        self.assertFalse(any(output.fail_closed for output in outputs))

    def test_generic_policy_id_can_opt_in_to_terminal_dwell(self):
        machine = recovery.ResidentPolicyRecoveryFSM(
            recovery.RecoveryConfig(policy_exit_hold_s=0.2),
            recovery_policy_id="future_getup",
        )
        machine.step(resident_sample(0.0))
        machine._baseline_reset_count = 0
        machine._episode_started_s = 0.1
        machine._sonic_generation = 11
        machine._state_entered_s = 0.2
        machine.state = recovery.ResidentRecoveryState.POLICY_RECOVERING
        active = resident_sample(
            0.5,
            sonic_writer_active=False,
            sonic_writer_paused=True,
            sonic_resume_first_write=False,
            policy_writer_active=True,
            policy_writer_paused=False,
            policy_first_write=True,
        )

        stable = machine.step(active)
        self.assertEqual(stable.state, recovery.ResidentRecoveryState.POLICY_STABLE)
        self.assertFalse(stable.request_policy_pause)
        self.assertEqual(stable.authority_policy_id, "future_getup")
        self.assertEqual(stable.recovery_policy_id, "future_getup")

        still_dwelling = machine.step(replace(active, now_s=0.69))
        self.assertEqual(
            still_dwelling.state, recovery.ResidentRecoveryState.POLICY_STABLE
        )
        self.assertFalse(still_dwelling.request_policy_pause)

        pause = machine.step(replace(active, now_s=0.71))
        self.assertEqual(
            pause.state, recovery.ResidentRecoveryState.POLICY_PAUSE_REQUESTED
        )
        self.assertTrue(pause.request_policy_pause)

    def test_sonic_refall_immediately_reenters_resident_kungfu_cycle(self):
        machine = recovery.ResidentPolicyRecoveryFSM()
        machine.step(resident_sample(0.0))
        machine._baseline_reset_count = 0
        machine._episode_started_s = 1.0
        machine._sonic_generation = 11
        machine._state_entered_s = 2.0
        machine.state = recovery.ResidentRecoveryState.SONIC_STABILIZING

        out = machine.step(resident_sample(2.5, fall_detected=True))

        self.assertTrue(out.request_sonic_pause)
        self.assertFalse(out.fail_closed)
        self.assertEqual(
            out.previous_state,
            recovery.ResidentRecoveryState.SONIC_STABILIZING,
        )
        self.assertEqual(
            out.state,
            recovery.ResidentRecoveryState.SONIC_PAUSE_REQUESTED,
        )
        self.assertEqual(machine._episode_started_s, 2.5)

    def test_resident_sonic_generation_change_fails_closed(self):
        machine = recovery.ResidentPolicyRecoveryFSM()
        machine.step(resident_sample(0.0))
        machine.step(resident_sample(0.1, fall_detected=True))
        out = machine.step(
            resident_sample(
                0.2,
                fall_detected=True,
                sonic_generation=12,
                sonic_writer_active=False,
                sonic_writer_paused=True,
                sonic_resume_first_write=False,
            )
        )
        self.assertTrue(out.fail_closed)
        self.assertEqual(out.failure_reason, "resident_sonic_generation_changed")

    def test_two_active_resident_writers_fail_closed(self):
        machine = recovery.ResidentPolicyRecoveryFSM()
        out = machine.step(
            resident_sample(0.0, policy_writer_active=True)
        )
        self.assertTrue(out.fail_closed)
        self.assertEqual(out.failure_reason, "multiple_active_policy_writers")


if __name__ == "__main__":
    unittest.main()
