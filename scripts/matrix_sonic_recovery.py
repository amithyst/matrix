#!/usr/bin/env python3
"""Pure control-plane FSM for physical SONIC -> recovery policy -> SONIC.

The machine performs no I/O and has no simulator-state mutation API.  In
particular, it cannot reset, reload, teleport, or write ``qpos``.  Its caller
owns processes and translates the returned one-shot action flags into control
plane operations.

The hand-off is deliberately conservative: the previous writer must cross an
authenticated hard-revocation fence and a fresh-to-stale LowCmd edge before
the next writer may be authorized.  Its process may finish non-writing cleanup
in parallel.  The replacement SONIC deploy must also have a newer generation
and produce a stale-to-fresh edge before game input can be re-armed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional


class RecoveryState(str, Enum):
    GAME_SONIC = "GAME_SONIC"
    # Convenient semantic alias used by callers after WAIT_NEUTRAL.
    GAME = "GAME_SONIC"
    SONIC_STOP_REQUESTED = "SONIC_STOP_REQUESTED"
    SONIC_QUIET = "SONIC_QUIET"
    POLICY_STARTING = "POLICY_STARTING"
    POLICY_RECOVERING = "POLICY_RECOVERING"
    POLICY_GETUP_STABLE = "POLICY_GETUP_STABLE"
    # Compatibility alias for older diagnostics/tests.
    POLICY_STABLE_HOLD = "POLICY_GETUP_STABLE"
    POLICY_AMP_HOLD_REQUESTED = "POLICY_AMP_HOLD_REQUESTED"
    POLICY_AMP_HOLDING = "POLICY_AMP_HOLDING"
    POLICY_STOP_REQUESTED = "POLICY_STOP_REQUESTED"
    POLICY_QUIET = "POLICY_QUIET"
    SONIC_RESTARTING = "SONIC_RESTARTING"
    SONIC_STABILIZING = "SONIC_STABILIZING"
    WAIT_NEUTRAL = "WAIT_NEUTRAL"
    FAILED = "FAILED"


@dataclass(frozen=True)
class RecoveryConfig:
    """Safety thresholds and monotonic-time deadlines, in seconds."""

    stable_hold_s: float = 1.5
    # A full stable predicate is already velocity- and contact-gated. Resident
    # recovery policies normally release authority on that first safe sample;
    # policies that require a terminal dwell may opt in explicitly.
    policy_exit_hold_s: float = 0.0
    takeover_settle_s: float = 0.35
    stable_root_z_m: float = 0.65
    stable_root_up_z: float = 0.85
    stable_root_linear_speed_m_s: float = 0.15
    stable_root_angular_speed_rad_s: float = 0.5
    stable_joint_velocity_rms_rad_s: float = 0.5
    policy_handoff_hold_s: float = 0.2
    # The first gate enters a physical braking hold; the stricter stable gate
    # below still controls whether that writer may stop in favor of SONIC.
    policy_handoff_root_z_m: float = 0.60
    policy_handoff_root_up_z: float = 0.95
    policy_handoff_root_linear_speed_m_s: float = 0.8
    policy_handoff_root_angular_speed_rad_s: float = 1.0
    policy_handoff_joint_velocity_rms_rad_s: float = 1.0
    max_lowcmd_age_s: float = 0.1
    use_amp_hold: bool = True
    takeover_root_linear_speed_m_s: float = 0.25
    takeover_root_angular_speed_rad_s: float = 1.0
    takeover_joint_velocity_rms_rad_s: float = 1.0

    sonic_stop_timeout_s: float = 5.0
    quiet_timeout_s: float = 2.0
    policy_start_timeout_s: float = 10.0
    policy_first_write_timeout_s: float = 2.0
    policy_recovery_timeout_s: float = 90.0
    policy_hold_start_timeout_s: float = 2.0
    policy_stop_timeout_s: float = 5.0
    sonic_prewarm_timeout_s: float = 45.0
    sonic_restart_timeout_s: float = 10.0
    # Once the replacement has a fresh first write, it must finish its bounded
    # controller blend promptly.  This is deliberately separate from the much
    # longer post-control physical stability hold.
    sonic_full_control_timeout_s: float = 10.0
    # Heyuan's pinned native deploy needs about 28.5 s for model construction
    # and about 7 s more for writer-free planner/history/policy admission.
    sonic_stabilize_timeout_s: float = 60.0
    neutral_timeout_s: float = 30.0
    episode_timeout_s: float = 120.0

    def __post_init__(self) -> None:
        if type(self.use_amp_hold) is not bool:
            raise ValueError("use_amp_hold must be a bool")
        positive = (
            "stable_hold_s",
            "takeover_settle_s",
            "stable_root_z_m",
            "stable_root_up_z",
            "stable_root_linear_speed_m_s",
            "stable_root_angular_speed_rad_s",
            "stable_joint_velocity_rms_rad_s",
            "policy_handoff_hold_s",
            "policy_handoff_root_z_m",
            "policy_handoff_root_up_z",
            "policy_handoff_root_linear_speed_m_s",
            "policy_handoff_root_angular_speed_rad_s",
            "policy_handoff_joint_velocity_rms_rad_s",
            "max_lowcmd_age_s",
            "takeover_root_linear_speed_m_s",
            "takeover_root_angular_speed_rad_s",
            "takeover_joint_velocity_rms_rad_s",
            "sonic_stop_timeout_s",
            "quiet_timeout_s",
            "policy_start_timeout_s",
            "policy_first_write_timeout_s",
            "policy_recovery_timeout_s",
            "policy_hold_start_timeout_s",
            "policy_stop_timeout_s",
            "sonic_prewarm_timeout_s",
            "sonic_restart_timeout_s",
            "sonic_full_control_timeout_s",
            "sonic_stabilize_timeout_s",
            "neutral_timeout_s",
            "episode_timeout_s",
        )
        for name in positive:
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if (
            not math.isfinite(float(self.policy_exit_hold_s))
            or self.policy_exit_hold_s < 0.0
        ):
            raise ValueError("policy_exit_hold_s must be finite and non-negative")
        if self.stable_root_up_z > 1.0:
            raise ValueError("stable_root_up_z cannot exceed 1")
        if self.policy_handoff_root_up_z > 1.0:
            raise ValueError("policy_handoff_root_up_z cannot exceed 1")
        if self.takeover_settle_s < 0.35:
            raise ValueError("takeover_settle_s cannot be shorter than 0.35 seconds")


@dataclass(frozen=True)
class RecoveryInput:
    """One immutable observation supplied by the runtime supervision loop."""

    now_s: float
    fall_detected: bool
    root_z_m: float
    root_up_z: float
    root_linear_speed_m_s: float
    root_angular_speed_rad_s: float
    joint_velocity_rms_rad_s: float
    lowcmd_fresh: bool
    lowcmd_age_s: Optional[float]
    deploy_alive: bool
    deploy_generation: int
    deploy_process_ready: bool
    deploy_writer_ready: bool
    deploy_writer_created: bool
    deploy_writer_revoked: bool
    deploy_first_write: bool
    deploy_policy_full_control: bool
    deploy_safe_idle_hold: bool
    policy_alive: bool
    policy_ready: bool
    policy_first_write: bool
    policy_hold_first_write: bool
    reset_count: int
    foot_contact: bool
    grounded_contact: bool
    neutral_confirmed: bool = False


@dataclass(frozen=True)
class RecoveryOutput:
    """One-shot actions plus the resulting state; all fields are immutable."""

    previous_state: RecoveryState
    state: RecoveryState
    request_sonic_stop: bool = False
    start_policy_process: bool = False
    authorize_policy_writer: bool = False
    request_policy_hold: bool = False
    request_policy_stop: bool = False
    start_sonic: bool = False
    authorize_sonic_writer: bool = False
    inhibit_game_input: bool = False
    resume_game: bool = False
    fail_closed: bool = False
    failure_reason: Optional[str] = None


class SingleWriterRecoveryFSM:
    """Fail-closed coordinator for the runtime's managed writer processes."""

    _STAGE_TIMEOUT_ATTR = {
        RecoveryState.SONIC_STOP_REQUESTED: "sonic_stop_timeout_s",
        RecoveryState.SONIC_QUIET: "quiet_timeout_s",
        RecoveryState.POLICY_STARTING: "policy_start_timeout_s",
        RecoveryState.POLICY_AMP_HOLD_REQUESTED: "policy_hold_start_timeout_s",
        RecoveryState.POLICY_STOP_REQUESTED: "policy_stop_timeout_s",
        RecoveryState.POLICY_QUIET: "quiet_timeout_s",
        RecoveryState.SONIC_RESTARTING: "sonic_restart_timeout_s",
        RecoveryState.SONIC_STABILIZING: "sonic_stabilize_timeout_s",
        RecoveryState.WAIT_NEUTRAL: "neutral_timeout_s",
    }

    def __init__(self, config: RecoveryConfig = RecoveryConfig()) -> None:
        self.config = config
        self.state = RecoveryState.GAME_SONIC
        self.failure_reason: Optional[str] = None
        self._state_entered_s: Optional[float] = None
        self._last_now_s: Optional[float] = None
        self._last_lowcmd_fresh = False
        self._baseline_reset_count: Optional[int] = None
        self._episode_started_s: Optional[float] = None
        self._fall_latched = False

        self._old_deploy_generation: Optional[int] = None
        self._new_deploy_generation: Optional[int] = None
        self._sonic_fresh_seen = False
        self._sonic_stale_confirmed = False
        self._policy_seen_alive = False
        self._policy_standby_requested = False
        self._policy_standby_requested_s: Optional[float] = None
        self._policy_standby_seen_alive = False
        self._policy_standby_ready_seen = False
        self._policy_writer_authorized = False
        self._policy_authorized_s: Optional[float] = None
        self._policy_first_write_seen = False
        self._policy_hold_requested = False
        self._policy_hold_first_write_seen = False
        self._policy_fresh_seen = False
        self._policy_stale_confirmed = False
        self._stable_since_s: Optional[float] = None
        self._sonic_stable_since_s: Optional[float] = None
        self._sonic_prewarm_requested = False
        self._sonic_prewarm_started_s: Optional[float] = None
        self._sonic_writer_authorized = False
        self._sonic_first_write_seen = False
        self._sonic_policy_full_control_seen = False
        self._sonic_safe_idle_hold_seen = False

    @property
    def failed(self) -> bool:
        return self.state is RecoveryState.FAILED

    def _transition(self, state: RecoveryState, now_s: float) -> RecoveryState:
        previous = self.state
        self.state = state
        self._state_entered_s = now_s
        return previous

    def _result(self, previous: RecoveryState, **actions: object) -> RecoveryOutput:
        defaults = {
            "inhibit_game_input": self.state is not RecoveryState.GAME_SONIC,
            "fail_closed": self.state is RecoveryState.FAILED,
            "failure_reason": self.failure_reason,
        }
        defaults.update(actions)
        return RecoveryOutput(previous_state=previous, state=self.state, **defaults)

    def _fail(self, observation: RecoveryInput, reason: str) -> RecoveryOutput:
        previous = self.state
        self.state = RecoveryState.FAILED
        self.failure_reason = reason
        self._state_entered_s = observation.now_s
        return self._result(
            previous,
            request_sonic_stop=bool(observation.deploy_alive),
            request_policy_stop=bool(observation.policy_alive),
            inhibit_game_input=True,
            fail_closed=True,
            failure_reason=reason,
        )

    def _validate(self, observation: RecoveryInput) -> Optional[str]:
        scalar_names = (
            "now_s",
            "root_z_m",
            "root_up_z",
            "root_linear_speed_m_s",
            "root_angular_speed_rad_s",
            "joint_velocity_rms_rad_s",
        )
        for name in scalar_names:
            value = getattr(observation, name)
            if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
                return f"invalid_{name}"
        if observation.root_linear_speed_m_s < 0.0:
            return "negative_root_linear_speed"
        if observation.root_angular_speed_rad_s < 0.0:
            return "negative_root_angular_speed"
        if observation.joint_velocity_rms_rad_s < 0.0:
            return "negative_joint_velocity_rms"
        if not isinstance(observation.deploy_generation, int) or observation.deploy_generation < 0:
            return "invalid_deploy_generation"
        if not isinstance(observation.reset_count, int) or observation.reset_count < 0:
            return "invalid_reset_count"
        for name in (
            "fall_detected",
            "lowcmd_fresh",
            "deploy_alive",
            "deploy_process_ready",
            "deploy_writer_ready",
            "deploy_writer_created",
            "deploy_writer_revoked",
            "deploy_first_write",
            "deploy_policy_full_control",
            "deploy_safe_idle_hold",
            "policy_alive",
            "policy_ready",
            "policy_first_write",
            "policy_hold_first_write",
            "foot_contact",
            "grounded_contact",
            "neutral_confirmed",
        ):
            if type(getattr(observation, name)) is not bool:
                return f"invalid_{name}"
        if observation.deploy_first_write and not observation.deploy_writer_created:
            return "sonic_first_write_without_writer"
        if (
            observation.deploy_policy_full_control
            and not observation.deploy_first_write
        ):
            return "sonic_policy_control_without_first_write"
        if observation.deploy_safe_idle_hold and not observation.deploy_first_write:
            return "sonic_safe_idle_hold_without_first_write"
        if observation.policy_hold_first_write and not observation.policy_first_write:
            return "policy_hold_write_without_initial_write"
        if observation.lowcmd_age_s is not None:
            age = observation.lowcmd_age_s
            if not isinstance(age, (int, float)) or not math.isfinite(float(age)) or age < 0.0:
                return "invalid_lowcmd_age"
        if observation.lowcmd_fresh:
            if observation.lowcmd_age_s is None:
                return "fresh_lowcmd_missing_age"
            if observation.lowcmd_age_s > self.config.max_lowcmd_age_s:
                return "fresh_lowcmd_age_exceeds_limit"
        if self._last_now_s is not None and observation.now_s < self._last_now_s:
            return "monotonic_time_regressed"
        return None

    def _is_stable(self, observation: RecoveryInput) -> bool:
        return (
            observation.lowcmd_fresh
            and observation.foot_contact
            and observation.root_z_m >= self.config.stable_root_z_m
            and observation.root_up_z >= self.config.stable_root_up_z
            and observation.root_linear_speed_m_s
            <= self.config.stable_root_linear_speed_m_s
            and observation.root_angular_speed_rad_s
            <= self.config.stable_root_angular_speed_rad_s
            and observation.joint_velocity_rms_rad_s
            <= self.config.stable_joint_velocity_rms_rad_s
        )

    def _is_policy_handoff_ready(self, observation: RecoveryInput) -> bool:
        """Accept an upright supported pose for a physical PD braking hold.

        This gate does *not* authorize SONIC.  It only moves the same physical
        writer from the learned get-up motion into a measured-joint PD brake.
        The brake must then satisfy ``_is_stable`` continuously before the
        worker can stop and SONIC can receive GO.  The geometry and dynamics
        here are tight enough to reject a low crouch or violent fly-through,
        while still catching the brief upright apex observed in live HoST.
        """

        return (
            observation.lowcmd_fresh
            and observation.foot_contact
            and observation.root_z_m >= self.config.policy_handoff_root_z_m
            and observation.root_up_z >= self.config.policy_handoff_root_up_z
            and observation.root_linear_speed_m_s
            <= self.config.policy_handoff_root_linear_speed_m_s
            and observation.root_angular_speed_rad_s
            <= self.config.policy_handoff_root_angular_speed_rad_s
            and observation.joint_velocity_rms_rad_s
            <= self.config.policy_handoff_joint_velocity_rms_rad_s
        )

    def _writer_invariant_failure(self, observation: RecoveryInput) -> Optional[str]:
        policy_writer_capable = observation.policy_alive and (
            self._policy_writer_authorized or observation.policy_first_write
        )
        deploy_writer_capable = (
            observation.deploy_alive
            and not observation.deploy_writer_revoked
            and (
                observation.deploy_writer_created
                or self._sonic_writer_authorized
                or observation.deploy_first_write
            )
        )
        writer_capable_count = int(deploy_writer_capable) + int(policy_writer_capable)
        if writer_capable_count > 1:
            return "multiple_writer_capable_processes_alive"
        if (
            observation.policy_alive
            and observation.policy_first_write
            and not self._policy_writer_authorized
        ):
            return "policy_wrote_before_authorization"
        if observation.policy_hold_first_write and not self._policy_hold_requested:
            return "policy_entered_amp_hold_before_request"
        if (
            self._sonic_prewarm_requested
            and observation.deploy_first_write
            and not self._sonic_writer_authorized
        ):
            return "sonic_wrote_before_authorization"
        return None

    def _prewarm_deploy_failure(self, observation: RecoveryInput) -> Optional[str]:
        """Validate a live replacement that is explicitly writer-free."""

        if not self._sonic_prewarm_requested:
            return "sonic_started_before_prewarm_request" if observation.deploy_alive else None
        if (
            observation.deploy_process_ready or observation.deploy_writer_ready
        ) and not observation.deploy_alive:
            return "dead_sonic_reported_writer_ready"
        if not observation.deploy_alive:
            if self._new_deploy_generation is not None:
                return "new_sonic_exited_during_prewarm"
            return None
        assert self._old_deploy_generation is not None
        if observation.deploy_generation <= self._old_deploy_generation:
            return "sonic_prewarm_did_not_advance_generation"
        if self._new_deploy_generation is None:
            self._new_deploy_generation = observation.deploy_generation
        elif observation.deploy_generation != self._new_deploy_generation:
            return "sonic_generation_changed_during_prewarm"
        if observation.deploy_writer_created or observation.deploy_first_write:
            return "sonic_prewarm_created_writer_before_go"
        if observation.deploy_writer_revoked:
            return "sonic_prewarm_inherited_revoked_writer_state"
        return None

    def _check_deadline(self, observation: RecoveryInput) -> Optional[str]:
        if (
            self.state is RecoveryState.GAME_SONIC
            and self._policy_standby_requested
        ):
            if self._policy_standby_ready_seen and not observation.policy_ready:
                return "policy_standby_readiness_lost"
            if (
                self._policy_standby_requested_s is not None
                and not observation.policy_ready
                and observation.now_s - self._policy_standby_requested_s
                > self.config.policy_start_timeout_s
            ):
                return "timeout_policy_standby"
        if (
            self.state
            in (
                RecoveryState.POLICY_RECOVERING,
                RecoveryState.POLICY_GETUP_STABLE,
                RecoveryState.POLICY_AMP_HOLD_REQUESTED,
                RecoveryState.POLICY_AMP_HOLDING,
                RecoveryState.POLICY_STOP_REQUESTED,
            )
            and self._policy_authorized_s is not None
            and observation.now_s - self._policy_authorized_s
            > self.config.policy_recovery_timeout_s
        ):
            # This is an absolute deadline from GO.  Alternating between the
            # recovering and stable-hold states must never extend policy
            # ownership indefinitely.
            return "timeout_policy_recovery"
        if (
            self._sonic_prewarm_started_s is not None
            and not observation.deploy_process_ready
            and observation.now_s - self._sonic_prewarm_started_s
            > self.config.sonic_prewarm_timeout_s
        ):
            return "timeout_sonic_prewarm"
        if (
            self.state is RecoveryState.SONIC_STABILIZING
            and not (
                self._sonic_policy_full_control_seen
                or self._sonic_safe_idle_hold_seen
                or observation.deploy_policy_full_control
                or observation.deploy_safe_idle_hold
            )
            and self._state_entered_s is not None
            and observation.now_s - self._state_entered_s
            > self.config.sonic_full_control_timeout_s
        ):
            return "timeout_sonic_full_control"
        if (
            self.state is RecoveryState.WAIT_NEUTRAL
            and not (
                self._sonic_policy_full_control_seen
                or observation.deploy_policy_full_control
            )
            and self._state_entered_s is not None
            and observation.now_s - self._state_entered_s
            > self.config.sonic_full_control_timeout_s
        ):
            # A safe IDLE hold is not permission to resume user input.  The
            # native controller must still finish its writer-ACKed IDLE policy
            # takeover and attest full control within the same deadline.
            return "timeout_sonic_full_control"
        timeout_attr = self._STAGE_TIMEOUT_ATTR.get(self.state)
        if timeout_attr is not None and self._state_entered_s is not None:
            if observation.now_s - self._state_entered_s > getattr(self.config, timeout_attr):
                return f"timeout_{self.state.value.lower()}"
        if self._episode_started_s is not None and self.state not in (
            RecoveryState.GAME_SONIC,
            RecoveryState.FAILED,
        ):
            if observation.now_s - self._episode_started_s > self.config.episode_timeout_s:
                return "timeout_recovery_episode"
        if (
            self.state is RecoveryState.POLICY_RECOVERING
            and self._policy_authorized_s is not None
            and not self._policy_first_write_seen
            and observation.now_s - self._policy_authorized_s
            > self.config.policy_first_write_timeout_s
        ):
            return "timeout_policy_first_write"
        return None

    def step(self, observation: RecoveryInput) -> RecoveryOutput:
        """Consume one observation and return one immutable action decision."""

        if self.state is RecoveryState.FAILED:
            return self._result(
                RecoveryState.FAILED,
                request_sonic_stop=bool(observation.deploy_alive),
                request_policy_stop=bool(observation.policy_alive),
                inhibit_game_input=True,
                fail_closed=True,
            )

        invalid = self._validate(observation)
        if invalid is not None:
            return self._fail(observation, invalid)

        if self._state_entered_s is None:
            self._state_entered_s = observation.now_s
        if self._baseline_reset_count is None:
            self._baseline_reset_count = observation.reset_count
        elif observation.reset_count != self._baseline_reset_count:
            return self._fail(observation, "reset_count_changed")

        invariant = self._writer_invariant_failure(observation)
        if invariant is not None:
            return self._fail(observation, invariant)
        deadline = self._check_deadline(observation)
        if deadline is not None:
            return self._fail(observation, deadline)

        previous_fresh = self._last_lowcmd_fresh
        self._last_lowcmd_fresh = bool(observation.lowcmd_fresh)
        self._last_now_s = observation.now_s

        standby_states = {
            RecoveryState.GAME_SONIC,
            RecoveryState.SONIC_STOP_REQUESTED,
            RecoveryState.SONIC_QUIET,
            RecoveryState.POLICY_STARTING,
        }
        if self.state in standby_states and self._policy_standby_requested:
            if observation.policy_alive:
                self._policy_standby_seen_alive = True
                self._policy_standby_ready_seen |= bool(observation.policy_ready)
            elif self._policy_standby_seen_alive:
                return self._fail(observation, "policy_standby_exited_before_go")

        if self.state is RecoveryState.GAME_SONIC:
            start_policy_process = False
            if not self._policy_standby_requested:
                self._policy_standby_requested = True
                self._policy_standby_requested_s = observation.now_s
                self._policy_standby_seen_alive = False
                self._policy_standby_ready_seen = False
                start_policy_process = True
            if not observation.fall_detected:
                self._fall_latched = False
            if observation.fall_detected and not self._fall_latched:
                if not observation.deploy_alive:
                    return self._fail(observation, "fall_without_live_sonic_deploy")
                self._fall_latched = True
                self._episode_started_s = observation.now_s
                self._old_deploy_generation = observation.deploy_generation
                self._new_deploy_generation = None
                self._sonic_fresh_seen = bool(observation.lowcmd_fresh)
                self._sonic_stale_confirmed = False
                self._policy_seen_alive = bool(observation.policy_alive)
                self._policy_writer_authorized = False
                self._policy_authorized_s = None
                self._policy_first_write_seen = False
                self._policy_hold_requested = False
                self._policy_hold_first_write_seen = False
                self._policy_fresh_seen = False
                self._policy_stale_confirmed = False
                self._stable_since_s = None
                self._sonic_stable_since_s = None
                self._sonic_prewarm_requested = False
                self._sonic_prewarm_started_s = None
                self._sonic_writer_authorized = False
                self._sonic_first_write_seen = False
                self._sonic_policy_full_control_seen = False
                self._sonic_safe_idle_hold_seen = False
                previous = self._transition(
                    RecoveryState.SONIC_STOP_REQUESTED, observation.now_s
                )
                return self._result(
                    previous,
                    request_sonic_stop=True,
                    start_policy_process=start_policy_process,
                )
            return self._result(
                RecoveryState.GAME_SONIC,
                start_policy_process=start_policy_process,
            )

        if self.state is RecoveryState.SONIC_STOP_REQUESTED:
            if observation.deploy_alive and observation.deploy_generation != self._old_deploy_generation:
                return self._fail(observation, "sonic_generation_changed_while_stopping")
            self._sonic_fresh_seen |= bool(observation.lowcmd_fresh)
            if observation.deploy_alive and not observation.deploy_writer_revoked:
                return self._result(RecoveryState.SONIC_STOP_REQUESTED)
            # The final DDS sample may age stale slightly before the deploy
            # process is reaped.  Fresh-seen plus current-stale still proves
            # the required fresh->stale transition, provided the process is
            # now dead.  Requiring the edge on this exact frame loses that
            # valid ordering and deadlocks in SONIC_QUIET.
            if self._sonic_fresh_seen and not observation.lowcmd_fresh:
                self._sonic_stale_confirmed = True
            previous = self._transition(RecoveryState.SONIC_QUIET, observation.now_s)
            return self._result(previous)

        if self.state is RecoveryState.SONIC_QUIET:
            if observation.deploy_alive:
                if observation.deploy_generation != self._old_deploy_generation:
                    return self._fail(
                        observation, "sonic_generation_changed_before_policy_handoff"
                    )
                if not observation.deploy_writer_revoked:
                    return self._fail(
                        observation, "sonic_writer_revived_before_policy_handoff"
                    )
            self._sonic_fresh_seen |= bool(observation.lowcmd_fresh)
            if self._sonic_fresh_seen and not observation.lowcmd_fresh:
                self._sonic_stale_confirmed = True
            if not self._sonic_stale_confirmed:
                return self._result(RecoveryState.SONIC_QUIET)
            previous = self._transition(RecoveryState.POLICY_STARTING, observation.now_s)
            return self._result(previous)

        if self.state is RecoveryState.POLICY_STARTING:
            if (
                (
                    observation.deploy_alive
                    and not observation.deploy_writer_revoked
                )
                or not self._sonic_stale_confirmed
            ):
                return self._fail(observation, "sonic_not_quiet_at_policy_start")
            self._policy_seen_alive |= bool(observation.policy_alive)
            if self._policy_seen_alive and not observation.policy_alive:
                return self._fail(observation, "policy_exited_during_startup")
            if not (observation.policy_alive and observation.policy_ready):
                return self._result(RecoveryState.POLICY_STARTING)
            assert self._episode_started_s is not None
            if (
                observation.now_s - self._episode_started_s
                < self.config.takeover_settle_s
            ):
                # Loading is writer-free and may overlap the physical settling
                # interval, but GO must not.  The simulator therefore holds the
                # last real SONIC PD target for at least 0.35 s after the fall.
                return self._result(RecoveryState.POLICY_STARTING)
            takeover_is_quiet_and_grounded = (
                observation.grounded_contact
                and observation.root_linear_speed_m_s
                <= self.config.takeover_root_linear_speed_m_s
                and observation.root_angular_speed_rad_s
                <= self.config.takeover_root_angular_speed_rad_s
                and observation.joint_velocity_rms_rad_s
                <= self.config.takeover_joint_velocity_rms_rad_s
            )
            if not takeover_is_quiet_and_grounded:
                # HoST is trained to begin from a settled physical pose.  Do
                # not spend its recovery attempt fighting residual impact
                # velocity; the writer remains absent while the body settles.
                return self._result(RecoveryState.POLICY_STARTING)
            self._policy_writer_authorized = True
            self._policy_authorized_s = observation.now_s
            previous = self._transition(RecoveryState.POLICY_RECOVERING, observation.now_s)
            return self._result(previous, authorize_policy_writer=True)

        if self.state in (
            RecoveryState.POLICY_RECOVERING,
            RecoveryState.POLICY_GETUP_STABLE,
            RecoveryState.POLICY_AMP_HOLD_REQUESTED,
            RecoveryState.POLICY_AMP_HOLDING,
        ):
            if not observation.policy_alive:
                return self._fail(observation, "policy_exited_before_recovery_complete")
            self._policy_first_write_seen |= bool(observation.policy_first_write)
            if not self._policy_first_write_seen:
                return self._result(RecoveryState.POLICY_RECOVERING)

            # Start the replacement as soon as HoST has demonstrably taken
            # ownership.  Its native gate keeps rt/lowcmd absent while TensorRT
            # warms, so this overlap does not violate the single-writer rule.
            if not self._sonic_prewarm_requested:
                if observation.deploy_alive:
                    if observation.deploy_generation != self._old_deploy_generation:
                        return self._fail(
                            observation,
                            "sonic_generation_changed_before_prewarm",
                        )
                    if not observation.deploy_writer_revoked:
                        return self._fail(
                            observation,
                            "old_sonic_writer_not_revoked_before_prewarm",
                        )
                    return self._result(self.state)
                self._sonic_prewarm_requested = True
                self._sonic_prewarm_started_s = observation.now_s
                return self._result(self.state, start_sonic=True)
            prewarm_failure = self._prewarm_deploy_failure(observation)
            if prewarm_failure is not None:
                return self._fail(observation, prewarm_failure)

            stable = self._is_stable(observation)
            # AMP handoff may use the wider, short-lived capture gate because
            # AMP remains a dynamic balance controller.  Direct SONIC handoff
            # deliberately keeps HoST in control until the stricter stability
            # gate has held continuously; a fixed measured-joint pose cannot
            # balance a free-standing humanoid through TensorRT prewarm.
            handoff_ready = (
                self._is_policy_handoff_ready(observation)
                if self.config.use_amp_hold
                else stable
            )
            if self.state is RecoveryState.POLICY_RECOVERING:
                if not handoff_ready:
                    return self._result(RecoveryState.POLICY_RECOVERING)
                self._stable_since_s = observation.now_s
                previous = self._transition(
                    RecoveryState.POLICY_GETUP_STABLE, observation.now_s
                )
                return self._result(previous)

            if self.state is RecoveryState.POLICY_GETUP_STABLE:
                if not handoff_ready:
                    self._stable_since_s = None
                    previous = self._transition(
                        RecoveryState.POLICY_RECOVERING, observation.now_s
                    )
                    return self._result(previous)
                assert self._stable_since_s is not None
                required_hold_s = (
                    self.config.policy_handoff_hold_s
                    if self.config.use_amp_hold
                    else self.config.stable_hold_s
                )
                if (
                    observation.now_s - self._stable_since_s
                    < required_hold_s
                ):
                    return self._result(RecoveryState.POLICY_GETUP_STABLE)
                if not self.config.use_amp_hold:
                    # Keep the learned HoST balance controller authoritative
                    # while replacement SONIC is writer-free and warming.  Stop
                    # HoST only after SONIC is ready and the real robot has met
                    # the strict stability gate continuously.
                    if not observation.deploy_writer_ready:
                        return self._result(
                            RecoveryState.POLICY_GETUP_STABLE
                        )
                    self._policy_fresh_seen = bool(observation.lowcmd_fresh)
                    previous = self._transition(
                        RecoveryState.POLICY_STOP_REQUESTED,
                        observation.now_s,
                    )
                    return self._result(
                        previous,
                        request_policy_stop=True,
                    )
                self._policy_hold_requested = True
                previous = self._transition(
                    RecoveryState.POLICY_AMP_HOLD_REQUESTED,
                    observation.now_s,
                )
                return self._result(previous, request_policy_hold=True)

            if self.state is RecoveryState.POLICY_AMP_HOLD_REQUESTED:
                self._policy_hold_first_write_seen |= bool(
                    observation.policy_hold_first_write
                )
                if not self._policy_hold_first_write_seen:
                    return self._result(RecoveryState.POLICY_AMP_HOLD_REQUESTED)
                self._stable_since_s = observation.now_s if stable else None
                previous = self._transition(
                    RecoveryState.POLICY_AMP_HOLDING, observation.now_s
                )
                return self._result(previous)

            assert self.state is RecoveryState.POLICY_AMP_HOLDING
            self._policy_hold_first_write_seen |= bool(
                observation.policy_hold_first_write
            )
            if not self._policy_hold_first_write_seen:
                return self._fail(observation, "AMP_hold_write_latch_lost")
            if not stable:
                self._stable_since_s = None
                return self._result(RecoveryState.POLICY_AMP_HOLDING)
            if self._stable_since_s is None:
                self._stable_since_s = observation.now_s
                return self._result(RecoveryState.POLICY_AMP_HOLDING)
            if observation.now_s - self._stable_since_s < self.config.stable_hold_s:
                return self._result(RecoveryState.POLICY_AMP_HOLDING)
            if not observation.deploy_writer_ready:
                return self._result(RecoveryState.POLICY_AMP_HOLDING)
            self._policy_fresh_seen = bool(observation.lowcmd_fresh)
            previous = self._transition(
                RecoveryState.POLICY_STOP_REQUESTED, observation.now_s
            )
            return self._result(previous, request_policy_stop=True)

        if self.state is RecoveryState.POLICY_STOP_REQUESTED:
            prewarm_failure = self._prewarm_deploy_failure(observation)
            if prewarm_failure is not None:
                return self._fail(observation, prewarm_failure)
            if not observation.deploy_writer_ready:
                return self._fail(observation, "sonic_lost_ready_before_policy_quiet")
            self._policy_fresh_seen |= bool(observation.lowcmd_fresh)
            if observation.policy_alive:
                return self._result(RecoveryState.POLICY_STOP_REQUESTED)
            self._policy_writer_authorized = False
            if self._policy_fresh_seen and not observation.lowcmd_fresh:
                self._policy_stale_confirmed = True
            previous = self._transition(RecoveryState.POLICY_QUIET, observation.now_s)
            return self._result(previous)

        if self.state is RecoveryState.POLICY_QUIET:
            prewarm_failure = self._prewarm_deploy_failure(observation)
            if prewarm_failure is not None:
                return self._fail(observation, prewarm_failure)
            if not observation.deploy_writer_ready:
                return self._fail(observation, "sonic_lost_ready_before_writer_go")
            if observation.policy_alive:
                return self._fail(observation, "policy_revived_before_sonic_restart")
            self._policy_fresh_seen |= bool(observation.lowcmd_fresh)
            if self._policy_fresh_seen and not observation.lowcmd_fresh:
                self._policy_stale_confirmed = True
            if not self._policy_stale_confirmed:
                return self._result(RecoveryState.POLICY_QUIET)
            self._sonic_writer_authorized = True
            previous = self._transition(RecoveryState.SONIC_RESTARTING, observation.now_s)
            return self._result(previous, authorize_sonic_writer=True)

        if self.state is RecoveryState.SONIC_RESTARTING:
            if observation.policy_alive:
                return self._fail(observation, "policy_alive_during_sonic_restart")
            if not observation.deploy_alive:
                return self._fail(observation, "new_sonic_exited_after_writer_go")
            if observation.deploy_generation != self._new_deploy_generation:
                return self._fail(observation, "sonic_generation_changed_after_writer_go")
            if not observation.deploy_writer_ready:
                return self._fail(observation, "sonic_writer_ready_latch_lost")
            self._sonic_first_write_seen |= bool(observation.deploy_first_write)
            if not (self._sonic_first_write_seen and observation.lowcmd_fresh):
                return self._result(RecoveryState.SONIC_RESTARTING)
            previous = self._transition(
                RecoveryState.SONIC_STABILIZING, observation.now_s
            )
            return self._result(previous)

        if self.state is RecoveryState.SONIC_STABILIZING:
            if observation.policy_alive:
                return self._fail(observation, "policy_alive_during_sonic_stabilization")
            if not observation.deploy_alive:
                return self._fail(observation, "new_sonic_exited_during_stabilization")
            if observation.deploy_generation != self._new_deploy_generation:
                return self._fail(observation, "sonic_generation_changed_during_stabilization")
            self._sonic_first_write_seen |= bool(observation.deploy_first_write)
            if not self._sonic_first_write_seen:
                return self._fail(observation, "sonic_first_write_latch_lost")
            self._sonic_policy_full_control_seen |= bool(
                observation.deploy_policy_full_control
            )
            self._sonic_safe_idle_hold_seen |= bool(
                observation.deploy_safe_idle_hold
            )
            if not (
                self._sonic_policy_full_control_seen
                or self._sonic_safe_idle_hold_seen
            ):
                self._sonic_stable_since_s = None
                return self._result(RecoveryState.SONIC_STABILIZING)
            if not self._is_stable(observation):
                self._sonic_stable_since_s = None
                return self._result(RecoveryState.SONIC_STABILIZING)
            if self._sonic_stable_since_s is None:
                self._sonic_stable_since_s = observation.now_s
                return self._result(RecoveryState.SONIC_STABILIZING)
            if (
                observation.now_s - self._sonic_stable_since_s
                < self.config.stable_hold_s
            ):
                return self._result(RecoveryState.SONIC_STABILIZING)
            previous = self._transition(RecoveryState.WAIT_NEUTRAL, observation.now_s)
            return self._result(previous)

        if self.state is RecoveryState.WAIT_NEUTRAL:
            if observation.policy_alive:
                return self._fail(observation, "policy_alive_while_waiting_for_neutral")
            if not observation.deploy_alive:
                return self._fail(observation, "sonic_exited_while_waiting_for_neutral")
            if observation.deploy_generation != self._new_deploy_generation:
                return self._fail(observation, "sonic_generation_changed_while_waiting_for_neutral")
            if not observation.lowcmd_fresh:
                return self._fail(observation, "sonic_lowcmd_stale_while_waiting_for_neutral")
            self._sonic_policy_full_control_seen |= bool(
                observation.deploy_policy_full_control
            )
            self._sonic_safe_idle_hold_seen |= bool(
                observation.deploy_safe_idle_hold
            )
            if not (
                observation.deploy_policy_full_control
                or observation.deploy_safe_idle_hold
            ):
                return self._fail(
                    observation,
                    "sonic_policy_control_latch_lost_while_waiting_for_neutral",
                )
            # Keep publishing only fixed deploy-frame IDLE and inhibit every
            # external game command until SONIC explicitly attests that its
            # writer-ACKed policy takeover is complete.
            if not self._sonic_policy_full_control_seen:
                return self._result(RecoveryState.WAIT_NEUTRAL)
            if observation.fall_detected or not self._is_stable(observation):
                self._sonic_stable_since_s = None
                return self._result(RecoveryState.WAIT_NEUTRAL)
            if not observation.neutral_confirmed:
                return self._result(RecoveryState.WAIT_NEUTRAL)
            self._episode_started_s = None
            self._policy_standby_requested = True
            self._policy_standby_requested_s = observation.now_s
            self._policy_standby_seen_alive = False
            self._policy_standby_ready_seen = False
            previous = self._transition(RecoveryState.GAME_SONIC, observation.now_s)
            return self._result(
                previous,
                start_policy_process=True,
                inhibit_game_input=False,
                resume_game=True,
            )

        return self._fail(observation, "unknown_recovery_state")


class ResidentRecoveryState(str, Enum):
    """Authority states for resident SONIC and recovery policy processes."""

    GAME_SONIC = "GAME_SONIC"
    SONIC_PAUSE_REQUESTED = "SONIC_PAUSE_REQUESTED"
    SONIC_QUIET = "SONIC_QUIET"
    POLICY_STARTING = "POLICY_STARTING"
    POLICY_RECOVERING = "POLICY_RECOVERING"
    POLICY_STABLE = "POLICY_STABLE"
    POLICY_PAUSE_REQUESTED = "POLICY_PAUSE_REQUESTED"
    POLICY_QUIET = "POLICY_QUIET"
    # Source compatibility for callers written before the policy-id registry.
    KUNGFU_STARTING = "POLICY_STARTING"
    KUNGFU_RECOVERING = "POLICY_RECOVERING"
    KUNGFU_STABLE = "POLICY_STABLE"
    KUNGFU_PAUSE_REQUESTED = "POLICY_PAUSE_REQUESTED"
    KUNGFU_QUIET = "POLICY_QUIET"
    SONIC_RESUME_REQUESTED = "SONIC_RESUME_REQUESTED"
    SONIC_STABILIZING = "SONIC_STABILIZING"
    WAIT_NEUTRAL = "WAIT_NEUTRAL"
    FAILED = "FAILED"


@dataclass(frozen=True)
class ResidentRecoveryInput:
    now_s: float
    fall_detected: bool
    root_z_m: float
    root_up_z: float
    root_linear_speed_m_s: float
    root_angular_speed_rad_s: float
    joint_velocity_rms_rad_s: float
    lowcmd_fresh: bool
    lowcmd_age_s: Optional[float]
    sonic_alive: bool
    sonic_generation: int
    sonic_resident_ready: bool
    sonic_writer_active: bool
    sonic_writer_paused: bool
    sonic_resume_first_write: bool
    policy_alive: bool
    policy_resident_ready: bool
    policy_writer_active: bool
    policy_writer_paused: bool
    policy_first_write: bool
    reset_count: int
    foot_contact: bool
    grounded_contact: bool
    neutral_confirmed: bool = False


@dataclass(frozen=True)
class ResidentRecoveryOutput:
    previous_state: ResidentRecoveryState
    state: ResidentRecoveryState
    start_policy_process: bool = False
    request_sonic_pause: bool = False
    authorize_policy_writer: bool = False
    request_policy_pause: bool = False
    resume_sonic_writer: bool = False
    inhibit_game_input: bool = False
    resume_game: bool = False
    fail_closed: bool = False
    failure_reason: Optional[str] = None
    # Compatibility fields keep the outer runtime free of lifecycle guessing.
    request_sonic_stop: bool = False
    request_policy_stop: bool = False
    request_policy_hold: bool = False
    start_sonic: bool = False
    authorize_sonic_writer: bool = False
    authority_policy_id: Optional[str] = None
    recovery_policy_id: Optional[str] = None


class ResidentPolicyRecoveryFSM:
    """Switch output authority without unloading or recreating any policy."""

    _STAGE_TIMEOUTS = {
        ResidentRecoveryState.SONIC_PAUSE_REQUESTED: "sonic_stop_timeout_s",
        ResidentRecoveryState.SONIC_QUIET: "quiet_timeout_s",
        ResidentRecoveryState.POLICY_STARTING: "policy_start_timeout_s",
        ResidentRecoveryState.POLICY_RECOVERING: "policy_recovery_timeout_s",
        ResidentRecoveryState.POLICY_STABLE: "policy_recovery_timeout_s",
        ResidentRecoveryState.POLICY_PAUSE_REQUESTED: "policy_stop_timeout_s",
        ResidentRecoveryState.POLICY_QUIET: "quiet_timeout_s",
        ResidentRecoveryState.SONIC_RESUME_REQUESTED: "sonic_restart_timeout_s",
        ResidentRecoveryState.SONIC_STABILIZING: "sonic_full_control_timeout_s",
        ResidentRecoveryState.WAIT_NEUTRAL: "neutral_timeout_s",
    }

    def __init__(
        self,
        config: RecoveryConfig = RecoveryConfig(),
        *,
        game_policy_id: str = "sonic",
        recovery_policy_id: str = "kungfu",
    ) -> None:
        self.config = config
        self.game_policy_id = self._normalize_policy_id(game_policy_id)
        self.recovery_policy_id = self._normalize_policy_id(recovery_policy_id)
        if self.game_policy_id == self.recovery_policy_id:
            raise ValueError("game and recovery policy IDs must be distinct")
        self.state = ResidentRecoveryState.GAME_SONIC
        self.failure_reason: Optional[str] = None
        self._state_entered_s: Optional[float] = None
        self._last_now_s: Optional[float] = None
        self._episode_started_s: Optional[float] = None
        self._baseline_reset_count: Optional[int] = None
        self._sonic_generation: Optional[int] = None
        self._fall_latched = False
        self._policy_start_requested = False
        self._policy_authorize_requested = False
        self._sonic_fresh_seen = False
        self._sonic_stale_confirmed = False
        self._policy_fresh_seen = False
        self._policy_stale_confirmed = False
        self._stable_since_s: Optional[float] = None
        self._sonic_stable_since_s: Optional[float] = None

    @staticmethod
    def _normalize_policy_id(policy_id: str) -> str:
        value = str(policy_id).strip().lower()
        if (
            not value
            or value[0] not in "abcdefghijklmnopqrstuvwxyz0123456789"
            or any(
                character not in "abcdefghijklmnopqrstuvwxyz0123456789._-"
                for character in value
            )
        ):
            raise ValueError(f"invalid resident policy id: {policy_id!r}")
        return value

    @property
    def failed(self) -> bool:
        return self.state is ResidentRecoveryState.FAILED

    def _transition(
        self, state: ResidentRecoveryState, now_s: float
    ) -> ResidentRecoveryState:
        previous = self.state
        self.state = state
        self._state_entered_s = now_s
        return previous

    def _result(
        self,
        previous: ResidentRecoveryState,
        **actions: object,
    ) -> ResidentRecoveryOutput:
        values = dict(actions)
        values.setdefault(
            "inhibit_game_input",
            self.state is not ResidentRecoveryState.GAME_SONIC,
        )
        values.setdefault("recovery_policy_id", self.recovery_policy_id)
        if "authority_policy_id" not in values:
            if self.state in {
                ResidentRecoveryState.GAME_SONIC,
                ResidentRecoveryState.SONIC_PAUSE_REQUESTED,
                ResidentRecoveryState.SONIC_STABILIZING,
                ResidentRecoveryState.WAIT_NEUTRAL,
            }:
                values["authority_policy_id"] = self.game_policy_id
            elif self.state in {
                ResidentRecoveryState.POLICY_RECOVERING,
                ResidentRecoveryState.POLICY_STABLE,
                ResidentRecoveryState.POLICY_PAUSE_REQUESTED,
            }:
                values["authority_policy_id"] = self.recovery_policy_id
        return ResidentRecoveryOutput(
            previous_state=previous,
            state=self.state,
            **values,
        )

    def _fail(
        self, observation: ResidentRecoveryInput, reason: str
    ) -> ResidentRecoveryOutput:
        previous = self._transition(ResidentRecoveryState.FAILED, observation.now_s)
        self.failure_reason = reason
        return self._result(
            previous,
            request_sonic_pause=bool(
                observation.sonic_alive and observation.sonic_writer_active
            ),
            request_policy_pause=bool(
                observation.policy_alive and observation.policy_writer_active
            ),
            inhibit_game_input=True,
            fail_closed=True,
            failure_reason=reason,
        )

    def _validate(self, observation: ResidentRecoveryInput) -> Optional[str]:
        numeric = (
            observation.now_s,
            observation.root_z_m,
            observation.root_up_z,
            observation.root_linear_speed_m_s,
            observation.root_angular_speed_rad_s,
            observation.joint_velocity_rms_rad_s,
        )
        if any(not math.isfinite(float(value)) for value in numeric):
            return "non_finite_observation"
        if self._last_now_s is not None and observation.now_s < self._last_now_s:
            return "monotonic_time_regressed"
        if not isinstance(observation.sonic_generation, int) or observation.sonic_generation < 0:
            return "invalid_sonic_generation"
        if not isinstance(observation.reset_count, int) or observation.reset_count < 0:
            return "invalid_reset_count"
        if observation.lowcmd_age_s is not None and (
            not math.isfinite(float(observation.lowcmd_age_s))
            or float(observation.lowcmd_age_s) < 0.0
        ):
            return "invalid_lowcmd_age"
        return None

    def _stable(self, observation: ResidentRecoveryInput) -> bool:
        return (
            not observation.fall_detected
            and observation.root_z_m >= self.config.stable_root_z_m
            and observation.root_up_z >= self.config.stable_root_up_z
            and observation.root_linear_speed_m_s
            <= self.config.stable_root_linear_speed_m_s
            and observation.root_angular_speed_rad_s
            <= self.config.stable_root_angular_speed_rad_s
            and observation.joint_velocity_rms_rad_s
            <= self.config.stable_joint_velocity_rms_rad_s
            and observation.foot_contact
            and observation.grounded_contact
            and observation.lowcmd_fresh
            and observation.lowcmd_age_s is not None
            and observation.lowcmd_age_s <= self.config.max_lowcmd_age_s
        )

    def _deadline_failure(
        self, observation: ResidentRecoveryInput
    ) -> Optional[str]:
        if (
            self._episode_started_s is not None
            and observation.now_s - self._episode_started_s
            > self.config.episode_timeout_s
        ):
            return "timeout_resident_episode"
        timeout_attr = self._STAGE_TIMEOUTS.get(self.state)
        if timeout_attr is None or self._state_entered_s is None:
            return None
        if observation.now_s - self._state_entered_s > float(
            getattr(self.config, timeout_attr)
        ):
            return f"timeout_{self.state.value.lower()}"
        return None

    def _begin_fall_episode(
        self,
        observation: ResidentRecoveryInput,
        *,
        start_policy_process: bool = False,
    ) -> ResidentRecoveryOutput:
        self._fall_latched = True
        self._episode_started_s = observation.now_s
        self._sonic_generation = observation.sonic_generation
        self._sonic_fresh_seen = bool(observation.lowcmd_fresh)
        self._sonic_stale_confirmed = False
        self._policy_fresh_seen = False
        self._policy_stale_confirmed = False
        self._policy_authorize_requested = False
        self._stable_since_s = None
        self._sonic_stable_since_s = None
        previous = self._transition(
            ResidentRecoveryState.SONIC_PAUSE_REQUESTED,
            observation.now_s,
        )
        return self._result(
            previous,
            request_sonic_pause=True,
            start_policy_process=start_policy_process,
        )

    def step(
        self, observation: ResidentRecoveryInput
    ) -> ResidentRecoveryOutput:
        if self.state is ResidentRecoveryState.FAILED:
            return self._result(
                ResidentRecoveryState.FAILED,
                request_sonic_pause=bool(
                    observation.sonic_alive and observation.sonic_writer_active
                ),
                request_policy_pause=bool(
                    observation.policy_alive and observation.policy_writer_active
                ),
                inhibit_game_input=True,
                fail_closed=True,
                failure_reason=self.failure_reason,
            )

        invalid = self._validate(observation)
        if invalid is not None:
            return self._fail(observation, invalid)
        self._last_now_s = observation.now_s
        if self._state_entered_s is None:
            self._state_entered_s = observation.now_s
        if self._baseline_reset_count is None:
            self._baseline_reset_count = observation.reset_count
        elif observation.reset_count != self._baseline_reset_count:
            return self._fail(observation, "reset_count_changed")
        if observation.sonic_writer_active and observation.policy_writer_active:
            return self._fail(observation, "multiple_active_policy_writers")
        deadline = self._deadline_failure(observation)
        if deadline is not None:
            return self._fail(observation, deadline)

        if self.state is ResidentRecoveryState.GAME_SONIC:
            start_policy = False
            if not self._policy_start_requested:
                self._policy_start_requested = True
                start_policy = True
            if not observation.fall_detected:
                self._fall_latched = False
            if observation.fall_detected and not self._fall_latched:
                if not (
                    observation.sonic_alive
                    and observation.sonic_resident_ready
                    and observation.sonic_writer_active
                    and observation.policy_alive
                    and observation.policy_resident_ready
                    and observation.policy_writer_paused
                ):
                    return self._fail(observation, "resident_policies_not_ready_at_fall")
                return self._begin_fall_episode(
                    observation,
                    start_policy_process=start_policy,
                )
            return self._result(
                ResidentRecoveryState.GAME_SONIC,
                start_policy_process=start_policy,
            )

        if not observation.sonic_alive:
            return self._fail(observation, "resident_sonic_process_exited")
        if self._sonic_generation is not None and (
            observation.sonic_generation != self._sonic_generation
        ):
            return self._fail(observation, "resident_sonic_generation_changed")
        if not observation.policy_alive:
            return self._fail(observation, "resident_recovery_policy_process_exited")
        if not (
            observation.sonic_resident_ready and observation.policy_resident_ready
        ):
            return self._fail(observation, "resident_policy_readiness_lost")

        if self.state is ResidentRecoveryState.SONIC_PAUSE_REQUESTED:
            self._sonic_fresh_seen |= bool(observation.lowcmd_fresh)
            if not observation.sonic_writer_paused:
                return self._result(ResidentRecoveryState.SONIC_PAUSE_REQUESTED)
            if self._sonic_fresh_seen and not observation.lowcmd_fresh:
                self._sonic_stale_confirmed = True
            previous = self._transition(
                ResidentRecoveryState.SONIC_QUIET, observation.now_s
            )
            return self._result(previous)

        if self.state is ResidentRecoveryState.SONIC_QUIET:
            if not observation.sonic_writer_paused:
                return self._fail(
                    observation, "sonic_writer_resumed_before_recovery_policy"
                )
            self._sonic_fresh_seen |= bool(observation.lowcmd_fresh)
            if self._sonic_fresh_seen and not observation.lowcmd_fresh:
                self._sonic_stale_confirmed = True
            if not self._sonic_stale_confirmed:
                return self._result(ResidentRecoveryState.SONIC_QUIET)
            previous = self._transition(
                ResidentRecoveryState.POLICY_STARTING, observation.now_s
            )
            return self._result(previous)

        if self.state is ResidentRecoveryState.POLICY_STARTING:
            assert self._episode_started_s is not None
            if not observation.policy_writer_active:
                if self._policy_authorize_requested:
                    return self._result(ResidentRecoveryState.POLICY_STARTING)
                if observation.now_s - self._episode_started_s < self.config.takeover_settle_s:
                    return self._result(ResidentRecoveryState.POLICY_STARTING)
                quiet_grounded = (
                    observation.grounded_contact
                    and observation.root_linear_speed_m_s
                    <= self.config.takeover_root_linear_speed_m_s
                    and observation.root_angular_speed_rad_s
                    <= self.config.takeover_root_angular_speed_rad_s
                    and observation.joint_velocity_rms_rad_s
                    <= self.config.takeover_joint_velocity_rms_rad_s
                )
                if not quiet_grounded:
                    return self._result(ResidentRecoveryState.POLICY_STARTING)
                self._policy_authorize_requested = True
                return self._result(
                    ResidentRecoveryState.POLICY_STARTING,
                    authorize_policy_writer=True,
                )
            if not (observation.policy_first_write and observation.lowcmd_fresh):
                return self._result(ResidentRecoveryState.POLICY_STARTING)
            self._policy_fresh_seen = True
            previous = self._transition(
                ResidentRecoveryState.POLICY_RECOVERING, observation.now_s
            )
            return self._result(previous)

        if self.state in {
            ResidentRecoveryState.POLICY_RECOVERING,
            ResidentRecoveryState.POLICY_STABLE,
        }:
            if not observation.policy_writer_active:
                return self._fail(observation, "policy_writer_lost_during_recovery")
            stable = self._stable(observation)
            if self.state is ResidentRecoveryState.POLICY_RECOVERING:
                if not stable:
                    return self._result(ResidentRecoveryState.POLICY_RECOVERING)
                self._stable_since_s = observation.now_s
                if self.config.policy_exit_hold_s <= 0.0:
                    self._policy_fresh_seen = bool(observation.lowcmd_fresh)
                    previous = self._transition(
                        ResidentRecoveryState.POLICY_PAUSE_REQUESTED,
                        observation.now_s,
                    )
                    return self._result(previous, request_policy_pause=True)
                previous = self._transition(
                    ResidentRecoveryState.POLICY_STABLE, observation.now_s
                )
                return self._result(previous)
            if not stable:
                self._stable_since_s = None
                previous = self._transition(
                    ResidentRecoveryState.POLICY_RECOVERING, observation.now_s
                )
                return self._result(previous)
            assert self._stable_since_s is not None
            if (
                observation.now_s - self._stable_since_s
                < self.config.policy_exit_hold_s
            ):
                return self._result(ResidentRecoveryState.POLICY_STABLE)
            self._policy_fresh_seen = bool(observation.lowcmd_fresh)
            previous = self._transition(
                ResidentRecoveryState.POLICY_PAUSE_REQUESTED,
                observation.now_s,
            )
            return self._result(previous, request_policy_pause=True)

        if self.state is ResidentRecoveryState.POLICY_PAUSE_REQUESTED:
            self._policy_fresh_seen |= bool(observation.lowcmd_fresh)
            if not observation.policy_writer_paused:
                return self._result(
                    ResidentRecoveryState.POLICY_PAUSE_REQUESTED
                )
            if self._policy_fresh_seen and not observation.lowcmd_fresh:
                self._policy_stale_confirmed = True
            previous = self._transition(
                ResidentRecoveryState.POLICY_QUIET, observation.now_s
            )
            return self._result(previous)

        if self.state is ResidentRecoveryState.POLICY_QUIET:
            if not observation.policy_writer_paused:
                return self._fail(observation, "policy_writer_revived_before_sonic")
            self._policy_fresh_seen |= bool(observation.lowcmd_fresh)
            if self._policy_fresh_seen and not observation.lowcmd_fresh:
                self._policy_stale_confirmed = True
            if not self._policy_stale_confirmed:
                return self._result(ResidentRecoveryState.POLICY_QUIET)
            previous = self._transition(
                ResidentRecoveryState.SONIC_RESUME_REQUESTED,
                observation.now_s,
            )
            return self._result(previous, resume_sonic_writer=True)

        if self.state is ResidentRecoveryState.SONIC_RESUME_REQUESTED:
            if observation.policy_writer_active:
                return self._fail(observation, "policy_active_during_sonic_resume")
            if not (
                observation.sonic_writer_active
                and observation.sonic_resume_first_write
                and observation.lowcmd_fresh
            ):
                return self._result(
                    ResidentRecoveryState.SONIC_RESUME_REQUESTED
                )
            self._sonic_stable_since_s = None
            previous = self._transition(
                ResidentRecoveryState.SONIC_STABILIZING,
                observation.now_s,
            )
            return self._result(previous)

        if self.state is ResidentRecoveryState.SONIC_STABILIZING:
            if not observation.sonic_writer_active or observation.policy_writer_active:
                return self._fail(observation, "resident_writer_authority_lost")
            if observation.fall_detected:
                if not observation.policy_writer_paused:
                    return self._fail(
                        observation,
                        "resident_recovery_policy_not_ready_for_sonic_refall",
                    )
                return self._begin_fall_episode(observation)
            if not self._stable(observation):
                self._sonic_stable_since_s = None
                return self._result(ResidentRecoveryState.SONIC_STABILIZING)
            if self._sonic_stable_since_s is None:
                self._sonic_stable_since_s = observation.now_s
                return self._result(ResidentRecoveryState.SONIC_STABILIZING)
            if observation.now_s - self._sonic_stable_since_s < self.config.stable_hold_s:
                return self._result(ResidentRecoveryState.SONIC_STABILIZING)
            previous = self._transition(
                ResidentRecoveryState.WAIT_NEUTRAL, observation.now_s
            )
            return self._result(previous)

        if self.state is ResidentRecoveryState.WAIT_NEUTRAL:
            if not observation.sonic_writer_active or observation.policy_writer_active:
                return self._fail(observation, "resident_writer_authority_lost")
            if observation.fall_detected:
                if not observation.policy_writer_paused:
                    return self._fail(
                        observation,
                        "resident_recovery_policy_not_ready_for_sonic_refall",
                    )
                return self._begin_fall_episode(observation)
            if not self._stable(observation):
                return self._result(ResidentRecoveryState.WAIT_NEUTRAL)
            if not observation.neutral_confirmed:
                return self._result(ResidentRecoveryState.WAIT_NEUTRAL)
            self._episode_started_s = None
            previous = self._transition(
                ResidentRecoveryState.GAME_SONIC, observation.now_s
            )
            return self._result(
                previous,
                inhibit_game_input=False,
                resume_game=True,
            )

        return self._fail(observation, "unknown_resident_recovery_state")


__all__ = (
    "RecoveryConfig",
    "RecoveryInput",
    "RecoveryOutput",
    "RecoveryState",
    "ResidentPolicyRecoveryFSM",
    "ResidentRecoveryInput",
    "ResidentRecoveryOutput",
    "ResidentRecoveryState",
    "SingleWriterRecoveryFSM",
)
