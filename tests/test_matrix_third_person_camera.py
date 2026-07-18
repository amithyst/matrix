from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import sys
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts/matrix_third_person_camera.py"
SPEC = importlib.util.spec_from_file_location("matrix_third_person_camera", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def all_capabilities():
    return MODULE.CameraBridgeCapabilities(
        authoritative_robot_pivot=True,
        final_view_readback=True,
        orbit_control=True,
        sphere_sweep=True,
        input_mode_readback=True,
        relative_pose_handoff=True,
        relative_lock_control=True,
    )


def clear_sweep(_start, _end, _radius):
    return MODULE.SphereSweepHit(blocking=False)


SESSION_ID = "matrix-camera-reference"


def native_frame(
    *,
    mode=MODULE.MODE_NATIVE_FOLLOW,
    session_id=SESSION_ID,
    sequence=10,
    produced_monotonic_ns=1_000_000_000,
    applied_request_id=None,
    render_frame_id=9000,
    input_captured=False,
    robot_pivot=None,
    camera_position=None,
    look_at=None,
):
    pivot = robot_pivot or MODULE.Vec3(1.0, 2.0, 1.0)
    camera = camera_position or MODULE.Vec3(-2.0, 2.0, 2.0)
    target = look_at or MODULE.Vec3(4.0, 3.0, 1.5)
    return MODULE.CameraBridgeFrame(
        session_id=session_id,
        sequence=sequence,
        produced_monotonic_ns=produced_monotonic_ns,
        applied_request_id=applied_request_id,
        render_frame_id=render_frame_id,
        mode=mode,
        robot_pivot=pivot,
        camera_position=camera,
        look_at=target,
        input_captured=input_captured,
    )


def free_frame(
    *,
    session_id=SESSION_ID,
    sequence=12,
    produced_monotonic_ns=1_000_000_000,
    applied_request_id=None,
    render_frame_id=9001,
    input_captured=False,
    robot_pivot=None,
    camera_position=None,
    look_at=None,
):
    return native_frame(
        mode=MODULE.MODE_NATIVE_FREE,
        session_id=session_id,
        sequence=sequence,
        produced_monotonic_ns=produced_monotonic_ns,
        applied_request_id=applied_request_id,
        render_frame_id=render_frame_id,
        input_captured=input_captured,
        robot_pivot=robot_pivot,
        camera_position=camera_position,
        look_at=look_at,
    )


def orbit_frame(
    *,
    session_id=SESSION_ID,
    sequence=13,
    produced_monotonic_ns=1_030_000_000,
    applied_request_id=1,
    render_frame_id=9002,
    robot_pivot=None,
    camera_position=None,
    desired_arm_m=None,
):
    pivot = robot_pivot or MODULE.Vec3(1.0, 2.0, 1.0)
    camera = camera_position or MODULE.Vec3(-2.0, 2.0, 2.0)
    actual_arm = (camera - pivot).length
    desired_arm = actual_arm if desired_arm_m is None else desired_arm_m
    return MODULE.CameraBridgeFrame(
        session_id=session_id,
        sequence=sequence,
        produced_monotonic_ns=produced_monotonic_ns,
        applied_request_id=applied_request_id,
        render_frame_id=render_frame_id,
        mode=MODULE.MODE_ORBIT_FOLLOW,
        robot_pivot=pivot,
        camera_position=camera,
        look_at=pivot,
        input_captured=False,
        desired_arm_m=desired_arm,
        actual_arm_m=actual_arm,
        collision_limited=actual_arm < desired_arm - 1e-9,
    )


class CapabilityAndModeTest(unittest.TestCase):
    def test_orbit_requires_every_authoritative_ue_capability(self) -> None:
        controller = MODULE.CameraModeController()
        unavailable = MODULE.CameraBridgeCapabilities(
            authoritative_robot_pivot=True,
            final_view_readback=True,
            orbit_control=True,
            sphere_sweep=False,
            input_mode_readback=True,
        )

        with self.assertRaisesRegex(
            MODULE.CameraModeUnavailable, "sphere_sweep"
        ):
            controller.select_follow(
                MODULE.MODE_ORBIT_FOLLOW,
                capabilities=unavailable,
                source_frame=native_frame(),
                now_monotonic_ns=1_020_000_000,
            )

        self.assertEqual(controller.mode, MODULE.MODE_NATIVE_FOLLOW)
        self.assertEqual(controller.transition_count, 0)

    def test_capability_mapping_is_an_exact_schema(self) -> None:
        value = {
            "protocol": MODULE.CAMERA_BRIDGE_PROTOCOL,
            "authoritative_robot_pivot": True,
            "final_view_readback": True,
            "orbit_control": True,
            "sphere_sweep": True,
            "input_mode_readback": True,
            "relative_pose_handoff": True,
            "relative_lock_control": True,
        }
        self.assertTrue(
            MODULE.CameraBridgeCapabilities.from_mapping(value).orbit_ready
        )
        value["unverified_shortcut"] = True
        with self.assertRaisesRegex(ValueError, "unknown"):
            MODULE.CameraBridgeCapabilities.from_mapping(value)

    def test_orbit_rejects_bridge_without_relative_pose_handoff(self) -> None:
        value = all_capabilities().__dict__ | {"relative_pose_handoff": False}
        capabilities = MODULE.CameraBridgeCapabilities(**value)

        with self.assertRaisesRegex(
            MODULE.CameraModeUnavailable, "relative_pose_handoff"
        ):
            capabilities.require_orbit()

    def test_initial_c_is_requested_then_confirmed_by_new_ue_frame(self) -> None:
        controller = MODULE.CameraModeController()
        capabilities = all_capabilities()
        source = native_frame()

        self.assertTrue(
            controller.on_orbit_toggle(
                capabilities=capabilities,
                source_frame=source,
                now_monotonic_ns=1_020_000_000,
            )
        )
        self.assertEqual(controller.mode, MODULE.MODE_NATIVE_FOLLOW)
        self.assertEqual(controller.pending_mode, MODULE.MODE_ORBIT_FOLLOW)
        self.assertEqual(controller.pending_request.request_id, 1)
        self.assertEqual(controller.transition_count, 0)
        self.assertTrue(controller.neutral_rearm_required)
        with self.assertRaisesRegex(MODULE.CameraModeUnavailable, "handoff"):
            controller.acknowledge_neutral_rearm()

        self.assertTrue(
            controller.complete_orbit_handoff(
                capabilities=capabilities,
                orbit_frame=orbit_frame(applied_request_id=1),
                now_monotonic_ns=1_040_000_000,
            )
        )
        self.assertEqual(controller.mode, MODULE.MODE_ORBIT_FOLLOW)
        self.assertIsNone(controller.pending_mode)
        self.assertEqual(controller.transition_count, 1)

    def test_select_follow_and_v_both_wait_for_confirmed_render_frame(self) -> None:
        controller = MODULE.CameraModeController()
        capabilities = all_capabilities()
        source = native_frame()
        self.assertTrue(
            controller.select_follow(
                MODULE.MODE_ORBIT_FOLLOW,
                capabilities=capabilities,
                source_frame=source,
                now_monotonic_ns=1_020_000_000,
            )
        )
        self.assertEqual(controller.mode, MODULE.MODE_NATIVE_FOLLOW)
        controller.complete_orbit_handoff(
            capabilities=capabilities,
            orbit_frame=orbit_frame(applied_request_id=1),
            now_monotonic_ns=1_040_000_000,
        )
        self.assertEqual(controller.mode, MODULE.MODE_ORBIT_FOLLOW)

        source_orbit = orbit_frame(
            sequence=14,
            produced_monotonic_ns=1_050_000_000,
            applied_request_id=1,
            render_frame_id=9003,
        )
        self.assertTrue(
            controller.on_v_edge(
                capabilities=capabilities,
                source_frame=source_orbit,
                now_monotonic_ns=1_060_000_000,
            )
        )
        self.assertEqual(controller.mode, MODULE.MODE_ORBIT_FOLLOW)
        self.assertEqual(controller.pending_mode, MODULE.MODE_NATIVE_FREE)
        confirmed_free = free_frame(
            sequence=15,
            produced_monotonic_ns=1_070_000_000,
            applied_request_id=2,
            render_frame_id=9004,
        )
        controller.complete_mode_transition(
            capabilities=capabilities,
            confirmed_frame=confirmed_free,
            now_monotonic_ns=1_080_000_000,
        )
        self.assertEqual(controller.mode, MODULE.MODE_NATIVE_FREE)
        self.assertEqual(controller.last_follow_mode, MODULE.MODE_ORBIT_FOLLOW)
        source_free = free_frame(
            sequence=16,
            produced_monotonic_ns=1_090_000_000,
            applied_request_id=2,
            render_frame_id=9005,
        )
        # C cannot bypass the V/free handoff policy.
        self.assertFalse(
            controller.on_orbit_toggle(
                capabilities=capabilities,
                source_frame=source_free,
                now_monotonic_ns=1_100_000_000,
            )
        )
        self.assertTrue(
            controller.on_v_edge(
                capabilities=capabilities,
                source_frame=source_free,
                now_monotonic_ns=1_100_000_000,
            )
        )
        self.assertEqual(controller.mode, MODULE.MODE_NATIVE_FREE)
        self.assertEqual(controller.pending_mode, MODULE.MODE_ORBIT_FOLLOW)
        controller.complete_orbit_handoff(
            capabilities=capabilities,
            orbit_frame=orbit_frame(
                sequence=17,
                produced_monotonic_ns=1_110_000_000,
                applied_request_id=3,
                render_frame_id=9006,
            ),
            now_monotonic_ns=1_120_000_000,
        )
        self.assertEqual(controller.mode, MODULE.MODE_ORBIT_FOLLOW)

    def test_relative_lock_c_to_orbit_also_waits_for_new_frame(self) -> None:
        controller = MODULE.CameraModeController()
        capabilities = all_capabilities()
        source = native_frame()
        controller.select_follow(
            MODULE.MODE_RELATIVE_LOCK,
            capabilities=capabilities,
            source_frame=source,
            now_monotonic_ns=1_020_000_000,
        )
        relative_request = controller.pending_request
        assert relative_request is not None
        relative_state = MODULE.RelativeLockController(relative_request).step(
            robot_pivot=source.robot_pivot
        )
        confirmed_relative = MODULE.CameraBridgeFrame(
            session_id=SESSION_ID,
            sequence=11,
            produced_monotonic_ns=1_030_000_000,
            applied_request_id=1,
            render_frame_id=9001,
            mode=MODULE.MODE_RELATIVE_LOCK,
            robot_pivot=relative_state.pivot,
            camera_position=relative_state.camera_position,
            look_at=relative_state.look_at,
            input_captured=False,
        )
        controller.complete_mode_transition(
            capabilities=capabilities,
            confirmed_frame=confirmed_relative,
            now_monotonic_ns=1_040_000_000,
        )
        source_relative = MODULE.CameraBridgeFrame(
            session_id=SESSION_ID,
            sequence=12,
            produced_monotonic_ns=1_050_000_000,
            applied_request_id=1,
            render_frame_id=9002,
            mode=MODULE.MODE_RELATIVE_LOCK,
            robot_pivot=relative_state.pivot,
            camera_position=relative_state.camera_position,
            look_at=relative_state.look_at,
            input_captured=False,
        )

        controller.on_orbit_toggle(
            capabilities=capabilities,
            source_frame=source_relative,
            now_monotonic_ns=1_060_000_000,
        )
        self.assertEqual(controller.mode, MODULE.MODE_RELATIVE_LOCK)
        self.assertEqual(controller.pending_mode, MODULE.MODE_ORBIT_FOLLOW)
        controller.complete_orbit_handoff(
            capabilities=capabilities,
            orbit_frame=orbit_frame(
                sequence=13,
                produced_monotonic_ns=1_070_000_000,
                applied_request_id=2,
                render_frame_id=9003,
            ),
            now_monotonic_ns=1_080_000_000,
        )
        self.assertEqual(controller.mode, MODULE.MODE_ORBIT_FOLLOW)

    def test_mode_request_rejects_stale_or_replayed_source(self) -> None:
        controller = MODULE.CameraModeController()
        capabilities = all_capabilities()
        with self.assertRaisesRegex(MODULE.CameraModeUnavailable, "stale"):
            controller.on_v_edge(
                capabilities=capabilities,
                source_frame=native_frame(),
                now_monotonic_ns=1_200_000_001,
            )
        source = native_frame()
        controller.on_v_edge(
            capabilities=capabilities,
            source_frame=source,
            now_monotonic_ns=1_020_000_000,
        )
        confirmed = free_frame(
            sequence=11,
            produced_monotonic_ns=1_030_000_000,
            applied_request_id=1,
            render_frame_id=9001,
        )
        controller.complete_mode_transition(
            capabilities=capabilities,
            confirmed_frame=confirmed,
            now_monotonic_ns=1_040_000_000,
        )
        self.assertEqual(controller.mode, MODULE.MODE_NATIVE_FREE)
        with self.assertRaisesRegex(MODULE.CameraModeUnavailable, "consumed"):
            controller.request_mode(
                MODULE.MODE_NATIVE_FOLLOW,
                capabilities=capabilities,
                source_frame=confirmed,
                now_monotonic_ns=1_040_000_000,
            )

    def test_completion_is_bound_to_request_time_session_and_pose(self) -> None:
        controller = MODULE.CameraModeController()
        capabilities = all_capabilities()
        source = native_frame()
        controller.on_orbit_toggle(
            capabilities=capabilities,
            source_frame=source,
            now_monotonic_ns=1_020_000_000,
        )

        cases = (
            (
                "session mismatch",
                orbit_frame(session_id="other-session", applied_request_id=1),
            ),
            (
                "request id mismatch",
                orbit_frame(applied_request_id=99),
            ),
            (
                "timestamp",
                orbit_frame(
                    produced_monotonic_ns=source.produced_monotonic_ns,
                    applied_request_id=1,
                ),
            ),
            (
                "follow its request",
                orbit_frame(
                    produced_monotonic_ns=1_010_000_000,
                    applied_request_id=1,
                ),
            ),
            (
                "unrelated",
                orbit_frame(
                    applied_request_id=1,
                    robot_pivot=MODULE.Vec3(1001.0, 2.0, 1.0),
                    camera_position=MODULE.Vec3(998.0, 2.0, 2.0),
                ),
            ),
            (
                "desired arm",
                orbit_frame(applied_request_id=1, desired_arm_m=4.0),
            ),
            (
                "direction",
                orbit_frame(
                    applied_request_id=1,
                    camera_position=MODULE.Vec3(1.0, -1.0, 2.0),
                ),
            ),
        )
        for message, frame in cases:
            with self.subTest(message=message):
                with self.assertRaisesRegex(MODULE.CameraModeUnavailable, message):
                    controller.complete_orbit_handoff(
                        capabilities=capabilities,
                        orbit_frame=frame,
                        now_monotonic_ns=1_040_000_000,
                    )
                self.assertEqual(controller.mode, MODULE.MODE_NATIVE_FOLLOW)
                self.assertEqual(controller.pending_mode, MODULE.MODE_ORBIT_FOLLOW)

        controller.complete_orbit_handoff(
            capabilities=capabilities,
            orbit_frame=orbit_frame(applied_request_id=1),
            now_monotonic_ns=1_040_000_000,
        )
        self.assertEqual(controller.mode, MODULE.MODE_ORBIT_FOLLOW)

    def test_pending_request_timeout_cancels_without_changing_mode(self) -> None:
        controller = MODULE.CameraModeController()
        controller.on_orbit_toggle(
            capabilities=all_capabilities(),
            source_frame=native_frame(),
            now_monotonic_ns=1_020_000_000,
        )
        request = controller.pending_request
        assert request is not None
        self.assertFalse(
            controller.expire_pending_request(
                now_monotonic_ns=1_520_000_000
            )
        )
        self.assertTrue(
            controller.expire_pending_request(
                now_monotonic_ns=1_520_000_001
            )
        )
        self.assertFalse(request.active)
        self.assertIsNone(controller.pending_request)
        self.assertEqual(controller.mode, MODULE.MODE_NATIVE_FOLLOW)
        self.assertTrue(controller.neutral_rearm_required)

    def test_capability_loss_disarms_without_faking_confirmed_mode(self) -> None:
        controller = MODULE.CameraModeController()
        capabilities = all_capabilities()
        controller.on_orbit_toggle(
            capabilities=capabilities,
            source_frame=native_frame(),
            now_monotonic_ns=1_020_000_000,
        )
        self.assertTrue(
            controller.reconcile_capabilities(MODULE.CameraBridgeCapabilities())
        )
        self.assertEqual(controller.mode, MODULE.MODE_NATIVE_FOLLOW)
        self.assertIsNone(controller.pending_mode)

        controller.on_orbit_toggle(
            capabilities=capabilities,
            source_frame=native_frame(
                sequence=11,
                produced_monotonic_ns=1_030_000_000,
                render_frame_id=9001,
            ),
            now_monotonic_ns=1_040_000_000,
        )
        controller.complete_orbit_handoff(
            capabilities=capabilities,
            orbit_frame=orbit_frame(
                sequence=12,
                produced_monotonic_ns=1_050_000_000,
                applied_request_id=2,
                render_frame_id=9002,
            ),
            now_monotonic_ns=1_060_000_000,
        )
        self.assertTrue(
            controller.reconcile_capabilities(MODULE.CameraBridgeCapabilities())
        )
        self.assertEqual(controller.mode, MODULE.MODE_ORBIT_FOLLOW)
        self.assertTrue(controller.bridge_faulted)
        with self.assertRaisesRegex(MODULE.CameraModeUnavailable, "fault"):
            controller.acknowledge_neutral_rearm()


class BridgeFrameAndHandoffTest(unittest.TestCase):
    def test_legacy_relative_lock_and_orbit_are_distinct_modes(self) -> None:
        self.assertEqual(
            MODULE.CAMERA_MODES,
            frozenset(
                (
                    MODULE.MODE_NATIVE_FOLLOW,
                    MODULE.MODE_NATIVE_FREE,
                    MODULE.MODE_RELATIVE_LOCK,
                    MODULE.MODE_ORBIT_FOLLOW,
                )
            ),
        )
        self.assertNotEqual(
            MODULE.MODE_NATIVE_FOLLOW, MODULE.MODE_RELATIVE_LOCK
        )

    def test_relative_lock_is_confirmed_and_translates_both_offsets(self) -> None:
        mode_controller = MODULE.CameraModeController()
        mode_controller.on_v_edge(
            capabilities=all_capabilities(),
            source_frame=native_frame(),
            now_monotonic_ns=1_020_000_000,
        )
        mode_controller.complete_mode_transition(
            capabilities=all_capabilities(),
            confirmed_frame=free_frame(
                sequence=11,
                produced_monotonic_ns=1_030_000_000,
                applied_request_id=1,
                render_frame_id=9001,
            ),
            now_monotonic_ns=1_040_000_000,
        )
        source = free_frame(
            sequence=12,
            produced_monotonic_ns=1_050_000_000,
            applied_request_id=1,
            render_frame_id=9002,
        )
        self.assertTrue(
            mode_controller.select_follow(
                MODULE.MODE_RELATIVE_LOCK,
                capabilities=all_capabilities(),
                source_frame=source,
                now_monotonic_ns=1_060_000_000,
            )
        )
        self.assertEqual(mode_controller.mode, MODULE.MODE_NATIVE_FREE)
        request = mode_controller.pending_request
        assert request is not None
        relative = MODULE.RelativeLockController(request)
        first_pivot = MODULE.Vec3(1.1, 2.0, 1.0)
        first = relative.step(robot_pivot=first_pivot)
        confirmed = MODULE.CameraBridgeFrame(
            session_id=SESSION_ID,
            sequence=13,
            produced_monotonic_ns=1_070_000_000,
            applied_request_id=2,
            render_frame_id=9003,
            mode=MODULE.MODE_RELATIVE_LOCK,
            robot_pivot=first.pivot,
            camera_position=first.camera_position,
            look_at=first.look_at,
            input_captured=False,
        )
        mode_controller.complete_mode_transition(
            capabilities=all_capabilities(),
            confirmed_frame=confirmed,
            now_monotonic_ns=1_080_000_000,
        )
        self.assertEqual(mode_controller.mode, MODULE.MODE_RELATIVE_LOCK)

        second_pivot = MODULE.Vec3(4.0, -3.0, 1.5)
        second = relative.step(robot_pivot=second_pivot)
        pivot_delta = second_pivot - first_pivot
        self.assertEqual(
            second.camera_position, first.camera_position + pivot_delta
        )
        self.assertEqual(second.look_at, first.look_at + pivot_delta)
        self.assertEqual(
            second.camera_position - second.pivot,
            first.camera_position - first.pivot,
        )

    def test_relative_lock_completion_rejects_wrong_captured_offset(self) -> None:
        source = native_frame()
        mode_controller = MODULE.CameraModeController()
        mode_controller.select_follow(
            MODULE.MODE_RELATIVE_LOCK,
            capabilities=all_capabilities(),
            source_frame=source,
            now_monotonic_ns=1_020_000_000,
        )
        wrong = MODULE.CameraBridgeFrame(
            session_id=SESSION_ID,
            sequence=11,
            produced_monotonic_ns=1_030_000_000,
            applied_request_id=1,
            render_frame_id=9001,
            mode=MODULE.MODE_RELATIVE_LOCK,
            robot_pivot=source.robot_pivot,
            camera_position=source.camera_position + MODULE.Vec3(0.1, 0.0, 0.0),
            look_at=source.look_at,
            input_captured=False,
        )
        with self.assertRaisesRegex(MODULE.CameraModeUnavailable, "camera offset"):
            mode_controller.complete_mode_transition(
                capabilities=all_capabilities(),
                confirmed_frame=wrong,
                now_monotonic_ns=1_040_000_000,
            )
        self.assertEqual(mode_controller.mode, MODULE.MODE_NATIVE_FOLLOW)

    def test_bridge_frame_mapping_is_exact_and_mode_dependent(self) -> None:
        value = {
            "protocol": MODULE.CAMERA_BRIDGE_PROTOCOL,
            "session_id": SESSION_ID,
            "sequence": 5,
            "produced_monotonic_ns": 1000,
            "applied_request_id": 7,
            "render_frame_id": 44,
            "mode": MODULE.MODE_ORBIT_FOLLOW,
            "robot_pivot_m": [1.0, 2.0, 3.0],
            "camera_position_m": [-1.0, 2.0, 3.0],
            "look_at_m": [1.0, 2.0, 3.0],
            "input_captured": False,
            "desired_arm_m": 3.0,
            "actual_arm_m": 2.0,
            "collision_limited": True,
        }
        frame = MODULE.CameraBridgeFrame.from_mapping(value)
        self.assertEqual(frame.robot_pivot, MODULE.Vec3(1.0, 2.0, 3.0))
        self.assertEqual(frame.render_frame_id, 44)

        value["unverified_pose"] = [0.0, 0.0, 0.0]
        with self.assertRaisesRegex(ValueError, "unknown"):
            MODULE.CameraBridgeFrame.from_mapping(value)

    def test_native_frame_cannot_claim_orbit_collision_state(self) -> None:
        with self.assertRaisesRegex(ValueError, "must not claim"):
            MODULE.CameraBridgeFrame(
                session_id=SESSION_ID,
                sequence=1,
                produced_monotonic_ns=1000,
                applied_request_id=None,
                render_frame_id=2,
                mode=MODULE.MODE_NATIVE_FREE,
                robot_pivot=MODULE.Vec3(0.0, 0.0, 1.0),
                camera_position=MODULE.Vec3(-2.0, 0.0, 1.0),
                look_at=MODULE.Vec3(0.0, 0.0, 1.0),
                input_captured=False,
                desired_arm_m=2.0,
                actual_arm_m=2.0,
                collision_limited=False,
            )

    def test_orbit_frame_rejects_false_camera_geometry(self) -> None:
        valid = {
            "session_id": SESSION_ID,
            "sequence": 3,
            "produced_monotonic_ns": 1000,
            "applied_request_id": 1,
            "render_frame_id": 4,
            "mode": MODULE.MODE_ORBIT_FOLLOW,
            "robot_pivot": MODULE.Vec3(0.0, 0.0, 1.0),
            "camera_position": MODULE.Vec3(-2.0, 0.0, 1.0),
            "look_at": MODULE.Vec3(0.0, 0.0, 1.0),
            "input_captured": False,
            "desired_arm_m": 3.0,
            "actual_arm_m": 2.0,
            "collision_limited": True,
        }
        with self.assertRaisesRegex(ValueError, "camera distance"):
            MODULE.CameraBridgeFrame(
                **(valid | {"camera_position": MODULE.Vec3(-1.0, 0.0, 1.0)})
            )
        with self.assertRaisesRegex(ValueError, "look_at"):
            MODULE.CameraBridgeFrame(
                **(valid | {"look_at": MODULE.Vec3(0.0, 0.02, 1.0)})
            )

    def test_camera_frame_freshness_rejects_stale_and_future_values(self) -> None:
        frame = free_frame(produced_monotonic_ns=1_000_000_000)
        frame.require_fresh(now_monotonic_ns=1_100_000_000)
        with self.assertRaisesRegex(MODULE.CameraModeUnavailable, "stale"):
            frame.require_fresh(now_monotonic_ns=1_100_000_001)
        with self.assertRaisesRegex(MODULE.CameraModeUnavailable, "future"):
            frame.require_fresh(now_monotonic_ns=999_999_999)

    def test_free_pose_is_stored_relative_to_robot_and_translates_exactly(self) -> None:
        source = free_frame()
        pose = MODULE.RelativeCameraPose.capture(source)
        next_pivot = MODULE.Vec3(4.0, -3.0, 1.5)
        camera, look_at = pose.translated_view(next_pivot)
        pivot_delta = next_pivot - source.robot_pivot

        self.assertEqual(camera, source.camera_position + pivot_delta)
        self.assertEqual(look_at, source.look_at + pivot_delta)
        self.assertEqual(camera - next_pivot, pose.camera_offset)
        self.assertEqual(look_at - next_pivot, pose.look_at_offset)

    def test_orbit_relock_preserves_position_then_centers_robot(self) -> None:
        source = free_frame()
        config = MODULE.OrbitCameraConfig(
            pivot_height_m=1.0,
            desired_arm_m=4.0,
            maximum_desired_arm_m=8.0,
            minimum_pitch_rad=math.radians(-30.0),
            maximum_pitch_rad=math.radians(60.0),
            initial_pitch_rad=0.0,
        )
        controller = MODULE.OrbitCameraController(config)
        locked = controller.relock_from_free(
            frame=source, dt_s=0.02, sphere_sweep=clear_sweep
        )

        self.assertEqual(locked.camera_position, source.camera_position)
        self.assertEqual(locked.look_at, source.robot_pivot)
        self.assertAlmostEqual(locked.desired_arm_m, math.sqrt(10.0))

        moved = controller.step(
            robot_position=MODULE.Vec3(4.0, -3.0, 0.5),
            yaw_delta_rad=0.0,
            pitch_delta_rad=0.0,
            dt_s=0.02,
            sphere_sweep=clear_sweep,
        )
        pivot_delta = moved.pivot - locked.pivot
        self.assertEqual(moved.camera_position, locked.camera_position + pivot_delta)
        self.assertEqual(moved.offset, locked.offset)
        self.assertEqual(moved.look_at, moved.pivot)

    def test_relock_requires_released_input_and_is_transactional(self) -> None:
        controller = MODULE.OrbitCameraController()
        with self.assertRaisesRegex(MODULE.CameraModeUnavailable, "released"):
            controller.relock_from_free(
                frame=free_frame(input_captured=True),
                dt_s=0.02,
                sphere_sweep=clear_sweep,
            )
        self.assertIsNone(controller.state)

    def test_look_at_handoff_preserves_view_then_smoothly_centers(self) -> None:
        source = native_frame()
        mode_controller = MODULE.CameraModeController()
        mode_controller.request_mode(
            MODULE.MODE_ORBIT_FOLLOW,
            capabilities=all_capabilities(),
            source_frame=source,
            now_monotonic_ns=1_020_000_000,
        )
        request = mode_controller.pending_request
        self.assertIsNotNone(request)
        assert request is not None and request.relative_pose is not None
        pose = request.relative_pose
        target = MODULE.OrbitCameraController(
            MODULE.OrbitCameraConfig(
                pivot_height_m=1.0,
                minimum_pitch_rad=math.radians(-30.0),
                maximum_pitch_rad=math.radians(60.0),
            )
        ).relock_from_request(
            request=request,
            dt_s=0.0,
            sphere_sweep=clear_sweep,
        )
        handoff = MODULE.LookAtHandoffController(request, duration_s=0.20)

        initial = handoff.step(target_orbit=target, dt_s=0.0)
        halfway = handoff.step(target_orbit=target, dt_s=0.10)
        complete = handoff.step(target_orbit=target, dt_s=0.10)

        self.assertEqual(initial.camera_position, source.camera_position)
        self.assertEqual(initial.look_at, source.look_at)
        self.assertFalse(initial.complete)
        self.assertAlmostEqual(halfway.progress, 0.5)
        self.assertGreater(
            (halfway.look_at - halfway.camera_position).length, 0.5
        )
        self.assertNotEqual(halfway.look_at, initial.look_at)
        self.assertNotEqual(halfway.look_at, target.pivot)
        self.assertTrue(complete.complete)
        self.assertEqual(complete.look_at, target.pivot)

    def test_look_at_slerp_handles_antipodal_directions_without_zero(self) -> None:
        source = native_frame(
            robot_pivot=MODULE.Vec3(0.0, 0.0, 1.0),
            camera_position=MODULE.Vec3(-2.0, 0.0, 1.0),
            look_at=MODULE.Vec3(-3.0, 0.0, 1.0),
        )
        mode_controller = MODULE.CameraModeController()
        mode_controller.request_mode(
            MODULE.MODE_ORBIT_FOLLOW,
            capabilities=all_capabilities(),
            source_frame=source,
            now_monotonic_ns=1_020_000_000,
        )
        request = mode_controller.pending_request
        assert request is not None
        target = MODULE.OrbitCameraController(
            MODULE.OrbitCameraConfig(
                pivot_height_m=1.0,
                desired_arm_m=2.0,
                initial_pitch_rad=0.0,
            )
        ).relock_from_request(
            request=request,
            dt_s=0.0,
            sphere_sweep=clear_sweep,
        )
        handoff = MODULE.LookAtHandoffController(request, duration_s=0.20)
        halfway = handoff.step(target_orbit=target, dt_s=0.10)
        direction = halfway.look_at - halfway.camera_position

        self.assertAlmostEqual(halfway.progress, 0.5)
        self.assertGreater(direction.length, 0.5)
        self.assertTrue(all(math.isfinite(v) for v in direction.as_list()))

    def test_look_at_handoff_rejects_arbitrary_or_wrong_request_state(self) -> None:
        source = native_frame()
        mode_controller = MODULE.CameraModeController()
        mode_controller.request_mode(
            MODULE.MODE_ORBIT_FOLLOW,
            capabilities=all_capabilities(),
            source_frame=source,
            now_monotonic_ns=1_020_000_000,
        )
        request = mode_controller.pending_request
        assert request is not None
        handoff = MODULE.LookAtHandoffController(request)
        valid_target = MODULE.OrbitCameraController().relock_from_request(
            request=request,
            dt_s=0.0,
            sphere_sweep=clear_sweep,
        )
        arbitrary = MODULE.OrbitCameraController().step(
            robot_position=MODULE.Vec3(1.0, 2.0, -0.15),
            yaw_delta_rad=0.0,
            pitch_delta_rad=0.0,
            dt_s=0.0,
            sphere_sweep=clear_sweep,
        )
        with self.assertRaisesRegex(MODULE.CameraModeUnavailable, "not derived"):
            handoff.step(target_orbit=arbitrary, dt_s=0.0)
        self.assertIsNone(handoff.state)

        other_controller = MODULE.CameraModeController()
        other_controller.request_mode(
            MODULE.MODE_ORBIT_FOLLOW,
            capabilities=all_capabilities(),
            source_frame=source,
            now_monotonic_ns=1_020_000_000,
        )
        other_request = other_controller.pending_request
        assert other_request is not None
        wrong_target = MODULE.OrbitCameraController().relock_from_request(
            request=other_request,
            dt_s=0.0,
            sphere_sweep=clear_sweep,
        )
        with self.assertRaisesRegex(MODULE.CameraModeUnavailable, "not derived"):
            handoff.step(target_orbit=wrong_target, dt_s=0.0)

        mode_controller.reconcile_capabilities(MODULE.CameraBridgeCapabilities())
        with self.assertRaisesRegex(MODULE.CameraModeUnavailable, "no longer"):
            handoff.step(target_orbit=valid_target, dt_s=0.0)


class OrbitGeometryTest(unittest.TestCase):
    def setUp(self) -> None:
        self.config = MODULE.OrbitCameraConfig(
            pivot_height_m=1.0,
            desired_arm_m=4.0,
            probe_radius_m=0.25,
            collision_padding_m=0.10,
            recovery_speed_mps=2.0,
            minimum_pitch_rad=math.radians(-30.0),
            maximum_pitch_rad=math.radians(60.0),
            initial_pitch_rad=0.0,
            maximum_step_s=0.25,
        )

    def test_robot_translation_keeps_exact_pivot_and_relative_offset(self) -> None:
        controller = MODULE.OrbitCameraController(self.config)
        first = controller.step(
            robot_position=MODULE.Vec3(1.0, 2.0, 0.0),
            yaw_delta_rad=0.0,
            pitch_delta_rad=0.0,
            dt_s=0.02,
            sphere_sweep=clear_sweep,
        )
        second = controller.step(
            robot_position=MODULE.Vec3(4.0, -3.0, 0.5),
            yaw_delta_rad=0.0,
            pitch_delta_rad=0.0,
            dt_s=0.02,
            sphere_sweep=clear_sweep,
        )

        self.assertEqual(first.pivot, MODULE.Vec3(1.0, 2.0, 1.0))
        self.assertEqual(second.pivot, MODULE.Vec3(4.0, -3.0, 1.5))
        self.assertEqual(first.offset, second.offset)
        self.assertEqual(second.look_at, second.pivot)

    def test_yaw_orbits_about_robot_and_pitch_is_clamped(self) -> None:
        controller = MODULE.OrbitCameraController(self.config)
        state = controller.step(
            robot_position=MODULE.Vec3(0.0, 0.0, 0.0),
            yaw_delta_rad=math.pi / 2.0,
            pitch_delta_rad=math.pi,
            dt_s=0.02,
            sphere_sweep=clear_sweep,
        )

        self.assertAlmostEqual(state.yaw_rad, math.pi / 2.0)
        self.assertAlmostEqual(state.pitch_rad, math.radians(60.0))
        self.assertAlmostEqual(state.offset.length, 4.0)
        self.assertAlmostEqual(state.offset.x, 0.0, places=9)
        self.assertLess(state.offset.y, 0.0)
        self.assertGreater(state.offset.z, 0.0)

    def test_new_wall_hit_contracts_immediately_before_the_hit(self) -> None:
        controller = MODULE.OrbitCameraController(self.config)

        def wall(_start, _end, radius):
            self.assertEqual(radius, 0.25)
            return MODULE.SphereSweepHit(blocking=True, distance_m=1.50)

        state = controller.step(
            robot_position=MODULE.Vec3(0.0, 0.0, 0.0),
            yaw_delta_rad=0.0,
            pitch_delta_rad=0.0,
            dt_s=0.02,
            sphere_sweep=wall,
        )

        self.assertTrue(state.collision_limited)
        self.assertAlmostEqual(state.actual_arm_m, 1.40)
        self.assertAlmostEqual(state.offset.length, 1.40)

    def test_low_orbit_sweeps_toward_floor_and_contracts_before_contact(self) -> None:
        controller = MODULE.OrbitCameraController(self.config)

        def floor(start, end, _radius):
            self.assertLess(end.z, start.z)
            return MODULE.SphereSweepHit(blocking=True, distance_m=1.10)

        state = controller.step(
            robot_position=MODULE.Vec3(0.0, 0.0, 0.0),
            yaw_delta_rad=0.0,
            pitch_delta_rad=math.radians(-30.0),
            dt_s=0.02,
            sphere_sweep=floor,
        )

        self.assertTrue(state.collision_limited)
        self.assertAlmostEqual(state.actual_arm_m, 1.0)
        self.assertGreater(state.camera_position.z, -1.0)

    def test_clearance_recovery_is_smooth_and_never_exceeds_verified_space(self) -> None:
        controller = MODULE.OrbitCameraController(self.config)
        blocked = lambda _start, _end, _radius: MODULE.SphereSweepHit(
            blocking=True, distance_m=1.10
        )
        first = controller.step(
            robot_position=MODULE.Vec3(0.0, 0.0, 0.0),
            yaw_delta_rad=0.0,
            pitch_delta_rad=0.0,
            dt_s=0.02,
            sphere_sweep=blocked,
        )
        recovered = controller.step(
            robot_position=MODULE.Vec3(0.0, 0.0, 0.0),
            yaw_delta_rad=0.0,
            pitch_delta_rad=0.0,
            dt_s=0.25,
            sphere_sweep=clear_sweep,
        )
        limited = controller.step(
            robot_position=MODULE.Vec3(0.0, 0.0, 0.0),
            yaw_delta_rad=0.0,
            pitch_delta_rad=0.0,
            dt_s=0.25,
            sphere_sweep=lambda _start, _end, _radius: MODULE.SphereSweepHit(
                blocking=True, distance_m=1.45
            ),
        )

        self.assertAlmostEqual(first.actual_arm_m, 1.0)
        self.assertAlmostEqual(recovered.actual_arm_m, 1.5)
        self.assertTrue(recovered.collision_limited)
        recovered_frame = MODULE.CameraBridgeFrame(
            session_id=SESSION_ID,
            sequence=20,
            produced_monotonic_ns=2_000_000_000,
            applied_request_id=1,
            render_frame_id=21,
            mode=MODULE.MODE_ORBIT_FOLLOW,
            robot_pivot=recovered.pivot,
            camera_position=recovered.camera_position,
            look_at=recovered.look_at,
            input_captured=False,
            desired_arm_m=recovered.desired_arm_m,
            actual_arm_m=recovered.actual_arm_m,
            collision_limited=recovered.collision_limited,
        )
        self.assertAlmostEqual(recovered_frame.actual_arm_m, 1.5)
        # The next recovery step would reach 2.0, but verified clearance is 1.35.
        self.assertAlmostEqual(limited.actual_arm_m, 1.35)

    def test_collision_failure_is_transactional_and_never_assumes_clear_space(self) -> None:
        controller = MODULE.OrbitCameraController(self.config)
        baseline = controller.step(
            robot_position=MODULE.Vec3(0.0, 0.0, 0.0),
            yaw_delta_rad=0.0,
            pitch_delta_rad=0.0,
            dt_s=0.02,
            sphere_sweep=clear_sweep,
        )

        with self.assertRaisesRegex(
            MODULE.CameraCollisionUnavailable, "invalid result"
        ):
            controller.step(
                robot_position=MODULE.Vec3(10.0, 0.0, 0.0),
                yaw_delta_rad=1.0,
                pitch_delta_rad=0.5,
                dt_s=0.02,
                sphere_sweep=lambda _start, _end, _radius: None,
            )

        self.assertIs(controller.state, baseline)

    def test_penetrating_or_too_close_pivot_fails_closed(self) -> None:
        controller = MODULE.OrbitCameraController(self.config)
        with self.assertRaisesRegex(
            MODULE.CameraCollisionUnavailable, "inside blocking"
        ):
            controller.step(
                robot_position=MODULE.Vec3(0.0, 0.0, 0.0),
                yaw_delta_rad=0.0,
                pitch_delta_rad=0.0,
                dt_s=0.02,
                sphere_sweep=lambda _start, _end, _radius: MODULE.SphereSweepHit(
                    blocking=True,
                    distance_m=0.0,
                    started_penetrating=True,
                ),
            )

        with self.assertRaisesRegex(
            MODULE.CameraCollisionUnavailable, "no operational"
        ):
            controller.step(
                robot_position=MODULE.Vec3(0.0, 0.0, 0.0),
                yaw_delta_rad=0.0,
                pitch_delta_rad=0.0,
                dt_s=0.02,
                sphere_sweep=lambda _start, _end, _radius: MODULE.SphereSweepHit(
                    blocking=True,
                    distance_m=0.12,
                ),
            )


if __name__ == "__main__":
    unittest.main()
