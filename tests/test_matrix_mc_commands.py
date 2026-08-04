import json
import math
from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import matrix_mc_commands as MODULE  # noqa: E402
import matrix_motion_settings as MOTION_SETTINGS  # noqa: E402
from matrix_world_state import MatrixWorldState, WorldPose  # noqa: E402


SESSION = "a" * 32
REQUEST_ID = "cmd-" + "b" * 32


class McCommandParserTest(unittest.TestCase):
    def test_parses_canonical_summon_with_relative_coordinates_and_tags(self) -> None:
        parsed = MODULE.parse_mc_command(
            '/summon matrix:teleport_point ~ ~1.5 -2 {Tags:["XX","home"]}'
        )

        self.assertIsInstance(parsed.command, MODULE.SummonTeleportPoint)
        self.assertEqual(parsed.command.tags, ("XX", "home"))
        self.assertEqual(
            parsed.command.coordinates,
            (
                MODULE.Coordinate(0.0, True),
                MODULE.Coordinate(1.5, True),
                MODULE.Coordinate(-2.0, False),
            ),
        )
        self.assertIsNone(parsed.warning)

    def test_exact_summom_alias_executes_with_warning(self) -> None:
        parsed = MODULE.parse_mc_command(
            '/summom matrix:teleport_point 1 2 3 {Tags:["XX"]}'
        )

        self.assertIsInstance(parsed.command, MODULE.SummonTeleportPoint)
        self.assertIn("/summon", parsed.warning or "")

    def test_other_misspellings_do_not_execute(self) -> None:
        for text in (
            '/sumon matrix:teleport_point 1 2 3 {Tags:["XX"]}',
            '/summonn matrix:teleport_point 1 2 3 {Tags:["XX"]}',
        ):
            with self.subTest(text=text), self.assertRaisesRegex(
                MODULE.CommandParseError, "did you mean /summon"
            ):
                MODULE.parse_mc_command(text)

    def test_parses_coordinate_and_selector_tp(self) -> None:
        coordinate = MODULE.parse_mc_command("/tp @s ~1 2.5 ~-3").command
        selector = MODULE.parse_mc_command(
            "/tp @s @e[type=matrix:teleport_point,tag=XX,limit=1,sort=nearest]"
        ).command

        self.assertIsInstance(coordinate, MODULE.TeleportCoordinates)
        self.assertIsInstance(selector, MODULE.TeleportSelector)
        self.assertEqual(selector.tag, "XX")

    def test_parses_pose_recover_mode_native_and_function_commands(self) -> None:
        pose = MODULE.parse_mc_command("/pose @s yaw ~90deg").command
        recover = MODULE.parse_mc_command("/recover").command
        movement = MODULE.parse_mc_command("/mode camera_strafe").command
        native = MODULE.parse_mc_command("/sonic mode 7").command
        auto = MODULE.parse_mc_command("/sonic mode auto").command
        pause = MODULE.parse_mc_command("/pause").command
        resume = MODULE.parse_mc_command("/continue").command
        world = MODULE.parse_mc_command("/world moon").command
        scene_alias = MODULE.parse_mc_command("/scene MoonWorld").command
        gait_threshold = MODULE.parse_mc_command(
            (
                "/data modify entity @s "
                "control.motion.gait_start_heading_error_rad "
                f"set value {math.radians(50.0):.10f}"
            )
        ).command
        file_function = MODULE.parse_mc_command("/function recover_here").command
        function = MODULE.parse_mc_command(
            "/function /tp @s ~1 ~ ~; /pose @s yaw 180deg"
        ).command

        self.assertEqual(pose, MODULE.PoseYawSet(MODULE.Angle(math.pi / 2.0, True)))
        self.assertIsInstance(recover, MODULE.RecoverHere)
        self.assertEqual(movement, MODULE.MovementModeSet("camera_strafe"))
        self.assertEqual(native, MODULE.NativeModeSet(7))
        self.assertEqual(auto, MODULE.NativeModeSet(None))
        self.assertEqual(pause, MODULE.RuntimePauseSet("paused", expected_epoch=None))
        self.assertEqual(resume, MODULE.RuntimePauseSet("running", expected_epoch=None))
        self.assertEqual(world, MODULE.WorldSceneSet("moon"))
        self.assertEqual(scene_alias, MODULE.WorldSceneSet("moon"))
        self.assertEqual(
            gait_threshold,
            MODULE.MotionSettingSet(
                MOTION_SETTINGS.GAIT_START_HEADING_ERROR_PATH,
                float(f"{math.radians(50.0):.10f}"),
            ),
        )
        self.assertEqual(file_function, MODULE.CommandFunctionCall("recover_here"))
        self.assertIsInstance(function, MODULE.CommandFunctionRun)
        self.assertEqual(len(function.commands), 2)

    def test_native_mode_manual_commands_exclude_auto_gait_family(self) -> None:
        self.assertEqual(
            MODULE.parse_mc_command("/sonic mode 4").command,
            MODULE.NativeModeSet(4),
        )
        self.assertEqual(
            MODULE.parse_mc_command("/sonic mode 19").command,
            MODULE.NativeModeSet(19),
        )
        for mode in range(4):
            with self.subTest(mode=mode), self.assertRaisesRegex(
                MODULE.CommandParseError, "use auto for modes 0-3"
            ):
                MODULE.parse_mc_command(f"/sonic mode {mode}")

    def test_data_modify_motion_setting_command_is_whitelisted_and_strict(self) -> None:
        command = MODULE.MotionSettingSet(
            MOTION_SETTINGS.GAIT_STOP_HEADING_ERROR_PATH,
            math.radians(70.0),
        )
        self.assertEqual(
            MODULE.command_from_mapping(MODULE.command_to_mapping(command)),
            command,
        )

        for text in (
            "/data modify entity @s control.motion.unknown set value 1.0",
            "/data modify entity @s control.motion.gait_stop_heading_error_rad set value true",
            "/data modify entity @s control.motion.gait_stop_heading_error_rad set value nan",
            "/data modify entity @s control.motion.gait_stop_heading_error_rad set value 1e999",
        ):
            with self.subTest(text=text), self.assertRaises(MODULE.CommandParseError):
                MODULE.parse_mc_command(text)

        for payload in (
            {
                "name": "motion_setting_set",
                "path": "control.motion.unknown",
                "value": 1.0,
            },
            {
                "name": "motion_setting_set",
                "path": MOTION_SETTINGS.GAIT_STOP_HEADING_ERROR_PATH,
                "value": float("nan"),
            },
        ):
            with self.subTest(payload=payload), self.assertRaises(
                MODULE.CommandProtocolError
            ):
                MODULE.command_from_mapping(payload)

    def test_world_scene_command_is_whitelisted_and_strict(self) -> None:
        command = MODULE.WorldSceneSet("luna")
        self.assertEqual(command.destination_id, "moon")
        self.assertEqual(
            MODULE.command_from_mapping(MODULE.command_to_mapping(command)),
            MODULE.WorldSceneSet("moon"),
        )
        self.assertEqual(MODULE.world_scene_target("earth")["scene_id"], 2)
        self.assertEqual(MODULE.world_scene_target("moon")["scene_id"], 15)
        self.assertEqual(MODULE.world_scene_target("realscan")["scene_id"], 18)
        self.assertEqual(
            MODULE.world_scene_target("robot-training-ground")["map_name"],
            "/Game/Maps/RobotTrainingGround",
        )

        for text in ("/world mars", "/scene 999", "/planet ../../moon"):
            with self.subTest(text=text), self.assertRaises(MODULE.CommandParseError):
                MODULE.parse_mc_command(text)

        with self.assertRaises(MODULE.CommandProtocolError):
            MODULE.command_from_mapping(
                {"name": "world_scene_set", "destination_id": "mars"}
            )

    def test_selector_order_is_irrelevant_but_contract_is_strict(self) -> None:
        selector = MODULE.parse_mc_command(
            "/tp @s @e[tag=XX,sort=nearest,limit=1,type=matrix:teleport_point]"
        ).command
        self.assertEqual(selector, MODULE.TeleportSelector("XX"))

        invalid = (
            "/tp @s @e[type=matrix:teleport_point,tag=XX]",
            "/tp @s @e[type=matrix:teleport_point,tag=XX,limit=2]",
            "/tp @s @e[type=matrix:teleport_point,tag=XX,limit=1,limit=1]",
            "/tp @s @e[type=pig,tag=XX,limit=1]",
            "/tp @s @e[type=matrix:teleport_point,tag=XX,limit=1,x=3]",
        )
        for text in invalid:
            with self.subTest(text=text), self.assertRaises(MODULE.CommandParseError):
                MODULE.parse_mc_command(text)

    def test_rejects_local_nonfinite_control_and_oversized_input(self) -> None:
        invalid = (
            "/tp @s ^1 2 3",
            "/tp @s 1e999 2 3",
            "/tp @s 1 2\n3",
            "/tp @s 1 2",
            "/summon pig 1 2 3 {Tags:[\"XX\"]}",
            "/summon matrix:teleport_point 1 2 3 {Tags:[]}",
            "/" + "x" * 513,
        )
        for text in invalid:
            with self.subTest(text=text), self.assertRaises(MODULE.CommandParseError):
                MODULE.parse_mc_command(text)


class McCommandProtocolTest(unittest.TestCase):
    def test_request_round_trip_carries_typed_ast_not_command_text(self) -> None:
        command = MODULE.parse_mc_command("/tp @s ~1 2 ~-3").command
        request = MODULE.GameCommandRequest(
            session=SESSION,
            sequence=7,
            request_id=REQUEST_ID,
            command=command,
        )

        payload = MODULE.encode_command_request(request)

        self.assertNotIn(b"/tp", payload)
        self.assertEqual(MODULE.decode_command_request(payload), request)

    def test_protocol_rejects_unknown_duplicate_nan_and_oversized_packets(self) -> None:
        request = MODULE.GameCommandRequest(
            session=SESSION,
            sequence=1,
            request_id=REQUEST_ID,
            command=MODULE.parse_mc_command("/tp @s 1 2 3").command,
        )
        mapping = request.to_mapping()
        mapping["unknown"] = True
        invalid = (
            json.dumps(mapping).encode(),
            b'{"protocol":"x","protocol":"y"}',
            b'{"value":NaN}',
            b"x" * (MODULE.MAX_COMMAND_PACKET_BYTES + 1),
        )
        for payload in invalid:
            with self.subTest(payload=payload[:40]), self.assertRaises(
                MODULE.CommandProtocolError
            ):
                MODULE.decode_command_request(payload)

    def test_response_round_trip_is_strict(self) -> None:
        response = MODULE.GameCommandResponse(
            session=SESSION,
            sequence=3,
            request_id=REQUEST_ID,
            ok=True,
            code="OK_TELEPORT_RESTART",
            message="Teleport saved",
            restart_required=True,
            data={"position": [1.0, 2.0, 3.0]},
        )
        self.assertEqual(
            MODULE.decode_command_response(MODULE.encode_command_response(response)),
            response,
        )

    def test_runtime_pause_command_round_trip_is_strict(self) -> None:
        request = MODULE.GameCommandRequest(
            session=SESSION,
            sequence=4,
            request_id=REQUEST_ID,
            command=MODULE.RuntimePauseSet("paused", expected_epoch=3),
        )

        decoded = MODULE.decode_command_request(MODULE.encode_command_request(request))

        self.assertEqual(decoded, request)


class McCommandExecutionTest(unittest.TestCase):
    def setUp(self) -> None:
        self.origin = WorldPose(10.0, 20.0, 0.8, 0.5)
        self.state = MatrixWorldState.empty(
            world_id="town10", world_revision="revision"
        ).checkpoint(self.origin, upright=True, now_unix_ns=1)

    def test_summon_then_selector_teleport_persists_point_and_resume_pose(self) -> None:
        summon = MODULE.parse_mc_command(
            '/summon matrix:teleport_point ~2 ~-3 ~ {Tags:["XX","home"]}'
        ).command
        summoned = MODULE.execute_command(
            summon,
            state=self.state,
            current_pose=self.origin,
            now_unix_ns=2,
        )

        self.assertFalse(summoned.restart_required)
        self.assertEqual(summoned.code, "OK_SUMMONED")
        point = summoned.state.teleport_points[0]
        self.assertEqual(point.pose, WorldPose(12.0, 17.0, 0.8, 0.5))
        self.assertEqual(summoned.state.home, point.pose)

        teleport = MODULE.parse_mc_command(
            "/tp @s @e[type=matrix:teleport_point,tag=XX,limit=1]"
        ).command
        effect = MODULE.execute_command(
            teleport,
            state=summoned.state,
            current_pose=self.origin,
            now_unix_ns=3,
        )

        self.assertFalse(effect.restart_required)
        self.assertEqual(effect.code, "OK_TELEPORT")
        self.assertIs(effect.data["hot_pose"], True)
        self.assertEqual(effect.state.last_exit, point.pose)
        self.assertEqual(effect.state.resume_source, "teleport_command")

    def test_relative_coordinate_tp_keeps_current_yaw(self) -> None:
        command = MODULE.parse_mc_command("/tp @s ~1 ~2 1.25").command

        effect = MODULE.execute_command(
            command,
            state=self.state,
            current_pose=self.origin,
            now_unix_ns=2,
        )

        self.assertEqual(effect.state.last_exit, WorldPose(11.0, 22.0, 1.25, 0.5))
        self.assertFalse(effect.restart_required)
        self.assertEqual(effect.code, "OK_TELEPORT")
        self.assertIs(effect.data["hot_pose"], True)
        self.assertEqual(effect.data["yaw_rad"], 0.5)

    def test_world_scene_command_requests_bounded_internal_reload(self) -> None:
        command = MODULE.parse_mc_command("/world moon").command
        effect = MODULE.execute_command(
            command,
            state=self.state,
            current_pose=self.origin,
            now_unix_ns=2,
        )

        self.assertTrue(effect.restart_required)
        self.assertEqual(effect.code, "OK_WORLD_RESTART")
        self.assertEqual(effect.state.last_exit, self.origin)
        self.assertEqual(effect.state.resume_source, "teleport_command")
        self.assertEqual(effect.data["destination_id"], "moon")
        self.assertEqual(effect.data["target_scene_id"], 15)
        self.assertEqual(effect.data["target_scene_name"], "MoonWorld")

    def test_pose_yaw_and_recover_here_write_resume_pose(self) -> None:
        pose_command = MODULE.parse_mc_command("/pose @s yaw ~90deg").command
        pose_effect = MODULE.execute_command(
            pose_command,
            state=self.state,
            current_pose=self.origin,
            now_unix_ns=2,
        )
        self.assertEqual(
            pose_effect.state.last_exit,
            WorldPose(10.0, 20.0, 0.8, 0.5 + math.pi / 2.0),
        )
        self.assertEqual(pose_effect.state.resume_source, "pose_command")

        fallen = WorldPose(12.0, 24.0, 0.2, -2.0)
        recover_command = MODULE.parse_mc_command("/tpstand @s").command
        recover_effect = MODULE.execute_command(
            recover_command,
            state=self.state,
            current_pose=fallen,
            now_unix_ns=3,
        )
        self.assertEqual(recover_effect.state.last_exit, WorldPose(12.0, 24.0, 1.3, 0.5))
        self.assertEqual(recover_effect.state.resume_source, "recover_here")
        self.assertEqual(recover_effect.data["reset_pose"], "standing")

    def test_recover_here_lifts_contaminated_low_town10_safe_pose(self) -> None:
        contaminated = WorldPose(3.0, 4.0, 0.30, 0.25)
        state = MatrixWorldState.empty(
            world_id="g1_29dof:scene_terrain_t10",
            world_revision="revision",
        ).checkpoint(contaminated, upright=True, now_unix_ns=1)
        recover_command = MODULE.parse_mc_command("/recover").command
        recover_effect = MODULE.execute_command(
            recover_command,
            state=state,
            current_pose=contaminated,
            now_unix_ns=2,
        )

        self.assertGreaterEqual(
            recover_effect.state.last_exit.z,
            contaminated.z + MODULE.RECOVER_ROOT_LIFT_M,
        )
        self.assertEqual(
            recover_effect.state.last_exit,
            WorldPose(3.0, 4.0, 0.80, 0.25),
        )
        self.assertEqual(recover_effect.data["reset_pose"], "standing")

    def test_function_world_commands_apply_sequential_resume_pose(self) -> None:
        command = MODULE.parse_mc_command(
            "/function /tp @s ~1 ~2 1.25; /pose @s yaw 180deg"
        ).command

        effect = MODULE.execute_command(
            command,
            state=self.state,
            current_pose=self.origin,
            now_unix_ns=4,
        )

        self.assertFalse(effect.restart_required)
        self.assertEqual(effect.code, "OK_FUNCTION")
        self.assertEqual(effect.state.last_exit, WorldPose(11.0, 22.0, 1.25, math.pi))

    def test_file_function_requires_runtime_support(self) -> None:
        command = MODULE.parse_mc_command("/function recover_here").command

        with self.assertRaises(MODULE.CommandExecutionError) as context:
            MODULE.execute_command(
                command,
                state=self.state,
                current_pose=self.origin,
                now_unix_ns=4,
            )

        self.assertEqual(context.exception.code, "E_FUNCTION_RUNTIME_ONLY")

    def test_motion_settings_command_requires_runtime_support(self) -> None:
        command = MODULE.parse_mc_command(
            (
                "/data modify entity @s "
                "control.motion.gait_start_heading_error_rad "
                f"set value {math.radians(50.0):.10f}"
            )
        ).command

        with self.assertRaises(MODULE.CommandExecutionError) as context:
            MODULE.execute_command(
                command,
                state=self.state,
                current_pose=self.origin,
                now_unix_ns=4,
            )

        self.assertEqual(context.exception.code, "E_MOTION_SETTING_RUNTIME_ONLY")

    def test_missing_selector_target_does_not_mutate_state(self) -> None:
        command = MODULE.parse_mc_command(
            "/tp @s @e[type=matrix:teleport_point,tag=missing,limit=1]"
        ).command

        with self.assertRaises(MODULE.CommandExecutionError) as context:
            MODULE.execute_command(
                command,
                state=self.state,
                current_pose=self.origin,
            )

        self.assertEqual(context.exception.code, "E_SELECTOR_NO_TARGET")
        self.assertEqual(self.state.teleport_points, ())

    def test_resolved_out_of_world_coordinate_fails_before_mutation(self) -> None:
        command = MODULE.parse_mc_command("/tp @s 100001 0 1").command
        with self.assertRaises(MODULE.CommandExecutionError) as context:
            MODULE.execute_command(
                command,
                state=self.state,
                current_pose=self.origin,
            )
        self.assertEqual(context.exception.code, "E_OUT_OF_WORLD")


if __name__ == "__main__":
    unittest.main()
