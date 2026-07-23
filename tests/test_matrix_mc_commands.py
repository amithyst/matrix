import json
import math
from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import matrix_mc_commands as MODULE  # noqa: E402
from matrix_world_state import MatrixWorldState, WorldPose  # noqa: E402


SESSION = "a" * 32
REQUEST_ID = "cmd-" + "b" * 32


class McCommandParserTest(unittest.TestCase):
    def test_parses_creative_inventory_spawn(self) -> None:
        parsed = MODULE.parse_mc_command("/item spawn Training_Blaster")

        self.assertEqual(
            parsed.command,
            MODULE.CreativeSpawnItem(item_id="training_blaster"),
        )

    def test_parses_whitelisted_data_modify_inputs(self) -> None:
        boolean_paths = (
            *(f"control.input.keyboard.{key}" for key in (
                "w", "a", "s", "d", "q", "e", "v", "ctrl", "alt", "shift"
            )),
            "control.input.mouse.left",
            "control.input.mouse.right",
            "control.input.mouse.middle",
        )
        for path in boolean_paths:
            with self.subTest(path=path):
                parsed = MODULE.parse_mc_command(
                    f"/data modify entity @s {path} set value true"
                )
                self.assertEqual(parsed.command, MODULE.DataModifyInput(path, True))

        numeric_cases = {
            "control.input.gamepad.forward": -1.0,
            "control.input.gamepad.right": 1.0,
            "control.input.gamepad.look_yaw": -0.25,
            "control.input.gamepad.look_pitch": 0.25,
            "control.input.mouse.dx": -4096.0,
            "control.input.mouse.dy": 4096.0,
        }
        for path, value in numeric_cases.items():
            with self.subTest(path=path):
                parsed = MODULE.parse_mc_command(
                    f"/data modify entity @s {path} set value {value}"
                )
                self.assertEqual(parsed.command, MODULE.DataModifyInput(path, value))

    def test_data_modify_input_rejects_unknown_type_nonfinite_and_range(self) -> None:
        invalid = (
            (
                "/data modify entity @s control.input.keyboard.space set value true",
                "E_DATA_PATH_UNKNOWN",
            ),
            (
                "/data modify entity @s control.input.gamepad.throttle set value 0",
                "E_DATA_PATH_UNKNOWN",
            ),
            (
                "/data modify entity @s control.input.mouse.wheel set value 0",
                "E_DATA_PATH_UNKNOWN",
            ),
            (
                "/data modify entity @s control.input.keyboard.w set value 1",
                "E_DATA_INPUT_TYPE",
            ),
            (
                "/data modify entity @s control.input.gamepad.forward set value true",
                "E_DATA_INPUT_TYPE",
            ),
            (
                "/data modify entity @s control.input.mouse.left set value falsey",
                "E_DATA_INPUT_TYPE",
            ),
            (
                "/data modify entity @s control.input.gamepad.forward set value NaN",
                "E_DATA_INPUT_NONFINITE",
            ),
            (
                "/data modify entity @s control.input.gamepad.forward set value 1e999",
                "E_DATA_INPUT_NONFINITE",
            ),
            (
                "/data modify entity @s control.input.gamepad.look_yaw set value 1.01",
                "E_DATA_INPUT_RANGE",
            ),
            (
                "/data modify entity @s control.input.mouse.dx set value -4096.01",
                "E_DATA_INPUT_RANGE",
            ),
        )
        for text, code in invalid:
            with self.subTest(text=text), self.assertRaises(
                MODULE.CommandParseError
            ) as context:
                MODULE.parse_mc_command(text)
            self.assertEqual(context.exception.code, code)

    def test_data_modify_input_rejects_noncanonical_syntax(self) -> None:
        invalid = (
            "/data modify entity @e control.input.keyboard.w set value true",
            "/data modify entity @s control.input.keyboard.w merge value true",
            "/data modify entity @s control.input.keyboard.w set true",
        )
        for text in invalid:
            with self.subTest(text=text), self.assertRaises(
                MODULE.CommandParseError
            ) as context:
                MODULE.parse_mc_command(text)
            self.assertEqual(context.exception.code, "E_DATA_SYNTAX")

    def test_parses_whitelisted_data_modify_number(self) -> None:
        parsed = MODULE.parse_mc_command(
            "/data modify entity @s "
            "control.motion.gears.walk.double_tap_speed_mps set value 1.2"
        )

        self.assertEqual(
            parsed.command,
            MODULE.DataModifyNumber(
                "control.motion.gears.walk.double_tap_speed_mps", 1.2
            ),
        )

    def test_data_modify_rejects_unknown_path_operation_and_nonfinite_value(self) -> None:
        invalid = (
            "/data modify entity @s control.motion.unknown set value 1",
            "/data modify entity @e control.motion.gears.walk.speed_mps set value 1",
            "/data modify entity @s control.motion.gears.walk.speed_mps merge value 1",
            "/data modify entity @s control.motion.gears.walk.speed_mps set value NaN",
            "/data modify entity @s control.motion.gears.walk.speed_mps set value 1e999",
        )
        for text in invalid:
            with self.subTest(text=text), self.assertRaises(MODULE.CommandParseError):
                MODULE.parse_mc_command(text)

    def test_parses_policy_slot_assignment(self) -> None:
        parsed = MODULE.parse_mc_command("/policy recovery KungFu")

        self.assertEqual(
            parsed.command,
            MODULE.PolicySlotAssignment(slot="recovery", policy_id="kungfu"),
        )

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

    def test_parses_minecraft_local_tp_coordinates(self) -> None:
        command = MODULE.parse_mc_command("/tp @s ^-1.5 ^ ^2").command

        self.assertEqual(
            command,
            MODULE.TeleportLocalCoordinates(left=-1.5, up=0.0, forward=2.0),
        )

    def test_teleport_list_is_bounded_unique_and_typed(self) -> None:
        parsed = MODULE.parse_mc_command(
            "/teleport list home moon.tranquility mars.utopia"
        )
        self.assertEqual(
            parsed.command,
            MODULE.TeleportList(("home", "moon.tranquility", "mars.utopia")),
        )
        for command in (
            "/teleport list",
            "/teleport list home home",
            "/teleport list " + " ".join(f"tag{index}" for index in range(9)),
        ):
            with self.subTest(command=command), self.assertRaises(
                MODULE.CommandParseError
            ):
                MODULE.parse_mc_command(command)

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

    def test_rejects_mixed_nonfinite_control_and_oversized_input(self) -> None:
        invalid = (
            "/tp @s ^1 2 3",
            "/tp @s ^1 ^2 ~3",
            "/tp @s ^1e999 ^ ^",
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
    def test_creative_spawn_round_trip_is_typed(self) -> None:
        request = MODULE.GameCommandRequest(
            session=SESSION,
            sequence=5,
            request_id=REQUEST_ID,
            command=MODULE.CreativeSpawnItem("training_blaster"),
        )

        payload = MODULE.encode_command_request(request)

        self.assertNotIn(b"/item", payload)
        self.assertEqual(MODULE.decode_command_request(payload), request)

    def test_data_modify_input_round_trip_is_typed(self) -> None:
        commands = (
            MODULE.DataModifyInput("control.input.keyboard.ctrl", True),
            MODULE.DataModifyInput("control.input.gamepad.look_pitch", -0.5),
            MODULE.DataModifyInput("control.input.mouse.dy", 12.5),
            MODULE.DataModifyInput("control.input.mouse.middle", False),
        )
        for sequence, command in enumerate(commands, start=10):
            with self.subTest(command=command):
                request = MODULE.GameCommandRequest(
                    session=SESSION,
                    sequence=sequence,
                    request_id=REQUEST_ID,
                    command=command,
                )
                payload = MODULE.encode_command_request(request)

                self.assertNotIn(b"/data", payload)
                self.assertIn(b'"name":"data_modify_input"', payload)
                self.assertEqual(MODULE.decode_command_request(payload), request)

    def test_data_modify_input_mapping_schema_is_strict(self) -> None:
        valid = {
            "name": "data_modify_input",
            "path": "control.input.gamepad.forward",
            "value": 0.5,
        }
        invalid = (
            {**valid, "unknown": True},
            {"name": "data_modify_input", "path": valid["path"]},
            {**valid, "path": "control.input.gamepad.unknown"},
            {**valid, "value": True},
            {**valid, "value": math.inf},
            {**valid, "value": 1.001},
            {
                **valid,
                "path": "control.input.keyboard.w",
                "value": 1,
            },
        )
        for mapping in invalid:
            with self.subTest(mapping=mapping), self.assertRaises(
                MODULE.CommandProtocolError
            ):
                MODULE.command_from_mapping(mapping)

    def test_data_modify_round_trip_is_typed(self) -> None:
        request = MODULE.GameCommandRequest(
            session=SESSION,
            sequence=5,
            request_id=REQUEST_ID,
            command=MODULE.parse_mc_command(
                "/data modify entity @s "
                "control.motion.gears.slow.speed_mps set value 0.15"
            ).command,
        )

        payload = MODULE.encode_command_request(request)

        self.assertNotIn(b"/data", payload)
        self.assertEqual(MODULE.decode_command_request(payload), request)

    def test_teleport_list_round_trip_contains_only_typed_tags(self) -> None:
        request = MODULE.GameCommandRequest(
            session=SESSION,
            sequence=6,
            request_id=REQUEST_ID,
            command=MODULE.TeleportList(("home", "moon.tranquility")),
        )

        payload = MODULE.encode_command_request(request)

        self.assertNotIn(b"/teleport", payload)
        self.assertEqual(MODULE.decode_command_request(payload), request)

    def test_policy_slot_assignment_round_trip_is_typed(self) -> None:
        request = MODULE.GameCommandRequest(
            session=SESSION,
            sequence=4,
            request_id=REQUEST_ID,
            command=MODULE.PolicySlotAssignment("recovery", "host"),
        )

        payload = MODULE.encode_command_request(request)
        decoded = MODULE.decode_command_request(payload)

        self.assertEqual(decoded, request)
        self.assertNotIn(b"/policy", payload)

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

    def test_local_coordinate_request_round_trip_is_typed(self) -> None:
        command = MODULE.parse_mc_command("/tp @s ^ ^1 ^2").command
        request = MODULE.GameCommandRequest(
            session=SESSION,
            sequence=8,
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

        self.assertTrue(effect.restart_required)
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
        self.assertTrue(effect.restart_required)

    def test_local_coordinate_tp_uses_yaw_left_up_forward_basis(self) -> None:
        origin = WorldPose(10.0, 20.0, 0.8, math.pi / 2.0)
        command = MODULE.parse_mc_command("/tp @s ^1 ^2 ^3").command

        effect = MODULE.execute_command(
            command,
            state=self.state,
            current_pose=origin,
            now_unix_ns=2,
        )

        assert effect.state.last_exit is not None
        self.assertAlmostEqual(effect.state.last_exit.x, 9.0)
        self.assertAlmostEqual(effect.state.last_exit.y, 23.0)
        self.assertAlmostEqual(effect.state.last_exit.z, 2.8)
        self.assertAlmostEqual(effect.state.last_exit.yaw_rad, math.pi / 2.0)
        self.assertTrue(effect.restart_required)

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

    def test_teleport_list_returns_requested_snapshot_without_mutation(self) -> None:
        state, point = self.state.add_teleport_point(
            WorldPose(12.0, 18.0, 0.8, 0.25),
            ("home",),
            entity_id="tp-" + "c" * 32,
            now_unix_ns=2,
        )

        effect = MODULE.execute_command(
            MODULE.TeleportList(("home", "moon.tranquility")),
            state=state,
            current_pose=self.origin,
            now_unix_ns=3,
        )

        self.assertIs(effect.state, state)
        self.assertFalse(effect.restart_required)
        self.assertEqual(effect.code, "OK_TELEPORT_LIST")
        self.assertEqual(
            effect.data,
            {
                "world_id": "town10",
                "teleport_points": [
                    {
                        "tag": "home",
                        "found": True,
                        "entity_id": point.entity_id,
                        "position": [12.0, 18.0, 0.8],
                        "yaw_rad": 0.25,
                    },
                    {"tag": "moon.tranquility", "found": False},
                ],
            },
        )

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
