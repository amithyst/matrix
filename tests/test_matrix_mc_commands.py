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
