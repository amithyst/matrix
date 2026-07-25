from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = REPO_ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import matrix_route_entry as MODULE  # noqa: E402


EARTH_WORLD = "g1_29dof:scene_terrain_t10"
MOON_WORLD = "g1_29dof:scene_terrain_moon_dynamic"


def route(
    *,
    destination_id: str = "moon-tranquility-outpost",
    teleport_tag: str = "moon.tranquility",
    scene_id: int = 15,
    world_id: str = MOON_WORLD,
    entity_id: str = "tp-" + "a" * 32,
    position: list[float] | None = None,
    yaw_rad: float | None = None,
) -> dict[str, object]:
    if yaw_rad is None:
        yaw_rad = 0.0
    return {
        "schema": "matrix-celestial-launch-route/v1",
        "destination_id": destination_id,
        "teleport_tag": teleport_tag,
        "target_scene_id": scene_id,
        "target_world_id": world_id,
        "entry_pose": {
            "position": [-94.7, -65.6, -5.251562023162842]
            if position is None
            else position,
            "yaw_rad": yaw_rad,
        },
        "entity_id": entity_id,
    }


class MatrixRouteEntryTest(unittest.TestCase):
    def parse(self, value: dict[str, object]) -> MODULE.RouteEntry:
        return MODULE.parse_route_entry(
            value,
            expected_world_id=MOON_WORLD,
            expected_scene_id=15,
        )

    def test_valid_moon_route_outputs_stable_json_line(self) -> None:
        entry = self.parse(route())

        line = MODULE.encode_route_entry_line(entry)
        self.assertEqual(
            line,
            '{"destination_id":"moon-tranquility-outpost","entity_id":"tp-'
            + "a" * 32
            + '","entry_x":-94.7,"entry_y":-65.6,'
            '"entry_yaw_rad":0.0,'
            '"entry_z":-5.251562023162842,'
            '"schema":"matrix-route-entry/v1","target_scene_id":15,'
            '"target_world_id":"g1_29dof:scene_terrain_moon_dynamic",'
            '"teleport_tag":"moon.tranquility"}',
        )
        decoded = json.loads(line)
        self.assertEqual(decoded["entry_x"], -94.7)
        self.assertEqual(decoded["entry_y"], -65.6)
        self.assertEqual(decoded["entry_z"], -5.251562023162842)
        self.assertEqual(decoded["entry_yaw_rad"], 0.0)
        self.assertEqual(decoded["teleport_tag"], "moon.tranquility")
        reparsed = MODULE.parse_route_entry_output_text(
            line,
            expected_world_id=MOON_WORLD,
            expected_scene_id=15,
        )
        self.assertEqual(reparsed, entry)

    def test_canonical_output_parser_rejects_tampering(self) -> None:
        canonical = self.parse(route()).output_mapping()
        cases = []
        missing = dict(canonical)
        del missing["entity_id"]
        cases.append((missing, "invalid schema"))
        extra = dict(canonical)
        extra["checkpoint_id"] = "cp-" + "f" * 32
        cases.append((extra, "invalid schema"))
        wrong_schema = dict(canonical)
        wrong_schema["schema"] = "matrix-route-entry/v2"
        cases.append((wrong_schema, "unsupported"))
        wrong_world = dict(canonical)
        wrong_world["target_world_id"] = EARTH_WORLD
        cases.append((wrong_world, "world mismatch"))
        boolean_pose = dict(canonical)
        boolean_pose["entry_z"] = True
        cases.append((boolean_pose, r"position\[2\]"))

        for value, pattern in cases:
            with self.subTest(pattern=pattern):
                with self.assertRaisesRegex(MODULE.RouteEntryError, pattern):
                    MODULE.parse_route_entry_output_text(
                        json.dumps(value, separators=(",", ":")),
                        expected_world_id=MOON_WORLD,
                        expected_scene_id=15,
                    )

    def test_valid_earth_route(self) -> None:
        entry = MODULE.parse_route_entry(
            route(
                destination_id="earth-overworld-home",
                teleport_tag="home",
                scene_id=2,
                world_id=EARTH_WORLD,
                entity_id="tp-" + "b" * 32,
                position=[0.0, 0.0, 0.793],
            ),
            expected_world_id=EARTH_WORLD,
            expected_scene_id=2,
        )

        self.assertEqual(entry.target_world_id, EARTH_WORLD)
        self.assertEqual(entry.target_scene_id, 2)
        self.assertEqual(entry.entry_pose.z, 0.793)
        self.assertEqual(entry.entity_id, "tp-" + "b" * 32)

    def test_json_decoder_rejects_duplicate_keys_and_nan(self) -> None:
        duplicate = (
            '{"schema":"matrix-celestial-launch-route/v1",'
            '"schema":"matrix-celestial-launch-route/v1"}'
        )
        with self.assertRaisesRegex(MODULE.RouteEntryError, "duplicate"):
            MODULE.parse_route_entry_text(
                duplicate,
                expected_world_id=MOON_WORLD,
                expected_scene_id=15,
            )

        malformed = json.dumps(route()).replace("-5.251562023162842", "NaN")
        with self.assertRaisesRegex(MODULE.RouteEntryError, "invalid JSON constant"):
            MODULE.parse_route_entry_text(
                malformed,
                expected_world_id=MOON_WORLD,
                expected_scene_id=15,
            )

    def test_rejects_non_ascii_route_json(self) -> None:
        value = route()
        value["destination_id"] = "moon-\u9759"
        with self.assertRaisesRegex(MODULE.RouteEntryError, "ASCII"):
            MODULE.parse_route_entry_text(
                json.dumps(value, ensure_ascii=False),
                expected_world_id=MOON_WORLD,
                expected_scene_id=15,
            )

    def test_rejects_missing_extra_and_bad_schema_fields(self) -> None:
        missing = route()
        del missing["entity_id"]
        extra = route()
        extra["required_assets"] = []
        bad_schema = route()
        bad_schema["schema"] = "matrix-celestial-launch-route/v2"

        for value, pattern in (
            (missing, "invalid schema"),
            (extra, "invalid schema"),
            (bad_schema, "unsupported"),
        ):
            with self.subTest(pattern=pattern):
                with self.assertRaisesRegex(MODULE.RouteEntryError, pattern):
                    self.parse(value)

    def test_rejects_mismatched_target_identity(self) -> None:
        with self.assertRaisesRegex(MODULE.RouteEntryError, "world mismatch"):
            MODULE.parse_route_entry(
                route(world_id=EARTH_WORLD),
                expected_world_id=MOON_WORLD,
                expected_scene_id=15,
            )
        with self.assertRaisesRegex(MODULE.RouteEntryError, "scene mismatch"):
            MODULE.parse_route_entry(
                route(scene_id=2),
                expected_world_id=MOON_WORLD,
                expected_scene_id=15,
            )

    def test_rejects_invalid_ids_entity_and_pose_bounds(self) -> None:
        cases = (
            ("destination_id", "Moon", "destination_id"),
            ("teleport_tag", "bad tag", "teleport tag"),
            ("target_world_id", "bad world", "world_id"),
            ("entity_id", "not-a-teleport-point", "entity_id"),
        )
        for key, value, pattern in cases:
            payload = route()
            payload[key] = value
            with self.subTest(key=key):
                with self.assertRaisesRegex(MODULE.RouteEntryError, pattern):
                    self.parse(payload)

        high_pose = route(position=[100001.0, 0.0, 0.0])
        with self.assertRaisesRegex(MODULE.RouteEntryError, "pose.x"):
            self.parse(high_pose)

        bool_scene = route()
        bool_scene["target_scene_id"] = True
        with self.assertRaisesRegex(MODULE.RouteEntryError, "target_scene_id"):
            self.parse(bool_scene)

    def test_cli_reads_json_arg_file_and_stdin(self) -> None:
        payload = json.dumps(route(), separators=(",", ":"))
        command = [
            sys.executable,
            str(SCRIPTS / "matrix_route_entry.py"),
            "--expected-world-id",
            MOON_WORLD,
            "--expected-scene-id",
            "15",
        ]
        direct = subprocess.run(
            [*command, "--json", payload],
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(direct.returncode, 0, direct.stderr)
        self.assertEqual(json.loads(direct.stdout)["entry_yaw_rad"], 0.0)
        self.assertEqual(direct.stderr, "")

        stdin_result = subprocess.run(
            command,
            input=payload,
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(stdin_result.returncode, 0, stdin_result.stderr)
        self.assertEqual(stdin_result.stdout, direct.stdout)

        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "route.json"
            path.write_text(payload, encoding="ascii")
            file_result = subprocess.run(
                [*command, "--file", os.fspath(path)],
                text=True,
                capture_output=True,
                check=False,
            )
        self.assertEqual(file_result.returncode, 0, file_result.stderr)
        self.assertEqual(file_result.stdout, direct.stdout)

    def test_cli_reports_fail_closed_errors(self) -> None:
        bad = subprocess.run(
            [
                sys.executable,
                str(SCRIPTS / "matrix_route_entry.py"),
                "--expected-world-id",
                MOON_WORLD,
                "--expected-scene-id",
                "15",
                "--json",
                json.dumps(route(scene_id=2)),
            ],
            text=True,
            capture_output=True,
            check=False,
        )

        self.assertEqual(bad.returncode, 2)
        self.assertEqual(bad.stdout, "")
        self.assertIn("matrix-route-entry ERROR:", bad.stderr)
        self.assertIn("scene mismatch", bad.stderr)

    def test_imports_under_isolated_no_site_python(self) -> None:
        code = (
            "import sys;"
            f"sys.path.insert(0,{str(SCRIPTS)!r});"
            "from matrix_route_entry import parse_route_entry_text;"
            f"payload={json.dumps(route(), separators=(',', ':'))!r};"
            "entry=parse_route_entry_text(payload,"
            f"expected_world_id={MOON_WORLD!r},expected_scene_id=15);"
            "print(entry.target_world_id)"
        )
        result = subprocess.run(
            [sys.executable, "-I", "-S", "-c", code],
            text=True,
            capture_output=True,
            check=False,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), MOON_WORLD)


if __name__ == "__main__":
    unittest.main()
