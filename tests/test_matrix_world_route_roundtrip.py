from __future__ import annotations

import json
import os
from pathlib import Path
import sys
import tempfile
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = REPO_ROOT / "scripts"
if os.fspath(SCRIPTS) not in sys.path:
    sys.path.insert(0, os.fspath(SCRIPTS))

import matrix_celestial_navigation as celestial  # noqa: E402
import matrix_route_entry as route_entry  # noqa: E402
import matrix_world_state as world_state  # noqa: E402


EARTH_WORLD = "g1_29dof:scene_terrain_t10"
MOON_WORLD = "g1_29dof:scene_terrain_moon_dynamic"


def _route(destination_id: str) -> dict[str, object]:
    catalog = celestial.load_catalog(REPO_ROOT / "config/universe/sol-2080.json")
    try:
        destination = catalog.destination(destination_id)
        assert destination.launch_route is not None
        return destination.launch_route.runtime_mapping(
            destination_id=destination.destination_id,
            teleport_tag=destination.teleport_tag,
        )
    finally:
        catalog.ephemeris.close()


def _stripped_original_environment(entries: list[str]) -> list[str]:
    return [
        entry
        for entry in entries
        if not entry.startswith("MATRIX_SONIC_RESTART_ROUTE_JSON=")
        and not entry.startswith("MATRIX_GAME_ROUTE_ENTRY_JSON=")
    ]


def _normalize_private_restart_route(
    launch_route: dict[str, object],
    *,
    expected_world_id: str,
    expected_scene_id: int,
) -> str:
    entry = route_entry.parse_route_entry(
        launch_route,
        expected_world_id=expected_world_id,
        expected_scene_id=expected_scene_id,
    )
    return route_entry.encode_route_entry_line(entry)


def _target_generation_environment(
    original_environment: list[str],
    canonical_route_entry: str,
) -> dict[str, str]:
    environment = dict(
        entry.split("=", 1)
        for entry in _stripped_original_environment(original_environment)
    )
    environment["MATRIX_GAME_ROUTE_ENTRY_JSON"] = canonical_route_entry
    return environment


def _loaded_target_start(
    state_file: Path,
    *,
    world_id: str,
    world_revision: str,
    canonical_route_entry: str,
    scene_id: int,
) -> dict[str, object]:
    store = world_state.WorldStateStore(
        state_file,
        world_id=world_id,
        world_revision=world_revision,
    )
    loaded = store.load()
    stale_resume = loaded.resolve_start()
    entry = route_entry.parse_route_entry_output_text(
        canonical_route_entry,
        expected_world_id=world_id,
        expected_scene_id=scene_id,
    )
    return {
        "load_status": store.load_status,
        "stale_checkpoint_id": stale_resume.checkpoint_id,
        "stale_generation": stale_resume.generation,
        "resume_checkpoint_id": None,
        "resume_generation": None,
        "spawn_pose": entry.entry_pose,
        "destination_id": entry.destination_id,
        "entity_id": entry.entity_id,
    }


def _write_stale_target_save(
    state_file: Path,
    *,
    world_id: str,
    world_revision: str,
    stale_pose: world_state.WorldPose,
) -> bytes:
    store = world_state.WorldStateStore(
        state_file,
        world_id=world_id,
        world_revision=world_revision,
    )
    store.save(
        store.state.checkpoint(
            stale_pose,
            upright=True,
            now_unix_ns=1,
        )
    )
    return state_file.read_bytes()


class MatrixWorldRouteRoundTripTest(unittest.TestCase):
    def test_earth_moon_earth_route_handoff_uses_one_shot_entry(self) -> None:
        launcher_source = (SCRIPTS / "run_matrix_sonic.sh").read_text(
            encoding="utf-8"
        )
        run_sim_source = (SCRIPTS / "run_sim.sh").read_text(encoding="utf-8")
        self.assertIn(
            "MATRIX_SONIC_RESTART_ROUTE_JSON=*|MATRIX_GAME_ROUTE_ENTRY_JSON=*",
            launcher_source,
        )
        self.assertIn("export MATRIX_GAME_ROUTE_ENTRY_JSON", launcher_source)
        self.assertIn("unset MATRIX_SONIC_RESTART_ROUTE_JSON", launcher_source)
        self.assertIn(
            'MATRIX_GAME_ROUTE_ENTRY_JSON_VALUE="${MATRIX_GAME_ROUTE_ENTRY_JSON:-}"',
            run_sim_source,
        )
        self.assertIn("unset MATRIX_GAME_ROUTE_ENTRY_JSON", run_sim_source)
        self.assertIn("parse_route_entry_output_text", run_sim_source)
        self.assertIn('GAME_WORLD_RESUME_CHECKPOINT_ID=""', run_sim_source)
        self.assertIn('GAME_WORLD_RESUME_GENERATION=""', run_sim_source)

        moon_route = _route("moon-tranquility-outpost")
        earth_route = _route("earth-overworld-home")
        original_environment = [
            "HOME=/tmp/matrix-home",
            "MATRIX_SONIC_RESTART_ROUTE_JSON=stale-private-route",
            "MATRIX_GAME_ROUTE_ENTRY_JSON=stale-canonical-entry",
            "MATRIX_GAME_WORLD_ID=" + EARTH_WORLD,
        ]

        stripped = _stripped_original_environment(original_environment)
        self.assertEqual(
            stripped,
            [
                "HOME=/tmp/matrix-home",
                "MATRIX_GAME_WORLD_ID=" + EARTH_WORLD,
            ],
        )

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            moon_state = root / "moon.json"
            earth_state = root / "earth.json"
            moon_revision = "moon-route-contract-v1"
            earth_revision = "earth-route-contract-v1"
            moon_old_bytes = _write_stale_target_save(
                moon_state,
                world_id=MOON_WORLD,
                world_revision=moon_revision,
                stale_pose=world_state.WorldPose(12.0, -7.0, -0.46045, 0.75),
            )
            earth_old_bytes = _write_stale_target_save(
                earth_state,
                world_id=EARTH_WORLD,
                world_revision=earth_revision,
                stale_pose=world_state.WorldPose(-18.0, 4.0, 0.92, -0.5),
            )

            moon_entry = _normalize_private_restart_route(
                moon_route,
                expected_world_id=MOON_WORLD,
                expected_scene_id=15,
            )
            moon_child_env = _target_generation_environment(
                original_environment,
                moon_entry,
            )
            self.assertNotIn("MATRIX_SONIC_RESTART_ROUTE_JSON", moon_child_env)
            self.assertEqual(moon_child_env["MATRIX_GAME_ROUTE_ENTRY_JSON"], moon_entry)
            moon_target = _loaded_target_start(
                moon_state,
                world_id=MOON_WORLD,
                world_revision=moon_revision,
                canonical_route_entry=moon_child_env["MATRIX_GAME_ROUTE_ENTRY_JSON"],
                scene_id=15,
            )
            self.assertEqual(moon_target["load_status"], "loaded")
            self.assertIsNotNone(moon_target["stale_checkpoint_id"])
            self.assertIsNone(moon_target["resume_checkpoint_id"])
            self.assertIsNone(moon_target["resume_generation"])
            self.assertEqual(
                moon_target["spawn_pose"],
                world_state.WorldPose(
                    -94.7,
                    -65.6,
                    -5.251562023162842,
                    0.0,
                ),
            )
            self.assertNotEqual(
                moon_target["spawn_pose"],
                world_state.WorldPose(12.0, -7.0, -0.46045, 0.75),
            )
            self.assertEqual(moon_target["destination_id"], "moon-tranquility-outpost")
            self.assertEqual(moon_state.read_bytes(), moon_old_bytes)

            earth_entry = _normalize_private_restart_route(
                earth_route,
                expected_world_id=EARTH_WORLD,
                expected_scene_id=2,
            )
            earth_child_env = _target_generation_environment(
                [
                    *stripped,
                    "MATRIX_SONIC_RESTART_ROUTE_JSON="
                    + json.dumps(
                        earth_route,
                        allow_nan=False,
                        separators=(",", ":"),
                        sort_keys=True,
                    ),
                    "MATRIX_GAME_ROUTE_ENTRY_JSON=" + moon_entry,
                ],
                earth_entry,
            )
            self.assertNotIn("MATRIX_SONIC_RESTART_ROUTE_JSON", earth_child_env)
            self.assertEqual(
                earth_child_env["MATRIX_GAME_ROUTE_ENTRY_JSON"],
                earth_entry,
            )
            earth_target = _loaded_target_start(
                earth_state,
                world_id=EARTH_WORLD,
                world_revision=earth_revision,
                canonical_route_entry=earth_child_env["MATRIX_GAME_ROUTE_ENTRY_JSON"],
                scene_id=2,
            )
            self.assertEqual(earth_target["load_status"], "loaded")
            self.assertIsNotNone(earth_target["stale_checkpoint_id"])
            self.assertIsNone(earth_target["resume_checkpoint_id"])
            self.assertIsNone(earth_target["resume_generation"])
            self.assertEqual(
                earth_target["spawn_pose"],
                world_state.WorldPose(0.0, 0.0, 0.793, 0.0),
            )
            self.assertNotEqual(
                earth_target["spawn_pose"],
                world_state.WorldPose(-18.0, 4.0, 0.92, -0.5),
            )
            self.assertEqual(earth_target["destination_id"], "earth-overworld-home")
            self.assertEqual(earth_state.read_bytes(), earth_old_bytes)

            self.assertNotEqual(
                json.loads(moon_entry)["target_world_id"],
                json.loads(earth_entry)["target_world_id"],
            )


if __name__ == "__main__":
    unittest.main()
