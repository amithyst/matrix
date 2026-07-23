from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = REPO_ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import matrix_celestial_navigation as MODULE  # noqa: E402


class CelestialCatalogTest(unittest.TestCase):
    def setUp(self) -> None:
        self.catalog = MODULE.load_catalog()

    def test_catalog_models_one_origin_rebased_sol_universe(self) -> None:
        self.assertEqual(self.catalog.universe_id, "sol-2080")
        self.assertTrue(self.catalog.origin_rebasing)
        self.assertEqual(self.catalog.simulation_local_bound_m, 100_000.0)
        self.assertEqual(self.catalog.default_body_id, "earth")
        self.assertEqual(
            [body.runtime_status for body in self.catalog.bodies],
            ["reference", "active", "planned", "planned"],
        )
        self.assertEqual(self.catalog.frame, "sol_heliocentric_icrf")
        self.assertEqual(self.catalog.ephemeris_provider, "matrix-analytical-v1")
        self.assertEqual(
            [destination.teleport_tag for destination in self.catalog.destinations],
            ["home", "moon.tranquility", "mars.utopia"],
        )

    def test_dynamic_inertial_coordinates_never_replace_local_physics_pose(self) -> None:
        mapping = self.catalog.navigation_mapping(
            {},
            command_available=True,
            in_flight=False,
            restart_required=False,
            outcome_unknown=False,
        )
        earth = next(body for body in mapping["bodies"] if body["id"] == "earth")
        moon = next(body for body in mapping["bodies"] if body["id"] == "moon")
        separation = sum(
            (moon["center_inertial_m"][axis] - earth["center_inertial_m"][axis]) ** 2
            for axis in range(3)
        ) ** 0.5
        self.assertGreater(separation, 300_000_000.0)
        self.assertLess(separation, 430_000_000.0)
        self.assertEqual(self.catalog.simulation_local_bound_m, 100_000.0)

    def test_catalog_rejects_invalid_surface_anchor(self) -> None:
        value = json.loads(MODULE.DEFAULT_CATALOG_PATH.read_text(encoding="utf-8"))
        value["destinations"][0]["surface_anchor"]["latitude_deg"] = 91.0
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "invalid.json"
            path.write_text(json.dumps(value), encoding="utf-8")
            with self.assertRaisesRegex(
                MODULE.CelestialNavigationError,
                "latitude",
            ):
                MODULE.load_catalog(path)

    def test_catalog_rejects_duplicate_json_fields(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "duplicate.json"
            path.write_text(
                '{"schema":"a","schema":"b"}',
                encoding="utf-8",
            )
            with self.assertRaisesRegex(
                MODULE.CelestialNavigationError,
                "duplicate",
            ):
                MODULE.load_catalog(path)

    def test_catalog_and_probe_reject_overflowing_json_integers(self) -> None:
        value = json.loads(MODULE.DEFAULT_CATALOG_PATH.read_text(encoding="utf-8"))
        value["bodies"][0]["ellipsoid_radii_m"][0] = 10**1000
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "overflow.json"
            path.write_text(json.dumps(value), encoding="utf-8")
            with self.assertRaisesRegex(
                MODULE.CelestialNavigationError,
                "finite number",
            ):
                MODULE.load_catalog(path)

        with self.assertRaisesRegex(
            MODULE.CelestialNavigationError,
            "finite number",
        ):
            MODULE.probes_from_response(
                {
                    "world_id": "town10:test",
                    "teleport_points": [
                        {
                            "tag": "home",
                            "found": True,
                            "entity_id": "tp-" + "a" * 32,
                            "position": [10**1000, 0.0, 1.0],
                            "yaw_rad": 0.0,
                        },
                        {"tag": "moon.tranquility", "found": False},
                        {"tag": "mars.utopia", "found": False},
                    ],
                },
                catalog=self.catalog,
            )

    def test_probe_response_drives_ready_and_honest_planned_statuses(self) -> None:
        probes = MODULE.probes_from_response(
            {
                "world_id": "town10:test",
                "teleport_points": [
                    {
                        "tag": "home",
                        "found": True,
                        "entity_id": "tp-" + "a" * 32,
                        "position": [160.0, 117.0, 1.2],
                        "yaw_rad": 0.0,
                    },
                    {"tag": "moon.tranquility", "found": False},
                    {"tag": "mars.utopia", "found": False},
                ],
            },
            catalog=self.catalog,
        )
        mapping = self.catalog.navigation_mapping(
            probes,
            command_available=True,
            in_flight=False,
            restart_required=False,
            outcome_unknown=False,
        )

        self.assertEqual(mapping["status"], "ready")
        earth, moon, mars = mapping["destinations"]
        self.assertEqual(earth["status"], "ready")
        self.assertTrue(earth["enabled"])
        self.assertEqual(earth["local_position_m"], [160.0, 117.0, 1.2])
        self.assertGreater(abs(earth["universe_position_m"][0]), 1_000_000_000.0)
        self.assertEqual(mapping["version"], 2)
        self.assertEqual(mapping["simulation_time"]["scenario_utc"], "2080-01-01T00:00:00Z")
        self.assertEqual(mapping["lighting"]["render_authority"], "state-only")
        self.assertEqual(moon["status"], "world_unavailable")
        self.assertEqual(mars["status"], "world_unavailable")
        self.assertFalse(moon["enabled"])
        self.assertFalse(mars["enabled"])

        unavailable = self.catalog.navigation_mapping(
            {},
            command_available=False,
            in_flight=False,
            restart_required=False,
            outcome_unknown=False,
        )
        self.assertEqual(unavailable["status"], "unavailable")
        self.assertFalse(unavailable["available"])

        restarting = self.catalog.navigation_mapping(
            probes,
            command_available=True,
            in_flight=False,
            restart_required=True,
            outcome_unknown=False,
        )
        self.assertEqual(restarting["status"], "unavailable")
        self.assertFalse(restarting["available"])
        self.assertTrue(
            all(
                destination["status"] == "unavailable"
                and destination["enabled"] is False
                for destination in restarting["destinations"]
            )
        )

    def test_probe_response_rejects_reordered_catalog_tags(self) -> None:
        with self.assertRaisesRegex(
            MODULE.CelestialNavigationError,
            "do not match",
        ):
            MODULE.probes_from_response(
                {
                    "world_id": "town10:test",
                    "teleport_points": [
                        {"tag": "moon.tranquility", "found": False},
                        {"tag": "home", "found": False},
                        {"tag": "mars.utopia", "found": False},
                    ],
                },
                catalog=self.catalog,
            )


if __name__ == "__main__":
    unittest.main()
