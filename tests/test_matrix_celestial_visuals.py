from __future__ import annotations

import importlib
import json
import os
from pathlib import Path
import tempfile
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = REPO_ROOT / "scripts"
if os.fspath(SCRIPTS) not in os.sys.path:
    os.sys.path.insert(0, os.fspath(SCRIPTS))
MODULE = importlib.import_module("matrix_celestial_visuals")


def earth_lighting(*, altitude: float = 12.5, azimuth: float = 231.0):
    return {
        "body_id": "earth",
        "atmosphere": "earth_nitrogen_oxygen",
        "sun_altitude_deg": altitude,
        "sun_azimuth_deg": azimuth,
    }


class CelestialVisualCatalogTest(unittest.TestCase):
    def test_locked_catalog_selects_reproducible_body_defaults(self) -> None:
        catalog = MODULE.load_visual_catalog()
        sample = catalog.sample(earth_lighting())

        self.assertEqual(sample.profile_id, "earth-wet-cloudy-v1")
        self.assertEqual(
            sample.profile_sha256,
            "c7c86fbaedb54de5bf1c571973e7bb05566c611b7ed2925851d0b81f66637ab0",
        )
        self.assertEqual(
            tuple(sample.parameters_mapping()), MODULE.CARLA_WEATHER_FIELDS
        )
        self.assertEqual(sample.parameters_mapping()["sun_altitude_angle"], 12.5)
        self.assertEqual(sample.parameters_mapping()["sun_azimuth_angle"], 231.0)
        self.assertEqual(sample.parameters_mapping()["cloudiness"], 60.0)
        self.assertEqual(sample.parameters_mapping()["precipitation_deposits"], 50.0)

    def test_profile_override_keeps_ephemeris_sun_angles(self) -> None:
        catalog = MODULE.load_visual_catalog()
        sample = catalog.sample(
            earth_lighting(altitude=-18.0, azimuth=359.5),
            profile_id="earth-clear-v1",
        )

        parameters = sample.parameters_mapping()
        self.assertEqual(sample.profile_id, "earth-clear-v1")
        self.assertEqual(parameters["sun_altitude_angle"], -18.0)
        self.assertEqual(parameters["sun_azimuth_angle"], 359.5)
        self.assertEqual(parameters["cloudiness"], 5.0)
        self.assertEqual(parameters["scattering_intensity"], 1.0)

    def test_vacuum_profile_disables_atmospheric_weather_channels(self) -> None:
        catalog = MODULE.load_visual_catalog()
        sample = catalog.sample(
            {
                "body_id": "moon",
                "atmosphere": "vacuum",
                "sun_altitude_deg": 5.0,
                "sun_azimuth_deg": 90.0,
            }
        )

        parameters = sample.parameters_mapping()
        self.assertEqual(sample.profile_id, "moon-vacuum-v1")
        for name in MODULE.CARLA_STATIC_WEATHER_FIELDS:
            self.assertEqual(parameters[name], 0.0)

    def test_profile_mapping_is_complete_and_json_safe(self) -> None:
        sample = MODULE.load_visual_catalog().sample(earth_lighting())
        mapping = sample.profile_mapping()

        self.assertEqual(mapping["schema"], MODULE.VISUAL_PROFILE_SCHEMA)
        self.assertEqual(set(mapping["weather_parameters"]), set(MODULE.CARLA_WEATHER_FIELDS))
        json.dumps(mapping, allow_nan=False)

    def test_cross_body_override_is_rejected(self) -> None:
        catalog = MODULE.load_visual_catalog()
        with self.assertRaisesRegex(MODULE.CelestialVisualError, "does not match"):
            catalog.sample(earth_lighting(), profile_id="mars-dust-v1")

    def test_non_finite_and_out_of_range_sun_angles_are_rejected(self) -> None:
        catalog = MODULE.load_visual_catalog()
        for altitude, azimuth in ((float("nan"), 0.0), (91.0, 0.0), (0.0, 360.0)):
            with self.subTest(altitude=altitude, azimuth=azimuth), self.assertRaises(
                MODULE.CelestialVisualError
            ):
                catalog.sample(earth_lighting(altitude=altitude, azimuth=azimuth))

    def test_catalog_rejects_unknown_fields_and_out_of_range_weather(self) -> None:
        source = json.loads(
            MODULE.DEFAULT_VISUAL_CATALOG_PATH.read_text(encoding="utf-8")
        )
        mutations = []
        unknown = json.loads(json.dumps(source))
        unknown["profiles"][0]["surprise"] = True
        mutations.append(unknown)
        out_of_range = json.loads(json.dumps(source))
        out_of_range["profiles"][0]["weather_parameters"]["cloudiness"] = 101.0
        mutations.append(out_of_range)
        bad_default = json.loads(json.dumps(source))
        bad_default["default_profiles"]["earth"] = "moon-vacuum-v1"
        mutations.append(bad_default)
        bad_source = json.loads(json.dumps(source))
        bad_source["source"]["revision"] = "0" * 40
        mutations.append(bad_source)

        for index, value in enumerate(mutations):
            with self.subTest(index=index), tempfile.TemporaryDirectory() as temporary:
                path = Path(temporary) / "visuals.json"
                path.write_text(json.dumps(value), encoding="utf-8")
                with self.assertRaises(MODULE.CelestialVisualError):
                    MODULE.load_visual_catalog(path)

    def test_catalog_rejects_duplicate_json_keys_and_symlinks(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            duplicate = root / "duplicate.json"
            duplicate.write_text('{"schema":"a","schema":"b"}', encoding="utf-8")
            with self.assertRaisesRegex(MODULE.CelestialVisualError, "duplicate"):
                MODULE.load_visual_catalog(duplicate)

            target = root / "target.json"
            target.write_text(
                MODULE.DEFAULT_VISUAL_CATALOG_PATH.read_text(encoding="utf-8"),
                encoding="utf-8",
            )
            link = root / "link.json"
            link.symlink_to(target)
            with self.assertRaisesRegex(MODULE.CelestialVisualError, "non-symlink"):
                MODULE.load_visual_catalog(link)


if __name__ == "__main__":
    unittest.main()
