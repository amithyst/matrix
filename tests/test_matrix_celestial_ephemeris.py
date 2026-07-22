from __future__ import annotations

import json
import hashlib
import math
from pathlib import Path
import sys
import tempfile
import threading
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = REPO_ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import matrix_celestial_ephemeris as EPHEMERIS  # noqa: E402
import matrix_celestial_navigation as NAVIGATION  # noqa: E402


class MutableMonotonicClock:
    def __init__(self, value: int = 0) -> None:
        self.value = value

    def __call__(self) -> int:
        return self.value


class CelestialEphemerisTest(unittest.TestCase):
    def setUp(self) -> None:
        self.catalog = NAVIGATION.load_catalog()

    def test_ephemeris_is_deterministic_and_planets_revolve(self) -> None:
        first = self.catalog.ephemeris.states(0)
        repeated = self.catalog.ephemeris.states(0)
        one_day = self.catalog.ephemeris.states(86_400_000_000_000)

        self.assertEqual(first["earth"].center_inertial_m, repeated["earth"].center_inertial_m)
        earth_motion = math.dist(
            first["earth"].center_inertial_m,
            one_day["earth"].center_inertial_m,
        )
        mars_motion = math.dist(
            first["mars"].center_inertial_m,
            one_day["mars"].center_inertial_m,
        )
        self.assertGreater(earth_motion, 2_000_000_000.0)
        self.assertGreater(mars_motion, 1_500_000_000.0)
        self.assertLess(earth_motion, 3_500_000_000.0)

    def test_body_frames_are_right_handed_and_rotate(self) -> None:
        initial = self.catalog.ephemeris.states(0)["earth"].body_to_inertial
        six_hours = self.catalog.ephemeris.states(21_600_000_000_000)[
            "earth"
        ].body_to_inertial
        columns = [tuple(initial[row][column] for row in range(3)) for column in range(3)]
        for column in columns:
            self.assertAlmostEqual(sum(value * value for value in column), 1.0, places=10)
        self.assertAlmostEqual(
            sum(columns[0][axis] * columns[1][axis] for axis in range(3)),
            0.0,
            places=10,
        )
        cross = (
            columns[0][1] * columns[1][2] - columns[0][2] * columns[1][1],
            columns[0][2] * columns[1][0] - columns[0][0] * columns[1][2],
            columns[0][0] * columns[1][1] - columns[0][1] * columns[1][0],
        )
        self.assertGreater(sum(cross[axis] * columns[2][axis] for axis in range(3)), 0.999999)
        self.assertGreater(math.dist(initial[0], six_hours[0]), 1.0)

    def test_surface_anchor_keeps_matrix_coordinates_local(self) -> None:
        destination = self.catalog.destination("earth-overworld-home")
        earth = self.catalog.body("earth")
        origin = destination.surface_anchor.local_position_to_body_fixed(
            earth.ellipsoid_radii_m,
            (0.0, 0.0, 0.0),
        )
        moved = destination.surface_anchor.local_position_to_body_fixed(
            earth.ellipsoid_radii_m,
            (100.0, -25.0, 3.0),
        )
        self.assertAlmostEqual(math.dist(origin, moved), math.sqrt(10_634.0), places=5)
        self.assertGreater(math.dist((0.0, 0.0, 0.0), origin), 6_300_000.0)

    def test_solar_state_is_normalized_and_physically_bounded(self) -> None:
        mapping = self.catalog.navigation_mapping(
            {},
            command_available=True,
            in_flight=False,
            restart_required=False,
            outcome_unknown=False,
        )
        lighting = mapping["lighting"]
        direction = lighting["sun_direction_local"]
        self.assertAlmostEqual(sum(value * value for value in direction), 1.0, places=10)
        self.assertGreater(lighting["solar_irradiance_w_m2"], 1_200.0)
        self.assertLess(lighting["solar_irradiance_w_m2"], 1_500.0)
        self.assertGreater(lighting["sun_angular_radius_deg"], 0.24)
        self.assertLess(lighting["sun_angular_radius_deg"], 0.29)
        self.assertGreaterEqual(lighting["eclipse_fraction"], 0.0)
        self.assertLessEqual(lighting["eclipse_fraction"], 1.0)

    def test_eclipse_disk_overlap_handles_total_partial_and_none(self) -> None:
        overlap = EPHEMERIS._circle_overlap_fraction
        self.assertEqual(
            overlap(sun_radius_rad=0.01, occluder_radius_rad=0.02, separation_rad=0.0),
            1.0,
        )
        self.assertEqual(
            overlap(sun_radius_rad=0.01, occluder_radius_rad=0.01, separation_rad=0.03),
            0.0,
        )
        partial = overlap(
            sun_radius_rad=0.01,
            occluder_radius_rad=0.01,
            separation_rad=0.01,
        )
        self.assertGreater(partial, 0.0)
        self.assertLess(partial, 1.0)

    def test_persistent_clock_survives_a_cold_reload(self) -> None:
        monotonic = MutableMonotonicClock(1_000_000_000)
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "universe-clock.json"
            clock = EPHEMERIS.PersistentSimulationClock(
                universe_id="sol-2080",
                reference_epoch_utc="2080-01-01T00:00:00Z",
                tai_minus_utc_at_epoch_s=37,
                rate_numerator=60,
                rate_denominator=1,
                state_path=path,
                monotonic_ns=monotonic,
            )
            monotonic.value += 2_000_000_000
            snapshot = clock.snapshot()
            self.assertEqual(snapshot.elapsed_tai_ns, 120_000_000_000)
            self.assertEqual(snapshot.scenario_utc, "2080-01-01T00:02:00Z")
            clock.close()

            saved = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(saved["elapsed_tai_ns"], 120_000_000_000)
            resumed = EPHEMERIS.PersistentSimulationClock(
                universe_id="sol-2080",
                reference_epoch_utc="2080-01-01T00:00:00Z",
                tai_minus_utc_at_epoch_s=37,
                rate_numerator=60,
                rate_denominator=1,
                state_path=path,
                monotonic_ns=monotonic,
            )
            monotonic.value += 500_000_000
            self.assertEqual(resumed.snapshot().elapsed_tai_ns, 150_000_000_000)

    def test_clock_rejects_identity_or_rate_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "clock.json"
            path.write_text(
                json.dumps(
                    {
                        "schema": EPHEMERIS.CLOCK_SCHEMA,
                        "universe_id": "other",
                        "reference_epoch_utc": "2080-01-01T00:00:00Z",
                        "elapsed_tai_ns": 0,
                        "rate_numerator": 1,
                        "rate_denominator": 1,
                    }
                ),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(
                EPHEMERIS.CelestialEphemerisError,
                "identity",
            ):
                EPHEMERIS.PersistentSimulationClock(
                    universe_id="sol-2080",
                    reference_epoch_utc="2080-01-01T00:00:00Z",
                    tai_minus_utc_at_epoch_s=37,
                    state_path=path,
                )

    def test_non_forced_clock_checkpoint_does_not_wait_for_storage(self) -> None:
        monotonic = MutableMonotonicClock(1_000_000_000)
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "clock.json"
            clock = EPHEMERIS.PersistentSimulationClock(
                universe_id="sol-2080",
                reference_epoch_utc="2080-01-01T00:00:00Z",
                tai_minus_utc_at_epoch_s=37,
                state_path=path,
                monotonic_ns=monotonic,
            )
            original_write = clock._write_payload
            write_started = threading.Event()
            release_write = threading.Event()

            def blocked_write(payload: dict[str, object]) -> None:
                write_started.set()
                if not release_write.wait(timeout=2.0):
                    raise TimeoutError("test did not release clock writer")
                original_write(payload)

            clock._write_payload = blocked_write  # type: ignore[method-assign]
            monotonic.value += 1_000_000_000
            checkpoint_finished = threading.Event()
            checkpoint_result: list[bool] = []

            def checkpoint() -> None:
                checkpoint_result.append(clock.checkpoint())
                checkpoint_finished.set()

            caller = threading.Thread(target=checkpoint)
            caller.start()
            try:
                self.assertTrue(write_started.wait(timeout=1.0))
                self.assertTrue(checkpoint_finished.wait(timeout=0.5))
                self.assertEqual(checkpoint_result, [True])
            finally:
                release_write.set()
                caller.join(timeout=1.0)
            clock.close()

    def test_clock_close_flushes_latest_value_and_stops_writer(self) -> None:
        monotonic = MutableMonotonicClock(1_000_000_000)
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "clock.json"
            clock = EPHEMERIS.PersistentSimulationClock(
                universe_id="sol-2080",
                reference_epoch_utc="2080-01-01T00:00:00Z",
                tai_minus_utc_at_epoch_s=37,
                state_path=path,
                monotonic_ns=monotonic,
            )
            original_write = clock._write_payload
            first_write_started = threading.Event()
            release_first_write = threading.Event()
            write_count = 0

            def delayed_first_write(payload: dict[str, object]) -> None:
                nonlocal write_count
                write_count += 1
                if write_count == 1:
                    first_write_started.set()
                    if not release_first_write.wait(timeout=2.0):
                        raise TimeoutError("test did not release first clock write")
                original_write(payload)

            clock._write_payload = delayed_first_write  # type: ignore[method-assign]
            monotonic.value += 1_000_000_000
            self.assertTrue(clock.checkpoint())
            self.assertTrue(first_write_started.wait(timeout=1.0))
            monotonic.value += 2_000_000_000
            self.assertTrue(clock.checkpoint())

            close_error: list[Exception] = []

            def close_clock() -> None:
                try:
                    clock.close()
                except Exception as exc:  # pragma: no cover - asserted below
                    close_error.append(exc)

            closer = threading.Thread(target=close_clock)
            closer.start()
            release_first_write.set()
            closer.join(timeout=2.0)
            self.assertFalse(closer.is_alive())
            self.assertEqual(close_error, [])
            saved = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(saved["elapsed_tai_ns"], 3_000_000_000)
            self.assertIsNotNone(clock._writer_thread)
            self.assertFalse(clock._writer_thread.is_alive())

    def test_clock_writer_failure_propagates_and_does_not_leak(self) -> None:
        monotonic = MutableMonotonicClock(1_000_000_000)
        with tempfile.TemporaryDirectory() as temporary:
            clock = EPHEMERIS.PersistentSimulationClock(
                universe_id="sol-2080",
                reference_epoch_utc="2080-01-01T00:00:00Z",
                tai_minus_utc_at_epoch_s=37,
                state_path=Path(temporary) / "clock.json",
                monotonic_ns=monotonic,
            )

            def failed_write(_payload: dict[str, object]) -> None:
                raise OSError("simulated disk failure")

            clock._write_payload = failed_write  # type: ignore[method-assign]
            monotonic.value += 1_000_000_000
            self.assertTrue(clock.checkpoint())
            with self.assertRaisesRegex(
                EPHEMERIS.CelestialEphemerisError,
                "simulated disk failure",
            ):
                clock.checkpoint(force=True)
            with self.assertRaisesRegex(
                EPHEMERIS.CelestialEphemerisError,
                "simulated disk failure",
            ):
                clock.close()
            self.assertIsNotNone(clock._writer_thread)
            self.assertFalse(clock._writer_thread.is_alive())

    def test_locked_ephemeris_assets_reject_tampering(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            kernel = root / "de440s.bsp"
            wheel = root / "jplephem-2.23-py3-none-any.whl"
            kernel.write_bytes(b"test-kernel")
            wheel.write_bytes(b"test-wheel")

            def asset(role: str, path: Path) -> dict[str, object]:
                payload = path.read_bytes()
                return {
                    "role": role,
                    "filename": path.name,
                    "size": len(payload),
                    "sha256": hashlib.sha256(payload).hexdigest(),
                    "urls": [f"https://example.invalid/{path.name}"],
                }

            manifest = root / "assets.json"
            manifest.write_text(
                json.dumps(
                    {
                        "schema": EPHEMERIS.ASSET_MANIFEST_SCHEMA,
                        "provider": EPHEMERIS.JPL_EPHEMERIS_PROVIDER,
                        "coverage": {
                            "start_utc": "2079-01-01T00:00:00Z",
                            "end_utc": "2081-01-01T00:00:00Z",
                        },
                        "assets": [
                            asset("de440s_spk", kernel),
                            asset("jplephem_wheel", wheel),
                        ],
                    }
                ),
                encoding="utf-8",
            )
            EPHEMERIS.verify_locked_ephemeris_assets(
                manifest,
                kernel_path=kernel,
                jplephem_wheel=wheel,
            )
            kernel.write_bytes(b"bad-kernel!")
            with self.assertRaisesRegex(
                EPHEMERIS.CelestialEphemerisError,
                "SHA256",
            ):
                EPHEMERIS.verify_locked_ephemeris_assets(
                    manifest,
                    kernel_path=kernel,
                    jplephem_wheel=wheel,
                )

    def test_repository_de440s_manifest_is_pinned(self) -> None:
        manifest = json.loads(
            NAVIGATION.DEFAULT_ASSET_MANIFEST_PATH.read_text(encoding="utf-8")
        )
        self.assertEqual(manifest["schema"], EPHEMERIS.ASSET_MANIFEST_SCHEMA)
        self.assertEqual(manifest["provider"], EPHEMERIS.JPL_EPHEMERIS_PROVIDER)
        assets = {asset["role"]: asset for asset in manifest["assets"]}
        self.assertEqual(assets["de440s_spk"]["size"], 32_726_016)
        self.assertEqual(
            assets["de440s_spk"]["sha256"],
            "c1c7feeab882263fc493a9d5a5b2ddd71b54826cdf65d8d17a76126b260a49f2",
        )
        self.assertEqual(assets["jplephem_wheel"]["size"], 49_368)

    def test_provisioned_de440s_matches_locked_2080_reference(self) -> None:
        root = (
            REPO_ROOT
            / "outputs/runtime/matrix-sonic-native-v2/celestial"
        )
        kernel = root / "de440s.bsp"
        wheel = root / "jplephem-2.23-py3-none-any.whl"
        if not kernel.is_file() or not wheel.is_file():
            self.skipTest("locked DE440s runtime is not provisioned")
        catalog = NAVIGATION.load_catalog(
            de440s_kernel=kernel,
            jplephem_wheel=wheel,
        )
        self.addCleanup(catalog.ephemeris.close)
        self.assertEqual(catalog.ephemeris_provider, "jpl-de440s-v1")
        states = catalog.ephemeris.states(0)
        expected_earth_m = (
            -23_887_718_789.791237,
            133_181_922_924.24686,
            57_713_368_520.74889,
        )
        self.assertLess(
            math.dist(states["earth"].center_inertial_m, expected_earth_m),
            1.0,
        )


if __name__ == "__main__":
    unittest.main()
