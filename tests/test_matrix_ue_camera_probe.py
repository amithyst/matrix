from __future__ import annotations

import hashlib
import importlib.util
import math
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts/matrix_ue_camera_probe.py"
SPEC = importlib.util.spec_from_file_location("matrix_ue_camera_probe", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def layout_mapping() -> dict[str, object]:
    return {
        "schema_version": 1,
        "binary": {
            "sha256": "11" * 32,
            "gnu_build_id": "22" * 8,
            "elf_type": "ET_EXEC",
        },
        "byte_order": "little",
        "pointer_size": 8,
        "player_index": 0,
        "addresses": {
            "gworld": 0x1000,
            "gworld_storage": "bare_pointer",
            "gworld_proxy_world_offset": 0,
        },
        "offsets": {
            "uworld_owning_game_instance": 0x10,
            "game_instance_local_players": 0x20,
            "tarray_data": 0,
            "tarray_num": 8,
            "tarray_max": 12,
            "local_player_player_controller": 0x30,
            "player_controller_camera_manager": 0x40,
            "camera_manager_camera_cache": 0x80,
            "camera_cache_timestamp": 0,
            "camera_cache_pov": 0x10,
            "pov_location": 0,
            "pov_rotation": 0x18,
        },
        "sizes": {"camera_cache": 96, "tarray": 16},
        "formats": {
            "pointer": "uint64",
            "pointer_representation": "raw_virtual_address",
            "tarray_count": "int32",
            "cache_timestamp": "float32",
            "location": "float64x3",
            "rotation": "float64x3",
        },
        "limits": {
            "min_pointer": 0x1000,
            "max_pointer": 0x0000FFFFFFFFFFFF,
            "max_local_players": 8,
            "max_abs_location": 1_000_000.0,
            "max_abs_angle_degrees": 1_000_000.0,
            "max_cache_timestamp": 1_000_000_000.0,
        },
    }


def camera_cache(
    *,
    timestamp: float = 12.5,
    location: tuple[float, float, float] = (1.0, 2.0, 3.0),
    rotation: tuple[float, float, float] = (10.0, 20.0, 30.0),
) -> bytes:
    result = bytearray(96)
    struct.pack_into("<f", result, 0, timestamp)
    struct.pack_into("<ddd", result, 0x10, *location)
    struct.pack_into("<ddd", result, 0x10 + 0x18, *rotation)
    return bytes(result)


class FakeMemory:
    def __init__(self) -> None:
        self.values: dict[tuple[int, int], bytes] = {}
        self.overrides: dict[tuple[int, int, int], bytes] = {}
        self.counts: dict[tuple[int, int], int] = {}

    def put(self, address: int, value: bytes) -> None:
        self.values[(address, len(value))] = value

    def put_pointer(self, address: int, value: int) -> None:
        self.put(address, struct.pack("<Q", value))

    def override(self, address: int, size: int, read_number: int, value: bytes) -> None:
        self.overrides[(address, size, read_number)] = value

    def read(self, pid: int, address: int, size: int) -> bytes:
        del pid
        key = (address, size)
        count = self.counts.get(key, 0) + 1
        self.counts[key] = count
        override = self.overrides.get((address, size, count))
        if override is not None:
            return override
        try:
            return self.values[key]
        except KeyError as exc:
            raise MODULE.RemoteMemoryReadError(
                f"no fake bytes at 0x{address:x} size={size}"
            ) from exc


class FakeIdentityVerifier:
    def __init__(self) -> None:
        self.verify_count = 0
        self.validate_count = 0
        self.fail_verify = False
        self.fail_validate = False

    def verify(self, pid: int, expected: object) -> object:
        del expected
        self.verify_count += 1
        if self.fail_verify:
            raise MODULE.BinaryIdentityError("wrong binary")
        return (pid, "verified")

    def validate(self, pid: int, token: object) -> None:
        self.validate_count += 1
        if self.fail_validate or token != (pid, "verified"):
            raise MODULE.BinaryIdentityError("PID reused")


def populated_memory() -> FakeMemory:
    memory = FakeMemory()
    memory.put_pointer(0x1000, 0x2000)
    memory.put_pointer(0x2010, 0x3000)
    memory.put(0x3020, struct.pack("<Qii", 0x4000, 1, 2))
    memory.put_pointer(0x4000, 0x5000)
    memory.put_pointer(0x5030, 0x6000)
    memory.put_pointer(0x6040, 0x7000)
    memory.put(0x7080, camera_cache())
    return memory


def observation(
    *,
    pid: int = 4242,
    monotonic_ns: int = 1_000_000_000,
    valid: bool = True,
    error: object = None,
    pitch: float = 10.0,
    yaw: float = 20.0,
    roll: float = 30.0,
) -> object:
    if error is None:
        error = MODULE.ProbeError.NONE if valid else MODULE.ProbeError.MEMORY_READ
    return MODULE.CameraProbeObservation(
        ue_pid=pid,
        monotonic_ns=monotonic_ns,
        valid=valid,
        error_code=error,
        pitch_deg=pitch,
        yaw_deg=yaw,
        roll_deg=roll,
        location_x=1.0,
        location_y=2.0,
        location_z=3.0,
        cache_timestamp_s=12.5,
    )


class LayoutTest(unittest.TestCase):
    def test_strict_mapping_loads_all_offsets(self) -> None:
        layout = MODULE.CameraMemoryLayout.from_mapping(layout_mapping())
        self.assertEqual(layout.gworld_address, 0x1000)
        self.assertEqual(layout.offsets["camera_manager_camera_cache"], 0x80)
        self.assertEqual(layout.binary.gnu_build_id, "22" * 8)

    def test_unknown_missing_and_boolean_fields_are_rejected(self) -> None:
        extra = layout_mapping()
        extra["shortcut"] = True
        with self.assertRaisesRegex(MODULE.LayoutError, "extra"):
            MODULE.CameraMemoryLayout.from_mapping(extra)

        missing = layout_mapping()
        del missing["offsets"]["pov_rotation"]  # type: ignore[index]
        with self.assertRaisesRegex(MODULE.LayoutError, "missing"):
            MODULE.CameraMemoryLayout.from_mapping(missing)

        boolean = layout_mapping()
        boolean["limits"]["max_local_players"] = True  # type: ignore[index]
        with self.assertRaisesRegex(MODULE.LayoutError, "integer"):
            MODULE.CameraMemoryLayout.from_mapping(boolean)

    def test_layout_rejects_pie_and_out_of_cache_offsets(self) -> None:
        pie = layout_mapping()
        pie["binary"]["elf_type"] = "ET_DYN"  # type: ignore[index]
        with self.assertRaisesRegex(MODULE.LayoutError, "ET_EXEC"):
            MODULE.CameraMemoryLayout.from_mapping(pie)

        overflow = layout_mapping()
        overflow["offsets"]["pov_rotation"] = 90  # type: ignore[index]
        with self.assertRaisesRegex(MODULE.LayoutError, "camera scalar offsets"):
            MODULE.CameraMemoryLayout.from_mapping(overflow)

    def test_pointer_representation_is_explicit_and_proxy_storage_is_supported(self) -> None:
        ambiguous = layout_mapping()
        ambiguous["formats"]["pointer_representation"] = "tobjectptr"  # type: ignore[index]
        with self.assertRaisesRegex(MODULE.LayoutError, "raw_virtual_address"):
            MODULE.CameraMemoryLayout.from_mapping(ambiguous)

        proxy = layout_mapping()
        proxy["addresses"]["gworld_storage"] = "proxy"  # type: ignore[index]
        proxy["addresses"]["gworld_proxy_world_offset"] = 8  # type: ignore[index]
        layout = MODULE.CameraMemoryLayout.from_mapping(proxy)
        self.assertEqual(layout.gworld_storage, "proxy")
        self.assertEqual(layout.gworld_proxy_world_offset, 8)

    def test_json_loader_rejects_duplicate_keys(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "layout.json"
            path.write_text('{"schema_version":1,"schema_version":1}', encoding="utf-8")
            with self.assertRaisesRegex(MODULE.LayoutError, "duplicate"):
                MODULE.load_layout(path)


class BinaryIdentityTest(unittest.TestCase):
    @staticmethod
    def elf_with_build_id(build_id: bytes, *, elf_type: int = 2) -> bytes:
        ident = b"\x7fELF" + bytes((2, 1, 1)) + bytes(9)
        note = struct.pack("<III", 4, len(build_id), 3) + b"GNU\0" + build_id
        while len(note) % 4:
            note += b"\0"
        header = struct.pack(
            "<16sHHIQQQIHHHHHH",
            ident,
            elf_type,
            62,
            1,
            0,
            64,
            0,
            0,
            64,
            56,
            1,
            0,
            0,
            0,
        )
        program = struct.pack("<IIQQQQQQ", 4, 4, 128, 0, 0, len(note), len(note), 4)
        return header + program + bytes(128 - len(header) - len(program)) + note

    def test_sha_build_id_and_et_exec_are_all_pinned(self) -> None:
        build_id = bytes.fromhex("0123456789abcdef")
        payload = self.elf_with_build_id(build_id)
        expected = MODULE.BinaryIdentity(
            sha256=hashlib.sha256(payload).hexdigest(),
            gnu_build_id=build_id.hex(),
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "ue"
            path.write_bytes(payload)
            MODULE.verify_binary_path(path, expected)

            wrong_hash = MODULE.BinaryIdentity("00" * 32, build_id.hex())
            with self.assertRaisesRegex(MODULE.BinaryIdentityError, "sha256"):
                MODULE.verify_binary_path(path, wrong_hash)

            path.write_bytes(self.elf_with_build_id(build_id, elf_type=3))
            with self.assertRaisesRegex(MODULE.BinaryIdentityError, "ET_EXEC"):
                MODULE.verify_binary_path(path, expected)


class ProbeTest(unittest.TestCase):
    def make_probe(
        self,
        memory: FakeMemory | None = None,
        identity: FakeIdentityVerifier | None = None,
        *,
        bind: bool = True,
    ) -> tuple[object, FakeMemory, FakeIdentityVerifier]:
        memory = memory or populated_memory()
        identity = identity or FakeIdentityVerifier()
        probe = MODULE.UECameraProbe(
            MODULE.CameraMemoryLayout.from_mapping(layout_mapping()),
            memory_reader=memory,
            identity_verifier=identity,
            monotonic_ns=lambda: 123_000_000,
        )
        if bind:
            probe.bind(4242)
        return probe, memory, identity

    def test_stable_double_read_returns_final_pov(self) -> None:
        probe, memory, identity = self.make_probe()
        not_ready = probe.sample(4242)
        self.assertEqual(not_ready.error_code, MODULE.ProbeError.CACHE_TIMESTAMP_NOT_READY)
        memory.put(0x7080, camera_cache(timestamp=12.75))
        result = probe.sample(4242)
        self.assertTrue(result.valid)
        self.assertEqual(result.error_code, MODULE.ProbeError.NONE)
        self.assertEqual((result.pitch_deg, result.yaw_deg, result.roll_deg), (10.0, 20.0, 30.0))
        self.assertEqual((result.location_x, result.location_y, result.location_z), (1.0, 2.0, 3.0))
        self.assertAlmostEqual(result.cache_timestamp_s, 12.75)
        self.assertEqual(identity.verify_count, 1)

        again = probe.sample(4242)
        self.assertTrue(again.valid)
        self.assertEqual(identity.verify_count, 1)
        self.assertEqual(identity.validate_count, 3)

    def test_proxy_gworld_slot_is_used_and_third_chain_is_checked(self) -> None:
        mapping = layout_mapping()
        mapping["addresses"]["gworld_storage"] = "proxy"  # type: ignore[index]
        mapping["addresses"]["gworld_proxy_world_offset"] = 8  # type: ignore[index]
        memory = populated_memory()
        del memory.values[(0x1000, 8)]
        memory.put_pointer(0x1008, 0x2000)
        probe = MODULE.UECameraProbe(
            MODULE.CameraMemoryLayout.from_mapping(mapping),
            memory_reader=memory,
            identity_verifier=FakeIdentityVerifier(),
            monotonic_ns=lambda: 123_000_000,
        )
        probe.bind(4242)
        first = probe.sample(4242)
        self.assertEqual(first.error_code, MODULE.ProbeError.CACHE_TIMESTAMP_NOT_READY)
        memory.put(0x7080, camera_cache(timestamp=12.75))
        self.assertTrue(probe.sample(4242).valid)

        memory = populated_memory()
        memory.override(0x1000, 8, 3, struct.pack("<Q", 0x2800))
        memory.put_pointer(0x2810, 0x3000)
        result = self.make_probe(memory)[0].sample(4242)
        self.assertEqual(result.error_code, MODULE.ProbeError.TORN_POINTER_CHAIN)

    def test_pointer_chain_change_is_rejected(self) -> None:
        probe, memory, _identity = self.make_probe()
        memory.override(0x1000, 8, 2, struct.pack("<Q", 0x2800))
        memory.put_pointer(0x2810, 0x3000)
        result = probe.sample(4242)
        self.assertFalse(result.valid)
        self.assertEqual(result.error_code, MODULE.ProbeError.TORN_POINTER_CHAIN)

    def test_any_camera_cache_change_is_rejected(self) -> None:
        probe, memory, _identity = self.make_probe()
        changed = camera_cache(rotation=(10.0, 21.0, 30.0))
        memory.override(0x7080, 96, 2, changed)
        result = probe.sample(4242)
        self.assertFalse(result.valid)
        self.assertEqual(result.error_code, MODULE.ProbeError.TORN_CAMERA_CACHE)

    def test_bad_pointer_and_tarray_bounds_fail_closed(self) -> None:
        bad_pointer = populated_memory()
        bad_pointer.put_pointer(0x6040, 0x7001)
        result = self.make_probe(bad_pointer)[0].sample(4242)
        self.assertEqual(result.error_code, MODULE.ProbeError.INVALID_POINTER)

        bad_array = populated_memory()
        bad_array.put(0x3020, struct.pack("<Qii", 0x4000, 9, 9))
        result = self.make_probe(bad_array)[0].sample(4242)
        self.assertEqual(result.error_code, MODULE.ProbeError.INVALID_TARRAY)

    def test_nonfinite_and_identity_mismatch_fail_closed(self) -> None:
        nonfinite = populated_memory()
        nonfinite.put(0x7080, camera_cache(rotation=(10.0, math.nan, 30.0)))
        result = self.make_probe(nonfinite)[0].sample(4242)
        self.assertEqual(result.error_code, MODULE.ProbeError.NONFINITE)

        identity = FakeIdentityVerifier()
        identity.fail_verify = True
        failed_probe = self.make_probe(identity=identity, bind=False)[0]
        with self.assertRaisesRegex(MODULE.BinaryIdentityError, "wrong binary"):
            failed_probe.bind(4242)
        result = failed_probe.sample(4242)
        self.assertEqual(result.error_code, MODULE.ProbeError.IDENTITY_MISMATCH)

    def test_cache_timestamp_regression_and_stall_are_fail_closed(self) -> None:
        now = [100]
        memory = populated_memory()
        probe = MODULE.UECameraProbe(
            MODULE.CameraMemoryLayout.from_mapping(layout_mapping()),
            memory_reader=memory,
            identity_verifier=FakeIdentityVerifier(),
            monotonic_ns=lambda: now[0],
            max_cache_stall_ns=5,
            cache_startup_grace_ns=5,
        )
        probe.bind(4242)
        self.assertEqual(
            probe.sample(4242).error_code, MODULE.ProbeError.CACHE_TIMESTAMP_NOT_READY
        )
        memory.put(0x7080, camera_cache(timestamp=13.0))
        now[0] = 101
        self.assertTrue(probe.sample(4242).valid)
        memory.put(0x7080, camera_cache(timestamp=12.0))
        now[0] = 102
        regressed = probe.sample(4242)
        self.assertEqual(regressed.error_code, MODULE.ProbeError.CACHE_TIMESTAMP_REGRESSION)

        now = [100]
        frozen = MODULE.UECameraProbe(
            MODULE.CameraMemoryLayout.from_mapping(layout_mapping()),
            memory_reader=populated_memory(),
            identity_verifier=FakeIdentityVerifier(),
            monotonic_ns=lambda: now[0],
            max_cache_stall_ns=5,
            cache_startup_grace_ns=5,
        )
        frozen.bind(4242)
        self.assertEqual(
            frozen.sample(4242).error_code, MODULE.ProbeError.CACHE_TIMESTAMP_NOT_READY
        )
        now[0] = 106
        stalled = frozen.sample(4242)
        self.assertEqual(stalled.error_code, MODULE.ProbeError.CACHE_TIMESTAMP_STALLED)

        paused = MODULE.UECameraProbe(
            MODULE.CameraMemoryLayout.from_mapping(layout_mapping()),
            memory_reader=populated_memory(),
            identity_verifier=FakeIdentityVerifier(),
            monotonic_ns=lambda: now[0],
            max_cache_stall_ns=None,
        )
        paused.bind(4242)
        self.assertEqual(
            paused.sample(4242).error_code, MODULE.ProbeError.CACHE_TIMESTAMP_NOT_READY
        )
        paused._memory.put(0x7080, camera_cache(timestamp=13.0))
        self.assertTrue(paused.sample(4242).valid)
        now[0] += 10_000_000_000
        self.assertTrue(paused.sample(4242).valid)

    def test_stable_world_transition_requires_a_fresh_timestamp_advance(self) -> None:
        memory = populated_memory()
        probe, _memory, _identity = self.make_probe(memory)
        self.assertEqual(
            probe.sample(4242).error_code,
            MODULE.ProbeError.CACHE_TIMESTAMP_NOT_READY,
        )
        memory.put(0x7080, camera_cache(timestamp=13.0))
        self.assertTrue(probe.sample(4242).valid)

        # Simulate the packaged startup world handing off to the requested
        # map.  Every pointer in the new chain is stable across the sample, but
        # its independent CameraCache timestamp starts from a lower value.
        memory.put_pointer(0x1000, 0x2200)
        memory.put_pointer(0x2210, 0x3200)
        memory.put(0x3220, struct.pack("<Qii", 0x4200, 1, 2))
        memory.put_pointer(0x4200, 0x5200)
        memory.put_pointer(0x5230, 0x6200)
        memory.put_pointer(0x6240, 0x7200)
        memory.put(0x7280, camera_cache(timestamp=1.0))

        first_new_world = probe.sample(4242)
        self.assertEqual(
            first_new_world.error_code,
            MODULE.ProbeError.CACHE_TIMESTAMP_NOT_READY,
        )
        memory.put(0x7280, camera_cache(timestamp=1.25))
        self.assertTrue(probe.sample(4242).valid)

        # A regression within that same stable chain remains fail-closed.
        memory.put(0x7280, camera_cache(timestamp=0.5))
        self.assertEqual(
            probe.sample(4242).error_code,
            MODULE.ProbeError.CACHE_TIMESTAMP_REGRESSION,
        )

    def test_short_custom_reader_is_a_memory_error(self) -> None:
        memory = populated_memory()
        memory.override(0x1000, 8, 1, b"short")
        result = self.make_probe(memory)[0].sample(4242)
        self.assertEqual(result.error_code, MODULE.ProbeError.MEMORY_READ)


class StateFileTest(unittest.TestCase):
    def test_live_validation_clock_is_sampled_after_locked_pread(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "camera.state"
            with MODULE.CameraStateWriter(path) as writer:
                writer.write(observation(monotonic_ns=1_000_000_010))

            order = []
            original_pread = MODULE._pread_state

            def ordered_pread(fd):
                order.append("pread")
                return original_pread(fd)

            def validation_clock():
                order.append("clock")
                return 1_000_000_020

            reader = MODULE.CameraStateReader(
                path,
                expected_ue_pid=4242,
                monotonic_ns=validation_clock,
            )
            with mock.patch.object(MODULE, "_pread_state", side_effect=ordered_pread):
                state = reader.read()

            self.assertIsNotNone(state)
            self.assertEqual(order, ["pread", "clock"])
            self.assertIsNone(reader.last_error)

    def test_live_validation_still_rejects_a_truly_future_record(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "camera.state"
            with MODULE.CameraStateWriter(path) as writer:
                writer.write(observation(monotonic_ns=2_000_000_020))
            reader = MODULE.CameraStateReader(
                path,
                expected_ue_pid=4242,
                monotonic_ns=lambda: 2_000_000_010,
            )

            self.assertIsNone(reader.read())
            self.assertEqual(reader.last_error, "future")

    def test_locked_writer_and_reader_enforce_pid_freshness_and_fields(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "camera.state"
            with MODULE.CameraStateWriter(path) as writer:
                written = writer.write(observation())
            self.assertEqual(written.sequence, 1)
            self.assertEqual(path.stat().st_size, MODULE.STATE_RECORD_SIZE)
            self.assertEqual(path.stat().st_mode & 0o777, 0o600)

            reader = MODULE.CameraStateReader(
                path, expected_ue_pid=4242, max_age_ns=100, angle_change_epsilon_deg=0.5
            )
            state = reader.read(1_000_000_050)
            assert state is not None
            self.assertEqual(state.ue_pid, 4242)
            self.assertEqual(state.yaw_deg, 20.0)
            self.assertEqual(state.sample_monotonic_s, 1.0)
            self.assertIsNone(reader.last_error)

            wrong_pid = MODULE.CameraStateReader(path, expected_ue_pid=4243)
            self.assertIsNone(wrong_pid.read(1_000_000_050))
            self.assertEqual(wrong_pid.last_error, "unexpected_ue_pid")

            stale = MODULE.CameraStateReader(path, expected_ue_pid=4242, max_age_ns=10)
            self.assertIsNone(stale.read(1_000_000_050))
            self.assertEqual(stale.last_error, "stale")

    def test_sequence_continues_and_wrapped_angle_change_is_detected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "camera.state"
            reader = MODULE.CameraStateReader(
                path,
                expected_ue_pid=4242,
                max_age_ns=1_000,
                angle_change_epsilon_deg=1.0,
            )
            with MODULE.CameraStateWriter(path) as writer:
                first = writer.write(observation(monotonic_ns=100, yaw=179.0))
                self.assertIsNotNone(reader.read(101))
                self.assertFalse(reader.angles_changed)
                second = writer.write(observation(monotonic_ns=110, yaw=-179.0))
                reading = reader.read_with_change(111)
            assert reading is not None
            self.assertEqual((first.sequence, second.sequence), (1, 2))
            self.assertTrue(reading.angles_changed)
            self.assertAlmostEqual(reading.max_angle_delta_deg, 2.0)

    def test_invalid_probe_record_corruption_and_future_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "camera.state"
            with MODULE.CameraStateWriter(path) as writer:
                writer.write(observation(valid=False, error=MODULE.ProbeError.MEMORY_READ))
                reader = MODULE.CameraStateReader(path, expected_ue_pid=4242)
                self.assertIsNone(reader.read(1_000_000_001))
                self.assertEqual(reader.last_error, "probe_error_2")
                writer.write(observation(monotonic_ns=2_000_000_000))
            self.assertIsNone(reader.read(1_999_999_999))
            self.assertEqual(reader.last_error, "future")

            raw = bytearray(path.read_bytes())
            raw[30] ^= 0x80
            path.write_bytes(raw)
            self.assertIsNone(reader.read(2_000_000_001))
            self.assertEqual(reader.last_error, "corrupt")

    def test_repeated_commit_sequence_catches_torn_record(self) -> None:
        state = MODULE.CameraProbeState(
            ue_pid=4242,
            sequence=7,
            monotonic_ns=100,
            pitch_deg=1.0,
            yaw_deg=2.0,
            roll_deg=3.0,
            location_x=4.0,
            location_y=5.0,
            location_z=6.0,
            cache_timestamp_s=7.0,
            valid=True,
            error_code=MODULE.ProbeError.NONE,
        )
        encoded = bytearray(MODULE._encode_state(state))
        struct.pack_into("<Q", encoded, MODULE.STATE_RECORD_SIZE - 12, 8)
        checksum = MODULE.zlib.crc32(encoded[:-4]) & 0xFFFFFFFF
        struct.pack_into("<I", encoded, MODULE.STATE_RECORD_SIZE - 4, checksum)
        with self.assertRaisesRegex(MODULE.StateFileError, "begin/end"):
            MODULE._decode_state(bytes(encoded))

    def test_probe_and_write_publishes_rejection_immediately(self) -> None:
        identity = FakeIdentityVerifier()
        identity.fail_verify = True
        probe, _memory, _identity = ProbeTest().make_probe(identity=identity, bind=False)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "camera.state"
            with MODULE.CameraStateWriter(path) as writer:
                state = MODULE.probe_and_write(probe, writer, 4242)
            self.assertFalse(state.valid)
            self.assertEqual(state.error_code, MODULE.ProbeError.IDENTITY_MISMATCH)

    def test_writer_exclusively_creates_one_private_single_link_inode(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "camera.state"
            with MODULE.CameraStateWriter(path):
                with self.assertRaisesRegex(MODULE.StateFileError, "cannot open"):
                    MODULE.CameraStateWriter(path)
            self.assertEqual(path.stat().st_nlink, 1)
            self.assertEqual(path.stat().st_mode & 0o777, 0o600)

    def test_reader_lock_contention_is_bounded_and_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "camera.state"
            with MODULE.CameraStateWriter(path) as writer:
                writer.write(observation())
                MODULE.fcntl.flock(
                    writer._fd, MODULE.fcntl.LOCK_EX | MODULE.fcntl.LOCK_NB
                )
                try:
                    reader = MODULE.CameraStateReader(
                        path,
                        expected_ue_pid=4242,
                        lock_timeout_ns=100_000,
                    )
                    self.assertIsNone(reader.read(1_000_000_001))
                    self.assertEqual(reader.last_error, "busy")
                finally:
                    MODULE.fcntl.flock(
                        writer._fd,
                        MODULE.fcntl.LOCK_UN | MODULE.fcntl.LOCK_NB,
                    )


if __name__ == "__main__":
    unittest.main()
