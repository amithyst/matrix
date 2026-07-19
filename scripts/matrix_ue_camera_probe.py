#!/usr/bin/env python3
"""Read Matrix UE's final camera cache without executing code in UE.

The intended caller is the supervisor that directly parents the packaged UE
process.  Linux then permits the supervisor to use ``process_vm_readv`` under
the usual Yama ``ptrace_scope=1`` policy.  This module never attaches with
ptrace, writes target memory, injects a library, or calls an Unreal function.

Every target address is supplied by a strict, build-pinned JSON layout.  A
sample is accepted only when two independently traversed pointer chains and
two complete camera-cache reads agree byte-for-byte.  Accepted and rejected
samples can be published through a small fixed-size, CRC-protected state file.
The state file uses ``flock`` plus positional I/O so readers never depend on a
shared file offset.
"""

from __future__ import annotations

import ctypes
from dataclasses import dataclass
from enum import IntEnum
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import stat
import struct
import time
from types import MappingProxyType
from typing import Callable, Mapping, Protocol
import zlib


LAYOUT_SCHEMA_VERSION = 1
STATE_MAGIC = b"MXUPOV1\0"
STATE_VERSION = 1
STATE_VALID = 1 << 0
DEFAULT_MAX_AGE_NS = 100_000_000
DEFAULT_ANGLE_CHANGE_EPSILON_DEG = 1.0e-4

_UINT64_MAX = (1 << 64) - 1
_MAX_LAYOUT_BYTES = 64 * 1024
_MAX_CAMERA_CACHE_BYTES = 1024 * 1024
_ELF64_HEADER = struct.Struct("<16sHHIQQQIHHHHHH")
_ELF64_PROGRAM_HEADER = struct.Struct("<IIQQQQQQ")
_ELF_NOTE_HEADER = struct.Struct("<III")
_PT_NOTE = 4
_NT_GNU_BUILD_ID = 3

# magic, version, record size, flags, error, UE PID, sequence, monotonic ns,
# cache timestamp, pitch/yaw/roll, location xyz, repeated commit sequence,
# CRC32.  The repeated sequence and checksum make a killed partial pwrite
# unambiguously invalid even after its flock is released by the kernel.
_STATE_WITHOUT_CRC = struct.Struct("<8sHHIIQQQdddddddQ")
_STATE_RECORD = struct.Struct("<8sHHIIQQQdddddddQI")
STATE_RECORD_SIZE = _STATE_RECORD.size


class LayoutError(ValueError):
    """The camera-memory layout is malformed or is not the fixed schema."""


class BinaryIdentityError(RuntimeError):
    """The supervised process does not match the pinned UE executable."""


class RemoteMemoryReadError(RuntimeError):
    """A remote read failed or returned fewer bytes than requested."""


class PointerValidationError(RuntimeError):
    """A pointer in the configured UE object chain is invalid."""


class TArrayValidationError(RuntimeError):
    """The LocalPlayers TArray header is not self-consistent."""


class StateFileError(RuntimeError):
    """A camera state record cannot be safely opened, encoded, or decoded."""


class _StateLockTimeout(StateFileError):
    pass


class ProbeError(IntEnum):
    """Stable error values stored in the binary state record."""

    NONE = 0
    IDENTITY_MISMATCH = 1
    MEMORY_READ = 2
    INVALID_POINTER = 3
    INVALID_TARRAY = 4
    TORN_POINTER_CHAIN = 5
    TORN_CAMERA_CACHE = 6
    NONFINITE = 7
    VALUE_OUT_OF_RANGE = 8
    INTERNAL = 9
    CACHE_TIMESTAMP_REGRESSION = 10
    CACHE_TIMESTAMP_STALLED = 11
    CACHE_TIMESTAMP_NOT_READY = 12


@dataclass(frozen=True)
class BinaryIdentity:
    sha256: str
    gnu_build_id: str
    elf_type: str = "ET_EXEC"


_OFFSET_KEYS = {
    "uworld_owning_game_instance",
    "game_instance_local_players",
    "tarray_data",
    "tarray_num",
    "tarray_max",
    "local_player_player_controller",
    "player_controller_camera_manager",
    "camera_manager_camera_cache",
    "camera_cache_timestamp",
    "camera_cache_pov",
    "pov_location",
    "pov_rotation",
}
_LIMIT_KEYS = {
    "min_pointer",
    "max_pointer",
    "max_local_players",
    "max_abs_location",
    "max_abs_angle_degrees",
    "max_cache_timestamp",
}
_FORMAT_VALUES = {
    "pointer": "uint64",
    # This v1 probe only accepts a build whose DWARF proves these fields are
    # canonical virtual addresses.  It never guesses TObjectPtr handle bits.
    "pointer_representation": "raw_virtual_address",
    "tarray_count": "int32",
    "cache_timestamp": "float32",
    "location": "float64x3",
    "rotation": "float64x3",
}


def _reject_duplicate_keys(pairs: list[tuple[str, object]]) -> dict[str, object]:
    value: dict[str, object] = {}
    for key, item in pairs:
        if key in value:
            raise LayoutError(f"duplicate JSON key: {key}")
        value[key] = item
    return value


def _exact_keys(value: object, expected: set[str], label: str) -> Mapping[str, object]:
    if not isinstance(value, dict):
        raise LayoutError(f"{label} must be an object")
    actual = set(value)
    if actual != expected:
        raise LayoutError(
            f"{label} keys differ from the fixed schema: "
            f"missing={sorted(expected - actual)} extra={sorted(actual - expected)}"
        )
    return value


def _integer(value: object, label: str, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise LayoutError(f"{label} must be an integer")
    if not minimum <= value <= maximum:
        raise LayoutError(f"{label} must be in [{minimum}, {maximum}]")
    return value


def _finite_number(value: object, label: str, minimum: float, maximum: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise LayoutError(f"{label} must be a number")
    result = float(value)
    if not math.isfinite(result) or not minimum <= result <= maximum:
        raise LayoutError(f"{label} must be finite and in [{minimum}, {maximum}]")
    return result


def _lower_hex(value: object, label: str, exact_length: int | None = None) -> str:
    if not isinstance(value, str) or value != value.lower():
        raise LayoutError(f"{label} must be a lowercase hexadecimal string")
    if exact_length is not None and len(value) != exact_length:
        raise LayoutError(f"{label} must contain exactly {exact_length} hex characters")
    if exact_length is None and (len(value) < 8 or len(value) > 128 or len(value) % 2):
        raise LayoutError(f"{label} must contain 8..128 even-numbered hex characters")
    try:
        bytes.fromhex(value)
    except ValueError as exc:
        raise LayoutError(f"{label} is not hexadecimal") from exc
    return value


@dataclass(frozen=True)
class CameraMemoryLayout:
    """Pinned offsets for one exact packaged UE executable."""

    binary: BinaryIdentity
    gworld_address: int
    gworld_storage: str
    gworld_proxy_world_offset: int
    offsets: Mapping[str, int]
    # This is sizeof(FCameraCacheEntry) from the matching debug artifact, not
    # merely the last scalar offset used below.  Reading the full object is what
    # lets the before/after byte comparison reject unrelated concurrent writes.
    camera_cache_size: int
    tarray_size: int
    player_index: int
    min_pointer: int
    max_pointer: int
    max_local_players: int
    max_abs_location: float
    max_abs_angle_degrees: float
    max_cache_timestamp: float

    @classmethod
    def from_mapping(cls, raw: object) -> "CameraMemoryLayout":
        root = _exact_keys(
            raw,
            {
                "schema_version",
                "binary",
                "byte_order",
                "pointer_size",
                "player_index",
                "addresses",
                "offsets",
                "sizes",
                "formats",
                "limits",
            },
            "layout",
        )
        if root["schema_version"] != LAYOUT_SCHEMA_VERSION:
            raise LayoutError(f"schema_version must be {LAYOUT_SCHEMA_VERSION}")
        if root["byte_order"] != "little":
            raise LayoutError("byte_order must be 'little'")
        if root["pointer_size"] != 8:
            raise LayoutError("pointer_size must be 8")

        binary_raw = _exact_keys(
            root["binary"], {"sha256", "gnu_build_id", "elf_type"}, "binary"
        )
        if binary_raw["elf_type"] != "ET_EXEC":
            raise LayoutError("binary.elf_type must be 'ET_EXEC' (non-PIE)")
        binary = BinaryIdentity(
            sha256=_lower_hex(binary_raw["sha256"], "binary.sha256", 64),
            gnu_build_id=_lower_hex(binary_raw["gnu_build_id"], "binary.gnu_build_id"),
        )

        addresses = _exact_keys(
            root["addresses"],
            {"gworld", "gworld_storage", "gworld_proxy_world_offset"},
            "addresses",
        )
        gworld = _integer(addresses["gworld"], "addresses.gworld", 1, _UINT64_MAX)
        gworld_storage = addresses["gworld_storage"]
        if gworld_storage not in ("bare_pointer", "proxy"):
            raise LayoutError(
                "addresses.gworld_storage must be 'bare_pointer' or 'proxy'"
            )
        proxy_world_offset = _integer(
            addresses["gworld_proxy_world_offset"],
            "addresses.gworld_proxy_world_offset",
            0,
            1 << 30,
        )
        if gworld_storage == "bare_pointer" and proxy_world_offset != 0:
            raise LayoutError(
                "addresses.gworld_proxy_world_offset must be zero for bare_pointer"
            )

        offsets_raw = _exact_keys(root["offsets"], _OFFSET_KEYS, "offsets")
        offsets = {
            key: _integer(value, f"offsets.{key}", 0, 1 << 30)
            for key, value in offsets_raw.items()
        }
        sizes = _exact_keys(root["sizes"], {"camera_cache", "tarray"}, "sizes")
        camera_cache_size = _integer(
            sizes["camera_cache"], "sizes.camera_cache", 1, _MAX_CAMERA_CACHE_BYTES
        )
        tarray_size = _integer(sizes["tarray"], "sizes.tarray", 16, 4096)

        formats = _exact_keys(root["formats"], set(_FORMAT_VALUES), "formats")
        for key, expected in _FORMAT_VALUES.items():
            if formats[key] != expected:
                raise LayoutError(f"formats.{key} must be {expected!r}")

        limits = _exact_keys(root["limits"], _LIMIT_KEYS, "limits")
        min_pointer = _integer(limits["min_pointer"], "limits.min_pointer", 1, _UINT64_MAX)
        max_pointer = _integer(
            limits["max_pointer"], "limits.max_pointer", min_pointer + 1, _UINT64_MAX
        )
        max_local_players = _integer(
            limits["max_local_players"], "limits.max_local_players", 1, 1024
        )
        player_index = _integer(
            root["player_index"], "player_index", 0, max_local_players - 1
        )
        max_abs_location = _finite_number(
            limits["max_abs_location"], "limits.max_abs_location", 1.0, 1.0e15
        )
        max_abs_angle = _finite_number(
            limits["max_abs_angle_degrees"],
            "limits.max_abs_angle_degrees",
            180.0,
            1.0e12,
        )
        max_cache_timestamp = _finite_number(
            limits["max_cache_timestamp"],
            "limits.max_cache_timestamp",
            1.0,
            1.0e15,
        )

        if gworld % 8:
            raise LayoutError("addresses.gworld must be 8-byte aligned")
        world_slot = gworld + proxy_world_offset
        if world_slot > _UINT64_MAX or world_slot % 8:
            raise LayoutError("configured GWorld pointer slot must be 8-byte aligned")
        if not min_pointer <= world_slot <= max_pointer - 8:
            raise LayoutError("addresses.gworld falls outside the configured address range")
        for key in ("tarray_data", "tarray_num", "tarray_max"):
            width = 8 if key == "tarray_data" else 4
            if offsets[key] + width > tarray_size:
                raise LayoutError(f"offsets.{key} exceeds sizes.tarray")

        timestamp_end = offsets["camera_cache_timestamp"] + 4
        location_end = offsets["camera_cache_pov"] + offsets["pov_location"] + 24
        rotation_end = offsets["camera_cache_pov"] + offsets["pov_rotation"] + 24
        if max(timestamp_end, location_end, rotation_end) > camera_cache_size:
            raise LayoutError("camera scalar offsets exceed sizes.camera_cache")

        return cls(
            binary=binary,
            gworld_address=gworld,
            gworld_storage=gworld_storage,
            gworld_proxy_world_offset=proxy_world_offset,
            offsets=MappingProxyType(offsets),
            camera_cache_size=camera_cache_size,
            tarray_size=tarray_size,
            player_index=player_index,
            min_pointer=min_pointer,
            max_pointer=max_pointer,
            max_local_players=max_local_players,
            max_abs_location=max_abs_location,
            max_abs_angle_degrees=max_abs_angle,
            max_cache_timestamp=max_cache_timestamp,
        )

    @classmethod
    def load(cls, path: str | os.PathLike[str]) -> "CameraMemoryLayout":
        layout_path = Path(path)
        flags = os.O_RDONLY | os.O_CLOEXEC
        if hasattr(os, "O_NOFOLLOW"):
            flags |= os.O_NOFOLLOW
        try:
            fd = os.open(layout_path, flags)
        except OSError as exc:
            raise LayoutError(f"cannot open layout {layout_path}: {exc}") from exc
        try:
            metadata = os.fstat(fd)
            if not stat.S_ISREG(metadata.st_mode):
                raise LayoutError(f"layout is not a regular file: {layout_path}")
            if metadata.st_size > _MAX_LAYOUT_BYTES:
                raise LayoutError(f"layout is unexpectedly large: {layout_path}")
            with os.fdopen(fd, "r", encoding="utf-8", closefd=False) as handle:
                raw = json.load(handle, object_pairs_hook=_reject_duplicate_keys)
        except (OSError, UnicodeError, json.JSONDecodeError) as exc:
            raise LayoutError(f"cannot parse layout {layout_path}: {exc}") from exc
        finally:
            os.close(fd)
        return cls.from_mapping(raw)


def load_layout(path: str | os.PathLike[str]) -> CameraMemoryLayout:
    return CameraMemoryLayout.load(path)


@dataclass(frozen=True)
class ProcessIdentityToken:
    pid: int
    start_time_ticks: int
    device: int
    inode: int
    size: int
    mtime_ns: int
    ctime_ns: int
    verification_duration_ns: int


class IdentityVerifier(Protocol):
    def verify(self, pid: int, expected: BinaryIdentity) -> object: ...

    def validate(self, pid: int, token: object) -> None: ...


def _pread_exact(fd: int, size: int, offset: int, label: str) -> bytes:
    if size < 0 or offset < 0:
        raise BinaryIdentityError(f"invalid {label} range")
    chunks: list[bytes] = []
    remaining = size
    position = offset
    while remaining:
        block = os.pread(fd, remaining, position)
        if not block:
            raise BinaryIdentityError(f"short read while reading {label}")
        chunks.append(block)
        remaining -= len(block)
        position += len(block)
    return b"".join(chunks)


def _sha256_fd(fd: int, size: int, *, deadline_ns: int | None = None) -> str:
    digest = hashlib.sha256()
    offset = 0
    while offset < size:
        if deadline_ns is not None and time.monotonic_ns() >= deadline_ns:
            raise BinaryIdentityError("UE executable hashing exceeded its startup budget")
        block = os.pread(fd, min(1024 * 1024, size - offset), offset)
        if not block:
            raise BinaryIdentityError("short read while hashing executable")
        digest.update(block)
        offset += len(block)
    return digest.hexdigest()


def _align4(value: int) -> int:
    return (value + 3) & ~3


def _elf_build_id(fd: int, file_size: int) -> str:
    header = _pread_exact(fd, _ELF64_HEADER.size, 0, "ELF header")
    unpacked = _ELF64_HEADER.unpack(header)
    ident = unpacked[0]
    if ident[:4] != b"\x7fELF" or ident[4] != 2 or ident[5] != 1 or ident[6] != 1:
        raise BinaryIdentityError("executable is not little-endian ELF64")
    elf_type = unpacked[1]
    program_offset = unpacked[5]
    program_entry_size = unpacked[9]
    program_count = unpacked[10]
    if elf_type != 2:
        raise BinaryIdentityError("executable is not ET_EXEC/non-PIE")
    if program_entry_size != _ELF64_PROGRAM_HEADER.size or not 1 <= program_count <= 4096:
        raise BinaryIdentityError("ELF program-header table is invalid")
    table_size = program_entry_size * program_count
    if program_offset > file_size or table_size > file_size - program_offset:
        raise BinaryIdentityError("ELF program-header table exceeds the file")
    table = _pread_exact(fd, table_size, program_offset, "ELF program headers")
    found: list[str] = []
    for index in range(program_count):
        start = index * program_entry_size
        fields = _ELF64_PROGRAM_HEADER.unpack_from(table, start)
        segment_type, segment_offset, segment_size = fields[0], fields[2], fields[5]
        if segment_type != _PT_NOTE:
            continue
        if segment_size > 16 * 1024 * 1024:
            raise BinaryIdentityError("ELF PT_NOTE segment is unexpectedly large")
        if segment_offset > file_size or segment_size > file_size - segment_offset:
            raise BinaryIdentityError("ELF PT_NOTE segment exceeds the file")
        notes = _pread_exact(fd, segment_size, segment_offset, "ELF PT_NOTE segment")
        cursor = 0
        while cursor < len(notes):
            if len(notes) - cursor < _ELF_NOTE_HEADER.size:
                if any(notes[cursor:]):
                    raise BinaryIdentityError("truncated ELF note header")
                break
            name_size, description_size, note_type = _ELF_NOTE_HEADER.unpack_from(notes, cursor)
            cursor += _ELF_NOTE_HEADER.size
            name_end = cursor + name_size
            description_start = cursor + _align4(name_size)
            description_end = description_start + description_size
            next_note = description_start + _align4(description_size)
            if name_end > len(notes) or description_end > len(notes) or next_note > len(notes):
                raise BinaryIdentityError("truncated ELF note payload")
            name = notes[cursor:name_end]
            description = notes[description_start:description_end]
            if note_type == _NT_GNU_BUILD_ID and name.rstrip(b"\0") == b"GNU":
                if not description:
                    raise BinaryIdentityError("GNU build ID is empty")
                found.append(description.hex())
            cursor = next_note
    if len(found) != 1:
        raise BinaryIdentityError(f"expected one GNU build ID, found {len(found)}")
    return found[0]


def verify_binary_fd(
    fd: int,
    expected: BinaryIdentity,
    *,
    deadline_ns: int | None = None,
) -> os.stat_result:
    metadata = os.fstat(fd)
    if not stat.S_ISREG(metadata.st_mode):
        raise BinaryIdentityError("executable is not a regular file")
    actual_build_id = _elf_build_id(fd, metadata.st_size)
    if deadline_ns is not None and time.monotonic_ns() >= deadline_ns:
        raise BinaryIdentityError("UE executable verification exceeded its startup budget")
    if actual_build_id != expected.gnu_build_id:
        raise BinaryIdentityError(
            f"GNU build ID mismatch: expected={expected.gnu_build_id} actual={actual_build_id}"
        )
    actual_sha256 = _sha256_fd(fd, metadata.st_size, deadline_ns=deadline_ns)
    if actual_sha256 != expected.sha256:
        raise BinaryIdentityError(
            f"executable sha256 mismatch: expected={expected.sha256} actual={actual_sha256}"
        )
    return metadata


def verify_binary_path(path: str | os.PathLike[str], expected: BinaryIdentity) -> None:
    flags = os.O_RDONLY | os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        fd = os.open(path, flags)
    except OSError as exc:
        raise BinaryIdentityError(f"cannot open executable {path}: {exc}") from exc
    try:
        verify_binary_fd(fd, expected)
    finally:
        os.close(fd)


def _process_start_time_ticks(pid: int) -> int:
    try:
        raw = Path(f"/proc/{pid}/stat").read_text(encoding="ascii")
    except (OSError, UnicodeError) as exc:
        raise BinaryIdentityError(f"cannot read process identity for PID {pid}: {exc}") from exc
    closing = raw.rfind(")")
    if closing < 0:
        raise BinaryIdentityError(f"malformed /proc/{pid}/stat")
    fields = raw[closing + 2 :].split()
    if len(fields) <= 19:
        raise BinaryIdentityError(f"truncated /proc/{pid}/stat")
    try:
        return int(fields[19])
    except ValueError as exc:
        raise BinaryIdentityError(f"invalid process start time for PID {pid}") from exc


class ProcessBinaryIdentityVerifier:
    """Hash and build-ID pin ``/proc/PID/exe``, then guard against PID reuse."""

    def __init__(
        self,
        *,
        max_verification_duration_ns: int = 30_000_000_000,
        on_verified: Callable[[ProcessIdentityToken], None] | None = None,
    ) -> None:
        if (
            isinstance(max_verification_duration_ns, bool)
            or not isinstance(max_verification_duration_ns, int)
            or max_verification_duration_ns <= 0
        ):
            raise ValueError("max_verification_duration_ns must be positive")
        self.max_verification_duration_ns = max_verification_duration_ns
        self.on_verified = on_verified
        self.last_verification: ProcessIdentityToken | None = None

    def verify(self, pid: int, expected: BinaryIdentity) -> ProcessIdentityToken:
        if isinstance(pid, bool) or not isinstance(pid, int) or pid <= 0:
            raise BinaryIdentityError("UE PID must be a positive integer")
        verification_started = time.monotonic_ns()
        verification_deadline = verification_started + self.max_verification_duration_ns
        start_before = _process_start_time_ticks(pid)
        try:
            fd = os.open(f"/proc/{pid}/exe", os.O_RDONLY | os.O_CLOEXEC)
        except OSError as exc:
            raise BinaryIdentityError(f"cannot open /proc/{pid}/exe: {exc}") from exc
        try:
            metadata = verify_binary_fd(fd, expected, deadline_ns=verification_deadline)
        finally:
            os.close(fd)
        start_after = _process_start_time_ticks(pid)
        if start_before != start_after:
            raise BinaryIdentityError("UE process changed while verifying its executable")
        duration = time.monotonic_ns() - verification_started
        if duration > self.max_verification_duration_ns:
            raise BinaryIdentityError(
                "UE executable verification exceeded its bounded startup budget: "
                f"duration_ns={duration} limit_ns={self.max_verification_duration_ns}"
            )
        token = ProcessIdentityToken(
            pid=pid,
            start_time_ticks=start_after,
            device=metadata.st_dev,
            inode=metadata.st_ino,
            size=metadata.st_size,
            mtime_ns=metadata.st_mtime_ns,
            ctime_ns=metadata.st_ctime_ns,
            verification_duration_ns=duration,
        )
        self.last_verification = token
        if self.on_verified is not None:
            self.on_verified(token)
        return token

    def validate(self, pid: int, token: object) -> None:
        if not isinstance(token, ProcessIdentityToken) or token.pid != pid:
            raise BinaryIdentityError("invalid cached process identity token")
        if _process_start_time_ticks(pid) != token.start_time_ticks:
            raise BinaryIdentityError("UE PID was reused")
        try:
            metadata = os.stat(f"/proc/{pid}/exe")
        except OSError as exc:
            raise BinaryIdentityError(f"cannot revalidate /proc/{pid}/exe: {exc}") from exc
        actual = (
            metadata.st_dev,
            metadata.st_ino,
            metadata.st_size,
            metadata.st_mtime_ns,
            metadata.st_ctime_ns,
        )
        expected = (
            token.device,
            token.inode,
            token.size,
            token.mtime_ns,
            token.ctime_ns,
        )
        if actual != expected:
            raise BinaryIdentityError("UE executable identity changed after verification")


class RemoteMemoryReader(Protocol):
    def read(self, pid: int, address: int, size: int) -> bytes: ...


class _IOVec(ctypes.Structure):
    _fields_ = (("base", ctypes.c_void_p), ("length", ctypes.c_size_t))


class ProcessVmMemoryReader:
    """Exact remote reads using Linux ``process_vm_readv(2)``."""

    def __init__(self, libc: object | None = None) -> None:
        self._libc = libc if libc is not None else ctypes.CDLL(None, use_errno=True)
        try:
            function = self._libc.process_vm_readv
        except AttributeError as exc:
            raise RemoteMemoryReadError("libc does not expose process_vm_readv") from exc
        function.argtypes = (
            ctypes.c_int,
            ctypes.POINTER(_IOVec),
            ctypes.c_ulong,
            ctypes.POINTER(_IOVec),
            ctypes.c_ulong,
            ctypes.c_ulong,
        )
        function.restype = ctypes.c_ssize_t
        self._readv = function

    def read(self, pid: int, address: int, size: int) -> bytes:
        if isinstance(pid, bool) or not isinstance(pid, int) or pid <= 0:
            raise RemoteMemoryReadError("PID must be a positive integer")
        if isinstance(address, bool) or not isinstance(address, int) or address <= 0:
            raise RemoteMemoryReadError("remote address must be a positive integer")
        if (
            isinstance(size, bool)
            or not isinstance(size, int)
            or not 1 <= size <= _MAX_CAMERA_CACHE_BYTES
        ):
            raise RemoteMemoryReadError("remote read size is outside the supported range")
        destination = ctypes.create_string_buffer(size)
        local = _IOVec(ctypes.cast(destination, ctypes.c_void_p), size)
        remote = _IOVec(ctypes.c_void_p(address), size)
        ctypes.set_errno(0)
        result = self._readv(pid, ctypes.byref(local), 1, ctypes.byref(remote), 1, 0)
        if result != size:
            error = ctypes.get_errno()
            if result < 0:
                raise RemoteMemoryReadError(
                    f"process_vm_readv PID {pid} address 0x{address:x} failed: "
                    f"errno={error} ({os.strerror(error)})"
                )
            raise RemoteMemoryReadError(
                f"process_vm_readv PID {pid} address 0x{address:x} was short: "
                f"expected={size} actual={result}"
            )
        return destination.raw


@dataclass(frozen=True)
class PointerChain:
    world: int
    game_instance: int
    local_players_data: int
    local_players_num: int
    local_players_max: int
    local_player: int
    player_controller: int
    camera_manager: int
    camera_cache: int


@dataclass(frozen=True)
class CameraProbeObservation:
    ue_pid: int
    monotonic_ns: int
    valid: bool
    error_code: ProbeError
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    roll_deg: float = 0.0
    location_x: float = 0.0
    location_y: float = 0.0
    location_z: float = 0.0
    cache_timestamp_s: float = 0.0


@dataclass
class _CacheProgress:
    timestamp_s: float
    first_seen_monotonic_ns: int
    last_advance_monotonic_ns: int
    has_advanced: bool = False


class UECameraProbe:
    """Build-pinned, fail-closed reader for PlayerCameraManager CameraCache."""

    def __init__(
        self,
        layout: CameraMemoryLayout,
        *,
        memory_reader: RemoteMemoryReader | None = None,
        identity_verifier: IdentityVerifier | None = None,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        max_cache_stall_ns: int | None = 500_000_000,
        cache_startup_grace_ns: int = 2_000_000_000,
        cache_timestamp_epsilon_s: float = 1.0e-6,
    ) -> None:
        if not isinstance(layout, CameraMemoryLayout):
            raise TypeError("layout must be CameraMemoryLayout")
        self.layout = layout
        self._memory = memory_reader if memory_reader is not None else ProcessVmMemoryReader()
        self._identity = (
            identity_verifier if identity_verifier is not None else ProcessBinaryIdentityVerifier()
        )
        self._monotonic_ns = monotonic_ns
        self._identity_tokens: dict[int, object] = {}
        self.identity_verification_duration_ns: dict[int, int] = {}
        if max_cache_stall_ns is not None and (
            isinstance(max_cache_stall_ns, bool)
            or not isinstance(max_cache_stall_ns, int)
            or max_cache_stall_ns <= 0
        ):
            raise ValueError("max_cache_stall_ns must be positive or None for paused UE")
        if (
            isinstance(cache_startup_grace_ns, bool)
            or not isinstance(cache_startup_grace_ns, int)
            or cache_startup_grace_ns <= 0
        ):
            raise ValueError("cache_startup_grace_ns must be positive")
        if (
            not math.isfinite(cache_timestamp_epsilon_s)
            or cache_timestamp_epsilon_s < 0.0
        ):
            raise ValueError("cache_timestamp_epsilon_s must be finite and non-negative")
        self.max_cache_stall_ns = max_cache_stall_ns
        self.cache_startup_grace_ns = cache_startup_grace_ns
        self.cache_timestamp_epsilon_s = cache_timestamp_epsilon_s
        self._cache_progress: dict[int, _CacheProgress] = {}

    def reset_identity(self, pid: int | None = None) -> None:
        if pid is None:
            self._identity_tokens.clear()
            self._cache_progress.clear()
        else:
            self._identity_tokens.pop(pid, None)
            self._cache_progress.pop(pid, None)

    def reset_cache_progress(self, pid: int | None = None) -> None:
        """Authorize a known UE world restart or a paused-to-running transition."""

        if pid is None:
            self._cache_progress.clear()
        else:
            self._cache_progress.pop(pid, None)

    def bind(self, pid: int) -> int | None:
        """Pin one child executable once, before entering the sampling loop.

        Hashing a 322 MB packaged executable inside ``sample`` would stall the
        sole supervisor at an unpredictable moment.  Callers therefore bind
        explicitly during their bounded startup phase and can report the
        returned duration.  Subsequent samples only perform the cheap PID-reuse
        validation.
        """

        if isinstance(pid, bool) or not isinstance(pid, int) or pid <= 0:
            raise BinaryIdentityError("UE PID must be a positive integer")
        if pid in self._identity_tokens:
            self._identity.validate(pid, self._identity_tokens[pid])
            return self.identity_verification_duration_ns.get(pid)
        token = self._identity.verify(pid, self.layout.binary)
        self._identity_tokens[pid] = token
        duration = getattr(token, "verification_duration_ns", None)
        if isinstance(duration, int) and duration >= 0:
            self.identity_verification_duration_ns[pid] = duration
            return duration
        return None

    def _validate_identity(self, pid: int) -> None:
        if pid not in self._identity_tokens:
            raise BinaryIdentityError("UE PID has not completed explicit probe binding")
        self._identity.validate(pid, self._identity_tokens[pid])

    def _address(self, base: int, offset: int, size: int, label: str) -> int:
        self._pointer(base, label)
        if offset < 0 or size <= 0:
            raise PointerValidationError(f"invalid {label} field range")
        if offset > self.layout.max_pointer - base:
            raise PointerValidationError(f"{label} field address overflows")
        address = base + offset
        if address < self.layout.min_pointer or size > self.layout.max_pointer - address + 1:
            raise PointerValidationError(f"{label} field exceeds the configured address range")
        return address

    def _pointer(self, value: int, label: str) -> int:
        if (
            value == 0
            or value % 8
            or value < self.layout.min_pointer
            or value > self.layout.max_pointer - 7
        ):
            raise PointerValidationError(f"invalid {label} pointer: 0x{value:x}")
        return value

    def _read(self, pid: int, address: int, size: int) -> bytes:
        data = self._memory.read(pid, address, size)
        if not isinstance(data, bytes) or len(data) != size:
            raise RemoteMemoryReadError(
                f"memory reader returned {len(data) if isinstance(data, bytes) else 'non-bytes'} "
                f"for a {size}-byte read"
            )
        return data

    def _read_pointer_at(self, pid: int, address: int, label: str) -> int:
        raw = self._read(pid, address, 8)
        return self._pointer(struct.unpack("<Q", raw)[0], label)

    def _field_pointer(self, pid: int, base: int, offset_key: str, label: str) -> int:
        address = self._address(base, self.layout.offsets[offset_key], 8, label)
        return self._read_pointer_at(pid, address, label)

    def _read_chain(self, pid: int) -> PointerChain:
        world_slot = self.layout.gworld_address
        if self.layout.gworld_storage == "proxy":
            world_slot += self.layout.gworld_proxy_world_offset
        world = self._read_pointer_at(pid, world_slot, "GWorld")
        game_instance = self._field_pointer(
            pid, world, "uworld_owning_game_instance", "OwningGameInstance"
        )
        array_address = self._address(
            game_instance,
            self.layout.offsets["game_instance_local_players"],
            self.layout.tarray_size,
            "LocalPlayers",
        )
        array = self._read(pid, array_address, self.layout.tarray_size)
        data = struct.unpack_from("<Q", array, self.layout.offsets["tarray_data"])[0]
        count = struct.unpack_from("<i", array, self.layout.offsets["tarray_num"])[0]
        capacity = struct.unpack_from("<i", array, self.layout.offsets["tarray_max"])[0]
        if (
            count <= self.layout.player_index
            or count < 0
            or capacity < count
            or capacity > self.layout.max_local_players
        ):
            raise TArrayValidationError(
                f"invalid LocalPlayers TArray: num={count} max={capacity} "
                f"player_index={self.layout.player_index}"
            )
        data = self._pointer(data, "LocalPlayers.Data")
        if capacity > (self.layout.max_pointer - data + 1) // 8:
            raise TArrayValidationError("LocalPlayers allocation range overflows")
        player_slot = data + self.layout.player_index * 8
        local_player = self._read_pointer_at(pid, player_slot, "LocalPlayer")
        player_controller = self._field_pointer(
            pid, local_player, "local_player_player_controller", "PlayerController"
        )
        camera_manager = self._field_pointer(
            pid,
            player_controller,
            "player_controller_camera_manager",
            "PlayerCameraManager",
        )
        camera_cache = self._address(
            camera_manager,
            self.layout.offsets["camera_manager_camera_cache"],
            self.layout.camera_cache_size,
            "CameraCachePrivate",
        )
        return PointerChain(
            world=world,
            game_instance=game_instance,
            local_players_data=data,
            local_players_num=count,
            local_players_max=capacity,
            local_player=local_player,
            player_controller=player_controller,
            camera_manager=camera_manager,
            camera_cache=camera_cache,
        )

    def _decode_cache(self, cache: bytes) -> tuple[float, ...]:
        offsets = self.layout.offsets
        timestamp = float(struct.unpack_from("<f", cache, offsets["camera_cache_timestamp"])[0])
        pov = offsets["camera_cache_pov"]
        location = struct.unpack_from("<ddd", cache, pov + offsets["pov_location"])
        rotation = struct.unpack_from("<ddd", cache, pov + offsets["pov_rotation"])
        values = (timestamp, *rotation, *location)
        if not all(math.isfinite(value) for value in values):
            raise _SampleError(ProbeError.NONFINITE, "camera cache contains a non-finite value")
        if timestamp < 0.0 or timestamp > self.layout.max_cache_timestamp:
            raise _SampleError(
                ProbeError.VALUE_OUT_OF_RANGE,
                "camera cache timestamp is out of range",
            )
        if any(abs(value) > self.layout.max_abs_angle_degrees for value in rotation):
            raise _SampleError(ProbeError.VALUE_OUT_OF_RANGE, "camera rotation is out of range")
        if any(abs(value) > self.layout.max_abs_location for value in location):
            raise _SampleError(ProbeError.VALUE_OUT_OF_RANGE, "camera location is out of range")
        return values

    def _check_cache_progress(self, pid: int, timestamp_s: float, now_ns: int) -> None:
        progress = self._cache_progress.get(pid)
        if progress is None:
            self._cache_progress[pid] = _CacheProgress(
                timestamp_s=timestamp_s,
                first_seen_monotonic_ns=now_ns,
                last_advance_monotonic_ns=now_ns,
            )
            raise _SampleError(
                ProbeError.CACHE_TIMESTAMP_NOT_READY,
                "UE CameraCachePrivate.Timestamp has not yet demonstrated one advance",
            )
        epsilon = self.cache_timestamp_epsilon_s
        if timestamp_s < progress.timestamp_s - epsilon:
            raise _SampleError(
                ProbeError.CACHE_TIMESTAMP_REGRESSION,
                "UE CameraCachePrivate.Timestamp regressed; reset requires explicit authorization",
            )
        if timestamp_s > progress.timestamp_s + epsilon:
            progress.timestamp_s = timestamp_s
            progress.last_advance_monotonic_ns = now_ns
            progress.has_advanced = True
            return
        if now_ns < progress.last_advance_monotonic_ns:
            raise _SampleError(ProbeError.INTERNAL, "local monotonic clock regressed")
        if self.max_cache_stall_ns is None:
            if progress.has_advanced:
                return
            raise _SampleError(
                ProbeError.CACHE_TIMESTAMP_NOT_READY,
                "paused UE has not yet demonstrated an initial camera-cache advance",
            )
        limit = (
            self.max_cache_stall_ns
            if progress.has_advanced
            else self.cache_startup_grace_ns
        )
        if now_ns - progress.last_advance_monotonic_ns > limit:
            raise _SampleError(
                ProbeError.CACHE_TIMESTAMP_STALLED,
                "UE CameraCachePrivate.Timestamp stopped advancing",
            )
        if not progress.has_advanced:
            raise _SampleError(
                ProbeError.CACHE_TIMESTAMP_NOT_READY,
                "UE CameraCachePrivate.Timestamp is still in startup grace",
            )

    def sample(self, ue_pid: int) -> CameraProbeObservation:
        now = self._monotonic_ns()
        if isinstance(ue_pid, bool) or not isinstance(ue_pid, int) or ue_pid <= 0:
            return CameraProbeObservation(
                ue_pid=0, monotonic_ns=now, valid=False, error_code=ProbeError.IDENTITY_MISMATCH
            )
        try:
            self._validate_identity(ue_pid)
            first_chain = self._read_chain(ue_pid)
            first_cache = self._read(
                ue_pid, first_chain.camera_cache, self.layout.camera_cache_size
            )
            second_chain = self._read_chain(ue_pid)
            if second_chain != first_chain:
                raise _SampleError(
                    ProbeError.TORN_POINTER_CHAIN,
                    "UE camera pointer chain changed during the sample",
                )
            second_cache = self._read(
                ue_pid, second_chain.camera_cache, self.layout.camera_cache_size
            )
            if second_cache != first_cache:
                raise _SampleError(
                    ProbeError.TORN_CAMERA_CACHE,
                    "UE camera cache changed during the sample",
                )
            third_chain = self._read_chain(ue_pid)
            if third_chain != first_chain:
                raise _SampleError(
                    ProbeError.TORN_POINTER_CHAIN,
                    "UE camera pointer chain changed after the second cache read",
                )
            timestamp, pitch, yaw, roll, x, y, z = self._decode_cache(second_cache)
            accepted_ns = self._monotonic_ns()
            self._check_cache_progress(ue_pid, timestamp, accepted_ns)
            return CameraProbeObservation(
                ue_pid=ue_pid,
                monotonic_ns=accepted_ns,
                valid=True,
                error_code=ProbeError.NONE,
                pitch_deg=pitch,
                yaw_deg=yaw,
                roll_deg=roll,
                location_x=x,
                location_y=y,
                location_z=z,
                cache_timestamp_s=timestamp,
            )
        except BinaryIdentityError:
            code = ProbeError.IDENTITY_MISMATCH
        except RemoteMemoryReadError:
            code = ProbeError.MEMORY_READ
        except PointerValidationError:
            code = ProbeError.INVALID_POINTER
        except TArrayValidationError:
            code = ProbeError.INVALID_TARRAY
        except _SampleError as exc:
            code = exc.code
        except Exception:
            code = ProbeError.INTERNAL
        return CameraProbeObservation(
            ue_pid=ue_pid,
            monotonic_ns=self._monotonic_ns(),
            valid=False,
            error_code=code,
        )


class _SampleError(RuntimeError):
    def __init__(self, code: ProbeError, message: str) -> None:
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class CameraProbeState:
    ue_pid: int
    sequence: int
    monotonic_ns: int
    pitch_deg: float
    yaw_deg: float
    roll_deg: float
    location_x: float
    location_y: float
    location_z: float
    cache_timestamp_s: float
    valid: bool
    error_code: ProbeError

    @property
    def sample_monotonic_s(self) -> float:
        return self.monotonic_ns / 1_000_000_000.0


@dataclass(frozen=True)
class CameraStateReading:
    state: CameraProbeState
    angles_changed: bool
    max_angle_delta_deg: float


def _encode_state(state: CameraProbeState) -> bytes:
    if state.ue_pid <= 0 or state.sequence <= 0 or state.monotonic_ns <= 0:
        raise StateFileError("state PID, sequence, and monotonic timestamp must be positive")
    if state.sequence > _UINT64_MAX or state.monotonic_ns > _UINT64_MAX:
        raise StateFileError("state sequence or timestamp exceeds uint64")
    try:
        error = ProbeError(state.error_code)
    except ValueError as exc:
        raise StateFileError("state has an unknown error code") from exc
    if state.valid != (error == ProbeError.NONE):
        raise StateFileError("state valid flag and error code disagree")
    values = (
        state.cache_timestamp_s,
        state.pitch_deg,
        state.yaw_deg,
        state.roll_deg,
        state.location_x,
        state.location_y,
        state.location_z,
    )
    if not all(math.isfinite(value) for value in values):
        raise StateFileError("state contains a non-finite value")
    flags = STATE_VALID if state.valid else 0
    prefix = _STATE_WITHOUT_CRC.pack(
        STATE_MAGIC,
        STATE_VERSION,
        STATE_RECORD_SIZE,
        flags,
        int(error),
        state.ue_pid,
        state.sequence,
        state.monotonic_ns,
        *values,
        state.sequence,
    )
    checksum = zlib.crc32(prefix) & 0xFFFFFFFF
    return prefix + struct.pack("<I", checksum)


def _decode_state(raw: bytes) -> CameraProbeState:
    if len(raw) != STATE_RECORD_SIZE:
        raise StateFileError(
            f"camera state has wrong size: expected={STATE_RECORD_SIZE} actual={len(raw)}"
        )
    fields = _STATE_RECORD.unpack(raw)
    magic, version, record_size, flags, raw_error = fields[:5]
    if magic != STATE_MAGIC or version != STATE_VERSION or record_size != STATE_RECORD_SIZE:
        raise StateFileError("camera state header is invalid")
    if flags & ~STATE_VALID:
        raise StateFileError("camera state contains unknown flags")
    expected_crc = zlib.crc32(raw[:-4]) & 0xFFFFFFFF
    if fields[-1] != expected_crc:
        raise StateFileError("camera state checksum mismatch")
    try:
        error = ProbeError(raw_error)
    except ValueError as exc:
        raise StateFileError("camera state has an unknown error code") from exc
    valid = bool(flags & STATE_VALID)
    if valid != (error == ProbeError.NONE):
        raise StateFileError("camera state valid flag and error code disagree")
    pid, sequence, monotonic_ns = fields[5:8]
    timestamp, pitch, yaw, roll, x, y, z = fields[8:15]
    if fields[15] != sequence:
        raise StateFileError("camera state begin/end sequences disagree")
    if pid <= 0 or sequence <= 0 or monotonic_ns <= 0:
        raise StateFileError("camera state identity fields are invalid")
    if not all(math.isfinite(value) for value in (timestamp, pitch, yaw, roll, x, y, z)):
        raise StateFileError("camera state contains a non-finite value")
    return CameraProbeState(
        ue_pid=pid,
        sequence=sequence,
        monotonic_ns=monotonic_ns,
        pitch_deg=pitch,
        yaw_deg=yaw,
        roll_deg=roll,
        location_x=x,
        location_y=y,
        location_z=z,
        cache_timestamp_s=timestamp,
        valid=valid,
        error_code=error,
    )


def _require_absolute_state_path(path: str | os.PathLike[str]) -> Path:
    result = Path(path)
    if not result.is_absolute() or ".." in result.parts:
        raise StateFileError("camera state path must be absolute and contain no '..'")
    return result


def _validate_state_fd(fd: int) -> os.stat_result:
    metadata = os.fstat(fd)
    if not stat.S_ISREG(metadata.st_mode):
        raise StateFileError("camera state is not a regular file")
    if metadata.st_uid != os.getuid():
        raise StateFileError("camera state is not owned by the current UID")
    if metadata.st_nlink != 1:
        raise StateFileError("camera state must have exactly one hard link")
    if stat.S_IMODE(metadata.st_mode) != 0o600:
        raise StateFileError("camera state mode must be exactly 0600")
    return metadata


def _bounded_flock(fd: int, operation: int, timeout_ns: int) -> None:
    """Acquire an advisory lock without ever blocking the supervisor forever."""

    deadline = time.monotonic_ns() + timeout_ns
    while True:
        try:
            fcntl.flock(fd, operation | fcntl.LOCK_NB)
            return
        except BlockingIOError as exc:
            now = time.monotonic_ns()
            if now >= deadline:
                raise _StateLockTimeout("camera state lock acquisition timed out") from exc
            remaining_s = (deadline - now) / 1_000_000_000.0
            time.sleep(min(0.0002, remaining_s))


def _pread_state(fd: int) -> bytes:
    metadata = _validate_state_fd(fd)
    if metadata.st_size != STATE_RECORD_SIZE:
        raise StateFileError(
            f"camera state file size is {metadata.st_size}, expected {STATE_RECORD_SIZE}"
        )
    raw = os.pread(fd, STATE_RECORD_SIZE, 0)
    if len(raw) != STATE_RECORD_SIZE:
        raise StateFileError("short pread from camera state")
    return raw


def _pwrite_all(fd: int, data: bytes, offset: int = 0) -> None:
    written = 0
    while written < len(data):
        count = os.pwrite(fd, data[written:], offset + written)
        if count <= 0:
            raise StateFileError("short pwrite to camera state")
        written += count


class CameraStateWriter:
    """Single-record writer with cross-process locking and monotonic sequence."""

    def __init__(
        self,
        path: str | os.PathLike[str],
        *,
        lock_timeout_ns: int = 5_000_000,
    ) -> None:
        self.path = _require_absolute_state_path(path)
        if (
            isinstance(lock_timeout_ns, bool)
            or not isinstance(lock_timeout_ns, int)
            or lock_timeout_ns <= 0
        ):
            raise ValueError("lock_timeout_ns must be positive")
        self.lock_timeout_ns = lock_timeout_ns
        # One supervisor owns one state inode for one UE lifetime.  Lifecycle
        # code must remove a previously validated stale file before binding a
        # new child; silently reopening a path could join an attacker inode or
        # a second writer.
        flags = os.O_RDWR | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC
        if hasattr(os, "O_NOFOLLOW"):
            flags |= os.O_NOFOLLOW
        try:
            self._fd = os.open(self.path, flags, 0o600)
        except OSError as exc:
            raise StateFileError(f"cannot open camera state {self.path}: {exc}") from exc
        try:
            os.fchmod(self._fd, 0o600)
            _validate_state_fd(self._fd)
        except Exception:
            os.close(self._fd)
            raise
        self._closed = False

    def close(self) -> None:
        if not self._closed:
            os.close(self._fd)
            self._closed = True

    def __enter__(self) -> "CameraStateWriter":
        return self

    def __exit__(self, _type: object, _value: object, _traceback: object) -> None:
        self.close()

    def write(self, observation: CameraProbeObservation) -> CameraProbeState:
        if self._closed:
            raise StateFileError("camera state writer is closed")
        if not isinstance(observation, CameraProbeObservation):
            raise TypeError("observation must be CameraProbeObservation")
        if observation.ue_pid <= 0 or observation.monotonic_ns <= 0:
            raise StateFileError("observation PID and monotonic timestamp must be positive")
        if observation.valid != (observation.error_code == ProbeError.NONE):
            raise StateFileError("observation valid flag and error code disagree")
        locked = False
        try:
            _bounded_flock(self._fd, fcntl.LOCK_EX, self.lock_timeout_ns)
            locked = True
            last_sequence = 0
            metadata = _validate_state_fd(self._fd)
            if metadata.st_size == STATE_RECORD_SIZE:
                try:
                    previous = _decode_state(_pread_state(self._fd))
                except StateFileError:
                    previous = None
                if previous is not None and previous.ue_pid == observation.ue_pid:
                    last_sequence = previous.sequence
            elif metadata.st_size != 0:
                os.ftruncate(self._fd, 0)
            if last_sequence >= _UINT64_MAX:
                raise StateFileError("camera state sequence exhausted uint64")
            state = CameraProbeState(
                ue_pid=observation.ue_pid,
                sequence=last_sequence + 1,
                monotonic_ns=observation.monotonic_ns,
                pitch_deg=observation.pitch_deg if observation.valid else 0.0,
                yaw_deg=observation.yaw_deg if observation.valid else 0.0,
                roll_deg=observation.roll_deg if observation.valid else 0.0,
                location_x=observation.location_x if observation.valid else 0.0,
                location_y=observation.location_y if observation.valid else 0.0,
                location_z=observation.location_z if observation.valid else 0.0,
                cache_timestamp_s=(
                    observation.cache_timestamp_s if observation.valid else 0.0
                ),
                valid=observation.valid,
                error_code=observation.error_code,
            )
            encoded = _encode_state(state)
            _pwrite_all(self._fd, encoded)
            os.ftruncate(self._fd, STATE_RECORD_SIZE)
            return state
        finally:
            if locked:
                fcntl.flock(self._fd, fcntl.LOCK_UN | fcntl.LOCK_NB)


def _angle_delta_degrees(first: float, second: float) -> float:
    return abs((second - first + 180.0) % 360.0 - 180.0)


class CameraStateReader:
    """Fail-closed consumer for fresh final-POV state from one expected UE PID."""

    def __init__(
        self,
        path: str | os.PathLike[str],
        *,
        expected_ue_pid: int,
        max_age_ns: int = DEFAULT_MAX_AGE_NS,
        angle_change_epsilon_deg: float = DEFAULT_ANGLE_CHANGE_EPSILON_DEG,
        lock_timeout_ns: int = 5_000_000,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        self.path = _require_absolute_state_path(path)
        if (
            isinstance(expected_ue_pid, bool)
            or not isinstance(expected_ue_pid, int)
            or expected_ue_pid <= 0
        ):
            raise ValueError("expected_ue_pid must be a positive integer")
        if isinstance(max_age_ns, bool) or not isinstance(max_age_ns, int) or max_age_ns <= 0:
            raise ValueError("max_age_ns must be a positive integer")
        if not math.isfinite(angle_change_epsilon_deg) or angle_change_epsilon_deg < 0.0:
            raise ValueError("angle_change_epsilon_deg must be finite and non-negative")
        if (
            isinstance(lock_timeout_ns, bool)
            or not isinstance(lock_timeout_ns, int)
            or lock_timeout_ns <= 0
        ):
            raise ValueError("lock_timeout_ns must be positive")
        self.expected_ue_pid = expected_ue_pid
        self.max_age_ns = max_age_ns
        self.angle_change_epsilon_deg = angle_change_epsilon_deg
        self.lock_timeout_ns = lock_timeout_ns
        self._monotonic_ns = monotonic_ns
        self.last_error: str | None = None
        self.last_state: CameraProbeState | None = None
        self.angles_changed = False
        self.max_angle_delta_deg = 0.0

    def _fail(self, reason: str) -> None:
        self.last_error = reason
        self.angles_changed = False
        self.max_angle_delta_deg = 0.0

    def read(self, now_monotonic_ns: int | None = None) -> CameraProbeState | None:
        now = self._monotonic_ns() if now_monotonic_ns is None else now_monotonic_ns
        if isinstance(now, bool) or not isinstance(now, int) or now <= 0:
            self._fail("invalid_now")
            return None
        flags = os.O_RDONLY | os.O_CLOEXEC
        if hasattr(os, "O_NOFOLLOW"):
            flags |= os.O_NOFOLLOW
        try:
            fd = os.open(self.path, flags)
        except FileNotFoundError:
            self._fail("missing")
            return None
        except OSError:
            self._fail("open_failed")
            return None
        locked = False
        try:
            _bounded_flock(fd, fcntl.LOCK_SH, self.lock_timeout_ns)
            locked = True
            raw = _pread_state(fd)
        except _StateLockTimeout:
            self._fail("busy")
            return None
        except (OSError, StateFileError):
            self._fail("corrupt")
            return None
        finally:
            try:
                if locked:
                    fcntl.flock(fd, fcntl.LOCK_UN | fcntl.LOCK_NB)
            finally:
                os.close(fd)
        try:
            state = _decode_state(raw)
        except StateFileError:
            self._fail("corrupt")
            return None
        if state.ue_pid != self.expected_ue_pid:
            self._fail("unexpected_ue_pid")
            return None
        if not state.valid or state.error_code != ProbeError.NONE:
            self._fail(f"probe_error_{int(state.error_code)}")
            return None
        if state.monotonic_ns > now:
            self._fail("future")
            return None
        if now - state.monotonic_ns > self.max_age_ns:
            self._fail("stale")
            return None
        previous = self.last_state
        if previous is not None and state.sequence < previous.sequence:
            self._fail("sequence_regression")
            return None
        self.angles_changed = False
        self.max_angle_delta_deg = 0.0
        if previous is not None and state.sequence > previous.sequence:
            deltas = (
                _angle_delta_degrees(previous.pitch_deg, state.pitch_deg),
                _angle_delta_degrees(previous.yaw_deg, state.yaw_deg),
                _angle_delta_degrees(previous.roll_deg, state.roll_deg),
            )
            self.max_angle_delta_deg = max(deltas)
            self.angles_changed = self.max_angle_delta_deg > self.angle_change_epsilon_deg
        self.last_state = state
        self.last_error = None
        return state

    def read_with_change(
        self, now_monotonic_ns: int | None = None
    ) -> CameraStateReading | None:
        state = self.read(now_monotonic_ns)
        if state is None:
            return None
        return CameraStateReading(
            state=state,
            angles_changed=self.angles_changed,
            max_angle_delta_deg=self.max_angle_delta_deg,
        )


def probe_and_write(
    probe: UECameraProbe,
    writer: CameraStateWriter,
    ue_pid: int,
) -> CameraProbeState:
    """Take one fail-closed observation and publish it with a new sequence."""

    return writer.write(probe.sample(ue_pid))
