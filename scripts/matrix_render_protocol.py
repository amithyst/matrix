"""Wire protocol used by Matrix's MuJoCo process to update the UE robot."""

from __future__ import annotations

from dataclasses import dataclass
import socket
import struct
from typing import Sequence

import numpy as np


_TIME_AND_SIZE = struct.Struct("<dI")
_SIZE = struct.Struct("<I")


@dataclass(frozen=True)
class MujocoRenderState:
    sim_time: float
    qpos: np.ndarray
    qvel: np.ndarray
    ctrl: np.ndarray


def _wire_vector(values: Sequence[float] | np.ndarray, *, name: str) -> np.ndarray:
    array = np.asarray(values, dtype="<f8")
    if array.ndim != 1:
        raise ValueError(f"{name} must be one-dimensional, got {array.shape}")
    if not np.isfinite(array).all():
        raise ValueError(f"{name} contains non-finite values")
    return np.ascontiguousarray(array)


def packet_size(*, nq: int, nv: int, nu: int) -> int:
    return 8 + 4 + (8 * nq) + 4 + (8 * nv) + 4 + (8 * nu)


def pack_mujoco_state(
    sim_time: float,
    qpos: Sequence[float] | np.ndarray,
    qvel: Sequence[float] | np.ndarray,
    ctrl: Sequence[float] | np.ndarray,
) -> bytes:
    if not np.isfinite(sim_time):
        raise ValueError("sim_time must be finite")
    qpos_array = _wire_vector(qpos, name="qpos")
    qvel_array = _wire_vector(qvel, name="qvel")
    ctrl_array = _wire_vector(ctrl, name="ctrl")
    return b"".join(
        (
            _TIME_AND_SIZE.pack(float(sim_time), qpos_array.size),
            qpos_array.tobytes(),
            _SIZE.pack(qvel_array.size),
            qvel_array.tobytes(),
            _SIZE.pack(ctrl_array.size),
            ctrl_array.tobytes(),
        )
    )


def unpack_mujoco_state(payload: bytes) -> MujocoRenderState:
    view = memoryview(payload)
    if len(view) < _TIME_AND_SIZE.size:
        raise ValueError("Matrix render packet is truncated before qpos")
    sim_time, nq = _TIME_AND_SIZE.unpack_from(view, 0)
    offset = _TIME_AND_SIZE.size

    def take_vector(size: int, name: str) -> np.ndarray:
        nonlocal offset
        byte_count = int(size) * 8
        if offset + byte_count > len(view):
            raise ValueError(f"Matrix render packet is truncated in {name}")
        result = np.frombuffer(view[offset : offset + byte_count], dtype="<f8").copy()
        offset += byte_count
        return result

    qpos = take_vector(nq, "qpos")
    if offset + _SIZE.size > len(view):
        raise ValueError("Matrix render packet is truncated before qvel")
    (nv,) = _SIZE.unpack_from(view, offset)
    offset += _SIZE.size
    qvel = take_vector(nv, "qvel")
    if offset + _SIZE.size > len(view):
        raise ValueError("Matrix render packet is truncated before ctrl")
    (nu,) = _SIZE.unpack_from(view, offset)
    offset += _SIZE.size
    ctrl = take_vector(nu, "ctrl")
    if offset != len(view):
        raise ValueError(
            f"Matrix render packet has {len(view) - offset} trailing bytes"
        )
    return MujocoRenderState(sim_time=sim_time, qpos=qpos, qvel=qvel, ctrl=ctrl)


class MatrixRenderPublisher:
    def __init__(self, host: str = "127.0.0.1", port: int = 9999) -> None:
        self.address = (host, int(port))
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.packet_count = 0

    def send(
        self,
        sim_time: float,
        qpos: Sequence[float] | np.ndarray,
        qvel: Sequence[float] | np.ndarray,
        ctrl: Sequence[float] | np.ndarray,
    ) -> int:
        payload = pack_mujoco_state(sim_time, qpos, qvel, ctrl)
        sent = self.socket.sendto(payload, self.address)
        if sent != len(payload):
            raise RuntimeError(f"partial Matrix UDP send: {sent}/{len(payload)} bytes")
        self.packet_count += 1
        return sent

    def close(self) -> None:
        self.socket.close()
