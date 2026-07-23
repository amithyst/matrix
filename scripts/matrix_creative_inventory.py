#!/usr/bin/env python3
"""Runtime authority for Matrix creative-mode standalone physical props."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import re
from typing import Any

import numpy as np

try:
    import mujoco
except ModuleNotFoundError:  # Optional in source-only CI and control-plane tools.
    mujoco = None

from inject_creative_inventory import InventoryItem, load_catalog


_JOINT_RE = re.compile(
    r"creative_item__(?P<item>[a-z0-9][a-z0-9_-]{0,47})__"
    r"(?P<index>[0-9]+)__freejoint\Z"
)
_ACTIVE_COLLISION_CONTYPE = 1
_ACTIVE_COLLISION_CONAFFINITY = 1


class CreativeInventoryError(RuntimeError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


@dataclass(frozen=True)
class SpawnedItem:
    item_id: str
    instance_name: str
    position: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]


@dataclass(frozen=True)
class _PoolEntry:
    item_id: str
    index: int
    instance_name: str
    qpos_address: int
    dof_address: int
    equality_id: int
    body_id: int
    collision_geom_id: int


def _quat_multiply(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


class CreativeInventoryRuntime:
    """One-shot freejoint placement followed exclusively by MuJoCo physics."""

    def __init__(self, simulator: Any, catalog_path: Path) -> None:
        if mujoco is None:
            raise CreativeInventoryError(
                "E_INVENTORY_DEPENDENCY",
                "MuJoCo Python bindings are required for creative inventory runtime",
            )
        self.simulator = simulator
        self.items = load_catalog(catalog_path)
        self.items_by_id: dict[str, InventoryItem] = {
            item.item_id: item for item in self.items
        }
        try:
            self.model = simulator.sim_env.mj_model
            self.data = simulator.sim_env.mj_data
            self.step_lock = simulator._step_lock
        except AttributeError as exc:
            raise CreativeInventoryError(
                "E_INVENTORY_BACKEND",
                "native simulator does not expose the audited MuJoCo owner",
            ) from exc
        self.pools: dict[str, list[_PoolEntry]] = {item.item_id: [] for item in self.items}
        for joint_id in range(self.model.njnt):
            joint_name = self.model.joint(joint_id).name
            match = _JOINT_RE.fullmatch(joint_name or "")
            if match is None or match.group("item") not in self.pools:
                continue
            if int(self.model.jnt_type[joint_id]) != int(mujoco.mjtJoint.mjJNT_FREE):
                raise CreativeInventoryError(
                    "E_INVENTORY_MODEL", f"{joint_name} is not a freejoint"
                )
            item_id = match.group("item")
            index = int(match.group("index"))
            instance_name = f"creative_item__{item_id}__{index}"
            equality_name = f"{instance_name}__storage_weld"
            equality_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_EQUALITY, equality_name
            )
            if equality_id < 0:
                raise CreativeInventoryError(
                    "E_INVENTORY_MODEL", f"storage weld is missing for {instance_name}"
                )
            body_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_BODY,
                instance_name,
            )
            if body_id < 0:
                raise CreativeInventoryError(
                    "E_INVENTORY_MODEL",
                    f"pool body is missing for {instance_name}",
                )
            collision_geom_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_GEOM,
                f"{instance_name}__collision",
            )
            if collision_geom_id < 0:
                raise CreativeInventoryError(
                    "E_INVENTORY_MODEL",
                    f"collision geom is missing for {instance_name}",
                )
            self.pools[item_id].append(
                _PoolEntry(
                    item_id=item_id,
                    index=index,
                    instance_name=instance_name,
                    qpos_address=int(self.model.jnt_qposadr[joint_id]),
                    dof_address=int(self.model.jnt_dofadr[joint_id]),
                    equality_id=equality_id,
                    body_id=body_id,
                    collision_geom_id=collision_geom_id,
                )
            )
        for item in self.items:
            entries = sorted(self.pools[item.item_id], key=lambda entry: entry.index)
            if len(entries) != item.pool_size:
                raise CreativeInventoryError(
                    "E_INVENTORY_MODEL",
                    f"{item.item_id} pool has {len(entries)} bodies, expected {item.pool_size}",
                )
            self.pools[item.item_id] = entries
        # Enforce the inactive-pool contract even when an older cached MJCF
        # predates the explicit contype/conaffinity attributes.  A body parked
        # below an infinite plane otherwise receives an enormous separating
        # impulse while its storage weld simultaneously holds it in place.
        with self.step_lock:
            for entries in self.pools.values():
                for entry in entries:
                    self.data.eq_active[entry.equality_id] = 1
                    self.model.body_contype[entry.body_id] = 0
                    self.model.body_conaffinity[entry.body_id] = 0
                    self.model.geom_contype[entry.collision_geom_id] = 0
                    self.model.geom_conaffinity[entry.collision_geom_id] = 0
            mujoco.mj_forward(self.model, self.data)
        self.spawned: set[str] = set()
        self.spawn_count = 0

    @property
    def expected_snapshot_dimensions(self) -> dict[str, int]:
        return {
            "qpos": int(self.model.nq),
            "qvel": int(self.model.nv),
            "ctrl": int(self.model.nu),
            "applied_torque": 29,
        }

    def mapping(self) -> dict[str, object]:
        return {
            "version": 1,
            "available": True,
            "spawn_count": self.spawn_count,
            "items": [
                {
                    "item_id": item.item_id,
                    "label": item.label,
                    "pool_size": item.pool_size,
                    "remaining": sum(
                        entry.instance_name not in self.spawned
                        for entry in self.pools[item.item_id]
                    ),
                }
                for item in self.items
            ],
        }

    def spawn(self, item_id: str, current_pose: Any) -> SpawnedItem:
        item = self.items_by_id.get(item_id)
        if item is None:
            raise CreativeInventoryError(
                "E_INVENTORY_ITEM", f"creative item {item_id!r} is unavailable"
            )
        entry = next(
            (
                candidate
                for candidate in self.pools[item_id]
                if candidate.instance_name not in self.spawned
            ),
            None,
        )
        if entry is None:
            raise CreativeInventoryError(
                "E_INVENTORY_FULL", f"no unused {item.label} instance remains"
            )
        try:
            x = float(current_pose.x)
            y = float(current_pose.y)
            yaw = float(current_pose.yaw_rad)
        except (AttributeError, TypeError, ValueError) as exc:
            raise CreativeInventoryError(
                "E_INVENTORY_POSE", "robot pose is invalid"
            ) from exc
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise CreativeInventoryError(
                "E_INVENTORY_POSE", "robot pose is not finite"
            )
        position = (
            x + math.cos(yaw) * item.spawn_distance_m,
            y + math.sin(yaw) * item.spawn_distance_m,
            item.spawn_height_m,
        )
        yaw_quat = (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))
        quaternion = _quat_multiply(yaw_quat, item.spawn_quat)
        norm = math.sqrt(sum(value * value for value in quaternion))
        quaternion = tuple(value / norm for value in quaternion)
        with self.step_lock:
            self.data.eq_active[entry.equality_id] = 0
            self.data.qpos[entry.qpos_address : entry.qpos_address + 3] = np.asarray(
                position, dtype=np.float64
            )
            self.data.qpos[entry.qpos_address + 3 : entry.qpos_address + 7] = np.asarray(
                quaternion, dtype=np.float64
            )
            self.data.qvel[entry.dof_address : entry.dof_address + 6] = 0.0
            # MuJoCo caches an aggregate collision mask on each body at
            # compile time.  Restore both levels so broadphase considers the
            # released prop after its collision geom is re-enabled.
            self.model.body_contype[entry.body_id] = _ACTIVE_COLLISION_CONTYPE
            self.model.body_conaffinity[
                entry.body_id
            ] = _ACTIVE_COLLISION_CONAFFINITY
            self.model.geom_contype[
                entry.collision_geom_id
            ] = _ACTIVE_COLLISION_CONTYPE
            self.model.geom_conaffinity[
                entry.collision_geom_id
            ] = _ACTIVE_COLLISION_CONAFFINITY
            mujoco.mj_forward(self.model, self.data)
        self.spawned.add(entry.instance_name)
        self.spawn_count += 1
        return SpawnedItem(
            item_id=item_id,
            instance_name=entry.instance_name,
            position=position,
            quaternion=quaternion,
        )


__all__ = [
    "CreativeInventoryError",
    "CreativeInventoryRuntime",
    "SpawnedItem",
]
