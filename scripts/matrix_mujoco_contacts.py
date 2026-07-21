#!/usr/bin/env python3
"""Small dependency-free helpers for classifying MuJoCo support contacts."""

from __future__ import annotations

import math
from typing import Any, AbstractSet


def robot_body_ids(model: Any, root_body_id: int) -> set[int]:
    """Return ``root_body_id`` and all descendants in the MuJoCo body tree."""

    root = int(root_body_id)
    count = int(model.nbody)
    if root <= 0 or root >= count:
        raise ValueError("robot root body id is outside the model body table")
    parents = model.body_parentid
    result: set[int] = set()
    for body_id in range(1, count):
        current = body_id
        visited: set[int] = set()
        while current > 0:
            if current in visited or current >= count:
                raise ValueError("MuJoCo body parent tree is invalid")
            visited.add(current)
            if current == root:
                result.add(body_id)
                break
            current = int(parents[current])
    return result


def has_external_foot_support(
    model: Any,
    data: Any,
    *,
    foot_body_ids: AbstractSet[int],
    robot_root_body_id: int,
    minimum_vertical_normal: float = 0.5,
    maximum_contact_distance_m: float = 1e-4,
) -> bool:
    """Return true for penetrating/touching foot contact with usable support.

    Robot self-collisions are excluded by walking the body tree.  Near-
    horizontal contact normals are rejected so a foot brushing a wall cannot
    satisfy the get-up stable hold.  External static or dynamic scene bodies
    remain valid physical support surfaces.
    """

    vertical = float(minimum_vertical_normal)
    distance_limit = float(maximum_contact_distance_m)
    if not math.isfinite(vertical) or not 0.0 < vertical <= 1.0:
        raise ValueError("minimum_vertical_normal must be in (0, 1]")
    if not math.isfinite(distance_limit) or distance_limit < 0.0:
        raise ValueError("maximum_contact_distance_m must be finite and non-negative")
    feet = {int(body_id) for body_id in foot_body_ids if int(body_id) > 0}
    if not feet:
        return False
    robot = robot_body_ids(model, robot_root_body_id)
    geom_body_ids = model.geom_bodyid
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        body1 = int(geom_body_ids[int(contact.geom1)])
        body2 = int(geom_body_ids[int(contact.geom2)])
        foot1 = body1 in feet
        foot2 = body2 in feet
        if not foot1 and not foot2:
            continue
        counterpart = body2 if foot1 else body1
        if counterpart in robot:
            continue
        try:
            distance = float(contact.dist)
            normal_z = float(contact.frame[2])
        except (AttributeError, IndexError, TypeError, ValueError, OverflowError):
            continue
        if not math.isfinite(distance) or distance > distance_limit:
            continue
        if not math.isfinite(normal_z) or abs(normal_z) < vertical:
            continue
        return True
    return False


def has_external_ground_support(
    model: Any,
    data: Any,
    *,
    robot_root_body_id: int,
    minimum_vertical_normal: float = 0.5,
    maximum_contact_distance_m: float = 1e-4,
) -> bool:
    """Return true when any robot body is supported by an external surface.

    This is intentionally broader than :func:`has_external_foot_support`.
    A fallen robot may be resting on its torso, back, elbow, or knee while its
    feet are airborne.  Such a grounded, low-energy pose is safe for a
    get-up-policy transition, but must not count as stable standing.
    """

    vertical = float(minimum_vertical_normal)
    distance_limit = float(maximum_contact_distance_m)
    if not math.isfinite(vertical) or not 0.0 < vertical <= 1.0:
        raise ValueError("minimum_vertical_normal must be in (0, 1]")
    if not math.isfinite(distance_limit) or distance_limit < 0.0:
        raise ValueError("maximum_contact_distance_m must be finite and non-negative")
    robot = robot_body_ids(model, robot_root_body_id)
    geom_body_ids = model.geom_bodyid
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        body1 = int(geom_body_ids[int(contact.geom1)])
        body2 = int(geom_body_ids[int(contact.geom2)])
        robot1 = body1 in robot
        robot2 = body2 in robot
        if robot1 == robot2:
            # Exclude robot self-collision and scene-scene contact.
            continue
        try:
            distance = float(contact.dist)
            normal_z = float(contact.frame[2])
        except (AttributeError, IndexError, TypeError, ValueError, OverflowError):
            continue
        if not math.isfinite(distance) or distance > distance_limit:
            continue
        if not math.isfinite(normal_z) or abs(normal_z) < vertical:
            continue
        return True
    return False


__all__ = (
    "has_external_foot_support",
    "has_external_ground_support",
    "robot_body_ids",
)
