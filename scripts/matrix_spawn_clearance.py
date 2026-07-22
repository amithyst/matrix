#!/usr/bin/env python3
"""Fail-closed MuJoCo spawn-clearance auditing for Matrix robots.

The contact classifier deliberately depends only on MuJoCo-shaped model/data
objects.  Importing MuJoCo is left to the caller so the safety contract remains
cheap to unit test and can be reused by launch preflight and evidence tooling.
"""

from __future__ import annotations

import math
import operator
from collections.abc import Mapping
from typing import Any


AUDIT_SCHEMA = "matrix-spawn-clearance-audit/v1"
BODY_PENETRATION_TOLERANCE_M = 0.002
FOOT_PENETRATION_TOLERANCE_M = 0.015
MINIMUM_FOOT_VERTICAL_NORMAL = 0.8

_PELVIS_BODY_NAME = "pelvis"
_FOOT_BODY_NAMES = (
    "left_ankle_roll_link",
    "right_ankle_roll_link",
)


class SpawnClearanceError(ValueError):
    """Raised internally when an audit input cannot be trusted."""


def _index(value: object, *, label: str) -> int:
    if isinstance(value, bool):
        raise SpawnClearanceError(f"{label} must be an integer")
    try:
        return operator.index(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise SpawnClearanceError(f"{label} must be an integer") from exc


def _finite(value: object, *, label: str) -> float:
    if isinstance(value, bool):
        raise SpawnClearanceError(f"{label} must be a finite number")
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise SpawnClearanceError(f"{label} must be a finite number") from exc
    if not math.isfinite(number):
        raise SpawnClearanceError(f"{label} must be finite")
    return number


def _vector(value: object, *, length: int, label: str) -> tuple[float, ...]:
    try:
        result = tuple(
            _finite(value[index], label=f"{label}[{index}]")  # type: ignore[index]
            for index in range(length)
        )
    except (AttributeError, IndexError, KeyError, TypeError) as exc:
        raise SpawnClearanceError(
            f"{label} must contain at least {length} finite values"
        ) from exc
    return result


def _validated_thresholds(
    *,
    body_penetration_tolerance_m: float,
    foot_penetration_tolerance_m: float,
    minimum_foot_vertical_normal: float,
) -> dict[str, float]:
    body_tolerance = _finite(
        body_penetration_tolerance_m,
        label="body penetration tolerance",
    )
    foot_tolerance = _finite(
        foot_penetration_tolerance_m,
        label="foot penetration tolerance",
    )
    vertical_normal = _finite(
        minimum_foot_vertical_normal,
        label="minimum foot vertical normal",
    )
    if body_tolerance < 0.0:
        raise SpawnClearanceError("body penetration tolerance must be non-negative")
    if foot_tolerance < 0.0:
        raise SpawnClearanceError("foot penetration tolerance must be non-negative")
    if not 0.0 < vertical_normal <= 1.0:
        raise SpawnClearanceError("minimum foot vertical normal must be in (0, 1]")
    return {
        "body_penetration_tolerance_m": body_tolerance,
        "foot_penetration_tolerance_m": foot_tolerance,
        "minimum_foot_vertical_normal": vertical_normal,
    }


def _default_thresholds() -> dict[str, float]:
    return {
        "body_penetration_tolerance_m": BODY_PENETRATION_TOLERANCE_M,
        "foot_penetration_tolerance_m": FOOT_PENETRATION_TOLERANCE_M,
        "minimum_foot_vertical_normal": MINIMUM_FOOT_VERTICAL_NORMAL,
    }


def _audit_error(
    error: BaseException,
    *,
    thresholds: Mapping[str, float] | None = None,
    evaluated_pose: Mapping[str, object] | None = None,
) -> dict[str, object]:
    result: dict[str, object] = {
        "schema": AUDIT_SCHEMA,
        "safe": False,
        "reason": "audit_error",
        "error": {
            "type": type(error).__name__,
            "message": str(error) or type(error).__name__,
        },
        "thresholds": dict(thresholds or _default_thresholds()),
        "robot": None,
        "contacts_checked": 0,
        "external_contact_count": 0,
        "allowed_contact_count": 0,
        "rejected_contact_count": 0,
        "ignored_self_contact_count": 0,
        "ignored_scene_contact_count": 0,
        "contacts": [],
        "worst": None,
    }
    if evaluated_pose is not None:
        result["evaluated_pose"] = dict(evaluated_pose)
    return result


def _model_count(model: Any, name: str) -> int:
    try:
        count = _index(getattr(model, name), label=f"model.{name}")
    except AttributeError as exc:
        raise SpawnClearanceError(f"model.{name} is missing") from exc
    if count <= 0:
        raise SpawnClearanceError(f"model.{name} must be positive")
    return count


def _named_body_id(model: Any, name: str, *, nbody: int) -> int:
    try:
        body = model.body(name)
        body_id = _index(body.id, label=f"body {name!r} id")
    except SpawnClearanceError:
        raise
    except (AttributeError, KeyError, TypeError, ValueError, IndexError) as exc:
        raise SpawnClearanceError(f"required body {name!r} is missing") from exc
    if not 0 < body_id < nbody:
        raise SpawnClearanceError(f"body {name!r} id is outside the model body table")
    return body_id


def _optional_name(model: Any, kind: str, item_id: int) -> str | None:
    try:
        item = getattr(model, kind)(item_id)
        name = item.name
    except (AttributeError, KeyError, TypeError, ValueError, IndexError):
        return None
    if isinstance(name, bytes):
        try:
            name = name.decode("utf-8")
        except UnicodeDecodeError:
            return None
    return name if isinstance(name, str) and name else None


def _robot_body_ids(model: Any, *, root_body_id: int, nbody: int) -> set[int]:
    try:
        parents = model.body_parentid
    except AttributeError as exc:
        raise SpawnClearanceError("model.body_parentid is missing") from exc
    result: set[int] = set()
    for body_id in range(1, nbody):
        current = body_id
        visited: set[int] = set()
        while current > 0:
            if current in visited:
                raise SpawnClearanceError("model body parent tree contains a cycle")
            if current >= nbody:
                raise SpawnClearanceError("model body parent id is out of range")
            visited.add(current)
            if current == root_body_id:
                result.add(body_id)
                break
            try:
                current = _index(
                    parents[current],
                    label=f"model.body_parentid[{current}]",
                )
            except (IndexError, KeyError, TypeError) as exc:
                raise SpawnClearanceError("model body parent table is truncated") from exc
            if current < 0 or current >= nbody:
                raise SpawnClearanceError("model body parent id is out of range")
    if root_body_id not in result:
        raise SpawnClearanceError("pelvis is not present in its robot body subtree")
    return result


def _geom_count(model: Any) -> int:
    try:
        getattr(model, "ngeom")
    except AttributeError:
        try:
            count = len(model.geom_bodyid)
        except (AttributeError, TypeError) as exc:
            raise SpawnClearanceError("model geom table is missing") from exc
        if count <= 0:
            raise SpawnClearanceError("model geom table must not be empty")
        return count
    return _model_count(model, "ngeom")


def _contact_mapping(
    *,
    model: Any,
    contact_index: int,
    geom1: int,
    geom2: int,
    body1: int,
    body2: int,
    robot1: bool,
    robot2: bool,
    distance: float,
    normal: tuple[float, ...],
    scene_to_robot_normal: tuple[float, ...],
    position: tuple[float, ...],
    allowed: bool,
    classification: str,
) -> dict[str, object]:
    robot_body_id = body1 if robot1 else body2
    scene_body_id = body2 if robot1 else body1
    return {
        "contact_index": contact_index,
        "distance_m": distance,
        "penetration_m": max(0.0, -distance),
        "normal": list(normal),
        "scene_to_robot_normal": list(scene_to_robot_normal),
        "position_m": list(position),
        "allowed": allowed,
        "classification": classification,
        "robot_body": {
            "id": robot_body_id,
            "name": _optional_name(model, "body", robot_body_id),
        },
        "scene_body": {
            "id": scene_body_id,
            "name": _optional_name(model, "body", scene_body_id),
        },
        "geom1": {
            "id": geom1,
            "name": _optional_name(model, "geom", geom1),
            "body_id": body1,
            "body_name": _optional_name(model, "body", body1),
            "robot": robot1,
        },
        "geom2": {
            "id": geom2,
            "name": _optional_name(model, "geom", geom2),
            "body_id": body2,
            "body_name": _optional_name(model, "body", body2),
            "robot": robot2,
        },
    }


def audit_spawn_clearance(
    model: Any,
    data: Any,
    *,
    body_penetration_tolerance_m: float = BODY_PENETRATION_TOLERANCE_M,
    foot_penetration_tolerance_m: float = FOOT_PENETRATION_TOLERANCE_M,
    minimum_foot_vertical_normal: float = MINIMUM_FOOT_VERTICAL_NORMAL,
) -> dict[str, object]:
    """Return a JSON-serializable, fail-closed robot/scene contact audit."""

    thresholds: dict[str, float] | None = None
    try:
        thresholds = _validated_thresholds(
            body_penetration_tolerance_m=body_penetration_tolerance_m,
            foot_penetration_tolerance_m=foot_penetration_tolerance_m,
            minimum_foot_vertical_normal=minimum_foot_vertical_normal,
        )
        nbody = _model_count(model, "nbody")
        ngeom = _geom_count(model)
        pelvis_id = _named_body_id(model, _PELVIS_BODY_NAME, nbody=nbody)
        foot_ids = tuple(
            _named_body_id(model, name, nbody=nbody) for name in _FOOT_BODY_NAMES
        )
        if len(set(foot_ids)) != len(_FOOT_BODY_NAMES):
            raise SpawnClearanceError("left and right foot bodies must be distinct")
        robot_ids = _robot_body_ids(
            model,
            root_body_id=pelvis_id,
            nbody=nbody,
        )
        if any(body_id not in robot_ids for body_id in foot_ids):
            raise SpawnClearanceError("foot body is not a descendant of pelvis")
        try:
            geom_body_ids = model.geom_bodyid
        except AttributeError as exc:
            raise SpawnClearanceError("model.geom_bodyid is missing") from exc
        ncon = _index(data.ncon, label="data.ncon")
        if ncon < 0:
            raise SpawnClearanceError("data.ncon must be non-negative")

        external: list[dict[str, object]] = []
        rejected: list[dict[str, object]] = []
        ignored_self = 0
        ignored_scene = 0
        for contact_index in range(ncon):
            try:
                contact = data.contact[contact_index]
            except (AttributeError, IndexError, KeyError, TypeError) as exc:
                raise SpawnClearanceError("data contact table is truncated") from exc
            geom1 = _index(contact.geom1, label=f"contact[{contact_index}].geom1")
            geom2 = _index(contact.geom2, label=f"contact[{contact_index}].geom2")
            if not 0 <= geom1 < ngeom or not 0 <= geom2 < ngeom:
                raise SpawnClearanceError("contact geom id is outside the model geom table")
            try:
                body1 = _index(
                    geom_body_ids[geom1],
                    label=f"model.geom_bodyid[{geom1}]",
                )
                body2 = _index(
                    geom_body_ids[geom2],
                    label=f"model.geom_bodyid[{geom2}]",
                )
            except (IndexError, KeyError, TypeError) as exc:
                raise SpawnClearanceError("model geom body table is truncated") from exc
            if not 0 <= body1 < nbody or not 0 <= body2 < nbody:
                raise SpawnClearanceError("contact body id is outside the model body table")
            distance = _finite(
                contact.dist,
                label=f"contact[{contact_index}].dist",
            )
            normal = _vector(
                contact.frame,
                length=3,
                label=f"contact[{contact_index}].frame",
            )
            position = _vector(
                contact.pos,
                length=3,
                label=f"contact[{contact_index}].pos",
            )
            robot1 = body1 in robot_ids
            robot2 = body2 in robot_ids
            if robot1 == robot2:
                if robot1:
                    ignored_self += 1
                else:
                    ignored_scene += 1
                continue

            robot_body_id = body1 if robot1 else body2
            # MuJoCo's contact-frame normal points from geom1 to geom2.  The
            # safety test needs one invariant direction, from the scene into
            # the robot, so reverse it exactly when the robot is geom1.  Its
            # sign is safety-critical: a foot pressed into a ceiling must not
            # be accepted as floor support merely because the normal is
            # vertical.
            scene_to_robot_normal = (
                tuple(-component for component in normal) if robot1 else normal
            )
            if robot_body_id in foot_ids:
                support_normal = scene_to_robot_normal[2]
                allowed = bool(
                    support_normal >= thresholds["minimum_foot_vertical_normal"]
                    and distance
                    >= -thresholds["foot_penetration_tolerance_m"]
                )
                if allowed:
                    classification = "allowed_foot_support"
                elif support_normal < thresholds["minimum_foot_vertical_normal"]:
                    classification = "unsafe_foot_contact_normal"
                else:
                    classification = "unsafe_foot_penetration"
            else:
                allowed = bool(
                    distance >= -thresholds["body_penetration_tolerance_m"]
                )
                classification = (
                    "allowed_body_contact_tolerance"
                    if allowed
                    else "scene_penetration"
                )
            item = _contact_mapping(
                model=model,
                contact_index=contact_index,
                geom1=geom1,
                geom2=geom2,
                body1=body1,
                body2=body2,
                robot1=robot1,
                robot2=robot2,
                distance=distance,
                normal=normal,
                scene_to_robot_normal=scene_to_robot_normal,
                position=position,
                allowed=allowed,
                classification=classification,
            )
            external.append(item)
            if not allowed:
                rejected.append(item)

        if rejected:
            worst = max(
                rejected,
                key=lambda item: (
                    float(item["penetration_m"]),
                    -int(item["contact_index"]),
                ),
            )
            reason = (
                "unsafe_foot_contact"
                if str(worst["classification"]).startswith("unsafe_foot_")
                else "scene_penetration"
            )
        else:
            worst = (
                min(
                    external,
                    key=lambda item: (
                        float(item["distance_m"]),
                        int(item["contact_index"]),
                    ),
                )
                if external
                else None
            )
            reason = "clear"
        return {
            "schema": AUDIT_SCHEMA,
            "safe": not rejected,
            "reason": reason,
            "error": None,
            "thresholds": thresholds,
            "robot": {
                "root_body": {
                    "id": pelvis_id,
                    "name": _PELVIS_BODY_NAME,
                },
                "body_count": len(robot_ids),
                "foot_bodies": [
                    {"id": body_id, "name": name}
                    for name, body_id in zip(_FOOT_BODY_NAMES, foot_ids)
                ],
            },
            "contacts_checked": ncon,
            "external_contact_count": len(external),
            "allowed_contact_count": len(external) - len(rejected),
            "rejected_contact_count": len(rejected),
            "ignored_self_contact_count": ignored_self,
            "ignored_scene_contact_count": ignored_scene,
            "contacts": external,
            "worst": worst,
        }
    except Exception as exc:
        return _audit_error(exc, thresholds=thresholds)


def _pose_mapping(pose: object) -> dict[str, object]:
    if isinstance(pose, Mapping):
        if "position" in pose:
            position = _vector(pose["position"], length=3, label="pose.position")
        else:
            try:
                position = tuple(
                    _finite(pose[name], label=f"pose.{name}")
                    for name in ("x", "y", "z")
                )
            except KeyError as exc:
                raise SpawnClearanceError(
                    "pose must provide position or x/y/z"
                ) from exc
        try:
            yaw = _finite(pose["yaw_rad"], label="pose.yaw_rad")
        except KeyError as exc:
            raise SpawnClearanceError("pose.yaw_rad is missing") from exc
    else:
        try:
            position = tuple(
                _finite(getattr(pose, name), label=f"pose.{name}")
                for name in ("x", "y", "z")
            )
            yaw = _finite(getattr(pose, "yaw_rad"), label="pose.yaw_rad")
        except AttributeError as exc:
            raise SpawnClearanceError(
                "pose must provide x, y, z, and yaw_rad"
            ) from exc
    return {
        "position": list(position),
        "yaw_rad": yaw,
    }


def _pelvis_free_qpos_address(mujoco: Any, model: Any, pelvis_id: int) -> int:
    try:
        start = _index(
            model.body_jntadr[pelvis_id],
            label="pelvis body_jntadr",
        )
        count = _index(
            model.body_jntnum[pelvis_id],
            label="pelvis body_jntnum",
        )
        free_type = _index(
            mujoco.mjtJoint.mjJNT_FREE,
            label="MuJoCo free-joint type",
        )
    except (AttributeError, IndexError, KeyError, TypeError) as exc:
        raise SpawnClearanceError("pelvis free-joint metadata is unavailable") from exc
    if start < 0 or count <= 0:
        raise SpawnClearanceError("pelvis has no root joint")
    free_joint_ids: list[int] = []
    for joint_id in range(start, start + count):
        try:
            joint_type = _index(
                model.jnt_type[joint_id],
                label=f"model.jnt_type[{joint_id}]",
            )
        except (IndexError, KeyError, TypeError) as exc:
            raise SpawnClearanceError("model joint type table is truncated") from exc
        if joint_type == free_type:
            free_joint_ids.append(joint_id)
    if len(free_joint_ids) != 1:
        raise SpawnClearanceError("pelvis must own exactly one free root joint")
    joint_id = free_joint_ids[0]
    try:
        address = _index(
            model.jnt_qposadr[joint_id],
            label=f"model.jnt_qposadr[{joint_id}]",
        )
    except (AttributeError, IndexError, KeyError, TypeError) as exc:
        raise SpawnClearanceError("model joint qpos table is unavailable") from exc
    nq = _model_count(model, "nq")
    if address < 0 or address + 7 > nq:
        raise SpawnClearanceError("pelvis free-joint qpos address is out of range")
    return address


def apply_root_pose_and_audit(
    mujoco: Any,
    model: Any,
    pose: object,
    *,
    body_penetration_tolerance_m: float = BODY_PENETRATION_TOLERANCE_M,
    foot_penetration_tolerance_m: float = FOOT_PENETRATION_TOLERANCE_M,
    minimum_foot_vertical_normal: float = MINIMUM_FOOT_VERTICAL_NORMAL,
) -> dict[str, object]:
    """Audit ``pose`` in a fresh ``MjData`` without touching live simulation data."""

    evaluated_pose: dict[str, object] | None = None
    thresholds: dict[str, float] | None = None
    try:
        thresholds = _validated_thresholds(
            body_penetration_tolerance_m=body_penetration_tolerance_m,
            foot_penetration_tolerance_m=foot_penetration_tolerance_m,
            minimum_foot_vertical_normal=minimum_foot_vertical_normal,
        )
        evaluated_pose = _pose_mapping(pose)
        nbody = _model_count(model, "nbody")
        pelvis_id = _named_body_id(model, _PELVIS_BODY_NAME, nbody=nbody)
        qpos_address = _pelvis_free_qpos_address(mujoco, model, pelvis_id)
        data = mujoco.MjData(model)
        mujoco.mj_resetData(model, data)
        position = evaluated_pose["position"]
        yaw = float(evaluated_pose["yaw_rad"])
        normalized_yaw = math.remainder(yaw, 2.0 * math.pi)
        quaternion = [
            math.cos(normalized_yaw / 2.0),
            0.0,
            0.0,
            math.sin(normalized_yaw / 2.0),
        ]
        try:
            data.qpos[qpos_address : qpos_address + 7] = [
                *position,  # type: ignore[misc]
                *quaternion,
            ]
        except (AttributeError, IndexError, TypeError, ValueError) as exc:
            raise SpawnClearanceError("MuJoCo qpos does not accept the root pose") from exc
        mujoco.mj_forward(model, data)
        result = audit_spawn_clearance(
            model,
            data,
            body_penetration_tolerance_m=thresholds[
                "body_penetration_tolerance_m"
            ],
            foot_penetration_tolerance_m=thresholds[
                "foot_penetration_tolerance_m"
            ],
            minimum_foot_vertical_normal=thresholds[
                "minimum_foot_vertical_normal"
            ],
        )
        result["evaluated_pose"] = evaluated_pose
        result["root_qpos_address"] = qpos_address
        return result
    except Exception as exc:
        return _audit_error(
            exc,
            thresholds=thresholds,
            evaluated_pose=evaluated_pose,
        )


__all__ = (
    "AUDIT_SCHEMA",
    "BODY_PENETRATION_TOLERANCE_M",
    "FOOT_PENETRATION_TOLERANCE_M",
    "MINIMUM_FOOT_VERTICAL_NORMAL",
    "SpawnClearanceError",
    "apply_root_pose_and_audit",
    "audit_spawn_clearance",
)
