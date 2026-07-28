#!/usr/bin/env python3
"""Fail-closed MuJoCo spawn-clearance auditing for Matrix robots.

The contact classifier deliberately depends only on MuJoCo-shaped model/data
objects.  Importing MuJoCo is left to the caller so the safety contract remains
cheap to unit test and can be reused by launch preflight and evidence tooling.
"""

from __future__ import annotations

import math
import operator
import re
import weakref
from collections.abc import Mapping
from typing import Any, Callable

try:
    import numpy as _np
except ImportError:  # Keep the contact-only helper importable in minimal tooling.
    _np = None


AUDIT_SCHEMA = "matrix-spawn-clearance-audit/v1"
GROUND_SUPPORT_SCHEMA = "matrix-ground-support-probe/v1"
BODY_PENETRATION_TOLERANCE_M = 0.002
FOOT_PENETRATION_TOLERANCE_M = 0.015
MINIMUM_FOOT_VERTICAL_NORMAL = 0.8
MAXIMUM_GROUND_SUPPORT_DROP_M = 0.12
MINIMUM_GROUND_SUPPORT_NORMAL_Z = 0.8
REQUIRED_GROUND_SUPPORT_HITS = 1
REQUIRED_MOON_GROUND_SUPPORT_HITS = 2
MAXIMUM_MOON_FOOT_SUPPORT_HEIGHT_DELTA_M = 0.04
MOON_CONTINUOUS_SUPPORT_ASSET_NAME = "matrix_moon_continuous_support_hfield"
MOON_CONTINUOUS_SUPPORT_GEOM_NAME = "matrix_moon_continuous_support"
MOON_SPAWN_PAD_GEOM_NAME = "matrix_moon_spawn_pad"
_MUJOCO_HFIELD_GEOM_TYPE = 1
_ROLLBACK_CLEARANCE_REASONS = frozenset(
    {"no_ground_support", "scene_penetration", "unsafe_foot_contact"}
)

_PELVIS_BODY_NAME = "pelvis"
_FOOT_BODY_NAMES = (
    "left_ankle_roll_link",
    "right_ankle_roll_link",
)
_PERSISTENT_MOON_MOCAP_BODY_RE = re.compile(
    r"gb_(?:[0-9]|1[0-5])_(?:[0-9]|1[0-5])\Z"
)


class _GroundSupportTopology:
    __slots__ = (
        "compatible_scene_geom_arrays",
        "compatible_scene_geom_arrays_by_probe",
        "compatible_scene_geom_ids",
        "compatible_scene_geom_ids_by_probe",
        "foot_geom_ids",
        "foot_ids",
    )

    def __init__(
        self,
        *,
        foot_ids: tuple[int, ...],
        foot_geom_ids: dict[int, tuple[int, ...]],
        compatible_scene_geom_ids: dict[int, tuple[int, ...]],
        compatible_scene_geom_arrays: dict[int, Any],
        compatible_scene_geom_arrays_by_probe: dict[tuple[int, int], Any],
        compatible_scene_geom_ids_by_probe: dict[tuple[int, int], tuple[int, ...]],
    ) -> None:
        self.foot_ids = foot_ids
        self.foot_geom_ids = foot_geom_ids
        self.compatible_scene_geom_ids = compatible_scene_geom_ids
        self.compatible_scene_geom_arrays = compatible_scene_geom_arrays
        self.compatible_scene_geom_arrays_by_probe = (
            compatible_scene_geom_arrays_by_probe
        )
        self.compatible_scene_geom_ids_by_probe = (
            compatible_scene_geom_ids_by_probe
        )


_GROUND_SUPPORT_TOPOLOGY_CACHE: weakref.WeakKeyDictionary[
    Any, _GroundSupportTopology
] = weakref.WeakKeyDictionary()


class SpawnClearanceError(ValueError):
    """Raised internally when an audit input cannot be trusted."""


def spawn_clearance_rollback_reason(audit: dict[str, object] | None) -> str | None:
    """Return the typed rollback reason for trusted serialized audit evidence."""

    if not isinstance(audit, dict):
        return None
    reason = audit.get("reason")
    if (
        audit.get("schema") != AUDIT_SCHEMA
        or audit.get("safe") is not False
        or audit.get("error") is not None
        or reason not in _ROLLBACK_CLEARANCE_REASONS
    ):
        return None
    if reason == "no_ground_support":
        support = audit.get("support")
        rejected_count = audit.get("rejected_contact_count")
        required_hits = (
            support.get("required_hits") if isinstance(support, dict) else None
        )
        accepted_hits = (
            support.get("accepted_hits") if isinstance(support, dict) else None
        )
        moon_spawn_gate = (
            support.get("moon_spawn_gate") if isinstance(support, dict) else None
        )
        ray_direction = (
            support.get("ray_direction") if isinstance(support, dict) else None
        )
        if (
            not isinstance(support, dict)
            or support.get("schema") != GROUND_SUPPORT_SCHEMA
            or support.get("supported") is not False
            or isinstance(required_hits, bool)
            or not isinstance(required_hits, int)
            or required_hits not in {1, 2}
            or isinstance(accepted_hits, bool)
            or not isinstance(accepted_hits, int)
            or not 0 <= accepted_hits <= required_hits
            or (required_hits == 1 and accepted_hits != 0)
            or type(moon_spawn_gate) is not bool
            or moon_spawn_gate != (required_hits == 2)
            or support.get("required_distinct_feet") != required_hits
            or support.get("method") != "downward_foot_geom_rays"
            or support.get("maximum_drop_m") != MAXIMUM_GROUND_SUPPORT_DROP_M
            or support.get("minimum_normal_z") != MINIMUM_GROUND_SUPPORT_NORMAL_Z
            or not isinstance(ray_direction, list)
            or len(ray_direction) != 3
            or any(
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
                for value in ray_direction
            )
            or [float(value) for value in ray_direction] != [0.0, 0.0, -1.0]
            or isinstance(rejected_count, bool)
            or rejected_count != 0
        ):
            return None
        probes = support.get("probes")
        if not isinstance(probes, list) or len(probes) != 2:
            return None
        supported_foot_names = support.get("supported_foot_names")
        height_delta_m = support.get("height_delta_m")
        maximum_height_delta_m = support.get("maximum_height_delta_m")
        height_delta_safe = support.get("height_delta_safe")
        rejection_reason = support.get("rejection_reason")
        if (
            not isinstance(supported_foot_names, list)
            or any(not isinstance(name, str) for name in supported_foot_names)
            or len(supported_foot_names) != accepted_hits
            or type(height_delta_safe) is not bool
            or height_delta_safe is not (required_hits == 1)
            or (
                required_hits == 1
                and (
                    height_delta_m is not None
                    or maximum_height_delta_m is not None
                    or rejection_reason != "insufficient_distinct_foot_support"
                )
            )
            or (
                required_hits == 2
                and (
                    maximum_height_delta_m
                    != MAXIMUM_MOON_FOOT_SUPPORT_HEIGHT_DELTA_M
                    or rejection_reason
                    not in {
                        "insufficient_distinct_foot_support",
                        "foot_support_height_delta",
                    }
                )
            )
        ):
            return None
        expected_names = {
            "left_ankle_roll_link",
            "right_ankle_roll_link",
        }
        observed_names: set[str] = set()
        accepted_names: list[str] = []
        accepted_support_heights: list[float] = []
        for probe in probes:
            if not isinstance(probe, dict):
                return None
            foot_body = probe.get("foot_body")
            origins = probe.get("origins")
            if (
                not isinstance(foot_body, dict)
                or isinstance(foot_body.get("id"), bool)
                or not isinstance(foot_body.get("id"), int)
                or int(foot_body["id"]) <= 0
                or foot_body.get("name") not in expected_names
                or not isinstance(origins, list)
                or not 1 <= len(origins) <= 32
                or type(probe.get("accepted")) is not bool
            ):
                return None
            observed_geom_ids: set[int] = set()
            for origin in origins:
                if not isinstance(origin, dict):
                    return None
                geom_id = origin.get("geom_id")
                geom_name = origin.get("geom_name")
                position = origin.get("position_m")
                if (
                    isinstance(geom_id, bool)
                    or not isinstance(geom_id, int)
                    or geom_id < 0
                    or geom_id in observed_geom_ids
                    or (
                        geom_name is not None
                        and (
                            not isinstance(geom_name, str)
                            or not geom_name
                            or len(geom_name) > 256
                        )
                    )
                    or not isinstance(position, list)
                    or len(position) != 3
                    or any(
                        isinstance(value, bool)
                        or not isinstance(value, (int, float))
                        or not math.isfinite(float(value))
                        for value in position
                    )
                ):
                    return None
                observed_geom_ids.add(geom_id)
            foot_name = str(foot_body["name"])
            observed_names.add(foot_name)
            if probe["accepted"] is False:
                if any(
                    probe.get(field) is not None
                    for field in (
                        "distance_m",
                        "normal",
                        "probe_geom",
                        "ray_origin_m",
                        "scene_geom",
                        "support_height_m",
                    )
                ):
                    return None
                continue
            distance = probe.get("distance_m")
            normal = probe.get("normal")
            probe_geom = probe.get("probe_geom")
            ray_origin = probe.get("ray_origin_m")
            scene_geom = probe.get("scene_geom")
            support_height = probe.get("support_height_m")
            if (
                isinstance(distance, bool)
                or not isinstance(distance, (int, float))
                or not 0.0 <= float(distance) <= MAXIMUM_GROUND_SUPPORT_DROP_M
                or not isinstance(normal, list)
                or len(normal) != 3
                or any(
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not math.isfinite(float(value))
                    for value in normal
                )
                or float(normal[2]) < MINIMUM_GROUND_SUPPORT_NORMAL_Z
                or not isinstance(probe_geom, dict)
                or isinstance(probe_geom.get("id"), bool)
                or not isinstance(probe_geom.get("id"), int)
                or probe_geom.get("id") not in observed_geom_ids
                or not isinstance(probe_geom.get("name"), str)
                or not probe_geom.get("name")
                or not isinstance(ray_origin, list)
                or len(ray_origin) != 3
                or any(
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not math.isfinite(float(value))
                    for value in ray_origin
                )
                or not isinstance(scene_geom, dict)
                or isinstance(scene_geom.get("id"), bool)
                or not isinstance(scene_geom.get("id"), int)
                or scene_geom.get("id") < 0
                or not isinstance(scene_geom.get("name"), str)
                or not scene_geom.get("name")
                or isinstance(support_height, bool)
                or not isinstance(support_height, (int, float))
                or not math.isfinite(float(support_height))
                or not math.isclose(
                    float(support_height),
                    float(ray_origin[2]) - float(distance),
                    rel_tol=0.0,
                    abs_tol=1e-9,
                )
                or (
                    moon_spawn_gate
                    and scene_geom.get("name")
                    not in {
                        MOON_CONTINUOUS_SUPPORT_GEOM_NAME,
                        MOON_SPAWN_PAD_GEOM_NAME,
                    }
                )
            ):
                return None
            accepted_names.append(foot_name)
            accepted_support_heights.append(float(support_height))
        observed_height_delta = (
            max(accepted_support_heights) - min(accepted_support_heights)
            if len(accepted_support_heights) >= 2
            else None
        )
        if (
            observed_names != expected_names
            or accepted_hits != len(accepted_names)
            or supported_foot_names != accepted_names
            or (height_delta_m is None and observed_height_delta is not None)
            or (
                height_delta_m is not None
                and (
                    isinstance(height_delta_m, bool)
                    or not isinstance(height_delta_m, (int, float))
                    or observed_height_delta is None
                    or not math.isclose(
                        float(height_delta_m),
                        observed_height_delta,
                        rel_tol=0.0,
                        abs_tol=1e-9,
                    )
                )
            )
            or (
                required_hits == 2
                and accepted_hits < 2
                and rejection_reason != "insufficient_distinct_foot_support"
            )
            or (
                required_hits == 2
                and accepted_hits == 2
                and (
                    rejection_reason != "foot_support_height_delta"
                    or observed_height_delta is None
                    or observed_height_delta
                    <= MAXIMUM_MOON_FOOT_SUPPORT_HEIGHT_DELTA_M
                )
            )
        ):
            return None
        return str(reason)
    rejected_count = audit.get("rejected_contact_count")
    worst = audit.get("worst")
    if (
        isinstance(rejected_count, bool)
        or not isinstance(rejected_count, int)
        or rejected_count <= 0
        or not isinstance(worst, dict)
        or worst.get("allowed") is not False
    ):
        return None
    classification = worst.get("classification")
    expected_classifications = (
        {"scene_penetration", "unsafe_body_contact"}
        if reason == "scene_penetration"
        else {
            "unsafe_foot_contact_normal",
            "unsafe_foot_penetration",
            "unsafe_foot_terrain_edge",
        }
    )
    if classification not in expected_classifications:
        return None
    return str(reason)


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


def _persistent_support_body(model: Any, body_id: int, *, nbody: int) -> bool:
    """Accept static world geometry and the deterministic Moon tile mocaps.

    Freejoint items are intentionally excluded: Matrix currently persists only
    the robot pose, so an item supporting the feet may return to storage on the
    next launch and cannot authorize a durable checkpoint.
    """

    try:
        parents = model.body_parentid
        body_joint_counts = model.body_jntnum
        body_mocap_ids = model.body_mocapid
    except AttributeError as exc:
        raise SpawnClearanceError(
            "model persistent-support body metadata is unavailable"
        ) from exc
    current = body_id
    visited: set[int] = set()
    while current > 0:
        if current in visited or not 0 <= current < nbody:
            raise SpawnClearanceError(
                "model persistent-support body tree is invalid"
            )
        visited.add(current)
        try:
            joint_count = _index(
                body_joint_counts[current],
                label=f"model.body_jntnum[{current}]",
            )
            mocap_id = _index(
                body_mocap_ids[current],
                label=f"model.body_mocapid[{current}]",
            )
            parent = _index(
                parents[current],
                label=f"model.body_parentid[{current}]",
            )
        except (IndexError, KeyError, TypeError) as exc:
            raise SpawnClearanceError(
                "model persistent-support body table is truncated"
            ) from exc
        if joint_count < 0 or mocap_id < -1 or not 0 <= parent < nbody:
            raise SpawnClearanceError(
                "model persistent-support body metadata is invalid"
            )
        if joint_count > 0:
            return False
        if mocap_id >= 0:
            name = _optional_name(model, "body", current)
            return bool(
                isinstance(name, str)
                and _PERSISTENT_MOON_MOCAP_BODY_RE.fullmatch(name)
            )
        current = parent
    return True


def _is_moon_mocap_body(model: Any, body_id: int, *, nbody: int) -> bool:
    """Identify one exact, live MoonWorld rolling-tile mocap body."""

    if not 0 <= body_id < nbody:
        raise SpawnClearanceError("MoonWorld body id is outside the body table")
    name = _optional_name(model, "body", body_id)
    if (
        not isinstance(name, str)
        or _PERSISTENT_MOON_MOCAP_BODY_RE.fullmatch(name) is None
    ):
        return False
    try:
        mocap_id = _index(
            model.body_mocapid[body_id],
            label=f"model.body_mocapid[{body_id}]",
        )
        nmocap = _index(model.nmocap, label="model.nmocap")
    except (AttributeError, IndexError, KeyError, TypeError) as exc:
        raise SpawnClearanceError(
            "MoonWorld mocap body metadata is unavailable"
        ) from exc
    if nmocap < 0 or mocap_id < -1:
        raise SpawnClearanceError("MoonWorld mocap body metadata is invalid")
    return 0 <= mocap_id < nmocap


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


def _moon_ground_collision_contract(
    model: Any,
    *,
    ngeom: int,
) -> dict[str, object] | None:
    """Validate one legal Moon phase and identify every active ground geom."""
    matches = [
        geom_id
        for geom_id in range(ngeom)
        if _optional_name(model, "geom", geom_id)
        == MOON_CONTINUOUS_SUPPORT_GEOM_NAME
    ]
    if not matches:
        return None
    if len(matches) != 1:
        raise SpawnClearanceError(
            "MoonWorld continuous support geom must be unique"
        )
    geom_id = matches[0]
    nhfield = _model_count(model, "nhfield")
    hfield_matches = [
        hfield_id
        for hfield_id in range(nhfield)
        if _optional_name(model, "hfield", hfield_id)
        == MOON_CONTINUOUS_SUPPORT_ASSET_NAME
    ]
    if not hfield_matches:
        raise SpawnClearanceError(
            "MoonWorld continuous support hfield asset is missing"
        )
    if len(hfield_matches) != 1:
        raise SpawnClearanceError(
            "MoonWorld continuous support hfield asset must be unique"
        )
    hfield_id = hfield_matches[0]
    try:
        body_id = _index(
            model.geom_bodyid[geom_id],
            label=f"model.geom_bodyid[{geom_id}]",
        )
        geom_type = _index(
            model.geom_type[geom_id],
            label=f"model.geom_type[{geom_id}]",
        )
        data_id = _index(
            model.geom_dataid[geom_id],
            label=f"model.geom_dataid[{geom_id}]",
        )
        contype = _index(
            model.geom_contype[geom_id],
            label=f"model.geom_contype[{geom_id}]",
        )
        conaffinity = _index(
            model.geom_conaffinity[geom_id],
            label=f"model.geom_conaffinity[{geom_id}]",
        )
    except (AttributeError, IndexError, KeyError, TypeError) as exc:
        raise SpawnClearanceError(
            "MoonWorld continuous support collision metadata is unavailable"
        ) from exc
    if body_id != 0:
        raise SpawnClearanceError(
            "MoonWorld continuous support must be attached to the world body"
        )
    if geom_type != _MUJOCO_HFIELD_GEOM_TYPE:
        raise SpawnClearanceError(
            "MoonWorld continuous support geom must be a MuJoCo hfield"
        )
    if data_id != hfield_id:
        raise SpawnClearanceError(
            "MoonWorld continuous support geom must bind the named hfield asset"
        )
    if (contype, conaffinity) not in {(0, 0), (1, 1)}:
        raise SpawnClearanceError(
            "MoonWorld continuous hfield collision mask is invalid"
        )
    pad_matches = [
        pad_id
        for pad_id in range(ngeom)
        if _optional_name(model, "geom", pad_id) == MOON_SPAWN_PAD_GEOM_NAME
    ]
    if not pad_matches:
        raise SpawnClearanceError("MoonWorld collision-only spawn pad is missing")
    if len(pad_matches) != 1:
        raise SpawnClearanceError(
            "MoonWorld collision-only spawn pad must be unique"
        )
    pad_id = pad_matches[0]
    try:
        pad_body_id = _index(
            model.geom_bodyid[pad_id], label=f"model.geom_bodyid[{pad_id}]"
        )
        pad_contype = _index(
            model.geom_contype[pad_id], label=f"model.geom_contype[{pad_id}]"
        )
        pad_conaffinity = _index(
            model.geom_conaffinity[pad_id],
            label=f"model.geom_conaffinity[{pad_id}]",
        )
    except (AttributeError, IndexError, KeyError, TypeError) as exc:
        raise SpawnClearanceError(
            "MoonWorld spawn-pad collision metadata is unavailable"
        ) from exc
    if pad_body_id != 0:
        raise SpawnClearanceError(
            "MoonWorld collision-only spawn pad must be attached to the world body"
        )
    tile_geom_ids: list[int] = []
    tile_masks: set[tuple[int, int]] = set()
    for tile_geom_id in range(ngeom):
        geom_name = _optional_name(model, "geom", tile_geom_id)
        if not isinstance(geom_name, str) or not geom_name.startswith("soil_"):
            continue
        try:
            tile_body_id = _index(
                model.geom_bodyid[tile_geom_id],
                label=f"model.geom_bodyid[{tile_geom_id}]",
            )
            tile_contype = _index(
                model.geom_contype[tile_geom_id],
                label=f"model.geom_contype[{tile_geom_id}]",
            )
            tile_conaffinity = _index(
                model.geom_conaffinity[tile_geom_id],
                label=f"model.geom_conaffinity[{tile_geom_id}]",
            )
        except (AttributeError, IndexError, KeyError, TypeError) as exc:
            raise SpawnClearanceError(
                "MoonWorld rolling-tile collision metadata is unavailable"
            ) from exc
        if not _is_moon_mocap_body(model, tile_body_id, nbody=_model_count(model, "nbody")):
            raise SpawnClearanceError(
                f"MoonWorld rolling tile is not on a persistent mocap body: {geom_name}"
            )
        tile_geom_ids.append(tile_geom_id)
        tile_masks.add((tile_contype, tile_conaffinity))

    pad_mask = (pad_contype, pad_conaffinity)
    support_mask = (contype, conaffinity)
    if (
        support_mask == (0, 0)
        and pad_mask == (1, 1)
        and (not tile_masks or tile_masks == {(0, 0)})
    ):
        return {
            "phase": "spawn-pad",
            "active_ground_geom_ids": frozenset((pad_id,)),
            "observation_hfield_geom_id": geom_id,
            "spawn_pad_geom_id": pad_id,
            "rolling_tile_geom_ids": tuple(tile_geom_ids),
        }
    if (
        support_mask == (1, 1)
        and pad_mask == (0, 0)
        and tile_geom_ids
        and tile_masks == {(0, 0)}
    ):
        return {
            "phase": "rolling-heightfield",
            "active_ground_geom_ids": frozenset((geom_id,)),
            "observation_hfield_geom_id": geom_id,
            "spawn_pad_geom_id": pad_id,
            "rolling_tile_geom_ids": tuple(tile_geom_ids),
        }
    raise SpawnClearanceError(
        "MoonWorld collision handoff must have exactly one active ground: "
        f"continuous_hfield={support_mask} "
        f"visual_tiles={sorted(tile_masks)} spawn_pad={pad_mask}"
    )


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
        moon_ground = _moon_ground_collision_contract(
            model,
            ngeom=ngeom,
        )
        moon_spawn_gate = moon_ground is not None
        moon_active_ground_geom_ids = (
            moon_ground["active_ground_geom_ids"]
            if moon_ground is not None
            else frozenset()
        )
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
                scene_body_id = body2 if robot1 else body1
                scene_geom_id = geom2 if robot1 else geom1
                moon_terrain_edge = _is_moon_mocap_body(
                    model,
                    scene_body_id,
                    nbody=nbody,
                )
                allowed = False
                if (
                    moon_terrain_edge
                    and scene_geom_id not in moon_active_ground_geom_ids
                ):
                    classification = "unsafe_foot_terrain_edge"
                elif (
                    support_normal >= thresholds["minimum_foot_vertical_normal"]
                    and distance
                    >= -thresholds["foot_penetration_tolerance_m"]
                ):
                    allowed = True
                    classification = "allowed_foot_support"
                elif support_normal < thresholds["minimum_foot_vertical_normal"]:
                    classification = "unsafe_foot_contact_normal"
                else:
                    classification = "unsafe_foot_penetration"
            else:
                if moon_spawn_gate:
                    allowed = False
                    classification = "unsafe_body_contact"
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
            "moon_spawn_gate": {
                "enabled": moon_spawn_gate,
                "phase": (
                    moon_ground["phase"] if moon_ground is not None else None
                ),
                "continuous_support_geom_id": (
                    moon_ground["observation_hfield_geom_id"]
                    if moon_ground is not None
                    else None
                ),
                "active_ground_geom_ids": (
                    sorted(moon_active_ground_geom_ids)
                    if moon_ground is not None
                    else []
                ),
                "required_supported_feet": (
                    REQUIRED_MOON_GROUND_SUPPORT_HITS if moon_spawn_gate else 1
                ),
                "maximum_foot_support_height_delta_m": (
                    MAXIMUM_MOON_FOOT_SUPPORT_HEIGHT_DELTA_M
                    if moon_spawn_gate
                    else None
                ),
                "foot_terrain_edge_allowed": False,
                "body_contact_allowed": not moon_spawn_gate,
            },
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


def _collision_compatible(
    model: Any,
    *,
    robot_geom_ids: tuple[int, ...],
    scene_geom_id: int,
) -> bool:
    try:
        scene_contype = _index(
            model.geom_contype[scene_geom_id],
            label=f"model.geom_contype[{scene_geom_id}]",
        )
        scene_conaffinity = _index(
            model.geom_conaffinity[scene_geom_id],
            label=f"model.geom_conaffinity[{scene_geom_id}]",
        )
    except (AttributeError, IndexError, KeyError, TypeError) as exc:
        raise SpawnClearanceError("model geom collision masks are unavailable") from exc
    if scene_contype < 0 or scene_conaffinity < 0:
        raise SpawnClearanceError("model geom collision masks must be non-negative")
    for geom_id in robot_geom_ids:
        try:
            robot_contype = _index(
                model.geom_contype[geom_id],
                label=f"model.geom_contype[{geom_id}]",
            )
            robot_conaffinity = _index(
                model.geom_conaffinity[geom_id],
                label=f"model.geom_conaffinity[{geom_id}]",
            )
        except (AttributeError, IndexError, KeyError, TypeError) as exc:
            raise SpawnClearanceError(
                "model robot geom collision masks are unavailable"
            ) from exc
        if robot_contype < 0 or robot_conaffinity < 0:
            raise SpawnClearanceError("model geom collision masks must be non-negative")
        if (
            robot_contype & scene_conaffinity
            or scene_contype & robot_conaffinity
        ):
            return True
    return False


def _ground_support_topology(model: Any) -> _GroundSupportTopology:
    try:
        cached = _GROUND_SUPPORT_TOPOLOGY_CACHE.get(model)
    except TypeError:
        cached = None
    if cached is not None:
        return cached

    nbody = _model_count(model, "nbody")
    ngeom = _geom_count(model)
    pelvis_id = _named_body_id(model, _PELVIS_BODY_NAME, nbody=nbody)
    foot_ids = tuple(
        _named_body_id(model, name, nbody=nbody) for name in _FOOT_BODY_NAMES
    )
    robot_ids = _robot_body_ids(
        model,
        root_body_id=pelvis_id,
        nbody=nbody,
    )
    try:
        geom_body_ids = model.geom_bodyid
    except AttributeError as exc:
        raise SpawnClearanceError("model geometry body table is unavailable") from exc

    scene_geom_ids: list[int] = []
    foot_geom_ids: dict[int, tuple[int, ...]] = {}
    for geom_id in range(ngeom):
        try:
            body_id = _index(
                geom_body_ids[geom_id],
                label=f"model.geom_bodyid[{geom_id}]",
            )
        except (IndexError, KeyError, TypeError) as exc:
            raise SpawnClearanceError("model geom body table is truncated") from exc
        if not 0 <= body_id < nbody:
            raise SpawnClearanceError("model geom body id is outside the body table")
        if body_id not in robot_ids and _persistent_support_body(
            model,
            body_id,
            nbody=nbody,
        ):
            scene_geom_ids.append(geom_id)
    for foot_id in foot_ids:
        owned = tuple(
            geom_id
            for geom_id in range(ngeom)
            if _index(
                geom_body_ids[geom_id],
                label=f"model.geom_bodyid[{geom_id}]",
            )
            == foot_id
        )
        if not owned:
            raise SpawnClearanceError(
                f"foot body {_optional_name(model, 'body', foot_id)!r} has no geoms"
            )
        foot_geom_ids[foot_id] = owned

    compatible_scene_geom_ids: dict[int, tuple[int, ...]] = {}
    compatible_scene_geom_ids_by_probe: dict[
        tuple[int, int], tuple[int, ...]
    ] = {}
    for foot_id in foot_ids:
        compatible_by_probe_geom = {
            robot_geom_id: tuple(
                scene_geom_id
                for scene_geom_id in scene_geom_ids
                if _collision_compatible(
                    model,
                    robot_geom_ids=(robot_geom_id,),
                    scene_geom_id=scene_geom_id,
                )
            )
            for robot_geom_id in foot_geom_ids[foot_id]
        }
        probe_geom_ids = tuple(
            geom_id
            for geom_id, compatible in compatible_by_probe_geom.items()
            if compatible
        )
        if not probe_geom_ids:
            raise SpawnClearanceError(
                f"foot body {_optional_name(model, 'body', foot_id)!r} "
                "has no collision-compatible geoms"
            )
        foot_geom_ids[foot_id] = probe_geom_ids
        for geom_id in probe_geom_ids:
            compatible_scene_geom_ids_by_probe[(foot_id, geom_id)] = (
                compatible_by_probe_geom[geom_id]
            )
        compatible_scene_geom_ids[foot_id] = tuple(
            sorted(
                {
                    scene_geom_id
                    for geom_id in probe_geom_ids
                    for scene_geom_id in compatible_by_probe_geom[geom_id]
                }
            )
        )
    compatible_scene_geom_arrays = {
        foot_id: (
            _np.asarray(geom_ids, dtype=_np.int64)
            if _np is not None
            else None
        )
        for foot_id, geom_ids in compatible_scene_geom_ids.items()
    }
    compatible_scene_geom_arrays_by_probe = {
        key: (
            _np.asarray(geom_ids, dtype=_np.int64)
            if _np is not None
            else None
        )
        for key, geom_ids in compatible_scene_geom_ids_by_probe.items()
    }
    topology = _GroundSupportTopology(
        foot_ids=foot_ids,
        foot_geom_ids=foot_geom_ids,
        compatible_scene_geom_ids=compatible_scene_geom_ids,
        compatible_scene_geom_arrays=compatible_scene_geom_arrays,
        compatible_scene_geom_arrays_by_probe=(
            compatible_scene_geom_arrays_by_probe
        ),
        compatible_scene_geom_ids_by_probe=compatible_scene_geom_ids_by_probe,
    )
    try:
        _GROUND_SUPPORT_TOPOLOGY_CACHE[model] = topology
    except TypeError:
        pass
    return topology


def _ground_support_broadphase(
    mujoco: Any,
    model: Any,
    data: Any,
    *,
    topology: _GroundSupportTopology,
    foot_id: int,
    probe_geom_id: int,
    origin: Any,
) -> tuple[int, ...]:
    geom_ids = topology.compatible_scene_geom_ids_by_probe[
        (foot_id, probe_geom_id)
    ]
    if not geom_ids:
        return ()
    try:
        plane_type = _index(
            mujoco.mjtGeom.mjGEOM_PLANE,
            label="MuJoCo plane geom type",
        )
        geom_positions = data.geom_xpos
        geom_rbounds = model.geom_rbound
        geom_types = model.geom_type
    except AttributeError as exc:
        raise SpawnClearanceError(
            "ground-support broadphase tables are unavailable"
        ) from exc

    if _np is not None:
        ids = topology.compatible_scene_geom_arrays_by_probe[
            (foot_id, probe_geom_id)
        ]
        try:
            positions = _np.asarray(geom_positions)[ids, :2]
            rbounds = _np.asarray(geom_rbounds)[ids]
            types = _np.asarray(geom_types)[ids]
            origin_xy = _np.asarray(origin, dtype=_np.float64)[:2]
        except (IndexError, TypeError, ValueError) as exc:
            raise SpawnClearanceError(
                "ground-support broadphase tables are invalid"
            ) from exc
        if (
            positions.shape != (len(geom_ids), 2)
            or rbounds.shape != (len(geom_ids),)
            or types.shape != (len(geom_ids),)
            or not _np.all(_np.isfinite(positions))
            or not _np.all(_np.isfinite(rbounds))
            or _np.any(rbounds < 0.0)
            or not _np.all(_np.isfinite(origin_xy))
        ):
            raise SpawnClearanceError(
                "ground-support broadphase tables contain invalid values"
            )
        delta = positions - origin_xy
        horizontal_distance_sq = _np.einsum("ij,ij->i", delta, delta)
        keep = (
            (types == plane_type)
            | (rbounds <= 0.0)
            | (horizontal_distance_sq <= rbounds * rbounds)
        )
        return tuple(int(value) for value in ids[keep])

    selected: list[int] = []
    origin_xy = _vector(origin, length=2, label="ground-support origin xy")
    for geom_id in geom_ids:
        geom_position = _vector(
            geom_positions[geom_id],
            length=2,
            label=f"data.geom_xpos[{geom_id}]",
        )
        rbound = _finite(
            geom_rbounds[geom_id],
            label=f"model.geom_rbound[{geom_id}]",
        )
        if rbound < 0.0:
            raise SpawnClearanceError("model geom bounding radius must be non-negative")
        geom_type = _index(
            geom_types[geom_id],
            label=f"model.geom_type[{geom_id}]",
        )
        horizontal_distance_sq = (
            (geom_position[0] - origin_xy[0]) ** 2
            + (geom_position[1] - origin_xy[1]) ** 2
        )
        if (
            geom_type == plane_type
            or rbound <= 0.0
            or horizontal_distance_sq <= rbound * rbound
        ):
            selected.append(geom_id)
    return tuple(selected)


def _ray_distance_and_normal(
    mujoco: Any,
    model: Any,
    data: Any,
    *,
    geom_id: int,
    origin: Any,
    direction: Any,
) -> tuple[float, tuple[float, float, float]]:
    try:
        geom_type = _index(
            model.geom_type[geom_id],
            label=f"model.geom_type[{geom_id}]",
        )
        mesh_type = _index(
            mujoco.mjtGeom.mjGEOM_MESH,
            label="MuJoCo mesh geom type",
        )
        hfield_type = _index(
            mujoco.mjtGeom.mjGEOM_HFIELD,
            label="MuJoCo hfield geom type",
        )
        normal = origin.copy()
        normal[:] = (0.0, 0.0, 0.0)
        if geom_type == mesh_type:
            distance = mujoco.mj_rayMesh(
                model,
                data,
                geom_id,
                origin,
                direction,
                normal,
            )
        elif geom_type == hfield_type:
            distance = mujoco.mj_rayHfield(
                model,
                data,
                geom_id,
                origin,
                direction,
                normal,
            )
        else:
            distance = mujoco.mju_rayGeom(
                data.geom_xpos[geom_id],
                data.geom_xmat[geom_id],
                model.geom_size[geom_id],
                origin,
                direction,
                geom_type,
                normal,
            )
    except SpawnClearanceError:
        raise
    except (AttributeError, IndexError, KeyError, TypeError, ValueError) as exc:
        raise SpawnClearanceError(
            f"ground-support ray failed for geom {geom_id}"
        ) from exc
    return (
        _finite(distance, label=f"ground-support ray distance for geom {geom_id}"),
        _vector(normal, length=3, label=f"ground-support normal for geom {geom_id}"),
    )


def probe_ground_support(
    mujoco: Any,
    model: Any,
    data: Any,
    *,
    maximum_drop_m: float = MAXIMUM_GROUND_SUPPORT_DROP_M,
    minimum_normal_z: float = MINIMUM_GROUND_SUPPORT_NORMAL_Z,
    maximum_foot_support_height_delta_m: float = (
        MAXIMUM_MOON_FOOT_SUPPORT_HEIGHT_DELTA_M
    ),
) -> dict[str, object]:
    """Probe below collision-capable foot geoms for scene support."""

    maximum_drop = _finite(maximum_drop_m, label="maximum ground-support drop")
    minimum_normal = _finite(
        minimum_normal_z,
        label="minimum ground-support normal z",
    )
    maximum_height_delta = _finite(
        maximum_foot_support_height_delta_m,
        label="maximum foot-support height delta",
    )
    if maximum_drop <= 0.0:
        raise SpawnClearanceError("maximum ground-support drop must be positive")
    if not 0.0 < minimum_normal <= 1.0:
        raise SpawnClearanceError("minimum ground-support normal z must be in (0, 1]")
    if maximum_height_delta < 0.0:
        raise SpawnClearanceError(
            "maximum foot-support height delta must be non-negative"
        )
    ngeom = _geom_count(model)
    moon_ground = _moon_ground_collision_contract(
        model,
        ngeom=ngeom,
    )
    moon_spawn_gate = moon_ground is not None
    moon_active_ground_geom_ids = (
        moon_ground["active_ground_geom_ids"]
        if moon_ground is not None
        else frozenset()
    )
    topology = _ground_support_topology(model)
    required_hits = (
        REQUIRED_MOON_GROUND_SUPPORT_HITS
        if moon_spawn_gate
        else REQUIRED_GROUND_SUPPORT_HITS
    )
    try:
        geom_body_ids = model.geom_bodyid
    except AttributeError as exc:
        raise SpawnClearanceError(
            "ground-support model/data geometry tables are unavailable"
        ) from exc

    probes: list[dict[str, object]] = []
    accepted_hits = 0
    support_heights_m: list[float] = []
    for foot_name, foot_id in zip(_FOOT_BODY_NAMES, topology.foot_ids):
        origins: list[dict[str, object]] = []
        best: (
            tuple[
                float,
                tuple[float, float, float],
                int,
                int,
                int,
                list[float],
            ]
            | None
        ) = None
        for probe_geom_id in topology.foot_geom_ids[foot_id]:
            try:
                origin = data.geom_xpos[probe_geom_id].copy()
                direction = data.geom_xpos[probe_geom_id].copy()
                direction[:] = (0.0, 0.0, -1.0)
            except (
                AttributeError,
                IndexError,
                KeyError,
                TypeError,
                ValueError,
            ) as exc:
                raise SpawnClearanceError(
                    f"ground-support origin is unavailable for {foot_name}"
                ) from exc
            origin_mapping = list(
                _vector(
                    origin,
                    length=3,
                    label=f"{foot_name} geom {probe_geom_id} support origin",
                )
            )
            origins.append(
                {
                    "geom_id": probe_geom_id,
                    "geom_name": _optional_name(model, "geom", probe_geom_id),
                    "position_m": origin_mapping,
                }
            )
            for geom_id in _ground_support_broadphase(
                mujoco,
                model,
                data,
                topology=topology,
                foot_id=foot_id,
                probe_geom_id=probe_geom_id,
                origin=origin,
            ):
                if moon_spawn_gate and geom_id not in moon_active_ground_geom_ids:
                    continue
                distance, normal = _ray_distance_and_normal(
                    mujoco,
                    model,
                    data,
                    geom_id=geom_id,
                    origin=origin,
                    direction=direction,
                )
                if (
                    distance < 0.0
                    or distance > maximum_drop
                    or normal[2] < minimum_normal
                ):
                    continue
                body_id = _index(
                    geom_body_ids[geom_id],
                    label=f"model.geom_bodyid[{geom_id}]",
                )
                candidate = (
                    distance,
                    normal,
                    geom_id,
                    body_id,
                    probe_geom_id,
                    origin_mapping,
                )
                if best is None or candidate[0] < best[0]:
                    best = candidate
        if best is None:
            probes.append(
                {
                    "foot_body": {"id": foot_id, "name": foot_name},
                    "origins": origins,
                    "accepted": False,
                    "distance_m": None,
                    "normal": None,
                    "probe_geom": None,
                    "ray_origin_m": None,
                    "scene_geom": None,
                    "support_height_m": None,
                }
            )
            continue
        distance, normal, geom_id, body_id, probe_geom_id, ray_origin = best
        support_height_m = float(ray_origin[2]) - distance
        if not math.isfinite(support_height_m):
            raise SpawnClearanceError(
                f"ground-support height is non-finite for {foot_name}"
            )
        accepted_hits += 1
        support_heights_m.append(support_height_m)
        probes.append(
            {
                "foot_body": {"id": foot_id, "name": foot_name},
                "origins": origins,
                "accepted": True,
                "distance_m": distance,
                "normal": list(normal),
                "probe_geom": {
                    "id": probe_geom_id,
                    "name": _optional_name(model, "geom", probe_geom_id),
                },
                "ray_origin_m": ray_origin,
                "scene_geom": {
                    "id": geom_id,
                    "name": _optional_name(model, "geom", geom_id),
                    "body_id": body_id,
                    "body_name": _optional_name(model, "body", body_id),
                },
                "support_height_m": support_height_m,
            }
        )
    height_delta_m = (
        max(support_heights_m) - min(support_heights_m)
        if len(support_heights_m) >= 2
        else None
    )
    height_delta_safe = bool(
        not moon_spawn_gate
        or (
            height_delta_m is not None
            and height_delta_m <= maximum_height_delta
        )
    )
    supported = bool(accepted_hits >= required_hits and height_delta_safe)
    return {
        "schema": GROUND_SUPPORT_SCHEMA,
        "supported": supported,
        "method": "downward_foot_geom_rays",
        "required_hits": required_hits,
        "accepted_hits": accepted_hits,
        "required_distinct_feet": required_hits,
        "supported_foot_names": [
            str(probe["foot_body"]["name"])
            for probe in probes
            if probe.get("accepted") is True
            and isinstance(probe.get("foot_body"), dict)
        ],
        "maximum_drop_m": maximum_drop,
        "minimum_normal_z": minimum_normal,
        "height_delta_m": height_delta_m,
        "maximum_height_delta_m": (
            maximum_height_delta if moon_spawn_gate else None
        ),
        "height_delta_safe": height_delta_safe,
        "rejection_reason": (
            None
            if supported
            else (
                "foot_support_height_delta"
                if accepted_hits >= required_hits and not height_delta_safe
                else "insufficient_distinct_foot_support"
            )
        ),
        "moon_spawn_gate": moon_spawn_gate,
        "ray_direction": [0.0, 0.0, -1.0],
        "probes": probes,
    }


def audit_spawn_safety(
    mujoco: Any,
    model: Any,
    data: Any,
    *,
    body_penetration_tolerance_m: float = BODY_PENETRATION_TOLERANCE_M,
    foot_penetration_tolerance_m: float = FOOT_PENETRATION_TOLERANCE_M,
    minimum_foot_vertical_normal: float = MINIMUM_FOOT_VERTICAL_NORMAL,
    maximum_ground_support_drop_m: float = MAXIMUM_GROUND_SUPPORT_DROP_M,
    minimum_ground_support_normal_z: float = MINIMUM_GROUND_SUPPORT_NORMAL_Z,
    maximum_foot_support_height_delta_m: float = (
        MAXIMUM_MOON_FOOT_SUPPORT_HEIGHT_DELTA_M
    ),
) -> dict[str, object]:
    """Combine collision clearance with a replayable below-foot support probe."""

    result = audit_spawn_clearance(
        model,
        data,
        body_penetration_tolerance_m=body_penetration_tolerance_m,
        foot_penetration_tolerance_m=foot_penetration_tolerance_m,
        minimum_foot_vertical_normal=minimum_foot_vertical_normal,
    )
    if result.get("safe") is not True:
        return result
    moon_gate = result.get("moon_spawn_gate")
    moon_spawn_gate = bool(
        isinstance(moon_gate, dict) and moon_gate.get("enabled") is True
    )
    contacts = result.get("contacts")
    allowed_foot_contacts = (
        [
            contact
            for contact in contacts
            if isinstance(contact, dict)
            and contact.get("classification") == "allowed_foot_support"
            and contact.get("allowed") is True
            and isinstance(contact.get("scene_body"), dict)
            and isinstance(contact["scene_body"].get("id"), int)
            and not isinstance(contact["scene_body"].get("id"), bool)
            and _persistent_support_body(
                model,
                int(contact["scene_body"]["id"]),
                nbody=_model_count(model, "nbody"),
            )
        ]
        if isinstance(contacts, list)
        else []
    )
    if allowed_foot_contacts and not moon_spawn_gate:
        supported_feet = {
            str(contact.get("robot_body", {}).get("name"))
            for contact in allowed_foot_contacts
            if isinstance(contact.get("robot_body"), dict)
            and contact["robot_body"].get("name") in _FOOT_BODY_NAMES
        }
        if supported_feet:
            result["support"] = {
                "schema": GROUND_SUPPORT_SCHEMA,
                "supported": True,
                "method": "allowed_foot_contacts",
                "required_hits": REQUIRED_GROUND_SUPPORT_HITS,
                "accepted_hits": len(supported_feet),
                "maximum_drop_m": MAXIMUM_GROUND_SUPPORT_DROP_M,
                "minimum_normal_z": MINIMUM_GROUND_SUPPORT_NORMAL_Z,
                "ray_direction": [0.0, 0.0, -1.0],
                "probes": [],
                "contact_indices": [
                    int(contact["contact_index"])
                    for contact in allowed_foot_contacts
                ],
            }
            return result
    try:
        support = probe_ground_support(
            mujoco,
            model,
            data,
            maximum_drop_m=maximum_ground_support_drop_m,
            minimum_normal_z=minimum_ground_support_normal_z,
            maximum_foot_support_height_delta_m=(
                maximum_foot_support_height_delta_m
            ),
        )
    except Exception as exc:
        return _audit_error(
            exc,
            thresholds=result.get("thresholds"),  # type: ignore[arg-type]
        )
    result["support"] = support
    if support["supported"] is not True:
        result["safe"] = False
        result["reason"] = "no_ground_support"
    return result


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
    data_preparer: Callable[[Any], None] | None = None,
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
        if data_preparer is not None:
            data_preparer(data)
        mujoco.mj_forward(model, data)
        result = audit_spawn_safety(
            mujoco,
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
    "GROUND_SUPPORT_SCHEMA",
    "MAXIMUM_GROUND_SUPPORT_DROP_M",
    "MINIMUM_FOOT_VERTICAL_NORMAL",
    "MINIMUM_GROUND_SUPPORT_NORMAL_Z",
    "REQUIRED_GROUND_SUPPORT_HITS",
    "SpawnClearanceError",
    "apply_root_pose_and_audit",
    "audit_spawn_clearance",
    "audit_spawn_safety",
    "probe_ground_support",
)
