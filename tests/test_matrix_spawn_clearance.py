from __future__ import annotations

from dataclasses import dataclass
import importlib.util
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "matrix_spawn_clearance.py"
SPEC = importlib.util.spec_from_file_location("matrix_spawn_clearance", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


VERTICAL = (0.0, 0.0, 1.0)
HORIZONTAL = (1.0, 0.0, 0.0)
POSITION = (1.0, 2.0, 3.0)


@dataclass
class Contact:
    geom1: int
    geom2: int
    dist: float = 0.0
    frame: tuple[float, ...] = VERTICAL
    pos: tuple[float, ...] = POSITION


class NamedItem:
    def __init__(self, item_id: int, name: str | None):
        self.id = item_id
        self.name = name


class FakeModel:
    """world, pelvis, two feet, torso, and one external scene body."""

    def __init__(self) -> None:
        self.nbody = 6
        self.ngeom = 7
        self.nmocap = 0
        self.nq = 10
        self.body_parentid = (0, 0, 1, 1, 1, 0)
        # left sole, right sole, torso, floor, wall, pelvis, platform
        self.geom_bodyid = (2, 3, 4, 0, 5, 1, 5)
        self.geom_contype = (1,) * self.ngeom
        self.geom_conaffinity = (1,) * self.ngeom
        self.geom_type = (0,) * self.ngeom
        self.geom_size = ([1.0, 1.0, 0.01],) * self.ngeom
        self.geom_rbound = (0.0,) * self.ngeom
        self.body_jntadr = (0, 0, 1, 1, 1, 1)
        self.body_jntnum = (0, 1, 0, 0, 0, 0)
        self.body_mocapid = (-1,) * self.nbody
        self.jnt_type = (0,)
        self.jnt_qposadr = (0,)
        self.qpos0 = [10.0, 20.0, 30.0, 1.0, 0.0, 0.0, 0.0, 7.0, 8.0, 9.0]
        self._bodies = {
            0: "world",
            1: "pelvis",
            2: "left_ankle_roll_link",
            3: "right_ankle_roll_link",
            4: "torso_link",
            5: "platform_body",
        }
        self._geoms = {
            0: "left_sole",
            1: "right_sole",
            2: "torso_collision",
            3: "floor",
            4: "wall",
            5: "pelvis_collision",
            6: "platform",
        }

    def body(self, key: int | str) -> NamedItem:
        if isinstance(key, str):
            for item_id, name in self._bodies.items():
                if name == key:
                    return NamedItem(item_id, name)
            raise KeyError(key)
        if key not in self._bodies:
            raise KeyError(key)
        return NamedItem(key, self._bodies[key])

    def geom(self, key: int) -> NamedItem:
        if key not in self._geoms:
            raise KeyError(key)
        return NamedItem(key, self._geoms[key])


class FakeData:
    def __init__(
        self,
        *contacts: Contact,
        qpos: list[float] | None = None,
    ) -> None:
        self.contact = list(contacts)
        self.ncon = len(contacts)
        self.qpos = list(qpos or [])
        self.xpos = [
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.79],
            [0.0, 0.0, 0.04],
            [1.0, 0.0, 0.04],
            [0.0, 0.0, 0.5],
            [0.0, 0.0, 0.0],
        ]
        self.geom_xpos = [
            [float(geom_id), 0.0, 0.0] for geom_id in range(7)
        ]
        self.geom_xmat = [
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            for _ in range(7)
        ]


class FakeMujoco:
    class mjtJoint:
        mjJNT_FREE = 0

    class mjtGeom:
        mjGEOM_PLANE = 0
        mjGEOM_HFIELD = 1
        mjGEOM_MESH = 7

    def __init__(
        self,
        contacts: tuple[Contact, ...] = (),
        *,
        ray_hits: dict[
            tuple[float, int], tuple[float, tuple[float, float, float]]
        ]
        | None = None,
        default_support: bool = True,
    ) -> None:
        self.contacts = contacts
        self.ray_hits = ray_hits or {}
        self.default_support = default_support
        self.created: list[FakeData] = []
        self.reset_calls = 0
        self.forward_calls = 0
        self.ray_calls: list[tuple[float, int]] = []

    def MjData(self, model: FakeModel) -> FakeData:
        data = FakeData(qpos=[-1.0] * model.nq)
        self.created.append(data)
        return data

    def mj_resetData(self, model: FakeModel, data: FakeData) -> None:
        self.reset_calls += 1
        data.qpos[:] = model.qpos0
        data.contact = []
        data.ncon = 0

    def mj_forward(self, _model: FakeModel, data: FakeData) -> None:
        self.forward_calls += 1
        data.contact = list(self.contacts)
        data.ncon = len(self.contacts)

    def mju_rayGeom(
        self,
        geom_pos,
        _geom_mat,
        _geom_size,
        origin,
        _direction,
        _geom_type,
        normal,
    ) -> float:
        key = (float(origin[0]), int(geom_pos[0]))
        self.ray_calls.append(key)
        hit = self.ray_hits.get(key)
        if hit is None and self.default_support and key[1] == 3:
            hit = (0.04, VERTICAL)
        if hit is None:
            return -1.0
        distance, hit_normal = hit
        normal[:] = hit_normal
        return distance

    def mj_rayMesh(
        self,
        _model,
        data,
        geom_id,
        origin,
        direction,
        normal,
    ) -> float:
        return self.mju_rayGeom(
            data.geom_xpos[geom_id],
            data.geom_xmat[geom_id],
            (),
            origin,
            direction,
            self.mjtGeom.mjGEOM_MESH,
            normal,
        )

    def mj_rayHfield(
        self,
        _model,
        data,
        geom_id,
        origin,
        direction,
        normal,
    ) -> float:
        return self.mju_rayGeom(
            data.geom_xpos[geom_id],
            data.geom_xmat[geom_id],
            (),
            origin,
            direction,
            self.mjtGeom.mjGEOM_HFIELD,
            normal,
        )


class SpawnClearanceAuditTest(unittest.TestCase):
    def setUp(self) -> None:
        self.model = FakeModel()

    def audit(self, *contacts: Contact) -> dict[str, object]:
        return MODULE.audit_spawn_clearance(self.model, FakeData(*contacts))

    def test_allows_bounded_vertical_contacts_on_both_soles(self) -> None:
        result = self.audit(
            Contact(0, 3, dist=-0.010, frame=(0.0, 0.0, -1.0)),
            Contact(3, 1, dist=-0.015, frame=VERTICAL),
        )

        self.assertTrue(result["safe"])
        self.assertEqual(result["reason"], "clear")
        self.assertEqual(result["external_contact_count"], 2)
        self.assertEqual(result["allowed_contact_count"], 2)
        self.assertEqual(result["rejected_contact_count"], 0)
        self.assertEqual(
            [item["classification"] for item in result["contacts"]],
            ["allowed_foot_support", "allowed_foot_support"],
        )
        self.assertEqual(result["worst"]["distance_m"], -0.015)
        self.assertEqual(result["worst"]["robot_body"]["name"], "right_ankle_roll_link")
        json.dumps(result, allow_nan=False)

    def test_body_penetration_threshold_is_inclusive_at_two_millimetres(self) -> None:
        touching = self.audit(Contact(2, 3, dist=-0.002))
        penetrating = self.audit(Contact(2, 3, dist=-0.002001))

        self.assertTrue(touching["safe"])
        self.assertEqual(
            touching["contacts"][0]["classification"],
            "allowed_body_contact_tolerance",
        )
        self.assertFalse(penetrating["safe"])
        self.assertEqual(penetrating["reason"], "scene_penetration")
        self.assertEqual(
            penetrating["worst"]["classification"],
            "scene_penetration",
        )

    def test_rejects_foot_penetration_beyond_fifteen_millimetres(self) -> None:
        result = self.audit(
            Contact(0, 3, dist=-0.015001, frame=(0.0, 0.0, -1.0))
        )

        self.assertFalse(result["safe"])
        self.assertEqual(result["reason"], "unsafe_foot_contact")
        self.assertEqual(
            result["worst"]["classification"],
            "unsafe_foot_penetration",
        )

    def test_rejects_foot_wall_contact_even_without_penetration(self) -> None:
        for contact in (
            Contact(0, 4, dist=0.0, frame=HORIZONTAL),
            Contact(4, 0, dist=0.0, frame=HORIZONTAL),
        ):
            with self.subTest(geom1=contact.geom1):
                result = self.audit(contact)
                self.assertFalse(result["safe"])
                self.assertEqual(result["reason"], "unsafe_foot_contact")
                self.assertEqual(
                    result["worst"]["classification"],
                    "unsafe_foot_contact_normal",
                )
                self.assertEqual(
                    result["worst"]["robot_body"]["name"],
                    "left_ankle_roll_link",
                )

    def test_allows_shallow_moon_mocap_tile_edge_contact(self) -> None:
        self.model.nmocap = 1
        self.model.body_mocapid = (-1, -1, -1, -1, -1, 0)
        self.model._bodies[5] = "gb_0_0"

        result = self.audit(Contact(0, 4, dist=-0.004, frame=HORIZONTAL))

        self.assertTrue(result["safe"])
        self.assertEqual(
            result["contacts"][0]["classification"],
            "allowed_foot_terrain_edge",
        )

    def test_rejects_same_named_non_mocap_tile_edge_contact(self) -> None:
        self.model._bodies[5] = "gb_0_0"

        result = self.audit(Contact(0, 4, dist=-0.004, frame=HORIZONTAL))

        self.assertFalse(result["safe"])
        self.assertEqual(
            result["worst"]["classification"],
            "unsafe_foot_contact_normal",
        )

    def test_rejects_deep_moon_mocap_tile_edge_penetration(self) -> None:
        self.model.nmocap = 1
        self.model.body_mocapid = (-1, -1, -1, -1, -1, 0)
        self.model._bodies[5] = "gb_0_0"

        result = self.audit(
            Contact(0, 4, dist=-0.015001, frame=HORIZONTAL)
        )

        self.assertFalse(result["safe"])
        self.assertEqual(
            result["worst"]["classification"],
            "unsafe_foot_contact_normal",
        )

    def test_rejects_downward_scene_to_robot_normal_as_ceiling_contact(self) -> None:
        contacts = (
            Contact(0, 3, dist=0.0, frame=VERTICAL),
            Contact(3, 0, dist=0.0, frame=(0.0, 0.0, -1.0)),
        )

        for contact in contacts:
            with self.subTest(geom1=contact.geom1):
                result = self.audit(contact)
                self.assertFalse(result["safe"])
                self.assertEqual(result["reason"], "unsafe_foot_contact")
                self.assertEqual(
                    result["worst"]["classification"],
                    "unsafe_foot_contact_normal",
                )
                self.assertEqual(
                    result["worst"]["scene_to_robot_normal"],
                    [0.0, 0.0, -1.0],
                )

    def test_geom_order_does_not_change_robot_scene_classification(self) -> None:
        normal_order = self.audit(Contact(2, 4, dist=-0.03))
        flipped_order = self.audit(Contact(4, 2, dist=-0.03))

        for result in (normal_order, flipped_order):
            self.assertFalse(result["safe"])
            self.assertEqual(result["worst"]["robot_body"]["id"], 4)
            self.assertEqual(result["worst"]["robot_body"]["name"], "torso_link")
            self.assertEqual(result["worst"]["scene_body"]["id"], 5)
        self.assertTrue(normal_order["worst"]["geom1"]["robot"])
        self.assertTrue(flipped_order["worst"]["geom2"]["robot"])

    def test_rejects_deep_hand_and_finger_descendant_contact(self) -> None:
        model = FakeModel()
        model.nbody = 9
        model.ngeom = 8
        model.body_parentid = (0, 0, 1, 1, 1, 0, 1, 6, 7)
        model.geom_bodyid = (*model.geom_bodyid, 8)
        model._bodies.update(
            {
                6: "left_wrist_link",
                7: "left_hand_link",
                8: "left_index_distal_link",
            }
        )
        model._geoms[7] = "left_index_distal_collision"

        result = MODULE.audit_spawn_clearance(
            model,
            FakeData(Contact(7, 4, dist=-0.03, frame=HORIZONTAL)),
        )

        self.assertFalse(result["safe"])
        self.assertEqual(result["reason"], "scene_penetration")
        self.assertEqual(
            result["worst"]["robot_body"]["name"],
            "left_index_distal_link",
        )
        self.assertEqual(
            result["worst"]["geom1"]["name"],
            "left_index_distal_collision",
        )

    def test_ignores_robot_self_and_scene_scene_contacts(self) -> None:
        result = self.audit(
            Contact(0, 2, dist=-0.5, frame=HORIZONTAL),
            Contact(3, 4, dist=-0.5, frame=HORIZONTAL),
        )

        self.assertTrue(result["safe"])
        self.assertEqual(result["contacts_checked"], 2)
        self.assertEqual(result["external_contact_count"], 0)
        self.assertEqual(result["ignored_self_contact_count"], 1)
        self.assertEqual(result["ignored_scene_contact_count"], 1)
        self.assertIsNone(result["worst"])

    def test_nonfinite_contact_fields_fail_closed(self) -> None:
        contacts = (
            Contact(2, 3, dist=float("nan")),
            Contact(2, 3, frame=(0.0, float("inf"), 1.0)),
            Contact(2, 3, pos=(0.0, 0.0, float("-inf"))),
        )
        for contact in contacts:
            with self.subTest(contact=contact):
                result = self.audit(contact)
                self.assertFalse(result["safe"])
                self.assertEqual(result["reason"], "audit_error")
                self.assertIsNotNone(result["error"])
                json.dumps(result, allow_nan=False)

    def test_missing_pelvis_or_foot_fails_closed(self) -> None:
        for missing_name in (
            "pelvis",
            "left_ankle_roll_link",
            "right_ankle_roll_link",
        ):
            with self.subTest(missing_name=missing_name):
                model = FakeModel()
                missing_id = next(
                    item_id
                    for item_id, name in model._bodies.items()
                    if name == missing_name
                )
                model._bodies[missing_id] = f"missing_{missing_id}"
                result = MODULE.audit_spawn_clearance(model, FakeData())
                self.assertFalse(result["safe"])
                self.assertEqual(result["reason"], "audit_error")

    def test_foot_outside_pelvis_subtree_fails_closed(self) -> None:
        model = FakeModel()
        model.body_parentid = (0, 0, 0, 1, 1, 0)

        result = MODULE.audit_spawn_clearance(model, FakeData())

        self.assertFalse(result["safe"])
        self.assertEqual(result["reason"], "audit_error")
        self.assertIn("descendant", result["error"]["message"])

    def test_bad_geom_body_and_contact_table_indices_fail_closed(self) -> None:
        bad_geom = self.audit(Contact(99, 3))
        self.assertFalse(bad_geom["safe"])
        self.assertEqual(bad_geom["reason"], "audit_error")

        model = FakeModel()
        model.geom_bodyid = (2, 3, 99, 0, 5, 1, 5)
        bad_body = MODULE.audit_spawn_clearance(model, FakeData(Contact(2, 3)))
        self.assertFalse(bad_body["safe"])
        self.assertEqual(bad_body["reason"], "audit_error")

        truncated = FakeData()
        truncated.ncon = 1
        bad_table = MODULE.audit_spawn_clearance(self.model, truncated)
        self.assertFalse(bad_table["safe"])
        self.assertEqual(bad_table["reason"], "audit_error")

    def test_invalid_thresholds_fail_closed(self) -> None:
        result = MODULE.audit_spawn_clearance(
            self.model,
            FakeData(),
            body_penetration_tolerance_m=-1.0,
        )

        self.assertFalse(result["safe"])
        self.assertEqual(result["reason"], "audit_error")


class ApplyRootPoseTest(unittest.TestCase):
    def test_uses_fresh_data_resets_defaults_and_only_overwrites_root_qpos(self) -> None:
        model = FakeModel()
        live_data = FakeData(qpos=[42.0] * model.nq)
        engine = FakeMujoco(
            contacts=(Contact(0, 3, dist=-0.005, frame=(0.0, 0.0, -1.0)),)
        )

        result = MODULE.apply_root_pose_and_audit(
            engine,
            model,
            {"position": [1.0, 2.0, 0.94], "yaw_rad": math.pi / 2.0},
        )

        self.assertTrue(result["safe"])
        self.assertEqual(result["root_qpos_address"], 0)
        self.assertEqual(engine.reset_calls, 1)
        self.assertEqual(engine.forward_calls, 1)
        self.assertEqual(len(engine.created), 1)
        isolated = engine.created[0]
        self.assertEqual(isolated.qpos[:3], [1.0, 2.0, 0.94])
        expected_quaternion = [math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)]
        for actual, expected in zip(isolated.qpos[3:7], expected_quaternion):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(isolated.qpos[7:], model.qpos0[7:])
        self.assertEqual(live_data.qpos, [42.0] * model.nq)
        self.assertEqual(model.qpos0[7:], [7.0, 8.0, 9.0])
        self.assertEqual(
            result["evaluated_pose"],
            {"position": [1.0, 2.0, 0.94], "yaw_rad": math.pi / 2.0},
        )

    def test_accepts_world_pose_shaped_object(self) -> None:
        result = MODULE.apply_root_pose_and_audit(
            FakeMujoco(),
            FakeModel(),
            SimpleNamespace(x=4.0, y=5.0, z=0.9, yaw_rad=-0.5),
        )

        self.assertTrue(result["safe"])
        self.assertEqual(result["evaluated_pose"]["position"], [4.0, 5.0, 0.9])

    def test_missing_free_joint_and_nonfinite_pose_fail_closed(self) -> None:
        no_joint = FakeModel()
        no_joint.body_jntnum = (0, 0, 0, 0, 0, 0)
        missing = MODULE.apply_root_pose_and_audit(
            FakeMujoco(),
            no_joint,
            {"position": [1.0, 2.0, 3.0], "yaw_rad": 0.0},
        )
        nonfinite = MODULE.apply_root_pose_and_audit(
            FakeMujoco(),
            FakeModel(),
            {"position": [1.0, float("nan"), 3.0], "yaw_rad": 0.0},
        )

        for result in (missing, nonfinite):
            self.assertFalse(result["safe"])
            self.assertEqual(result["reason"], "audit_error")
            json.dumps(result, allow_nan=False)


class GroundSupportProbeTest(unittest.TestCase):
    def setUp(self) -> None:
        self.model = FakeModel()
        self.data = FakeData()

    def test_accepts_collision_compatible_support_under_both_feet(self) -> None:
        engine = FakeMujoco()

        support = MODULE.probe_ground_support(engine, self.model, self.data)

        self.assertTrue(support["supported"])
        self.assertEqual(support["accepted_hits"], 2)
        self.assertEqual(support["required_hits"], 1)
        self.assertEqual(
            [probe["scene_geom"]["name"] for probe in support["probes"]],
            ["floor", "floor"],
        )
        self.assertEqual(
            [probe["distance_m"] for probe in support["probes"]],
            [0.04, 0.04],
        )
        json.dumps(support, allow_nan=False)

    def test_one_foot_hit_is_sufficient(self) -> None:
        engine = FakeMujoco(
            ray_hits={(0.0, 3): (0.04, VERTICAL)},
            default_support=False,
        )

        support = MODULE.probe_ground_support(engine, self.model, self.data)

        self.assertTrue(support["supported"])
        self.assertEqual(support["accepted_hits"], 1)
        self.assertEqual(
            [probe["accepted"] for probe in support["probes"]],
            [True, False],
        )

    def test_rejects_when_both_foot_rays_miss(self) -> None:
        support = MODULE.probe_ground_support(
            FakeMujoco(default_support=False),
            self.model,
            self.data,
        )

        self.assertFalse(support["supported"])
        self.assertEqual(support["accepted_hits"], 0)
        for probe in support["probes"]:
            self.assertFalse(probe["accepted"])
            self.assertIsNone(probe["distance_m"])
            self.assertIsNone(probe["normal"])
            self.assertTrue(probe["origins"])
            self.assertIsNone(probe["probe_geom"])
            self.assertIsNone(probe["ray_origin_m"])
            self.assertIsNone(probe["scene_geom"])

    def test_uses_collision_geoms_as_multipoint_origins(self) -> None:
        self.model.geom_bodyid = (2, 3, 4, 0, 5, 1, 2)
        self.model.geom_contype = (0, 1, 1, 1, 1, 1, 1)
        self.model.geom_conaffinity = (0, 1, 1, 1, 1, 1, 1)
        engine = FakeMujoco(
            ray_hits={
                (6.0, 3): (0.04, VERTICAL),
                (1.0, 3): (0.04, VERTICAL),
            },
            default_support=False,
        )

        support = MODULE.probe_ground_support(engine, self.model, self.data)

        self.assertTrue(support["supported"])
        self.assertEqual(support["accepted_hits"], 2)
        self.assertEqual(
            [origin["geom_id"] for origin in support["probes"][0]["origins"]],
            [6],
        )
        self.assertFalse(any(origin_x == 0.0 for origin_x, _ in engine.ray_calls))

    def test_allowed_persistent_foot_contact_skips_rays(self) -> None:
        data = FakeData(Contact(3, 0, dist=-0.001))
        engine = FakeMujoco(default_support=False)

        result = MODULE.audit_spawn_safety(engine, self.model, data)

        self.assertTrue(result["safe"])
        self.assertEqual(result["support"]["method"], "allowed_foot_contacts")
        self.assertEqual(result["support"]["accepted_hits"], 1)
        self.assertEqual(engine.ray_calls, [])

    def test_rejects_too_distant_or_steep_hits(self) -> None:
        engine = FakeMujoco(
            ray_hits={
                (0.0, 3): (0.120001, VERTICAL),
                (1.0, 3): (0.04, HORIZONTAL),
            },
            default_support=False,
        )

        support = MODULE.probe_ground_support(engine, self.model, self.data)

        self.assertFalse(support["supported"])
        self.assertEqual(support["accepted_hits"], 0)

    def test_skips_non_collidable_scene_geoms(self) -> None:
        self.model.geom_contype = (1, 1, 1, 0, 1, 1, 1)
        self.model.geom_conaffinity = (1, 1, 1, 0, 1, 1, 1)
        engine = FakeMujoco(
            ray_hits={(0.0, 3): (0.04, VERTICAL)},
            default_support=False,
        )

        support = MODULE.probe_ground_support(engine, self.model, self.data)

        self.assertFalse(support["supported"])
        self.assertNotIn((0.0, 3), engine.ray_calls)

    def test_combined_audit_reports_typed_no_ground_evidence(self) -> None:
        result = MODULE.audit_spawn_safety(
            FakeMujoco(default_support=False),
            self.model,
            self.data,
        )

        self.assertFalse(result["safe"])
        self.assertEqual(result["reason"], "no_ground_support")
        self.assertEqual(result["rejected_contact_count"], 0)
        self.assertEqual(result["support"]["schema"], MODULE.GROUND_SUPPORT_SCHEMA)
        self.assertFalse(result["support"]["supported"])

    def test_apply_root_pose_rejects_unsupported_candidate(self) -> None:
        result = MODULE.apply_root_pose_and_audit(
            FakeMujoco(default_support=False),
            self.model,
            {"position": [10.0, 0.0, 0.79], "yaw_rad": 0.0},
        )

        self.assertFalse(result["safe"])
        self.assertEqual(result["reason"], "no_ground_support")
        self.assertEqual(result["evaluated_pose"]["position"], [10.0, 0.0, 0.79])
        json.dumps(result, allow_nan=False)

    def test_apply_root_pose_prepares_dynamic_data_before_forward(self) -> None:
        engine = FakeMujoco()
        events: list[str] = []
        original_forward = engine.mj_forward

        def prepare(data: FakeData) -> None:
            self.assertEqual(data.qpos[:3], [2.0, 3.0, 0.79])
            events.append("prepare")

        def forward(model: FakeModel, data: FakeData) -> None:
            self.assertEqual(events, ["prepare"])
            events.append("forward")
            original_forward(model, data)

        engine.mj_forward = forward
        result = MODULE.apply_root_pose_and_audit(
            engine,
            self.model,
            {"position": [2.0, 3.0, 0.79], "yaw_rad": 0.0},
            data_preparer=prepare,
        )

        self.assertTrue(result["safe"])
        self.assertEqual(events, ["prepare", "forward"])


class OptionalTown10SmokeTest(unittest.TestCase):
    def test_town10_model_when_explicitly_available(self) -> None:
        model_path = os.environ.get("MATRIX_TOWN10_MODEL")
        if not model_path:
            self.skipTest("MATRIX_TOWN10_MODEL is not configured")
        path = Path(model_path)
        if not path.is_file():
            self.skipTest("configured Town10 model is unavailable")
        try:
            import mujoco
        except ImportError:
            self.skipTest("MuJoCo Python module is unavailable")
        model = mujoco.MjModel.from_xml_path(str(path))
        w, x, y, z = (float(value) for value in model.qpos0[3:7])
        yaw = math.atan2(
            2.0 * ((w * z) + (x * y)),
            1.0 - 2.0 * ((y * y) + (z * z)),
        )

        result = MODULE.apply_root_pose_and_audit(
            mujoco,
            model,
            {"position": list(model.qpos0[:3]), "yaw_rad": yaw},
        )

        self.assertEqual(result["schema"], MODULE.AUDIT_SCHEMA)
        self.assertIs(type(result["safe"]), bool)
        self.assertNotEqual(result["reason"], "audit_error")
        json.dumps(result, allow_nan=False)


if __name__ == "__main__":
    unittest.main()
