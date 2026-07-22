from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest import mock
import xml.etree.ElementTree as ET


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "prepare_sonic_physics_model.py"
SPEC = importlib.util.spec_from_file_location("prepare_sonic_physics_model", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class PrepareSonicPhysicsModelTest(unittest.TestCase):
    def test_keeps_body_actuators_and_fixes_finger_joints(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_dir:
            root = Path(temporary_dir)
            canonical = root / "canonical.xml"
            meshes = root / "canonical_meshes"
            native = root / "xgb"
            output = root / "output"
            meshes.mkdir()
            (meshes / "body.stl").write_bytes(b"body")
            (native / "assets").mkdir(parents=True)
            (native / "height.png").write_bytes(b"height")
            canonical.write_text(
                """<mujoco><compiler meshdir="meshes" />
<default><default class="visual"><geom material="body_material" /></default></default>
<asset><mesh name="body" file="body.stl" />
<texture name="body_texture" builtin="flat" rgb1="0.7 0.2 0.1" width="1" height="1" />
<material name="body_material" texture="body_texture" />
<texture name="demo_ground" builtin="checker" />
<material name="demo_ground_material" texture="demo_ground" /></asset>
<worldbody><body name="pelvis"><freejoint name="floating" />
<geom class="visual" mesh="body" />
<joint name="joint_a" /><joint name="joint_b" /><body name="finger">
<joint name="finger_joint" /></body></body>
<light name="demo_light" /><geom name="demo_floor" type="plane"
material="demo_ground_material" /></worldbody>
<actuator><motor name="a" joint="joint_a" /><motor name="b" joint="joint_b" />
<motor name="finger" joint="finger_joint" /></actuator>
<sensor><jointpos name="a_pos" joint="joint_a" />
<jointpos name="finger_pos" joint="finger_joint" /></sensor>
<statistic center="1 2 3" /><visual><global azimuth="10" /></visual>
</mujoco>""",
                encoding="utf-8",
            )
            scene = native / "scene.xml"
            scene.write_text(
                """<mujoco><include file="xgb.xml" /><asset>
<hfield name="height" file="../height.png" /></asset>
<worldbody><geom name="floor" type="plane" /></worldbody></mujoco>""",
                encoding="utf-8",
            )

            output_scene = MODULE.prepare_sonic_physics_model(
                canonical,
                meshes,
                scene,
                output,
                body_joint_names=("joint_a", "joint_b"),
            )

            robot = ET.parse(output / "robot.xml").getroot()
            self.assertEqual(
                [item.get("joint") for item in robot.find("actuator")],
                ["joint_a", "joint_b"],
            )
            self.assertEqual(
                [item.get("name") for item in robot.iter("joint")],
                ["joint_a", "joint_b"],
            )
            self.assertEqual(robot.find("worldbody/body/freejoint").get("name"), "floating")
            self.assertEqual(len(robot.findall("worldbody")), 1)
            self.assertEqual(
                [(item.tag, item.get("name")) for item in robot.find("worldbody")],
                [("body", "pelvis")],
            )
            self.assertIsNone(robot.find("statistic"))
            self.assertIsNone(robot.find("visual"))
            self.assertIsNone(
                next(
                    (
                        item
                        for asset in robot.findall("asset")
                        for item in asset
                        if item.get("name") == "demo_ground"
                    ),
                    None,
                )
            )
            retained_assets = {
                item.get("name")
                for asset in robot.findall("asset")
                for item in asset
            }
            self.assertIn("body_material", retained_assets)
            self.assertIn("body_texture", retained_assets)
            self.assertNotIn("demo_ground_material", retained_assets)
            self.assertEqual(
                [item.get("joint") for item in robot.find("sensor")],
                ["joint_a"],
            )
            self.assertEqual(ET.parse(output_scene).getroot().find("include").get("file"), "robot.xml")
            self.assertTrue((output / "meshes" / "body.stl").is_file())
            self.assertTrue((output / "height.png").is_file())

            MODULE.prepare_sonic_physics_model(
                canonical,
                meshes,
                scene,
                output,
                body_joint_names=("joint_a", "joint_b"),
                spawn_xyz=(124.0, -105.05, 0.793),
                spawn_yaw=math.pi / 2.0,
            )
            root_body = ET.parse(output / "robot.xml").getroot().find(
                "worldbody/body"
            )
            self.assertEqual(root_body.get("pos"), "124 -105.05 0.793")
            quaternion = [float(value) for value in root_body.get("quat").split()]
            self.assertAlmostEqual(quaternion[0], math.sqrt(0.5))
            self.assertEqual(quaternion[1:3], [0.0, 0.0])
            self.assertAlmostEqual(quaternion[3], math.sqrt(0.5))
            manifest = json.loads((output / "manifest.json").read_text())
            self.assertEqual(manifest["spawn_xyz"], [124.0, -105.05, 0.793])
            self.assertAlmostEqual(manifest["spawn_yaw_rad"], math.pi / 2.0)
            self.assertEqual(
                manifest["derived_robot_sha256"],
                MODULE._file_sha256(output / "robot.xml"),
            )
            self.assertEqual(
                manifest["derived_scene_sha256"],
                MODULE._file_sha256(output / scene.name),
            )
            self.assertEqual(
                manifest["derived_meshes_sha256"],
                MODULE._tree_sha256(output / "meshes"),
            )
            self.assertEqual(
                manifest["derived_bundle_sha256"],
                MODULE._bundle_sha256(output),
            )
            self.assertEqual(
                manifest["native_scene_assets"],
                [
                    {
                        "path": str((native / "height.png").resolve()),
                        "relative_path": "height.png",
                        "size": len(b"height"),
                        "sha256": MODULE._file_sha256(native / "height.png"),
                    }
                ],
            )

            (output / "height.png").write_bytes(b"tampered")
            MODULE.prepare_sonic_physics_model(
                canonical,
                meshes,
                scene,
                output,
                body_joint_names=("joint_a", "joint_b"),
                spawn_xyz=(124.0, -105.05, 0.793),
                spawn_yaw=math.pi / 2.0,
            )
            self.assertEqual((output / "height.png").read_bytes(), b"height")

    def test_injects_creative_inventory_into_canonical_sonic_robot(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_dir:
            root = Path(temporary_dir)
            canonical = root / "canonical.xml"
            meshes = root / "canonical_meshes"
            native = root / "xgb"
            output = root / "output"
            inventory = root / "inventory"
            catalog = inventory / "catalog.json"
            meshes.mkdir()
            inventory.mkdir()
            (meshes / "body.stl").write_bytes(b"body")
            (inventory / "prop.stl").write_bytes(b"prop")
            (native / "assets").mkdir(parents=True)
            canonical.write_text(
                """<mujoco><compiler meshdir="meshes" /><asset>
<mesh name="body" file="body.stl" /></asset>
<worldbody><body name="pelvis"><freejoint name="floating" />
<joint name="joint_a" /><geom mesh="body" /></body>
<body name="demo_body"><geom type="box" size="1 1 1" /></body>
<light name="demo_light" /></worldbody>
<actuator><motor name="a" joint="joint_a" /></actuator></mujoco>""",
                encoding="utf-8",
            )
            catalog.write_text(
                json.dumps(
                    {
                        "schema": "matrix-creative-inventory/v1",
                        "items": [
                            {
                                "item_id": "prop",
                                "label": "Prop",
                                "pool_size": 1,
                                "mass_kg": 1.0,
                                "collision_half_size": [0.1, 0.1, 0.1],
                                "spawn_distance_m": 0.9,
                                "spawn_height_m": 1.0,
                                "spawn_quat": [1.0, 0.0, 0.0, 0.0],
                                "visuals": [
                                    {
                                        "mesh": "prop.stl",
                                        "rgba": [0.2, 0.4, 0.8, 1.0],
                                        "scale": [1.0, 1.0, 1.0],
                                    }
                                ],
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )
            scene = native / "scene.xml"
            scene.write_text(
                """<mujoco><include file="xgb.xml" />
<worldbody><geom name="floor" type="plane" /></worldbody></mujoco>""",
                encoding="utf-8",
            )

            MODULE.prepare_sonic_physics_model(
                canonical,
                meshes,
                scene,
                output,
                body_joint_names=("joint_a",),
                creative_inventory_catalog=catalog,
            )

            robot = ET.parse(output / "robot.xml").getroot()
            worldbody_names = [
                child.get("name") for child in robot.find("worldbody")
            ]
            self.assertEqual(
                worldbody_names,
                ["pelvis", "creative_item__prop__0"],
            )
            self.assertIsNotNone(
                robot.find(
                    ".//weld[@name='creative_item__prop__0__storage_weld']"
                )
            )
            retained_assets = {
                item.get("name") for item in robot.find("asset")
            }
            self.assertIn("creative_prop_0", retained_assets)
            self.assertIn("matrix_source_creative_prop_0", retained_assets)
            self.assertTrue((output / "meshes" / "creative_prop_0.stl").is_file())
            inventory_geoms = robot.findall(
                ".//body[@name='creative_item__prop__0']/geom"
            )
            self.assertTrue(inventory_geoms)
            self.assertTrue(
                all("class" not in geom.attrib for geom in inventory_geoms)
            )
            manifest = json.loads((output / "manifest.json").read_text())
            self.assertEqual(
                manifest["creative_inventory"]["catalog_sha256"],
                MODULE._file_sha256(catalog),
            )

    def test_town10_open_boundary_removes_four_walls_and_retains_floor(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_dir:
            root = Path(temporary_dir)
            canonical = root / "canonical.xml"
            meshes = root / "canonical_meshes"
            native = root / "xgb"
            output = root / "output"
            meshes.mkdir()
            (native / "assets").mkdir(parents=True)
            canonical.write_text(
                """<mujoco><worldbody><body name="pelvis">
<freejoint name="floating" /><joint name="joint_a" />
</body></worldbody><actuator><motor name="a" joint="joint_a" />
</actuator></mujoco>""",
                encoding="utf-8",
            )
            scene = native / "scene_terrain_t10.xml"
            scene.write_text(
                """<mujoco><include file="xgb.xml" /><worldbody>
<geom name="floor" size="0 0 0.01" type="plane" />
<geom name="ps_Cube" type="box" size="125.0 0.05 1.5" pos="0.9 72.6 1.5" quat="1 0 0 0" />
<geom name="ps_Cube2" type="box" size="125.0 0.05 1.5" pos="0.9 -125.7 1.5" quat="1 0 0 0" />
<geom name="ps_Cube3" type="box" size="125.0 0.05 1.5" pos="104.4 -21.6 1.5" quat="0.707107 0 0 -0.707107" />
<geom name="ps_Cube4" type="box" size="125.0 0.05 1.5" pos="-109.0 -21.6 1.5" quat="0.707107 0 0 -0.707107" />
<geom name="building" type="box" size="1 1 1" />
</worldbody></mujoco>""",
                encoding="utf-8",
            )
            source_sha256 = MODULE._file_sha256(scene)
            with mock.patch.object(
                MODULE, "TOWN10_SOURCE_SCENE_SHA256", source_sha256
            ):
                output_scene = MODULE.prepare_sonic_physics_model(
                    canonical,
                    meshes,
                    scene,
                    output,
                    body_joint_names=("joint_a",),
                    scene_transform=MODULE.TOWN10_OPEN_BOUNDARY_TRANSFORM,
                )

            names = [
                geom.get("name")
                for geom in ET.parse(output_scene).getroot().iter("geom")
            ]
            self.assertIn("floor", names)
            self.assertIn("building", names)
            for wall in MODULE.TOWN10_PERIMETER_WALL_NAMES:
                self.assertNotIn(wall, names)
            manifest = json.loads((output / "manifest.json").read_text())
            self.assertEqual(manifest["pipeline_version"], 7)
            self.assertEqual(
                manifest["scene_transform"],
                MODULE.TOWN10_OPEN_BOUNDARY_TRANSFORM,
            )
            self.assertEqual(
                manifest["removed_environment_geoms"],
                list(MODULE.TOWN10_PERIMETER_WALL_NAMES),
            )

            drifted = scene.read_text(encoding="utf-8").replace(
                'name="ps_Cube4" type="box" size="125.0 0.05 1.5"',
                'name="ps_Cube4" type="box" size="124.0 0.05 1.5"',
            )
            scene.write_text(drifted, encoding="utf-8")
            with (
                mock.patch.object(
                    MODULE,
                    "TOWN10_SOURCE_SCENE_SHA256",
                    MODULE._file_sha256(scene),
                ),
                self.assertRaisesRegex(
                    MODULE.SonicPhysicsModelError, "collision contract drifted"
                ),
            ):
                MODULE.prepare_sonic_physics_model(
                    canonical,
                    meshes,
                    scene,
                    root / "drifted-output",
                    body_joint_names=("joint_a",),
                    scene_transform=MODULE.TOWN10_OPEN_BOUNDARY_TRANSFORM,
                )

    def test_moon_dynamic_ground_transform_staticizes_freejoints(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_dir:
            root = Path(temporary_dir)
            canonical = root / "canonical.xml"
            meshes = root / "canonical_meshes"
            native = root / "xgb"
            output = root / "output"
            meshes.mkdir()
            (native / "assets").mkdir(parents=True)
            canonical.write_text(
                """<mujoco><worldbody><body name="pelvis">
<freejoint name="floating" /><joint name="joint_a" />
</body></worldbody><actuator><motor name="a" joint="joint_a" />
</actuator></mujoco>""",
                encoding="utf-8",
            )
            scene = native / "scene_terrain_moon_dynamic.xml"
            scene.write_text(
                """<mujoco><include file="xgb.xml" /><worldbody>
<body name="gb_0_0" pos="-0.75 -0.75 0" gravcomp="1">
  <joint type="free" name="gb_joint_0_0" />
  <geom name="soil_0_0" type="box" size="0.049 0.049 0.5" pos="0 0 -0.5" mass="100000000" />
</body>
<body name="gb_0_1" pos="-0.75 -0.65 0" gravcomp="1">
  <joint type="free" name="gb_joint_0_1" />
  <geom name="soil_0_1" type="box" size="0.049 0.049 0.5" pos="0 0 -0.5" mass="100000000" />
</body>
</worldbody></mujoco>""",
                encoding="utf-8",
            )

            with (
                mock.patch.object(
                    MODULE,
                    "MOON_DYNAMIC_GROUND_SOURCE_SCENE_SHA256",
                    MODULE._file_sha256(scene),
                ),
                mock.patch.object(
                    MODULE,
                    "MOON_DYNAMIC_GROUND_FREEJOINT_BODY_COUNT",
                    2,
                ),
            ):
                output_scene = MODULE.prepare_sonic_physics_model(
                    canonical,
                    meshes,
                    scene,
                    output,
                    body_joint_names=("joint_a",),
                    scene_transform=MODULE.MOON_DYNAMIC_GROUND_STATIC_TRANSFORM,
                )

            scene_root = ET.parse(output_scene).getroot()
            self.assertEqual(
                [(body.get("name"), body.get("pos")) for body in scene_root.iter("body")],
                [("gb_0_0", "-0.75 -0.75 0"), ("gb_0_1", "-0.75 -0.65 0")],
            )
            self.assertEqual([joint.get("name") for joint in scene_root.iter("joint")], [])
            self.assertEqual(
                [geom.get("name") for geom in scene_root.iter("geom")],
                ["soil_0_0", "soil_0_1"],
            )
            manifest = json.loads((output / "manifest.json").read_text())
            self.assertEqual(manifest["pipeline_version"], 7)
            self.assertEqual(
                manifest["scene_transform"],
                MODULE.MOON_DYNAMIC_GROUND_STATIC_TRANSFORM,
            )
            self.assertEqual(manifest["removed_environment_geoms"], [])
            self.assertEqual(
                manifest["staticized_freejoint_bodies"],
                ["gb_0_0", "gb_0_1"],
            )


if __name__ == "__main__":
    unittest.main()
