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
<asset><mesh name="body" file="body.stl" /></asset>
<worldbody><body name="pelvis"><freejoint name="floating" />
<joint name="joint_a" /><joint name="joint_b" /><body name="finger">
<joint name="finger_joint" /></body></body></worldbody>
<actuator><motor name="a" joint="joint_a" /><motor name="b" joint="joint_b" />
<motor name="finger" joint="finger_joint" /></actuator>
<sensor><jointpos name="a_pos" joint="joint_a" />
<jointpos name="finger_pos" joint="finger_joint" /></sensor></mujoco>""",
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
            self.assertEqual(manifest["pipeline_version"], 5)
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

    def test_moon_dynamic_ground_transform_converts_tiles_to_mocap(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_dir:
            root = Path(temporary_dir)
            canonical = root / "canonical.xml"
            meshes = root / "canonical_meshes"
            native = root / "xgb"
            output = root / "output"
            meshes.mkdir()
            native.mkdir()
            canonical.write_text(
                """<mujoco><worldbody><body name="pelvis">
<freejoint name="floating" /><joint name="joint_a" />
</body></worldbody><actuator><motor name="a" joint="joint_a" />
</actuator></mujoco>""",
                encoding="utf-8",
            )
            scene = native / MODULE.MOON_DYNAMIC_GROUND_SCENE_NAME
            bodies = []
            for i in range(16):
                for j in range(16):
                    bodies.append(
                        f"""<body name="gb_{i}_{j}" pos="{i * 0.1:.1f} {j * 0.1:.1f} 0" gravcomp="1">
  <joint type="free" name="gb_joint_{i}_{j}" />
  <geom name="soil_{i}_{j}" type="box" size="0.049 0.049 0.5" pos="0 0 -0.5" mass="100000000" />
</body>"""
                    )
            scene.write_text(
                "<mujoco><include file=\"xgb.xml\" /><worldbody>\n"
                + "\n".join(bodies)
                + "\n</worldbody></mujoco>",
                encoding="utf-8",
            )
            with mock.patch.object(
                MODULE,
                "MOON_DYNAMIC_GROUND_SOURCE_SCENE_SHA256",
                MODULE._file_sha256(scene),
            ):
                output_scene = MODULE.prepare_sonic_physics_model(
                    canonical,
                    meshes,
                    scene,
                    output,
                    body_joint_names=("joint_a",),
                    scene_transform=MODULE.MOON_DYNAMIC_GROUND_MOCAP_TRANSFORM,
                    moon_dynamic_ground_collision_mode=(
                        MODULE.MOON_DYNAMIC_GROUND_COLLISION_TILES
                    ),
                )

            manifest = json.loads((output / "manifest.json").read_text())
            self.assertEqual(
                manifest["scene_transform"],
                MODULE.MOON_DYNAMIC_GROUND_MOCAP_TRANSFORM,
            )
            self.assertEqual(
                manifest["staticized_freejoint_bodies"][0],
                "gb_0_0",
            )
            contract = manifest["scene_transform_contract"]["dynamic_ground"]
            self.assertEqual(contract["body_count"], 256)
            self.assertEqual(
                contract["collision"]["mode"],
                MODULE.MOON_DYNAMIC_GROUND_COLLISION_TILES,
            )

            xml = ET.parse(output_scene).getroot()
            tile_bodies = [
                body
                for body in xml.iter("body")
                if (body.get("name") or "").startswith("gb_")
            ]
            self.assertEqual(len(tile_bodies), 256)
            self.assertTrue(all(body.get("mocap") == "true" for body in tile_bodies))
            self.assertFalse(list(xml.iter("joint")))
            soil = next(geom for geom in xml.iter("geom") if geom.get("name") == "soil_7_8")
            self.assertEqual(soil.get("contype"), "1")
            self.assertEqual(soil.get("conaffinity"), "1")
            self.assertEqual(
                contract["collision"]["source_tile_compiled_collision_mask"],
                [1, 1],
            )
            self.assertTrue(
                contract["collision"]["source_tile_collision_enabled_after_handoff"]
            )
            self.assertIsNotNone(
                next(
                    geom
                    for geom in xml.iter("geom")
                    if geom.get("name") == MODULE.MOON_SPAWN_PAD_GEOM_NAME
                )
            )

    def test_moon_dynamic_ground_transform_defaults_to_mainline_rolling_tiles(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_dir:
            root = Path(temporary_dir)
            canonical = root / "canonical.xml"
            meshes = root / "canonical_meshes"
            native = root / "xgb"
            output = root / "output"
            meshes.mkdir()
            native.mkdir()
            canonical.write_text(
                """<mujoco><worldbody><body name="pelvis">
<freejoint name="floating" /><joint name="joint_a" />
</body></worldbody><actuator><motor name="a" joint="joint_a" />
</actuator></mujoco>""",
                encoding="utf-8",
            )
            scene = native / MODULE.MOON_DYNAMIC_GROUND_SCENE_NAME
            bodies = []
            for i in range(16):
                for j in range(16):
                    bodies.append(
                        f"""<body name="gb_{i}_{j}" pos="{i * 0.1:.1f} {j * 0.1:.1f} 0" gravcomp="1">
  <joint type="free" name="gb_joint_{i}_{j}" />
  <geom name="soil_{i}_{j}" type="box" size="0.049 0.049 0.5" pos="0 0 -0.5" mass="100000000" />
</body>"""
                    )
            scene.write_text(
                "<mujoco><include file=\"xgb.xml\" /><worldbody>\n"
                + "\n".join(bodies)
                + "\n</worldbody></mujoco>",
                encoding="utf-8",
            )
            with mock.patch.object(
                MODULE,
                "MOON_DYNAMIC_GROUND_SOURCE_SCENE_SHA256",
                MODULE._file_sha256(scene),
            ):
                MODULE.prepare_sonic_physics_model(
                    canonical,
                    meshes,
                    scene,
                    output,
                    body_joint_names=("joint_a",),
                    scene_transform=MODULE.MOON_DYNAMIC_GROUND_MOCAP_TRANSFORM,
                )

            manifest = json.loads((output / "manifest.json").read_text())
            contract = manifest["scene_transform_contract"]["dynamic_ground"]
            self.assertEqual(
                contract["collision"]["mode"],
                MODULE.MOON_DYNAMIC_GROUND_COLLISION_TILES,
            )
            self.assertEqual(
                contract["collision"]["source_tile_compiled_collision_mask"],
                [1, 1],
            )
            self.assertTrue(
                contract["collision"]["source_tile_collision_enabled_after_handoff"]
            )


if __name__ == "__main__":
    unittest.main()
