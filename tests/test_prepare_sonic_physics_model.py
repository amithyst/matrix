from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
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


if __name__ == "__main__":
    unittest.main()
