from __future__ import annotations

import importlib.util
from pathlib import Path
import struct
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "build_weapon_prop_poc_urdf.py"
SPEC = importlib.util.spec_from_file_location("build_weapon_prop_poc_urdf", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


SOURCE_URDF = """<robot name="g1">
  <link name="pelvis">
    <visual><geometry><mesh filename="meshes/pelvis.STL" /></geometry></visual>
  </link>
  <link name="right_rubber_hand" />
  <joint name="right_wrist" type="revolute">
    <parent link="pelvis" /><child link="right_rubber_hand" />
  </joint>
</robot>"""

WEAPON_OBJ = """v 0 0 0
v 1 0 0
v 0 1 0
f 1 2 3
"""

PALETTE_WEAPON_OBJ = """v 0 0 0
v 1 0 0
v 0 1 0
vt 0.21875 0.5
vt 0.34375 0.5
vt 0.59375 0.5
vt 0.71875 0.5
f 1/1 2/1 3/1
f 1/2 2/2 3/2
f 1/3 2/3 3/3
f 1/4 2/4 3/4
"""


class BuildWeaponPropPocUrdfTest(unittest.TestCase):
    def _fixture(self, root: Path) -> tuple[Path, Path]:
        source = root / "source"
        (source / "meshes").mkdir(parents=True)
        (source / "meshes" / "pelvis.STL").write_bytes(b"mesh")
        urdf = source / "g1.urdf"
        urdf.write_text(SOURCE_URDF, encoding="utf-8")
        weapon = root / "blaster.obj"
        weapon.write_text(WEAPON_OBJ, encoding="utf-8")
        return urdf, weapon

    def test_builds_visual_only_fixed_weapon_and_asset_closure(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            urdf, weapon = self._fixture(root)
            license_path = root / "License.txt"
            license_path.write_text("CC0", encoding="utf-8")

            result = MODULE.build_weapon_urdf(
                urdf,
                weapon,
                root / "derived",
                license_path=license_path,
            )

            parsed = ET.parse(result["output_urdf"]).getroot()
            weapon_link = parsed.find(f"link[@name='{MODULE.WEAPON_LINK}']")
            self.assertIsNotNone(weapon_link)
            self.assertIsNone(weapon_link.find("inertial"))
            self.assertIsNone(weapon_link.find("collision"))
            self.assertEqual(
                weapon_link.find("visual/geometry/mesh").get("filename"),
                f"assets/{MODULE.WEAPON_MESH}",
            )
            self.assertEqual(
                weapon_link.find("visual/geometry/mesh").get("scale"),
                MODULE.WEAPON_SCALE,
            )
            self.assertEqual(
                weapon_link.find("visual/origin").attrib,
                {"xyz": "0.12 0 0", "rpy": "0 1.57079632679 0"},
            )
            self.assertEqual(
                weapon_link.find("visual/material/color").get("rgba"),
                MODULE.WEAPON_RGBA,
            )
            self.assertEqual(
                weapon_link.find("visual/material").get("name"),
                "matrix_source_training_orange",
            )
            joint = parsed.find(f"joint[@name='{MODULE.WEAPON_JOINT}']")
            self.assertEqual(joint.get("type"), "fixed")
            self.assertEqual(joint.find("parent").get("link"), "right_rubber_hand")
            self.assertEqual(joint.find("child").get("link"), MODULE.WEAPON_LINK)
            self.assertEqual(joint.find("origin").get("xyz"), "0 0 0")
            self.assertEqual(
                joint.find("origin").get("rpy"), "0 0 0"
            )
            self.assertEqual(
                [item.get("type") for item in parsed.findall("joint")].count("revolute"),
                1,
            )

            assets = Path(result["assets_dir"])
            self.assertTrue((assets / "pelvis.STL").is_file())
            self.assertEqual((assets / "License.txt").read_text(), "CC0")
            weapon_stl = (assets / MODULE.WEAPON_MESH).read_bytes()
            self.assertEqual(struct.unpack("<I", weapon_stl[80:84])[0], 1)
            self.assertEqual(len(weapon_stl), 84 + 50)

    def test_rejects_missing_parent_link(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            urdf, weapon = self._fixture(root)
            with self.assertRaisesRegex(ValueError, "parent link"):
                MODULE.build_weapon_urdf(
                    urdf,
                    weapon,
                    root / "derived",
                    parent_link="missing_hand",
                )

    def test_splits_uv_atlas_into_four_palette_meshes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            weapon = root / "palette_blaster.obj"
            weapon.write_text(PALETTE_WEAPON_OBJ, encoding="utf-8")
            output = root / "palette"
            output.mkdir()

            counts = MODULE.convert_obj_to_palette_stls(weapon, output)

            self.assertEqual(
                counts,
                {"dark": 1, "graphite": 1, "silver_blue": 1, "orange": 1},
            )
            for group in counts:
                stl = (output / f"training_blaster_{group}.stl").read_bytes()
                self.assertEqual(struct.unpack("<I", stl[80:84])[0], 1)
                self.assertEqual(len(stl), 84 + 50)


if __name__ == "__main__":
    unittest.main()
