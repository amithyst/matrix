from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "apply_urdf_visual_materials.py"
SPEC = importlib.util.spec_from_file_location(
    "apply_urdf_visual_materials", SCRIPT_PATH
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


URDF = """<robot name="g1">
  <material name="white"><color rgba="0.7 0.7 0.7 1" /></material>
  <material name="dark"><color rgba="0.2 0.2 0.2 1" /></material>
  <link name="pelvis"><visual><geometry><mesh filename="meshes/pelvis.STL" /></geometry>
    <material name="dark" /></visual></link>
  <link name="torso"><visual><geometry><mesh filename="meshes/torso.STL" /></geometry>
    <material name="white" /></visual></link>
  <link name="head"><visual><geometry><mesh filename="meshes/head.STL" /></geometry>
    <material><color rgba="0.1 0.3 0.8 1" /></material></visual></link>
</robot>"""

MJCF = """<mujoco><default><default class="visual"><geom material="default_material"
contype="0" conaffinity="0" group="2" /></default></default>
<asset><material name="default_material" rgba="0.75294 0.75294 0.75294 1" />
<mesh name="pelvis" file="pelvis.STL" /><mesh name="torso" file="torso.STL" />
<mesh name="head" file="head.STL" /></asset><worldbody><body name="pelvis">
<geom name="pelvis_visual" type="mesh" mesh="pelvis" class="visual" />
<geom name="pelvis_collision" type="box" size="0.1 0.1 0.1" class="collision" />
<body name="torso"><geom type="mesh" mesh="torso" class="visual" /></body>
<body name="renamed_head"><geom name="head_visual" type="mesh" mesh="head"
class="visual" /></body></body></worldbody></mujoco>"""

PROFILE_LINKS = (
    "pelvis",
    "left_hip_pitch_link",
    "right_hip_pitch_link",
    "left_ankle_roll_link",
    "right_ankle_roll_link",
    "torso_link",
    "head_link",
    "logo_link",
    "left_shoulder_pitch_link",
    "right_shoulder_pitch_link",
    "left_wrist_roll_link",
)


def _profile_urdf() -> str:
    links = "".join(
        f"""<link name="{name}"><visual><geometry>
        <mesh filename="meshes/{name}.STL" /></geometry>
        <material name="source_gray"><color rgba="0.7 0.7 0.7 1" /></material>
        </visual></link>"""
        for name in PROFILE_LINKS
    )
    return f'<robot name="g1_29dof">{links}</robot>'


def _profile_mjcf() -> str:
    meshes = "".join(
        f'<mesh name="{name}" file="{name}.STL" />' for name in PROFILE_LINKS
    )
    bodies = "".join(
        f"""<body name="{name}"><geom name="{name}_visual" type="mesh"
        mesh="{name}" class="visual" /></body>"""
        for name in PROFILE_LINKS
    )
    return (
        '<mujoco><asset><material name="default_material" '
        f'rgba="0.75 0.75 0.75 1" />{meshes}</asset><worldbody>{bodies}'
        "</worldbody></mujoco>"
    )


class ApplyUrdfVisualMaterialsTest(unittest.TestCase):
    def test_preserves_named_inline_and_mesh_fallback_colors(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            urdf = root / "g1.urdf"
            mjcf = root / "g1.xml"
            urdf.write_text(URDF, encoding="utf-8")
            mjcf.write_text(MJCF, encoding="utf-8")

            summary = MODULE.apply_urdf_visual_materials(urdf, mjcf)

            self.assertEqual(summary.source_visuals, 3)
            self.assertEqual(summary.source_styles, 3)
            self.assertEqual(summary.styled_geoms, 3)
            self.assertEqual(summary.unmatched_visual_geoms, 0)
            parsed = ET.parse(mjcf).getroot()
            generated = {
                item.get("name"): item.get("rgba")
                for item in parsed.find("asset").findall("material")
                if item.get("name", "").startswith(MODULE.GENERATED_PREFIX)
            }
            self.assertEqual(
                set(generated.values()),
                {"0.7 0.7 0.7 1", "0.2 0.2 0.2 1", "0.1 0.3 0.8 1"},
            )
            visual_geoms = [
                geom for geom in parsed.iter("geom") if geom.get("type") == "mesh"
            ]
            self.assertEqual(
                {geom.get("rgba") for geom in visual_geoms},
                {"0.7 0.7 0.7 1", "0.2 0.2 0.2 1", "0.1 0.3 0.8 1"},
            )
            self.assertTrue(
                all(
                    geom.get("material", "").startswith(MODULE.GENERATED_PREFIX)
                    for geom in visual_geoms
                )
            )
            collision = next(
                geom
                for geom in parsed.iter("geom")
                if geom.get("name") == "pelvis_collision"
            )
            self.assertIsNone(collision.get("rgba"))

    def test_reapplication_is_byte_stable(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            urdf = root / "g1.urdf"
            mjcf = root / "g1.xml"
            urdf.write_text(URDF, encoding="utf-8")
            mjcf.write_text(MJCF, encoding="utf-8")
            MODULE.apply_urdf_visual_materials(urdf, mjcf)
            first = mjcf.read_bytes()

            MODULE.apply_urdf_visual_materials(urdf, mjcf)

            self.assertEqual(mjcf.read_bytes(), first)

    def test_applies_aue_g1_surface_profile_by_link_semantics(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            urdf = root / "g1_29dof.urdf"
            mjcf = root / "g1_29dof.xml"
            urdf.write_text(_profile_urdf(), encoding="utf-8")
            mjcf.write_text(_profile_mjcf(), encoding="utf-8")

            summary = MODULE.apply_urdf_visual_materials(urdf, mjcf)

            self.assertEqual(summary.profile_id, "aue_g1_v1")
            self.assertEqual(summary.source_styles, 3)
            self.assertEqual(summary.styled_geoms, len(PROFILE_LINKS))
            self.assertEqual(summary.unmatched_visual_geoms, 0)
            parsed = ET.parse(mjcf).getroot()
            materials = {
                material.get("name"): material
                for material in parsed.find("asset").findall("material")
            }
            geom_by_body = {
                body.get("name"): body.find("geom") for body in parsed.iter("body")
            }

            expected = {
                "pelvis": ("0.035 0.035 0.035 1", "0.62"),
                "left_ankle_roll_link": ("0.035 0.035 0.035 1", "0.62"),
                "left_wrist_roll_link": ("0.035 0.035 0.035 1", "0.62"),
                "left_hip_pitch_link": ("0.16 0.16 0.16 1", "0.58"),
                "head_link": ("0.16 0.16 0.16 1", "0.58"),
                "torso_link": ("0.72 0.72 0.68 1", "0.52"),
            }
            for body_name, (rgba, roughness) in expected.items():
                geom = geom_by_body[body_name]
                self.assertIsNotNone(geom)
                self.assertEqual(geom.get("rgba"), rgba)
                material = materials[geom.get("material")]
                self.assertEqual(material.get("rgba"), rgba)
                self.assertEqual(material.get("roughness"), roughness)
                self.assertEqual(material.get("metallic"), "0")

    def test_explicit_urdf_profile_disables_g1_override(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            urdf = root / "g1_29dof.urdf"
            mjcf = root / "g1_29dof.xml"
            urdf.write_text(_profile_urdf(), encoding="utf-8")
            mjcf.write_text(_profile_mjcf(), encoding="utf-8")

            summary = MODULE.apply_urdf_visual_materials(
                urdf, mjcf, profile=MODULE.URDF_PROFILE
            )

            self.assertEqual(summary.profile_id, MODULE.URDF_PROFILE)
            self.assertEqual(summary.source_styles, 1)
            parsed = ET.parse(mjcf).getroot()
            visual_geoms = [geom for geom in parsed.iter("geom")]
            self.assertTrue(
                all(geom.get("rgba") == "0.7 0.7 0.7 1" for geom in visual_geoms)
            )

    def test_rejects_out_of_range_color(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            urdf = root / "bad.urdf"
            mjcf = root / "g1.xml"
            urdf.write_text(
                URDF.replace("0.7 0.7 0.7 1", "1.2 0.7 0.7 1"),
                encoding="utf-8",
            )
            mjcf.write_text(MJCF, encoding="utf-8")
            with self.assertRaises(MODULE.VisualMaterialError):
                MODULE.apply_urdf_visual_materials(urdf, mjcf)


if __name__ == "__main__":
    unittest.main()
