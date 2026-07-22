from __future__ import annotations

import importlib.util
import json
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
    "pelvis_contour_link",
    "left_hip_pitch_link",
    "right_hip_pitch_link",
    "left_hip_yaw_link",
    "left_ankle_roll_link",
    "right_ankle_roll_link",
    "left_knee_link",
    "torso_link",
    "head_link",
    "logo_link",
    "left_shoulder_pitch_link",
    "right_shoulder_pitch_link",
    "left_shoulder_roll_link",
    "left_elbow_link",
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
        mesh="{name}" class="visual" /><geom name="{name}_collision"
        type="mesh" mesh="{name}" class="collision" contype="1"
        conaffinity="1" density="12" /></body>"""
        for name in PROFILE_LINKS
    )
    return (
        '<mujoco><asset><material name="default_material" '
        f'rgba="0.75 0.75 0.75 1" />{meshes}</asset><worldbody>{bodies}'
        "</worldbody></mujoco>"
    )


class ApplyUrdfVisualMaterialsTest(unittest.TestCase):
    def test_preserves_explicit_mjcf_source_material_for_inventory_mesh(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            urdf = root / "g1.urdf"
            urdf.write_text(
                """<robot name="g1"><link name="pelvis"><visual><geometry><mesh filename="pelvis.STL" /></geometry></visual></link></robot>""",
                encoding="utf-8",
            )
            mjcf = root / "g1.xml"
            mjcf.write_text(
                """<mujoco><asset>
  <mesh name="pelvis" file="pelvis.STL" />
  <mesh name="creative_prop" file="creative_prop.stl" />
  <material name="matrix_source_creative_prop" rgba="0.2 0.4 0.8 1" />
</asset><worldbody>
  <body name="pelvis"><geom type="mesh" mesh="pelvis" class="visual" /></body>
  <body name="creative_item__prop__0">
    <geom name="creative_item__prop__0__visual" type="mesh" mesh="creative_prop" class="visual" material="matrix_source_creative_prop" />
  </body>
</worldbody></mujoco>""",
                encoding="utf-8",
            )

            summary = MODULE.apply_urdf_visual_materials(
                urdf,
                mjcf,
                profile="urdf",
                profile_scope_alpha=0.99609375,
            )

            parsed = ET.parse(mjcf).getroot()
            geom = parsed.find(".//geom[@name='creative_item__prop__0__visual']")
            self.assertIsNotNone(geom)
            self.assertTrue(geom.get("material", "").startswith("urdf_visual_"))
            self.assertEqual(geom.get("rgba"), "0.2 0.4 0.8 0.99609375")
            self.assertEqual(summary.unmatched_visual_geoms, 0)
    def test_profile_preserves_explicit_source_material_for_attachment(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            urdf = root / "g1_29dof.urdf"
            mjcf = root / "g1_29dof.xml"
            accessory_urdf = """<link name="training_blaster_link"><visual>
            <geometry><mesh filename="assets/training_blaster.stl" /></geometry>
            <material name="matrix_source_training_orange">
            <color rgba="0.95 0.19 0.035 1" /></material></visual></link>"""
            accessory_mjcf_mesh = (
                '<mesh name="training_blaster" file="training_blaster.stl" />'
            )
            accessory_mjcf_body = """<body name="training_blaster_link">
            <geom name="training_blaster_link_visual" type="mesh"
            mesh="training_blaster" class="visual" /></body>"""
            urdf.write_text(
                _profile_urdf().replace("</robot>", accessory_urdf + "</robot>"),
                encoding="utf-8",
            )
            mjcf.write_text(
                _profile_mjcf()
                .replace("</asset>", accessory_mjcf_mesh + "</asset>")
                .replace("</worldbody>", accessory_mjcf_body + "</worldbody>"),
                encoding="utf-8",
            )

            summary = MODULE.apply_urdf_visual_materials(
                urdf,
                mjcf,
                profile_scope_alpha=0.99609375,
            )

            self.assertEqual(summary.profile_id, "matrix_g1_stock_v1")
            parsed = ET.parse(mjcf).getroot()
            accessory_geom = next(
                geom
                for geom in parsed.iter("geom")
                if geom.get("name") == "training_blaster_link_visual"
            )
            self.assertEqual(
                accessory_geom.get("rgba"),
                "0.95 0.19 0.035 0.99609375",
            )
            self.assertTrue(
                accessory_geom.get("material", "").startswith(
                    MODULE.GENERATED_PREFIX + "matrix_source_training_orange_"
                )
            )

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

    def test_applies_matrix_g1_surface_profile_by_link_semantics(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            urdf = root / "g1_29dof.urdf"
            mjcf = root / "g1_29dof.xml"
            urdf.write_text(_profile_urdf(), encoding="utf-8")
            mjcf.write_text(_profile_mjcf(), encoding="utf-8")

            summary = MODULE.apply_urdf_visual_materials(urdf, mjcf)

            self.assertEqual(summary.profile_id, "matrix_g1_stock_v1")
            self.assertEqual(summary.source_styles, 4)
            self.assertEqual(summary.styled_geoms, len(PROFILE_LINKS))
            self.assertEqual(summary.styled_collision_geoms, len(PROFILE_LINKS))
            self.assertEqual(summary.unmatched_visual_geoms, 0)
            parsed = ET.parse(mjcf).getroot()
            materials = {
                material.get("name"): material
                for material in parsed.find("asset").findall("material")
            }
            body_by_name = {
                body.get("name"): body for body in parsed.iter("body")
            }

            expected = {
                "pelvis": ("0.018 0.024 0.035 1", "0.62"),
                "left_ankle_roll_link": ("0.018 0.024 0.035 1", "0.62"),
                "left_wrist_roll_link": ("0.018 0.024 0.035 1", "0.62"),
                "left_hip_pitch_link": ("0.055 0.075 0.11 1", "0.58"),
                "head_link": ("0.055 0.075 0.11 1", "0.58"),
                "torso_link": ("0.9 0.94 1 1", "0.48"),
                "pelvis_contour_link": ("0.42 0.42 0.42 1", "0.38"),
                "left_hip_yaw_link": ("0.42 0.42 0.42 1", "0.38"),
                "left_knee_link": ("0.42 0.42 0.42 1", "0.38"),
                "logo_link": ("0.42 0.42 0.42 1", "0.38"),
                "left_shoulder_roll_link": ("0.42 0.42 0.42 1", "0.38"),
                "left_elbow_link": ("0.42 0.42 0.42 1", "0.38"),
            }
            for body_name, (rgba, roughness) in expected.items():
                body = body_by_name[body_name]
                geom = body.find("geom")
                self.assertIsNotNone(geom)
                self.assertEqual(geom.get("rgba"), rgba)
                material = materials[geom.get("material")]
                self.assertEqual(material.get("rgba"), rgba)
                self.assertEqual(material.get("roughness"), roughness)
                self.assertEqual(
                    material.get("metallic"),
                    "0.35" if rgba == "0.42 0.42 0.42 1" else "0",
                )
                collision = next(
                    geom
                    for geom in body.findall("geom")
                    if geom.get("name") == f"{body_name}_collision"
                )
                self.assertEqual(collision.get("rgba"), rgba)
                self.assertEqual(collision.get("material"), geom.get("material"))
                self.assertEqual(collision.get("contype"), "1")
                self.assertEqual(collision.get("conaffinity"), "1")
                self.assertEqual(collision.get("density"), "12")

    def test_matrix_blue_skin_remains_selectable(self) -> None:
        selection = MODULE.resolve_g1_skin("matrix-blue")
        self.assertEqual(selection.profile_id, "matrix_g1_v2")
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            urdf = root / "g1_29dof.urdf"
            mjcf = root / "g1_29dof.xml"
            urdf.write_text(_profile_urdf(), encoding="utf-8")
            mjcf.write_text(_profile_mjcf(), encoding="utf-8")

            summary = MODULE.apply_urdf_visual_materials(
                urdf,
                mjcf,
                profile_path=selection.profile_path,
            )

            self.assertEqual(summary.profile_id, "matrix_g1_v2")
            body = next(
                item
                for item in ET.parse(mjcf).getroot().iter("body")
                if item.get("name") == "left_knee_link"
            )
            self.assertEqual(body.find("geom").get("rgba"), "0.015 0.2 0.95 1")

    def test_ue_scope_tag_marks_only_registered_g1_materials(self) -> None:
        selection = MODULE.resolve_g1_skin()
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            urdf = root / "g1_29dof.urdf"
            mjcf = root / "g1_29dof.xml"
            urdf.write_text(_profile_urdf(), encoding="utf-8")
            mjcf.write_text(_profile_mjcf(), encoding="utf-8")

            MODULE.apply_urdf_visual_materials(
                urdf,
                mjcf,
                profile_path=selection.profile_path,
                profile_scope_alpha=selection.ue_scope_alpha,
            )

            tagged = [
                geom.get("rgba")
                for geom in ET.parse(mjcf).getroot().iter("geom")
            ]
            self.assertTrue(
                all(value.endswith(" 0.99609375") for value in tagged)
            )

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

    def test_skin_registry_is_extensible_and_defaults_to_stock(self) -> None:
        stock = MODULE.resolve_g1_skin()
        blue = MODULE.resolve_g1_skin("matrix-blue")
        self.assertEqual(stock.skin_id, "unitree-stock")
        self.assertEqual(stock.profile_path, MODULE.DEFAULT_PROFILE_PATH)
        self.assertEqual(stock.ue_colors[-1], (0.42, 0.42, 0.42))
        self.assertEqual(blue.ue_colors[-1], (0.015, 0.2, 0.95))
        self.assertEqual(
            MODULE.resolve_g1_skin_for_profile("matrix_g1_v2").skin_id,
            "matrix-blue",
        )
        with self.assertRaisesRegex(
            MODULE.VisualMaterialError,
            "not requested profile",
        ):
            MODULE._resolve_requested_skin(
                "unitree-stock",
                "matrix_g1_v2",
                MODULE.DEFAULT_SKIN_REGISTRY_PATH,
            )
        with self.assertRaisesRegex(MODULE.VisualMaterialError, "available skins"):
            MODULE.resolve_g1_skin("gold")

        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            profile = MODULE._load_profile(MODULE.DEFAULT_PROFILE_PATH)
            profile["profile_id"] = "matrix_g1_gold_v1"
            profile["materials"]["silver_gray_accent"]["rgba"] = [
                0.75,
                0.45,
                0.05,
                1.0,
            ]
            (root / "gold.json").write_text(
                json.dumps(profile),
                encoding="utf-8",
            )
            registry = {
                "schema_version": 1,
                "robot_id": "unitree_g1",
                "default_skin": "gold",
                "ue_scope_alpha": 0.99609375,
                "skins": {
                    "gold": {
                        "label": "黄金",
                        "profile": "gold.json",
                    }
                },
            }
            registry_path = root / "g1_skins.json"
            registry_path.write_text(json.dumps(registry), encoding="utf-8")

            gold = MODULE.resolve_g1_skin(registry_path=registry_path)

            self.assertEqual(gold.skin_id, "gold")
            self.assertEqual(gold.profile_id, "matrix_g1_gold_v1")
            self.assertIn("0.75,0.45,0.05", gold.ue_palette)
            self.assertEqual(gold.ue_scope_alpha, 0.99609375)

            registry["ue_scope_alpha"] = 0.99999
            registry_path.write_text(json.dumps(registry), encoding="utf-8")
            with self.assertRaisesRegex(
                MODULE.VisualMaterialError,
                "0.999",
            ):
                MODULE.resolve_g1_skin(registry_path=registry_path)

    def test_ue_material_bridge_consumes_selected_skin_palette(self) -> None:
        bridge = (
            REPO_ROOT / "src" / "ue_shims" / "matrix_ue_material_fix.c"
        ).read_text(encoding="utf-8")

        self.assertIn('getenv("MATRIX_G1_SKIN")', bridge)
        self.assertIn('getenv("MATRIX_G1_MATERIAL_PALETTE")', bridge)
        self.assertIn('getenv("MATRIX_G1_MATERIAL_SCOPE_ALPHA")', bridge)
        self.assertIn("MAX_G1_PROFILE_COLORS = 16", bridge)
        self.assertIn("g1_profile_colors", bridge)
        self.assertIn("component_matches(color.alpha, g1_scope_alpha)", bridge)
        self.assertIn("color.alpha = 1.0f", bridge)
        self.assertIn("if (material_index == -1)", bridge)
        self.assertIn("mapped G1 material profile ", bridge)
        self.assertIn("section to slot 0\\n", bridge)

    def test_default_profile_and_launcher_pipeline_contract(self) -> None:
        self.assertEqual(
            MODULE.DEFAULT_PROFILE_PATH.name,
            "matrix_g1_stock_v1.json",
        )
        launcher = (REPO_ROOT / "scripts" / "run_custom_urdf.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("PIPELINE_VERSION=19", launcher)
        self.assertIn("--describe-skin", launcher)
        self.assertIn("--ue-scope-tag", launcher)
        self.assertIn(
            'MATRIX_G1_MATERIAL_PALETTE="$G1_MATERIAL_PALETTE"',
            launcher,
        )
        self.assertIn(
            'MATRIX_G1_MATERIAL_SCOPE_ALPHA="$G1_MATERIAL_SCOPE_ALPHA"',
            launcher,
        )
        self.assertIn(
            '"$SCRIPT_DIR/apply_urdf_visual_materials.py"',
            launcher,
        )
        outer = (REPO_ROOT / "scripts" / "run_matrix_sonic.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("--skin ID", outer)
        self.assertIn('export MATRIX_G1_SKIN="$G1_SKIN"', outer)
        inner = (REPO_ROOT / "scripts" / "run_sim.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("--creative-inventory-catalog", inner)
        self.assertLess(
            inner.index(
                "unset MATRIX_G1_MATERIAL_PALETTE "
                "MATRIX_G1_MATERIAL_SCOPE_ALPHA"
            ),
            inner.index("# 基础"),
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
