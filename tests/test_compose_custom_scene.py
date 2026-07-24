from __future__ import annotations

import importlib.util
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "compose_custom_scene.py"
SPEC = importlib.util.spec_from_file_location("compose_custom_scene", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class ComposeCustomSceneTest(unittest.TestCase):
    def test_replaces_robot_include_and_copies_native_assets(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_dir:
            root = Path(temporary_dir)
            native = root / "xgb"
            custom = root / "custom"
            (native / "assets").mkdir(parents=True)
            (native / "assets" / "curb.stl").write_bytes(b"curb")
            (native / "height.png").write_bytes(b"height")
            source = native / "scene_terrain_apart2.xml"
            source.write_text(
                """<mujoco model="XGB scene">
  <include file="xgb.xml" />
  <asset>
    <mesh name="curb" file="curb.stl" />
    <hfield name="terrain" file="../height.png" />
  </asset>
  <worldbody><geom name="wall" type="box" size="1 1 1" /></worldbody>
</mujoco>
""",
                encoding="utf-8",
            )

            output = custom / "scene_terrain_apart2.xml"
            copied = MODULE.compose_custom_scene(source, output)

            scene = ET.parse(output).getroot()
            self.assertEqual(scene.find("include").get("file"), "current.xml")
            self.assertEqual(scene.find("worldbody/geom").get("name"), "wall")
            self.assertEqual((custom / "assets" / "curb.stl").read_bytes(), b"curb")
            self.assertEqual((custom / "height.png").read_bytes(), b"height")
            self.assertEqual(
                scene.find("asset/mesh").get("file"),
                (custom / "assets" / "curb.stl").resolve().as_posix(),
            )
            self.assertEqual(
                scene.find("asset/hfield").get("file"),
                (custom / "height.png").resolve().as_posix(),
            )
            self.assertEqual(len(copied), 2)

    def test_removes_only_exact_requested_geoms_and_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_dir:
            root = Path(temporary_dir)
            native = root / "xgb"
            native.mkdir()
            source = native / "scene.xml"
            source.write_text(
                """<mujoco><include file="xgb.xml" /><worldbody>
<geom name="floor" type="plane" />
<geom name="perimeter" type="box" size="5 0.1 1" />
<geom name="building" type="box" size="1 1 1" />
</worldbody></mujoco>""",
                encoding="utf-8",
            )
            output = root / "custom" / "scene.xml"

            MODULE.compose_custom_scene(
                source, output, remove_geoms=("perimeter",)
            )
            names = [
                geom.get("name")
                for geom in ET.parse(output).getroot().iter("geom")
            ]
            self.assertEqual(names, ["floor", "building"])

            with self.assertRaisesRegex(
                MODULE.SceneCompositionError, "missing requested geoms"
            ):
                MODULE.compose_custom_scene(
                    source, root / "missing.xml", remove_geoms=("missing",)
                )
            with self.assertRaisesRegex(
                MODULE.SceneCompositionError, "must not contain duplicates"
            ):
                MODULE.compose_custom_scene(
                    source,
                    root / "duplicate.xml",
                    remove_geoms=("perimeter", "perimeter"),
                )

    def test_staticizes_freejoint_bodies_without_removing_geometry(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_dir:
            root = Path(temporary_dir)
            native = root / "xgb"
            native.mkdir()
            source = native / "scene.xml"
            source.write_text(
                """<mujoco><include file="xgb.xml" /><worldbody>
<body name="dynamic_tile" pos="1 2 0">
  <joint name="tile_free" type="free" />
  <geom name="tile_collision" type="box" size="1 1 0.1" />
</body>
<body name="legacy_tile" pos="3 4 0">
  <freejoint name="legacy_free" />
  <geom name="legacy_collision" type="box" size="1 1 0.1" />
</body>
<body name="hinged_prop">
  <joint name="hinge" type="hinge" />
  <geom name="hinged_collision" type="box" size="1 1 1" />
</body>
</worldbody></mujoco>""",
                encoding="utf-8",
            )
            output = root / "custom" / "scene.xml"

            MODULE.compose_custom_scene(
                source,
                output,
                staticize_freejoint_bodies=True,
            )

            scene = ET.parse(output).getroot()
            self.assertEqual(
                [body.get("name") for body in scene.iter("body")],
                ["dynamic_tile", "legacy_tile", "hinged_prop"],
            )
            self.assertEqual(
                [geom.get("name") for geom in scene.iter("geom")],
                ["tile_collision", "legacy_collision", "hinged_collision"],
            )
            self.assertEqual(
                [(joint.get("name"), joint.get("type")) for joint in scene.iter("joint")],
                [("hinge", "hinge")],
            )
            self.assertEqual(
                MODULE.freejoint_body_names(ET.parse(source).getroot()),
                ("dynamic_tile", "legacy_tile"),
            )

    def test_rejects_asset_collision_with_custom_robot(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_dir:
            root = Path(temporary_dir)
            native = root / "xgb"
            custom = root / "custom"
            (native / "assets").mkdir(parents=True)
            (custom / "assets").mkdir(parents=True)
            (native / "assets" / "shared.stl").write_bytes(b"native")
            (custom / "assets" / "shared.stl").write_bytes(b"robot")
            source = native / "scene.xml"
            source.write_text(
                """<mujoco><include file="xgb.xml" />
<asset><mesh name="shared" file="shared.stl" /></asset></mujoco>""",
                encoding="utf-8",
            )

            with self.assertRaises(MODULE.SceneCompositionError):
                MODULE.compose_custom_scene(source, custom / "scene.xml")


if __name__ == "__main__":
    unittest.main()
