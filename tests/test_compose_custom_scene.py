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
            self.assertEqual(len(copied), 2)

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
