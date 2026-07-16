from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "compose_overworld_scene.py"
SPEC = importlib.util.spec_from_file_location("compose_overworld_scene", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class ComposeOverworldSceneTest(unittest.TestCase):
    def _fixture(self, root: Path, *, overlap: bool = False) -> tuple[Path, Path]:
        native = root / "xgb"
        (native / "assets").mkdir(parents=True)
        (native / "assets" / "curb.stl").write_bytes(b"mesh")
        (native / "scene_a.xml").write_text(
            """<mujoco><include file="robot.xml" /><asset>
<mesh name="curb" file="curb.stl" /></asset><worldbody>
<geom name="floor_a" type="plane" /><geom name="curb_a" type="mesh" mesh="curb" />
</worldbody></mujoco>""",
            encoding="utf-8",
        )
        (native / "scene_b.xml").write_text(
            """<mujoco><include file="robot.xml" /><worldbody>
<geom name="floor_b" type="plane" /><geom name="keep" type="box" size="1 1 1" />
<geom name="wall" type="box" size="0.1 1 1" />
</worldbody></mujoco>""",
            encoding="utf-8",
        )
        layout = {
            "schema_version": 1,
            "world_name": "overworld",
            "mode": "adjacent_physics_v1",
            "visual_contract": {"status": "blocked_cooked_maps"},
            "ground": {"z": 0.0, "friction": [1.0, 0.01, 0.01]},
            "expected_environment_geoms": 3,
            "scenes": [
                {
                    "key": "a",
                    "source_scene": "scene_a.xml",
                    "translation": [0, 0, 0],
                    "source_bounds_xy": [0, 0, 2, 2],
                    "expected_copied_geoms": 1,
                    "remove_geoms": [],
                },
                {
                    "key": "b",
                    "source_scene": "scene_b.xml",
                    "translation": [1 if overlap else 3, 0, 0],
                    "source_bounds_xy": [0, 0, 2, 2],
                    "expected_copied_geoms": 1,
                    "remove_geoms": ["wall"],
                },
            ],
            "connectors": [
                {
                    "key": "a_to_b",
                    "from": "a",
                    "to": "b",
                    "center_xy": [2.5, 1.0],
                    "half_extents_xy": [0.5, 0.5],
                }
            ],
        }
        layout_path = root / "layout.json"
        layout_path.write_text(json.dumps(layout), encoding="utf-8")
        return native, layout_path

    def test_composes_namespaced_adjacent_static_scenes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_dir:
            root = Path(temporary_dir)
            native, layout = self._fixture(root)
            output = root / "output" / "scene_overworld.xml"

            manifest = MODULE.compose_overworld_scene(layout, native, output)

            world = ET.parse(output).getroot()
            self.assertEqual(world.find("include").get("file"), "xgb.xml")
            self.assertIsNotNone(world.find("worldbody/geom[@name='overworld_ground']"))
            self.assertIsNotNone(world.find(".//geom[@name='a__curb_a']"))
            self.assertIsNotNone(world.find(".//geom[@name='b__keep']"))
            self.assertIsNone(world.find(".//geom[@name='b__wall']"))
            self.assertEqual(
                world.find(".//geom[@name='a__curb_a']").get("mesh"), "a__curb"
            )
            self.assertTrue((output.parent / "assets" / "a" / "curb.stl").is_file())
            self.assertEqual(manifest["environment_geoms"], 3)
            self.assertEqual(manifest["scenes"][1]["world_bounds_xy"], [3.0, 0.0, 5.0, 2.0])

    def test_rejects_overlapping_scene_bounds(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_dir:
            root = Path(temporary_dir)
            native, layout = self._fixture(root, overlap=True)
            with self.assertRaisesRegex(MODULE.OverworldCompositionError, "overlap"):
                MODULE.compose_overworld_scene(
                    layout, native, root / "output" / "scene.xml"
                )

    def test_failed_rebuild_preserves_last_complete_bundle(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_dir:
            root = Path(temporary_dir)
            native, layout = self._fixture(root)
            output = root / "output" / "scene_overworld.xml"
            MODULE.compose_overworld_scene(layout, native, output)
            original_scene = output.read_bytes()
            original_manifest = (output.parent / "manifest.json").read_bytes()
            original_asset = (output.parent / "assets" / "a" / "curb.stl").read_bytes()

            invalid_layout = json.loads(layout.read_text(encoding="utf-8"))
            invalid_layout["expected_environment_geoms"] = 999
            layout.write_text(json.dumps(invalid_layout), encoding="utf-8")

            with self.assertRaisesRegex(
                MODULE.OverworldCompositionError, "composed environment"
            ):
                MODULE.compose_overworld_scene(layout, native, output)

            self.assertEqual(output.read_bytes(), original_scene)
            self.assertEqual(
                (output.parent / "manifest.json").read_bytes(), original_manifest
            )
            self.assertEqual(
                (output.parent / "assets" / "a" / "curb.stl").read_bytes(),
                original_asset,
            )
            self.assertEqual(
                list(output.parent.glob(".scene_overworld.staging.*")), []
            )


if __name__ == "__main__":
    unittest.main()
