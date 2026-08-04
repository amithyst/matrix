from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "compile_town10_mujoco_collision.py"
SPEC = importlib.util.spec_from_file_location(
    "compile_town10_mujoco_collision",
    SCRIPT_PATH,
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class CompileTown10MujocoCollisionTest(unittest.TestCase):
    def _source_document(self) -> dict[str, object]:
        return {
            "schema": MODULE.SOURCE_SCHEMA,
            "source": {
                "dataset": "synthetic-town10-vision-smoke",
                "revision": "unit-test",
            },
            "coordinate_frame": {
                "id": "frame.synthetic_camera_fused_zup",
                "meters_per_unit": 0.01,
                "up_axis": "Z",
                "handedness": "right",
            },
            "objects": [
                {
                    "id": "town10.road_barrier",
                    "kind": "barrier",
                    "transform": {
                        "translation": [100.0, -50.0, 20.0],
                        "rotation_xyzw": [0.0, 0.0, 0.70710678, 0.70710678],
                        "scale": [1.0, 2.0, 1.0],
                    },
                    "geometry": {
                        "type": "box",
                        "size": [40.0, 20.0, 10.0],
                    },
                },
                {
                    "id": "town10.visual_building_mesh",
                    "kind": "building",
                    "transform": {
                        "translation": [0.0, 0.0, 0.0],
                        "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
                        "scale": [2.0, 1.0, 1.0],
                    },
                    "geometry": {
                        "type": "mesh",
                        "vertices": [
                            [-10.0, -5.0, 0.0],
                            [20.0, -5.0, 0.0],
                            [20.0, 15.0, 0.0],
                            [-10.0, 15.0, 0.0],
                            [-10.0, -5.0, 30.0],
                            [20.0, -5.0, 30.0],
                            [20.0, 15.0, 30.0],
                            [-10.0, 15.0, 30.0],
                        ],
                        "faces": [
                            [0, 1, 2],
                            [0, 2, 3],
                            [4, 6, 5],
                            [4, 7, 6],
                        ],
                    },
                },
                {
                    "id": "town10.visual_only",
                    "collision_enabled": False,
                    "geometry": {
                        "type": "box",
                        "half_extents": [1.0, 1.0, 1.0],
                    },
                },
            ],
        }

    def test_compiles_units_mesh_simplification_manifest_and_smoke(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source = root / "town10_source.json"
            output = root / "out"
            source.write_text(json.dumps(self._source_document()), encoding="utf-8")

            result = MODULE.compile_town10_mujoco_collision(source, output)

            self.assertEqual(result.xml_path, output / "collision.xml")
            self.assertTrue(result.manifest_path.is_file())
            xml_root = ET.parse(result.xml_path).getroot()
            self.assertEqual(xml_root.get("model"), "matrix_town10_collision")
            compiler = xml_root.find("compiler")
            self.assertIsNotNone(compiler)
            self.assertEqual(compiler.get("meshdir"), "meshes")

            barrier_body = xml_root.find(
                ".//body[@name='body_town10.road_barrier']"
            )
            self.assertIsNotNone(barrier_body)
            self.assertEqual(barrier_body.get("pos"), "1 -0.5 0.2")
            self.assertEqual(
                barrier_body.get("quat"),
                "0.707106781187 0 0 0.707106781187",
            )
            barrier_geom = barrier_body.find("geom")
            self.assertEqual(barrier_geom.get("type"), "box")
            self.assertEqual(barrier_geom.get("size"), "0.2 0.2 0.05")
            self.assertEqual(barrier_geom.get("contype"), "1")
            self.assertEqual(barrier_geom.get("conaffinity"), "1")

            meshes = list((output / "meshes").glob("*.obj"))
            self.assertEqual(
                [mesh.name for mesh in meshes],
                ["town10.visual_building_mesh__aabb.obj"],
            )
            mesh_text = meshes[0].read_text(encoding="utf-8").splitlines()
            self.assertEqual(mesh_text[0], "# matrix-town10 mesh-aabb-box-v1")
            self.assertEqual(len([line for line in mesh_text if line.startswith("v ")]), 8)
            self.assertEqual(len([line for line in mesh_text if line.startswith("f ")]), 12)
            self.assertIn("v -0.2 -0.05 0", mesh_text)
            self.assertIn("v 0.4 0.15 0.3", mesh_text)

            manifest = json.loads(result.manifest_path.read_text(encoding="utf-8"))
            self.assertEqual(manifest["schema"], MODULE.BUNDLE_SCHEMA)
            self.assertEqual(manifest["source"]["sha256"], MODULE._file_sha256(source))
            self.assertEqual(manifest["artifacts"]["xml"]["sha256"], MODULE._file_sha256(result.xml_path))
            self.assertEqual(
                manifest["artifacts"]["bundle_sha256"],
                MODULE._tree_sha256(output, exclude=frozenset({"manifest.json"})),
            )
            objects = {
                item["id"]: item
                for item in manifest["collision"]["objects"]
            }
            self.assertEqual(
                objects["town10.road_barrier"]["strategy"],
                "analytic-box-v1",
            )
            self.assertEqual(
                objects["town10.road_barrier"]["box"]["half_extents_m"],
                [0.2, 0.2, 0.05],
            )
            self.assertEqual(
                objects["town10.visual_building_mesh"]["strategy"],
                "mesh-aabb-box-v1",
            )
            self.assertEqual(
                objects["town10.visual_building_mesh"]["source_mesh"],
                {"vertices": 8, "faces": 4},
            )
            self.assertEqual(
                objects["town10.visual_building_mesh"]["simplified_mesh"][
                    "aabb_max_m"
                ],
                [0.4, 0.15, 0.3],
            )
            self.assertEqual(
                objects["town10.visual_only"]["strategy"],
                "skipped-collision-disabled",
            )

            first_manifest = result.manifest_path.read_text(encoding="utf-8")
            MODULE.compile_town10_mujoco_collision(source, output)
            self.assertEqual(
                result.manifest_path.read_text(encoding="utf-8"),
                first_manifest,
            )

            smoke = MODULE.smoke_mujoco_collision(result.xml_path)
            self.assertEqual(smoke["structural"], "passed")
            self.assertEqual(smoke["active_collision_geom_count"], 2)

    def test_rejects_non_canonical_frame_and_degenerate_mesh(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source = root / "town10_source.json"
            document = self._source_document()
            document["coordinate_frame"] = {
                "meters_per_unit": 1.0,
                "up_axis": "Y",
                "handedness": "right",
            }
            source.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(
                MODULE.Town10CollisionCompileError,
                "Z-up",
            ):
                MODULE.compile_town10_mujoco_collision(source, root / "out")

            document = self._source_document()
            document["objects"] = [
                {
                    "id": "town10.flat_mesh",
                    "geometry": {
                        "type": "mesh",
                        "vertices": [[0, 0, 0], [1, 0, 0], [0, 1, 0]],
                        "faces": [[0, 1, 2]],
                    },
                }
            ]
            source.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(
                MODULE.Town10CollisionCompileError,
                "degenerate",
            ):
                MODULE.compile_town10_mujoco_collision(source, root / "out")


if __name__ == "__main__":
    unittest.main()
