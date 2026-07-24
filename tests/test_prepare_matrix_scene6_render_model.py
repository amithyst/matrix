from __future__ import annotations

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts" / "prepare_matrix_scene6_render_model.py"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "prepare_matrix_scene6_render_model", SCRIPT
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


MODULE = load_module()


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_closure_sha256(root: Path) -> str:
    entries = [
        {
            "relative_path": name,
            "sha256": sha256(root / "source-meshes" / name),
        }
        for name in ("body.STL", "hand.STL")
    ]
    return MODULE._closure_digest(entries)


def write_fixture(root: Path, *, with_added_mesh: bool = False) -> tuple[Path, Path]:
    meshes = root / "source-meshes"
    meshes.mkdir()
    (meshes / "body.STL").write_bytes(b"fixture-body-mesh")
    (meshes / "hand.STL").write_bytes(b"fixture-hand-mesh")
    existing = (
        f'<mesh name="{MODULE.UE_MESH_NAME}" file="already.stl" />'
        if with_added_mesh
        else ""
    )
    robot = root / MODULE.ROBOT_BASENAME
    robot.write_text(
        f"""<mujoco model="scene6_robot">
  <compiler meshdir="{meshes}" />
  <asset>
    <mesh name="body" file="body.STL" />
    <mesh name="hand" file="hand.STL" />
    {existing}
  </asset>
  <worldbody>
    <body name="pelvis"><freejoint name="floating_base_joint" />
      <geom type="mesh" mesh="body" />
      <body name="hand"><geom type="mesh" mesh="hand" /></body>
    </body>
    <body name="pick_cube" pos="6.0544 4.5 0.886">
      <freejoint name="pick_cube_joint" />
      <geom name="pick_cube_visual" type="box" size="0.03 0.03 0.03" rgba="0.95 0.18 0.05 1" contype="0" conaffinity="0" group="2" />
      <geom name="pick_cube_collision" type="box" size="0.03 0.03 0.03" mass="0.08" rgba="0.95 0.18 0.05 0" friction="3.0 0.03 0.003" group="1" />
    </body>
  </worldbody>
  <actuator><motor name="demo" joint="unused" /></actuator>
</mujoco>
""",
        encoding="utf-8",
    )
    scene = root / MODULE.SCENE_BASENAME
    scene.write_text(
        f"""<mujoco model="scene6">
  <include file="{MODULE.ROBOT_BASENAME}" />
  <worldbody><geom name="worktop" type="box" size="1 1 0.1" /></worldbody>
</mujoco>
""",
        encoding="utf-8",
    )
    return scene, robot


class Scene6RenderModelTest(unittest.TestCase):
    def test_adds_mesh_visual_and_preserves_source_geoms(self) -> None:
        with tempfile.TemporaryDirectory(dir="/tmp") as temporary:
            root = Path(temporary)
            scene, robot = write_fixture(root)
            source_scene = scene.read_bytes()
            source_robot = robot.read_bytes()
            output = root / "render-model-v1"

            receipt = MODULE.derive_render_model(
                source_scene_path=scene,
                source_robot_path=robot,
                output_dir=output,
            )

            derived_scene = output / MODULE.SCENE_BASENAME
            derived_robot = output / MODULE.ROBOT_BASENAME
            receipt_path = output / MODULE.RECEIPT_BASENAME
            self.assertEqual(derived_scene.read_bytes(), source_scene)
            parsed = ET.parse(derived_robot).getroot()
            body = parsed.find(f'.//body[@name="{MODULE.TARGET_BODY}"]')
            assert body is not None
            source_visual = body.find(f'geom[@name="{MODULE.SOURCE_VISUAL}"]')
            collision = body.find(f'geom[@name="{MODULE.SOURCE_COLLISION}"]')
            added = body.find(f'geom[@name="{MODULE.UE_VISUAL}"]')
            assert source_visual is not None and collision is not None and added is not None
            self.assertEqual(source_visual.get("type"), "box")
            self.assertEqual(source_visual.get("size"), "0.03 0.03 0.03")
            self.assertEqual(source_visual.get("rgba"), "0.95 0.18 0.05 0")
            self.assertEqual(collision.get("mass"), "0.08")
            self.assertEqual(added.get("type"), "mesh")
            self.assertEqual(added.get("mesh"), MODULE.UE_MESH_NAME)
            self.assertEqual(added.get("mass"), "0")
            self.assertEqual(added.get("contype"), "0")
            self.assertEqual(added.get("conaffinity"), "0")
            self.assertEqual(parsed.find("compiler").get("meshdir"), "meshes")

            generated = output / "meshes" / MODULE.UE_MESH_FILE
            generated_bytes = generated.read_bytes()
            self.assertEqual(len(generated_bytes), 684)
            self.assertEqual(struct.unpack("<I", generated_bytes[80:84])[0], 12)
            self.assertEqual((output / "meshes" / "body.STL").read_bytes(), b"fixture-body-mesh")
            self.assertEqual((output / "meshes" / "hand.STL").read_bytes(), b"fixture-hand-mesh")
            self.assertEqual(receipt["outputs"]["mesh_closure"]["file_count"], 3)
            self.assertFalse(receipt["invariants"]["trace_frames_changed"])
            self.assertTrue(
                receipt["invariants"]["source_visual_render_alpha_zeroed"]
            )
            self.assertTrue(
                receipt["invariants"]["added_geom_is_massless_and_noncolliding"]
            )
            self.assertEqual(
                receipt["outputs"]["render_robot_model"]["sha256"],
                sha256(derived_robot),
            )
            self.assertEqual(
                json.loads(receipt_path.read_text(encoding="utf-8")), receipt
            )
            self.assertEqual(scene.read_bytes(), source_scene)
            self.assertEqual(robot.read_bytes(), source_robot)

    def test_cube_stl_is_deterministic_and_has_expected_bounds(self) -> None:
        first = MODULE._cube_stl()
        second = MODULE._cube_stl()
        self.assertEqual(first, second)
        self.assertEqual(len(first), 684)
        self.assertEqual(
            hashlib.sha256(first).hexdigest(),
            "88ac8fa62c96ad821bfde5f3346935502799a2884b2e73007ffed2d3922851fc",
        )
        coordinates: list[float] = []
        signed_volume = 0.0
        for index in range(12):
            record = first[84 + index * 50 : 84 + (index + 1) * 50]
            unpacked = struct.unpack("<12fH", record)
            coordinates.extend(unpacked[3:12])
            a = unpacked[3:6]
            b = unpacked[6:9]
            c = unpacked[9:12]
            signed_volume += (
                a[0] * (b[1] * c[2] - b[2] * c[1])
                + a[1] * (b[2] * c[0] - b[0] * c[2])
                + a[2] * (b[0] * c[1] - b[1] * c[0])
            ) / 6.0
        self.assertAlmostEqual(min(coordinates), -0.03, places=7)
        self.assertAlmostEqual(max(coordinates), 0.03, places=7)
        self.assertAlmostEqual(signed_volume, 0.000216, places=9)

    def test_refuses_existing_output_without_mutating_it(self) -> None:
        with tempfile.TemporaryDirectory(dir="/tmp") as temporary:
            root = Path(temporary)
            scene, robot = write_fixture(root)
            output = root / "render-model-v1"
            output.mkdir()
            marker = output / "owner-data"
            marker.write_text("keep", encoding="utf-8")

            with self.assertRaisesRegex(
                MODULE.RenderModelError, "output directory already exists"
            ):
                MODULE.derive_render_model(
                    source_scene_path=scene,
                    source_robot_path=robot,
                    output_dir=output,
                )
            self.assertEqual(marker.read_text(encoding="utf-8"), "keep")

    def test_rejects_model_that_already_has_added_mesh(self) -> None:
        with tempfile.TemporaryDirectory(dir="/tmp") as temporary:
            root = Path(temporary)
            scene, robot = write_fixture(root, with_added_mesh=True)
            output = root / "render-model-v1"
            with self.assertRaisesRegex(MODULE.RenderModelError, "already contains"):
                MODULE.derive_render_model(
                    source_scene_path=scene,
                    source_robot_path=robot,
                    output_dir=output,
                )
            self.assertFalse(output.exists())

    def test_rejects_symlink_mesh_without_publishing(self) -> None:
        with tempfile.TemporaryDirectory(dir="/tmp") as temporary:
            root = Path(temporary)
            scene, robot = write_fixture(root)
            mesh = root / "source-meshes" / "hand.STL"
            real = root / "real-hand.STL"
            mesh.rename(real)
            mesh.symlink_to(real)
            output = root / "render-model-v1"
            with self.assertRaisesRegex(MODULE.RenderModelError, "symlink"):
                MODULE.derive_render_model(
                    source_scene_path=scene,
                    source_robot_path=robot,
                    output_dir=output,
                )
            self.assertFalse(output.exists())

    def test_rejects_wrong_expected_hash_before_output(self) -> None:
        with tempfile.TemporaryDirectory(dir="/tmp") as temporary:
            root = Path(temporary)
            scene, robot = write_fixture(root)
            output = root / "render-model-v1"
            with self.assertRaisesRegex(MODULE.RenderModelError, "SHA256 mismatch"):
                MODULE.derive_render_model(
                    source_scene_path=scene,
                    source_robot_path=robot,
                    output_dir=output,
                    expected_robot_sha256="0" * 64,
                )
            self.assertFalse(output.exists())

    def test_rejects_wrong_mesh_closure_hash_before_output(self) -> None:
        with tempfile.TemporaryDirectory(dir="/tmp") as temporary:
            root = Path(temporary)
            scene, robot = write_fixture(root)
            output = root / "render-model-v1"
            with self.assertRaisesRegex(
                MODULE.RenderModelError, "mesh closure SHA256 mismatch"
            ):
                MODULE.derive_render_model(
                    source_scene_path=scene,
                    source_robot_path=robot,
                    output_dir=output,
                    expected_mesh_closure_sha256="0" * 64,
                )
            self.assertFalse(output.exists())

    def test_rejects_unsupported_file_asset_before_output(self) -> None:
        with tempfile.TemporaryDirectory(dir="/tmp") as temporary:
            root = Path(temporary)
            scene, robot = write_fixture(root)
            tree = ET.parse(robot)
            assets = tree.getroot().find("asset")
            assert assets is not None
            ET.SubElement(assets, "texture", {"name": "bad", "file": "bad.png"})
            tree.write(robot, encoding="utf-8")
            output = root / "render-model-v1"
            with self.assertRaisesRegex(
                MODULE.RenderModelError, "unsupported non-mesh file assets"
            ):
                MODULE.derive_render_model(
                    source_scene_path=scene,
                    source_robot_path=robot,
                    output_dir=output,
                )
            self.assertFalse(output.exists())

    def test_cli_pins_hashes_and_reports_published_files(self) -> None:
        with tempfile.TemporaryDirectory(dir="/tmp") as temporary:
            root = Path(temporary)
            scene, robot = write_fixture(root)
            output = root / "render-model-v1"
            completed = subprocess.run(
                [
                    sys.executable,
                    os.fspath(SCRIPT),
                    "--source-scene",
                    os.fspath(scene),
                    "--source-robot-model",
                    os.fspath(robot),
                    "--output-dir",
                    os.fspath(output),
                    "--expected-source-scene-sha256",
                    sha256(scene),
                    "--expected-source-robot-model-sha256",
                    sha256(robot),
                    "--expected-source-mesh-closure-sha256",
                    source_closure_sha256(root),
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            stdout = json.loads(completed.stdout)
            self.assertTrue(stdout["passed"])
            self.assertEqual(stdout["mesh_closure"]["file_count"], 3)
            self.assertEqual(
                stdout["render_robot_model"]["sha256"],
                sha256(output / MODULE.ROBOT_BASENAME),
            )
            self.assertEqual(
                stdout["receipt"]["sha256"],
                sha256(output / MODULE.RECEIPT_BASENAME),
            )


if __name__ == "__main__":
    unittest.main()
