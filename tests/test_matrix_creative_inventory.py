from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
from threading import RLock
import unittest
from unittest import mock

try:
    import mujoco
except ModuleNotFoundError:
    mujoco = None


ROOT = Path(__file__).resolve().parents[1]


def _load(name: str, relative_path: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative_path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


INJECT = _load("inject_creative_inventory", "scripts/inject_creative_inventory.py")
RUNTIME = _load("matrix_creative_inventory", "scripts/matrix_creative_inventory.py")


def _write_tetrahedron(path: Path) -> None:
    vertices = ((0.0, 0.0, 0.0), (0.1, 0.0, 0.0), (0.0, 0.1, 0.0), (0.0, 0.0, 0.1))
    faces = ((0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3))
    with path.open("wb") as stream:
        stream.write(b"creative inventory test".ljust(80, b"\0"))
        stream.write(struct.pack("<I", len(faces)))
        for face in faces:
            stream.write(struct.pack("<3f", 0.0, 0.0, 1.0))
            for index in face:
                stream.write(struct.pack("<3f", *vertices[index]))
            stream.write(struct.pack("<H", 0))


class _Pose:
    x = 0.0
    y = 0.0
    yaw_rad = 0.0


class _Environment:
    def __init__(self, model, data) -> None:
        self.mj_model = model
        self.mj_data = data


class _Simulator:
    def __init__(self, model, data) -> None:
        self.sim_env = _Environment(model, data)
        self._step_lock = RLock()


class CreativeInventoryTest(unittest.TestCase):
    def _fixture(self, root: Path, *, pool_size: int = 2) -> tuple[Path, Path, Path]:
        assets = root / "assets"
        assets.mkdir()
        mesh = root / "item.stl"
        _write_tetrahedron(mesh)
        catalog = root / "catalog.json"
        catalog.write_text(
            json.dumps(
                {
                    "schema": "matrix-creative-inventory/v1",
                    "items": [
                        {
                            "item_id": "test_prop",
                            "label": "Test prop",
                            "pool_size": pool_size,
                            "mass_kg": 1.2,
                            "collision_half_size": [0.1, 0.05, 0.05],
                            "spawn_distance_m": 1.0,
                            "spawn_height_m": 1.0,
                            "spawn_quat": [1.0, 0.0, 0.0, 0.0],
                            "visuals": [
                                {
                                    "mesh": "item.stl",
                                    "rgba": [0.8, 0.2, 0.1, 1.0],
                                    "scale": [1.0, 1.0, 1.0],
                                }
                            ],
                        }
                    ],
                }
            ),
            encoding="utf-8",
        )
        mjcf = root / "model.xml"
        mjcf.write_text(
            """<mujoco model="inventory-test">
  <compiler meshdir="assets" />
  <default>
    <default class="visual"><geom contype="0" conaffinity="0" /></default>
    <default class="collision"><geom rgba="0 0 0 0" /></default>
  </default>
  <worldbody>
    <geom name="floor" type="plane" size="0 0 0.05" />
    <body name="pelvis" pos="0 0 1"><freejoint name="floating_base" />
      <geom type="sphere" size="0.05" mass="1" />
    </body>
  </worldbody>
</mujoco>
""",
            encoding="utf-8",
        )
        return mjcf, assets, catalog

    def test_runtime_fails_closed_without_mujoco_bindings(self) -> None:
        with mock.patch.object(RUNTIME, "mujoco", None):
            with self.assertRaises(RUNTIME.CreativeInventoryError) as context:
                RUNTIME.CreativeInventoryRuntime(None, Path("unused.json"))
        self.assertEqual(context.exception.code, "E_INVENTORY_DEPENDENCY")

    @unittest.skipIf(mujoco is None, "MuJoCo Python bindings are unavailable")
    def test_injects_bounded_pool_and_runtime_spawns_once_into_physics(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            mjcf, assets, catalog = self._fixture(root)

            summary = INJECT.inject_catalog(mjcf, assets, catalog)
            model = mujoco.MjModel.from_xml_path(str(mjcf))
            data = mujoco.MjData(model)
            runtime = RUNTIME.CreativeInventoryRuntime(
                _Simulator(model, data), catalog
            )

            self.assertEqual(summary["pool_bodies"], 2)
            self.assertEqual(model.nq, 21)
            self.assertEqual(model.nv, 18)
            self.assertEqual(runtime.mapping()["items"][0]["remaining"], 2)
            spawned = runtime.spawn("test_prop", _Pose())
            self.assertEqual(spawned.position, (1.0, 0.0, 1.0))
            self.assertEqual(runtime.mapping()["items"][0]["remaining"], 1)
            joint_id = mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_JOINT,
                "creative_item__test_prop__0__freejoint",
            )
            qpos_address = int(model.jnt_qposadr[joint_id])
            self.assertAlmostEqual(float(data.qpos[qpos_address]), 1.0)
            equality_id = mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_EQUALITY,
                "creative_item__test_prop__0__storage_weld",
            )
            self.assertEqual(int(data.eq_active[equality_id]), 0)
            before_z = float(data.qpos[qpos_address + 2])
            for _ in range(5):
                mujoco.mj_step(model, data)
            self.assertLess(float(data.qpos[qpos_address + 2]), before_z)
            self.assertTrue(math.isfinite(float(data.qpos[qpos_address + 2])))

    @unittest.skipIf(mujoco is None, "MuJoCo Python bindings are unavailable")
    def test_pool_exhaustion_is_explicit(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            mjcf, assets, catalog = self._fixture(root, pool_size=1)
            INJECT.inject_catalog(mjcf, assets, catalog)
            model = mujoco.MjModel.from_xml_path(str(mjcf))
            data = mujoco.MjData(model)
            runtime = RUNTIME.CreativeInventoryRuntime(
                _Simulator(model, data), catalog
            )
            runtime.spawn("test_prop", _Pose())
            with self.assertRaisesRegex(RUNTIME.CreativeInventoryError, "no unused"):
                runtime.spawn("test_prop", _Pose())


if __name__ == "__main__":
    unittest.main()
