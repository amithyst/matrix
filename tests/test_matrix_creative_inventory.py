from __future__ import annotations

from dataclasses import replace
import importlib.util
import hashlib
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
PACK = sys.modules["matrix_item_asset_pack"]


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

    def test_item_asset_pack_inventory_resolves_into_existing_runtime_dto(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source_mesh = root / "source.stl"
            _write_tetrahedron(source_mesh)
            payload = source_mesh.read_bytes()
            pack_document = {
                "schema": "matrix-item-asset-pack/v1",
                "pack": {
                    "pack_id": "benchmark.test-props",
                    "revision": "v1",
                    "license": {"spdx_id": "CC0-1.0", "attribution": ""},
                    "provenance": {
                        "source_name": "Test benchmark",
                        "source_uri": "https://example.invalid/props",
                        "source_revision": "v1",
                        "source_item_ids": ["prop-1"],
                    },
                    "coordinate_frame": {
                        "up_axis": "+Z",
                        "forward_axis": "+X",
                        "handedness": "right",
                        "meters_per_unit": 1.0,
                    },
                    "files": [
                        {
                            "file_id": "mesh",
                            "path": "meshes/prop.stl",
                            "size_bytes": len(payload),
                            "sha256": hashlib.sha256(payload).hexdigest(),
                            "role": "visual_mesh",
                            "media_type": "model/stl",
                            "format": "stl",
                        }
                    ],
                    "items": [
                        {
                            "item_id": "prop",
                            "label": "Benchmark prop",
                            "physics": {
                                "mass_kg": 1.2,
                                "collision": {
                                    "shape": "box",
                                    "half_extents_m": [0.1, 0.05, 0.05],
                                },
                            },
                            "visual_parts": [
                                {
                                    "part_id": "body",
                                    "file_id": "mesh",
                                    "rgba": [0.8, 0.2, 0.1, 1.0],
                                    "scale": [1.0, 1.0, 1.0],
                                    "translation_m": [0.0, 0.0, 0.0],
                                    "rotation_wxyz": [1.0, 0.0, 0.0, 0.0],
                                }
                            ],
                        }
                    ],
                },
            }
            digest = PACK.asset_pack_digest(pack_document)
            pack_root = root / "registry/sha256" / digest[:2] / digest
            (pack_root / "meshes").mkdir(parents=True)
            (pack_root / "meshes/prop.stl").write_bytes(payload)
            (pack_root / "matrix-item-asset-pack.json").write_text(
                json.dumps(pack_document),
                encoding="utf-8",
            )
            inventory = root / "inventory.json"
            inventory.write_text(
                json.dumps(
                    {
                        "schema": "matrix-item-inventory/v1",
                        "inventory": {
                            "inventory_id": "test",
                            "entries": [
                                {
                                    "slot_id": "benchmark_prop",
                                    "pack_digest": f"sha256:{digest}",
                                    "item_id": "prop",
                                    "pool_size": 3,
                                    "spawn": {
                                        "distance_m": 1.5,
                                        "height_m": 0.8,
                                        "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                                    },
                                }
                            ],
                        },
                    }
                ),
                encoding="utf-8",
            )

            items = INJECT.load_catalog(
                inventory,
                item_pack_root=root / "registry",
            )

            self.assertEqual(len(items), 1)
            self.assertEqual(items[0].item_id, "benchmark_prop")
            self.assertEqual(items[0].pool_size, 3)
            self.assertEqual(items[0].visuals[0].mesh.read_bytes(), payload)

    def test_generic_pack_adapter_preserves_runtime_safety_bounds(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            mesh = Path(temporary_directory) / "item.stl"
            _write_tetrahedron(mesh)
            visual = PACK.LegacyInjectorVisualSpec(
                mesh=mesh,
                rgba=(0.8, 0.2, 0.1, 1.0),
                scale=(1.0, 1.0, 1.0),
            )
            baseline = PACK.LegacyInjectorItemSpec(
                item_id="benchmark_prop",
                label="Benchmark prop",
                pool_size=3,
                mass_kg=1.2,
                collision_half_size=(0.1, 0.05, 0.05),
                spawn_distance_m=1.5,
                spawn_height_m=0.8,
                spawn_quat=(1.0, 0.0, 0.0, 0.0),
                visuals=(visual,),
            )
            mutations = (
                replace(baseline, item_id="benchmark.prop"),
                replace(baseline, label="x" * 41),
                replace(baseline, mass_kg=0.001),
                replace(baseline, mass_kg=101.0),
                replace(baseline, collision_half_size=(5.1, 0.1, 0.1)),
                replace(baseline, spawn_distance_m=0.1),
                replace(baseline, spawn_height_m=4.0),
                replace(
                    baseline,
                    visuals=(
                        replace(visual, scale=(101.0, 1.0, 1.0)),
                    ),
                ),
                replace(baseline, visuals=(visual,) * 17),
            )
            for candidate in mutations:
                with self.subTest(candidate=candidate), self.assertRaises(
                    INJECT.InventoryCatalogError
                ):
                    INJECT._runtime_item_from_resolved(candidate)

            accepted = INJECT._runtime_item_from_resolved(baseline)
            self.assertEqual(accepted.item_id, "benchmark_prop")
            self.assertEqual(accepted.mass_kg, 1.2)

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
            pool_entries = runtime.pools["test_prop"]
            for entry in pool_entries:
                self.assertEqual(int(model.body_contype[entry.body_id]), 0)
                self.assertEqual(int(model.body_conaffinity[entry.body_id]), 0)
                self.assertEqual(int(model.geom_contype[entry.collision_geom_id]), 0)
                self.assertEqual(
                    int(model.geom_conaffinity[entry.collision_geom_id]), 0
                )
            inactive_qpos = [
                tuple(
                    float(value)
                    for value in data.qpos[
                        entry.qpos_address : entry.qpos_address + 7
                    ]
                )
                for entry in pool_entries
            ]
            for _ in range(8):
                mujoco.mj_step(model, data)
            for entry, expected_qpos in zip(
                pool_entries, inactive_qpos, strict=True
            ):
                actual_qpos = tuple(
                    float(value)
                    for value in data.qpos[
                        entry.qpos_address : entry.qpos_address + 7
                    ]
                )
                self.assertLess(
                    max(
                        abs(actual - expected)
                        for actual, expected in zip(
                            actual_qpos, expected_qpos, strict=True
                        )
                    ),
                    1e-3,
                )
                self.assertTrue(
                    all(
                        abs(float(value)) < 0.02
                        for value in data.qvel[
                            entry.dof_address : entry.dof_address + 6
                        ]
                    )
                )
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
            collision_geom_id = mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_GEOM,
                "creative_item__test_prop__0__collision",
            )
            spawned_entry = pool_entries[0]
            self.assertEqual(int(model.body_contype[spawned_entry.body_id]), 1)
            self.assertEqual(int(model.body_conaffinity[spawned_entry.body_id]), 1)
            self.assertEqual(int(model.geom_contype[collision_geom_id]), 1)
            self.assertEqual(int(model.geom_conaffinity[collision_geom_id]), 1)
            equality_id = mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_EQUALITY,
                "creative_item__test_prop__0__storage_weld",
            )
            self.assertEqual(int(data.eq_active[equality_id]), 0)
            before_z = float(data.qpos[qpos_address + 2])
            floor_geom_id = mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_GEOM,
                "floor",
            )
            prop_touched_floor = False
            for _ in range(600):
                mujoco.mj_step(model, data)
                prop_touched_floor = prop_touched_floor or any(
                    {
                        int(data.contact[contact_index].geom1),
                        int(data.contact[contact_index].geom2),
                    }
                    == {floor_geom_id, collision_geom_id}
                    for contact_index in range(data.ncon)
                )
            self.assertLess(float(data.qpos[qpos_address + 2]), before_z)
            self.assertTrue(math.isfinite(float(data.qpos[qpos_address + 2])))
            self.assertTrue(prop_touched_floor)
            self.assertAlmostEqual(float(data.qpos[qpos_address + 2]), 0.05, delta=0.02)

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
