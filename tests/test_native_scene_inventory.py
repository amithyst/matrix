from __future__ import annotations

import copy
import importlib.util
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = PROJECT_ROOT / "scripts/validate_native_scene_inventory.py"
SPEC = importlib.util.spec_from_file_location("validate_native_scene_inventory", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)


class NativeSceneInventoryTest(unittest.TestCase):
    def setUp(self) -> None:
        self.inventory = AUDIT.load_json(AUDIT.DEFAULT_INVENTORY)
        self.launcher = AUDIT.parse_launcher_scenes(AUDIT.DEFAULT_LAUNCHER)

    def test_committed_inventory_matches_launcher(self) -> None:
        self.assertEqual(AUDIT.validate_inventory(self.inventory, self.launcher), [])

    def test_scene_count_drift_is_reported(self) -> None:
        modified = copy.deepcopy(self.inventory)
        modified["counts"]["curated_selectable_scenes"] = 19
        errors = AUDIT.validate_inventory(modified, self.launcher)
        self.assertTrue(any("curated_selectable_scenes" in error for error in errors))

    def test_3dgs_aliases_are_explicit(self) -> None:
        scene = next(
            item
            for item in self.inventory["selectable_scenes"]
            if item["package_name"] == "3DGSWorld"
        )
        self.assertEqual(scene["launcher_ids"], [14, 16, 17])

    def test_every_native_scene_has_a_complete_physics_proxy_count(self) -> None:
        for scene in self.inventory["selectable_scenes"]:
            proxy = scene["physics_proxy"]
            self.assertEqual(proxy["geom_count"], sum(proxy["geom_types"].values()))


if __name__ == "__main__":
    unittest.main()
