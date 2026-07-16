from __future__ import annotations

import json
from pathlib import Path
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]


class UrbanV1ContractTest(unittest.TestCase):
    def test_contract_matches_native_town10_inventory(self) -> None:
        contract = json.loads(
            (REPO_ROOT / "research" / "urban_v1" / "scene.json").read_text(
                encoding="utf-8"
            )
        )
        inventory = json.loads(
            (
                REPO_ROOT
                / "research"
                / "sonic_integration"
                / "native_scenes.json"
            ).read_text(encoding="utf-8")
        )
        town10 = next(
            scene
            for scene in inventory["selectable_scenes"]
            if scene["package_name"] == "Town10World"
        )

        visual = contract["visual_source"]
        physics = contract["physics_source"]
        self.assertIn(visual["launcher_scene_id"], town10["launcher_ids"])
        self.assertEqual(visual["package_name"], town10["package_name"])
        self.assertEqual(visual["ue_map"], town10["ue_map"])
        self.assertEqual(visual["release_sha256"], town10["release_package"]["sha256"])
        self.assertEqual(physics["mujoco_scene"], town10["mujoco_scene"])
        self.assertEqual(
            physics["environment_geom_count"],
            town10["physics_proxy"]["geom_count"],
        )
        self.assertEqual(
            physics["dynamic_environment_body_count"],
            town10["physics_proxy"]["dynamic_body_count"],
        )

    def test_launcher_fixes_scene_two_and_rejects_override(self) -> None:
        launcher = (
            REPO_ROOT / "scripts" / "run_matrix_sonic_urban_v1.sh"
        ).read_text(encoding="utf-8")
        self.assertIn('run_matrix_sonic.sh" --scene 2', launcher)
        self.assertIn('argument" == "--scene"', launcher)
        self.assertIn("people and vehicles are UE presentation assets", launcher)


if __name__ == "__main__":
    unittest.main()
