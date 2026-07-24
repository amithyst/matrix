from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "matrix_creative_prop_bridge.py"
SPEC = importlib.util.spec_from_file_location(
    "matrix_creative_prop_bridge",
    SCRIPT_PATH,
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class CreativePropBridgeTest(unittest.TestCase):
    def test_missing_packaged_ue_consumer_is_unavailable(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            capability = MODULE.detect_creative_prop_visual_bridge(
                Path(temporary),
                render_sync_enabled=True,
            )

        self.assertFalse(capability.available)
        mapping = capability.mapping()
        self.assertEqual(
            mapping["reason"],
            "packaged_ue_creative_prop_consumer_missing",
        )
        self.assertIn("canonical G1 robot state", mapping["evidence"][1])

    def test_packaged_ue_manifest_without_runtime_transport_still_blocks(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest = root / MODULE.DEFAULT_CAPABILITY_RELATIVE_PATH
            manifest.parent.mkdir(parents=True)
            manifest.write_text(
                json.dumps(
                    {
                        "schema": MODULE.CAPABILITY_SCHEMA,
                        "bridge_id": MODULE.BRIDGE_ID,
                        "transport": "packaged-ue-native-consumer",
                        "consumer": "Matrix packaged UE",
                        "transform_units": "meters,wxyz",
                    }
                ),
                encoding="utf-8",
            )

            capability = MODULE.detect_creative_prop_visual_bridge(
                root,
                render_sync_enabled=True,
            )

        self.assertFalse(capability.available)
        self.assertEqual(
            capability.mapping()["reason"],
            "runtime_creative_prop_transform_transport_unimplemented",
        )

    def test_render_sync_disabled_blocks_visibility(self) -> None:
        capability = MODULE.detect_creative_prop_visual_bridge(
            Path("/unused"),
            render_sync_enabled=False,
        )

        self.assertFalse(capability.available)
        self.assertEqual(capability.mapping()["reason"], "render_sync_disabled")


if __name__ == "__main__":
    unittest.main()
