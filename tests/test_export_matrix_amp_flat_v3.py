import hashlib
from pathlib import Path
import sys
import tempfile
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
import export_matrix_amp_flat_v3 as exporter  # noqa: E402


def base_config():
    return {
        "policy_joint_names": list(exporter.G1_29_JOINT_NAMES),
        "obs_config": {
            "history_length": 4,
            "policy": [
                {"name": name}
                for name in exporter.OBSERVATION_ORDER
            ],
        },
        "obs_joint_pos_relative": True,
        "default_joint_pos": [0.0] * 29,
        "action_scale": [0.25] * 29,
        "stiffness": [20.0] * 29,
        "damping": [1.0] * 29,
        "armature": [0.01] * 29,
        "action_clip": 10.0,
        "sim": {"control_dt": 0.02},
        "sim2sim_alignment_notes": {"source_task": "legacy"},
    }


class FlatV3ExporterTests(unittest.TestCase):
    def test_sha256_validation_uses_exact_file_bytes(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "checkpoint.pt"
            path.write_bytes(b"flat-v3-checkpoint")
            expected = hashlib.sha256(path.read_bytes()).hexdigest()
            self.assertEqual(
                exporter.require_sha256(path, expected, "checkpoint"),
                expected,
            )
            with self.assertRaisesRegex(ValueError, "mismatch"):
                exporter.require_sha256(path, "0" * 64, "checkpoint")

    def test_deployment_config_records_normalized_actor_provenance(self):
        source = base_config()
        result = exporter.build_deployment_config(
            source,
            model_filename="flat_v3_m14000.onnx",
            model_sha256="1" * 64,
            checkpoint_path=Path("/artifacts/flat_v3_m14000.pt"),
            checkpoint_sha256="2" * 64,
            source_root=Path("/source/amprecovery"),
            source_commit=exporter.SOURCE_COMMIT,
            source_repository="ssh://git.example/amprecovery.git",
            base_config_sha256="3" * 64,
        )

        self.assertNotIn("onnx", source)
        self.assertEqual(result["matrix_provenance"]["policy_id"], "amp-flat-v3")
        self.assertEqual(
            result["matrix_provenance"]["actor_widths"],
            [384, 512, 256, 128, 29],
        )
        self.assertEqual(
            result["matrix_provenance"]["observation_normalizer"]["formula"],
            "(obs - mean) / (std + 0.01)",
        )
        self.assertTrue(result["onnx"]["includes_observation_normalizer"])
        self.assertEqual(result["onnx"]["input_width"], 384)
        self.assertEqual(result["onnx"]["output_width"], 29)

    def test_base_config_rejects_joint_order_and_zero_armature(self):
        wrong_order = base_config()
        wrong_order["policy_joint_names"] = list(
            reversed(wrong_order["policy_joint_names"])
        )
        with self.assertRaisesRegex(ValueError, "joint order"):
            exporter.validate_base_config(wrong_order)

        zero_armature = base_config()
        zero_armature["armature"][4] = 0.0
        with self.assertRaisesRegex(ValueError, "armature values"):
            exporter.validate_base_config(zero_armature)


if __name__ == "__main__":
    unittest.main()
