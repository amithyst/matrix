from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts/matrix_policy_slots.py"
SPEC = importlib.util.spec_from_file_location("matrix_policy_slots", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)

MANIFEST = REPO_ROOT / "config/runtime/policy-slots/bfm-sonic-teacher50k.json"
RUNTIME_LOCK = REPO_ROOT / "config/runtime/matrix-sonic.lock.json"


class MatrixPolicySlotsTest(unittest.TestCase):
    def test_teacher50k_manifest_freezes_full_provenance_and_contract(self) -> None:
        manifest = MODULE.load_policy_candidate_manifest(MANIFEST)

        self.assertEqual(manifest.policy_id, "bfm-sonic-teacher50k")
        self.assertEqual(
            manifest.source.commit,
            "5e264ae2bee2315dc0522c48c64b4506977b2e25",
        )
        self.assertEqual(
            {artifact.name: artifact.sha256 for artifact in manifest.artifacts},
            {
                "checkpoint": "613274ee5956db59b8f2509c408619ee600e7b847c4609ad1643e6c7ffd410c1",
                "config": "e7bed95642a3627cc6f6cff416da784fe2d0841b697d0f34e7039fd73af10e3f",
                "teacher_onnx": "edbec19062d6c34621dd97df864c596d29937432d8a019dd949d03785d9cdc45",
                "runtime_adapter": "3375ffc19f68cc5a2be5541712af656620dcbff27ff6c7209573ac1b395a4dae",
                "g1_xml": "8c586e4747da85804180fe44d8692e0fd8231356728b6327e256dca498087a78",
                "formal_ik": "c8776f1e7651a4f179ea75e17b9746c41fa77a15be2cacf5809fe648340a7ab2",
            },
        )
        self.assertEqual(
            (
                manifest.model_input_dim,
                manifest.tokenizer_dim,
                manifest.command_dim,
                manifest.height_map_dim,
                manifest.orientation_dim,
                manifest.actor_observation_dim,
                manifest.history_length,
                manifest.compatibility_zero_dim,
                manifest.action_dim,
                manifest.action_clip,
                manifest.activation_blend_seconds,
                manifest.activation_contract,
                manifest.standby_history_contract,
                manifest.turn_reference_contract,
                manifest.turn_reference_forward_mps,
                manifest.command_heading_contract,
                manifest.command_yaw_gain,
                manifest.command_yaw_limit_rad_s,
                manifest.turn_command_yaw_limit_rad_s,
                manifest.turn_command_yaw_damping_seconds,
                manifest.proxy99_exact_zero,
            ),
            (
                1790,
                761,
                580,
                121,
                60,
                1029,
                10,
                99,
                29,
                20.0,
                0.1,
                "current-lowstate-smoothstep-no-teleport",
                "repeat-current-frame-zero-unapplied-actions",
                "yaw-only-pfnn-forward-seed-v1",
                0.00051,
                "matrix-wire-facing-formal7168-pd-v2",
                4.0,
                1.5,
                0.6,
                0.1,
                True,
            ),
        )
        self.assertEqual(
            manifest.trees[0].sha256,
            "d1d0a7255a2f8898e81522570a09a3b56624fd7b955a2d7d02b87800f47585cb",
        )

    def test_missing_runtime_is_visible_but_fails_closed_without_processes(self) -> None:
        with mock.patch.object(MODULE.subprocess, "run") as run:
            state = MODULE.evaluate_policy_candidate(
                MANIFEST,
                RUNTIME_LOCK,
                project_root=REPO_ROOT,
                environment={},
            )

        run.assert_not_called()
        self.assertFalse(state.available)
        self.assertFalse(state.resident)
        self.assertFalse(state.provenance_verified)
        self.assertEqual(
            state.unavailable_reason,
            "missing_source_env:MATRIX_BFM_SONIC_SOURCE_ROOT",
        )
        self.assertIn(
            "missing_artifact_env:checkpoint:MATRIX_BFM_SONIC_CHECKPOINT",
            state.unavailable_reasons,
        )
        self.assertIn(
            "missing_tree_env:pfnn_weights:MATRIX_BFM_SONIC_PFNN_WEIGHTS",
            state.unavailable_reasons,
        )

    def test_abbreviated_source_hash_is_rejected(self) -> None:
        payload = json.loads(MANIFEST.read_text(encoding="utf-8"))
        payload["source"]["commit"] = "5e264ae2"
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "candidate.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(
                MODULE.PolicyCandidateError,
                "full lowercase Git SHA",
            ):
                MODULE.load_policy_candidate_manifest(path)


if __name__ == "__main__":
    unittest.main()
