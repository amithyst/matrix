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
                "runtime_adapter": None,
            },
        )
        self.assertEqual(
            (
                manifest.decoder_input_dim,
                manifest.token_dim,
                manifest.deployable_proprio_dim,
                manifest.compatibility_zero_dim,
                manifest.action_dim,
                manifest.proxy99_exact_zero,
            ),
            (1093, 64, 930, 99, 29, True),
        )

    def test_unregistered_adapter_is_visible_but_fails_closed_without_processes(self) -> None:
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
            "artifact_sha256_unlocked:runtime_adapter",
        )
        self.assertIn("runtime_adapter_not_registered", state.unavailable_reasons)
        self.assertIn(
            "missing_artifact_env:checkpoint:MATRIX_BFM_SONIC_CHECKPOINT",
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
