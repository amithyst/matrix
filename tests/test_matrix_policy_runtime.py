from __future__ import annotations

import unittest

from matrix_policy_runtime import (
    ResidentPolicyAdapter,
    ResidentPolicyRegistry,
    create_inference_session,
)


class FakeSessionOptions:
    def __init__(self):
        self.entries = {}

    def add_session_config_entry(self, key, value):
        self.entries[key] = value


class FakeSession:
    def __init__(self, providers):
        self._providers = list(providers)

    def get_providers(self):
        return list(self._providers)


class FakeOrt:
    def __init__(self, *, available, active):
        self.available = list(available)
        self.active = list(active)
        self.options = None
        self.model_path = None
        self.requested = None

    def get_available_providers(self):
        return list(self.available)

    def SessionOptions(self):
        self.options = FakeSessionOptions()
        return self.options

    def InferenceSession(self, model_path, *, sess_options, providers):
        self.model_path = model_path
        self.options = sess_options
        self.requested = list(providers)
        return FakeSession(self.active)


class PolicyRuntimeTests(unittest.TestCase):
    def test_resident_policy_registry_dispatches_arbitrary_policy_id(self):
        starts = []
        registry = ResidentPolicyRegistry("CUDAExecutionProvider")
        adapter = ResidentPolicyAdapter(
            policy_id="future_getup",
            controller="FUTURE_GETUP",
            execution_provider="CUDAExecutionProvider",
            command_config={"kp": 100.0},
            start_episode_fn=lambda state, now: starts.append((state, now)),
            infer_target_fn=lambda state, now: (state, now),
            status_fields_fn=lambda now, started: {
                "policy_elapsed_s": now - started
            },
        )
        registry.register(adapter)

        selected = registry.require("future_getup")
        self.assertIs(selected, adapter)
        self.assertIs(registry.for_controller("FUTURE_GETUP"), adapter)
        self.assertEqual(registry.policy_ids, ("future_getup",))
        selected.start_episode("lowstate", 1.0)
        self.assertEqual(starts, [("lowstate", 1.0)])
        self.assertEqual(selected.infer_target("next", 1.1), ("next", 1.1))
        self.assertAlmostEqual(selected.status_fields(1.2)["policy_elapsed_s"], 0.2)

    def test_resident_policy_registry_rejects_duplicate_or_mixed_provider(self):
        def adapter(policy_id, controller, provider="CUDAExecutionProvider"):
            return ResidentPolicyAdapter(
                policy_id=policy_id,
                controller=controller,
                execution_provider=provider,
                command_config=None,
                start_episode_fn=lambda _state, _now: None,
                infer_target_fn=lambda _state, _now: None,
                status_fields_fn=lambda _now, _started: {},
            )

        registry = ResidentPolicyRegistry("CUDAExecutionProvider")
        registry.register(adapter("first", "FIRST"))
        with self.assertRaisesRegex(ValueError, "duplicate resident policy id"):
            registry.register(adapter("first", "SECOND"))
        with self.assertRaisesRegex(ValueError, "duplicate resident controller"):
            registry.register(adapter("second", "FIRST"))
        with self.assertRaisesRegex(ValueError, "provider mismatch"):
            registry.register(adapter("cpu", "CPU", "CPUExecutionProvider"))

    def test_cuda_disables_cpu_fallback_and_accepts_reported_secondary_cpu(self):
        ort = FakeOrt(
            available=("CUDAExecutionProvider", "CPUExecutionProvider"),
            active=("CUDAExecutionProvider", "CPUExecutionProvider"),
        )
        _session, provider = create_inference_session(ort, "policy.onnx", "cuda")

        self.assertEqual(provider, "CUDAExecutionProvider")
        self.assertEqual(ort.requested, ["CUDAExecutionProvider"])
        self.assertEqual(
            ort.options.entries,
            {"session.disable_cpu_ep_fallback": "1"},
        )

    def test_cpu_session_does_not_set_cuda_fallback_option(self):
        ort = FakeOrt(
            available=("CPUExecutionProvider",),
            active=("CPUExecutionProvider",),
        )
        _session, provider = create_inference_session(ort, "policy.onnx", "cpu")

        self.assertEqual(provider, "CPUExecutionProvider")
        self.assertEqual(ort.requested, ["CPUExecutionProvider"])
        self.assertEqual(ort.options.entries, {})

    def test_missing_or_misselected_cuda_provider_fails_closed(self):
        unavailable = FakeOrt(
            available=("CPUExecutionProvider",),
            active=("CPUExecutionProvider",),
        )
        with self.assertRaisesRegex(RuntimeError, "unavailable"):
            create_inference_session(unavailable, "policy.onnx", "cuda")

        wrong_primary = FakeOrt(
            available=("CUDAExecutionProvider", "CPUExecutionProvider"),
            active=("CPUExecutionProvider",),
        )
        with self.assertRaisesRegex(RuntimeError, "requested primary"):
            create_inference_session(wrong_primary, "policy.onnx", "cuda")


if __name__ == "__main__":
    unittest.main()
