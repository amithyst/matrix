#!/usr/bin/env python3
"""Shared ONNX execution-provider and residency helpers for Matrix policies."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Callable, Mapping


CPU_PROVIDER = "CPUExecutionProvider"
CUDA_PROVIDER = "CUDAExecutionProvider"


@dataclass
class ResidentPolicyAdapter:
    """Uniform runtime contract for one preloaded recovery controller."""

    policy_id: str
    controller: str
    execution_provider: str
    command_config: Any
    start_episode_fn: Callable[[Any, float], None]
    infer_target_fn: Callable[[Any, float], Any]
    status_fields_fn: Callable[[float, float], Mapping[str, Any]]
    started_monotonic: float | None = field(default=None, init=False)

    def start_episode(self, state: Any, now_s: float) -> None:
        now = float(now_s)
        if not math.isfinite(now):
            raise ValueError("policy start time must be finite")
        self.start_episode_fn(state, now)
        self.started_monotonic = now

    def infer_target(self, state: Any, now_s: float) -> Any:
        if self.started_monotonic is None:
            raise RuntimeError(f"resident policy {self.policy_id!r} was not started")
        return self.infer_target_fn(state, float(now_s))

    def status_fields(self, now_s: float) -> dict[str, Any]:
        if self.started_monotonic is None:
            raise RuntimeError(f"resident policy {self.policy_id!r} was not started")
        return dict(self.status_fields_fn(float(now_s), self.started_monotonic))


class ResidentPolicyRegistry:
    """Validated policy-id/controller dispatch for resident policy workers."""

    def __init__(self, execution_provider: str) -> None:
        self.execution_provider = str(execution_provider)
        if not self.execution_provider:
            raise ValueError("resident policy execution provider cannot be empty")
        self._by_policy_id: dict[str, ResidentPolicyAdapter] = {}
        self._by_controller: dict[str, ResidentPolicyAdapter] = {}

    @staticmethod
    def _validate_policy_id(policy_id: str) -> str:
        value = str(policy_id).strip().lower()
        if (
            not value
            or value[0] not in "abcdefghijklmnopqrstuvwxyz0123456789"
            or any(character not in "abcdefghijklmnopqrstuvwxyz0123456789._-" for character in value)
        ):
            raise ValueError(f"invalid resident policy id: {policy_id!r}")
        return value

    def register(self, adapter: ResidentPolicyAdapter) -> None:
        policy_id = self._validate_policy_id(adapter.policy_id)
        controller = str(adapter.controller).strip()
        if not controller:
            raise ValueError("resident policy controller cannot be empty")
        if adapter.execution_provider != self.execution_provider:
            raise ValueError(
                "resident policy provider mismatch: "
                f"registry={self.execution_provider!r} "
                f"policy={adapter.execution_provider!r}"
            )
        if policy_id in self._by_policy_id:
            raise ValueError(f"duplicate resident policy id: {policy_id!r}")
        if controller in self._by_controller:
            raise ValueError(f"duplicate resident controller: {controller!r}")
        adapter.policy_id = policy_id
        self._by_policy_id[policy_id] = adapter
        self._by_controller[controller] = adapter

    def require(self, policy_id: str) -> ResidentPolicyAdapter:
        normalized = self._validate_policy_id(policy_id)
        try:
            return self._by_policy_id[normalized]
        except KeyError as exc:
            raise ValueError(
                f"resident policy {normalized!r} is not registered; "
                f"available={self.policy_ids!r}"
            ) from exc

    def for_controller(self, controller: str) -> ResidentPolicyAdapter | None:
        return self._by_controller.get(str(controller))

    @property
    def policy_ids(self) -> tuple[str, ...]:
        return tuple(self._by_policy_id)


def execution_providers(ort: Any, requested: str) -> list[str]:
    """Resolve one explicit provider without silently falling back."""

    mode = str(requested).strip().lower()
    provider = (
        CUDA_PROVIDER
        if mode == "cuda"
        else CPU_PROVIDER if mode == "cpu" else None
    )
    if provider is None:
        raise ValueError("execution provider must be 'cuda' or 'cpu'")
    available = tuple(str(item) for item in ort.get_available_providers())
    if provider not in available:
        raise RuntimeError(
            f"requested ONNX provider {provider} is unavailable; "
            f"available={available!r}"
        )
    # One provider prevents silent CPU placement when GPU residency is required.
    return [provider]


def create_inference_session(
    ort: Any,
    model_path: str,
    requested: str,
) -> tuple[Any, str]:
    """Create one policy session with CUDA CPU fallback explicitly disabled."""

    provider = requested_provider_name(requested)
    session_options = ort.SessionOptions()
    if provider == CUDA_PROVIDER:
        # ORT still reports CPUExecutionProvider as a registered secondary EP
        # even when only CUDA is requested. This setting makes graph placement
        # fail instead of silently assigning unsupported nodes to that CPU EP.
        session_options.add_session_config_entry(
            "session.disable_cpu_ep_fallback",
            "1",
        )
    session = ort.InferenceSession(
        str(model_path),
        sess_options=session_options,
        providers=execution_providers(ort, requested),
    )
    return session, session_provider(session, requested)


def session_provider(session: Any, requested: str) -> str:
    """Attest the selected primary EP after strict session construction."""

    expected = requested_provider_name(requested)
    providers = tuple(str(item) for item in session.get_providers())
    if not providers or providers[0] != expected:
        raise RuntimeError(
            "policy session did not select the requested primary provider: "
            f"requested={expected!r} active={providers!r}"
        )
    return expected


def requested_provider_name(requested: str) -> str:
    mode = str(requested).strip().lower()
    if mode == "cuda":
        return CUDA_PROVIDER
    if mode == "cpu":
        return CPU_PROVIDER
    raise ValueError("execution provider must be 'cuda' or 'cpu'")
