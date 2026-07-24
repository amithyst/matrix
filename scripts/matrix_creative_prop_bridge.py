#!/usr/bin/env python3
"""Capability probe for visible creative-prop rendering in Matrix UE.

MuJoCo remains the physics authority for creative inventory props.  A spawned
prop is truthful only when the UE runtime also has an explicit consumer for
named prop transforms; otherwise the operator would see an inventory decrement
with no visible object.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path


CAPABILITY_SCHEMA = "matrix-creative-prop-visual-bridge/v1"
BRIDGE_ID = "matrix-creative-prop-named-transform-v1"
DEFAULT_CAPABILITY_RELATIVE_PATH = Path(
    "Content/MatrixCreativePropBridge/creative-prop-bridge.json"
)


class CreativePropBridgeError(ValueError):
    """Raised when a creative prop visual capability marker is malformed."""


@dataclass(frozen=True)
class CreativePropVisualBridgeCapability:
    available: bool
    mode: str
    reason: str | None
    manifest: str | None
    render_sync_enabled: bool
    evidence: tuple[str, ...]

    def mapping(self) -> dict[str, object]:
        return {
            "schema": CAPABILITY_SCHEMA,
            "bridge_id": BRIDGE_ID,
            "available": self.available,
            "mode": self.mode,
            "reason": self.reason,
            "manifest": self.manifest,
            "render_sync_enabled": self.render_sync_enabled,
            "evidence": list(self.evidence),
        }


def _reject_duplicate_keys(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise CreativePropBridgeError(f"duplicate JSON key {key!r}")
        result[key] = value
    return result


def _load_manifest(path: Path) -> dict[str, object]:
    if not path.is_file() or path.is_symlink():
        raise CreativePropBridgeError("manifest is not a regular file")
    try:
        payload = json.loads(
            path.read_text(encoding="utf-8"),
            object_pairs_hook=_reject_duplicate_keys,
        )
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise CreativePropBridgeError(f"cannot read manifest: {exc}") from exc
    if not isinstance(payload, dict):
        raise CreativePropBridgeError("manifest root must be an object")
    expected_keys = {
        "schema",
        "bridge_id",
        "transport",
        "consumer",
        "transform_units",
    }
    if set(payload) != expected_keys:
        raise CreativePropBridgeError(
            "manifest keys differ from schema: "
            f"missing={sorted(expected_keys - set(payload))} "
            f"extra={sorted(set(payload) - expected_keys)}"
        )
    if payload["schema"] != CAPABILITY_SCHEMA:
        raise CreativePropBridgeError("manifest schema is unsupported")
    if payload["bridge_id"] != BRIDGE_ID:
        raise CreativePropBridgeError("manifest bridge_id is unsupported")
    if payload["transport"] != "packaged-ue-native-consumer":
        raise CreativePropBridgeError("manifest transport is unsupported")
    if payload["consumer"] != "Matrix packaged UE":
        raise CreativePropBridgeError("manifest consumer is unsupported")
    if payload["transform_units"] != "meters,wxyz":
        raise CreativePropBridgeError("manifest transform_units are unsupported")
    return payload


def detect_creative_prop_visual_bridge(
    ue_runtime_root: Path,
    *,
    render_sync_enabled: bool,
    manifest_path: Path | None = None,
) -> CreativePropVisualBridgeCapability:
    """Return the currently provable UE creative-prop visual capability."""

    if not render_sync_enabled:
        return CreativePropVisualBridgeCapability(
            available=False,
            mode="unavailable",
            reason="render_sync_disabled",
            manifest=None,
            render_sync_enabled=False,
            evidence=(
                "creative inventory props cannot be visible when UE render sync is disabled",
            ),
        )

    root = Path(ue_runtime_root)
    manifest = manifest_path or root / DEFAULT_CAPABILITY_RELATIVE_PATH
    try:
        payload = _load_manifest(manifest)
    except CreativePropBridgeError as exc:
        return CreativePropVisualBridgeCapability(
            available=False,
            mode="unavailable",
            reason="packaged_ue_creative_prop_consumer_missing",
            manifest=str(manifest),
            render_sync_enabled=True,
            evidence=(
                f"no valid creative prop bridge manifest: {exc}",
                "current render protocol publishes only canonical G1 robot state",
            ),
        )

    transport = payload["transport"]
    return CreativePropVisualBridgeCapability(
        available=False,
        mode="unavailable",
        reason="runtime_creative_prop_transform_transport_unimplemented",
        manifest=str(manifest),
        render_sync_enabled=True,
        evidence=(
            f"validated packaged UE manifest requests {transport}",
            "runtime has no approved named-transform transport for creative props",
        ),
    )


__all__ = [
    "BRIDGE_ID",
    "CAPABILITY_SCHEMA",
    "CreativePropBridgeError",
    "CreativePropVisualBridgeCapability",
    "DEFAULT_CAPABILITY_RELATIVE_PATH",
    "detect_creative_prop_visual_bridge",
]
