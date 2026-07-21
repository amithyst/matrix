#!/usr/bin/env python3
"""Fail-closed provenance checks for optional Matrix policy-slot candidates.

This module deliberately stops at registration and admission.  It does not
start a policy process, grant a LowCmd lease, or define an adapter for a model
whose runtime writer ABI has not been implemented and reviewed.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
from typing import Mapping


MANIFEST_SCHEMA = "matrix.locomotion-policy-candidate.v1"
ADAPTER_PROTOCOL = "matrix.locomotion-policy-writer.v1"
BFM_TEACHER50K_POLICY_ID = "bfm-sonic-teacher50k"
BFM_TEACHER50K_DISPLAY_NAME = "BFM SONIC Teacher50k"
_SHA256_RE = re.compile(r"[0-9a-f]{64}\Z")
_GIT_COMMIT_RE = re.compile(r"[0-9a-f]{40}\Z")
_ENV_NAME_RE = re.compile(r"[A-Z][A-Z0-9_]{1,127}\Z")
_POLICY_ID_RE = re.compile(r"[a-z0-9][a-z0-9._-]{0,63}\Z")


class PolicyCandidateError(ValueError):
    """Raised when a policy-candidate manifest violates its strict schema."""


@dataclass(frozen=True)
class ArtifactRequirement:
    name: str
    path_env: str
    sha256: str | None
    executable: bool = False


@dataclass(frozen=True)
class SourceRequirement:
    repository: str
    commit: str
    root_env: str


@dataclass(frozen=True)
class PolicyCandidateManifest:
    policy_id: str
    display_name: str
    slot: str
    backend: str
    source: SourceRequirement
    artifacts: tuple[ArtifactRequirement, ...]
    manifest_path: Path
    manifest_sha256: str
    checkpoint_path_hint: str | None
    adapter_protocol: str
    decoder_input_dim: int
    token_dim: int
    deployable_proprio_dim: int
    compatibility_zero_dim: int
    action_dim: int
    proxy99_exact_zero: bool

    def provenance_mapping(self) -> dict[str, object]:
        return {
            "manifest": os.fspath(self.manifest_path),
            "manifest_sha256": self.manifest_sha256,
            "repository": self.source.repository,
            "source_commit": self.source.commit,
            "checkpoint_path_hint": self.checkpoint_path_hint,
            "artifacts": {
                artifact.name: {
                    "path_env": artifact.path_env,
                    "sha256": artifact.sha256,
                }
                for artifact in self.artifacts
            },
            "adapter_protocol": self.adapter_protocol,
            "decoder_contract": {
                "input_dim": self.decoder_input_dim,
                "token_dim": self.token_dim,
                "deployable_proprio_dim": self.deployable_proprio_dim,
                "compatibility_zero_dim": self.compatibility_zero_dim,
                "action_dim": self.action_dim,
                "proxy99_exact_zero": self.proxy99_exact_zero,
            },
        }


@dataclass(frozen=True)
class PolicyCandidateState:
    policy_id: str
    display_name: str
    slot: str
    resident: bool
    available: bool
    provenance_verified: bool
    unavailable_reasons: tuple[str, ...]
    provenance: dict[str, object]

    @property
    def unavailable_reason(self) -> str | None:
        return self.unavailable_reasons[0] if self.unavailable_reasons else None

    def to_mapping(self) -> dict[str, object]:
        return {
            "policy_id": self.policy_id,
            "name": self.display_name,
            "resident": self.resident,
            "available": self.available,
            "provenance_verified": self.provenance_verified,
            "unavailable_reason": self.unavailable_reason,
            "unavailable_reasons": list(self.unavailable_reasons),
            "provenance": self.provenance,
        }


def _exact_keys(value: object, expected: set[str], context: str) -> dict[str, object]:
    if not isinstance(value, dict) or set(value) != expected:
        raise PolicyCandidateError(
            f"{context} must contain exactly: {', '.join(sorted(expected))}"
        )
    return value


def _nonempty_string(value: object, context: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise PolicyCandidateError(f"{context} must be a non-empty string")
    return value.strip()


def _sha256_or_none(value: object, context: str) -> str | None:
    if value is None:
        return None
    if not isinstance(value, str) or _SHA256_RE.fullmatch(value) is None:
        raise PolicyCandidateError(f"{context} must be null or a lowercase SHA256")
    return value


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_policy_candidate_manifest(path: Path) -> PolicyCandidateManifest:
    resolved = path.resolve()
    try:
        raw_bytes = resolved.read_bytes()
        payload = json.loads(raw_bytes.decode("utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise PolicyCandidateError(f"cannot read policy manifest: {exc}") from exc

    root = _exact_keys(
        payload,
        {
            "schema",
            "policy_id",
            "display_name",
            "slot",
            "backend",
            "source",
            "artifacts",
            "runtime_contract",
            "evidence",
        },
        "manifest",
    )
    if root["schema"] != MANIFEST_SCHEMA:
        raise PolicyCandidateError(f"unsupported policy manifest: {root['schema']!r}")
    policy_id = _nonempty_string(root["policy_id"], "policy_id")
    if _POLICY_ID_RE.fullmatch(policy_id) is None:
        raise PolicyCandidateError("policy_id is invalid")
    slot = _nonempty_string(root["slot"], "slot")
    if slot != "locomotion":
        raise PolicyCandidateError("BFM policy candidate must use the locomotion slot")

    source_raw = _exact_keys(
        root["source"],
        {
            "repository",
            "commit",
            "main_at_handoff",
            "root_env",
            "checkpoint_path_hint",
        },
        "source",
    )
    commit = _nonempty_string(source_raw["commit"], "source.commit")
    if _GIT_COMMIT_RE.fullmatch(commit) is None:
        raise PolicyCandidateError("source.commit must be a full lowercase Git SHA")
    main_at_handoff = _nonempty_string(
        source_raw["main_at_handoff"], "source.main_at_handoff"
    )
    if _GIT_COMMIT_RE.fullmatch(main_at_handoff) is None:
        raise PolicyCandidateError(
            "source.main_at_handoff must be a full lowercase Git SHA"
        )
    root_env = _nonempty_string(source_raw["root_env"], "source.root_env")
    if _ENV_NAME_RE.fullmatch(root_env) is None:
        raise PolicyCandidateError("source.root_env is invalid")
    checkpoint_path_hint_raw = source_raw["checkpoint_path_hint"]
    checkpoint_path_hint = (
        _nonempty_string(checkpoint_path_hint_raw, "source.checkpoint_path_hint")
        if checkpoint_path_hint_raw is not None
        else None
    )

    artifacts_raw = root["artifacts"]
    if not isinstance(artifacts_raw, list) or len(artifacts_raw) != 3:
        raise PolicyCandidateError("artifacts must contain exactly three entries")
    artifacts: list[ArtifactRequirement] = []
    for index, raw in enumerate(artifacts_raw):
        entry = _exact_keys(
            raw,
            {"name", "path_env", "sha256", "executable"},
            f"artifacts[{index}]",
        )
        name = _nonempty_string(entry["name"], f"artifacts[{index}].name")
        path_env = _nonempty_string(
            entry["path_env"], f"artifacts[{index}].path_env"
        )
        if _ENV_NAME_RE.fullmatch(path_env) is None:
            raise PolicyCandidateError(f"artifacts[{index}].path_env is invalid")
        if type(entry["executable"]) is not bool:
            raise PolicyCandidateError(
                f"artifacts[{index}].executable must be boolean"
            )
        artifacts.append(
            ArtifactRequirement(
                name=name,
                path_env=path_env,
                sha256=_sha256_or_none(
                    entry["sha256"], f"artifacts[{index}].sha256"
                ),
                executable=entry["executable"],
            )
        )
    if {artifact.name for artifact in artifacts} != {
        "checkpoint",
        "config",
        "runtime_adapter",
    }:
        raise PolicyCandidateError(
            "artifacts must be checkpoint, config, and runtime_adapter"
        )
    if sum(artifact.executable for artifact in artifacts) != 1 or not next(
        artifact for artifact in artifacts if artifact.name == "runtime_adapter"
    ).executable:
        raise PolicyCandidateError("only runtime_adapter may be executable")

    contract = _exact_keys(
        root["runtime_contract"],
        {
            "adapter_protocol",
            "decoder_input_dim",
            "token_dim",
            "deployable_proprio_dim",
            "compatibility_zero_dim",
            "action_dim",
            "proxy99_exact_zero",
        },
        "runtime_contract",
    )
    expected_contract: dict[str, object] = {
        "adapter_protocol": ADAPTER_PROTOCOL,
        "decoder_input_dim": 1093,
        "token_dim": 64,
        "deployable_proprio_dim": 930,
        "compatibility_zero_dim": 99,
        "action_dim": 29,
        "proxy99_exact_zero": True,
    }
    if contract != expected_contract:
        raise PolicyCandidateError("runtime_contract does not match the frozen BFM ABI")
    _exact_keys(
        root["evidence"],
        {
            "first_core_document_id",
            "second_core_mirror_id",
            "checkpoint_contract_commit",
            "checkpoint_contract_path",
            "runtime_contract_commit",
            "runtime_contract_path",
        },
        "evidence",
    )

    return PolicyCandidateManifest(
        policy_id=policy_id,
        display_name=_nonempty_string(root["display_name"], "display_name"),
        slot=slot,
        backend=_nonempty_string(root["backend"], "backend"),
        source=SourceRequirement(
            repository=_nonempty_string(source_raw["repository"], "source.repository"),
            commit=commit,
            root_env=root_env,
        ),
        artifacts=tuple(artifacts),
        manifest_path=resolved,
        manifest_sha256=hashlib.sha256(raw_bytes).hexdigest(),
        checkpoint_path_hint=checkpoint_path_hint,
        adapter_protocol=str(contract["adapter_protocol"]),
        decoder_input_dim=int(contract["decoder_input_dim"]),
        token_dim=int(contract["token_dim"]),
        deployable_proprio_dim=int(contract["deployable_proprio_dim"]),
        compatibility_zero_dim=int(contract["compatibility_zero_dim"]),
        action_dim=int(contract["action_dim"]),
        proxy99_exact_zero=bool(contract["proxy99_exact_zero"]),
    )


def _locked_manifest_sha256(
    runtime_lock_path: Path,
    manifest_path: Path,
    *,
    project_root: Path,
) -> str | None:
    try:
        lock = json.loads(runtime_lock_path.read_text(encoding="utf-8"))
        relative = manifest_path.resolve().relative_to(project_root.resolve()).as_posix()
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError):
        return None
    policy_slots = lock.get("policy_slots") if isinstance(lock, dict) else None
    manifests = policy_slots.get("manifests") if isinstance(policy_slots, dict) else None
    if not isinstance(manifests, list):
        return None
    matches = [
        entry
        for entry in manifests
        if isinstance(entry, dict) and entry.get("path") == relative
    ]
    if len(matches) != 1:
        return None
    digest = matches[0].get("sha256")
    return digest if isinstance(digest, str) and _SHA256_RE.fullmatch(digest) else None


def _source_checkout_failures(
    requirement: SourceRequirement,
    environment: Mapping[str, str],
) -> list[str]:
    configured = environment.get(requirement.root_env, "").strip()
    if not configured:
        return [f"missing_source_env:{requirement.root_env}"]
    root = Path(configured)
    if not root.is_absolute():
        return [f"source_path_not_absolute:{requirement.root_env}"]
    if not root.is_dir():
        return [f"source_checkout_missing:{requirement.root_env}"]
    command_env = dict(os.environ)
    command_env.update(environment)
    command_env["GIT_OPTIONAL_LOCKS"] = "0"

    def git_value(*arguments: str) -> tuple[int, str]:
        try:
            result = subprocess.run(
                ("git", "-C", os.fspath(root), *arguments),
                check=False,
                capture_output=True,
                text=True,
                timeout=5.0,
                env=command_env,
            )
        except (OSError, subprocess.TimeoutExpired):
            return 1, ""
        return result.returncode, result.stdout.strip()

    failures: list[str] = []
    code, head = git_value("rev-parse", "HEAD")
    if code != 0:
        failures.append("source_checkout_not_git")
    elif head != requirement.commit:
        failures.append("source_commit_mismatch")
    code, remote = git_value("config", "--get", "remote.origin.url")
    if code != 0 or remote != requirement.repository:
        failures.append("source_repository_mismatch")
    code, dirty = git_value("status", "--porcelain", "--untracked-files=no")
    if code != 0:
        failures.append("source_status_unavailable")
    elif dirty:
        failures.append("source_checkout_dirty")
    return failures


def evaluate_policy_candidate(
    manifest_path: Path,
    runtime_lock_path: Path,
    *,
    project_root: Path,
    environment: Mapping[str, str] | None = None,
) -> PolicyCandidateState:
    """Validate provenance without starting or authorizing the candidate."""

    env = os.environ if environment is None else environment
    try:
        manifest = load_policy_candidate_manifest(manifest_path)
    except PolicyCandidateError as exc:
        return PolicyCandidateState(
            policy_id=BFM_TEACHER50K_POLICY_ID,
            display_name=BFM_TEACHER50K_DISPLAY_NAME,
            slot="locomotion",
            resident=False,
            available=False,
            provenance_verified=False,
            unavailable_reasons=(f"invalid_manifest:{exc}",),
            provenance={"manifest": os.fspath(manifest_path.resolve())},
        )

    failures: list[str] = []
    locked_sha256 = _locked_manifest_sha256(
        runtime_lock_path,
        manifest.manifest_path,
        project_root=project_root,
    )
    if locked_sha256 is None:
        failures.append("manifest_not_runtime_locked")
    elif locked_sha256 != manifest.manifest_sha256:
        failures.append("manifest_lock_sha256_mismatch")

    for artifact in manifest.artifacts:
        if artifact.sha256 is None:
            failures.append(f"artifact_sha256_unlocked:{artifact.name}")
    failures.extend(_source_checkout_failures(manifest.source, env))
    for artifact in manifest.artifacts:
        configured = env.get(artifact.path_env, "").strip()
        if not configured:
            failures.append(f"missing_artifact_env:{artifact.name}:{artifact.path_env}")
            continue
        path = Path(configured)
        if not path.is_absolute():
            failures.append(f"artifact_path_not_absolute:{artifact.name}")
            continue
        if not path.is_file():
            failures.append(f"artifact_missing:{artifact.name}")
            continue
        if artifact.sha256 is not None:
            try:
                actual_sha256 = _file_sha256(path)
            except OSError:
                failures.append(f"artifact_unreadable:{artifact.name}")
            else:
                if actual_sha256 != artifact.sha256:
                    failures.append(f"artifact_sha256_mismatch:{artifact.name}")
        if artifact.executable and not os.access(path, os.X_OK):
            failures.append(f"artifact_not_executable:{artifact.name}")

    provenance_verified = not failures
    # A verified artifact set is still not a writer registration.  Admission
    # remains closed until a future reviewed adapter owns a resident session
    # and participates in the existing single-writer hand-off protocol.
    failures.append("runtime_adapter_not_registered")
    return PolicyCandidateState(
        policy_id=manifest.policy_id,
        display_name=manifest.display_name,
        slot=manifest.slot,
        resident=False,
        available=False,
        provenance_verified=provenance_verified,
        unavailable_reasons=tuple(failures),
        provenance=manifest.provenance_mapping(),
    )
