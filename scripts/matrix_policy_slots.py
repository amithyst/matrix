#!/usr/bin/env python3
"""Fail-closed provenance checks for optional Matrix policy-slot candidates."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
from typing import Mapping


MANIFEST_SCHEMA = "matrix.locomotion-policy-candidate.v2"
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
class TreeRequirement:
    name: str
    path_env: str
    sha256: str
    file_count: int


@dataclass(frozen=True)
class PolicyCandidateManifest:
    policy_id: str
    display_name: str
    slot: str
    backend: str
    source: SourceRequirement
    runtime_sources: tuple[SourceRequirement, ...]
    artifacts: tuple[ArtifactRequirement, ...]
    trees: tuple[TreeRequirement, ...]
    manifest_path: Path
    manifest_sha256: str
    checkpoint_path_hint: str | None
    adapter_protocol: str
    model_input_dim: int
    tokenizer_dim: int
    command_dim: int
    height_map_dim: int
    orientation_dim: int
    actor_observation_dim: int
    history_length: int
    compatibility_zero_dim: int
    action_dim: int
    action_clip: float
    activation_blend_seconds: float
    activation_contract: str
    standby_history_contract: str
    turn_reference_contract: str
    turn_reference_forward_mps: float
    command_heading_contract: str
    command_yaw_gain: float
    command_yaw_limit_rad_s: float
    turn_command_yaw_limit_rad_s: float
    turn_command_yaw_damping_seconds: float
    proxy99_exact_zero: bool

    def provenance_mapping(self) -> dict[str, object]:
        return {
            "manifest": os.fspath(self.manifest_path),
            "manifest_sha256": self.manifest_sha256,
            "repository": self.source.repository,
            "source_commit": self.source.commit,
            "runtime_sources": [
                {
                    "repository": source.repository,
                    "commit": source.commit,
                    "root_env": source.root_env,
                }
                for source in self.runtime_sources
            ],
            "checkpoint_path_hint": self.checkpoint_path_hint,
            "artifacts": {
                artifact.name: {
                    "path_env": artifact.path_env,
                    "sha256": artifact.sha256,
                }
                for artifact in self.artifacts
            },
            "trees": {
                tree.name: {
                    "path_env": tree.path_env,
                    "sha256": tree.sha256,
                    "file_count": tree.file_count,
                }
                for tree in self.trees
            },
            "adapter_protocol": self.adapter_protocol,
            "teacher_contract": {
                "model_input_dim": self.model_input_dim,
                "tokenizer_dim": self.tokenizer_dim,
                "command_dim": self.command_dim,
                "height_map_dim": self.height_map_dim,
                "orientation_dim": self.orientation_dim,
                "actor_observation_dim": self.actor_observation_dim,
                "history_length": self.history_length,
                "compatibility_zero_dim": self.compatibility_zero_dim,
                "action_dim": self.action_dim,
                "action_clip": self.action_clip,
                "activation_blend_seconds": self.activation_blend_seconds,
                "activation_contract": self.activation_contract,
                "standby_history_contract": self.standby_history_contract,
                "turn_reference_contract": self.turn_reference_contract,
                "turn_reference_forward_mps": self.turn_reference_forward_mps,
                "command_heading_contract": self.command_heading_contract,
                "command_yaw_gain": self.command_yaw_gain,
                "command_yaw_limit_rad_s": self.command_yaw_limit_rad_s,
                "turn_command_yaw_limit_rad_s": (
                    self.turn_command_yaw_limit_rad_s
                ),
                "turn_command_yaw_damping_seconds": (
                    self.turn_command_yaw_damping_seconds
                ),
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


def _directory_tree_sha256(path: Path) -> tuple[str, int]:
    files = sorted(candidate for candidate in path.rglob("*") if candidate.is_file())
    digest = hashlib.sha256()
    for candidate in files:
        relative = candidate.relative_to(path).as_posix()
        digest.update(
            f"{_file_sha256(candidate)}  ./{relative}\n".encode("utf-8")
        )
    return digest.hexdigest(), len(files)


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
            "runtime_sources",
            "artifacts",
            "trees",
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

    runtime_sources_raw = root["runtime_sources"]
    if not isinstance(runtime_sources_raw, list) or len(runtime_sources_raw) != 2:
        raise PolicyCandidateError(
            "runtime_sources must contain RealScan and Robo-PFNN"
        )
    runtime_sources: list[SourceRequirement] = []
    for index, raw in enumerate(runtime_sources_raw):
        entry = _exact_keys(
            raw,
            {"repository", "commit", "root_env"},
            f"runtime_sources[{index}]",
        )
        runtime_commit = _nonempty_string(
            entry["commit"], f"runtime_sources[{index}].commit"
        )
        if _GIT_COMMIT_RE.fullmatch(runtime_commit) is None:
            raise PolicyCandidateError(
                f"runtime_sources[{index}].commit must be a full lowercase Git SHA"
            )
        runtime_root_env = _nonempty_string(
            entry["root_env"], f"runtime_sources[{index}].root_env"
        )
        if _ENV_NAME_RE.fullmatch(runtime_root_env) is None:
            raise PolicyCandidateError(
                f"runtime_sources[{index}].root_env is invalid"
            )
        runtime_sources.append(
            SourceRequirement(
                repository=_nonempty_string(
                    entry["repository"],
                    f"runtime_sources[{index}].repository",
                ),
                commit=runtime_commit,
                root_env=runtime_root_env,
            )
        )

    artifacts_raw = root["artifacts"]
    if not isinstance(artifacts_raw, list) or len(artifacts_raw) != 6:
        raise PolicyCandidateError("artifacts must contain exactly six entries")
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
        "teacher_onnx",
        "runtime_adapter",
        "g1_xml",
        "formal_ik",
    }:
        raise PolicyCandidateError(
            "artifacts must lock checkpoint, config, Teacher ONNX, adapter, "
            "G1 XML, and formal IK"
        )
    if sum(artifact.executable for artifact in artifacts) != 1 or not next(
        artifact for artifact in artifacts if artifact.name == "runtime_adapter"
    ).executable:
        raise PolicyCandidateError("only runtime_adapter may be executable")

    trees_raw = root["trees"]
    if not isinstance(trees_raw, list) or len(trees_raw) != 1:
        raise PolicyCandidateError("trees must contain exactly pfnn_weights")
    tree_entry = _exact_keys(
        trees_raw[0],
        {"name", "path_env", "sha256", "file_count"},
        "trees[0]",
    )
    tree_name = _nonempty_string(tree_entry["name"], "trees[0].name")
    if tree_name != "pfnn_weights":
        raise PolicyCandidateError("trees[0].name must be pfnn_weights")
    tree_path_env = _nonempty_string(
        tree_entry["path_env"], "trees[0].path_env"
    )
    if _ENV_NAME_RE.fullmatch(tree_path_env) is None:
        raise PolicyCandidateError("trees[0].path_env is invalid")
    tree_sha256 = _sha256_or_none(tree_entry["sha256"], "trees[0].sha256")
    if tree_sha256 is None:
        raise PolicyCandidateError("trees[0].sha256 must be locked")
    tree_file_count = tree_entry["file_count"]
    if (
        type(tree_file_count) is not int
        or tree_file_count <= 0
        or tree_file_count > 100000
    ):
        raise PolicyCandidateError("trees[0].file_count is invalid")
    trees = (
        TreeRequirement(
            name=tree_name,
            path_env=tree_path_env,
            sha256=tree_sha256,
            file_count=tree_file_count,
        ),
    )

    contract = _exact_keys(
        root["runtime_contract"],
        {
            "adapter_protocol",
            "model_input_dim",
            "tokenizer_dim",
            "command_dim",
            "height_map_dim",
            "orientation_dim",
            "actor_observation_dim",
            "history_length",
            "compatibility_zero_dim",
            "action_dim",
            "action_clip",
            "activation_blend_seconds",
            "activation_contract",
            "standby_history_contract",
            "turn_reference_contract",
            "turn_reference_forward_mps",
            "command_heading_contract",
            "command_yaw_gain",
            "command_yaw_limit_rad_s",
            "turn_command_yaw_limit_rad_s",
            "turn_command_yaw_damping_seconds",
            "proxy99_exact_zero",
        },
        "runtime_contract",
    )
    expected_contract: dict[str, object] = {
        "adapter_protocol": ADAPTER_PROTOCOL,
        "model_input_dim": 1790,
        "tokenizer_dim": 761,
        "command_dim": 580,
        "height_map_dim": 121,
        "orientation_dim": 60,
        "actor_observation_dim": 1029,
        "history_length": 10,
        "compatibility_zero_dim": 99,
        "action_dim": 29,
        "action_clip": 20,
        "activation_blend_seconds": 0.1,
        "activation_contract": "current-lowstate-smoothstep-no-teleport",
        "standby_history_contract": (
            "repeat-current-frame-zero-unapplied-actions"
        ),
        "turn_reference_contract": "yaw-only-pfnn-forward-seed-v1",
        "turn_reference_forward_mps": 0.00051,
        "command_heading_contract": "matrix-wire-facing-formal7168-pd-v2",
        "command_yaw_gain": 4.0,
        "command_yaw_limit_rad_s": 1.5,
        "turn_command_yaw_limit_rad_s": 0.6,
        "turn_command_yaw_damping_seconds": 0.1,
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
        runtime_sources=tuple(runtime_sources),
        artifacts=tuple(artifacts),
        trees=trees,
        manifest_path=resolved,
        manifest_sha256=hashlib.sha256(raw_bytes).hexdigest(),
        checkpoint_path_hint=checkpoint_path_hint,
        adapter_protocol=str(contract["adapter_protocol"]),
        model_input_dim=int(contract["model_input_dim"]),
        tokenizer_dim=int(contract["tokenizer_dim"]),
        command_dim=int(contract["command_dim"]),
        height_map_dim=int(contract["height_map_dim"]),
        orientation_dim=int(contract["orientation_dim"]),
        actor_observation_dim=int(contract["actor_observation_dim"]),
        history_length=int(contract["history_length"]),
        compatibility_zero_dim=int(contract["compatibility_zero_dim"]),
        action_dim=int(contract["action_dim"]),
        action_clip=float(contract["action_clip"]),
        activation_blend_seconds=float(
            contract["activation_blend_seconds"]
        ),
        activation_contract=str(contract["activation_contract"]),
        standby_history_contract=str(contract["standby_history_contract"]),
        turn_reference_contract=str(contract["turn_reference_contract"]),
        turn_reference_forward_mps=float(
            contract["turn_reference_forward_mps"]
        ),
        command_heading_contract=str(contract["command_heading_contract"]),
        command_yaw_gain=float(contract["command_yaw_gain"]),
        command_yaw_limit_rad_s=float(
            contract["command_yaw_limit_rad_s"]
        ),
        turn_command_yaw_limit_rad_s=float(
            contract["turn_command_yaw_limit_rad_s"]
        ),
        turn_command_yaw_damping_seconds=float(
            contract["turn_command_yaw_damping_seconds"]
        ),
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
    for source in manifest.runtime_sources:
        failures.extend(_source_checkout_failures(source, env))
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

    for tree in manifest.trees:
        configured = env.get(tree.path_env, "").strip()
        if not configured:
            failures.append(f"missing_tree_env:{tree.name}:{tree.path_env}")
            continue
        path = Path(configured)
        if not path.is_absolute():
            failures.append(f"tree_path_not_absolute:{tree.name}")
            continue
        if not path.is_dir():
            failures.append(f"tree_missing:{tree.name}")
            continue
        try:
            actual_sha256, actual_count = _directory_tree_sha256(path)
        except OSError:
            failures.append(f"tree_unreadable:{tree.name}")
            continue
        if actual_count != tree.file_count:
            failures.append(f"tree_file_count_mismatch:{tree.name}")
        if actual_sha256 != tree.sha256:
            failures.append(f"tree_sha256_mismatch:{tree.name}")

    provenance_verified = not failures
    return PolicyCandidateState(
        policy_id=manifest.policy_id,
        display_name=manifest.display_name,
        slot=manifest.slot,
        resident=False,
        available=provenance_verified,
        provenance_verified=provenance_verified,
        unavailable_reasons=tuple(failures),
        provenance=manifest.provenance_mapping(),
    )
