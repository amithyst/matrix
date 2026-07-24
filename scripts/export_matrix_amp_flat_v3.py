#!/usr/bin/env python3
"""Export the provenance-locked AMP flat_v3 checkpoint for Matrix.

The upstream checkpoint stores a 384-wide actor and an empirical observation
normalizer separately.  Matrix consumes a single ONNX graph, so this exporter
reconstructs the exact ELU actor, embeds ``(obs - mean) / (std + 0.01)`` in the
graph, validates ONNX Runtime parity, and emits a deployment config derived
from an already-qualified G1 AMP sim2sim config.

Checkpoint loading is deliberately ``weights_only=True``.  The legacy
``amp_normalizer`` object is allow-listed only so PyTorch can traverse the
checkpoint; it is never executed or used to build the exported policy.
"""

from __future__ import annotations

import argparse
import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from typing import Any, Mapping, Sequence


OBSERVATION_WIDTH = 384
ACTION_WIDTH = 29
ACTOR_WIDTHS = (OBSERVATION_WIDTH, 512, 256, 128, ACTION_WIDTH)
ACTOR_LINEAR_INDICES = (0, 2, 4, 6)
NORMALIZER_EPSILON = 0.01
ONNX_OPSET = 18
POLICY_ID = "amp-flat-v3"
SOURCE_COMMIT = "87a3b8cae853e4a6b7d233fe0779be90d7f4cfc5"
CHECKPOINT_SHA256 = (
    "574e52f8d91aed356a2cedf4f24af2485466daab8ddec2d69d6cc3590fb2df4c"
)
G1_29_JOINT_NAMES = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)
OBSERVATION_ORDER = (
    "RootAngVelB",
    "ProjectedGravityB",
    "Command",
    "JointPos",
    "JointVel",
    "PrevActions",
)


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def require_sha256(path: Path, expected: str, label: str) -> str:
    normalized = str(expected).strip().lower()
    if len(normalized) != 64 or any(
        character not in "0123456789abcdef" for character in normalized
    ):
        raise ValueError(f"{label} SHA256 must be 64 lowercase hex characters")
    actual = file_sha256(path)
    if actual != normalized:
        raise ValueError(
            f"{label} SHA256 mismatch: expected={normalized} actual={actual}"
        )
    return actual


def verify_source_revision(source_root: Path, expected_commit: str) -> str:
    root = source_root.resolve()
    if not root.is_dir():
        raise ValueError(f"source root is missing: {root}")
    commit = subprocess.run(
        ("git", "-C", os.fspath(root), "rev-parse", "HEAD"),
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    if commit != expected_commit:
        raise ValueError(
            f"source commit mismatch: expected={expected_commit} actual={commit}"
        )
    dirty = subprocess.run(
        ("git", "-C", os.fspath(root), "status", "--porcelain"),
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    if dirty:
        raise ValueError(f"source snapshot is dirty: {root}")
    return commit


def _finite_vector(value: Any, size: int, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != size:
        raise ValueError(f"{label} must contain exactly {size} values")
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{label} contains a non-finite value")
    return result


def validate_base_config(config: Mapping[str, Any]) -> None:
    if tuple(config.get("policy_joint_names", ())) != G1_29_JOINT_NAMES:
        raise ValueError("base config G1 joint order does not match Matrix DDS")
    obs_config = config.get("obs_config")
    if not isinstance(obs_config, Mapping):
        raise ValueError("base config obs_config must be an object")
    if int(obs_config.get("history_length", 0)) != 4:
        raise ValueError("flat_v3 requires observation history_length=4")
    policies = obs_config.get("policy", ())
    if not isinstance(policies, list):
        raise ValueError("base config observation policy must be a list")
    actual_order = tuple(
        item.get("name") if isinstance(item, Mapping) else None
        for item in policies
    )
    if actual_order != OBSERVATION_ORDER:
        raise ValueError(f"unsupported observation order: {actual_order!r}")
    if config.get("obs_joint_pos_relative") is not True:
        raise ValueError("flat_v3 requires relative joint-position observations")
    sim = config.get("sim")
    if not isinstance(sim, Mapping) or not math.isclose(
        float(sim.get("control_dt", 0.0)),
        0.02,
        rel_tol=0.0,
        abs_tol=1e-9,
    ):
        raise ValueError("flat_v3 requires a 50 Hz control contract")
    for label in (
        "default_joint_pos",
        "action_scale",
        "stiffness",
        "damping",
        "armature",
    ):
        values = _finite_vector(config.get(label), ACTION_WIDTH, label)
        if label in {"action_scale", "stiffness", "damping", "armature"} and any(
            value <= 0.0 for value in values
        ):
            raise ValueError(f"{label} values must be positive")
    action_clip = float(config.get("action_clip", 0.0))
    if not math.isfinite(action_clip) or action_clip <= 0.0:
        raise ValueError("action_clip must be finite and positive")


def build_deployment_config(
    base_config: Mapping[str, Any],
    *,
    model_filename: str,
    model_sha256: str,
    checkpoint_path: Path,
    checkpoint_sha256: str,
    source_root: Path,
    source_commit: str,
    source_repository: str,
    base_config_sha256: str,
) -> dict[str, Any]:
    validate_base_config(base_config)
    result = json.loads(json.dumps(base_config))
    result["onnx"] = {
        "file": model_filename,
        "sha256": model_sha256,
        "input_name": "obs",
        "output_name": "actions",
        "input_width": OBSERVATION_WIDTH,
        "output_width": ACTION_WIDTH,
        "opset": ONNX_OPSET,
        "dynamic_batch": True,
        "single_file": True,
        "includes_observation_normalizer": True,
        "observation_normalizer_epsilon": NORMALIZER_EPSILON,
    }
    notes = result.setdefault("sim2sim_alignment_notes", {})
    if not isinstance(notes, dict):
        raise ValueError("sim2sim_alignment_notes must be an object")
    notes.update(
        {
            "source_task": "Unitree-G1-AMP-Flat",
            "model_source": os.fspath(checkpoint_path.resolve()),
            "model_checkpoint_iteration": 14000,
            "onnx_export": (
                "Matrix exporter reconstructs the exact ELU actor and embeds "
                "the checkpoint obs_norm_state_dict with eps=0.01."
            ),
        }
    )
    result["matrix_provenance"] = {
        "policy_id": POLICY_ID,
        "display_name": "AMP flat_v3 m14000",
        "slot": "recovery",
        "source_repository": source_repository,
        "source_commit": source_commit,
        "source_root": os.fspath(source_root.resolve()),
        "checkpoint": os.fspath(checkpoint_path.resolve()),
        "checkpoint_sha256": checkpoint_sha256,
        "base_config_sha256": base_config_sha256,
        "actor_widths": list(ACTOR_WIDTHS),
        "activation": "elu",
        "observation_normalizer": {
            "state_dict_key": "obs_norm_state_dict",
            "formula": "(obs - mean) / (std + 0.01)",
        },
    }
    return result


def _load_checkpoint_weights_only(torch: Any, numpy: Any, path: Path) -> Any:
    """Load a legacy checkpoint without allowing arbitrary pickle execution."""

    import types

    original_modules = {
        name: sys.modules.get(name)
        for name in ("rsl_rl", "rsl_rl.utils", "rsl_rl.utils.utils")
    }
    rsl_module = types.ModuleType("rsl_rl")
    utils_package = types.ModuleType("rsl_rl.utils")
    utils_module = types.ModuleType("rsl_rl.utils.utils")
    legacy_normalizer = type("Normalizer", (), {})
    legacy_normalizer.__module__ = "rsl_rl.utils.utils"
    utils_module.Normalizer = legacy_normalizer
    sys.modules["rsl_rl"] = rsl_module
    sys.modules["rsl_rl.utils"] = utils_package
    sys.modules["rsl_rl.utils.utils"] = utils_module
    try:
        reconstruct = numpy.core.multiarray._reconstruct
        safe = [
            legacy_normalizer,
            reconstruct,
            numpy.ndarray,
            numpy.dtype,
            type(numpy.dtype(numpy.float64)),
        ]
        with torch.serialization.safe_globals(safe):
            return torch.load(path, map_location="cpu", weights_only=True)
    finally:
        for name, previous in original_modules.items():
            if previous is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = previous


def _validate_checkpoint(torch: Any, checkpoint: Any) -> tuple[Any, Any, Any]:
    if not isinstance(checkpoint, Mapping):
        raise ValueError("checkpoint must be a mapping")
    iteration = checkpoint.get("iter")
    if type(iteration) is not int or iteration != 14000:
        raise ValueError(f"checkpoint iteration must be 14000, got {iteration!r}")
    model_state = checkpoint.get("model_state_dict")
    normalizer_state = checkpoint.get("obs_norm_state_dict")
    if not isinstance(model_state, Mapping) or not isinstance(
        normalizer_state, Mapping
    ):
        raise ValueError("checkpoint is missing model/normalizer state dictionaries")

    expected_actor_keys: set[str] = set()
    for layer_index, input_width, output_width in zip(
        ACTOR_LINEAR_INDICES,
        ACTOR_WIDTHS[:-1],
        ACTOR_WIDTHS[1:],
    ):
        weight_key = f"actor.{layer_index}.weight"
        bias_key = f"actor.{layer_index}.bias"
        expected_actor_keys.update((weight_key, bias_key))
        weight = model_state.get(weight_key)
        bias = model_state.get(bias_key)
        if not isinstance(weight, torch.Tensor) or tuple(weight.shape) != (
            output_width,
            input_width,
        ):
            raise ValueError(
                f"{weight_key} must have shape {(output_width, input_width)}"
            )
        if not isinstance(bias, torch.Tensor) or tuple(bias.shape) != (
            output_width,
        ):
            raise ValueError(f"{bias_key} must have shape {(output_width,)}")
        if not torch.isfinite(weight).all() or not torch.isfinite(bias).all():
            raise ValueError(f"{weight_key}/{bias_key} contains non-finite values")
    actual_actor_keys = {
        str(key) for key in model_state if str(key).startswith("actor.")
    }
    if actual_actor_keys != expected_actor_keys:
        raise ValueError(
            "checkpoint actor keys drifted: "
            f"expected={sorted(expected_actor_keys)!r} "
            f"actual={sorted(actual_actor_keys)!r}"
        )

    mean = normalizer_state.get("_mean")
    std = normalizer_state.get("_std")
    count = normalizer_state.get("count")
    for label, value in (("_mean", mean), ("_std", std)):
        if not isinstance(value, torch.Tensor) or tuple(value.shape) != (
            1,
            OBSERVATION_WIDTH,
        ):
            raise ValueError(
                f"obs_norm_state_dict[{label!r}] must have shape "
                f"(1, {OBSERVATION_WIDTH})"
            )
        if not torch.isfinite(value).all():
            raise ValueError(f"normalizer {label} contains non-finite values")
    # The three commanded-twist inputs are always zero in every one of the
    # four history frames, so their learned standard deviations are exactly
    # zero.  Upstream deliberately handles those dimensions through eps=0.01.
    if bool(torch.any(std < 0.0)):
        raise ValueError("normalizer standard deviations must be non-negative")
    if not isinstance(count, torch.Tensor) or count.numel() != 1:
        raise ValueError("normalizer count must be one scalar tensor")
    return model_state, mean, std


def _build_policy_module(torch: Any, model_state: Mapping[str, Any], mean: Any, std: Any) -> Any:
    class NormalizedActor(torch.nn.Module):
        def __init__(self) -> None:
            super().__init__()
            self.register_buffer("obs_mean", mean.detach().clone().float())
            self.register_buffer("obs_std", std.detach().clone().float())
            layers: list[Any] = []
            for index, (input_width, output_width) in enumerate(
                zip(ACTOR_WIDTHS[:-1], ACTOR_WIDTHS[1:])
            ):
                layers.append(torch.nn.Linear(input_width, output_width))
                if index + 1 < len(ACTOR_WIDTHS) - 1:
                    layers.append(torch.nn.ELU())
            self.actor = torch.nn.Sequential(*layers)
            with torch.no_grad():
                for module_index in ACTOR_LINEAR_INDICES:
                    linear = self.actor[module_index]
                    linear.weight.copy_(
                        model_state[f"actor.{module_index}.weight"].float()
                    )
                    linear.bias.copy_(
                        model_state[f"actor.{module_index}.bias"].float()
                    )

        def forward(self, obs: Any) -> Any:
            normalized = (obs - self.obs_mean) / (
                self.obs_std + NORMALIZER_EPSILON
            )
            return self.actor(normalized)

    module = NormalizedActor().cpu().eval()
    with torch.no_grad():
        smoke = module(torch.zeros(2, OBSERVATION_WIDTH, dtype=torch.float32))
    if tuple(smoke.shape) != (2, ACTION_WIDTH) or not torch.isfinite(smoke).all():
        raise ValueError("reconstructed actor failed its finite shape smoke test")
    return module


def _single_file_export_kwargs(torch: Any) -> dict[str, bool]:
    try:
        parameters = inspect.signature(torch.onnx.export).parameters
    except (TypeError, ValueError):
        return {}
    if "external_data" in parameters:
        return {"external_data": False}
    if "use_external_data_format" in parameters:
        return {"use_external_data_format": False}
    return {}


def _atomic_json_write(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(value, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except Exception:
        try:
            os.close(descriptor)
        except OSError:
            pass
        Path(temporary).unlink(missing_ok=True)
        raise


def export_onnx(
    torch: Any,
    numpy: Any,
    onnx: Any,
    onnxruntime: Any,
    module: Any,
    output_path: Path,
    *,
    metadata: Mapping[str, str],
) -> tuple[str, float]:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output_path.name}.", suffix=".onnx", dir=output_path.parent
    )
    os.close(descriptor)
    temporary_path = Path(temporary_name)
    try:
        dummy = torch.zeros(1, OBSERVATION_WIDTH, dtype=torch.float32)
        torch.onnx.export(
            module,
            dummy,
            os.fspath(temporary_path),
            export_params=True,
            opset_version=ONNX_OPSET,
            input_names=("obs",),
            output_names=("actions",),
            dynamic_axes={"obs": {0: "batch"}, "actions": {0: "batch"}},
            **_single_file_export_kwargs(torch),
        )
        model = onnx.load(os.fspath(temporary_path), load_external_data=True)
        del model.metadata_props[:]
        for key, value in sorted(metadata.items()):
            entry = model.metadata_props.add()
            entry.key = str(key)
            entry.value = str(value)
        onnx.checker.check_model(model)
        onnx.save_model(
            model,
            os.fspath(temporary_path),
            save_as_external_data=False,
        )
        external_data = Path(f"{temporary_path}.data")
        if external_data.exists():
            raise ValueError("ONNX export unexpectedly produced external data")

        session = onnxruntime.InferenceSession(
            os.fspath(temporary_path),
            providers=("CPUExecutionProvider",),
        )
        inputs = session.get_inputs()
        outputs = session.get_outputs()
        if (
            len(inputs) != 1
            or inputs[0].name != "obs"
            or inputs[0].shape[-1] != OBSERVATION_WIDTH
            or not outputs
            or outputs[0].name != "actions"
            or outputs[0].shape[-1] != ACTION_WIDTH
        ):
            raise ValueError("exported ONNX I/O contract is not obs[384] -> actions[29]")
        probes = numpy.random.default_rng(20260723).normal(
            size=(7, OBSERVATION_WIDTH)
        ).astype(numpy.float32)
        with torch.no_grad():
            expected = module(torch.from_numpy(probes)).numpy()
        actual = session.run(("actions",), {"obs": probes})[0]
        max_abs_error = float(numpy.max(numpy.abs(expected - actual)))
        if not math.isfinite(max_abs_error) or max_abs_error > 5e-5:
            raise ValueError(
                f"ONNX Runtime parity failed: max_abs_error={max_abs_error}"
            )
        os.replace(temporary_path, output_path)
        return file_sha256(output_path), max_abs_error
    finally:
        temporary_path.unlink(missing_ok=True)
        Path(f"{temporary_path}.data").unlink(missing_ok=True)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--base-config", type=Path, required=True)
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--output-model", type=Path, required=True)
    parser.add_argument("--output-config", type=Path, required=True)
    parser.add_argument(
        "--expected-checkpoint-sha256",
        default=CHECKPOINT_SHA256,
    )
    parser.add_argument("--expected-source-commit", default=SOURCE_COMMIT)
    parser.add_argument(
        "--source-repository",
        default=(
            "ssh://git@gitlab.xvirobotics.com:2222/"
            "shizhanxu/amprecovery_mjlab.git"
        ),
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    checkpoint = args.checkpoint.resolve()
    base_config_path = args.base_config.resolve()
    source_root = args.source_root.resolve()
    output_model = args.output_model.resolve()
    output_config = args.output_config.resolve()
    checkpoint_sha256 = require_sha256(
        checkpoint,
        args.expected_checkpoint_sha256,
        "flat_v3 checkpoint",
    )
    source_commit = verify_source_revision(
        source_root,
        args.expected_source_commit,
    )
    base_config_sha256 = file_sha256(base_config_path)
    with base_config_path.open("r", encoding="utf-8") as stream:
        base_config = json.load(stream)
    if not isinstance(base_config, dict):
        raise ValueError("base config must be a JSON object")
    validate_base_config(base_config)

    import numpy
    import onnx
    import onnxruntime
    import torch

    checkpoint_value = _load_checkpoint_weights_only(torch, numpy, checkpoint)
    model_state, mean, std = _validate_checkpoint(torch, checkpoint_value)
    module = _build_policy_module(torch, model_state, mean, std)
    model_sha256, max_abs_error = export_onnx(
        torch,
        numpy,
        onnx,
        onnxruntime,
        module,
        output_model,
        metadata={
            "matrix.policy_id": POLICY_ID,
            "matrix.source_commit": source_commit,
            "matrix.checkpoint_sha256": checkpoint_sha256,
            "matrix.actor_widths": ",".join(str(value) for value in ACTOR_WIDTHS),
            "matrix.activation": "elu",
            "matrix.observation_normalizer": "obs_norm_state_dict",
            "matrix.observation_normalizer_epsilon": str(NORMALIZER_EPSILON),
        },
    )
    deployment_config = build_deployment_config(
        base_config,
        model_filename=output_model.name,
        model_sha256=model_sha256,
        checkpoint_path=checkpoint,
        checkpoint_sha256=checkpoint_sha256,
        source_root=source_root,
        source_commit=source_commit,
        source_repository=args.source_repository,
        base_config_sha256=base_config_sha256,
    )
    _atomic_json_write(output_config, deployment_config)
    print(
        json.dumps(
            {
                "policy_id": POLICY_ID,
                "source_commit": source_commit,
                "checkpoint_sha256": checkpoint_sha256,
                "model": os.fspath(output_model),
                "model_sha256": model_sha256,
                "config": os.fspath(output_config),
                "config_sha256": file_sha256(output_config),
                "onnxruntime_max_abs_error": max_abs_error,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
