#!/usr/bin/env python3
"""Convert a NuRec SH-Gaussian layer to the PLY schema used by Matrix UE.

This is an offline asset conversion tool.  Matrix does not depend on the NuRec
runtime after conversion: the output is the conventional 3DGS PLY imported by
the existing ThreeDGaussians Unreal plugin and cooked into a map package.
"""

from __future__ import annotations

import argparse
import gzip
import hashlib
import json
from pathlib import Path
import sys
import zipfile

try:
    import msgpack
    import numpy as np
except ImportError as exc:  # pragma: no cover - exercised by operator environments
    raise SystemExit(
        "convert_nurec_to_matrix_ply.py requires numpy and msgpack"
    ) from exc


SOURCE_SHA256 = "2b67231becf613036d4acdec796cffcad9ae3e2456dd311a96f8a00932df85cd"
NUREC_MEMBER = "model.nurec"
PREFIX = ".gaussians_nodes.gaussians."
EXPECTED_VERSION = "0.2.576"
# Recast scene-mesh AABB with a 0.5 m import margin.  This removes a small set
# of high-opacity reconstruction outliers hundreds of metres from the scan.
POSITION_MIN_M = (-29.59851265, -14.27500057, -2.54999995)
POSITION_MAX_M = (23.10065842, 46.42500305, 15.94401550)
MAX_LOG_SCALE = 0.0  # exp(0) = 1 m maximum principal scale
EXPECTED_FIELDS = {
    "positions": 3,
    "rotations": 4,
    "scales": 3,
    "densities": 1,
    "features_albedo": 3,
    "features_specular": 45,
}


class ConversionError(RuntimeError):
    """The source is not the locked NuRec Gaussian layout."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_nurec(usdz_path: Path) -> tuple[dict[str, object], dict[str, object]]:
    if not usdz_path.is_file() or usdz_path.is_symlink():
        raise ConversionError(f"source must be a regular non-symlink file: {usdz_path}")
    actual_hash = sha256_file(usdz_path)
    if actual_hash != SOURCE_SHA256:
        raise ConversionError(
            f"source SHA256 mismatch: expected {SOURCE_SHA256}, got {actual_hash}"
        )
    try:
        with zipfile.ZipFile(usdz_path) as archive:
            member = archive.getinfo(NUREC_MEMBER)
            if member.compress_type != zipfile.ZIP_STORED:
                raise ConversionError(
                    "model.nurec must remain byte-for-byte ZIP_STORED"
                )
            with archive.open(member) as raw, gzip.GzipFile(fileobj=raw) as unpacked:
                root = msgpack.unpack(unpacked, raw=False, strict_map_key=False)
    except (OSError, KeyError, zipfile.BadZipFile, msgpack.UnpackException) as exc:
        raise ConversionError(f"cannot decode locked NuRec asset: {exc}") from exc
    if not isinstance(root, dict) or set(root) != {"nre_data"}:
        raise ConversionError("unexpected NuRec root schema")
    nre_data = root["nre_data"]
    if not isinstance(nre_data, dict):
        raise ConversionError("nre_data must be an object")
    config = nre_data.get("config")
    state_dict = nre_data.get("state_dict")
    if (
        nre_data.get("version") != EXPECTED_VERSION
        or nre_data.get("model") != "nre"
        or not isinstance(config, dict)
        or not isinstance(state_dict, dict)
    ):
        raise ConversionError("unexpected NuRec model identity")
    layers = config.get("layers")
    gaussian_config = layers.get("gaussians") if isinstance(layers, dict) else None
    if not isinstance(gaussian_config, dict):
        raise ConversionError("NuRec Gaussian layer is missing")
    expected_activation = {
        "name": "sh-gaussians",
        "density_activation": "sigmoid",
        "scale_activation": "exp",
        "rotation_activation": "normalize",
        "precision": 16,
    }
    for key, expected in expected_activation.items():
        if gaussian_config.get(key) != expected:
            raise ConversionError(
                f"unsupported Gaussian {key}: {gaussian_config.get(key)!r}"
            )
    return nre_data, state_dict


def tensor_view(
    state_dict: dict[str, object], name: str, *, point_count: int | None = None
) -> "np.ndarray":
    key = PREFIX + name
    shape = state_dict.get(key + ".shape")
    payload = state_dict.get(key)
    if (
        not isinstance(shape, list)
        or len(shape) != 2
        or not all(type(value) is int and value >= 0 for value in shape)
        or not isinstance(payload, bytes)
    ):
        raise ConversionError(f"invalid tensor encoding for {name}")
    expected_width = EXPECTED_FIELDS[name]
    if shape[1] != expected_width:
        raise ConversionError(
            f"{name} must have width {expected_width}, got {shape[1]}"
        )
    if point_count is not None and shape[0] != point_count:
        raise ConversionError(f"{name} point count drifted")
    if len(payload) != shape[0] * shape[1] * 2:
        raise ConversionError(f"{name} payload size does not match float16 shape")
    array = np.frombuffer(payload, dtype="<f2").reshape(shape)
    if not np.isfinite(array).all():
        raise ConversionError(f"{name} contains non-finite values")
    return array


def select_indices(
    densities: "np.ndarray", eligible: "np.ndarray", max_points: int
) -> "np.ndarray":
    candidates = np.flatnonzero(eligible)
    if max_points <= 0 or max_points >= len(candidates):
        return candidates
    raw = densities[:, 0]
    candidate_density = raw[candidates]
    cutoff = np.partition(candidate_density, len(candidates) - max_points)[
        len(candidates) - max_points
    ]
    above = candidates[candidate_density > cutoff]
    equal = candidates[candidate_density == cutoff]
    remaining = max_points - len(above)
    if remaining < 0 or remaining > len(equal):
        raise ConversionError("cannot deterministically select opacity-ranked points")
    selected = np.concatenate((above, equal[:remaining]))
    selected.sort()
    if len(selected) != max_points:
        raise ConversionError("opacity selection produced the wrong point count")
    return selected


def ply_header(point_count: int) -> bytes:
    properties = ["x", "y", "z", "nx", "ny", "nz"]
    properties += [f"f_dc_{index}" for index in range(3)]
    properties += [f"f_rest_{index}" for index in range(45)]
    properties += ["opacity"]
    properties += [f"scale_{index}" for index in range(3)]
    properties += [f"rot_{index}" for index in range(4)]
    lines = [
        "ply",
        "format binary_little_endian 1.0",
        "comment Matrix native ThreeDGaussians import; converted from locked NuRec",
        f"element vertex {point_count}",
    ]
    lines.extend(f"property float {name}" for name in properties)
    lines.append("end_header")
    return ("\n".join(lines) + "\n").encode("ascii")


def write_ply(
    output_path: Path,
    tensors: dict[str, "np.ndarray"],
    indices: "np.ndarray",
    *,
    chunk_points: int,
) -> None:
    if output_path.exists() or output_path.is_symlink():
        raise ConversionError(f"refusing to overwrite output: {output_path}")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    field_names = (
        ["x", "y", "z", "nx", "ny", "nz"]
        + [f"f_dc_{i}" for i in range(3)]
        + [f"f_rest_{i}" for i in range(45)]
        + ["opacity"]
        + [f"scale_{i}" for i in range(3)]
        + [f"rot_{i}" for i in range(4)]
    )
    dtype = np.dtype([(name, "<f4") for name in field_names])
    with output_path.open("xb") as stream:
        stream.write(ply_header(len(indices)))
        for start in range(0, len(indices), chunk_points):
            selected = indices[start : start + chunk_points]
            block = np.zeros(len(selected), dtype=dtype)
            for offset, name in enumerate(("x", "y", "z")):
                block[name] = tensors["positions"][selected, offset]
            for offset in range(3):
                block[f"f_dc_{offset}"] = tensors["features_albedo"][selected, offset]
            specular = tensors["features_specular"][selected]
            for offset in range(45):
                block[f"f_rest_{offset}"] = specular[:, offset]
            block["opacity"] = tensors["densities"][selected, 0]
            for offset in range(3):
                block[f"scale_{offset}"] = tensors["scales"][selected, offset]
            for offset in range(4):
                block[f"rot_{offset}"] = tensors["rotations"][selected, offset]
            stream.write(block.tobytes(order="C"))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-usdz", type=Path, required=True)
    parser.add_argument("--output-ply", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--max-points", type=int, default=3_000_000)
    parser.add_argument("--chunk-points", type=int, default=50_000)
    args = parser.parse_args(argv)
    if args.max_points < 0 or args.chunk_points <= 0:
        raise SystemExit(
            "--max-points must be non-negative and --chunk-points positive"
        )
    if args.report.exists() or args.report.is_symlink():
        raise SystemExit(f"refusing to overwrite report: {args.report}")

    nre_data, state_dict = load_nurec(args.source_usdz)
    positions = tensor_view(state_dict, "positions")
    point_count = positions.shape[0]
    tensors = {"positions": positions}
    for name in EXPECTED_FIELDS:
        if name != "positions":
            tensors[name] = tensor_view(state_dict, name, point_count=point_count)
    position_min = np.asarray(POSITION_MIN_M, dtype=np.float32)
    position_max = np.asarray(POSITION_MAX_M, dtype=np.float32)
    eligible = (
        (tensors["positions"] >= position_min) & (tensors["positions"] <= position_max)
    ).all(axis=1)
    eligible &= tensors["scales"].max(axis=1) <= MAX_LOG_SCALE
    indices = select_indices(tensors["densities"], eligible, args.max_points)
    write_ply(
        args.output_ply,
        tensors,
        indices,
        chunk_points=args.chunk_points,
    )
    selected_density = tensors["densities"][indices, 0].astype(np.float32)
    report = {
        "schema": "matrix-realscan-nurec-to-ply/v1",
        "source_usdz_sha256": SOURCE_SHA256,
        "nurec_version": nre_data["version"],
        "source_point_count": point_count,
        "eligible_point_count": int(eligible.sum()),
        "output_point_count": len(indices),
        "selection": "locked_bounds_scale_then_stable_top_raw_density",
        "position_bounds_m": {
            "minimum": list(POSITION_MIN_M),
            "maximum": list(POSITION_MAX_M),
        },
        "maximum_log_scale": MAX_LOG_SCALE,
        "minimum_selected_raw_density": float(selected_density.min()),
        "minimum_selected_opacity": float(
            1.0 / (1.0 + np.exp(-selected_density.min()))
        ),
        "output_ply": str(args.output_ply.resolve()),
        "output_size_bytes": args.output_ply.stat().st_size,
        "output_sha256": sha256_file(args.output_ply),
        "coordinate_frame": {"up_axis": "Z", "meters_per_unit": 1.0},
        "runtime_dependency": "none_after_ue_import_and_cook",
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps(report, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ConversionError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc
