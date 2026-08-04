#!/usr/bin/env python3
"""Build a deterministic lower-floor MuJoCo proxy from a RealScan navmesh.

The proxy intentionally uses MuJoCo only.  A regular height field preserves the
walkable surface while a merged boundary band blocks walls, shelving footprints,
and holes in the navmesh.  It is a navigation/training proxy, not PhysX USD.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

try:
    import cv2
    import numpy as np
    from PIL import Image
    from scipy import ndimage
except ImportError as exc:  # pragma: no cover - operator environment dependency
    raise SystemExit(
        "build_realscan_mujoco_proxy.py requires numpy, scipy, Pillow, and OpenCV"
    ) from exc


NAVMESH_SHA256 = "bcb26378b769c5256df3246d17317f82f0dea65ced907f76d53943f92b4b75ee"
NAVMESH_SIZE = 547833
SCENE_NAME = "robot-training-ground_20260715-142024"
XML_NAME = "scene_terrain_robot_training_ground.xml"
HEIGHTFIELD_NAME = "robot_training_ground_lower_floor.png"
REPORT_NAME = "robot_training_ground_mujoco_proxy.json"


class ProxyError(RuntimeError):
    """The locked navmesh cannot be converted safely."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_ascii_tri_ply(path: Path) -> tuple["np.ndarray", "np.ndarray"]:
    if not path.is_file() or path.is_symlink():
        raise ProxyError(f"navmesh must be a regular non-symlink file: {path}")
    if path.stat().st_size != NAVMESH_SIZE or sha256_file(path) != NAVMESH_SHA256:
        raise ProxyError("navmesh identity does not match the locked RealScan asset")
    with path.open("r", encoding="ascii", newline="") as stream:
        if stream.readline().strip() != "ply":
            raise ProxyError("navmesh is not PLY")
        if stream.readline().strip() != "format ascii 1.0":
            raise ProxyError("navmesh must be ASCII PLY 1.0")
        vertex_count = None
        face_count = None
        vertex_properties: list[str] = []
        section = None
        for line in stream:
            value = line.strip()
            if value.startswith("element vertex "):
                vertex_count = int(value.rsplit(" ", 1)[1])
                section = "vertex"
            elif value.startswith("element face "):
                face_count = int(value.rsplit(" ", 1)[1])
                section = "face"
            elif value.startswith("property ") and section == "vertex":
                vertex_properties.append(value)
            elif value == "end_header":
                break
        if (
            vertex_count is None
            or face_count is None
            or vertex_properties
            != [
                "property float x",
                "property float y",
                "property float z",
            ]
        ):
            raise ProxyError("navmesh PLY schema drifted")
        vertices = np.empty((vertex_count, 3), dtype=np.float64)
        for index in range(vertex_count):
            fields = stream.readline().split()
            if len(fields) != 3:
                raise ProxyError(f"vertex {index} is malformed")
            vertices[index] = [float(field) for field in fields]
        faces = np.empty((face_count, 3), dtype=np.int32)
        for index in range(face_count):
            fields = stream.readline().split()
            if len(fields) != 4 or fields[0] != "3":
                raise ProxyError(f"face {index} is not a triangle")
            faces[index] = [int(field) for field in fields[1:]]
        if stream.read(1):
            raise ProxyError("navmesh has trailing content")
    if (
        not np.isfinite(vertices).all()
        or faces.min() < 0
        or faces.max() >= len(vertices)
    ):
        raise ProxyError("navmesh contains invalid numeric data")
    return vertices, faces


def rasterize_lower_floor(
    vertices: "np.ndarray",
    faces: "np.ndarray",
    *,
    resolution_m: float,
    maximum_surface_z: float,
) -> tuple["np.ndarray", "np.ndarray", "np.ndarray"]:
    triangles = vertices[faces]
    triangle_z = triangles[:, :, 2].mean(axis=1)
    triangles = triangles[triangle_z <= maximum_surface_z]
    if len(triangles) == 0:
        raise ProxyError("no lower-floor triangles matched the configured Z range")
    xy = triangles[:, :, :2].reshape(-1, 2)
    minimum = np.floor(xy.min(axis=0) / resolution_m) * resolution_m - resolution_m
    maximum = np.ceil(xy.max(axis=0) / resolution_m) * resolution_m + resolution_m
    width_height = np.rint((maximum - minimum) / resolution_m).astype(int) + 1
    width, height = int(width_height[0]), int(width_height[1])
    if width < 2 or height < 2 or width * height > 1_000_000:
        raise ProxyError(f"unsafe height-field dimensions: {width}x{height}")
    walkable = np.zeros((height, width), dtype=np.uint8)
    for triangle in triangles:
        pixels = np.rint((triangle[:, :2] - minimum) / resolution_m).astype(np.int32)
        pixels[:, 1] = height - 1 - pixels[:, 1]
        cv2.fillConvexPoly(walkable, pixels, 1)
    walkable = cv2.morphologyEx(
        walkable,
        cv2.MORPH_CLOSE,
        np.ones((3, 3), dtype=np.uint8),
    )

    samples_sum = np.zeros((height, width), dtype=np.float64)
    samples_count = np.zeros((height, width), dtype=np.uint32)
    floor_vertices = triangles.reshape(-1, 3)
    sample_pixels = np.rint((floor_vertices[:, :2] - minimum) / resolution_m).astype(
        np.int32
    )
    sample_rows = height - 1 - sample_pixels[:, 1]
    sample_columns = sample_pixels[:, 0]
    np.add.at(samples_sum, (sample_rows, sample_columns), floor_vertices[:, 2])
    np.add.at(samples_count, (sample_rows, sample_columns), 1)
    known = samples_count > 0
    if not known.any():
        raise ProxyError("height-field raster has no vertex samples")
    sampled = np.zeros((height, width), dtype=np.float64)
    sampled[known] = samples_sum[known] / samples_count[known]
    nearest = ndimage.distance_transform_edt(
        ~known, return_distances=False, return_indices=True
    )
    heights = sampled[tuple(nearest)]
    heights = np.clip(heights, vertices[:, 2].min(), maximum_surface_z)
    return walkable, heights, minimum


def merged_boundary_rectangles(
    walkable: "np.ndarray",
) -> list[tuple[int, int, int, int]]:
    dilated = cv2.dilate(walkable, np.ones((5, 5), dtype=np.uint8))
    boundary = (dilated > 0) & (walkable == 0)
    rectangles: list[tuple[int, int, int, int]] = []
    active: dict[tuple[int, int], tuple[int, int]] = {}
    for row in range(boundary.shape[0]):
        columns = np.flatnonzero(boundary[row])
        runs: list[tuple[int, int]] = []
        if len(columns):
            cuts = np.flatnonzero(np.diff(columns) > 1) + 1
            runs = [
                (int(run[0]), int(run[-1]))
                for run in np.split(columns, cuts)
                if len(run)
            ]
        current: dict[tuple[int, int], tuple[int, int]] = {}
        for run in runs:
            start_row = active.get(run, (row, row))[0]
            current[run] = (start_row, row)
        for run, (start_row, end_row) in active.items():
            if run not in current:
                rectangles.append((start_row, end_row, run[0], run[1]))
        active = current
    rectangles.extend(
        (start_row, end_row, run[0], run[1])
        for run, (start_row, end_row) in active.items()
    )
    return rectangles


def write_outputs(
    output_dir: Path,
    walkable: "np.ndarray",
    heights: "np.ndarray",
    minimum_xy: "np.ndarray",
    rectangles: list[tuple[int, int, int, int]],
    *,
    resolution_m: float,
) -> dict[str, object]:
    if output_dir.exists() or output_dir.is_symlink():
        raise ProxyError(f"refusing to overwrite output directory: {output_dir}")
    output_dir.mkdir(parents=True)
    min_z = float(heights.min())
    max_z = float(heights.max())
    elevation = max(max_z - min_z, 0.01)
    normalized = np.rint((heights - min_z) / elevation * 255.0).astype(np.uint8)
    heightfield_path = output_dir / HEIGHTFIELD_NAME
    Image.fromarray(normalized, mode="L").save(heightfield_path, optimize=False)

    height, width = walkable.shape
    centre_x = float(minimum_xy[0] + (width - 1) * resolution_m / 2.0)
    centre_y = float(minimum_xy[1] + (height - 1) * resolution_m / 2.0)
    x_radius = (width - 1) * resolution_m / 2.0
    y_radius = (height - 1) * resolution_m / 2.0
    obstacle_height = 1.8
    geoms: list[str] = []
    for index, (row0, row1, column0, column1) in enumerate(rectangles):
        x0 = minimum_xy[0] + (column0 - 0.5) * resolution_m
        x1 = minimum_xy[0] + (column1 + 0.5) * resolution_m
        y_top = minimum_xy[1] + (height - 1 - row0 + 0.5) * resolution_m
        y_bottom = minimum_xy[1] + (height - 1 - row1 - 0.5) * resolution_m
        centre_column = (column0 + column1) // 2
        centre_row = (row0 + row1) // 2
        ground_z = float(heights[centre_row, centre_column])
        geoms.append(
            '    <geom name="rtg_boundary_{:04d}" type="box" '
            'pos="{:.6f} {:.6f} {:.6f}" size="{:.6f} {:.6f} {:.6f}" '
            'rgba="0.15 0.35 0.85 0" friction="1.0 0.02 0.002" '
            'solref="0.02 1" solimp="0.9 0.95 0.001"/>'.format(
                index,
                (x0 + x1) / 2.0,
                (y_bottom + y_top) / 2.0,
                ground_z + obstacle_height / 2.0,
                (x1 - x0) / 2.0,
                (y_top - y_bottom) / 2.0,
                obstacle_height / 2.0,
            )
        )
    xml = f"""<mujoco model=\"Matrix RealScan robot training ground\">
  <include file=\"xgb.xml\" />
  <statistic center=\"{centre_x:.6f} {centre_y:.6f} {min_z + 0.8:.6f}\" extent=\"30\" />
  <visual>
    <headlight diffuse=\"0.6 0.6 0.6\" ambient=\"0.3 0.3 0.3\" specular=\"0 0 0\" />
    <map znear=\"0.01\" zfar=\"150\" />
  </visual>
  <asset>
    <hfield name=\"robot_training_ground_lower_floor\" file=\"../{HEIGHTFIELD_NAME}\"
            size=\"{x_radius:.6f} {y_radius:.6f} {elevation:.6f} 0.20\" />
  </asset>
  <worldbody>
    <geom name=\"robot_training_ground_surface\" type=\"hfield\"
          hfield=\"robot_training_ground_lower_floor\"
          pos=\"{centre_x:.6f} {centre_y:.6f} {min_z:.6f}\"
          rgba=\"0.25 0.30 0.35 0\" friction=\"1.0 0.02 0.002\"
          solref=\"0.02 1\" solimp=\"0.9 0.95 0.001\" />
{chr(10).join(geoms)}
  </worldbody>
</mujoco>
"""
    xml_path = output_dir / XML_NAME
    xml_path.write_text(xml)
    report = {
        "schema": "matrix-realscan-mujoco-proxy/v1",
        "scene": SCENE_NAME,
        "physics_backend": "mujoco",
        "source_navmesh_sha256": NAVMESH_SHA256,
        "scope": "lower_floor_navigation_proxy",
        "resolution_m": resolution_m,
        "grid": {"width": width, "height": height},
        "bounds_xyz": {
            "minimum": [float(minimum_xy[0]), float(minimum_xy[1]), min_z],
            "maximum": [
                float(minimum_xy[0] + (width - 1) * resolution_m),
                float(minimum_xy[1] + (height - 1) * resolution_m),
                max_z,
            ],
        },
        "walkable_cells": int(walkable.sum()),
        "boundary_box_count": len(rectangles),
        "heightfield": {
            "filename": HEIGHTFIELD_NAME,
            "sha256": sha256_file(heightfield_path),
        },
        "xml": {"filename": XML_NAME, "sha256": sha256_file(xml_path)},
        "limitations": [
            "lower floor only",
            "static navigation collision proxy",
            "upper floor and movable objects require later explicit bodies",
        ],
    }
    report_path = output_dir / REPORT_NAME
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--navmesh", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--resolution-m", type=float, default=0.25)
    parser.add_argument("--maximum-surface-z", type=float, default=0.25)
    args = parser.parse_args(argv)
    if (
        not math.isfinite(args.resolution_m)
        or not 0.05 <= args.resolution_m <= 1.0
        or not math.isfinite(args.maximum_surface_z)
    ):
        raise SystemExit("invalid raster configuration")
    vertices, faces = read_ascii_tri_ply(args.navmesh)
    walkable, heights, minimum_xy = rasterize_lower_floor(
        vertices,
        faces,
        resolution_m=args.resolution_m,
        maximum_surface_z=args.maximum_surface_z,
    )
    rectangles = merged_boundary_rectangles(walkable)
    report = write_outputs(
        args.output_dir,
        walkable,
        heights,
        minimum_xy,
        rectangles,
        resolution_m=args.resolution_m,
    )
    print(json.dumps(report, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ProxyError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc
