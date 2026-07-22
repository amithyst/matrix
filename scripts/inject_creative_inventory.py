#!/usr/bin/env python3
"""Inject a bounded pool of standalone physical props into a Matrix MJCF."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
import shutil
import xml.etree.ElementTree as ET


CATALOG_SCHEMA = "matrix-creative-inventory/v1"
ITEM_ID_RE = re.compile(r"[a-z0-9][a-z0-9_-]{0,47}\Z")
MAX_ITEMS = 16
MAX_POOL_SIZE = 32


class InventoryCatalogError(ValueError):
    pass


def _finite_vector(value: object, *, length: int, name: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != length:
        raise InventoryCatalogError(f"{name} must contain {length} numbers")
    result: list[float] = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise InventoryCatalogError(f"{name} must contain only numbers")
        number = float(item)
        if not math.isfinite(number):
            raise InventoryCatalogError(f"{name} must contain finite numbers")
        result.append(number)
    return tuple(result)


def _format_vector(values: tuple[float, ...]) -> str:
    return " ".join(f"{value:.9g}" for value in values)


@dataclass(frozen=True)
class VisualPart:
    mesh: Path
    rgba: tuple[float, float, float, float]
    scale: tuple[float, float, float]


@dataclass(frozen=True)
class InventoryItem:
    item_id: str
    label: str
    pool_size: int
    mass_kg: float
    collision_half_size: tuple[float, float, float]
    spawn_distance_m: float
    spawn_height_m: float
    spawn_quat: tuple[float, float, float, float]
    visuals: tuple[VisualPart, ...]


def load_catalog(path: Path) -> tuple[InventoryItem, ...]:
    path = path.resolve()
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise InventoryCatalogError(f"cannot read inventory catalog: {exc}") from exc
    if not isinstance(value, dict) or set(value) != {"schema", "items"}:
        raise InventoryCatalogError("inventory catalog has an invalid root schema")
    if value.get("schema") != CATALOG_SCHEMA:
        raise InventoryCatalogError("inventory catalog schema is unsupported")
    raw_items = value.get("items")
    if not isinstance(raw_items, list) or not 1 <= len(raw_items) <= MAX_ITEMS:
        raise InventoryCatalogError(f"items must contain 1..{MAX_ITEMS} entries")
    seen_ids: set[str] = set()
    result: list[InventoryItem] = []
    for index, raw in enumerate(raw_items):
        if not isinstance(raw, dict) or set(raw) != {
            "item_id",
            "label",
            "pool_size",
            "mass_kg",
            "collision_half_size",
            "spawn_distance_m",
            "spawn_height_m",
            "spawn_quat",
            "visuals",
        }:
            raise InventoryCatalogError(f"items[{index}] has an invalid schema")
        item_id = raw.get("item_id")
        if not isinstance(item_id, str) or ITEM_ID_RE.fullmatch(item_id) is None:
            raise InventoryCatalogError(f"items[{index}].item_id is invalid")
        if item_id in seen_ids:
            raise InventoryCatalogError(f"duplicate item_id {item_id!r}")
        seen_ids.add(item_id)
        label = raw.get("label")
        if not isinstance(label, str) or not label or len(label) > 40:
            raise InventoryCatalogError(f"items[{index}].label is invalid")
        pool_size = raw.get("pool_size")
        if (
            isinstance(pool_size, bool)
            or not isinstance(pool_size, int)
            or not 1 <= pool_size <= MAX_POOL_SIZE
        ):
            raise InventoryCatalogError(
                f"items[{index}].pool_size must be in [1, {MAX_POOL_SIZE}]"
            )
        mass_kg = raw.get("mass_kg")
        if (
            isinstance(mass_kg, bool)
            or not isinstance(mass_kg, (int, float))
            or not math.isfinite(float(mass_kg))
            or not 0.01 <= float(mass_kg) <= 100.0
        ):
            raise InventoryCatalogError(f"items[{index}].mass_kg is invalid")
        collision_half_size = _finite_vector(
            raw.get("collision_half_size"),
            length=3,
            name=f"items[{index}].collision_half_size",
        )
        if any(not 0.005 <= value <= 5.0 for value in collision_half_size):
            raise InventoryCatalogError(
                f"items[{index}].collision_half_size is outside [0.005, 5]"
            )
        distance = raw.get("spawn_distance_m")
        height = raw.get("spawn_height_m")
        for name, number, minimum, maximum in (
            ("spawn_distance_m", distance, 0.3, 5.0),
            ("spawn_height_m", height, 0.05, 3.0),
        ):
            if (
                isinstance(number, bool)
                or not isinstance(number, (int, float))
                or not math.isfinite(float(number))
                or not minimum <= float(number) <= maximum
            ):
                raise InventoryCatalogError(f"items[{index}].{name} is invalid")
        spawn_quat = _finite_vector(
            raw.get("spawn_quat"),
            length=4,
            name=f"items[{index}].spawn_quat",
        )
        norm = math.sqrt(sum(value * value for value in spawn_quat))
        if norm < 1e-9:
            raise InventoryCatalogError(f"items[{index}].spawn_quat is zero")
        spawn_quat = tuple(value / norm for value in spawn_quat)
        raw_visuals = raw.get("visuals")
        if not isinstance(raw_visuals, list) or not 1 <= len(raw_visuals) <= 16:
            raise InventoryCatalogError(f"items[{index}].visuals is invalid")
        visuals: list[VisualPart] = []
        for visual_index, raw_visual in enumerate(raw_visuals):
            if not isinstance(raw_visual, dict) or set(raw_visual) != {
                "mesh",
                "rgba",
                "scale",
            }:
                raise InventoryCatalogError(
                    f"items[{index}].visuals[{visual_index}] has an invalid schema"
                )
            mesh_value = raw_visual.get("mesh")
            if (
                not isinstance(mesh_value, str)
                or not mesh_value
                or Path(mesh_value).is_absolute()
                or ".." in Path(mesh_value).parts
            ):
                raise InventoryCatalogError(
                    f"items[{index}].visuals[{visual_index}].mesh is unsafe"
                )
            mesh = (path.parent / mesh_value).resolve()
            if not mesh.is_file() or mesh.suffix.lower() != ".stl":
                raise InventoryCatalogError(f"inventory mesh is missing or not STL: {mesh}")
            rgba = _finite_vector(
                raw_visual.get("rgba"),
                length=4,
                name=f"items[{index}].visuals[{visual_index}].rgba",
            )
            if any(not 0.0 <= value <= 1.0 for value in rgba):
                raise InventoryCatalogError("visual RGBA values must be within [0, 1]")
            scale = _finite_vector(
                raw_visual.get("scale"),
                length=3,
                name=f"items[{index}].visuals[{visual_index}].scale",
            )
            if any(not 0.001 <= value <= 100.0 for value in scale):
                raise InventoryCatalogError("visual scale values are invalid")
            visuals.append(VisualPart(mesh=mesh, rgba=rgba, scale=scale))
        result.append(
            InventoryItem(
                item_id=item_id,
                label=label,
                pool_size=pool_size,
                mass_kg=float(mass_kg),
                collision_half_size=collision_half_size,
                spawn_distance_m=float(distance),
                spawn_height_m=float(height),
                spawn_quat=spawn_quat,
                visuals=tuple(visuals),
            )
        )
    return tuple(result)


def palette(items: tuple[InventoryItem, ...]) -> str:
    seen: set[tuple[float, float, float]] = set()
    result: list[str] = []
    for item in items:
        for visual in item.visuals:
            rgb = visual.rgba[:3]
            if rgb in seen:
                continue
            seen.add(rgb)
            result.append(",".join(f"{value:.9g}" for value in rgb))
    return ";".join(result)


def inject_catalog(
    mjcf_path: Path,
    assets_dir: Path,
    catalog_path: Path,
    *,
    use_default_classes: bool = True,
) -> dict[str, object]:
    mjcf_path = mjcf_path.resolve()
    assets_dir = assets_dir.resolve()
    items = load_catalog(catalog_path)
    tree = ET.parse(mjcf_path)
    root = tree.getroot()
    if any(
        body.get("name", "").startswith("creative_item__")
        for body in root.iter("body")
    ):
        raise InventoryCatalogError("MJCF already contains a creative inventory pool")
    asset = root.find("asset")
    if asset is None:
        asset = ET.Element("asset")
        root.insert(0, asset)
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise InventoryCatalogError("MJCF has no worldbody")
    equality = root.find("equality")
    if equality is None:
        equality = ET.SubElement(root, "equality")
    assets_dir.mkdir(parents=True, exist_ok=True)
    copied: dict[str, Path] = {}
    body_count = 0
    for item in items:
        mesh_names: list[str] = []
        material_names: list[str] = []
        for visual_index, visual in enumerate(item.visuals):
            destination_name = f"creative_{item.item_id}_{visual_index}.stl"
            destination = assets_dir / destination_name
            previous = copied.get(destination_name)
            if previous is not None and previous.read_bytes() != visual.mesh.read_bytes():
                raise InventoryCatalogError(f"conflicting mesh basename {destination_name}")
            if previous is None:
                shutil.copy2(visual.mesh, destination)
                copied[destination_name] = visual.mesh
            mesh_name = f"creative_{item.item_id}_{visual_index}"
            material_name = f"matrix_source_creative_{item.item_id}_{visual_index}"
            ET.SubElement(
                asset,
                "mesh",
                {
                    "name": mesh_name,
                    "file": destination_name,
                    "scale": _format_vector(visual.scale),
                },
            )
            ET.SubElement(
                asset,
                "material",
                {"name": material_name, "rgba": _format_vector(visual.rgba)},
            )
            mesh_names.append(mesh_name)
            material_names.append(material_name)
        for pool_index in range(item.pool_size):
            storage_z = -50.0 - body_count * 2.0
            body_name = f"creative_item__{item.item_id}__{pool_index}"
            body = ET.SubElement(
                worldbody,
                "body",
                {"name": body_name, "pos": f"0 0 {storage_z:.9g}"},
            )
            ET.SubElement(body, "freejoint", {"name": f"{body_name}__freejoint"})
            for visual_index, (mesh_name, material_name) in enumerate(
                zip(mesh_names, material_names, strict=True)
            ):
                visual_attributes = {
                    "name": f"{body_name}__visual_{visual_index}",
                    "type": "mesh",
                    "mesh": mesh_name,
                    "material": material_name,
                    "contype": "0",
                    "conaffinity": "0",
                    "density": "0",
                    "group": "2",
                }
                if use_default_classes:
                    visual_attributes["class"] = "visual"
                ET.SubElement(
                    body,
                    "geom",
                    visual_attributes,
                )
            collision_attributes = {
                "name": f"{body_name}__collision",
                "type": "box",
                "size": _format_vector(item.collision_half_size),
                "mass": f"{item.mass_kg:.9g}",
                "rgba": "0 0 0 0",
            }
            if use_default_classes:
                collision_attributes["class"] = "collision"
            ET.SubElement(
                body,
                "geom",
                collision_attributes,
            )
            ET.SubElement(
                equality,
                "weld",
                {
                    "name": f"{body_name}__storage_weld",
                    "body1": body_name,
                    "active": "true",
                    "relpose": f"0 0 {storage_z:.9g} 1 0 0 0",
                },
            )
            body_count += 1
    ET.indent(tree, space="  ")
    tree.write(mjcf_path, encoding="utf-8", xml_declaration=False)
    return {
        "catalog": str(catalog_path.resolve()),
        "items": [
            {"item_id": item.item_id, "label": item.label, "pool_size": item.pool_size}
            for item in items
        ],
        "pool_bodies": body_count,
        "copied_meshes": len(copied),
        "palette": palette(items),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog", type=Path, required=True)
    parser.add_argument("--mjcf", type=Path)
    parser.add_argument("--assets-dir", type=Path)
    parser.add_argument("--print-palette", action="store_true")
    args = parser.parse_args()
    if args.print_palette:
        if args.mjcf is not None or args.assets_dir is not None:
            parser.error("--print-palette cannot be combined with injection arguments")
        print(palette(load_catalog(args.catalog)))
        return 0
    if args.mjcf is None or args.assets_dir is None:
        parser.error("--mjcf and --assets-dir are required for injection")
    print(
        json.dumps(
            inject_catalog(args.mjcf, args.assets_dir, args.catalog),
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
