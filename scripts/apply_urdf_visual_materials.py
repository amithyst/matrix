#!/usr/bin/env python3
"""Preserve URDF colors or apply a provenance-bearing visual profile to MJCF."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import stat
import sys
import xml.etree.ElementTree as ET


DEFAULT_RGBA = "0.75294 0.75294 0.75294 1"
GENERATED_PREFIX = "urdf_visual_"
REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE_PATH = REPO_ROOT / "config" / "materials" / "aue_g1_v1.json"
AUTO_PROFILE = "auto"
URDF_PROFILE = "urdf"


class VisualMaterialError(RuntimeError):
    """Raised when URDF material data cannot be applied safely."""


@dataclass(frozen=True)
class VisualStyle:
    source_name: str
    rgba: str
    material_name: str
    roughness: str | None = None
    metallic: str | None = None


@dataclass(frozen=True)
class MaterialSummary:
    profile_id: str
    source_visuals: int
    source_styles: int
    styled_geoms: int
    unmatched_visual_geoms: int
    generated_materials: int


def _canonical_rgba(raw: str, *, context: str) -> str:
    parts = raw.split()
    if len(parts) == 3:
        parts.append("1")
    if len(parts) != 4:
        raise VisualMaterialError(f"{context} must contain three or four RGBA values")
    try:
        values = [float(value) for value in parts]
    except ValueError as exc:
        raise VisualMaterialError(
            f"{context} contains a non-numeric RGBA value"
        ) from exc
    if any(not math.isfinite(value) or value < 0.0 or value > 1.0 for value in values):
        raise VisualMaterialError(
            f"{context} RGBA values must be finite and within [0, 1]"
        )
    return " ".join(f"{value:.9g}" for value in values)


def _canonical_unit_scalar(raw: object, *, context: str) -> str:
    try:
        value = float(raw)
    except (TypeError, ValueError) as exc:
        raise VisualMaterialError(f"{context} must be numeric") from exc
    if not math.isfinite(value) or value < 0.0 or value > 1.0:
        raise VisualMaterialError(f"{context} must be finite and within [0, 1]")
    return f"{value:.9g}"


def _material_slug(name: str) -> str:
    slug = re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")
    return slug[:40] or "color"


def _style(
    source_name: str,
    rgba: str,
    *,
    roughness: object | None = None,
    metallic: object | None = None,
) -> VisualStyle:
    canonical = _canonical_rgba(rgba, context=f"material {source_name!r}")
    canonical_roughness = (
        _canonical_unit_scalar(roughness, context=f"material {source_name!r} roughness")
        if roughness is not None
        else None
    )
    canonical_metallic = (
        _canonical_unit_scalar(metallic, context=f"material {source_name!r} metallic")
        if metallic is not None
        else None
    )
    digest_source = "\0".join(
        (source_name, canonical, canonical_roughness or "", canonical_metallic or "")
    )
    digest = hashlib.sha256(digest_source.encode()).hexdigest()[:8]
    return VisualStyle(
        source_name=source_name,
        rgba=canonical,
        material_name=f"{GENERATED_PREFIX}{_material_slug(source_name)}_{digest}",
        roughness=canonical_roughness,
        metallic=canonical_metallic,
    )


def _load_profile(profile_path: Path) -> dict[str, object]:
    try:
        profile = json.loads(profile_path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise VisualMaterialError(
            f"material profile cannot be read: {profile_path}"
        ) from exc
    except json.JSONDecodeError as exc:
        raise VisualMaterialError(
            f"material profile is invalid JSON: {profile_path}"
        ) from exc
    if not isinstance(profile, dict) or profile.get("schema_version") != 1:
        raise VisualMaterialError("material profile schema_version must be 1")
    profile_id = profile.get("profile_id")
    materials = profile.get("materials")
    rules = profile.get("rules")
    default_material = profile.get("default_material")
    matcher = profile.get("matcher")
    if not isinstance(profile_id, str) or not profile_id:
        raise VisualMaterialError("material profile_id must be a non-empty string")
    if not isinstance(materials, dict) or not materials:
        raise VisualMaterialError("material profile must define materials")
    if default_material not in materials:
        raise VisualMaterialError("material profile default_material is undefined")
    if not isinstance(rules, list):
        raise VisualMaterialError("material profile rules must be a list")
    if not isinstance(matcher, dict) or not isinstance(
        matcher.get("required_links"), list
    ):
        raise VisualMaterialError(
            "material profile matcher.required_links must be a list"
        )
    for key, material in materials.items():
        if not isinstance(key, str) or not isinstance(material, dict):
            raise VisualMaterialError("material profile entries must be named objects")
        if not isinstance(material.get("rgba"), list):
            raise VisualMaterialError(f"profile material {key!r} must define rgba")
        _style(
            f"{profile_id}:{key}",
            " ".join(str(value) for value in material["rgba"]),
            roughness=material.get("roughness"),
            metallic=material.get("metallic"),
        )
    for rule in rules:
        if (
            not isinstance(rule, dict)
            or rule.get("material") not in materials
            or not isinstance(rule.get("tokens"), list)
            or not all(isinstance(token, str) and token for token in rule["tokens"])
        ):
            raise VisualMaterialError("material profile contains an invalid rule")
    return profile


def _profile_matches(urdf_root: ET.Element, profile: dict[str, object]) -> bool:
    matcher = profile["matcher"]
    assert isinstance(matcher, dict)
    required_links = matcher["required_links"]
    assert isinstance(required_links, list)
    available_links = {
        link.get("name") for link in urdf_root.findall("link") if link.get("name")
    }
    return set(required_links).issubset(available_links)


def _select_profile(
    urdf_root: ET.Element,
    requested_profile: str,
    profile_path: Path,
) -> dict[str, object] | None:
    if requested_profile == URDF_PROFILE:
        return None
    profile = _load_profile(profile_path)
    profile_id = profile["profile_id"]
    if requested_profile not in {AUTO_PROFILE, profile_id}:
        raise VisualMaterialError(
            f"unknown material profile {requested_profile!r}; expected auto, urdf, or {profile_id}"
        )
    matches = _profile_matches(urdf_root, profile)
    if requested_profile == AUTO_PROFILE:
        return profile if matches else None
    if not matches:
        raise VisualMaterialError(
            f"URDF does not satisfy the required link signature for profile {profile_id}"
        )
    return profile


def _profile_styles(profile: dict[str, object]) -> dict[str, VisualStyle]:
    profile_id = profile["profile_id"]
    materials = profile["materials"]
    assert isinstance(profile_id, str) and isinstance(materials, dict)
    styles: dict[str, VisualStyle] = {}
    for key, material in materials.items():
        assert isinstance(key, str) and isinstance(material, dict)
        rgba = material["rgba"]
        assert isinstance(rgba, list)
        styles[key] = _style(
            f"{profile_id}:{key}",
            " ".join(str(value) for value in rgba),
            roughness=material.get("roughness"),
            metallic=material.get("metallic"),
        )
    return styles


def _profile_visual_style(
    profile: dict[str, object],
    styles: dict[str, VisualStyle],
    *,
    link_name: str,
    mesh_name: str,
    filename: str,
) -> VisualStyle:
    searchable = f"{link_name}/{mesh_name}/{filename}".lower()
    rules = profile["rules"]
    assert isinstance(rules, list)
    for rule in rules:
        assert isinstance(rule, dict)
        tokens = rule["tokens"]
        assert isinstance(tokens, list)
        if any(token.lower() in searchable for token in tokens):
            material_key = rule["material"]
            assert isinstance(material_key, str)
            return styles[material_key]
    default_material = profile["default_material"]
    assert isinstance(default_material, str)
    return styles[default_material]


def _global_materials(urdf_root: ET.Element) -> dict[str, str]:
    colors: dict[str, str] = {}
    for material in urdf_root.findall("material"):
        name = material.get("name")
        color = material.find("color")
        if name and color is not None and color.get("rgba"):
            colors[name] = _canonical_rgba(
                color.get("rgba", ""), context=f"global material {name!r}"
            )
    return colors


def _visual_style(visual: ET.Element, global_materials: dict[str, str]) -> VisualStyle:
    material = visual.find("material")
    source_name = "default"
    rgba = DEFAULT_RGBA
    if material is not None:
        source_name = material.get("name") or "inline"
        color = material.find("color")
        if color is not None and color.get("rgba"):
            rgba = color.get("rgba", DEFAULT_RGBA)
        elif material.get("name") in global_materials:
            rgba = global_materials[material.get("name", "")]
    return _style(source_name, rgba)


def _source_styles(
    urdf_root: ET.Element,
    profile: dict[str, object] | None,
) -> tuple[
    dict[tuple[str, str], VisualStyle],
    dict[str, VisualStyle],
    int,
]:
    global_materials = _global_materials(urdf_root)
    profile_materials = _profile_styles(profile) if profile is not None else None
    by_link_mesh: dict[tuple[str, str], VisualStyle] = {}
    mesh_candidates: dict[str, set[VisualStyle]] = {}
    source_visuals = 0
    for link in urdf_root.findall("link"):
        link_name = link.get("name")
        if not link_name:
            continue
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            filename = mesh.get("filename") if mesh is not None else None
            if not filename:
                continue
            source_visuals += 1
            mesh_name = Path(filename).stem
            style = (
                _profile_visual_style(
                    profile,
                    profile_materials,
                    link_name=link_name,
                    mesh_name=mesh_name,
                    filename=filename,
                )
                if profile is not None and profile_materials is not None
                else _visual_style(visual, global_materials)
            )
            key = (link_name, mesh_name)
            previous = by_link_mesh.get(key)
            if previous is not None and previous != style:
                raise VisualMaterialError(
                    f"URDF assigns conflicting materials to {link_name}/{mesh_name}"
                )
            by_link_mesh[key] = style
            mesh_candidates.setdefault(mesh_name, set()).add(style)
    by_unique_mesh = {
        mesh_name: next(iter(styles))
        for mesh_name, styles in mesh_candidates.items()
        if len(styles) == 1
    }
    return by_link_mesh, by_unique_mesh, source_visuals


def _is_visual_mesh(geom: ET.Element) -> bool:
    if geom.get("type") != "mesh":
        return False
    return (
        geom.get("class") in {"visual", "visualgeom"}
        or geom.get("group") == "2"
        or geom.get("name", "").endswith("_visual")
    )


def _ensure_materials(asset: ET.Element, styles: set[VisualStyle]) -> int:
    for material in list(asset.findall("material")):
        if material.get("name", "").startswith(GENERATED_PREFIX):
            asset.remove(material)

    children = list(asset)
    insert_at = 0
    for index, child in enumerate(children):
        if child.tag == "material":
            insert_at = index + 1
    for style in sorted(styles, key=lambda item: item.material_name):
        attributes = {"name": style.material_name, "rgba": style.rgba}
        if style.roughness is not None:
            attributes["roughness"] = style.roughness
        if style.metallic is not None:
            attributes["metallic"] = style.metallic
        asset.insert(
            insert_at,
            ET.Element("material", attrib=attributes),
        )
        insert_at += 1
    return len(styles)


def _write_atomic(tree: ET.ElementTree, path: Path) -> None:
    temporary = path.with_name(f".{path.name}.materials.tmp")
    mode = stat.S_IMODE(path.stat().st_mode)
    try:
        tree.write(temporary, encoding="utf-8", xml_declaration=False)
        os.chmod(temporary, mode)
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def apply_urdf_visual_materials(
    urdf_path: Path,
    mjcf_path: Path,
    *,
    profile: str = AUTO_PROFILE,
    profile_path: Path = DEFAULT_PROFILE_PATH,
) -> MaterialSummary:
    urdf_path = urdf_path.resolve()
    mjcf_path = mjcf_path.resolve()
    if not urdf_path.is_file():
        raise VisualMaterialError(f"URDF does not exist: {urdf_path}")
    if not mjcf_path.is_file():
        raise VisualMaterialError(f"MJCF does not exist: {mjcf_path}")

    urdf_root = ET.parse(urdf_path).getroot()
    selected_profile = _select_profile(urdf_root, profile, profile_path.resolve())
    by_link_mesh, by_unique_mesh, source_visuals = _source_styles(
        urdf_root, selected_profile
    )
    tree = ET.parse(mjcf_path)
    root = tree.getroot()
    asset = root.find("asset")
    if asset is None:
        asset = ET.Element("asset")
        root.insert(0, asset)

    styles = set(by_link_mesh.values())
    generated_materials = _ensure_materials(asset, styles)
    styled_geoms = 0
    unmatched_visual_geoms = 0
    for body in root.iter("body"):
        link_name = body.get("name", "")
        for geom in body.findall("geom"):
            if not _is_visual_mesh(geom):
                continue
            mesh_name = geom.get("mesh", "")
            style = by_link_mesh.get((link_name, mesh_name))
            if style is None:
                style = by_unique_mesh.get(mesh_name)
            if style is None:
                unmatched_visual_geoms += 1
                continue
            geom.set("material", style.material_name)
            geom.set("rgba", style.rgba)
            styled_geoms += 1

    _write_atomic(tree, mjcf_path)
    return MaterialSummary(
        profile_id=(
            str(selected_profile["profile_id"])
            if selected_profile is not None
            else URDF_PROFILE
        ),
        source_visuals=source_visuals,
        source_styles=len(styles),
        styled_geoms=styled_geoms,
        unmatched_visual_geoms=unmatched_visual_geoms,
        generated_materials=generated_materials,
    )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--mjcf", type=Path, required=True)
    parser.add_argument(
        "--profile",
        default=os.environ.get("MATRIX_CUSTOM_MATERIAL_PROFILE", AUTO_PROFILE),
        help="auto, urdf, or the configured profile id",
    )
    parser.add_argument("--profile-path", type=Path, default=DEFAULT_PROFILE_PATH)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    try:
        summary = apply_urdf_visual_materials(
            args.urdf,
            args.mjcf,
            profile=args.profile,
            profile_path=args.profile_path,
        )
    except (OSError, ET.ParseError, VisualMaterialError) as exc:
        print(
            f"[ERROR] URDF visual material application failed: {exc}", file=sys.stderr
        )
        return 1
    print("[INFO] URDF visual materials " + json.dumps(asdict(summary), sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
