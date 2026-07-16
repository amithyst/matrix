#!/usr/bin/env python3
"""Validate the audited native-scene inventory against Matrix sources."""

from __future__ import annotations

import argparse
import json
import re
import sys
import urllib.request
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INVENTORY = PROJECT_ROOT / "research/sonic_integration/native_scenes.json"
DEFAULT_LAUNCHER = PROJECT_ROOT / "scripts/run_sim.sh"


def load_json(source: str | Path) -> dict[str, Any]:
    value = str(source)
    if value.startswith(("https://", "http://")):
        with urllib.request.urlopen(value, timeout=30) as response:  # noqa: S310
            return json.load(response)
    with Path(value).open(encoding="utf-8") as stream:
        return json.load(stream)


def parse_launcher_scenes(path: Path) -> dict[int, tuple[str, str]]:
    text = path.read_text(encoding="utf-8")
    entries: dict[int, tuple[str, str]] = {}
    pattern = re.compile(
        r"^\s*(\d+)\)\s*(.*?)(?=^\s*(?:\d+|\*)\))",
        flags=re.MULTILINE | re.DOTALL,
    )
    for match in pattern.finditer(text):
        scene_id = int(match.group(1))
        body = match.group(2)
        scene_match = re.search(r'SCENE="([^"]+)"', body)
        map_match = re.search(r'MAPNAME="([^"]+)"', body)
        if scene_match and map_match:
            entries[scene_id] = (scene_match.group(1), map_match.group(1))
    return entries


def validate_inventory(
    inventory: dict[str, Any], launcher_entries: dict[int, tuple[str, str]]
) -> list[str]:
    errors: list[str] = []
    scenes = inventory.get("selectable_scenes", [])
    internal_maps = inventory.get("internal_base_maps", [])
    declared = inventory.get("counts", {})

    calculated = {
        "curated_selectable_scenes": sum(
            scene.get("kind") == "curated" for scene in scenes
        ),
        "custom_templates": sum(scene.get("kind") == "template" for scene in scenes),
        "unique_selectable_map_packages": len(
            {scene.get("package_name") for scene in scenes}
        ),
        "launcher_scene_ids": sum(len(scene.get("launcher_ids", [])) for scene in scenes),
        "internal_base_maps": len(internal_maps),
        "total_packaged_map_assets": len(scenes) + len(internal_maps),
    }
    for name, actual in calculated.items():
        expected = declared.get(name)
        if expected != actual:
            errors.append(f"count {name}: declared={expected!r}, calculated={actual}")

    package_files: set[str] = set()
    scene_ids: set[int] = set()
    for scene in scenes:
        name = scene.get("package_name", "<unnamed>")
        package = scene.get("release_package", {})
        package_file = package.get("file")
        if package_file in package_files:
            errors.append(f"duplicate release package file: {package_file}")
        package_files.add(package_file)

        proxy = scene.get("physics_proxy")
        if not isinstance(proxy, dict):
            errors.append(f"missing physics proxy inventory: {name}")
        else:
            geom_count = proxy.get("geom_count")
            typed_count = sum(proxy.get("geom_types", {}).values())
            if geom_count != typed_count:
                errors.append(
                    f"physics proxy {name}: geom_count={geom_count}, "
                    f"typed_count={typed_count}"
                )

        for scene_id in scene.get("launcher_ids", []):
            if scene_id in scene_ids:
                errors.append(f"duplicate launcher scene id: {scene_id}")
            scene_ids.add(scene_id)
            actual = launcher_entries.get(scene_id)
            expected = (scene.get("mujoco_scene"), scene.get("ue_map"))
            if actual != expected:
                errors.append(
                    f"launcher id {scene_id} ({name}): expected={expected}, actual={actual}"
                )

    extra_ids = set(launcher_entries) - scene_ids
    if extra_ids:
        errors.append(f"launcher contains untracked scene ids: {sorted(extra_ids)}")

    aliases = next(
        (scene for scene in scenes if scene.get("package_name") == "3DGSWorld"), None
    )
    if aliases is None or aliases.get("launcher_ids") != [14, 16, 17]:
        errors.append("3DGSWorld must record launcher aliases [14, 16, 17]")

    excluded = " ".join(inventory.get("scope", {}).get("excluded_external_scenes", []))
    if "overworld" not in excluded.lower():
        errors.append("scope must explicitly exclude the AUE overworld scene")
    return errors


def validate_release_manifest(
    inventory: dict[str, Any], manifest: dict[str, Any]
) -> list[str]:
    errors: list[str] = []
    manifest_maps = {
        item["name"]: item for item in manifest.get("packages", {}).get("maps", [])
    }
    scenes = inventory.get("selectable_scenes", [])
    inventory_names = {scene["package_name"] for scene in scenes}
    if inventory_names != set(manifest_maps):
        errors.append(
            "release map names differ: "
            f"inventory_only={sorted(inventory_names - set(manifest_maps))}, "
            f"manifest_only={sorted(set(manifest_maps) - inventory_names)}"
        )

    for scene in scenes:
        name = scene["package_name"]
        expected = scene["release_package"]
        actual = manifest_maps.get(name)
        if actual is None:
            continue
        for inventory_key, manifest_key in (
            ("file", "file"),
            ("size_bytes", "size"),
            ("sha256", "sha256"),
        ):
            if expected[inventory_key] != actual.get(manifest_key):
                errors.append(
                    f"manifest {name}.{manifest_key}: "
                    f"inventory={expected[inventory_key]!r}, "
                    f"manifest={actual.get(manifest_key)!r}"
                )

    internal_maps = inventory.get("internal_base_maps", [])
    if internal_maps:
        expected_base = internal_maps[0]["release_package"]
        actual_base = manifest.get("packages", {}).get("base", {})
        for inventory_key, manifest_key in (
            ("file", "file"),
            ("size_bytes", "size"),
            ("sha256", "sha256"),
        ):
            if expected_base[inventory_key] != actual_base.get(manifest_key):
                errors.append(
                    f"manifest base.{manifest_key}: "
                    f"inventory={expected_base[inventory_key]!r}, "
                    f"manifest={actual_base.get(manifest_key)!r}"
                )
    return errors


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inventory", default=str(DEFAULT_INVENTORY))
    parser.add_argument("--launcher", default=str(DEFAULT_LAUNCHER))
    parser.add_argument(
        "--manifest",
        help="Optional release manifest path or URL to validate package metadata",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    inventory = load_json(args.inventory)
    launcher_entries = parse_launcher_scenes(Path(args.launcher))
    errors = validate_inventory(inventory, launcher_entries)
    if args.manifest:
        errors.extend(validate_release_manifest(inventory, load_json(args.manifest)))

    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1

    counts = inventory["counts"]
    print(
        "native scene inventory OK: "
        f"{counts['curated_selectable_scenes']} curated + "
        f"{counts['custom_templates']} template = "
        f"{counts['unique_selectable_map_packages']} selectable packages; "
        f"{counts['total_packaged_map_assets']} including EmptyWorld"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
