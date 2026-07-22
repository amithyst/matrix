#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

for argument in "$@"; do
    if [[ "$argument" == "--scene" || "$argument" == --scene=* ]]; then
        echo "[ERROR] moon-v1 fixes the native scene to MoonWorld (scene 15)" >&2
        exit 2
    fi
done

echo "[INFO] moon-v1 visual map: Matrix native MoonWorld (scene 15)"
echo "[INFO] moon-v1 physics: Matrix scene_terrain_moon_dynamic.xml plus SONIC G1"
echo "[WARN] moon-v1 requires MoonWorld chunk 26 and dynamicmaps/moonworld.bin from the locked runtime."
echo "[WARN] visual MoonWorld does not by itself prove 1.62 m/s^2 low-gravity physics acceptance."

exec bash "$SCRIPT_DIR/run_matrix_sonic.sh" --scene 15 "$@"
