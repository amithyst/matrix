#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

for argument in "$@"; do
    if [[ "$argument" == "--scene" || "$argument" == --scene=* ]]; then
        echo "[ERROR] house-v1 fixes the native scene to HouseWorld (scene 6)" >&2
        exit 2
    fi
done

echo "[INFO] house-v1 visual map: Matrix native HouseWorld (scene 6)"
echo "[INFO] house-v1 physics: Matrix scene_terrain_house.xml plus SONIC G1"
echo "[WARN] house-v1 requires HouseWorld chunk 17 from the locked runtime."
echo "[WARN] HouseWorld furniture is static MuJoCo collision proxy geometry, not editable or dynamic furniture interaction."

exec bash "$SCRIPT_DIR/run_matrix_sonic.sh" --scene 6 "$@"
