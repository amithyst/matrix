#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

for argument in "$@"; do
    if [[ "$argument" == "--scene" || "$argument" == --scene=* ]]; then
        echo "[ERROR] urban-v1 fixes the native scene to Town10World (scene 2)" >&2
        exit 2
    fi
done

echo "[INFO] urban-v1 visual map: Matrix native Town10World (scene 2)"
echo "[INFO] urban-v1 physics: 869 static native environment geoms plus SONIC G1"
echo "[WARN] Native people and vehicles are UE presentation assets, not dynamic MuJoCo actors."

exec bash "$SCRIPT_DIR/run_matrix_sonic.sh" --scene 2 "$@"
