#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

for argument in "$@"; do
    case "$argument" in
        --scene|--control-source)
            echo "[ERROR] RobotTrainingGround PICO launcher fixes --scene 18 and --control-source pico" >&2
            exit 2
            ;;
    esac
done

python3 "$SCRIPT_DIR/verify_realscan_scene_install.py" \
    --project-root "$PROJECT_ROOT"

exec bash "$SCRIPT_DIR/run_matrix_sonic.sh" \
    --scene 18 \
    --control-source pico \
    "$@"
