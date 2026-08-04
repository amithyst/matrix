#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

for argument in "$@"; do
    case "$argument" in
        --scene|--scene=*|--control-source|--control-source=*)
            echo "[ERROR] RobotTrainingGround PICO launcher fixes --scene 18 and --control-source pico" >&2
            exit 2
            ;;
    esac
done

exec bash "$SCRIPT_DIR/run_matrix_pico.sh" \
    --scene 18 \
    "$@"
