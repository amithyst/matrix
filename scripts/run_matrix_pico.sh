#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

usage() {
    cat <<'EOF'
Usage: bash scripts/run_matrix_pico.sh [--scene ID] [--dry-run] [Matrix options]

Launch Matrix with its locked native PICO control source. Town10 (scene 2) is
the default. All remaining options are forwarded to run_matrix_sonic.sh.

Options:
  --scene ID   Matrix native scene id (default: 2, Town10)
  --dry-run    Print the resolved launch without starting Matrix
  -h, --help   Show this help

--control-source is intentionally rejected because this launcher always uses
the verified PICO path. Scene 18 additionally requires a verified
RobotTrainingGround visual and physics install before launch.
EOF
}

SCENE_ID=2
DRY_RUN=0
FORWARDED_ARGS=()
while (($#)); do
    case "$1" in
        --scene)
            if (($# < 2)); then
                echo "[ERROR] --scene requires a value" >&2
                exit 2
            fi
            SCENE_ID="$2"
            shift 2
            ;;
        --scene=*)
            SCENE_ID="${1#*=}"
            shift
            ;;
        --control-source|--control-source=*)
            echo "[ERROR] PICO launcher fixes --control-source pico" >&2
            exit 2
            ;;
        --dry-run)
            DRY_RUN=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            FORWARDED_ARGS+=("$1")
            shift
            ;;
    esac
done

if [[ ! "$SCENE_ID" =~ ^[0-9]+$ ]] || ((10#$SCENE_ID > 99)); then
    echo "[ERROR] --scene must be an integer in [0, 99]" >&2
    exit 2
fi
SCENE_ID="$((10#$SCENE_ID))"

if [[ "$SCENE_ID" == "18" ]]; then
    python3 "$SCRIPT_DIR/verify_realscan_scene_install.py" \
        --project-root "$PROJECT_ROOT"
fi

COMMAND=(
    bash "$SCRIPT_DIR/run_matrix_sonic.sh"
    --scene "$SCENE_ID"
    --control-source pico
    "${FORWARDED_ARGS[@]}"
)
if [[ "$DRY_RUN" == "1" ]]; then
    printf 'PICO_LAUNCH scene=%s control_source=pico command=' "$SCENE_ID"
    printf '%q ' "${COMMAND[@]}"
    printf '\n'
    exit 0
fi

exec "${COMMAND[@]}"
