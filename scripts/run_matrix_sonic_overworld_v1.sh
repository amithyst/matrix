#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

LAYOUT="$PROJECT_ROOT/research/overworld_v1/layout.json"
CONTROL_SOURCE="planner"
WALK_AFTER="2"
VX=""
VY="0.0"
YAW_RATE="0.0"
MAX_SECONDS="70"
STARTUP_BAND=1
STARTUP_BAND_HOLD="4"
STARTUP_BAND_FADE="3"
SPAWN_X=""
SPAWN_Y=""
SPAWN_Z=""
SPAWN_YAW=""

usage() {
    printf '%s\n' \
        "Usage: bash scripts/run_matrix_sonic_overworld_v1.sh [options]" \
        "" \
        "Runs the adjacent six-scene Overworld physics model without claiming UE visual composition." \
        "" \
        "Options:" \
        "  --layout PATH              Layout JSON (default: research/overworld_v1/layout.json)" \
        "  --control-source SOURCE    planner, pico, or external (default: planner)" \
        "  --walk-after SECONDS       Start walking after active lowcmd (default: 2)" \
        "  --vx MPS                    Forward velocity; defaults to layout acceptance value" \
        "  --vy MPS                    Lateral velocity (default: 0)" \
        "  --yaw-rate RAD_S           Yaw velocity (default: 0)" \
        "  --max-seconds SECONDS      Bounded runtime (default: 70)" \
        "  --spawn-x/--spawn-y METERS Override layout acceptance spawn" \
        "  --spawn-z METERS           Override root height" \
        "  --spawn-yaw RADIANS        Override root yaw" \
        "  --no-startup-band          Disable temporary SONIC INIT stabilization" \
        "  --startup-band-hold SEC    Hold duration (default: 4)" \
        "  --startup-band-fade SEC    Fade duration (default: 3)"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --layout) LAYOUT="$2"; shift 2 ;;
        --control-source) CONTROL_SOURCE="$2"; shift 2 ;;
        --walk-after) WALK_AFTER="$2"; shift 2 ;;
        --vx) VX="$2"; shift 2 ;;
        --vy) VY="$2"; shift 2 ;;
        --yaw-rate) YAW_RATE="$2"; shift 2 ;;
        --max-seconds) MAX_SECONDS="$2"; shift 2 ;;
        --spawn-x) SPAWN_X="$2"; shift 2 ;;
        --spawn-y) SPAWN_Y="$2"; shift 2 ;;
        --spawn-z) SPAWN_Z="$2"; shift 2 ;;
        --spawn-yaw) SPAWN_YAW="$2"; shift 2 ;;
        --startup-band) STARTUP_BAND=1; shift ;;
        --no-startup-band) STARTUP_BAND=0; shift ;;
        --startup-band-hold) STARTUP_BAND_HOLD="$2"; shift 2 ;;
        --startup-band-fade) STARTUP_BAND_FADE="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "[ERROR] Unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

if [[ ! -f "$LAYOUT" ]]; then
    echo "[ERROR] Overworld layout is missing: $LAYOUT" >&2
    exit 2
fi
if ! command -v jq >/dev/null 2>&1; then
    echo "[ERROR] jq is required to read the Overworld layout" >&2
    exit 1
fi

find_first_dir() {
    local candidate
    for candidate in "$@"; do
        if [[ -d "$candidate" ]]; then
            realpath "$candidate"
            return 0
        fi
    done
    return 1
}

MATRIX_AUE_ROOT="${MATRIX_AUE_ROOT:-$(find_first_dir \
    "$PROJECT_ROOT/../aue-sim" \
    "$HOME/aue-split-lab/repos/aue-sim" || true)}"
MATRIX_GEAR_SONIC_ROOT="${MATRIX_GEAR_SONIC_ROOT:-$(find_first_dir \
    "$MATRIX_AUE_ROOT/third_party/GR00T-WholeBodyControl" \
    "$HOME/code_bryce/GR00T-WholeBodyControl" \
    "$HOME/metabot-workspace/GR00T-WholeBodyControl" || true)}"
MATRIX_UNITREE_SDK2_ROOT="${MATRIX_UNITREE_SDK2_ROOT:-$MATRIX_GEAR_SONIC_ROOT/gear_sonic_deploy/thirdparty/unitree_sdk2}"
MATRIX_NATIVE_SCENE_ROOT="${MATRIX_NATIVE_SCENE_ROOT:-$PROJECT_ROOT/src/robot_mujoco/zsibot_robots/xgb}"
MATRIX_SONIC_CANONICAL_MODEL="${MATRIX_SONIC_CANONICAL_MODEL:-$MATRIX_GEAR_SONIC_ROOT/gear_sonic/data/robot_model/model_data/g1/g1_29dof_with_hand.xml}"
MATRIX_SONIC_CANONICAL_MESHES="${MATRIX_SONIC_CANONICAL_MESHES:-$MATRIX_GEAR_SONIC_ROOT/gear_sonic/data/robot_model/model_data/g1/meshes}"
if [[ -x "$PROJECT_ROOT/.venv-audit/bin/python" ]]; then
    DEFAULT_PYTHON="$PROJECT_ROOT/.venv-audit/bin/python"
else
    DEFAULT_PYTHON="$(command -v python3)"
fi
MATRIX_SONIC_PYTHON="${MATRIX_SONIC_PYTHON:-$DEFAULT_PYTHON}"
export PATH="$(dirname "$MATRIX_SONIC_PYTHON"):$PATH"

for required in \
    "$PROJECT_ROOT/scripts/compose_overworld_scene.py" \
    "$PROJECT_ROOT/scripts/prepare_sonic_physics_model.py" \
    "$PROJECT_ROOT/scripts/run_matrix_sonic.py" \
    "$MATRIX_AUE_ROOT/src/androidtwin/control/sonic_sim/fused_sink.py" \
    "$MATRIX_GEAR_SONIC_ROOT/gear_sonic_deploy/target/release/g1_deploy_onnx_ref" \
    "$MATRIX_UNITREE_SDK2_ROOT/lib/x86_64/libunitree_sdk2.a" \
    "$MATRIX_SONIC_CANONICAL_MODEL" \
    "$MATRIX_SONIC_PYTHON"; do
    if [[ ! -e "$required" ]]; then
        echo "[ERROR] Matrix Overworld runtime dependency is missing: $required" >&2
        exit 1
    fi
done
if [[ ! -d "$MATRIX_SONIC_CANONICAL_MESHES" ]]; then
    echo "[ERROR] Canonical SONIC G1 meshes are missing: $MATRIX_SONIC_CANONICAL_MESHES" >&2
    exit 1
fi

readarray -t LAYOUT_SPAWN < <(jq -r '.acceptance.spawn_xyz[], .acceptance.spawn_yaw_rad, .acceptance.walk_vx_mps' "$LAYOUT")
SPAWN_X="${SPAWN_X:-${LAYOUT_SPAWN[0]}}"
SPAWN_Y="${SPAWN_Y:-${LAYOUT_SPAWN[1]}}"
SPAWN_Z="${SPAWN_Z:-${LAYOUT_SPAWN[2]}}"
SPAWN_YAW="${SPAWN_YAW:-${LAYOUT_SPAWN[3]}}"
VX="${VX:-${LAYOUT_SPAWN[4]}}"

RUNTIME_ROOT="${MATRIX_OVERWORLD_RUNTIME_DIR:-$PROJECT_ROOT/outputs/runtime/matrix_overworld_v1}"
NATIVE_OUTPUT="$RUNTIME_ROOT/native/scene_overworld_v1.xml"
SONIC_OUTPUT_DIR="$RUNTIME_ROOT/sonic"
STATUS_FILE="${MATRIX_OVERWORLD_STATUS_FILE:-$PROJECT_ROOT/outputs/matrix_overworld_v1_status.json}"
mkdir -p "$RUNTIME_ROOT/native" "$PROJECT_ROOT/outputs/logs"

"$MATRIX_SONIC_PYTHON" "$PROJECT_ROOT/scripts/compose_overworld_scene.py" \
    --layout "$LAYOUT" \
    --native-scene-root "$MATRIX_NATIVE_SCENE_ROOT" \
    --output-scene "$NATIVE_OUTPUT"

"$MATRIX_SONIC_PYTHON" "$PROJECT_ROOT/scripts/prepare_sonic_physics_model.py" \
    --canonical-model "$MATRIX_SONIC_CANONICAL_MODEL" \
    --canonical-meshes "$MATRIX_SONIC_CANONICAL_MESHES" \
    --native-scene "$NATIVE_OUTPUT" \
    --output-dir "$SONIC_OUTPUT_DIR"

STARTUP_ARGS=()
if [[ "$STARTUP_BAND" == "1" ]]; then
    STARTUP_ARGS+=(--startup-band)
fi

echo "[INFO] Overworld V1 physics contains six adjacent native proxies."
echo "[WARN] UE visual composition is blocked by cooked maps; render sync is intentionally disabled."
exec "$MATRIX_SONIC_PYTHON" "$PROJECT_ROOT/scripts/run_matrix_sonic.py" \
    --model "$SONIC_OUTPUT_DIR/scene_overworld_v1.xml" \
    --aue-root "$MATRIX_AUE_ROOT" \
    --gear-sonic-root "$MATRIX_GEAR_SONIC_ROOT" \
    --unitree-sdk-root "$MATRIX_UNITREE_SDK2_ROOT" \
    --control-source "$CONTROL_SOURCE" \
    --planner-bind "${MATRIX_SONIC_PLANNER_BIND:-tcp://0.0.0.0:5556}" \
    --physics-hz "${MATRIX_SONIC_PHYSICS_HZ:-200}" \
    --walk-after "$WALK_AFTER" \
    --vx "$VX" \
    --vy "$VY" \
    --yaw-rate "$YAW_RATE" \
    --max-seconds "$MAX_SECONDS" \
    --spawn-x "$SPAWN_X" \
    --spawn-y "$SPAWN_Y" \
    --spawn-z "$SPAWN_Z" \
    --spawn-yaw "$SPAWN_YAW" \
    --no-render-sync \
    "${STARTUP_ARGS[@]}" \
    --startup-band-hold "$STARTUP_BAND_HOLD" \
    --startup-band-fade "$STARTUP_BAND_FADE" \
    --status-file "$STATUS_FILE"
