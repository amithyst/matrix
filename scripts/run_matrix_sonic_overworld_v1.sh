#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
export MATRIX_PROJECT_ROOT="$PROJECT_ROOT"

LAYOUT="$PROJECT_ROOT/research/overworld_v1/layout.json"
PROFILE="${MATRIX_PROFILE:-}"
CONTROL_SOURCE="planner"
WALK_AFTER="2"
VX=""
VY="0.0"
YAW_RATE="0.0"
MAX_SECONDS="70"
STARTUP_BAND=1
STARTUP_BAND_HOLD="4"
STARTUP_BAND_FADE="3"

usage() {
    printf '%s\n' \
        "Usage: bash scripts/run_matrix_sonic_overworld_v1.sh [options]" \
        "" \
        "Runs the adjacent six-scene Overworld physics model without claiming UE visual composition." \
        "" \
        "Options:" \
        "  --profile NAME             Required for the default bounded qualification" \
        "  --control-source SOURCE    planner, pico, or external (default: planner)" \
        "  --walk-after SECONDS       Start walking after active lowcmd (default: 2)" \
        "  --vx MPS                    Forward velocity; defaults to layout acceptance value" \
        "  --vy MPS                    Lateral velocity (default: 0)" \
        "  --yaw-rate RAD_S           Yaw velocity (default: 0)" \
        "  --max-seconds SECONDS      Bounded runtime (default: 70)" \
        "  --no-startup-band          Disable temporary SONIC INIT stabilization" \
        "  --startup-band-hold SEC    Hold duration (default: 4)" \
        "  --startup-band-fade SEC    Fade duration (default: 3)"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --profile) PROFILE="$2"; shift 2 ;;
        --control-source) CONTROL_SOURCE="$2"; shift 2 ;;
        --walk-after) WALK_AFTER="$2"; shift 2 ;;
        --vx) VX="$2"; shift 2 ;;
        --vy) VY="$2"; shift 2 ;;
        --yaw-rate) YAW_RATE="$2"; shift 2 ;;
        --max-seconds) MAX_SECONDS="$2"; shift 2 ;;
        --startup-band) STARTUP_BAND=1; shift ;;
        --no-startup-band) STARTUP_BAND=0; shift ;;
        --startup-band-hold) STARTUP_BAND_HOLD="$2"; shift 2 ;;
        --startup-band-fade) STARTUP_BAND_FADE="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "[ERROR] Unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

if [[ -f "$PROJECT_ROOT/.matrix/local.env" ]]; then
    # shellcheck disable=SC1091
    source "$PROJECT_ROOT/.matrix/local.env"
fi
if [[ -n "$PROFILE" ]]; then
    PROFILE_FILE="$PROJECT_ROOT/config/hosts/$PROFILE.env"
    if [[ ! -f "$PROFILE_FILE" ]]; then
        echo "[ERROR] Unknown host profile: $PROFILE" >&2
        exit 2
    fi
    # shellcheck disable=SC1090
    source "$PROFILE_FILE"
fi

if [[ ! -f "$LAYOUT" ]]; then
    echo "[ERROR] Overworld layout is missing: $LAYOUT" >&2
    exit 2
fi
for required_command in flock git grep jq realpath; do
    if ! command -v "$required_command" >/dev/null 2>&1; then
        echo "[ERROR] $required_command is required by the Overworld launcher" >&2
        exit 1
    fi
done
MATRIX_SONIC_HOST_LOCK="${MATRIX_SONIC_HOST_LOCK:-/tmp/matrix-sonic-${UID}.lock}"
exec 9>"$MATRIX_SONIC_HOST_LOCK"
if ! flock -n 9; then
    echo "[ERROR] Another Matrix SONIC launcher owns this host: $MATRIX_SONIC_HOST_LOCK" >&2
    exit 1
fi
export MATRIX_SONIC_HOST_LOCK_FD=9
STATUS_FILE="${MATRIX_OVERWORLD_STATUS_FILE:-$PROJECT_ROOT/outputs/matrix_overworld_v1_status.json}"
rm -f -- "$STATUS_FILE"
LOCK_FILE="$PROJECT_ROOT/config/runtime/matrix-sonic.lock.json"
if ! QUALIFICATION_REQUESTED="$(python3 - "$MAX_SECONDS" <<'PY'
import math
import sys
try:
    value = float(sys.argv[1])
except ValueError as exc:
    raise SystemExit(f"invalid --max-seconds: {sys.argv[1]}") from exc
if not math.isfinite(value) or value < 0.0:
    raise SystemExit("--max-seconds must be non-negative and finite")
print("1" if value > 0.0 else "0")
PY
)"; then
    exit 2
fi
QUALIFICATION_ARGS=()
if [[ "$QUALIFICATION_REQUESTED" == "1" ]]; then
    if [[ -z "$PROFILE" ]]; then
        echo "[ERROR] Bounded Overworld qualification requires --profile" >&2
        exit 2
    fi
    if [[ "${MATRIX_VERIFY_RUNTIME:-1}" == "0" ]]; then
        echo "[ERROR] Bounded Overworld qualification cannot disable runtime verification" >&2
        exit 2
    fi
    if [[ -n "$(git -C "$PROJECT_ROOT" status --porcelain --untracked-files=normal)" ]]; then
        echo "[ERROR] Bounded Overworld qualification requires a clean Matrix Git checkout" >&2
        exit 2
    fi
    readarray -t QUALIFICATION_HASHES < <(python3 - "$LOCK_FILE" "$LAYOUT" <<'PY'
import hashlib
from pathlib import Path
import sys
for value in sys.argv[1:]:
    print(hashlib.sha256(Path(value).read_bytes()).hexdigest())
PY
    )
    if [[ "${#QUALIFICATION_HASHES[@]}" != "2" ]]; then
        echo "[ERROR] Failed to hash the Overworld qualification inputs" >&2
        exit 1
    fi
    QUALIFICATION_ARGS+=(
        --qualified-runtime
        --qualification-profile "$PROFILE"
        --runtime-lock-sha256 "${QUALIFICATION_HASHES[0]}"
        --scenario-layout-sha256 "${QUALIFICATION_HASHES[1]}"
        --matrix-commit "$(git -C "$PROJECT_ROOT" rev-parse HEAD)"
        --verification-receipt "$PROJECT_ROOT/outputs/runtime-verification-overworld-${PROFILE}-launch-$$.json"
    )
    VERIFICATION_RECEIPT="$PROJECT_ROOT/outputs/runtime-verification-overworld-${PROFILE}-launch-$$.json"
    rm -f -- "$VERIFICATION_RECEIPT"
fi
readarray -t ACCEPTANCE_LOCK < <(jq -r '
    .acceptance.low_cmd_fresh_timeout_seconds,
    .acceptance.active_lowcmd_seconds_min,
    .acceptance.root_displacement_xy_min_m,
    .acceptance.physics_hz_min,
    .acceptance.rtf_min,
    .acceptance.instability_resets_max
' "$LOCK_FILE")
if [[ "${#ACCEPTANCE_LOCK[@]}" != "6" ]] \
    || printf '%s\n' "${ACCEPTANCE_LOCK[@]}" | grep -qx null; then
    echo "[ERROR] Runtime acceptance lock is incomplete: $LOCK_FILE" >&2
    exit 1
fi
LOW_CMD_FRESH_TIMEOUT_SECONDS="${ACCEPTANCE_LOCK[0]}"
MIN_ACTIVE_SECONDS="${ACCEPTANCE_LOCK[1]}"
MIN_DISPLACEMENT_M="${ACCEPTANCE_LOCK[2]}"
MIN_PHYSICS_HZ="${ACCEPTANCE_LOCK[3]}"
MIN_RTF="${ACCEPTANCE_LOCK[4]}"
MAX_RESETS="${ACCEPTANCE_LOCK[5]}"

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

MATRIX_RUNTIME_ROOT="${MATRIX_RUNTIME_ROOT:-$PROJECT_ROOT/outputs/runtime/matrix-sonic-native-v2}"
MATRIX_SONIC_ROOT="${MATRIX_SONIC_ROOT:-$(find_first_dir \
    "$MATRIX_RUNTIME_ROOT/GR00T-WholeBodyControl" \
    "$HOME/worktrees/sonic-matrix-native-final" \
    "$HOME/GR00T-WholeBodyControl" \
    "$HOME/metabot-workspace/GR00T-WholeBodyControl" || true)}"
MATRIX_UNITREE_SDK2_ROOT="${MATRIX_UNITREE_SDK2_ROOT:-$MATRIX_SONIC_ROOT/gear_sonic_deploy/thirdparty/unitree_sdk2}"
MATRIX_INFERENCE_ROOT="${MATRIX_INFERENCE_ROOT:-$MATRIX_RUNTIME_ROOT/inference}"
MATRIX_NATIVE_SCENE_ROOT="${MATRIX_NATIVE_SCENE_ROOT:-$PROJECT_ROOT/src/robot_mujoco/zsibot_robots/xgb}"
MATRIX_SONIC_CANONICAL_MODEL="${MATRIX_SONIC_CANONICAL_MODEL:-$MATRIX_SONIC_ROOT/gear_sonic/data/robot_model/model_data/g1/g1_29dof_with_hand.xml}"
MATRIX_SONIC_CANONICAL_MESHES="${MATRIX_SONIC_CANONICAL_MESHES:-$MATRIX_SONIC_ROOT/gear_sonic/data/robot_model/model_data/g1/meshes}"
if [[ -x "$PROJECT_ROOT/.venv-audit/bin/python" ]]; then
    DEFAULT_PYTHON="$PROJECT_ROOT/.venv-audit/bin/python"
else
    DEFAULT_PYTHON="$(command -v python3)"
fi
MATRIX_SONIC_PYTHON="${MATRIX_SONIC_PYTHON:-$DEFAULT_PYTHON}"
MATRIX_PICO_PYTHON="${MATRIX_PICO_PYTHON:-$MATRIX_SONIC_PYTHON}"
export PATH="$(dirname "$MATRIX_SONIC_PYTHON"):$PATH"

for required in \
    "$PROJECT_ROOT/scripts/compose_overworld_scene.py" \
    "$PROJECT_ROOT/scripts/prepare_sonic_physics_model.py" \
    "$PROJECT_ROOT/scripts/run_matrix_sonic.py" \
    "$MATRIX_SONIC_ROOT/gear_sonic/scripts/run_sim_loop.py" \
    "$MATRIX_SONIC_ROOT/gear_sonic_deploy/target/release/g1_deploy_onnx_ref" \
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

prepend_library_dir() {
    local directory="$1"
    if [[ -d "$directory" ]]; then
        LD_LIBRARY_PATH="$directory${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    fi
}

LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"
MATRIX_CUDA_ROOT="${MATRIX_CUDA_ROOT:-/usr/local/cuda}"
prepend_library_dir "$MATRIX_CUDA_ROOT/lib64"
prepend_library_dir "$MATRIX_CUDA_ROOT/lib"
prepend_library_dir "$MATRIX_UNITREE_SDK2_ROOT/thirdparty/lib/x86_64"
prepend_library_dir "$MATRIX_SONIC_ROOT/external_dependencies/XRoboToolkit-PC-Service-Pybind_X86_and_ARM64/lib"
prepend_library_dir "$MATRIX_INFERENCE_ROOT/onnxruntime/lib"
prepend_library_dir "$MATRIX_INFERENCE_ROOT/TensorRT/lib"
if [[ -n "${MATRIX_ROS_PREFIX:-}" ]]; then
    prepend_library_dir "$MATRIX_ROS_PREFIX/lib"
fi
if [[ -n "${MATRIX_NATIVE_DEPS_ROOT:-}" ]]; then
    prepend_library_dir "$MATRIX_NATIVE_DEPS_ROOT/usr/local/lib"
    prepend_library_dir "$MATRIX_NATIVE_DEPS_ROOT/usr/lib/x86_64-linux-gnu"
    prepend_library_dir "$MATRIX_NATIVE_DEPS_ROOT/usr/lib"
fi
export LD_LIBRARY_PATH
export TensorRT_ROOT="${TensorRT_ROOT:-$MATRIX_INFERENCE_ROOT/TensorRT}"
export PYTHONNOUSERSITE=1

if [[ "$CONTROL_SOURCE" == "pico" \
    && "${MATRIX_VERIFY_RUNTIME:-1}" != "0" \
    && -z "$PROFILE" ]]; then
    echo "[ERROR] Locked PICO acceptance requires --profile for runtime verification" >&2
    exit 2
fi
if [[ -n "$PROFILE" && "${MATRIX_VERIFY_RUNTIME:-1}" != "0" ]]; then
    VERIFY_RUNTIME_ARGS=(
        --runtime-root "$MATRIX_RUNTIME_ROOT"
        --matrix-root "$PROJECT_ROOT"
        --sonic-root "$MATRIX_SONIC_ROOT"
        --python "$MATRIX_SONIC_PYTHON"
        --profile "$PROFILE"
        --fast
    )
    if [[ "$CONTROL_SOURCE" == "pico" ]]; then
        if [[ -z "${MATRIX_PICO_WHEEL:-}" ]]; then
            echo "[ERROR] MATRIX_PICO_WHEEL is required for PICO artifact verification" >&2
            exit 1
        fi
        VERIFY_RUNTIME_ARGS+=(
            --pico-python "$MATRIX_PICO_PYTHON"
            --pico-wheel "$MATRIX_PICO_WHEEL"
        )
    fi
    if [[ "$QUALIFICATION_REQUESTED" == "1" ]]; then
        VERIFY_RUNTIME_ARGS+=(--json-output "$VERIFICATION_RECEIPT")
    fi
    python3 "$PROJECT_ROOT/scripts/verify_matrix_sonic_runtime.py" \
        "${VERIFY_RUNTIME_ARGS[@]}"
fi

readarray -t LAYOUT_SPAWN < <(jq -r '
    .acceptance.spawn_xyz[],
    .acceptance.spawn_yaw_rad,
    .acceptance.walk_vx_mps,
    .acceptance.final_x_min,
    (.acceptance.final_x_min - .acceptance.spawn_xyz[0])
' "$LAYOUT")
if [[ "${#LAYOUT_SPAWN[@]}" != "7" ]] \
    || printf '%s\n' "${LAYOUT_SPAWN[@]}" | grep -qx null; then
    echo "[ERROR] Overworld acceptance layout is incomplete: $LAYOUT" >&2
    exit 1
fi
SPAWN_X="${LAYOUT_SPAWN[0]}"
SPAWN_Y="${LAYOUT_SPAWN[1]}"
SPAWN_Z="${LAYOUT_SPAWN[2]}"
SPAWN_YAW="${LAYOUT_SPAWN[3]}"
VX="${VX:-${LAYOUT_SPAWN[4]}}"
FINAL_X_MIN="${LAYOUT_SPAWN[5]}"
MIN_FORWARD_X_M="${LAYOUT_SPAWN[6]}"

RUNTIME_ROOT="${MATRIX_OVERWORLD_RUNTIME_DIR:-$PROJECT_ROOT/outputs/runtime/matrix_overworld_v1}"
NATIVE_OUTPUT="$RUNTIME_ROOT/native/scene_overworld_v1.xml"
SONIC_OUTPUT_DIR="$RUNTIME_ROOT/sonic"
mkdir -p "$RUNTIME_ROOT/native" "$PROJECT_ROOT/outputs/logs"

"$MATRIX_SONIC_PYTHON" "$PROJECT_ROOT/scripts/compose_overworld_scene.py" \
    --layout "$LAYOUT" \
    --native-scene-root "$MATRIX_NATIVE_SCENE_ROOT" \
    --output-scene "$NATIVE_OUTPUT"

"$MATRIX_SONIC_PYTHON" "$PROJECT_ROOT/scripts/prepare_sonic_physics_model.py" \
    --canonical-model "$MATRIX_SONIC_CANONICAL_MODEL" \
    --canonical-meshes "$MATRIX_SONIC_CANONICAL_MESHES" \
    --native-scene "$NATIVE_OUTPUT" \
    --output-dir "$SONIC_OUTPUT_DIR" \
    --spawn-x "$SPAWN_X" \
    --spawn-y "$SPAWN_Y" \
    --spawn-z "$SPAWN_Z" \
    --spawn-yaw "$SPAWN_YAW"

STARTUP_ARGS=()
if [[ "$STARTUP_BAND" == "1" ]]; then
    STARTUP_ARGS+=(--startup-band)
fi

echo "[INFO] Overworld V1 physics contains six adjacent native proxies."
echo "[WARN] UE visual composition is blocked by cooked maps; render sync is intentionally disabled."
exec "$MATRIX_SONIC_PYTHON" "$PROJECT_ROOT/scripts/run_matrix_sonic.py" \
    --model "$SONIC_OUTPUT_DIR/scene_overworld_v1.xml" \
    --sonic-root "$MATRIX_SONIC_ROOT" \
    --control-source "$CONTROL_SOURCE" \
    --planner-bind "${MATRIX_SONIC_PLANNER_BIND:-tcp://127.0.0.1:5556}" \
    --pico-python "$MATRIX_PICO_PYTHON" \
    --dds-interface lo \
    --physics-hz "${MATRIX_SONIC_PHYSICS_HZ:-200}" \
    --walk-after "$WALK_AFTER" \
    --vx "$VX" \
    --vy "$VY" \
    --yaw-rate "$YAW_RATE" \
    --max-seconds "$MAX_SECONDS" \
    --low-cmd-fresh-timeout-seconds "$LOW_CMD_FRESH_TIMEOUT_SECONDS" \
    --min-active-seconds "$MIN_ACTIVE_SECONDS" \
    --min-displacement-m "$MIN_DISPLACEMENT_M" \
    --min-final-x "$FINAL_X_MIN" \
    --min-forward-x-m "$MIN_FORWARD_X_M" \
    --min-physics-hz "$MIN_PHYSICS_HZ" \
    --min-rtf "$MIN_RTF" \
    --fail-on-fall \
    --max-resets "$MAX_RESETS" \
    "${QUALIFICATION_ARGS[@]}" \
    --no-render-sync \
    "${STARTUP_ARGS[@]}" \
    --startup-band-hold "$STARTUP_BAND_HOLD" \
    --startup-band-fade "$STARTUP_BAND_FADE" \
    --status-file "$STATUS_FILE"
