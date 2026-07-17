#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
export MATRIX_PROJECT_ROOT="$PROJECT_ROOT"
ORIGINAL_ARGS=("$@")

PROFILE="${MATRIX_PROFILE:-}"
for ((index = 0; index < ${#ORIGINAL_ARGS[@]}; index++)); do
    if [[ "${ORIGINAL_ARGS[$index]}" == "--profile" ]]; then
        if ((index + 1 >= ${#ORIGINAL_ARGS[@]})); then
            echo "[ERROR] --profile requires a value" >&2
            exit 2
        fi
        PROFILE="${ORIGINAL_ARGS[$((index + 1))]}"
    fi
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
    # Profile files provide defaults with ${VAR:-...}; loading them after the
    # local file keeps explicit host overrides while recomputing runtime paths.
    # shellcheck disable=SC1090
    source "$PROFILE_FILE"
fi

SCENE_ID=21
CUSTOM_URDF="${MATRIX_G1_URDF:-}"
CUSTOM_NAME="g1_29dof"
CONTROL_SOURCE="planner"
WALK_AFTER="-1"
VX="0.30"
VY="0.0"
YAW_RATE="0.0"
MAX_SECONDS="0"
MIN_ACTIVE_SECONDS="0"
OFFSCREEN=0
STARTUP_BAND=1
STARTUP_BAND_HOLD="4"
STARTUP_BAND_FADE="3"

usage() {
    printf '%s\n' \
        "Usage: bash scripts/run_matrix_sonic.sh [--profile NAME] [options]" \
        "" \
        "Options:" \
        "  --profile NAME             Load config/hosts/NAME.env" \
        "  --scene ID                 Matrix native scene id (default: 21 ApartmentWorld)" \
        "  --urdf PATH                G1 visual URDF; defaults to the locked runtime" \
        "  --name NAME                Custom robot cache name (default: g1_29dof)" \
        "  --control-source SOURCE    planner, pico, or external (default: planner)" \
        "  --walk-after SECONDS       Start planner walking after delay; -1 stays idle" \
        "  --vx MPS                    Forward command after walk delay (default: 0.30)" \
        "  --vy MPS                    Lateral command after walk delay" \
        "  --yaw-rate RAD_S           Yaw command after walk delay" \
        "  --max-seconds SECONDS      Stop a bounded smoke automatically; 0 is unlimited" \
        "  --min-active-seconds SEC   Fail if fresh lowcmd is active for less than SEC" \
        "  --no-startup-band          Disable the temporary SONIC INIT root stabilizer" \
        "  --startup-band-hold SEC    Root hold before fade (default: 4)" \
        "  --startup-band-fade SEC    Root stabilizer fade duration (default: 3)" \
        "  --offscreen                 Start Matrix UE offscreen"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --profile) PROFILE="$2"; shift 2 ;;
        --scene) SCENE_ID="$2"; shift 2 ;;
        --urdf) CUSTOM_URDF="$2"; shift 2 ;;
        --name) CUSTOM_NAME="$2"; shift 2 ;;
        --control-source) CONTROL_SOURCE="$2"; shift 2 ;;
        --walk-after) WALK_AFTER="$2"; shift 2 ;;
        --vx) VX="$2"; shift 2 ;;
        --vy) VY="$2"; shift 2 ;;
        --yaw-rate) YAW_RATE="$2"; shift 2 ;;
        --max-seconds) MAX_SECONDS="$2"; shift 2 ;;
        --min-active-seconds) MIN_ACTIVE_SECONDS="$2"; shift 2 ;;
        --startup-band) STARTUP_BAND=1; shift ;;
        --no-startup-band) STARTUP_BAND=0; shift ;;
        --startup-band-hold) STARTUP_BAND_HOLD="$2"; shift 2 ;;
        --startup-band-fade) STARTUP_BAND_FADE="$2"; shift 2 ;;
        --offscreen) OFFSCREEN=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "[ERROR] Unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

if [[ -n "${MATRIX_CPUSET:-}" && "${MATRIX_CPUSET_APPLIED:-0}" != "1" ]]; then
    if ! command -v taskset >/dev/null; then
        echo "[ERROR] Host profile requires taskset for MATRIX_CPUSET=$MATRIX_CPUSET" >&2
        exit 1
    fi
    exec taskset -c "$MATRIX_CPUSET" /usr/bin/env MATRIX_CPUSET_APPLIED=1 \
        "$PROJECT_ROOT/scripts/run_matrix_sonic.sh" "${ORIGINAL_ARGS[@]}"
fi

find_first_dir() {
    local candidate
    for candidate in "$@"; do
        if [[ -n "$candidate" && -d "$candidate" ]]; then
            realpath "$candidate"
            return 0
        fi
    done
    return 1
}

RUNTIME_ROOT="${MATRIX_RUNTIME_ROOT:-$PROJECT_ROOT/outputs/runtime/matrix-sonic-v1}"
MATRIX_AUE_ROOT="${MATRIX_AUE_ROOT:-$(find_first_dir \
    "$RUNTIME_ROOT/aue-sim" \
    "$PROJECT_ROOT/../aue-sim" \
    "$HOME/aue-split-lab/repos/aue-sim" || true)}"
MATRIX_GEAR_SONIC_ROOT="${MATRIX_GEAR_SONIC_ROOT:-$(find_first_dir \
    "$RUNTIME_ROOT/GR00T-WholeBodyControl" \
    "$MATRIX_AUE_ROOT/third_party/GR00T-WholeBodyControl" \
    "$HOME/code_bryce/GR00T-WholeBodyControl" \
    "$HOME/metabot-workspace/GR00T-WholeBodyControl" || true)}"
MATRIX_UNITREE_SDK2_ROOT="${MATRIX_UNITREE_SDK2_ROOT:-$MATRIX_GEAR_SONIC_ROOT/gear_sonic_deploy/thirdparty/unitree_sdk2}"
MATRIX_INFERENCE_ROOT="${MATRIX_INFERENCE_ROOT:-$RUNTIME_ROOT/inference}"
MATRIX_SONIC_CANONICAL_MODEL="${MATRIX_SONIC_CANONICAL_MODEL:-$MATRIX_GEAR_SONIC_ROOT/gear_sonic/data/robot_model/model_data/g1/g1_29dof_with_hand.xml}"
MATRIX_SONIC_CANONICAL_MESHES="${MATRIX_SONIC_CANONICAL_MESHES:-$MATRIX_GEAR_SONIC_ROOT/gear_sonic/data/robot_model/model_data/g1/meshes}"
CUSTOM_URDF="${CUSTOM_URDF:-$RUNTIME_ROOT/g1-visual/g1_29dof.urdf}"

if [[ -x "$PROJECT_ROOT/.venv-audit/bin/python" ]]; then
    DEFAULT_PYTHON="$PROJECT_ROOT/.venv-audit/bin/python"
else
    DEFAULT_PYTHON="$(command -v python3)"
fi
MATRIX_SONIC_PYTHON="${MATRIX_SONIC_PYTHON:-$DEFAULT_PYTHON}"

for required in \
    "$CUSTOM_URDF" \
    "$MATRIX_AUE_ROOT/src/androidtwin/control/sonic_sim/fused_sink.py" \
    "$MATRIX_GEAR_SONIC_ROOT/gear_sonic_deploy/target/release/g1_deploy_onnx_ref" \
    "$MATRIX_UNITREE_SDK2_ROOT/lib/x86_64/libunitree_sdk2.a" \
    "$MATRIX_SONIC_PYTHON"; do
    if [[ ! -e "$required" ]]; then
        echo "[ERROR] Matrix SONIC runtime dependency is missing: $required" >&2
        exit 1
    fi
done

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
prepend_library_dir "$PROJECT_ROOT/src/UeSim/Linux/Engine/Binaries/Linux"
prepend_library_dir "$PROJECT_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux"
if [[ -n "${MATRIX_ROS_PREFIX:-}" ]]; then
    prepend_library_dir "$MATRIX_ROS_PREFIX/lib"
fi
if [[ -n "${MATRIX_NATIVE_DEPS_ROOT:-}" ]]; then
    prepend_library_dir "$MATRIX_NATIVE_DEPS_ROOT/usr/local/lib"
    prepend_library_dir "$MATRIX_NATIVE_DEPS_ROOT/usr/lib/x86_64-linux-gnu"
    prepend_library_dir "$MATRIX_NATIVE_DEPS_ROOT/usr/lib"
fi
prepend_library_dir "$MATRIX_UNITREE_SDK2_ROOT/thirdparty/lib/x86_64"
prepend_library_dir "$MATRIX_INFERENCE_ROOT/onnxruntime/lib"
prepend_library_dir "$MATRIX_INFERENCE_ROOT/TensorRT/lib"
export LD_LIBRARY_PATH

if [[ -n "${MATRIX_ROS_PREFIX:-}" && -d "$MATRIX_ROS_PREFIX" ]]; then
    export AMENT_PREFIX_PATH="${MATRIX_ROS_PREFIX}${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
    export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
fi
export TensorRT_ROOT="${TensorRT_ROOT:-$MATRIX_INFERENCE_ROOT/TensorRT}"
export PATH="$(dirname "$MATRIX_SONIC_PYTHON"):$PATH"

if [[ -n "$PROFILE" && "${MATRIX_VERIFY_RUNTIME:-1}" != "0" ]]; then
    python3 "$PROJECT_ROOT/scripts/verify_matrix_sonic_runtime.py" \
        --runtime-root "$RUNTIME_ROOT" \
        --matrix-root "$PROJECT_ROOT" \
        --profile "$PROFILE" \
        --fast
fi

mkdir -p "$PROJECT_ROOT/outputs"
exec 9>"$PROJECT_ROOT/outputs/.matrix-sonic-launch.lock"
if ! flock -n 9; then
    echo "[ERROR] Another Matrix SONIC launcher owns this checkout" >&2
    exit 1
fi

export MATRIX_SONIC=1
export MATRIX_DISABLE_MC=1
export MATRIX_AUE_ROOT MATRIX_GEAR_SONIC_ROOT MATRIX_UNITREE_SDK2_ROOT
export MATRIX_SONIC_PYTHON MATRIX_SONIC_CANONICAL_MODEL MATRIX_SONIC_CANONICAL_MESHES
export MATRIX_SONIC_CONTROL_SOURCE="$CONTROL_SOURCE"
export MATRIX_SONIC_WALK_AFTER="$WALK_AFTER"
export MATRIX_SONIC_VX="$VX"
export MATRIX_SONIC_VY="$VY"
export MATRIX_SONIC_YAW_RATE="$YAW_RATE"
export MATRIX_SONIC_MAX_SECONDS="$MAX_SECONDS"
export MATRIX_SONIC_MIN_ACTIVE_SECONDS="$MIN_ACTIVE_SECONDS"
export MATRIX_SONIC_FAIL_ON_FALL=1
export MATRIX_SONIC_STARTUP_BAND="$STARTUP_BAND"
export MATRIX_SONIC_STARTUP_BAND_HOLD="$STARTUP_BAND_HOLD"
export MATRIX_SONIC_STARTUP_BAND_FADE="$STARTUP_BAND_FADE"
if [[ -x "$RUNTIME_ROOT/bridge/g1_sonic_sim_udp_dds_bridge_accepted" ]]; then
    export ANDROIDTWIN_FUSED_SONIC_UDP_DDS_BIN="$RUNTIME_ROOT/bridge/g1_sonic_sim_udp_dds_bridge_accepted"
fi

# Matrix's upstream launcher rewrites these tracked files. Restore the exact
# pre-launch bytes so switching the same feature branch on two hosts stays clean.
CONFIG_BACKUP="$(mktemp -d /tmp/matrix-sonic-config.XXXXXX)"
MUTABLE_FILES=(
    "config/config.json"
    "src/robot_mujoco/simulate/config.yaml"
    "src/robot_mc/run_mc.sh"
)
for relative in "${MUTABLE_FILES[@]}"; do
    if [[ -f "$PROJECT_ROOT/$relative" ]]; then
        mkdir -p "$CONFIG_BACKUP/$(dirname "$relative")"
        cp -a "$PROJECT_ROOT/$relative" "$CONFIG_BACKUP/$relative"
    fi
done

restore_tracked_config() {
    local relative
    for relative in "${MUTABLE_FILES[@]}"; do
        if [[ -f "$CONFIG_BACKUP/$relative" ]]; then
            cp -a "$CONFIG_BACKUP/$relative" "$PROJECT_ROOT/$relative"
        fi
    done
    rm -rf "$CONFIG_BACKUP"
}
trap restore_tracked_config EXIT

set +e
"$PROJECT_ROOT/scripts/run_sim.sh" \
    custom "$SCENE_ID" "$OFFSCREEN" 0 1 "$CUSTOM_URDF" "$CUSTOM_NAME"
exit_code=$?
set -e
exit "$exit_code"
