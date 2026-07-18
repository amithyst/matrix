#!/usr/bin/env bash
set -euo pipefail
ORIGINAL_ENVIRONMENT=()
while IFS= read -r -d '' entry; do
    ORIGINAL_ENVIRONMENT+=("$entry")
done < "/proc/$$/environ"
if ((BASH_VERSINFO[0] < 5 \
    || (BASH_VERSINFO[0] == 5 && BASH_VERSINFO[1] < 1))); then
    echo "[ERROR] Matrix restart supervision requires Bash 5.1 or newer" >&2
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
export MATRIX_PROJECT_ROOT="$PROJECT_ROOT"
# The audit venv is a locked runtime artifact.  Interactive, unbounded runs
# import the same packages as qualification runs, so they must not leave
# unowned __pycache__ files that make the next preflight fail content closure.
export PYTHONDONTWRITEBYTECODE=1
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

# shellcheck disable=SC1091
source "$SCRIPT_DIR/matrix_local_env.sh"
if ! load_matrix_local_env "$PROJECT_ROOT"; then
    exit 2
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
GAME_INPUT_SOURCE="${MATRIX_GAME_INPUT_SOURCE:-auto}"
GAME_CAMERA_YAW_SOURCE="${MATRIX_GAME_CAMERA_YAW_SOURCE:-fixed}"
GAME_LOOK_BUTTON="${MATRIX_GAME_LOOK_BUTTON:-left}"
GAME_INITIAL_CAMERA_YAW_DEG="${MATRIX_GAME_INITIAL_CAMERA_YAW_DEG:-0.0}"
GAME_MOUSE_SENSITIVITY_DEG="${MATRIX_GAME_MOUSE_SENSITIVITY_DEG:-0.12}"
GAME_CAMERA_YAW_SIGN="${MATRIX_GAME_CAMERA_YAW_SIGN:--1}"
GAME_CAMERA_YAW_OFFSET_DEG="${MATRIX_GAME_CAMERA_YAW_OFFSET_DEG:-0.0}"
GAME_CARLA_HOST="${MATRIX_GAME_CARLA_HOST:-127.0.0.1}"
GAME_CARLA_PORT="${MATRIX_GAME_CARLA_PORT:-2000}"
GAMEPAD_LOOK_YAW_RATE_DEG_S="${MATRIX_GAMEPAD_LOOK_YAW_RATE_DEG_S:-120.0}"
GAMEPAD_LOOK_PITCH_RATE_DEG_S="${MATRIX_GAMEPAD_LOOK_PITCH_RATE_DEG_S:-90.0}"
GAMEPAD_LOOK_DEADZONE="${MATRIX_GAMEPAD_LOOK_DEADZONE:-0.12}"
GAMEPAD_LOOK_MIN_PITCH_DEG="${MATRIX_GAMEPAD_LOOK_MIN_PITCH_DEG:--80.0}"
GAMEPAD_LOOK_MAX_PITCH_DEG="${MATRIX_GAMEPAD_LOOK_MAX_PITCH_DEG:-60.0}"
GAME_MAX_SPEED="${MATRIX_GAME_MAX_SPEED:-0.30}"
GAME_INPUT_TIMEOUT="${MATRIX_GAME_INPUT_TIMEOUT:-0.15}"
WALK_AFTER="-1"
VX="0.30"
VY="0.0"
YAW_RATE="0.0"
MAX_SECONDS="0"
LOCK_FILE="$PROJECT_ROOT/config/runtime/matrix-sonic.lock.json"
read_acceptance_lock() {
    /usr/bin/python3 -I - "$LOCK_FILE" "$1" <<'PY'
import json
import sys
print(json.load(open(sys.argv[1], encoding="utf-8"))["acceptance"][sys.argv[2]])
PY
}
MIN_ACTIVE_SECONDS="${MATRIX_SONIC_MIN_ACTIVE_SECONDS:-$(read_acceptance_lock active_lowcmd_seconds_min)}"
MIN_DISPLACEMENT_M="${MATRIX_SONIC_MIN_DISPLACEMENT_M:-$(read_acceptance_lock root_displacement_xy_min_m)}"
LOW_CMD_FRESH_TIMEOUT_SECONDS="${MATRIX_SONIC_LOW_CMD_FRESH_TIMEOUT_SECONDS:-$(read_acceptance_lock low_cmd_fresh_timeout_seconds)}"
MIN_PHYSICS_HZ="${MATRIX_SONIC_MIN_PHYSICS_HZ:-$(read_acceptance_lock physics_hz_min)}"
MIN_RTF="${MATRIX_SONIC_MIN_RTF:-$(read_acceptance_lock rtf_min)}"
MAX_RESETS="${MATRIX_SONIC_MAX_RESETS:-$(read_acceptance_lock instability_resets_max)}"
OFFSCREEN=0
STARTUP_BAND=1
STARTUP_BAND_HOLD="4"
STARTUP_BAND_FADE="3"

usage() {
    printf '%s\n' \
        "Usage: bash scripts/run_matrix_sonic.sh [--profile NAME] [options]" \
        "" \
        "Options:" \
        "  --profile NAME             Load host defaults; required for bounded qualification" \
        "  --scene ID                 Matrix native scene id (default: 21 ApartmentWorld)" \
        "  --urdf PATH                G1 visual URDF; defaults to the locked runtime" \
        "  --name NAME                Custom robot cache name (default: g1_29dof)" \
        "  --control-source SOURCE    planner, game, pico, or external (default: planner)" \
        "  --game-input-source SOURCE auto, keyboard, or gamepad (default: auto)" \
        "  --game-camera-yaw-source S x11-mirror, carla, or fixed (default: fixed)" \
        "  --game-look-button BUTTON  Camera drag button: left, middle, or right" \
        "  --game-initial-yaw DEG     Initial provider/UE camera yaw before sign and offset" \
        "  --game-mouse-sensitivity DEG_PER_PX  Calibrated mirror scale (default: 0.12)" \
        "  --game-camera-yaw-sign N   Provider-to-SONIC sign: -1 or 1" \
        "  --game-camera-yaw-offset DEG  Provider-to-SONIC zero-frame offset" \
        "  --game-carla-host HOST     Optional fail-closed CARLA spectator host" \
        "  --game-carla-port PORT     Optional CARLA spectator RPC port" \
        "  --gamepad-look-yaw-rate DEG_S    Full-stick spectator yaw rate" \
        "  --gamepad-look-pitch-rate DEG_S  Full-stick spectator pitch rate" \
        "  --gamepad-look-deadzone VALUE    Radial right-stick deadzone" \
        "  --gamepad-look-min-pitch DEG     Spectator pitch lower limit" \
        "  --gamepad-look-max-pitch DEG     Spectator pitch upper limit" \
        "  --game-max-speed MPS       Maximum interactive speed (default: 0.30)" \
        "  --game-input-timeout SEC   Deadman timeout (default: 0.15)" \
        "  --walk-after SECONDS       Start planner walking after delay; -1 stays idle" \
        "  --vx MPS                    Forward command after walk delay (default: 0.30)" \
        "  --vy MPS                    Lateral command after walk delay" \
        "  --yaw-rate RAD_S           Yaw command after walk delay" \
        "  --max-seconds SECONDS      Stop a bounded smoke automatically; 0 is unlimited" \
        "  --min-active-seconds SEC   Fail if fresh lowcmd is active for less than SEC" \
        "  --min-displacement-m M     Fail if final XY displacement is below M" \
        "  --low-cmd-fresh-timeout-seconds SEC  Maximum accepted DDS lowcmd age (default: 0.1)" \
        "  --min-physics-hz HZ         Acceptance floor (default: 195)" \
        "  --min-rtf VALUE             Acceptance floor (default: 0.95)" \
        "  --max-resets COUNT          Maximum authoritative SONIC reset count" \
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
        --game-input-source) GAME_INPUT_SOURCE="$2"; shift 2 ;;
        --game-camera-yaw-source) GAME_CAMERA_YAW_SOURCE="$2"; shift 2 ;;
        --game-look-button) GAME_LOOK_BUTTON="$2"; shift 2 ;;
        --game-initial-yaw) GAME_INITIAL_CAMERA_YAW_DEG="$2"; shift 2 ;;
        --game-mouse-sensitivity) GAME_MOUSE_SENSITIVITY_DEG="$2"; shift 2 ;;
        --game-camera-yaw-sign) GAME_CAMERA_YAW_SIGN="$2"; shift 2 ;;
        --game-camera-yaw-offset) GAME_CAMERA_YAW_OFFSET_DEG="$2"; shift 2 ;;
        --game-carla-host) GAME_CARLA_HOST="$2"; shift 2 ;;
        --game-carla-port) GAME_CARLA_PORT="$2"; shift 2 ;;
        --gamepad-look-yaw-rate) GAMEPAD_LOOK_YAW_RATE_DEG_S="$2"; shift 2 ;;
        --gamepad-look-pitch-rate) GAMEPAD_LOOK_PITCH_RATE_DEG_S="$2"; shift 2 ;;
        --gamepad-look-deadzone) GAMEPAD_LOOK_DEADZONE="$2"; shift 2 ;;
        --gamepad-look-min-pitch) GAMEPAD_LOOK_MIN_PITCH_DEG="$2"; shift 2 ;;
        --gamepad-look-max-pitch) GAMEPAD_LOOK_MAX_PITCH_DEG="$2"; shift 2 ;;
        --game-max-speed) GAME_MAX_SPEED="$2"; shift 2 ;;
        --game-input-timeout) GAME_INPUT_TIMEOUT="$2"; shift 2 ;;
        --walk-after) WALK_AFTER="$2"; shift 2 ;;
        --vx) VX="$2"; shift 2 ;;
        --vy) VY="$2"; shift 2 ;;
        --yaw-rate) YAW_RATE="$2"; shift 2 ;;
        --max-seconds) MAX_SECONDS="$2"; shift 2 ;;
        --min-active-seconds) MIN_ACTIVE_SECONDS="$2"; shift 2 ;;
        --min-displacement-m) MIN_DISPLACEMENT_M="$2"; shift 2 ;;
        --low-cmd-fresh-timeout-seconds) LOW_CMD_FRESH_TIMEOUT_SECONDS="$2"; shift 2 ;;
        --min-physics-hz) MIN_PHYSICS_HZ="$2"; shift 2 ;;
        --min-rtf) MIN_RTF="$2"; shift 2 ;;
        --max-resets) MAX_RESETS="$2"; shift 2 ;;
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

if ! command -v flock >/dev/null 2>&1; then
    echo "[ERROR] flock is required by the Matrix SONIC launcher" >&2
    exit 1
fi
MATRIX_SONIC_HOST_LOCK="${MATRIX_SONIC_HOST_LOCK:-/tmp/matrix-sonic-${UID}.lock}"
if [[ "${MATRIX_SONIC_RESTART_LOCK_FD:-}" == "9" ]]; then
    inherited_target="$(readlink -f "/proc/$$/fd/9" 2>/dev/null || true)"
    expected_target="$(realpath -m "$MATRIX_SONIC_HOST_LOCK")"
    if [[ "$inherited_target" != "$expected_target" ]] || ! flock -n 9; then
        echo "[ERROR] Restart did not inherit the verified Matrix SONIC lock" >&2
        exit 1
    fi
    unset MATRIX_SONIC_RESTART_LOCK_FD
else
    exec 9>"$MATRIX_SONIC_HOST_LOCK"
    if ! flock -n 9; then
        echo "[ERROR] Another Matrix SONIC launcher owns this host: $MATRIX_SONIC_HOST_LOCK" >&2
        exit 1
    fi
fi
export MATRIX_SONIC_HOST_LOCK_FD=9

# The host lock serializes every mutation of Saved/Paks.  Clear a verified
# leftover from an interrupted generation before any runtime audit, then verify
# the configured external bundle while the launcher still owns that lock.
MATRIX_CENTERED_CAMERA_OVERLAY_CONTRACT="${MATRIX_CENTERED_CAMERA_OVERLAY_CONTRACT:-$PROJECT_ROOT/config/runtime/matrix-centered-camera-overlay-v3.json}"
MATRIX_CENTERED_CAMERA_OVERLAY_BUNDLE="${MATRIX_CENTERED_CAMERA_OVERLAY_BUNDLE:-}"
/usr/bin/python3 -I "$PROJECT_ROOT/scripts/matrix_ue_overlay.py" \
    purge-stale \
    --contract "$MATRIX_CENTERED_CAMERA_OVERLAY_CONTRACT" \
    --project-root "$PROJECT_ROOT"
if [[ -n "$MATRIX_CENTERED_CAMERA_OVERLAY_BUNDLE" ]]; then
    /usr/bin/python3 -I "$PROJECT_ROOT/scripts/matrix_ue_overlay.py" \
        verify-bundle \
        --contract "$MATRIX_CENTERED_CAMERA_OVERLAY_CONTRACT" \
        --bundle "$MATRIX_CENTERED_CAMERA_OVERLAY_BUNDLE"
    echo "[INFO] Verified Matrix centered-camera overlay bundle: " \
        "$MATRIX_CENTERED_CAMERA_OVERLAY_BUNDLE"
fi
export MATRIX_CENTERED_CAMERA_OVERLAY_CONTRACT
export MATRIX_CENTERED_CAMERA_OVERLAY_BUNDLE

MATRIX_MOUSE_SETTINGS_FILE="${MATRIX_MOUSE_SETTINGS_FILE:-${XDG_CONFIG_HOME:-$HOME/.config}/matrix/mouse-control.json}"
if [[ "$MATRIX_MOUSE_SETTINGS_FILE" != /* ]]; then
    echo "[ERROR] MATRIX_MOUSE_SETTINGS_FILE must be absolute" >&2
    exit 2
fi
MATRIX_MOUSE_SETTINGS_FILE="$(realpath -m "$MATRIX_MOUSE_SETTINGS_FILE")"
MOUSE_LAUNCH_FIELDS="$(
    /usr/bin/python3 -I "$PROJECT_ROOT/scripts/matrix_mouse_settings.py" \
        launch-fields --file "$MATRIX_MOUSE_SETTINGS_FILE"
)"
IFS=$'\t' read -r MATRIX_MOUSE_APPLIED_PROFILE \
    MATRIX_MOUSE_APPLIED_SPEED_SCALE MATRIX_MOUSE_SETTINGS_LOAD_STATUS \
    <<<"$MOUSE_LAUNCH_FIELDS"
if [[ "$MATRIX_MOUSE_APPLIED_PROFILE" != "local" \
    && "$MATRIX_MOUSE_APPLIED_PROFILE" != "remote" ]] \
    || [[ ! "$MATRIX_MOUSE_APPLIED_SPEED_SCALE" =~ ^(0\.[2-9][0-9]*|1\.0+)$ ]] \
    || [[ "$MATRIX_MOUSE_SETTINGS_LOAD_STATUS" != "loaded" \
        && "$MATRIX_MOUSE_SETTINGS_LOAD_STATUS" != "missing" \
        && "$MATRIX_MOUSE_SETTINGS_LOAD_STATUS" != "invalid" ]]; then
    echo "[ERROR] Invalid mouse-settings helper output" >&2
    exit 2
fi
export MATRIX_MOUSE_SETTINGS_FILE MATRIX_MOUSE_APPLIED_PROFILE
export MATRIX_MOUSE_APPLIED_SPEED_SCALE MATRIX_MOUSE_SETTINGS_LOAD_STATUS
echo "[INFO] Mouse launch profile: $MATRIX_MOUSE_APPLIED_PROFILE " \
    "scale=$MATRIX_MOUSE_APPLIED_SPEED_SCALE status=$MATRIX_MOUSE_SETTINGS_LOAD_STATUS"
MATRIX_SONIC_STATUS_FILE="${MATRIX_SONIC_STATUS_FILE:-$PROJECT_ROOT/outputs/matrix_sonic_status.json}"
export MATRIX_SONIC_STATUS_FILE
rm -f -- "$MATRIX_SONIC_STATUS_FILE"
if ! QUALIFICATION_REQUESTED="$(/usr/bin/python3 -I - "$MAX_SECONDS" <<'PY'
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
MATRIX_SONIC_QUALIFIED_RUNTIME=0
MATRIX_SONIC_QUALIFICATION_PROFILE=""
MATRIX_SONIC_RUNTIME_LOCK_SHA256=""
MATRIX_SONIC_MATRIX_COMMIT=""
MATRIX_SONIC_VERIFICATION_RECEIPT=""
if [[ "$QUALIFICATION_REQUESTED" == "1" ]]; then
    if [[ -z "$PROFILE" ]]; then
        echo "[ERROR] Bounded qualification requires --profile" >&2
        exit 2
    fi
    if [[ "${MATRIX_VERIFY_RUNTIME:-1}" == "0" ]]; then
        echo "[ERROR] Bounded qualification cannot disable runtime verification" >&2
        exit 2
    fi
    if [[ "$CONTROL_SOURCE" == "game" ]]; then
        if [[ "$GAME_CAMERA_YAW_SOURCE" == "fixed" ]]; then
            echo "[ERROR] Bounded game-control qualification requires an observed or calibrated camera yaw source; fixed is not admissible" >&2
            exit 2
        fi
        if [[ -n "${MATRIX_GAME_INPUT_PYTHON:-}" ]]; then
            echo "[ERROR] Bounded game-control qualification rejects MATRIX_GAME_INPUT_PYTHON; the provider uses the verified runtime Python" >&2
            exit 2
        fi
        GAME_NO_INPUT_PROVIDER_VALUE="${MATRIX_GAME_NO_INPUT_PROVIDER:-0}"
        case "${GAME_NO_INPUT_PROVIDER_VALUE,,}" in
            1|true|yes|on)
                echo "[ERROR] Bounded game-control qualification cannot disable the supervised input provider" >&2
                exit 2
                ;;
            0|false|no|off|"") ;;
            *)
                echo "[ERROR] MATRIX_GAME_NO_INPUT_PROVIDER must be a boolean" >&2
                exit 2
                ;;
        esac
    fi
    for launcher_root in "${SIM_LAUNCHER_ROOT:-}" "${MATRIX_ROOT:-}"; do
        if [[ -n "$launcher_root" ]] \
            && [[ "$(realpath -m "$launcher_root")" != "$PROJECT_ROOT" ]]; then
            echo "[ERROR] Bounded qualification rejects an alternate Matrix launcher root: $launcher_root" >&2
            exit 2
        fi
    done
    if [[ "${SIM_LAUNCHER_SKIP_CUSTOM_URDF_WRAPPER:-0}" == "1" \
        || "${MATRIX_SKIP_ENV_CHECK:-0}" == "1" ]]; then
        echo "[ERROR] Bounded qualification rejects launcher skip overrides" >&2
        exit 2
    fi
    # Pin every launcher hop to this verified checkout. The custom wrapper sets
    # its private recursion flag only for the final handoff back to run_sim.sh.
    export SIM_LAUNCHER_ROOT="$PROJECT_ROOT"
    export MATRIX_ROOT="$PROJECT_ROOT"
    export SIM_LAUNCHER_SKIP_CUSTOM_URDF_WRAPPER=0
    export MATRIX_SKIP_ENV_CHECK=0
    # Redirect any historical source-tree __pycache__ away from the pinned
    # SONIC sources.  Bytecode writes are already disabled for every launch.
    export PYTHONPYCACHEPREFIX="$(mktemp -d /tmp/matrix-qualified-pycache.XXXXXX)"
    if ! command -v git >/dev/null 2>&1 \
        || [[ -n "$(git -C "$PROJECT_ROOT" status --porcelain --untracked-files=normal)" ]]; then
        echo "[ERROR] Bounded qualification requires a clean Matrix Git checkout" >&2
        exit 2
    fi
    MATRIX_SONIC_QUALIFIED_RUNTIME=1
    MATRIX_SONIC_QUALIFICATION_PROFILE="$PROFILE"
    MATRIX_SONIC_RUNTIME_LOCK_SHA256="$(/usr/bin/python3 -I - "$LOCK_FILE" <<'PY'
import hashlib
from pathlib import Path
import sys
print(hashlib.sha256(Path(sys.argv[1]).read_bytes()).hexdigest())
PY
)"
    MATRIX_SONIC_MATRIX_COMMIT="$(git -C "$PROJECT_ROOT" rev-parse HEAD)"
    MATRIX_SONIC_VERIFICATION_RECEIPT="$PROJECT_ROOT/outputs/runtime-verification-${PROFILE}-launch-$$.json"
    rm -f -- "$MATRIX_SONIC_VERIFICATION_RECEIPT"
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

RUNTIME_ROOT="${MATRIX_RUNTIME_ROOT:-$PROJECT_ROOT/outputs/runtime/matrix-sonic-native-v2}"
MATRIX_SONIC_ROOT="${MATRIX_SONIC_ROOT:-$(find_first_dir \
    "$RUNTIME_ROOT/GR00T-WholeBodyControl" \
    "$HOME/worktrees/sonic-matrix-native-final" \
    "$HOME/GR00T-WholeBodyControl" \
    "$HOME/metabot-workspace/GR00T-WholeBodyControl" || true)}"
MATRIX_UNITREE_SDK2_ROOT="${MATRIX_UNITREE_SDK2_ROOT:-$MATRIX_SONIC_ROOT/gear_sonic_deploy/thirdparty/unitree_sdk2}"
MATRIX_INFERENCE_ROOT="${MATRIX_INFERENCE_ROOT:-$RUNTIME_ROOT/inference}"
MATRIX_SONIC_CANONICAL_MODEL="${MATRIX_SONIC_CANONICAL_MODEL:-$MATRIX_SONIC_ROOT/gear_sonic/data/robot_model/model_data/g1/g1_29dof_with_hand.xml}"
MATRIX_SONIC_CANONICAL_MESHES="${MATRIX_SONIC_CANONICAL_MESHES:-$MATRIX_SONIC_ROOT/gear_sonic/data/robot_model/model_data/g1/meshes}"
CUSTOM_URDF="${CUSTOM_URDF:-$RUNTIME_ROOT/g1-visual/g1_29dof.urdf}"

if [[ -x "$PROJECT_ROOT/.venv-audit/bin/python" ]]; then
    DEFAULT_PYTHON="$PROJECT_ROOT/.venv-audit/bin/python"
else
    DEFAULT_PYTHON="$(command -v python3)"
fi
MATRIX_SONIC_PYTHON="${MATRIX_SONIC_PYTHON:-$DEFAULT_PYTHON}"
MATRIX_PICO_PYTHON="${MATRIX_PICO_PYTHON:-$MATRIX_SONIC_PYTHON}"

for required in \
    "$PROJECT_ROOT/scripts/matrix_game_control.py" \
    "$CUSTOM_URDF" \
    "$MATRIX_SONIC_ROOT/gear_sonic/scripts/run_sim_loop.py" \
    "$MATRIX_SONIC_ROOT/gear_sonic/utils/mujoco_sim/base_sim.py" \
    "$MATRIX_SONIC_ROOT/gear_sonic/utils/teleop/zmq/zmq_planner_sender.py" \
    "$MATRIX_SONIC_ROOT/gear_sonic_deploy/target/release/g1_deploy_onnx_ref" \
    "$MATRIX_UNITREE_SDK2_ROOT/lib/x86_64/libunitree_sdk2.a" \
    "$MATRIX_SONIC_PYTHON"; do
    if [[ ! -e "$required" ]]; then
        echo "[ERROR] Matrix SONIC runtime dependency is missing: $required" >&2
        exit 1
    fi
done
if [[ "$CONTROL_SOURCE" == "game" \
    && ! -f "$PROJECT_ROOT/scripts/matrix_game_control_input.py" ]]; then
    echo "[ERROR] Matrix game-control input provider is missing: $PROJECT_ROOT/scripts/matrix_game_control_input.py" >&2
    exit 1
fi

require_qualified_path() {
    local label="$1"
    local actual="$2"
    local expected="$3"
    local actual_resolved expected_resolved
    actual_resolved="$(realpath -m "$actual")"
    expected_resolved="$(realpath -m "$expected")"
    if [[ "$actual_resolved" != "$expected_resolved" ]]; then
        echo "[ERROR] Bounded qualification requires locked $label: expected=$expected_resolved actual=$actual_resolved" >&2
        exit 2
    fi
}

require_qualified_executable_path() {
    local label="$1"
    local actual="$2"
    local expected="$3"
    local actual_absolute expected_absolute
    actual_absolute="$(cd "$(dirname "$actual")" && pwd -P)/$(basename "$actual")"
    expected_absolute="$(cd "$(dirname "$expected")" && pwd -P)/$(basename "$expected")"
    if [[ "$actual_absolute" != "$expected_absolute" ]]; then
        echo "[ERROR] Bounded qualification requires locked $label: expected=$expected_absolute actual=$actual_absolute" >&2
        exit 2
    fi
}

if [[ "$QUALIFICATION_REQUESTED" == "1" ]]; then
    if [[ -n "${LD_LIBRARY_PATH:-}" || -n "${PYTHONPATH:-}" ]]; then
        echo "[ERROR] Bounded qualification rejects inherited LD_LIBRARY_PATH/PYTHONPATH" >&2
        exit 2
    fi
    require_qualified_path "Unitree SDK root" \
        "$MATRIX_UNITREE_SDK2_ROOT" \
        "$MATRIX_SONIC_ROOT/gear_sonic_deploy/thirdparty/unitree_sdk2"
    require_qualified_path "inference root" \
        "$MATRIX_INFERENCE_ROOT" "$RUNTIME_ROOT/inference"
    require_qualified_path "canonical model" \
        "$MATRIX_SONIC_CANONICAL_MODEL" \
        "$MATRIX_SONIC_ROOT/gear_sonic/data/robot_model/model_data/g1/g1_29dof_with_hand.xml"
    require_qualified_path "canonical meshes" \
        "$MATRIX_SONIC_CANONICAL_MESHES" \
        "$MATRIX_SONIC_ROOT/gear_sonic/data/robot_model/model_data/g1/meshes"
    require_qualified_path "visual URDF" \
        "$CUSTOM_URDF" "$RUNTIME_ROOT/g1-visual/g1_29dof.urdf"
    require_qualified_executable_path "runtime Python" \
        "$MATRIX_SONIC_PYTHON" "$PROJECT_ROOT/.venv-audit/bin/python"
    require_qualified_path "native dependency root" \
        "${MATRIX_NATIVE_DEPS_ROOT:-$RUNTIME_ROOT/matrix-native-deps}" \
        "$RUNTIME_ROOT/matrix-native-deps"
    case "$PROFILE" in
        trna)
            expected_ros_prefix="/opt/ros/humble"
            expected_cuda_root="/usr/local/cuda"
            ;;
        heyuan)
            expected_ros_prefix="$RUNTIME_ROOT/ros2-humble-prefix"
            expected_cuda_root="/usr/local/cuda"
            ;;
        zza)
            expected_ros_prefix="$RUNTIME_ROOT/ros2-humble-prefix"
            expected_cuda_root="/data/user_data/matrix-tools/cuda-runtime-12.1"
            ;;
        *)
            echo "[ERROR] Unsupported bounded qualification profile: $PROFILE" >&2
            exit 2
            ;;
    esac
    require_qualified_path "ROS prefix" \
        "${MATRIX_ROS_PREFIX:-}" "$expected_ros_prefix"
    require_qualified_path "CUDA root" \
        "${MATRIX_CUDA_ROOT:-/usr/local/cuda}" "$expected_cuda_root"
    require_qualified_path "TensorRT root" \
        "${TensorRT_ROOT:-$MATRIX_INFERENCE_ROOT/TensorRT}" \
        "$RUNTIME_ROOT/inference/TensorRT"
    readarray -t VISUAL_LOCK_HASHES < <(/usr/bin/python3 -I - "$LOCK_FILE" <<'PY'
import json
import sys

lock = json.load(open(sys.argv[1], encoding="utf-8"))
files = {(entry["root"], entry["path"]): entry["sha256"] for entry in lock["runtime_files"]}
trees = {(entry["root"], entry["path"]): entry["sha256"] for entry in lock["runtime_trees"]}
print(files[("visual", "g1_29dof.urdf")])
print(trees[("visual", "meshes")])
PY
    )
    if [[ "${#VISUAL_LOCK_HASHES[@]}" != "2" ]]; then
        echo "[ERROR] Runtime lock is missing the qualified G1 visual closure" >&2
        exit 2
    fi
    export MATRIX_G1_VISUAL_URDF_SHA256="${VISUAL_LOCK_HASHES[0]}"
    export MATRIX_G1_VISUAL_MESHES_SHA256="${VISUAL_LOCK_HASHES[1]}"
    # A bounded run always rebuilds the visual cache from the just-verified
    # source closure; an older or locally tampered conversion is never reused.
    export SIM_LAUNCHER_FORCE_REIMPORT_CUSTOM_URDF=1
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
prepend_library_dir "$PROJECT_ROOT/src/UeSim/Linux/Engine/Binaries/Linux"
prepend_library_dir "$PROJECT_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux"
if [[ -n "${MATRIX_ROS_PREFIX:-}" ]]; then
    prepend_library_dir "$MATRIX_ROS_PREFIX/lib"
fi
if [[ -n "${MATRIX_NATIVE_DEPS_ROOT:-}" ]]; then
    prepend_library_dir "$MATRIX_NATIVE_DEPS_ROOT/usr/lib/x86_64-linux-gnu"
    prepend_library_dir "$MATRIX_NATIVE_DEPS_ROOT/usr/lib"
fi
prepend_library_dir "$MATRIX_UNITREE_SDK2_ROOT/thirdparty/lib/x86_64"
prepend_library_dir "$MATRIX_SONIC_ROOT/external_dependencies/XRoboToolkit-PC-Service-Pybind_X86_and_ARM64/lib"
prepend_library_dir "$MATRIX_INFERENCE_ROOT/onnxruntime/lib"
prepend_library_dir "$MATRIX_INFERENCE_ROOT/TensorRT/lib"
export LD_LIBRARY_PATH

if [[ -n "${MATRIX_ROS_PREFIX:-}" && -d "$MATRIX_ROS_PREFIX" ]]; then
    export AMENT_PREFIX_PATH="${MATRIX_ROS_PREFIX}${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
    export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
fi
export TensorRT_ROOT="${TensorRT_ROOT:-$MATRIX_INFERENCE_ROOT/TensorRT}"
export PATH="$(dirname "$MATRIX_SONIC_PYTHON"):$PATH"

if [[ "$CONTROL_SOURCE" == "pico" \
    && "${MATRIX_VERIFY_RUNTIME:-1}" != "0" \
    && -z "$PROFILE" ]]; then
    echo "[ERROR] Locked PICO acceptance requires --profile for runtime verification" >&2
    exit 2
fi
if [[ -n "$PROFILE" && "${MATRIX_VERIFY_RUNTIME:-1}" != "0" ]]; then
    VERIFY_RUNTIME_ARGS=(
        --runtime-root "$RUNTIME_ROOT"
        --matrix-root "$PROJECT_ROOT"
        --sonic-root "$MATRIX_SONIC_ROOT"
        --python "$MATRIX_SONIC_PYTHON"
        --profile "$PROFILE"
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
        VERIFY_RUNTIME_ARGS+=(
            --require-git-sonic
            --json-output "$MATRIX_SONIC_VERIFICATION_RECEIPT"
        )
    else
        VERIFY_RUNTIME_ARGS+=(--fast)
    fi
    /usr/bin/python3 -I "$PROJECT_ROOT/scripts/verify_matrix_sonic_runtime.py" \
        "${VERIFY_RUNTIME_ARGS[@]}"
fi

mkdir -p "$PROJECT_ROOT/outputs"

export MATRIX_SONIC=1
export MATRIX_DISABLE_MC=1
export MATRIX_SONIC_ROOT MATRIX_UNITREE_SDK2_ROOT
export MATRIX_SONIC_PYTHON MATRIX_PICO_PYTHON
export MATRIX_SONIC_CANONICAL_MODEL MATRIX_SONIC_CANONICAL_MESHES
export MATRIX_SONIC_CONTROL_SOURCE="$CONTROL_SOURCE"
export MATRIX_GAME_INPUT_SOURCE="$GAME_INPUT_SOURCE"
export MATRIX_GAME_CAMERA_YAW_SOURCE="$GAME_CAMERA_YAW_SOURCE"
export MATRIX_GAME_LOOK_BUTTON="$GAME_LOOK_BUTTON"
export MATRIX_GAME_INITIAL_CAMERA_YAW_DEG="$GAME_INITIAL_CAMERA_YAW_DEG"
export MATRIX_GAME_MOUSE_SENSITIVITY_DEG="$GAME_MOUSE_SENSITIVITY_DEG"
export MATRIX_GAME_CAMERA_YAW_SIGN="$GAME_CAMERA_YAW_SIGN"
export MATRIX_GAME_CAMERA_YAW_OFFSET_DEG="$GAME_CAMERA_YAW_OFFSET_DEG"
export MATRIX_GAME_CARLA_HOST="$GAME_CARLA_HOST"
export MATRIX_GAME_CARLA_PORT="$GAME_CARLA_PORT"
export MATRIX_GAMEPAD_LOOK_YAW_RATE_DEG_S="$GAMEPAD_LOOK_YAW_RATE_DEG_S"
export MATRIX_GAMEPAD_LOOK_PITCH_RATE_DEG_S="$GAMEPAD_LOOK_PITCH_RATE_DEG_S"
export MATRIX_GAMEPAD_LOOK_DEADZONE="$GAMEPAD_LOOK_DEADZONE"
export MATRIX_GAMEPAD_LOOK_MIN_PITCH_DEG="$GAMEPAD_LOOK_MIN_PITCH_DEG"
export MATRIX_GAMEPAD_LOOK_MAX_PITCH_DEG="$GAMEPAD_LOOK_MAX_PITCH_DEG"
export MATRIX_GAME_MAX_SPEED="$GAME_MAX_SPEED"
export MATRIX_GAME_INPUT_TIMEOUT="$GAME_INPUT_TIMEOUT"
export MATRIX_GAME_INPUT_STATUS_FILE="${MATRIX_GAME_INPUT_STATUS_FILE:-$PROJECT_ROOT/outputs/matrix_game_control_input.json}"
if [[ "$CONTROL_SOURCE" == "game" ]]; then
    rm -f -- "$MATRIX_GAME_INPUT_STATUS_FILE"
fi
export MATRIX_SONIC_WALK_AFTER="$WALK_AFTER"
export MATRIX_SONIC_VX="$VX"
export MATRIX_SONIC_VY="$VY"
export MATRIX_SONIC_YAW_RATE="$YAW_RATE"
export MATRIX_SONIC_MAX_SECONDS="$MAX_SECONDS"
export MATRIX_SONIC_MIN_ACTIVE_SECONDS="$MIN_ACTIVE_SECONDS"
export MATRIX_SONIC_MIN_DISPLACEMENT_M="$MIN_DISPLACEMENT_M"
export MATRIX_SONIC_LOW_CMD_FRESH_TIMEOUT_SECONDS="$LOW_CMD_FRESH_TIMEOUT_SECONDS"
export MATRIX_SONIC_MIN_PHYSICS_HZ="$MIN_PHYSICS_HZ"
export MATRIX_SONIC_MIN_RTF="$MIN_RTF"
export MATRIX_SONIC_MAX_RESETS="$MAX_RESETS"
export MATRIX_SONIC_QUALIFIED_RUNTIME
export MATRIX_SONIC_QUALIFICATION_PROFILE
export MATRIX_SONIC_RUNTIME_LOCK_SHA256
export MATRIX_SONIC_MATRIX_COMMIT
export MATRIX_SONIC_VERIFICATION_RECEIPT
export MATRIX_SONIC_FAIL_ON_FALL=1
export MATRIX_SONIC_STARTUP_BAND="$STARTUP_BAND"
export MATRIX_SONIC_STARTUP_BAND_HOLD="$STARTUP_BAND_HOLD"
export MATRIX_SONIC_STARTUP_BAND_FADE="$STARTUP_BAND_FADE"

# Matrix's upstream launcher rewrites these tracked files. Restore the exact
# pre-launch bytes so switching the same feature branch on two hosts stays clean.
CONFIG_BACKUP="$(mktemp -d "${TMPDIR:-/tmp}/matrix-sonic-config.XXXXXX")"
GAME_RUNTIME_DIR=""
GENERATED_GAME_INPUT_SOCKET=0
cleanup_prelaunch_temp() {
    rm -rf -- "$CONFIG_BACKUP"
    if [[ -n "$GAME_RUNTIME_DIR" ]]; then
        rm -rf -- "$GAME_RUNTIME_DIR"
    fi
}
trap cleanup_prelaunch_temp EXIT
if [[ "$CONTROL_SOURCE" == "game" ]]; then
    GAME_RUNTIME_DIR="$(mktemp -d "${XDG_RUNTIME_DIR:-/tmp}/matrix-game-control-${UID}.XXXXXX")"
    chmod 700 "$GAME_RUNTIME_DIR"
    if [[ -z "${MATRIX_GAME_INPUT_SOCKET:-}" ]]; then
        export MATRIX_GAME_INPUT_SOCKET="$GAME_RUNTIME_DIR/input.sock"
        GENERATED_GAME_INPUT_SOCKET=1
    fi
    export MATRIX_GAME_RESTART_REQUEST_FILE="$GAME_RUNTIME_DIR/restart-request.json"
    export MATRIX_GAME_RESTART_CAPABILITY_FILE="$GAME_RUNTIME_DIR/restart-capability"
    /usr/bin/python3 -I "$PROJECT_ROOT/scripts/matrix_restart_request.py" \
        create-capability --file "$MATRIX_GAME_RESTART_CAPABILITY_FILE"
fi
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
    if [[ "${TRACKED_CONFIG_RESTORED:-0}" == "1" ]]; then
        return 0
    fi
    local relative
    local failed=0
    local destination temporary_restore
    for relative in "${MUTABLE_FILES[@]}"; do
        if [[ -f "$CONFIG_BACKUP/$relative" ]]; then
            destination="$PROJECT_ROOT/$relative"
            temporary_restore="${destination}.matrix-restore.$$"
            rm -f -- "$temporary_restore"
            if ! cp -a "$CONFIG_BACKUP/$relative" "$temporary_restore"; then
                echo "[ERROR] Failed to stage tracked config restore: $relative" >&2
                rm -f -- "$temporary_restore"
                failed=1
                continue
            fi
            if ! mv -f -- "$temporary_restore" "$destination" \
                || ! cmp -s -- "$CONFIG_BACKUP/$relative" "$destination"; then
                echo "[ERROR] Failed to verify tracked config restore: $relative" >&2
                rm -f -- "$temporary_restore"
                failed=1
            fi
        fi
    done
    if [[ "$failed" != "0" ]]; then
        echo "[ERROR] Preserving failed restore backup at $CONFIG_BACKUP" >&2
        return 1
    fi
    if [[ -n "$GAME_RUNTIME_DIR" ]]; then
        if ! rm -rf -- "$GAME_RUNTIME_DIR"; then
            echo "[ERROR] Failed to remove game runtime directory" >&2
            return 1
        fi
    fi
    if ! rm -rf -- "$CONFIG_BACKUP"; then
        echo "[ERROR] Failed to remove tracked-config backup" >&2
        return 1
    fi
    if [[ "$GENERATED_GAME_INPUT_SOCKET" == "1" ]]; then
        unset MATRIX_GAME_INPUT_SOCKET
    fi
    unset MATRIX_GAME_RESTART_REQUEST_FILE MATRIX_GAME_RESTART_CAPABILITY_FILE
    TRACKED_CONFIG_RESTORED=1
}
trap restore_tracked_config EXIT

RUN_SIM_PID=""
FORWARDED_SIGNAL_EXIT_CODE=0
RESTART_REQUEST_VALID=0
STOP_REQUESTED=0
FORCED_STOP=0
INTERNAL_RESTART_TIMEOUT=0
RUN_SIM_STOP_TIMEOUT_SECONDS="${MATRIX_RUN_SIM_STOP_TIMEOUT_SECONDS:-25}"
if [[ ! "$RUN_SIM_STOP_TIMEOUT_SECONDS" \
    =~ ^([1-9][0-9]*(\.[0-9]+)?|0\.[0-9]*[1-9][0-9]*)$ ]]; then
    echo "[ERROR] MATRIX_RUN_SIM_STOP_TIMEOUT_SECONDS must be positive" >&2
    exit 2
fi
forward_signal() {
    local signal_name="$1"
    local exit_code="$2"
    FORWARDED_SIGNAL_EXIT_CODE="$exit_code"
    STOP_REQUESTED=1
    if [[ -n "$RUN_SIM_PID" ]] && kill -0 "$RUN_SIM_PID" 2>/dev/null; then
        kill "-$signal_name" "$RUN_SIM_PID" 2>/dev/null || true
    fi
}
trap 'forward_signal INT 130' SIGINT
trap 'forward_signal TERM 143' SIGTERM
trap 'forward_signal HUP 129' SIGHUP

if [[ "$FORWARDED_SIGNAL_EXIT_CODE" != "0" ]]; then
    exit "$FORWARDED_SIGNAL_EXIT_CODE"
fi
export MATRIX_SONIC_LAUNCHER_PID="$$"

set +e
"$PROJECT_ROOT/scripts/run_sim.sh" \
    custom "$SCENE_ID" "$OFFSCREEN" 0 1 "$CUSTOM_URDF" "$CUSTOM_NAME" &
RUN_SIM_PID=$!
if [[ "$FORWARDED_SIGNAL_EXIT_CODE" != "0" ]]; then
    kill -TERM "$RUN_SIM_PID" 2>/dev/null || true
fi
RESTART_WATCHER_PID=""
if [[ "$CONTROL_SOURCE" == "game" ]]; then
    /usr/bin/python3 -I "$PROJECT_ROOT/scripts/matrix_restart_request.py" \
        watch \
        --file "$MATRIX_GAME_RESTART_REQUEST_FILE" \
        --launcher-pid "$$" \
        --run-sim-pid "$RUN_SIM_PID" \
        --provider-script "$PROJECT_ROOT/scripts/matrix_game_control_input.py" \
        --capability-file "$MATRIX_GAME_RESTART_CAPABILITY_FILE" \
        --poll-seconds 0.2 9>&- &
    RESTART_WATCHER_PID=$!
fi
exit_code=0
RUN_SIM_COMPLETED=0
STOP_TIMER_PID=""
while [[ "$RUN_SIM_COMPLETED" == "0" ]]; do
    if [[ "$STOP_REQUESTED" == "1" \
        && -z "$STOP_TIMER_PID" \
        && ( "$INTERNAL_RESTART_TIMEOUT" == "0" \
            || "$FORWARDED_SIGNAL_EXIT_CODE" != "0" ) ]]; then
        sleep "$RUN_SIM_STOP_TIMEOUT_SECONDS" 9>&- &
        STOP_TIMER_PID=$!
    fi
    WAIT_PIDS=("$RUN_SIM_PID")
    if [[ -n "$RESTART_WATCHER_PID" ]]; then
        WAIT_PIDS+=("$RESTART_WATCHER_PID")
    fi
    if [[ -n "$STOP_TIMER_PID" ]]; then
        WAIT_PIDS+=("$STOP_TIMER_PID")
    fi
    COMPLETED_PID=""
    wait -n -p COMPLETED_PID "${WAIT_PIDS[@]}"
    wait_status=$?
    if [[ "${COMPLETED_PID:-}" == "$RUN_SIM_PID" ]]; then
        exit_code="$wait_status"
        RUN_SIM_COMPLETED=1
        for helper_pid in "$RESTART_WATCHER_PID" "$STOP_TIMER_PID"; do
            if [[ -n "$helper_pid" ]]; then
                kill -TERM "$helper_pid" 2>/dev/null || true
                wait "$helper_pid" 2>/dev/null || true
            fi
        done
        break
    fi
    if [[ -n "$RESTART_WATCHER_PID" \
        && "${COMPLETED_PID:-}" == "$RESTART_WATCHER_PID" ]]; then
        RESTART_WATCHER_PID=""
        if [[ "$wait_status" == "75" ]]; then
            RESTART_REQUEST_VALID=1
            STOP_REQUESTED=1
            echo "[INFO] Validated full Matrix runtime restart request"
            kill -TERM "$RUN_SIM_PID" 2>/dev/null || true
        elif [[ "$wait_status" != "0" ]]; then
            echo "[WARN] Restart watcher exited with code $wait_status; " \
                "continuing without in-session restart" >&2
        fi
        continue
    fi
    if [[ -n "$STOP_TIMER_PID" \
        && "${COMPLETED_PID:-}" == "$STOP_TIMER_PID" ]]; then
        STOP_TIMER_PID=""
        if [[ "$FORWARDED_SIGNAL_EXIT_CODE" == "0" \
            && "$RESTART_REQUEST_VALID" == "1" ]]; then
            # An in-session F9 request is not authority to orphan an old native
            # process tree.  Cancel this restart and keep supervising the exact
            # original child for as long as it needs.  A later real external
            # signal can arm a fresh bounded forced-stop timer.
            RESTART_REQUEST_VALID=0
            INTERNAL_RESTART_TIMEOUT=1
            echo "[ERROR] Internal Matrix restart timed out after " \
                "${RUN_SIM_STOP_TIMEOUT_SECONDS}s; restart cancelled and " \
                "continuing to supervise the original runtime" >&2
            continue
        fi
        echo "[ERROR] run_sim did not finish external-signal cleanup within " \
            "${RUN_SIM_STOP_TIMEOUT_SECONDS} seconds" >&2
        kill -KILL "$RUN_SIM_PID" 2>/dev/null || true
        wait "$RUN_SIM_PID" 2>/dev/null
        exit_code=$?
        RUN_SIM_COMPLETED=1
        FORCED_STOP=1
        RESTART_REQUEST_VALID=0
        if [[ -n "$RESTART_WATCHER_PID" ]]; then
            kill -TERM "$RESTART_WATCHER_PID" 2>/dev/null || true
            wait "$RESTART_WATCHER_PID" 2>/dev/null || true
            RESTART_WATCHER_PID=""
        fi
    fi
done
if [[ "$FORWARDED_SIGNAL_EXIT_CODE" != "0" ]]; then
    exit_code="$FORWARDED_SIGNAL_EXIT_CODE"
fi
if [[ "$FORWARDED_SIGNAL_EXIT_CODE" == "0" \
    && "$RESTART_REQUEST_VALID" == "1" \
    && "$FORCED_STOP" == "0" \
    && "$exit_code" == "143" ]]; then
    if ! restore_tracked_config; then
        RESTART_REQUEST_VALID=0
        echo "[ERROR] Refusing restart after tracked-config restore failure" >&2
    else
        # Restoration can run external commands.  A stop signal received while
        # waiting for one of them must still win over the already-authorized
        # restart.  Once restoration is complete, remove its EXIT trap and use
        # immediate-exit handlers for the commit-to-exec window: signals not
        # already recorded by forward_signal then terminate instead of allowing
        # one unwanted new generation.
        trap - EXIT
        trap 'exit 130' SIGINT
        trap 'exit 143' SIGTERM
        trap 'exit 129' SIGHUP
        if [[ "$FORWARDED_SIGNAL_EXIT_CODE" != "0" ]]; then
            RESTART_REQUEST_VALID=0
            exit_code="$FORWARDED_SIGNAL_EXIT_CODE"
            echo "[INFO] External stop cancelled the pending Matrix restart" >&2
        else
            exec /usr/bin/env -i \
                "${ORIGINAL_ENVIRONMENT[@]}" \
                MATRIX_SONIC_RESTART_LOCK_FD=9 \
                "$PROJECT_ROOT/scripts/run_matrix_sonic.sh" "${ORIGINAL_ARGS[@]}"
            echo "[ERROR] Failed to exec restarted Matrix launcher" >&2
            exit 1
        fi
    fi
fi
if [[ "$RESTART_REQUEST_VALID" == "1" ]]; then
    echo "[ERROR] Aborting restart because run_sim did not exit cleanly" >&2
fi
# Run the normal-path restore explicitly while errexit is still disabled.  An
# EXIT trap that returns non-zero under `set -e` replaces an explicit exit
# status (for example, run_sim's expected 143 after a restart request).  Keep
# the original non-zero runtime status, but surface a restore failure when the
# runtime itself otherwise succeeded.
if ! restore_tracked_config; then
    if [[ "$exit_code" == "0" ]]; then
        exit_code=1
    fi
fi
trap - EXIT
# The explicit restore above can itself be interrupted while waiting for cp,
# mv, cmp, or directory cleanup.  Signals handled there must override the
# runtime's earlier status.  Immediate handlers close the final check-to-exit
# window without rerunning the already-completed restore.
trap 'exit 130' SIGINT
trap 'exit 143' SIGTERM
trap 'exit 129' SIGHUP
if [[ "$FORWARDED_SIGNAL_EXIT_CODE" != "0" ]]; then
    exit_code="$FORWARDED_SIGNAL_EXIT_CODE"
fi
exit "$exit_code"
