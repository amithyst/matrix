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
G1_SKIN="${MATRIX_G1_SKIN:-}"
CONTROL_SOURCE="${MATRIX_SONIC_CONTROL_SOURCE:-planner}"
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
GAME_WORLD_PERSISTENCE="${MATRIX_GAME_WORLD_PERSISTENCE:-auto}"
GAME_AUTO_RESPAWN="${MATRIX_GAME_AUTO_RESPAWN:-auto}"
GAME_WORLD_CHECKPOINT_SECONDS="${MATRIX_GAME_WORLD_CHECKPOINT_SECONDS:-0.75}"
GAME_FALL_RECOVERY="${MATRIX_GAME_FALL_RECOVERY:-auto}"
GAME_FALL_RECOVERY_TIMEOUT="${MATRIX_GAME_FALL_RECOVERY_TIMEOUT:-15.0}"
PHYSICAL_RECOVERY_WORKER="${MATRIX_PHYSICAL_RECOVERY_WORKER:-$PROJECT_ROOT/scripts/matrix_sonic_host_worker.py}"
PHYSICAL_RECOVERY_INITIAL_CONTROLLER="${MATRIX_PHYSICAL_RECOVERY_INITIAL_CONTROLLER:-host}"
PHYSICAL_RECOVERY_HANDOFF="${MATRIX_PHYSICAL_RECOVERY_HANDOFF:-amp}"
PHYSICAL_RECOVERY_RESIDENT_POLICIES="${MATRIX_PHYSICAL_RECOVERY_RESIDENT_POLICIES:-0}"
PHYSICAL_RECOVERY_EXECUTION_PROVIDER="${MATRIX_PHYSICAL_RECOVERY_EXECUTION_PROVIDER:-cpu}"
PHYSICAL_RECOVERY_PYTHON="${MATRIX_PHYSICAL_RECOVERY_PYTHON:-}"
PHYSICAL_RECOVERY_MODEL="${MATRIX_PHYSICAL_RECOVERY_MODEL:-}"
PHYSICAL_RECOVERY_MODEL_SHA256="${MATRIX_PHYSICAL_RECOVERY_MODEL_SHA256:-}"
PHYSICAL_RECOVERY_FALLBACK_MODEL="${MATRIX_PHYSICAL_RECOVERY_FALLBACK_MODEL:-}"
PHYSICAL_RECOVERY_AMP_CONFIG="${MATRIX_PHYSICAL_RECOVERY_AMP_CONFIG:-}"
PHYSICAL_RECOVERY_AMP_MODEL="${MATRIX_PHYSICAL_RECOVERY_AMP_MODEL:-}"
PHYSICAL_RECOVERY_AMP_CONFIG_SHA256="${MATRIX_PHYSICAL_RECOVERY_AMP_CONFIG_SHA256:-}"
PHYSICAL_RECOVERY_AMP_MODEL_SHA256="${MATRIX_PHYSICAL_RECOVERY_AMP_MODEL_SHA256:-}"
PHYSICAL_RECOVERY_KUNGFU_MODEL="${MATRIX_KUNGFU_RECOVERY_MODEL:-}"
PHYSICAL_RECOVERY_KUNGFU_MOTION="${MATRIX_KUNGFU_RECOVERY_MOTION:-}"
PHYSICAL_RECOVERY_KUNGFU_MODEL_SHA256="${MATRIX_KUNGFU_RECOVERY_MODEL_SHA256:-}"
PHYSICAL_RECOVERY_KUNGFU_MODEL_DATA_SHA256="${MATRIX_KUNGFU_RECOVERY_MODEL_DATA_SHA256:-}"
PHYSICAL_RECOVERY_KUNGFU_MOTION_SHA256="${MATRIX_KUNGFU_RECOVERY_MOTION_SHA256:-}"
PHYSICAL_RECOVERY_KUNGFU_REFERENCE_FRAME="${MATRIX_KUNGFU_RECOVERY_REFERENCE_FRAME:-0}"
PHYSICAL_RECOVERY_KUNGFU_GAIN_SCALE="${MATRIX_KUNGFU_RECOVERY_GAIN_SCALE:-1.0}"
PHYSICAL_RECOVERY_FALLBACK_AFTER_SECONDS="${MATRIX_PHYSICAL_RECOVERY_FALLBACK_AFTER_SECONDS:-10.0}"
PHYSICAL_RECOVERY_STABLE_HOLD_SECONDS="${MATRIX_PHYSICAL_RECOVERY_STABLE_HOLD_SECONDS:-1.5}"
PHYSICAL_RECOVERY_POLICY_EXIT_HOLD_SECONDS="${MATRIX_PHYSICAL_RECOVERY_POLICY_EXIT_HOLD_SECONDS:-0}"
PHYSICAL_RECOVERY_CONTROL_SOCKET="${MATRIX_PHYSICAL_RECOVERY_CONTROL_SOCKET:-${XDG_RUNTIME_DIR:-/tmp}/matrix-sonic-recovery-${UID}-$$.sock}"
PHYSICAL_RECOVERY_SONIC_CONTROL_SOCKET="${MATRIX_PHYSICAL_RECOVERY_SONIC_CONTROL_SOCKET:-${XDG_RUNTIME_DIR:-/tmp}/matrix-sonic-recovery-sonic-${UID}-$$.sock}"
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
        "  --skin ID                  Registered G1 skin (default: unitree-stock)" \
        "  --control-source SOURCE    planner, game, pico, or external (default: planner)" \
        "  --game-input-source SOURCE auto, keyboard, or gamepad (default: auto)" \
        "  --game-camera-yaw-source S ue-final-pov, x11-mirror, x11-core-gated, x11-absolute, carla, or fixed" \
        "  --game-look-button BUTTON  Camera drag button: left, middle, or right" \
        "  --game-initial-yaw DEG     Initial provider/UE camera yaw before sign and offset" \
        "  --game-mouse-sensitivity DEG_PER_UNIT  Calibrated selected X11 mirror scale (default: 0.12)" \
        "  --game-camera-yaw-sign N   Provider-to-SONIC sign: -1 or 1" \
        "  --game-camera-yaw-offset DEG  Provider-to-SONIC zero-frame offset" \
        "  --game-carla-host HOST     Optional fail-closed CARLA spectator host" \
        "  --game-carla-port PORT     Optional CARLA spectator RPC port" \
        "  --gamepad-look-yaw-rate DEG_S    Full-stick spectator yaw rate" \
        "  --gamepad-look-pitch-rate DEG_S  Full-stick spectator pitch rate" \
        "  --gamepad-look-deadzone VALUE    Radial right-stick deadzone" \
        "  --gamepad-look-min-pitch DEG     Spectator pitch lower limit" \
        "  --gamepad-look-max-pitch DEG     Spectator pitch upper limit" \
        "  --game-max-speed MPS       Analog SLOW_WALK cap (default 0.30; max 0.80)" \
        "  --game-input-timeout SEC   Deadman timeout (default: 0.15)" \
        "  --game-world-persistence MODE  auto, on, or off (default: auto)" \
        "  --game-auto-respawn MODE   auto, on, or off; cold-reloads after a fall" \
        "  --game-world-checkpoint-seconds SEC  Durable last-exit interval (default: 0.75)" \
        "  --game-fall-recovery MODE  auto, off, sonic, or physical (Heyuan/TRNA auto uses physical for unbounded game runs)" \
        "  --game-fall-recovery-timeout SEC  Mark recovery timed out after SEC while continuing IDLE" \
        "  --physical-recovery-worker PATH  Writer-gated physical get-up worker" \
        "  --physical-recovery-initial-controller NAME  Initial policy: host, amp, or kungfu" \
        "  --physical-recovery-handoff NAME  Stable handoff: amp or sonic" \
        "  --physical-recovery-python PATH  Python with numpy, onnxruntime, and unitree_sdk2py" \
        "  --physical-recovery-model PATH   Primary physical get-up ONNX" \
        "  --physical-recovery-amp-config PATH  AMP dynamic-hold JSON" \
        "  --physical-recovery-amp-model PATH   AMP dynamic-hold ONNX" \
        "  --physical-recovery-fallback-model PATH  Physically continuous fallback ONNX" \
        "  --physical-recovery-kungfu-model PATH  KungFuAthleteBot recovery ONNX" \
        "  --physical-recovery-kungfu-motion PATH  KungFuAthleteBot 1307 NPZ" \
        "  --physical-recovery-kungfu-reference-frame N  0=50 Hz sequence; N>0=frozen target" \
        "  --physical-recovery-kungfu-gain-scale N  Recovery PD gain multiplier" \
        "  --physical-recovery-fallback-after-seconds SEC  Switch to fallback policy after SEC" \
        "  --physical-recovery-stable-hold-seconds SEC  Parent snapshot stability hold" \
        "  --physical-recovery-policy-exit-hold-seconds SEC  Optional recovery-policy terminal dwell" \
        "  --physical-recovery-control-socket PATH  Private worker handoff socket" \
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
        --skin) G1_SKIN="$2"; shift 2 ;;
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
        --game-world-persistence) GAME_WORLD_PERSISTENCE="$2"; shift 2 ;;
        --game-auto-respawn) GAME_AUTO_RESPAWN="$2"; shift 2 ;;
        --game-world-checkpoint-seconds) GAME_WORLD_CHECKPOINT_SECONDS="$2"; shift 2 ;;
        --game-fall-recovery) GAME_FALL_RECOVERY="$2"; shift 2 ;;
        --game-fall-recovery-timeout) GAME_FALL_RECOVERY_TIMEOUT="$2"; shift 2 ;;
        --physical-recovery-worker) PHYSICAL_RECOVERY_WORKER="$2"; shift 2 ;;
        --physical-recovery-initial-controller) PHYSICAL_RECOVERY_INITIAL_CONTROLLER="$2"; shift 2 ;;
        --physical-recovery-handoff) PHYSICAL_RECOVERY_HANDOFF="$2"; shift 2 ;;
        --physical-recovery-python) PHYSICAL_RECOVERY_PYTHON="$2"; shift 2 ;;
        --physical-recovery-model) PHYSICAL_RECOVERY_MODEL="$2"; shift 2 ;;
        --physical-recovery-amp-config) PHYSICAL_RECOVERY_AMP_CONFIG="$2"; shift 2 ;;
        --physical-recovery-amp-model) PHYSICAL_RECOVERY_AMP_MODEL="$2"; shift 2 ;;
        --physical-recovery-fallback-model) PHYSICAL_RECOVERY_FALLBACK_MODEL="$2"; shift 2 ;;
        --physical-recovery-kungfu-model) PHYSICAL_RECOVERY_KUNGFU_MODEL="$2"; shift 2 ;;
        --physical-recovery-kungfu-motion) PHYSICAL_RECOVERY_KUNGFU_MOTION="$2"; shift 2 ;;
        --physical-recovery-kungfu-reference-frame) PHYSICAL_RECOVERY_KUNGFU_REFERENCE_FRAME="$2"; shift 2 ;;
        --physical-recovery-kungfu-gain-scale) PHYSICAL_RECOVERY_KUNGFU_GAIN_SCALE="$2"; shift 2 ;;
        --physical-recovery-fallback-after-seconds) PHYSICAL_RECOVERY_FALLBACK_AFTER_SECONDS="$2"; shift 2 ;;
        --physical-recovery-stable-hold-seconds) PHYSICAL_RECOVERY_STABLE_HOLD_SECONDS="$2"; shift 2 ;;
        --physical-recovery-policy-exit-hold-seconds) PHYSICAL_RECOVERY_POLICY_EXIT_HOLD_SECONDS="$2"; shift 2 ;;
        --physical-recovery-control-socket) PHYSICAL_RECOVERY_CONTROL_SOCKET="$2"; shift 2 ;;
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

if [[ -n "$G1_SKIN" ]]; then
    export MATRIX_G1_SKIN="$G1_SKIN"
fi

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
    || [[ "$MATRIX_MOUSE_SETTINGS_LOAD_STATUS" != "loaded" \
        && "$MATRIX_MOUSE_SETTINGS_LOAD_STATUS" != "missing" \
        && "$MATRIX_MOUSE_SETTINGS_LOAD_STATUS" != "invalid" ]]; then
    echo "[ERROR] Invalid mouse-settings helper output" >&2
    exit 2
fi
if ! MATRIX_MOUSE_APPLIED_SPEED_SCALE="$(
    /usr/bin/python3 -I "$PROJECT_ROOT/scripts/matrix_mouse_settings.py" \
        canonical-scale --value "$MATRIX_MOUSE_APPLIED_SPEED_SCALE"
)"; then
    echo "[ERROR] Invalid mouse speed preset from settings helper" >&2
    exit 2
fi
export MATRIX_MOUSE_SETTINGS_FILE MATRIX_MOUSE_APPLIED_PROFILE
export MATRIX_MOUSE_APPLIED_SPEED_SCALE MATRIX_MOUSE_SETTINGS_LOAD_STATUS
echo "[INFO] Mouse launch profile: $MATRIX_MOUSE_APPLIED_PROFILE " \
    "scale=$MATRIX_MOUSE_APPLIED_SPEED_SCALE status=$MATRIX_MOUSE_SETTINGS_LOAD_STATUS"
MATRIX_MOTION_SETTINGS_PROFILE="${MATRIX_HOST_PROFILE:-${PROFILE:-local}}"
MATRIX_MOTION_SETTINGS_FILE="${MATRIX_MOTION_SETTINGS_FILE:-${XDG_CONFIG_HOME:-$HOME/.config}/matrix/hosts/${MATRIX_MOTION_SETTINGS_PROFILE}/motion-control.json}"
if [[ "$MATRIX_MOTION_SETTINGS_FILE" != /* ]]; then
    echo "[ERROR] MATRIX_MOTION_SETTINGS_FILE must be absolute" >&2
    exit 2
fi
MATRIX_MOTION_SETTINGS_FILE="$(realpath -m "$MATRIX_MOTION_SETTINGS_FILE")"
export MATRIX_MOTION_SETTINGS_FILE
echo "[INFO] Motion settings file: $MATRIX_MOTION_SETTINGS_FILE"
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
if [[ ! -v MATRIX_VERIFY_RUNTIME && "$QUALIFICATION_REQUESTED" == "0" ]]; then
    export MATRIX_VERIFY_RUNTIME="${MATRIX_PROFILE_VERIFY_RUNTIME_DEFAULT:-1}"
fi
case "${GAME_WORLD_PERSISTENCE,,}" in
    auto)
        if [[ "$CONTROL_SOURCE" == "game" \
            && "$QUALIFICATION_REQUESTED" == "0" ]]; then
            GAME_WORLD_PERSISTENCE=1
        else
            GAME_WORLD_PERSISTENCE=0
        fi
        ;;
    1|true|yes|on) GAME_WORLD_PERSISTENCE=1 ;;
    0|false|no|off) GAME_WORLD_PERSISTENCE=0 ;;
    *)
        echo "[ERROR] --game-world-persistence must be auto, on, or off" >&2
        exit 2
        ;;
esac
case "${GAME_AUTO_RESPAWN,,}" in
    auto)
        if [[ "$CONTROL_SOURCE" == "game" \
            && "$QUALIFICATION_REQUESTED" == "0" \
            && "$GAME_WORLD_PERSISTENCE" == "1" ]]; then
            GAME_AUTO_RESPAWN=1
        else
            GAME_AUTO_RESPAWN=0
        fi
        ;;
    1|true|yes|on) GAME_AUTO_RESPAWN=1 ;;
    0|false|no|off) GAME_AUTO_RESPAWN=0 ;;
    *)
        echo "[ERROR] --game-auto-respawn must be auto, on, or off" >&2
        exit 2
        ;;
esac
if [[ "$GAME_WORLD_PERSISTENCE" == "1" ]]; then
    if [[ "$CONTROL_SOURCE" != "game" ]]; then
        echo "[ERROR] Persistent world state requires --control-source game" >&2
        exit 2
    fi
    if [[ "$QUALIFICATION_REQUESTED" == "1" ]]; then
        echo "[ERROR] Bounded qualification rejects persistent world state" >&2
        exit 2
    fi
fi
if [[ "$GAME_AUTO_RESPAWN" == "1" \
    && "$GAME_WORLD_PERSISTENCE" != "1" ]]; then
    echo "[ERROR] Auto respawn requires persistent world state" >&2
    exit 2
fi
if ! /usr/bin/python3 -I - "$GAME_WORLD_CHECKPOINT_SECONDS" <<'PY'
import math
import sys
try:
    value = float(sys.argv[1])
except ValueError as exc:
    raise SystemExit("checkpoint interval is not numeric") from exc
if not math.isfinite(value) or not 0.1 <= value <= 60.0:
    raise SystemExit("checkpoint interval must be in [0.1, 60]")
PY
then
    echo "[ERROR] Invalid --game-world-checkpoint-seconds" >&2
    exit 2
fi
export MATRIX_PROFILE="${PROFILE:-local}"
case "$GAME_FALL_RECOVERY" in
    auto)
        if [[ "$CONTROL_SOURCE" == "game" \
            && "$QUALIFICATION_REQUESTED" == "0" ]]; then
            if [[ "$PROFILE" == "heyuan" \
                || "$PROFILE" == "trna" \
                || "${MATRIX_HOST_PROFILE:-}" == "heyuan" \
                || "${MATRIX_HOST_PROFILE:-}" == "trna" ]]; then
                GAME_FALL_RECOVERY="physical"
            else
                GAME_FALL_RECOVERY="sonic"
            fi
        else
            GAME_FALL_RECOVERY="off"
        fi
        ;;
    off|sonic|physical) ;;
    *)
        echo "[ERROR] --game-fall-recovery must be auto, off, sonic, or physical" >&2
        exit 2
        ;;
esac
if [[ "$GAME_FALL_RECOVERY" == "sonic" \
    || "$GAME_FALL_RECOVERY" == "physical" ]]; then
    if [[ "$CONTROL_SOURCE" != "game" ]]; then
        echo "[ERROR] Fall recovery requires --control-source game" >&2
        exit 2
    fi
    if [[ "$QUALIFICATION_REQUESTED" == "1" ]]; then
        echo "[ERROR] Bounded qualification requires fail-fast fall handling" >&2
        exit 2
    fi
fi
echo "[INFO] Fall policy: $GAME_FALL_RECOVERY timeout=${GAME_FALL_RECOVERY_TIMEOUT}s"
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
        if [[ "$GAME_CAMERA_YAW_SOURCE" == "x11-core-gated" \
            || "$GAME_CAMERA_YAW_SOURCE" == "x11-absolute" \
            || "$GAME_CAMERA_YAW_SOURCE" == "ue-final-pov" ]]; then
            echo "[ERROR] Bounded game-control qualification rejects experimental camera yaw sources" >&2
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

physical_recovery_python_works() {
    local candidate="$1"
    PYTHONNOUSERSITE=1 \
        PYTHONPATH="$MATRIX_SONIC_ROOT${PYTHONPATH:+:$PYTHONPATH}" \
        "$candidate" -c \
        'import numpy, onnxruntime; from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber; from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_; from unitree_sdk2py.utils.crc import CRC' \
        >/dev/null 2>&1
}

select_physical_recovery_python() {
    local -a candidates=()
    local candidate resolved
    if [[ -n "$PHYSICAL_RECOVERY_PYTHON" ]]; then
        candidates+=("$PHYSICAL_RECOVERY_PYTHON")
    else
        local artifact_root="${MATRIX_PHYSICAL_RECOVERY_ARTIFACT_ROOT:-/home/kaijie/matrix-artifacts/g1-host-getup-v1}"
        candidates+=(
            "$artifact_root/.venv/bin/python"
            "$artifact_root/venv/bin/python"
            "$artifact_root/.venv-host/bin/python"
            "$PROJECT_ROOT/.venv-physical-recovery/bin/python"
            "$MATRIX_SONIC_PYTHON"
        )
    fi
    for candidate in "${candidates[@]}"; do
        resolved="$candidate"
        if [[ "$candidate" != */* ]]; then
            resolved="$(command -v "$candidate" 2>/dev/null || true)"
        fi
        if [[ -n "$resolved" && -x "$resolved" ]] \
            && physical_recovery_python_works "$resolved"; then
            PHYSICAL_RECOVERY_PYTHON="$resolved"
            return 0
        fi
    done
    echo "[ERROR] Physical recovery requires one Python importing numpy," \
        "onnxruntime, and unitree_sdk2py; set MATRIX_PHYSICAL_RECOVERY_PYTHON" >&2
    return 1
}

if [[ "$GAME_FALL_RECOVERY" == "physical" ]]; then
    case "$PHYSICAL_RECOVERY_INITIAL_CONTROLLER" in
        host|amp|kungfu) ;;
        *)
            echo "[ERROR] Physical recovery initial controller must be host, amp, or kungfu" >&2
            exit 2
            ;;
    esac
    case "$PHYSICAL_RECOVERY_HANDOFF" in
        amp|sonic) ;;
        *)
            echo "[ERROR] Physical recovery handoff must be amp or sonic" >&2
            exit 2
            ;;
    esac
    case "$PHYSICAL_RECOVERY_EXECUTION_PROVIDER" in
        cuda|cpu) ;;
        *)
            echo "[ERROR] Physical recovery execution provider must be cuda or cpu" >&2
            exit 2
            ;;
    esac
    if [[ "$PHYSICAL_RECOVERY_RESIDENT_POLICIES" == "1" ]]; then
        if [[ "$PHYSICAL_RECOVERY_INITIAL_CONTROLLER" != "kungfu" \
            || "$PHYSICAL_RECOVERY_HANDOFF" != "sonic" \
            || "$PHYSICAL_RECOVERY_EXECUTION_PROVIDER" != "cuda" ]]; then
            echo "[ERROR] Resident recovery requires kungfu -> sonic with CUDA" >&2
            exit 2
        fi
    fi
    for recovery_file in \
        "$PHYSICAL_RECOVERY_WORKER" \
        "$PHYSICAL_RECOVERY_MODEL" \
        "$PHYSICAL_RECOVERY_AMP_CONFIG" \
        "$PHYSICAL_RECOVERY_AMP_MODEL"; do
        if [[ -z "$recovery_file" || ! -f "$recovery_file" ]]; then
            echo "[ERROR] Physical recovery artifact is missing: $recovery_file" >&2
            exit 1
        fi
    done
    if [[ -n "$PHYSICAL_RECOVERY_FALLBACK_MODEL" \
        && ! -f "$PHYSICAL_RECOVERY_FALLBACK_MODEL" ]]; then
        echo "[ERROR] Physical recovery fallback model is missing:" \
            "$PHYSICAL_RECOVERY_FALLBACK_MODEL" >&2
        exit 1
    fi
    if [[ -n "$PHYSICAL_RECOVERY_MODEL_SHA256" ]]; then
        if [[ ! "$PHYSICAL_RECOVERY_MODEL_SHA256" =~ ^[0-9a-f]{64}$ ]]; then
            echo "[ERROR] Physical recovery model SHA256 must be 64 lowercase hex characters" >&2
            exit 2
        fi
        actual_recovery_sha256="$(/usr/bin/python3 -I - "$PHYSICAL_RECOVERY_MODEL" <<'PY'
import hashlib
from pathlib import Path
import sys
print(hashlib.sha256(Path(sys.argv[1]).read_bytes()).hexdigest())
PY
)"
        if [[ "$actual_recovery_sha256" != "$PHYSICAL_RECOVERY_MODEL_SHA256" ]]; then
            echo "[ERROR] Physical recovery model SHA256 mismatch:" \
                "expected=$PHYSICAL_RECOVERY_MODEL_SHA256 actual=$actual_recovery_sha256" >&2
            exit 1
        fi
    fi
    if [[ "$PHYSICAL_RECOVERY_CONTROL_SOCKET" != /* ]]; then
        echo "[ERROR] Physical recovery control socket must be absolute" >&2
        exit 2
    fi
    for digest in \
        "$PHYSICAL_RECOVERY_AMP_CONFIG_SHA256" \
        "$PHYSICAL_RECOVERY_AMP_MODEL_SHA256"; do
        if [[ ! "$digest" =~ ^[0-9a-f]{64}$ ]]; then
            echo "[ERROR] Physical recovery AMP SHA256 values are required" >&2
            exit 2
        fi
    done
    if [[ "$PHYSICAL_RECOVERY_INITIAL_CONTROLLER" == "kungfu" ]]; then
        for kungfu_file in \
            "$PHYSICAL_RECOVERY_KUNGFU_MODEL" \
            "${PHYSICAL_RECOVERY_KUNGFU_MODEL}.data" \
            "$PHYSICAL_RECOVERY_KUNGFU_MOTION"; do
            if [[ -z "$kungfu_file" || ! -f "$kungfu_file" ]]; then
                echo "[ERROR] KungFu recovery artifact is missing: $kungfu_file" >&2
                exit 1
            fi
        done
        for digest in \
            "$PHYSICAL_RECOVERY_KUNGFU_MODEL_SHA256" \
            "$PHYSICAL_RECOVERY_KUNGFU_MODEL_DATA_SHA256" \
            "$PHYSICAL_RECOVERY_KUNGFU_MOTION_SHA256"; do
            if [[ ! "$digest" =~ ^[0-9a-f]{64}$ ]]; then
                echo "[ERROR] KungFu recovery SHA256 values are required" >&2
                exit 2
            fi
        done
        if [[ ! "$PHYSICAL_RECOVERY_KUNGFU_REFERENCE_FRAME" =~ ^[0-9]+$ ]]; then
            echo "[ERROR] KungFu reference frame must be a non-negative integer" >&2
            exit 2
        fi
    fi
    if [[ "$PHYSICAL_RECOVERY_SONIC_CONTROL_SOCKET" != /* ]]; then
        echo "[ERROR] SONIC writer control socket must be absolute" >&2
        exit 2
    fi
    if ! select_physical_recovery_python; then
        exit 1
    fi
    echo "[INFO] Physical recovery Python: $PHYSICAL_RECOVERY_PYTHON"
fi

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
if [[ "$CONTROL_SOURCE" == "game" ]]; then
    for required in \
        "$PROJECT_ROOT/scripts/matrix_game_control_input.py" \
        "$PROJECT_ROOT/scripts/matrix_external_control.py" \
        "$PROJECT_ROOT/scripts/matrix_calibration_overlay.py" \
        "$PROJECT_ROOT/scripts/matrix_mc_commands.py" \
        "$PROJECT_ROOT/scripts/matrix_motion_settings.py" \
        "$PROJECT_ROOT/scripts/matrix_spawn_clearance.py" \
        "$PROJECT_ROOT/scripts/matrix_world_state.py" \
        "$PROJECT_ROOT/scripts/prepare_sonic_physics_model.py" \
        "$PROJECT_ROOT/scripts/compose_custom_scene.py"; do
        if [[ ! -f "$required" ]]; then
            echo "[ERROR] Matrix game-control dependency is missing: $required" >&2
            exit 1
        fi
    done
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
export MATRIX_GAME_WORLD_PERSISTENCE="$GAME_WORLD_PERSISTENCE"
export MATRIX_GAME_AUTO_RESPAWN="$GAME_AUTO_RESPAWN"
export MATRIX_GAME_WORLD_CHECKPOINT_SECONDS="$GAME_WORLD_CHECKPOINT_SECONDS"
export MATRIX_GAME_FALL_RECOVERY="$GAME_FALL_RECOVERY"
export MATRIX_GAME_FALL_RECOVERY_TIMEOUT="$GAME_FALL_RECOVERY_TIMEOUT"
export MATRIX_PHYSICAL_RECOVERY_WORKER="$PHYSICAL_RECOVERY_WORKER"
export MATRIX_PHYSICAL_RECOVERY_INITIAL_CONTROLLER="$PHYSICAL_RECOVERY_INITIAL_CONTROLLER"
export MATRIX_PHYSICAL_RECOVERY_HANDOFF="$PHYSICAL_RECOVERY_HANDOFF"
export MATRIX_PHYSICAL_RECOVERY_RESIDENT_POLICIES="$PHYSICAL_RECOVERY_RESIDENT_POLICIES"
export MATRIX_PHYSICAL_RECOVERY_EXECUTION_PROVIDER="$PHYSICAL_RECOVERY_EXECUTION_PROVIDER"
export MATRIX_PHYSICAL_RECOVERY_PYTHON="$PHYSICAL_RECOVERY_PYTHON"
export MATRIX_PHYSICAL_RECOVERY_MODEL="$PHYSICAL_RECOVERY_MODEL"
export MATRIX_PHYSICAL_RECOVERY_MODEL_SHA256="$PHYSICAL_RECOVERY_MODEL_SHA256"
export MATRIX_PHYSICAL_RECOVERY_FALLBACK_MODEL="$PHYSICAL_RECOVERY_FALLBACK_MODEL"
export MATRIX_PHYSICAL_RECOVERY_AMP_CONFIG="$PHYSICAL_RECOVERY_AMP_CONFIG"
export MATRIX_PHYSICAL_RECOVERY_AMP_MODEL="$PHYSICAL_RECOVERY_AMP_MODEL"
export MATRIX_PHYSICAL_RECOVERY_AMP_CONFIG_SHA256="$PHYSICAL_RECOVERY_AMP_CONFIG_SHA256"
export MATRIX_PHYSICAL_RECOVERY_AMP_MODEL_SHA256="$PHYSICAL_RECOVERY_AMP_MODEL_SHA256"
export MATRIX_KUNGFU_RECOVERY_MODEL="$PHYSICAL_RECOVERY_KUNGFU_MODEL"
export MATRIX_KUNGFU_RECOVERY_MOTION="$PHYSICAL_RECOVERY_KUNGFU_MOTION"
export MATRIX_KUNGFU_RECOVERY_MODEL_SHA256="$PHYSICAL_RECOVERY_KUNGFU_MODEL_SHA256"
export MATRIX_KUNGFU_RECOVERY_MODEL_DATA_SHA256="$PHYSICAL_RECOVERY_KUNGFU_MODEL_DATA_SHA256"
export MATRIX_KUNGFU_RECOVERY_MOTION_SHA256="$PHYSICAL_RECOVERY_KUNGFU_MOTION_SHA256"
export MATRIX_KUNGFU_RECOVERY_REFERENCE_FRAME="$PHYSICAL_RECOVERY_KUNGFU_REFERENCE_FRAME"
export MATRIX_KUNGFU_RECOVERY_GAIN_SCALE="$PHYSICAL_RECOVERY_KUNGFU_GAIN_SCALE"
export MATRIX_PHYSICAL_RECOVERY_FALLBACK_AFTER_SECONDS="$PHYSICAL_RECOVERY_FALLBACK_AFTER_SECONDS"
export MATRIX_PHYSICAL_RECOVERY_STABLE_HOLD_SECONDS="$PHYSICAL_RECOVERY_STABLE_HOLD_SECONDS"
export MATRIX_PHYSICAL_RECOVERY_POLICY_EXIT_HOLD_SECONDS="$PHYSICAL_RECOVERY_POLICY_EXIT_HOLD_SECONDS"
export MATRIX_PHYSICAL_RECOVERY_CONTROL_SOCKET="$PHYSICAL_RECOVERY_CONTROL_SOCKET"
export MATRIX_PHYSICAL_RECOVERY_SONIC_CONTROL_SOCKET="$PHYSICAL_RECOVERY_SONIC_CONTROL_SOCKET"
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
if [[ "$GAME_AUTO_RESPAWN" == "1" \
    || "$GAME_FALL_RECOVERY" == "sonic" \
    || "$GAME_FALL_RECOVERY" == "physical" ]]; then
    export MATRIX_SONIC_FAIL_ON_FALL=0
else
    export MATRIX_SONIC_FAIL_ON_FALL=1
fi
export MATRIX_SONIC_STARTUP_BAND="$STARTUP_BAND"
export MATRIX_SONIC_STARTUP_BAND_HOLD="$STARTUP_BAND_HOLD"
export MATRIX_SONIC_STARTUP_BAND_FADE="$STARTUP_BAND_FADE"

# Matrix's upstream launcher rewrites these tracked files. Restore the exact
# pre-launch bytes so switching the same feature branch on two hosts stays clean.
CONFIG_BACKUP="$(mktemp -d "${TMPDIR:-/tmp}/matrix-sonic-config.XXXXXX")"
GAME_RUNTIME_DIR=""
EXTERNAL_CONTROL_RUNTIME_DIR=""
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
    EXTERNAL_CONTROL_SOCKET="${MATRIX_GAME_EXTERNAL_CONTROL_SOCKET:-}"
    EXTERNAL_CONTROL_CAPABILITY_FILE="${MATRIX_GAME_EXTERNAL_CONTROL_CAPABILITY_FILE:-}"
    if [[ -n "$EXTERNAL_CONTROL_SOCKET" \
        && -z "$EXTERNAL_CONTROL_CAPABILITY_FILE" ]] \
        || [[ -z "$EXTERNAL_CONTROL_SOCKET" \
            && -n "$EXTERNAL_CONTROL_CAPABILITY_FILE" ]]; then
        echo "[ERROR] Matrix external-control socket/capability are all-or-none" >&2
        exit 2
    fi
    if [[ -z "$EXTERNAL_CONTROL_SOCKET" ]]; then
        EXTERNAL_CONTROL_PROFILE="${MATRIX_PROFILE:-local}"
        if [[ ! "$EXTERNAL_CONTROL_PROFILE" =~ ^[A-Za-z0-9_.-]{1,64}$ ]]; then
            echo "[ERROR] Matrix external-control profile is invalid: $EXTERNAL_CONTROL_PROFILE" >&2
            exit 2
        fi
        EXTERNAL_CONTROL_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}/matrix-external-control-${UID}"
        if ! /usr/bin/python3 -I - "$EXTERNAL_CONTROL_RUNTIME_DIR" <<'PY'
import os
from pathlib import Path
import stat
import sys

path = Path(sys.argv[1])
try:
    path.mkdir(mode=0o700)
except FileExistsError:
    pass
metadata = path.stat(follow_symlinks=False)
if not stat.S_ISDIR(metadata.st_mode) or metadata.st_uid != os.getuid():
    raise SystemExit("external-control runtime path is not an owned directory")
os.chmod(path, 0o700, follow_symlinks=False)
if stat.S_IMODE(path.stat(follow_symlinks=False).st_mode) != 0o700:
    raise SystemExit("external-control runtime directory is not private")
PY
        then
            echo "[ERROR] Could not prepare private Matrix external-control runtime directory" >&2
            exit 2
        fi
        EXTERNAL_CONTROL_SOCKET="$EXTERNAL_CONTROL_RUNTIME_DIR/$EXTERNAL_CONTROL_PROFILE.sock"
        EXTERNAL_CONTROL_CAPABILITY_FILE="$EXTERNAL_CONTROL_RUNTIME_DIR/$EXTERNAL_CONTROL_PROFILE.cap"
    fi
    export MATRIX_GAME_EXTERNAL_CONTROL_SOCKET="$EXTERNAL_CONTROL_SOCKET"
    export MATRIX_GAME_EXTERNAL_CONTROL_CAPABILITY_FILE="$EXTERNAL_CONTROL_CAPABILITY_FILE"
    export MATRIX_GAME_EXTERNAL_CONTROL_DEADMAN_SECONDS="${MATRIX_GAME_EXTERNAL_CONTROL_DEADMAN_SECONDS:-0.15}"
    if ! /usr/bin/python3 -I - \
        "$MATRIX_GAME_INPUT_SOCKET" \
        "$MATRIX_GAME_EXTERNAL_CONTROL_SOCKET" \
        "$MATRIX_GAME_EXTERNAL_CONTROL_CAPABILITY_FILE" \
        "$MATRIX_GAME_RESTART_REQUEST_FILE" \
        "$MATRIX_GAME_RESTART_CAPABILITY_FILE" \
        "$MATRIX_GAME_EXTERNAL_CONTROL_DEADMAN_SECONDS" <<'PY'
import math
from pathlib import Path
import sys

paths = [Path(value) for value in sys.argv[1:6]]
if any(not path.is_absolute() for path in paths):
    raise SystemExit("all Matrix game IPC paths must be absolute")
if any(not path.parent.is_dir() for path in paths):
    raise SystemExit("all Matrix game IPC parent directories must exist")
canonical = [path.resolve(strict=False) for path in paths]
if len(set(canonical)) != len(canonical):
    raise SystemExit("all Matrix game IPC paths must be strictly distinct")
try:
    deadman = float(sys.argv[6])
except ValueError as exc:
    raise SystemExit("external-control deadman is not numeric") from exc
if not math.isfinite(deadman) or not 0.01 <= deadman <= 0.15:
    raise SystemExit("external-control deadman must be in [0.01, 0.15]")
PY
    then
        echo "[ERROR] Invalid Matrix external-control IPC configuration" >&2
        exit 2
    fi
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
    # An unexpected shell exit must not remove the private gate directory while
    # a rollback transaction or its authorizer still has it open.  These
    # functions are defined later, so guard the early preflight EXIT path.
    if declare -F cancel_rollback_helper >/dev/null 2>&1; then
        cancel_rollback_helper
    fi
    if declare -F reap_rollback_gate_children >/dev/null 2>&1; then
        reap_rollback_gate_children
    fi
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
ROLLBACK_HELPER_PID=""
ROLLBACK_AUTHORIZER_PID=""
ROLLBACK_CANCEL_FILE=""
FORWARDED_SIGNAL_EXIT_CODE=0
RESTART_REQUEST_VALID=0
RESTART_EXPECTED_EXIT_CODE=143
GAME_RESUME_ROLLBACK_COUNT="${MATRIX_GAME_RESUME_ROLLBACK_COUNT:-0}"
# Keep this equal to matrix_world_state.MAX_RESUME_CHECKPOINTS: each failed
# generation may quarantine exactly one member of the bounded resume ring.
GAME_RESUME_ROLLBACK_MAX=16
NEXT_GAME_RESUME_ROLLBACK_COUNT="$GAME_RESUME_ROLLBACK_COUNT"
GAME_RESUME_ROLLBACK_WINDOW="${MATRIX_GAME_RESUME_ROLLBACK_WINDOW_EPOCH:-0}"
GAME_RESUME_ROLLBACK_RATE_COUNT="${MATRIX_GAME_RESUME_ROLLBACK_RATE_COUNT:-0}"
GAME_RESUME_ROLLBACK_RATE_MAX="${MATRIX_GAME_RESUME_ROLLBACK_MAX_PER_MINUTE:-16}"
INTERNAL_RESTART_WINDOW="${MATRIX_GAME_INTERNAL_RESTART_WINDOW_EPOCH:-0}"
INTERNAL_RESTART_COUNT="${MATRIX_GAME_INTERNAL_RESTART_COUNT:-0}"
INTERNAL_RESTART_MAX="${MATRIX_GAME_INTERNAL_RESTART_MAX_PER_MINUTE:-16}"
STOP_REQUESTED=0
FORCED_STOP=0
INTERNAL_RESTART_TIMEOUT=0
RUN_SIM_STOP_TIMEOUT_SECONDS="${MATRIX_RUN_SIM_STOP_TIMEOUT_SECONDS:-25}"
if [[ ! "$RUN_SIM_STOP_TIMEOUT_SECONDS" \
    =~ ^([1-9][0-9]*(\.[0-9]+)?|0\.[0-9]*[1-9][0-9]*)$ ]]; then
    echo "[ERROR] MATRIX_RUN_SIM_STOP_TIMEOUT_SECONDS must be positive" >&2
    exit 2
fi
if [[ ! "$GAME_RESUME_ROLLBACK_COUNT" =~ ^[0-9]+$ ]] \
    || ((GAME_RESUME_ROLLBACK_COUNT > GAME_RESUME_ROLLBACK_MAX)); then
    echo "[ERROR] MATRIX_GAME_RESUME_ROLLBACK_COUNT must be in " \
        "[0, $GAME_RESUME_ROLLBACK_MAX]" >&2
    exit 2
fi
if [[ ! "$GAME_RESUME_ROLLBACK_WINDOW" =~ ^[0-9]+$ \
    || ! "$GAME_RESUME_ROLLBACK_RATE_COUNT" =~ ^[0-9]+$ \
    || ! "$GAME_RESUME_ROLLBACK_RATE_MAX" =~ ^[1-9][0-9]*$ ]]; then
    echo "[ERROR] Invalid Matrix resume-rollback rate guard state" >&2
    exit 2
fi
if [[ ! "$INTERNAL_RESTART_WINDOW" =~ ^[0-9]+$ \
    || ! "$INTERNAL_RESTART_COUNT" =~ ^[0-9]+$ \
    || ! "$INTERNAL_RESTART_MAX" =~ ^[1-9][0-9]*$ ]]; then
    echo "[ERROR] Invalid Matrix internal-restart guard state" >&2
    exit 2
fi
publish_rollback_gate_marker_exec() {
    local marker_path="$1"
    local marker_payload="$2"
    # The asynchronous authorizer calls this function directly.  exec keeps
    # $! bound to the actual publisher for its whole lifetime, so a signal
    # cannot leave an untracked Python descendant behind.
    exec /usr/bin/python3 -I - "$marker_path" "$marker_payload" <<'PY'
import os
from pathlib import Path
import secrets
import stat
import sys

path = Path(sys.argv[1])
payload = (sys.argv[2] + "\n").encode("ascii")
if not path.is_absolute() or path.name in {"", ".", ".."}:
    raise SystemExit(1)
directory_flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
directory_flags |= getattr(os, "O_CLOEXEC", 0)
directory_flags |= getattr(os, "O_NOFOLLOW", 0)
directory_fd = os.open(path.parent, directory_flags)
temporary_name = f".{path.name}.{os.getpid()}.{secrets.token_hex(8)}"
temporary_fd = None
try:
    directory_stat = os.fstat(directory_fd)
    if (
        not stat.S_ISDIR(directory_stat.st_mode)
        or directory_stat.st_uid != os.getuid()
        or directory_stat.st_mode & 0o077
    ):
        raise SystemExit(1)
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    flags |= getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    temporary_fd = os.open(temporary_name, flags, 0o600, dir_fd=directory_fd)
    offset = 0
    while offset < len(payload):
        offset += os.write(temporary_fd, payload[offset:])
    os.fsync(temporary_fd)
    os.close(temporary_fd)
    temporary_fd = None
    # The private gate directory and fixed payload make replacing an identical
    # repeated cancel marker safe. rename publishes a fully fsynced, nlink=1
    # inode in one step, so the polling helper can never observe partial bytes
    # or the transient two-link state created by a link/unlink protocol.
    os.replace(
        temporary_name,
        path.name,
        src_dir_fd=directory_fd,
        dst_dir_fd=directory_fd,
    )
    os.fsync(directory_fd)
finally:
    if temporary_fd is not None:
        os.close(temporary_fd)
    try:
        os.unlink(temporary_name, dir_fd=directory_fd)
    except FileNotFoundError:
        pass
    os.close(directory_fd)
PY
}

publish_rollback_gate_marker() {
    # Synchronous callers still need the launcher shell after publication.
    (publish_rollback_gate_marker_exec "$@")
}

child_job_is_running() {
    local expected_pid="$1"
    local child_pid
    while IFS= read -r child_pid; do
        if [[ "$child_pid" == "$expected_pid" ]]; then
            return 0
        fi
    done < <(jobs -pr)
    return 1
}

WAITED_CHILD_STATUS=127
wait_for_tracked_child() {
    local child_pid="$1"
    local last_status=127
    local wait_status=127

    # A foreground wait can be interrupted by one of our traps while the child
    # remains live.  Consult Bash's own job table and retry only while this exact
    # child job is still running; never inspect /proc after a reap.
    while child_job_is_running "$child_pid"; do
        wait "$child_pid"
        last_status=$?
    done
    # If the job completed between the job-table check and wait, one final wait
    # reaps it.  If an earlier wait already reaped it, Bash returns 127 and the
    # earlier status is retained.  Bash's job table prevents PID-reuse aliasing.
    wait "$child_pid" 2>/dev/null
    wait_status=$?
    if [[ "$wait_status" != "127" || "$last_status" == "127" ]]; then
        last_status="$wait_status"
    fi
    WAITED_CHILD_STATUS="$last_status"
}

cancel_rollback_helper() {
    # Signal the transactional helper first.  Before its commit point SIGTERM
    # makes the operation fail closed; after that point its handler deliberately
    # finishes both replicas.  Only then stop the exact authorizer job and add a
    # durable cancel marker as a second, filesystem-level cancellation channel.
    if [[ -n "$ROLLBACK_HELPER_PID" ]] \
        && child_job_is_running "$ROLLBACK_HELPER_PID"; then
        kill -TERM "$ROLLBACK_HELPER_PID" 2>/dev/null || true
    fi
    if [[ -n "$ROLLBACK_AUTHORIZER_PID" ]] \
        && child_job_is_running "$ROLLBACK_AUTHORIZER_PID"; then
        kill -TERM "$ROLLBACK_AUTHORIZER_PID" 2>/dev/null || true
    fi
    if [[ -n "$ROLLBACK_CANCEL_FILE" ]]; then
        publish_rollback_gate_marker \
            "$ROLLBACK_CANCEL_FILE" \
            matrix-world-state-reject-cancel/v1 2>/dev/null || true
    fi
}

reap_rollback_gate_children() {
    if [[ -n "$ROLLBACK_AUTHORIZER_PID" ]]; then
        wait_for_tracked_child "$ROLLBACK_AUTHORIZER_PID"
        ROLLBACK_AUTHORIZER_PID=""
    fi
    if [[ -n "$ROLLBACK_HELPER_PID" ]]; then
        wait_for_tracked_child "$ROLLBACK_HELPER_PID"
        ROLLBACK_HELPER_PID=""
    fi
    ROLLBACK_CANCEL_FILE=""
}

forward_signal() {
    local signal_name="$1"
    local exit_code="$2"
    FORWARDED_SIGNAL_EXIT_CODE="$exit_code"
    STOP_REQUESTED=1
    cancel_rollback_helper
    if [[ -n "$RUN_SIM_PID" ]] && kill -0 "$RUN_SIM_PID" 2>/dev/null; then
        kill "-$signal_name" "$RUN_SIM_PID" 2>/dev/null || true
    fi
}
trap 'forward_signal INT 130' SIGINT
trap 'forward_signal TERM 143' SIGTERM
trap 'forward_signal HUP 129' SIGHUP

run_gated_resume_rejection() {
    local ready_file="$GAME_RUNTIME_DIR/world-state-reject-ready"
    local authorize_file="$GAME_RUNTIME_DIR/world-state-reject-authorize"
    local cancel_file="$GAME_RUNTIME_DIR/world-state-reject-cancel"
    local result_file="$GAME_RUNTIME_DIR/world-state-reject-result.json"
    local error_file="$GAME_RUNTIME_DIR/world-state-reject-error.log"
    local ready_seen=0
    local helper_status=1
    local authorize_status=1
    local poll

    ROLLBACK_CANCEL_FILE="$cancel_file"
    (
        umask 077
        exec /usr/bin/python3 -I \
            "$PROJECT_ROOT/scripts/matrix_world_state.py" \
            reject-checkpoint \
            --file "$ROLLBACK_STATE_FILE" \
            --world-id "$ROLLBACK_WORLD_ID" \
            --world-revision "$ROLLBACK_WORLD_REVISION" \
            --checkpoint-id "$ROLLBACK_CHECKPOINT_ID" \
            --expected-generation "$ROLLBACK_GENERATION" \
            --reason "$ROLLBACK_REJECTION_REASON" \
            --run-id "$ROLLBACK_RUN_ID" \
            --commit-ready-file "$ready_file" \
            --commit-authorize-file "$authorize_file" \
            --commit-cancel-file "$cancel_file" \
            --commit-timeout-seconds 15
    ) >"$result_file" 2>"$error_file" &
    ROLLBACK_HELPER_PID=$!
    chmod 600 "$result_file" "$error_file" 2>/dev/null || true

    # The helper publishes ready only after acquiring the state lock and
    # validating the exact checkpoint ID/generation. Keep this loop in the
    # shell so INT/TERM/HUP traps can cancel the still-reversible transaction.
    for ((poll = 0; poll < 800; poll++)); do
        if [[ -f "$ready_file" && ! -L "$ready_file" ]]; then
            ready_seen=1
            break
        fi
        if [[ "$FORWARDED_SIGNAL_EXIT_CODE" != "0" \
            || "$FORCED_STOP" != "0" ]] \
            || ! child_job_is_running "$ROLLBACK_HELPER_PID"; then
            break
        fi
        sleep 0.02
    done

    if [[ "$ready_seen" != "1" ]]; then
        cancel_rollback_helper
        # No authorize marker exists, so a helper that is still blocked before
        # ready has no world-state write authority and may be boundedly killed.
        for ((poll = 0; poll < 100; poll++)); do
            ! child_job_is_running "$ROLLBACK_HELPER_PID" && break
            sleep 0.01
        done
        if child_job_is_running "$ROLLBACK_HELPER_PID"; then
            kill -KILL "$ROLLBACK_HELPER_PID" 2>/dev/null || true
        fi
    else
        # Give a pending launcher signal one scheduling turn before publishing
        # authorization. This delay is paid only after an actual bad-resume
        # proposal, never on a normal launch.
        sleep 0.10
        if [[ "$FORWARDED_SIGNAL_EXIT_CODE" == "0" \
            && "$FORCED_STOP" == "0" ]]; then
            publish_rollback_gate_marker_exec \
                "$authorize_file" \
                matrix-world-state-reject-authorize/v1 &
            ROLLBACK_AUTHORIZER_PID=$!
            wait_for_tracked_child "$ROLLBACK_AUTHORIZER_PID"
            authorize_status="$WAITED_CHILD_STATUS"
            ROLLBACK_AUTHORIZER_PID=""
            if [[ "$authorize_status" != "0" ]]; then
                cancel_rollback_helper
            fi
        else
            cancel_rollback_helper
        fi
    fi

    wait_for_tracked_child "$ROLLBACK_HELPER_PID"
    helper_status="$WAITED_CHILD_STATUS"
    ROLLBACK_HELPER_PID=""
    ROLLBACK_AUTHORIZER_PID=""
    ROLLBACK_CANCEL_FILE=""

    if [[ "$helper_status" != "0" ]]; then
        if [[ -s "$error_file" ]]; then
            echo "[ERROR] Matrix resume rollback helper: " \
                "$(head -c 2048 "$error_file")" >&2
        fi
        return 1
    fi
    if [[ ! -f "$result_file" || -L "$result_file" ]]; then
        return 1
    fi
    ROLLBACK_REJECTION_JSON="$(<"$result_file")"
    [[ -n "$ROLLBACK_REJECTION_JSON" ]]
}

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
# The exact run_sim child has been reaped. Clear its numeric PID before the
# longer status-validation/rollback commit path so a late external signal can
# never be forwarded to an unrelated process that reused the number.
RUN_SIM_PID=""
if [[ "$FORWARDED_SIGNAL_EXIT_CODE" != "0" ]]; then
    exit_code="$FORWARDED_SIGNAL_EXIT_CODE"
fi
if [[ "$FORWARDED_SIGNAL_EXIT_CODE" == "0" \
    && "$FORCED_STOP" == "0" \
    && "$exit_code" == "76" \
    && "$GAME_WORLD_PERSISTENCE" == "1" ]]; then
    if ((GAME_RESUME_ROLLBACK_COUNT >= GAME_RESUME_ROLLBACK_MAX)); then
        echo "[ERROR] Matrix resume rollback limit reached; " \
            "leaving the runtime stopped" >&2
    elif VERIFIED_ROLLBACK_OUTPUT="$(
        /usr/bin/python3 -I - "$MATRIX_SONIC_STATUS_FILE" <<'PY'
import json
import math
from pathlib import Path
import re
import sys

path = Path(sys.argv[1])
if not path.is_file() or path.is_symlink():
    raise SystemExit(1)
try:
    status = json.loads(path.read_text(encoding="utf-8"))
except (OSError, UnicodeError, json.JSONDecodeError):
    raise SystemExit(1)
if not isinstance(status, dict):
    raise SystemExit(1)
termination_reason = status.get("termination_reason")
if termination_reason not in {"numerical_instability", "spawn_clearance_failed"}:
    raise SystemExit(1)
if "termination_signal" not in status or status["termination_signal"] is not None:
    raise SystemExit(1)
for field in ("failed_child_name", "failed_child_exit_code"):
    if field not in status or status[field] is not None:
        raise SystemExit(1)
elapsed_wall_s = status.get("elapsed_wall_s")
if (
    isinstance(elapsed_wall_s, bool)
    or not isinstance(elapsed_wall_s, (int, float))
    or not math.isfinite(float(elapsed_wall_s))
    or float(elapsed_wall_s) < 0.0
):
    raise SystemExit(1)
numerical_error = status.get("numerical_error")
dynamic_resume_clearance = False
if termination_reason == "numerical_instability":
    if float(elapsed_wall_s) > 5.0:
        raise SystemExit(1)
    if not isinstance(numerical_error, str) or not numerical_error.startswith(
        ("snapshot_non_finite:", "snapshot_sim_time_not_increasing:")
    ):
        raise SystemExit(1)
    rollback_reason = "startup_numerical_instability"
else:
    clearance = status.get("spawn_clearance")
    if not isinstance(clearance, dict):
        raise SystemExit(1)
    clearance_reason = clearance.get("reason")
    if (
        clearance.get("schema") != "matrix-spawn-clearance-audit/v1"
        or clearance.get("safe") is not False
        or clearance.get("error") is not None
        or clearance_reason not in {"scene_penetration", "unsafe_foot_contact"}
    ):
        raise SystemExit(1)
    rejected_count = clearance.get("rejected_contact_count")
    worst = clearance.get("worst")
    if (
        isinstance(rejected_count, bool)
        or not isinstance(rejected_count, int)
        or rejected_count <= 0
        or not isinstance(worst, dict)
        or worst.get("allowed") is not False
    ):
        raise SystemExit(1)
    classification = worst.get("classification")
    if clearance_reason == "scene_penetration":
        if classification != "scene_penetration":
            raise SystemExit(1)
    elif classification not in {
        "unsafe_foot_contact_normal",
        "unsafe_foot_penetration",
    }:
        raise SystemExit(1)
    rollback_reason = f"spawn_clearance:{clearance_reason}"
    if numerical_error != rollback_reason:
        raise SystemExit(1)
    probation = status.get("resume_probation")
    dynamic_resume_clearance = bool(
        isinstance(probation, dict)
        and probation.get("enabled") is True
        and probation.get("active") is True
        and probation.get("completed") is False
        and probation.get("failed") is True
        and probation.get("phase") == "failed"
        and probation.get("checkpoint_writes_blocked") is True
        and probation.get("failure_reason") == clearance_reason
        and probation.get("last_clearance_audit") == clearance
        and probation.get("stable_idle_required_s") == 1.5
        and probation.get("stable_idle_clock") == "sim_time"
        and probation.get("audit_interval_s") == 0.1
        and type(probation.get("first_fresh_lowcmd_observed")) is bool
        and type(probation.get("startup_band_released")) is bool
        and isinstance(probation.get("stable_idle_elapsed_s"), (int, float))
        and not isinstance(probation.get("stable_idle_elapsed_s"), bool)
        and math.isfinite(float(probation.get("stable_idle_elapsed_s")))
        and float(probation.get("stable_idle_elapsed_s")) >= 0.0
        and probation.get("stable_idle_sim_elapsed_s")
        == probation.get("stable_idle_elapsed_s")
        and probation.get("sim_time_sample_valid") is True
        and isinstance(probation.get("current_sim_time_s"), (int, float))
        and not isinstance(probation.get("current_sim_time_s"), bool)
        and math.isfinite(float(probation.get("current_sim_time_s")))
        and float(probation.get("current_sim_time_s")) >= 0.0
        and probation.get("max_sim_sample_gap_s") == 0.05
        and isinstance(probation.get("audit_count"), int)
        and not isinstance(probation.get("audit_count"), bool)
        and probation.get("audit_count") > 0
    )
    if not dynamic_resume_clearance and float(elapsed_wall_s) > 5.0:
        raise SystemExit(1)
for field in ("completed", "interrupted", "passed"):
    if status.get(field) is not False:
        raise SystemExit(1)
if type(status.get("fall_detected")) is not bool:
    raise SystemExit(1)
for field in ("active_lowcmd", "low_cmd_received"):
    if type(status.get(field)) is not bool:
        raise SystemExit(1)
for field in ("active_elapsed_s", "active_lowcmd_longest_s"):
    value = status.get(field)
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or float(value) < 0.0
    ):
        raise SystemExit(1)
active_frames = status.get("active_frames")
if (
    isinstance(active_frames, bool)
    or not isinstance(active_frames, int)
    or active_frames < 0
):
    raise SystemExit(1)
if dynamic_resume_clearance:
    probation = status["resume_probation"]
    if status.get("fall_detected") is not False:
        raise SystemExit(1)
    if (
        status.get("active_lowcmd") is True
        and status.get("low_cmd_received") is not True
    ):
        raise SystemExit(1)
    if (
        status.get("low_cmd_received") is True
        and probation.get("first_fresh_lowcmd_observed") is not True
    ):
        raise SystemExit(1)
else:
    if (
        status.get("active_lowcmd") is not False
        or status.get("low_cmd_received") is not False
    ):
        raise SystemExit(1)
    if active_frames != 0:
        raise SystemExit(1)
    if any(
        float(status[field]) != 0.0
        for field in ("active_elapsed_s", "active_lowcmd_longest_s")
    ):
        raise SystemExit(1)
if (
    status.get("instability_resets") != 0
    or isinstance(status.get("instability_resets"), bool)
    or status.get("last_reset_reason") is not None
):
    raise SystemExit(1)
internal = status.get("internal_restart")
if not isinstance(internal, dict):
    raise SystemExit(1)
if internal.get("requested") is not False or internal.get("reason") is not None:
    raise SystemExit(1)
run_id = status.get("run_id")
if not isinstance(run_id, str) or re.fullmatch(r"[0-9a-f]{32}", run_id) is None:
    raise SystemExit(1)
world = status.get("game_world_state")
if not isinstance(world, dict) or world.get("last_error") is not None:
    raise SystemExit(1)
rollback = world.get("resume_rollback")
if not isinstance(rollback, dict):
    raise SystemExit(1)
if rollback.get("requested") is not True or rollback.get("applied") is not False:
    raise SystemExit(1)
if rollback.get("reason") != rollback_reason:
    raise SystemExit(1)
if rollback.get("run_id") != run_id:
    raise SystemExit(1)
checkpoint_id = rollback.get("rejected_checkpoint_id")
if (
    not isinstance(checkpoint_id, str)
    or re.fullmatch(r"cp-[0-9a-f]{32}", checkpoint_id) is None
    or world.get("selected_resume_checkpoint_id") != checkpoint_id
    or world.get("active_resume_checkpoint_id") != checkpoint_id
):
    raise SystemExit(1)
if (
    dynamic_resume_clearance
    and status["resume_probation"].get("selected_checkpoint_id") != checkpoint_id
):
    raise SystemExit(1)
generation = rollback.get("rejected_generation")
if (
    isinstance(generation, bool)
    or not isinstance(generation, int)
    or generation < 0
    or world.get("selected_resume_generation") != generation
    or world.get("generation") != generation
):
    raise SystemExit(1)
if world.get("resume_rollback_ineligibility") is not None:
    raise SystemExit(1)
state_path = world.get("path")
world_id = world.get("world_id")
world_revision = world.get("world_revision")
if (
    not isinstance(state_path, str)
    or not state_path.startswith("/")
    or len(state_path) > 4096
    or any(ord(character) < 0x20 or ord(character) == 0x7F for character in state_path)
):
    raise SystemExit(1)
if (
    not isinstance(world_id, str)
    or re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.:/-]{0,159}", world_id) is None
):
    raise SystemExit(1)
if (
    not isinstance(world_revision, str)
    or not world_revision
    or len(world_revision) > 256
    or any(ord(character) < 0x21 or ord(character) > 0x7E for character in world_revision)
):
    raise SystemExit(1)
for value in (
    state_path,
    world_id,
    world_revision,
    checkpoint_id,
    str(generation),
    run_id,
    rollback_reason,
):
    print(value)
PY
    )"; then
        mapfile -t VERIFIED_ROLLBACK_FIELDS <<<"$VERIFIED_ROLLBACK_OUTPUT"
        if [[ "${#VERIFIED_ROLLBACK_FIELDS[@]}" != "7" ]]; then
            echo "[ERROR] Refusing malformed Matrix resume rollback proposal" >&2
        else
            ROLLBACK_STATE_FILE="${VERIFIED_ROLLBACK_FIELDS[0]}"
            ROLLBACK_WORLD_ID="${VERIFIED_ROLLBACK_FIELDS[1]}"
            ROLLBACK_WORLD_REVISION="${VERIFIED_ROLLBACK_FIELDS[2]}"
            ROLLBACK_CHECKPOINT_ID="${VERIFIED_ROLLBACK_FIELDS[3]}"
            ROLLBACK_GENERATION="${VERIFIED_ROLLBACK_FIELDS[4]}"
            ROLLBACK_RUN_ID="${VERIFIED_ROLLBACK_FIELDS[5]}"
            ROLLBACK_REJECTION_REASON="${VERIFIED_ROLLBACK_FIELDS[6]}"
            EXPECTED_ROLLBACK_STATE_FILE="${MATRIX_GAME_WORLD_STATE_FILE:-}"
            if [[ -z "$EXPECTED_ROLLBACK_STATE_FILE" ]]; then
                EXPECTED_ROLLBACK_STATE_FILE="$(
                    /usr/bin/python3 -I \
                        "$PROJECT_ROOT/scripts/matrix_world_state.py" \
                        default-path \
                        --profile "${MATRIX_PROFILE:-local}" \
                        --world-id "$ROLLBACK_WORLD_ID"
                )" || EXPECTED_ROLLBACK_STATE_FILE=""
            fi
            if [[ -z "$EXPECTED_ROLLBACK_STATE_FILE" \
                || "$ROLLBACK_STATE_FILE" != "$EXPECTED_ROLLBACK_STATE_FILE" ]]; then
                echo "[ERROR] Refusing Matrix resume rollback for an unexpected " \
                    "world-state path" >&2
            else
                ROLLBACK_RESTART_NOW="$(date +%s)"
                if ((GAME_RESUME_ROLLBACK_WINDOW == 0 \
                    || ROLLBACK_RESTART_NOW - GAME_RESUME_ROLLBACK_WINDOW >= 60)); then
                    GAME_RESUME_ROLLBACK_WINDOW="$ROLLBACK_RESTART_NOW"
                    GAME_RESUME_ROLLBACK_RATE_COUNT=0
                fi
                GAME_RESUME_ROLLBACK_RATE_COUNT=$((GAME_RESUME_ROLLBACK_RATE_COUNT + 1))
                if ((GAME_RESUME_ROLLBACK_RATE_COUNT > GAME_RESUME_ROLLBACK_RATE_MAX)); then
                        echo "[ERROR] Matrix resume rollback rate limit exceeded; " \
                            "leaving the runtime stopped" >&2
                elif [[ "$FORWARDED_SIGNAL_EXIT_CODE" != "0" \
                        || "$FORCED_STOP" != "0" ]]; then
                    echo "[INFO] External stop cancelled the pending Matrix " \
                        "resume rollback" >&2
                elif run_gated_resume_rejection \
                        && /usr/bin/python3 -I - \
                        "$ROLLBACK_REJECTION_JSON" \
                        "$ROLLBACK_CHECKPOINT_ID" \
                        "$ROLLBACK_GENERATION" <<'PY'
import json
import re
import sys

try:
    payload = json.loads(sys.argv[1])
except json.JSONDecodeError:
    raise SystemExit(1)
if not isinstance(payload, dict):
    raise SystemExit(1)
if payload.get("schema") != "matrix-world-state-rejection/v1":
    raise SystemExit(1)
if payload.get("rejected_checkpoint_id") != sys.argv[2]:
    raise SystemExit(1)
if payload.get("idempotent") is not False:
    raise SystemExit(1)
generation = payload.get("generation")
if isinstance(generation, bool) or generation != int(sys.argv[3]) + 1:
    raise SystemExit(1)
replacement = payload.get("replacement_checkpoint_id")
if replacement is not None and (
    not isinstance(replacement, str)
    or re.fullmatch(r"cp-[0-9a-f]{32}", replacement) is None
    or replacement == sys.argv[2]
):
    raise SystemExit(1)
PY
                then
                    if [[ "$FORWARDED_SIGNAL_EXIT_CODE" != "0" \
                            || "$FORCED_STOP" != "0" ]]; then
                        echo "[INFO] Matrix resume rollback crossed its " \
                            "authorized commit point; checkpoint quarantine " \
                            "completed but the external stop cancelled restart" >&2
                    else
                        RESTART_REQUEST_VALID=1
                        RESTART_EXPECTED_EXIT_CODE=76
                        NEXT_GAME_RESUME_ROLLBACK_COUNT=$((GAME_RESUME_ROLLBACK_COUNT + 1))
                        echo "[INFO] Quarantined failed Matrix resume checkpoint " \
                            "id=$ROLLBACK_CHECKPOINT_ID generation=$ROLLBACK_GENERATION; " \
                            "validated fallback restart count=${GAME_RESUME_ROLLBACK_RATE_COUNT}/${GAME_RESUME_ROLLBACK_RATE_MAX}"
                    fi
                else
                    echo "[ERROR] Refusing Matrix resume rollback because the " \
                        "exact checkpoint rejection did not commit" >&2
                fi
            fi
        fi
    else
        echo "[ERROR] Refusing unverified Matrix resume rollback proposal" >&2
    fi
    if [[ "$RESTART_REQUEST_VALID" != "1" \
        || "$RESTART_EXPECTED_EXIT_CODE" != "76" ]]; then
        # Exit 76 is proposal authority only.  Any malformed, ineligible,
        # rejected, or rate-limited proposal is a normal runtime failure.
        exit_code=2
    fi
fi
if [[ "$FORWARDED_SIGNAL_EXIT_CODE" == "0" \
    && "$FORCED_STOP" == "0" \
    && "$exit_code" == "75" \
    && "$GAME_WORLD_PERSISTENCE" == "1" ]]; then
    if VERIFIED_INTERNAL_RESTART_REASON="$(
        /usr/bin/python3 -I - "$MATRIX_SONIC_STATUS_FILE" <<'PY'
import json
from pathlib import Path
import sys

path = Path(sys.argv[1])
if not path.is_file() or path.is_symlink():
    raise SystemExit(1)
try:
    status = json.loads(path.read_text(encoding="utf-8"))
except (OSError, UnicodeError, json.JSONDecodeError):
    raise SystemExit(1)
if not isinstance(status, dict):
    raise SystemExit(1)
internal = status.get("internal_restart")
world = status.get("game_world_state")
if not isinstance(internal, dict) or internal.get("requested") is not True:
    raise SystemExit(1)
reason = internal.get("reason")
if reason not in {"game_fall_respawn", "game_teleport"}:
    raise SystemExit(1)
if status.get("termination_reason") != reason:
    raise SystemExit(1)
if "termination_signal" not in status or status["termination_signal"] is not None:
    raise SystemExit(1)
for field in ("failed_child_name", "failed_child_exit_code"):
    if field not in status or status[field] is not None:
        raise SystemExit(1)
if not isinstance(world, dict):
    raise SystemExit(1)
if world.get("last_error") is not None:
    raise SystemExit(1)
if world.get("has_last_exit") is not True:
    raise SystemExit(1)
if reason == "game_fall_respawn" and status.get("game_auto_respawn") is not True:
    raise SystemExit(1)
print(reason)
PY
    )"; then
        INTERNAL_RESTART_NOW="$(date +%s)"
        if [[ "$INTERNAL_RESTART_WINDOW" == "0" ]] \
            || ((INTERNAL_RESTART_NOW - INTERNAL_RESTART_WINDOW >= 60)); then
                INTERNAL_RESTART_WINDOW="$INTERNAL_RESTART_NOW"
                INTERNAL_RESTART_COUNT=0
        fi
        INTERNAL_RESTART_COUNT=$((INTERNAL_RESTART_COUNT + 1))
        if ((INTERNAL_RESTART_COUNT <= INTERNAL_RESTART_MAX)); then
            RESTART_REQUEST_VALID=1
            RESTART_EXPECTED_EXIT_CODE=75
            echo "[INFO] Validated Matrix world reload " \
                "reason=$VERIFIED_INTERNAL_RESTART_REASON " \
                "count=${INTERNAL_RESTART_COUNT}/${INTERNAL_RESTART_MAX}"
        else
            echo "[ERROR] Matrix world reload rate limit exceeded; " \
                "leaving the runtime stopped" >&2
        fi
    else
        echo "[ERROR] Refusing unverified Matrix world reload request" >&2
    fi
fi
if [[ "$FORWARDED_SIGNAL_EXIT_CODE" == "0" \
    && "$FORCED_STOP" == "0" \
    && "$RESTART_REQUEST_VALID" == "1" \
    && "$RESTART_EXPECTED_EXIT_CODE" == "143" \
    && "$exit_code" == "143" \
    && "$GAME_WORLD_PERSISTENCE" == "1" ]]; then
    if /usr/bin/python3 -I - "$MATRIX_SONIC_STATUS_FILE" <<'PY'
import json
from pathlib import Path
import re
import signal
import sys

path = Path(sys.argv[1])
if not path.is_file() or path.is_symlink():
    raise SystemExit(1)
try:
    status = json.loads(path.read_text(encoding="utf-8"))
except (OSError, UnicodeError, json.JSONDecodeError):
    raise SystemExit(1)
if not isinstance(status, dict):
    raise SystemExit(1)
if status.get("termination_reason") != "signal":
    raise SystemExit(1)
if status.get("termination_signal") != signal.SIGTERM:
    raise SystemExit(1)
for field in ("failed_child_name", "failed_child_exit_code"):
    if field not in status or status[field] is not None:
        raise SystemExit(1)
internal = status.get("internal_restart")
if not isinstance(internal, dict):
    raise SystemExit(1)
if internal.get("requested") is not False or internal.get("reason") is not None:
    raise SystemExit(1)
world = status.get("game_world_state")
if not isinstance(world, dict) or world.get("has_last_exit") is not True:
    raise SystemExit(1)
if world.get("last_error") is not None:
    raise SystemExit(1)
run_id = status.get("run_id")
if not isinstance(run_id, str) or re.fullmatch(r"[0-9a-f]{32}", run_id) is None:
    raise SystemExit(1)
final_checkpoint = status.get("final_checkpoint")
if not isinstance(final_checkpoint, dict) or set(final_checkpoint) != {
    "schema",
    "run_id",
    "checkpoint_id",
    "generation",
}:
    raise SystemExit(1)
checkpoint_id = final_checkpoint.get("checkpoint_id")
generation = final_checkpoint.get("generation")
if (
    final_checkpoint.get("schema") != "matrix-final-world-checkpoint/v1"
    or final_checkpoint.get("run_id") != run_id
    or not isinstance(checkpoint_id, str)
    or re.fullmatch(r"cp-[0-9a-f]{32}", checkpoint_id) is None
    or isinstance(generation, bool)
    or not isinstance(generation, int)
    or generation < 0
    or world.get("active_resume_checkpoint_id") != checkpoint_id
    or world.get("generation") != generation
):
    raise SystemExit(1)
PY
    then
        echo "[INFO] Verified final Matrix world checkpoint for requested restart"
    else
        RESTART_REQUEST_VALID=0
        echo "[ERROR] Refusing Matrix restart without a verified final world checkpoint" >&2
    fi
fi
if [[ "$FORWARDED_SIGNAL_EXIT_CODE" == "0" \
    && "$RESTART_REQUEST_VALID" == "1" \
    && "$FORCED_STOP" == "0" \
    && "$exit_code" == "$RESTART_EXPECTED_EXIT_CODE" ]]; then
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
                MATRIX_GAME_INTERNAL_RESTART_WINDOW_EPOCH="${INTERNAL_RESTART_WINDOW:-0}" \
                MATRIX_GAME_INTERNAL_RESTART_COUNT="${INTERNAL_RESTART_COUNT:-0}" \
                MATRIX_GAME_RESUME_ROLLBACK_COUNT="$NEXT_GAME_RESUME_ROLLBACK_COUNT" \
                MATRIX_GAME_RESUME_ROLLBACK_WINDOW_EPOCH="$GAME_RESUME_ROLLBACK_WINDOW" \
                MATRIX_GAME_RESUME_ROLLBACK_RATE_COUNT="$GAME_RESUME_ROLLBACK_RATE_COUNT" \
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
