#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

TRACE=""
MODEL=""
MATRIX_ROOT="$PROJECT_ROOT"
STATE_DIR=""
STATUS_FILE="${MATRIX_SONIC_STATUS_FILE:-$PROJECT_ROOT/outputs/matrix_scene6_trace_replay_status.json}"
SUMMARY_FILE="$PROJECT_ROOT/outputs/matrix_scene6_trace_replay_summary.json"
PRE_ROLL_SECONDS="2"
FINAL_HOLD_SECONDS="6"
RESTORE_RECEIPT=""
CAMERA_MODE="${MATRIX_SCENE6_CAMERA_MODE:-robot}"
CAMERA_DISTANCE_CM="${MATRIX_SCENE6_CAMERA_DISTANCE_CM:-180}"
OVERLAY_BUNDLE="${MATRIX_CENTERED_CAMERA_OVERLAY_BUNDLE:-}"
OVERLAY_CONTRACT="${MATRIX_CENTERED_CAMERA_OVERLAY_CONTRACT:-}"
CAMERA_RECEIPT=""
CAMERA_READY_FILE=""
CAMERA_READY_TIMEOUT_SECONDS="120"
CAMERA_SETTLE_SECONDS="0.5"

usage() {
    echo "Usage: $0 --trace FILE [--model FILE] [--matrix-root DIR]" \
        "[--status-file FILE] [--summary FILE] [--pre-roll SECONDS]" \
        "[--final-hold SECONDS] [--camera-mode robot|spectator-overlay]" \
        "[--camera-distance-cm CM] [--overlay-bundle DIR]" \
        "[--overlay-contract FILE] [--camera-receipt FILE]" \
        "[--camera-ready-file FILE] [--camera-ready-timeout SECONDS]" \
        "[--camera-settle SECONDS]" >&2
}

path_is_equal_or_within() {
    local candidate="$1"
    local directory="$2"
    [[ "$candidate" == "$directory" || "$candidate" == "$directory/"* ]]
}

while (($#)); do
    case "$1" in
        --trace)
            TRACE="${2:-}"
            shift 2
            ;;
        --model)
            MODEL="${2:-}"
            shift 2
            ;;
        --matrix-root)
            MATRIX_ROOT="${2:-}"
            shift 2
            ;;
        --state-dir)
            STATE_DIR="${2:-}"
            shift 2
            ;;
        --status-file)
            STATUS_FILE="${2:-}"
            shift 2
            ;;
        --summary)
            SUMMARY_FILE="${2:-}"
            shift 2
            ;;
        --restore-receipt)
            RESTORE_RECEIPT="${2:-}"
            shift 2
            ;;
        --pre-roll)
            PRE_ROLL_SECONDS="${2:-}"
            shift 2
            ;;
        --final-hold)
            FINAL_HOLD_SECONDS="${2:-}"
            shift 2
            ;;
        --camera-mode)
            CAMERA_MODE="${2:-}"
            shift 2
            ;;
        --camera-distance-cm)
            CAMERA_DISTANCE_CM="${2:-}"
            shift 2
            ;;
        --overlay-bundle)
            OVERLAY_BUNDLE="${2:-}"
            shift 2
            ;;
        --overlay-contract)
            OVERLAY_CONTRACT="${2:-}"
            shift 2
            ;;
        --camera-receipt)
            CAMERA_RECEIPT="${2:-}"
            shift 2
            ;;
        --camera-ready-file)
            CAMERA_READY_FILE="${2:-}"
            shift 2
            ;;
        --camera-ready-timeout)
            CAMERA_READY_TIMEOUT_SECONDS="${2:-}"
            shift 2
            ;;
        --camera-settle)
            CAMERA_SETTLE_SECONDS="${2:-}"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "[ERROR] Unknown argument: $1" >&2
            usage
            exit 2
            ;;
    esac
done

if [[ -z "$TRACE" ]]; then
    echo "[ERROR] --trace is required" >&2
    usage
    exit 2
fi
for timing_value in \
    "$PRE_ROLL_SECONDS" "$FINAL_HOLD_SECONDS" \
    "$CAMERA_READY_TIMEOUT_SECONDS" "$CAMERA_SETTLE_SECONDS"; do
    if [[ ! "$timing_value" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
        echo "[ERROR] Replay timing values must be non-negative numbers:" \
            "$timing_value" >&2
        exit 2
    fi
done
case "$CAMERA_MODE" in
    robot|spectator-overlay) ;;
    *)
        echo "[ERROR] --camera-mode must be robot or spectator-overlay:" \
            "$CAMERA_MODE" >&2
        exit 2
        ;;
esac
if [[ ! "$CAMERA_DISTANCE_CM" =~ ^(0|[1-9][0-9]*)([.][0-9]+)?$ ]] \
    || ! awk -v value="$CAMERA_DISTANCE_CM" \
        'BEGIN { exit !(value >= 80.0 && value <= 500.0) }'; then
    echo "[ERROR] --camera-distance-cm must be within 80..500:" \
        "$CAMERA_DISTANCE_CM" >&2
    exit 2
fi
while [[ "$CAMERA_DISTANCE_CM" == *.* && "$CAMERA_DISTANCE_CM" == *0 ]]; do
    CAMERA_DISTANCE_CM="${CAMERA_DISTANCE_CM%0}"
done
CAMERA_DISTANCE_CM="${CAMERA_DISTANCE_CM%.}"
MATRIX_ROOT="$(realpath -- "$MATRIX_ROOT")"
TRACE="$(realpath -- "$TRACE")"
if [[ -n "$MODEL" ]]; then
    MODEL="$(realpath -- "$MODEL")"
fi
if [[ -n "$STATE_DIR" ]]; then
    STATE_DIR="$(realpath -m -- "$STATE_DIR")"
fi
if [[ -z "$OVERLAY_CONTRACT" ]]; then
    OVERLAY_CONTRACT="$MATRIX_ROOT/config/runtime/matrix-centered-camera-overlay-v3.json"
fi
if [[ -L "$OVERLAY_CONTRACT" ]]; then
    echo "[ERROR] --overlay-contract must not be a symlink:" \
        "$OVERLAY_CONTRACT" >&2
    exit 2
fi
OVERLAY_CONTRACT="$(realpath -- "$OVERLAY_CONTRACT")"
if [[ ! -f "$OVERLAY_CONTRACT" ]]; then
    echo "[ERROR] --overlay-contract must be a real file:" \
        "$OVERLAY_CONTRACT" >&2
    exit 2
fi
if [[ "$CAMERA_MODE" == "spectator-overlay" ]]; then
    if [[ -z "$OVERLAY_BUNDLE" ]]; then
        echo "[ERROR] --overlay-bundle is required for spectator-overlay" >&2
        exit 2
    fi
    if [[ -L "$OVERLAY_BUNDLE" ]]; then
        echo "[ERROR] --overlay-bundle must not be a symlink:" \
            "$OVERLAY_BUNDLE" >&2
        exit 2
    fi
    OVERLAY_BUNDLE="$(realpath -- "$OVERLAY_BUNDLE")"
    if [[ ! -d "$OVERLAY_BUNDLE" ]]; then
        echo "[ERROR] --overlay-bundle must be a real directory:" \
            "$OVERLAY_BUNDLE" >&2
        exit 2
    fi
    if path_is_equal_or_within "$OVERLAY_BUNDLE" "$MATRIX_ROOT"; then
        echo "[ERROR] --overlay-bundle must be external to the Matrix checkout:" \
            "$OVERLAY_BUNDLE" >&2
        exit 2
    fi
fi
STATUS_FILE="$(realpath -m -- "$STATUS_FILE")"
SUMMARY_FILE="$(realpath -m -- "$SUMMARY_FILE")"
if [[ -z "$CAMERA_RECEIPT" ]]; then
    CAMERA_RECEIPT="${SUMMARY_FILE%.json}.camera.json"
fi
CAMERA_RECEIPT="$(realpath -m -- "$CAMERA_RECEIPT")"
if [[ -n "$CAMERA_READY_FILE" ]]; then
    CAMERA_READY_FILE="$(realpath -m -- "$CAMERA_READY_FILE")"
fi
if [[ -z "$RESTORE_RECEIPT" ]]; then
    RESTORE_RECEIPT="${SUMMARY_FILE%.json}.restore.json"
fi
RESTORE_RECEIPT="$(realpath -m -- "$RESTORE_RECEIPT")"
PROTECTED_ACTIVE="$(realpath -m -- \
    "$MATRIX_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Saved/Paks/MatrixCenteredCameraActive")"
PROTECTED_UE_LOG="$(realpath -m -- \
    "$MATRIX_ROOT/src/UeSim/Linux/zsibot_mujoco_ue.log")"
MUTABLE_REPLAY_PATHS=(
    "$STATUS_FILE" "$SUMMARY_FILE" "$RESTORE_RECEIPT" "$CAMERA_RECEIPT"
)
if [[ -n "$CAMERA_READY_FILE" ]]; then
    MUTABLE_REPLAY_PATHS+=("$CAMERA_READY_FILE")
fi
for mutable_path in "${MUTABLE_REPLAY_PATHS[@]}"; do
    if [[ "$mutable_path" == "$OVERLAY_CONTRACT" \
        || "$mutable_path" == "$PROTECTED_UE_LOG" ]] \
        || path_is_equal_or_within "$mutable_path" "$PROTECTED_ACTIVE" \
        || { [[ "$CAMERA_MODE" == "spectator-overlay" ]] \
            && path_is_equal_or_within "$mutable_path" "$OVERLAY_BUNDLE"; }; then
        echo "[ERROR] Replay output aliases protected camera input:" \
            "$mutable_path" >&2
        exit 2
    fi
done
if [[ -n "$STATE_DIR" ]]; then
    if path_is_equal_or_within "$STATE_DIR" "$PROTECTED_ACTIVE" \
        || path_is_equal_or_within "$PROTECTED_ACTIVE" "$STATE_DIR" \
        || path_is_equal_or_within "$OVERLAY_CONTRACT" "$STATE_DIR" \
        || { [[ "$CAMERA_MODE" == "spectator-overlay" ]] \
            && { path_is_equal_or_within "$STATE_DIR" "$OVERLAY_BUNDLE" \
                || path_is_equal_or_within "$OVERLAY_BUNDLE" "$STATE_DIR"; }; }; then
        echo "[ERROR] Replay state directory overlaps protected camera input:" \
            "$STATE_DIR" >&2
        exit 2
    fi
fi
for required in \
    "$MATRIX_ROOT/scripts/run_sim.sh" \
    "$MATRIX_ROOT/scripts/replay_matrix_physics_trace.py" \
    "$MATRIX_ROOT/scripts/stage_matrix_trace_model.py" \
    "$MATRIX_ROOT/scripts/matrix_ue_overlay.py" \
    "$MATRIX_ROOT/scripts/matrix_scene6_camera_receipt.py"; do
    if [[ ! -f "$required" ]]; then
        echo "[ERROR] Required Matrix replay component is missing: $required" >&2
        exit 2
    fi
done

PYTHON="${MATRIX_EXTERNAL_REPLAY_PYTHON:-${MATRIX_SONIC_PYTHON:-$(command -v python3)}}"
INSPECT_COMMAND=(
    "$PYTHON" "$MATRIX_ROOT/scripts/replay_matrix_physics_trace.py"
    --trace "$TRACE" --inspect
)
if [[ -n "$MODEL" ]]; then
    INSPECT_COMMAND+=(--model "$MODEL")
fi
"${INSPECT_COMMAND[@]}" >/dev/null

mkdir -p "$MATRIX_ROOT/outputs/runtime" "$MATRIX_ROOT/outputs/logs"
if ! command -v flock >/dev/null 2>&1; then
    echo "[ERROR] flock is required by the Matrix trace-replay launcher" >&2
    exit 2
fi
MATRIX_SONIC_HOST_LOCK="${MATRIX_SONIC_HOST_LOCK:-/tmp/matrix-sonic-${UID}.lock}"
if [[ "${MATRIX_SONIC_HOST_LOCK_FD:-}" == "9" ]]; then
    inherited_target="$(readlink -f "/proc/$$/fd/9" 2>/dev/null || true)"
    expected_target="$(realpath -m "$MATRIX_SONIC_HOST_LOCK")"
    if [[ "$inherited_target" != "$expected_target" ]] || ! flock -n 9; then
        echo "[ERROR] Matrix trace replay did not inherit the verified host lock" >&2
        exit 2
    fi
else
    exec 9>"$MATRIX_SONIC_HOST_LOCK"
    if ! flock -n 9; then
        echo "[ERROR] Another Matrix launcher owns this host:" \
            "$MATRIX_SONIC_HOST_LOCK" >&2
        exit 2
    fi
fi
export MATRIX_SONIC_HOST_LOCK_FD=9

# The host lock proves no live Matrix launcher owns the active PAK directory.
# Purge only a fully verified stale overlay before every replay mode: otherwise
# a prior SIGKILL could make a nominal robot-camera run mount an undisclosed
# Spectator overlay, or block the next atomic install.
"$PYTHON" -I "$MATRIX_ROOT/scripts/matrix_ue_overlay.py" purge-stale \
    --contract "$OVERLAY_CONTRACT" \
    --project-root "$MATRIX_ROOT"
if [[ "$CAMERA_MODE" == "spectator-overlay" ]]; then
    "$PYTHON" -I "$MATRIX_ROOT/scripts/matrix_ue_overlay.py" verify-bundle \
        --contract "$OVERLAY_CONTRACT" \
        --bundle "$OVERLAY_BUNDLE"
fi

# Recover a journal left by a SIGKILL/power-loss boundary before creating a new
# one.  The shared host lock proves no live Matrix launcher can still own these
# active files; restore itself remains hash-gated and fails closed on drift.
shopt -s nullglob
for stale_state_path in \
    "$MATRIX_ROOT"/outputs/runtime/matrix-scene6-stage.*/state.json; do
    stale_state_dir="$(dirname -- "$stale_state_path")"
    stale_state_name="$(basename -- "$stale_state_dir")"
    recovered_receipt="$MATRIX_ROOT/outputs/runtime/recovered-${stale_state_name}.json"
    echo "[INFO] Recovering prior Matrix scene6 stage journal: $stale_state_dir"
    "$PYTHON" "$MATRIX_ROOT/scripts/stage_matrix_trace_model.py" restore \
        --matrix-root "$MATRIX_ROOT" \
        --state-dir "$stale_state_dir" \
        --receipt "$recovered_receipt"
    rm -rf -- "$stale_state_dir"
done
shopt -u nullglob

for stale in \
    "$STATUS_FILE" "$SUMMARY_FILE" "$RESTORE_RECEIPT" "$CAMERA_RECEIPT"; do
    if [[ "$stale" == "$TRACE" \
        || ( -n "$MODEL" && "$stale" == "$MODEL" ) ]]; then
        echo "[ERROR] Replay output path aliases a source artifact: $stale" >&2
        exit 2
    fi
    if [[ -L "$stale" || -d "$stale" ]]; then
        echo "[ERROR] Replay output must not be a symlink or directory: $stale" >&2
        exit 2
    fi
    rm -f -- "$stale"
done
if [[ "$STATUS_FILE" == "$SUMMARY_FILE" \
    || "$STATUS_FILE" == "$RESTORE_RECEIPT" \
    || "$SUMMARY_FILE" == "$RESTORE_RECEIPT" \
    || "$CAMERA_RECEIPT" == "$STATUS_FILE" \
    || "$CAMERA_RECEIPT" == "$SUMMARY_FILE" \
    || "$CAMERA_RECEIPT" == "$RESTORE_RECEIPT" ]]; then
    echo "[ERROR] Replay status, summary, camera, and restore receipts must be distinct" >&2
    exit 2
fi
if [[ -n "$CAMERA_READY_FILE" ]]; then
    if [[ "$CAMERA_READY_FILE" == "$TRACE" \
        || ( -n "$MODEL" && "$CAMERA_READY_FILE" == "$MODEL" ) \
        || "$CAMERA_READY_FILE" == "$STATUS_FILE" \
        || "$CAMERA_READY_FILE" == "$SUMMARY_FILE" \
        || "$CAMERA_READY_FILE" == "$RESTORE_RECEIPT" \
        || "$CAMERA_READY_FILE" == "$CAMERA_RECEIPT" ]]; then
        echo "[ERROR] Camera-ready path must be distinct from replay artifacts" >&2
        exit 2
    fi
    if [[ -L "$CAMERA_READY_FILE" || -d "$CAMERA_READY_FILE" ]]; then
        echo "[ERROR] Camera-ready path must not be a symlink or directory:" \
            "$CAMERA_READY_FILE" >&2
        exit 2
    fi
    rm -f -- "$CAMERA_READY_FILE"
fi
mkdir -p -- "$(dirname -- "$CAMERA_RECEIPT")"

REMOVE_STATE_DIR=0
if [[ -z "$STATE_DIR" ]]; then
    STATE_DIR="$(mktemp -d "$MATRIX_ROOT/outputs/runtime/matrix-scene6-stage.XXXXXX")"
    REMOVE_STATE_DIR=1
fi

restore_model() {
    local incoming_exit=$?
    local restore_exit=0
    trap - EXIT INT TERM HUP
    if [[ -f "$STATE_DIR/state.json" && ! -L "$STATE_DIR/state.json" ]]; then
        if ! "$PYTHON" "$MATRIX_ROOT/scripts/stage_matrix_trace_model.py" restore \
            --matrix-root "$MATRIX_ROOT" \
            --state-dir "$STATE_DIR" \
            --receipt "$RESTORE_RECEIPT"; then
            restore_exit=1
            echo "[ERROR] Failed to restore Matrix custom/current.xml" >&2
        fi
    fi
    if [[ "$REMOVE_STATE_DIR" == "1" && "$restore_exit" == "0" ]]; then
        rm -rf -- "$STATE_DIR"
    fi
    if [[ "$incoming_exit" == "0" && "$restore_exit" != "0" ]]; then
        incoming_exit=2
    fi
    exit "$incoming_exit"
}
trap restore_model EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

STAGE_COMMAND=(
    "$PYTHON" "$MATRIX_ROOT/scripts/stage_matrix_trace_model.py" stage
    --matrix-root "$MATRIX_ROOT"
    --trace "$TRACE"
    --state-dir "$STATE_DIR"
)
if [[ -n "$MODEL" ]]; then
    STAGE_COMMAND+=(--model "$MODEL")
fi
"${STAGE_COMMAND[@]}"

export MATRIX_EXTERNAL_REPLAY=1
export MATRIX_EXTERNAL_REPLAY_PYTHON="$PYTHON"
export MATRIX_EXTERNAL_REPLAY_TRACE="$TRACE"
export MATRIX_EXTERNAL_REPLAY_STATUS_FILE="$STATUS_FILE"
export MATRIX_EXTERNAL_REPLAY_SUMMARY="$SUMMARY_FILE"
export MATRIX_EXTERNAL_REPLAY_PRE_ROLL_SECONDS="$PRE_ROLL_SECONDS"
export MATRIX_EXTERNAL_REPLAY_FINAL_HOLD_SECONDS="$FINAL_HOLD_SECONDS"
export MATRIX_EXTERNAL_REPLAY_CAMERA_RECEIPT="$CAMERA_RECEIPT"
export MATRIX_EXTERNAL_REPLAY_CAMERA_READY_FILE="$CAMERA_READY_FILE"
export MATRIX_EXTERNAL_REPLAY_CAMERA_READY_TIMEOUT_SECONDS="$CAMERA_READY_TIMEOUT_SECONDS"
export MATRIX_EXTERNAL_REPLAY_CAMERA_SETTLE_SECONDS="$CAMERA_SETTLE_SECONDS"
export MATRIX_DISABLE_MC=1
export MATRIX_SONIC=0
export MATRIX_UE_MAX_FPS=25
export SIM_LAUNCHER_SKIP_CUSTOM_URDF_WRAPPER=1
export MATRIX_GAME_CAMERA_DISTANCE_CM="$CAMERA_DISTANCE_CM"
case "$CAMERA_MODE" in
    robot)
        export MATRIX_EXTERNAL_REPLAY_CENTERED_CAMERA=0
        if [[ -z "${MATRIX_UE_EXTRA_EXEC_CMDS:-}" ]]; then
            MATRIX_UE_EXTRA_EXEC_CMDS="set Engine.SpringArmComponent bEnableCameraLag False"
            MATRIX_UE_EXTRA_EXEC_CMDS+=",set Engine.SpringArmComponent bEnableCameraRotationLag False"
            MATRIX_UE_EXTRA_EXEC_CMDS+=",set Engine.SpringArmComponent bDoCollisionTest True"
            MATRIX_UE_EXTRA_EXEC_CMDS+=",set Engine.SpringArmComponent TargetArmLength ${CAMERA_DISTANCE_CM}"
            MATRIX_UE_EXTRA_EXEC_CMDS+=",viewclass MujocoSim_Custom_C"
            export MATRIX_UE_EXTRA_EXEC_CMDS
        fi
        CAMERA_VIEW_CLASS="MujocoSim_Custom_C"
        ;;
    spectator-overlay)
        export MATRIX_EXTERNAL_REPLAY_CENTERED_CAMERA=1
        export MATRIX_CENTERED_CAMERA_OVERLAY_BUNDLE="$OVERLAY_BUNDLE"
        export MATRIX_CENTERED_CAMERA_OVERLAY_CONTRACT="$OVERLAY_CONTRACT"
        CAMERA_VIEW_CLASS="Spectator_C"
        ;;
esac
if [[ -n "$MODEL" ]]; then
    export MATRIX_EXTERNAL_REPLAY_MODEL="$MODEL"
else
    unset MATRIX_EXTERNAL_REPLAY_MODEL || true
fi

echo "[INFO] physics_execution=offline_mujoco_persistent_world"
echo "[INFO] render_mode=matrix_ue_trace_replay"
echo "[INFO] manipulation=contact-gated constrained grasp + anchored stance"
echo "[INFO] replay_camera_mode=$CAMERA_MODE requested_viewclass=$CAMERA_VIEW_CLASS" \
    "spring_arm_cm=$CAMERA_DISTANCE_CM"
echo "[INFO] replay_camera_extra_exec_cmds=${MATRIX_UE_EXTRA_EXEC_CMDS:-none}"

cd "$MATRIX_ROOT"
bash scripts/run_sim.sh custom 6 0 0 1 "" twinbot_scene6_trace_replay
