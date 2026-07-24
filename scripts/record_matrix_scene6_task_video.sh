#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

TRACE=""
MODEL=""
MATRIX_ROOT="$PROJECT_ROOT"
OUTPUT="$PROJECT_ROOT/outputs/matrix-scene6-twinbot-task.mp4"
METADATA=""
STATUS_FILE=""
SUMMARY_FILE=""
RESTORE_RECEIPT=""
POSTFLIGHT_RECEIPT=""
DISPLAY_VALUE="${DISPLAY:-:0}"
XAUTHORITY_VALUE="${XAUTHORITY:-}"
ENCODER="auto"
PRE_ROLL_SECONDS="2"
CAPTURE_TAIL_SECONDS="1"
FINAL_HOLD_SECONDS=""
WINDOW_TIMEOUT_SECONDS="120"
READY_TIMEOUT_SECONDS="180"
LAUNCHER_EXIT_TIMEOUT_SECONDS=""

usage() {
    echo "Usage: $0 --trace FILE [--model FILE] [--output FILE]" \
        "[--matrix-root DIR] [--display DISPLAY] [--xauthority FILE]" >&2
}

while (($#)); do
    case "$1" in
        --trace) TRACE="${2:-}"; shift 2 ;;
        --model) MODEL="${2:-}"; shift 2 ;;
        --matrix-root) MATRIX_ROOT="${2:-}"; shift 2 ;;
        --output) OUTPUT="${2:-}"; shift 2 ;;
        --metadata) METADATA="${2:-}"; shift 2 ;;
        --status-file) STATUS_FILE="${2:-}"; shift 2 ;;
        --summary) SUMMARY_FILE="${2:-}"; shift 2 ;;
        --restore-receipt) RESTORE_RECEIPT="${2:-}"; shift 2 ;;
        --postflight-receipt) POSTFLIGHT_RECEIPT="${2:-}"; shift 2 ;;
        --display) DISPLAY_VALUE="${2:-}"; shift 2 ;;
        --xauthority) XAUTHORITY_VALUE="${2:-}"; shift 2 ;;
        --encoder) ENCODER="${2:-}"; shift 2 ;;
        --pre-roll) PRE_ROLL_SECONDS="${2:-}"; shift 2 ;;
        --capture-tail) CAPTURE_TAIL_SECONDS="${2:-}"; shift 2 ;;
        --final-hold) FINAL_HOLD_SECONDS="${2:-}"; shift 2 ;;
        --window-timeout) WINDOW_TIMEOUT_SECONDS="${2:-}"; shift 2 ;;
        --ready-timeout) READY_TIMEOUT_SECONDS="${2:-}"; shift 2 ;;
        --launcher-exit-timeout) LAUNCHER_EXIT_TIMEOUT_SECONDS="${2:-}"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
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
for numeric_value in \
    "$PRE_ROLL_SECONDS" "$CAPTURE_TAIL_SECONDS" "$WINDOW_TIMEOUT_SECONDS" \
    "$READY_TIMEOUT_SECONDS"; do
    if [[ ! "$numeric_value" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
        echo "[ERROR] Replay/capture timing values must be non-negative numbers:" \
            "$numeric_value" >&2
        exit 2
    fi
done
if [[ -n "$FINAL_HOLD_SECONDS" \
    && ! "$FINAL_HOLD_SECONDS" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    echo "[ERROR] --final-hold must be a non-negative number:" \
        "$FINAL_HOLD_SECONDS" >&2
    exit 2
fi
if [[ -n "$LAUNCHER_EXIT_TIMEOUT_SECONDS" \
    && ! "$LAUNCHER_EXIT_TIMEOUT_SECONDS" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    echo "[ERROR] --launcher-exit-timeout must be a non-negative number:" \
        "$LAUNCHER_EXIT_TIMEOUT_SECONDS" >&2
    exit 2
fi
MATRIX_ROOT="$(realpath -- "$MATRIX_ROOT")"
TRACE="$(realpath -- "$TRACE")"
OUTPUT="$(realpath -m -- "$OUTPUT")"
if [[ "${OUTPUT,,}" != *.mp4 ]]; then
    echo "[ERROR] --output must use the .mp4 extension" >&2
    exit 2
fi
if [[ -n "$MODEL" ]]; then
    MODEL="$(realpath -- "$MODEL")"
fi
for required in \
    "$MATRIX_ROOT/scripts/replay_matrix_physics_trace.py" \
    "$MATRIX_ROOT/scripts/run_matrix_scene6_trace_replay.sh" \
    "$MATRIX_ROOT/scripts/record_matrix_sonic_video.sh" \
    "$MATRIX_ROOT/scripts/verify_matrix_scene6_task_video.py"; do
    if [[ ! -f "$required" ]]; then
        echo "[ERROR] Required Matrix scene6 recording component is missing:" \
            "$required" >&2
        exit 2
    fi
done
if [[ -z "$METADATA" ]]; then
    METADATA="${OUTPUT%.mp4}.json"
fi
if [[ -z "$STATUS_FILE" ]]; then
    STATUS_FILE="${OUTPUT%.mp4}.replay-status.json"
fi
if [[ -z "$SUMMARY_FILE" ]]; then
    SUMMARY_FILE="${OUTPUT%.mp4}.replay-summary.json"
fi
if [[ -z "$RESTORE_RECEIPT" ]]; then
    RESTORE_RECEIPT="${OUTPUT%.mp4}.restore.json"
fi
if [[ -z "$POSTFLIGHT_RECEIPT" ]]; then
    POSTFLIGHT_RECEIPT="${OUTPUT%.mp4}.verified.json"
fi
METADATA="$(realpath -m -- "$METADATA")"
STATUS_FILE="$(realpath -m -- "$STATUS_FILE")"
SUMMARY_FILE="$(realpath -m -- "$SUMMARY_FILE")"
RESTORE_RECEIPT="$(realpath -m -- "$RESTORE_RECEIPT")"
POSTFLIGHT_RECEIPT="$(realpath -m -- "$POSTFLIGHT_RECEIPT")"
ARTIFACT_PATHS=(
    "$OUTPUT" "$METADATA" "$STATUS_FILE" "$SUMMARY_FILE"
    "$RESTORE_RECEIPT" "$POSTFLIGHT_RECEIPT"
)
for ((left = 0; left < ${#ARTIFACT_PATHS[@]}; left++)); do
    artifact="${ARTIFACT_PATHS[$left]}"
    if [[ "$artifact" == "$TRACE" \
        || ( -n "$MODEL" && "$artifact" == "$MODEL" ) ]]; then
        echo "[ERROR] Recording output aliases a source artifact: $artifact" >&2
        exit 2
    fi
    if [[ -L "$artifact" || -d "$artifact" ]]; then
        echo "[ERROR] Recording output must not be a symlink or directory:" \
            "$artifact" >&2
        exit 2
    fi
    for ((right = left + 1; right < ${#ARTIFACT_PATHS[@]}; right++)); do
        if [[ "$artifact" == "${ARTIFACT_PATHS[$right]}" ]]; then
            echo "[ERROR] Recording output paths must be distinct: $artifact" >&2
            exit 2
        fi
    done
done

PYTHON="${MATRIX_VIDEO_PYTHON:-${MATRIX_EXTERNAL_REPLAY_PYTHON:-${MATRIX_SONIC_PYTHON:-$(command -v python3)}}}"
INSPECT_COMMAND=(
    "$PYTHON" "$MATRIX_ROOT/scripts/replay_matrix_physics_trace.py"
    --trace "$TRACE" --inspect-frame-count
)
if [[ -n "$MODEL" ]]; then
    INSPECT_COMMAND+=(--model "$MODEL")
fi
FRAME_COUNT="$("${INSPECT_COMMAND[@]}")"
if [[ ! "$FRAME_COUNT" =~ ^[1-9][0-9]*$ ]]; then
    echo "[ERROR] Trace inspection returned invalid frame count: $FRAME_COUNT" >&2
    exit 2
fi
CAPTURE_DURATION="$(
    awk -v frames="$FRAME_COUNT" \
        -v pre="$PRE_ROLL_SECONDS" \
        -v tail="$CAPTURE_TAIL_SECONDS" \
        'BEGIN {
            duration = pre + (frames / 25.0) + tail
            if (duration <= 0) exit 2
            printf "%.3f", duration
        }'
)"
if [[ -z "$FINAL_HOLD_SECONDS" ]]; then
    FINAL_HOLD_SECONDS="$(
        awk -v duration="$CAPTURE_DURATION" \
            'BEGIN { printf "%.3f", duration + 5.0 }'
    )"
fi
if [[ -z "$LAUNCHER_EXIT_TIMEOUT_SECONDS" ]]; then
    LAUNCHER_EXIT_TIMEOUT_SECONDS="$(
        awk -v hold="$FINAL_HOLD_SECONDS" \
            'BEGIN { printf "%.3f", hold + 60.0 }'
    )"
fi
if ! awk -v timeout="$LAUNCHER_EXIT_TIMEOUT_SECONDS" \
    'BEGIN { exit !(timeout > 0.0) }'; then
    echo "[ERROR] --launcher-exit-timeout must be positive" >&2
    exit 2
fi
if ! awk -v hold="$FINAL_HOLD_SECONDS" -v capture="$CAPTURE_DURATION" \
    'BEGIN { exit !(hold > capture) }'; then
    echo "[ERROR] --final-hold must be longer than the derived capture duration" \
        "($CAPTURE_DURATION s)" >&2
    exit 2
fi

RECORDER_COMMAND=(
    bash "$MATRIX_ROOT/scripts/record_matrix_sonic_video.sh"
    --output "$OUTPUT"
    --metadata "$METADATA"
    --duration "$CAPTURE_DURATION"
    --fps 25
    --encoder "$ENCODER"
    --display "$DISPLAY_VALUE"
    --window-timeout "$WINDOW_TIMEOUT_SECONDS"
    --ready status
    --ready-timeout "$READY_TIMEOUT_SECONDS"
    --wait-launcher-exit-timeout "$LAUNCHER_EXIT_TIMEOUT_SECONDS"
    --status-file "$STATUS_FILE"
    --notes "TwinBot scene6 task; physics_execution=offline_mujoco_persistent_world; render_mode=matrix_ue_trace_replay; grasp=contact-gated constrained + anchored stance"
)
if [[ -n "$XAUTHORITY_VALUE" ]]; then
    RECORDER_COMMAND+=(--xauthority "$XAUTHORITY_VALUE")
fi
RECORDER_COMMAND+=(
    --
    bash "$MATRIX_ROOT/scripts/run_matrix_scene6_trace_replay.sh"
    --matrix-root "$MATRIX_ROOT"
    --trace "$TRACE"
    --status-file "$STATUS_FILE"
    --summary "$SUMMARY_FILE"
    --restore-receipt "$RESTORE_RECEIPT"
    --pre-roll "$PRE_ROLL_SECONDS"
    --final-hold "$FINAL_HOLD_SECONDS"
)
if [[ -n "$MODEL" ]]; then
    RECORDER_COMMAND+=(--model "$MODEL")
fi

echo "[INFO] source frames=$FRAME_COUNT fps=25 capture=${CAPTURE_DURATION}s" \
    "final_hold=${FINAL_HOLD_SECONDS}s"
echo "[INFO] This is Matrix UE trace replay, not live SONIC manipulation."
if [[ -L "$POSTFLIGHT_RECEIPT" || -d "$POSTFLIGHT_RECEIPT" ]]; then
    echo "[ERROR] Postflight receipt must not be a symlink or directory:" \
        "$POSTFLIGHT_RECEIPT" >&2
    exit 2
fi
rm -f -- "$POSTFLIGHT_RECEIPT"

"${RECORDER_COMMAND[@]}"

"$PYTHON" "$MATRIX_ROOT/scripts/verify_matrix_scene6_task_video.py" \
    --output "$OUTPUT" \
    --metadata "$METADATA" \
    --replay-summary "$SUMMARY_FILE" \
    --restore-receipt "$RESTORE_RECEIPT" \
    --matrix-root "$MATRIX_ROOT" \
    --receipt "$POSTFLIGHT_RECEIPT"
