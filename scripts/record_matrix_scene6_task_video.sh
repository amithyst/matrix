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
CAMERA_MODE="${MATRIX_SCENE6_CAMERA_MODE:-robot}"
CAMERA_DISTANCE_CM="${MATRIX_SCENE6_CAMERA_DISTANCE_CM:-180}"
OVERLAY_BUNDLE="${MATRIX_CENTERED_CAMERA_OVERLAY_BUNDLE:-}"
OVERLAY_CONTRACT="${MATRIX_CENTERED_CAMERA_OVERLAY_CONTRACT:-}"
CAMERA_RECEIPT=""
CAMERA_READY_FILE=""
CAMERA_READY_TIMEOUT_SECONDS="120"
CAMERA_SETTLE_SECONDS="0.5"

usage() {
    echo "Usage: $0 --trace FILE [--model FILE] [--output FILE]" \
        "[--matrix-root DIR] [--display DISPLAY] [--xauthority FILE]" \
        "[--camera-mode robot|spectator-overlay]" \
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
        --camera-mode) CAMERA_MODE="${2:-}"; shift 2 ;;
        --camera-distance-cm) CAMERA_DISTANCE_CM="${2:-}"; shift 2 ;;
        --overlay-bundle) OVERLAY_BUNDLE="${2:-}"; shift 2 ;;
        --overlay-contract) OVERLAY_CONTRACT="${2:-}"; shift 2 ;;
        --camera-receipt) CAMERA_RECEIPT="${2:-}"; shift 2 ;;
        --camera-ready-file) CAMERA_READY_FILE="${2:-}"; shift 2 ;;
        --camera-ready-timeout) CAMERA_READY_TIMEOUT_SECONDS="${2:-}"; shift 2 ;;
        --camera-settle) CAMERA_SETTLE_SECONDS="${2:-}"; shift 2 ;;
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
    "$READY_TIMEOUT_SECONDS" "$CAMERA_READY_TIMEOUT_SECONDS" \
    "$CAMERA_SETTLE_SECONDS"; do
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
OUTPUT="$(realpath -m -- "$OUTPUT")"
if [[ "${OUTPUT,,}" != *.mp4 ]]; then
    echo "[ERROR] --output must use the .mp4 extension" >&2
    exit 2
fi
if [[ -n "$MODEL" ]]; then
    MODEL="$(realpath -- "$MODEL")"
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
    if [[ -z "$CAMERA_READY_FILE" ]]; then
        echo "[ERROR] spectator-overlay recording requires --camera-ready-file" >&2
        exit 2
    fi
fi
for required in \
    "$MATRIX_ROOT/scripts/replay_matrix_physics_trace.py" \
    "$MATRIX_ROOT/scripts/run_matrix_scene6_trace_replay.sh" \
    "$MATRIX_ROOT/scripts/record_matrix_sonic_video.sh" \
    "$MATRIX_ROOT/scripts/matrix_scene6_camera_receipt.py" \
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
if [[ -z "$CAMERA_RECEIPT" ]]; then
    CAMERA_RECEIPT="${OUTPUT%.mp4}.camera.json"
fi
METADATA="$(realpath -m -- "$METADATA")"
STATUS_FILE="$(realpath -m -- "$STATUS_FILE")"
SUMMARY_FILE="$(realpath -m -- "$SUMMARY_FILE")"
RESTORE_RECEIPT="$(realpath -m -- "$RESTORE_RECEIPT")"
POSTFLIGHT_RECEIPT="$(realpath -m -- "$POSTFLIGHT_RECEIPT")"
CAMERA_RECEIPT="$(realpath -m -- "$CAMERA_RECEIPT")"
if [[ -n "$CAMERA_READY_FILE" ]]; then
    CAMERA_READY_FILE="$(realpath -m -- "$CAMERA_READY_FILE")"
fi
ARTIFACT_PATHS=(
    "$OUTPUT" "$METADATA" "$STATUS_FILE" "$SUMMARY_FILE"
    "$RESTORE_RECEIPT" "$POSTFLIGHT_RECEIPT" "$CAMERA_RECEIPT"
)
if [[ -n "$CAMERA_READY_FILE" ]]; then
    ARTIFACT_PATHS+=("$CAMERA_READY_FILE")
fi
OUTPUT_DIRECTORY="$(dirname -- "$OUTPUT")"
OUTPUT_BASENAME="$(basename -- "$OUTPUT")"
OUTPUT_STEM="${OUTPUT_BASENAME%.*}"
RECORDER_INTERNAL_PATHS=(
    "$OUTPUT_DIRECTORY/.${OUTPUT_STEM}.partial.mp4"
    "$OUTPUT_DIRECTORY/${OUTPUT_STEM}.rejected.mp4"
    "$OUTPUT_DIRECTORY/${OUTPUT_STEM}.preview.jpg"
    "$OUTPUT_DIRECTORY/${OUTPUT_STEM}.rejected.preview.jpg"
    "$OUTPUT_DIRECTORY/${OUTPUT_STEM}.launch.log"
    "$OUTPUT_DIRECTORY/${OUTPUT_STEM}.ffmpeg.log"
)
ARTIFACT_PATHS+=("${RECORDER_INTERNAL_PATHS[@]}")
PROTECTED_ACTIVE="$(realpath -m -- \
    "$MATRIX_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Saved/Paks/MatrixCenteredCameraActive")"
PROTECTED_UE_LOG="$(realpath -m -- \
    "$MATRIX_ROOT/src/UeSim/Linux/zsibot_mujoco_ue.log")"
for ((left = 0; left < ${#ARTIFACT_PATHS[@]}; left++)); do
    artifact="${ARTIFACT_PATHS[$left]}"
    if [[ "$artifact" == "$TRACE" \
        || ( -n "$MODEL" && "$artifact" == "$MODEL" ) ]]; then
        echo "[ERROR] Recording output aliases a source artifact: $artifact" >&2
        exit 2
    fi
    if [[ "$artifact" == "$OVERLAY_CONTRACT" \
        || "$artifact" == "$PROTECTED_UE_LOG" ]] \
        || path_is_equal_or_within "$artifact" "$PROTECTED_ACTIVE" \
        || { [[ "$CAMERA_MODE" == "spectator-overlay" ]] \
            && path_is_equal_or_within "$artifact" "$OVERLAY_BUNDLE"; }; then
        echo "[ERROR] Recording output aliases protected camera input:" \
            "$artifact" >&2
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
    --notes "TwinBot scene6 task; physics_execution=offline_mujoco_persistent_world; render_mode=matrix_ue_trace_replay; grasp=contact-gated constrained + anchored stance; camera_mode=$CAMERA_MODE; camera_distance_cm=$CAMERA_DISTANCE_CM"
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
    --camera-mode "$CAMERA_MODE"
    --camera-distance-cm "$CAMERA_DISTANCE_CM"
    --camera-receipt "$CAMERA_RECEIPT"
    --camera-ready-timeout "$CAMERA_READY_TIMEOUT_SECONDS"
    --camera-settle "$CAMERA_SETTLE_SECONDS"
)
if [[ -n "$CAMERA_READY_FILE" ]]; then
    RECORDER_COMMAND+=(--camera-ready-file "$CAMERA_READY_FILE")
fi
if [[ "$CAMERA_MODE" == "spectator-overlay" ]]; then
    RECORDER_COMMAND+=(
        --overlay-bundle "$OVERLAY_BUNDLE"
        --overlay-contract "$OVERLAY_CONTRACT"
    )
fi
if [[ -n "$MODEL" ]]; then
    RECORDER_COMMAND+=(--model "$MODEL")
fi

echo "[INFO] source frames=$FRAME_COUNT fps=25 capture=${CAPTURE_DURATION}s" \
    "final_hold=${FINAL_HOLD_SECONDS}s"
echo "[INFO] camera_mode=$CAMERA_MODE camera_distance_cm=$CAMERA_DISTANCE_CM" \
    "overlay_bundle=${OVERLAY_BUNDLE:-none}"
echo "[INFO] This is Matrix UE trace replay, not live SONIC manipulation."
if [[ -L "$POSTFLIGHT_RECEIPT" || -d "$POSTFLIGHT_RECEIPT" ]]; then
    echo "[ERROR] Postflight receipt must not be a symlink or directory:" \
        "$POSTFLIGHT_RECEIPT" >&2
    exit 2
fi
rm -f -- "$POSTFLIGHT_RECEIPT"

"${RECORDER_COMMAND[@]}"

"$PYTHON" -I - \
    "$METADATA" "$CAMERA_RECEIPT" <<'PY'
import json
import os
from pathlib import Path
import sys

metadata_path, camera_receipt_path = sys.argv[1:]
path = Path(metadata_path)
payload = json.loads(path.read_text(encoding="utf-8"))
if not isinstance(payload, dict):
    raise SystemExit("recording metadata root must be an object")
camera_receipt = json.loads(Path(camera_receipt_path).read_text(encoding="utf-8"))
if not isinstance(camera_receipt, dict):
    raise SystemExit("camera receipt root must be an object")
payload["matrix_scene6_extension_schema"] = "matrix.scene6_video_metadata.v2"
payload["matrix_scene6_camera"] = camera_receipt
temporary = path.with_name(f".{path.name}.tmp.{os.getpid()}")
temporary.write_text(
    json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
    encoding="utf-8",
)
os.replace(temporary, path)
PY

"$PYTHON" "$MATRIX_ROOT/scripts/verify_matrix_scene6_task_video.py" \
    --output "$OUTPUT" \
    --metadata "$METADATA" \
    --replay-summary "$SUMMARY_FILE" \
    --restore-receipt "$RESTORE_RECEIPT" \
    --camera-receipt "$CAMERA_RECEIPT" \
    --matrix-root "$MATRIX_ROOT" \
    --receipt "$POSTFLIGHT_RECEIPT"
