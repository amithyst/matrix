#!/usr/bin/env bash
set -euo pipefail

MATRIX_UE_G1_MATERIAL_PALETTE_CONTRACT="${MATRIX_G1_MATERIAL_PALETTE:-}"
MATRIX_UE_G1_SCOPE_ALPHA_CONTRACT="${MATRIX_G1_MATERIAL_SCOPE_ALPHA:-}"
unset MATRIX_G1_MATERIAL_PALETTE MATRIX_G1_MATERIAL_SCOPE_ALPHA

#######################################
# 基础
#######################################
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_ROOT"

ROBOT_ARG="${1:-xgb}"
SCENE_ID="${2:-1}"
OFFSCREEN="${3:-0}"
PIXELSTREAM="${4:-0}"
MUJOCORUNNING="${5:-0}"
CUSTOM_URDF="${6:-}"
CUSTOM_NAME="${7:-}"
MATRIX_DISABLE_MC="${MATRIX_DISABLE_MC:-0}"
MATRIX_SONIC="${MATRIX_SONIC:-0}"
MATRIX_GAME_CENTERED_CAMERA="${MATRIX_GAME_CENTERED_CAMERA:-1}"
MATRIX_GAME_CAMERA_VIEW_CLASS="${MATRIX_GAME_CAMERA_VIEW_CLASS:-}"
MATRIX_CENTERED_CAMERA_OVERLAY_CONTRACT="${MATRIX_CENTERED_CAMERA_OVERLAY_CONTRACT:-$PROJECT_ROOT/config/runtime/matrix-centered-camera-overlay-v3.json}"
MATRIX_CENTERED_CAMERA_OVERLAY_BUNDLE="${MATRIX_CENTERED_CAMERA_OVERLAY_BUNDLE:-}"
MATRIX_UE_CAMERA_LAYOUT="${MATRIX_UE_CAMERA_LAYOUT:-$PROJECT_ROOT/config/runtime/matrix-ue-camera-layout-v1.json}"
CENTERED_CAMERA_OVERLAY_STEM="pakchunk99-MatrixCentered-Linux_P"
MATRIX_GAME_CAMERA_DISTANCE_EXPLICIT=0
if [[ -n "${MATRIX_GAME_CAMERA_DISTANCE_CM+x}" ]]; then
    MATRIX_GAME_CAMERA_DISTANCE_EXPLICIT=1
fi
MATRIX_GAME_CAMERA_DISTANCE_CM="${MATRIX_GAME_CAMERA_DISTANCE_CM:-150}"
MATRIX_SETTINGS_PROFILE="${MATRIX_HOST_PROFILE:-${MATRIX_PROFILE:-local}}"
if [[ ! "$MATRIX_SETTINGS_PROFILE" =~ ^[A-Za-z0-9_.-]{1,64}$ ]]; then
    echo "[ERROR] Matrix settings profile is invalid: $MATRIX_SETTINGS_PROFILE" >&2
    exit 1
fi
MATRIX_SETTINGS_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/matrix/hosts/$MATRIX_SETTINGS_PROFILE"
MATRIX_MOUSE_SETTINGS_FILE="${MATRIX_MOUSE_SETTINGS_FILE:-$MATRIX_SETTINGS_DIR/mouse-control.json}"
MATRIX_UI_SETTINGS_FILE="${MATRIX_UI_SETTINGS_FILE:-$MATRIX_SETTINGS_DIR/ui-settings.json}"
MATRIX_MOTION_SETTINGS_FILE="${MATRIX_MOTION_SETTINGS_FILE:-$MATRIX_SETTINGS_DIR/motion-control.json}"
MATRIX_VIDEO_SETTINGS_FILE="${MATRIX_VIDEO_SETTINGS_FILE:-$MATRIX_SETTINGS_DIR/video-settings.json}"
for settings_path_name in \
    MATRIX_MOUSE_SETTINGS_FILE \
    MATRIX_UI_SETTINGS_FILE \
    MATRIX_MOTION_SETTINGS_FILE \
    MATRIX_VIDEO_SETTINGS_FILE; do
    settings_path="${!settings_path_name}"
    if [[ "$settings_path" != /* ]]; then
        echo "[ERROR] $settings_path_name must be absolute" >&2
        exit 1
    fi
    printf -v "$settings_path_name" '%s' "$(realpath -m "$settings_path")"
done
if [[ -z "${MATRIX_GAME_APPLIED_VIDEO_SETTINGS_JSON:-}" ]]; then
    if [[ -f "$PROJECT_ROOT/scripts/matrix_video_settings.py" ]]; then
        if ! MATRIX_GAME_APPLIED_VIDEO_SETTINGS_JSON="$(
            /usr/bin/python3 -I "$PROJECT_ROOT/scripts/matrix_video_settings.py" \
                --settings-file "$MATRIX_VIDEO_SETTINGS_FILE" \
                launch-json
        )"; then
            echo "[ERROR] Invalid video settings file: $MATRIX_VIDEO_SETTINGS_FILE" >&2
            exit 1
        fi
    else
        MATRIX_GAME_APPLIED_VIDEO_SETTINGS_JSON='{"camera_distance_cm":150,"camera_distance_max_cm":500,"camera_distance_min_cm":80,"camera_smoothing":"medium","fps_limit":60,"quality":"high","resolution":"1920x1080","resolution_height":1080,"resolution_width":1920,"revision":0,"window_mode":"borderless"}'
    fi
fi
if [[ "$MATRIX_GAME_CAMERA_DISTANCE_EXPLICIT" == "0" ]]; then
    if ! MATRIX_GAME_CAMERA_DISTANCE_CM="$(
        /usr/bin/python3 -I - "$MATRIX_GAME_APPLIED_VIDEO_SETTINGS_JSON" <<'PY'
import json
import sys
value = json.loads(sys.argv[1])
distance = value.get("camera_distance_cm")
if type(distance) is not int:
    raise SystemExit("camera_distance_cm must be an integer")
print(distance)
PY
    )"; then
        echo "[ERROR] Invalid video camera distance from settings" >&2
        exit 1
    fi
fi
export MATRIX_HOST_PROFILE="${MATRIX_HOST_PROFILE:-$MATRIX_SETTINGS_PROFILE}"
export MATRIX_SETTINGS_PROFILE MATRIX_MOUSE_SETTINGS_FILE MATRIX_UI_SETTINGS_FILE
export MATRIX_MOTION_SETTINGS_FILE MATRIX_VIDEO_SETTINGS_FILE
export MATRIX_GAME_APPLIED_VIDEO_SETTINGS_JSON MATRIX_GAME_CAMERA_DISTANCE_CM

case "${MATRIX_DISABLE_MC,,}" in
    1|true|yes|on)
        MATRIX_MC_DISABLED=true
        ;;
    0|false|no|off|"")
        MATRIX_MC_DISABLED=false
        ;;
    *)
        echo "[ERROR] MATRIX_DISABLE_MC must be a boolean: $MATRIX_DISABLE_MC" >&2
        exit 1
        ;;
esac

case "${MATRIX_GAME_CENTERED_CAMERA,,}" in
    1|true|yes|on)
        GAME_CENTERED_CAMERA_ENABLED=true
        ;;
    0|false|no|off)
        GAME_CENTERED_CAMERA_ENABLED=false
        ;;
    *)
        echo "[ERROR] MATRIX_GAME_CENTERED_CAMERA must be a boolean:" \
            "$MATRIX_GAME_CENTERED_CAMERA" >&2
        exit 1
        ;;
esac

# viewclass accepts a short reflected class name.  Keep this override to one
# Blueprint-generated class token: whitespace or console separators here would
# turn a data override into an additional UE console command.
if [[ -n "$MATRIX_GAME_CAMERA_VIEW_CLASS" \
    && ! "$MATRIX_GAME_CAMERA_VIEW_CLASS" \
        =~ ^[A-Za-z_][A-Za-z0-9_]{0,126}_C$ ]]; then
    echo "[ERROR] MATRIX_GAME_CAMERA_VIEW_CLASS must be a short Blueprint" \
        "class ending in _C: $MATRIX_GAME_CAMERA_VIEW_CLASS" >&2
    exit 1
fi

SIM_LAUNCHER_ROOT="${SIM_LAUNCHER_ROOT:-$PROJECT_ROOT}"
CUSTOM_WRAPPER="$SIM_LAUNCHER_ROOT/scripts/run_custom_urdf.sh"

join_ld_library_path() {
    local joined=""
    local dir
    for dir in "$@"; do
        if [[ -d "$dir" ]]; then
            joined="${joined}${joined:+:}$dir"
        fi
    done
    if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
        joined="${joined}${joined:+:}${LD_LIBRARY_PATH}"
    fi
    printf '%s\n' "$joined"
}

setup_runtime_environment() {
    case "${MATRIX_SONIC,,}" in
        1|true|yes|on)
            # The native SONIC launcher already constructed and verified the
            # locked ROS/LD environment. Sourcing the legacy system overlay here
            # would mutate PYTHONPATH after the qualification receipt was issued.
            return
            ;;
    esac
    if [[ -f /opt/ros/humble/setup.bash ]]; then
        set +u
        # shellcheck disable=SC1091
        source /opt/ros/humble/setup.bash
        set -u
    fi
}

mujoco_ld_library_path() {
    join_ld_library_path \
        "$PROJECT_ROOT/src/robot_mujoco/simulate/build" \
        "/opt/ros/humble/lib" \
        "/opt/ros/humble/lib/x86_64-linux-gnu" \
        "$PROJECT_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux" \
        "$PROJECT_ROOT/src/UeSim/Linux/Engine/Binaries/Linux"
}

ue_ld_library_path() {
    join_ld_library_path \
        "$PROJECT_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux" \
        "$PROJECT_ROOT/src/UeSim/Linux/Engine/Binaries/Linux" \
        "$PROJECT_ROOT/src/UeSim/Linux/Engine/Plugins/Runtime/OpenCV/Binaries/ThirdParty/Linux"
}

mc_ld_library_path() {
    join_ld_library_path "$PROJECT_ROOT/src/robot_mc/build/export/mc/bin"
}

setup_runtime_environment

cleanup_runtime_generated_integrity_files() {
    # UE/MuJoCo writes this diagnostic beside binaries on some crashes.  The
    # launcher verifies Binaries/Linux as a locked runtime tree, so leaving the
    # generated log there makes the next launch fail before Matrix can even
    # start.  The file is runtime output, not an installed artifact.
    rm -f -- \
        "$PROJECT_ROOT/src/UeSim/Linux/MUJOCO_LOG.TXT" \
        "$PROJECT_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux/MUJOCO_LOG.TXT"
}

cleanup_runtime_generated_integrity_files

if [[ "${SIM_LAUNCHER_SKIP_CUSTOM_URDF_WRAPPER:-0}" != "1" ]] && [[ "$ROBOT_ARG" == "custom" || "$ROBOT_ARG" == "7" ]] && [[ -n "$CUSTOM_URDF" ]]; then
    if [[ -f "$CUSTOM_WRAPPER" ]]; then
        echo "[INFO] Delegating custom URDF setup to $CUSTOM_WRAPPER"
        exec "$CUSTOM_WRAPPER" "$ROBOT_ARG" "$SCENE_ID" "$OFFSCREEN" "$PIXELSTREAM" "$MUJOCORUNNING" "$CUSTOM_URDF" "$CUSTOM_NAME"
    else
        echo "[ERROR] Custom URDF wrapper not found at: $CUSTOM_WRAPPER" >&2
        exit 1
    fi
fi

run_env_check() {
    if [[ "${MATRIX_SKIP_ENV_CHECK:-0}" == "1" ]]; then
        echo "[INFO] Environment check skipped by MATRIX_SKIP_ENV_CHECK=1"
        return 0
    fi

    local checker="$PROJECT_ROOT/scripts/check_env.sh"
    if [[ ! -x "$checker" ]]; then
        echo "[WARN] Environment checker not found or not executable: $checker"
        return 0
    fi

    local checked_mujoco="$MUJOCORUNNING"
    case "${MATRIX_SONIC,,}" in
        1|true|yes|on)
            # SONIC owns the external MuJoCo process. The bundled robot_mujoco
            # executable and /opt/ros are not part of this launch topology.
            checked_mujoco=0
            ;;
    esac

    "$checker" runtime \
        --robot "$ROBOT_ARG" \
        --scene "$SCENE_ID" \
        --mujoco "$checked_mujoco" \
        --offscreen "$OFFSCREEN"
}

run_env_check

#######################################
# 全局 PID 管理
#######################################

PROCESS_PATTERNS=(
    "robot_mujoco"
    "jszr_mujoco_ue"
    "zsibot_mujoco_ue"
    "UnrealGame"
    "UE4Editor"
    "mc_ctrl"
)

kill_known_processes() {
    local signal="$1"
    local pattern
    for pattern in "${PROCESS_PATTERNS[@]}"; do
        pkill "-${signal}" -f "${pattern}" 2>/dev/null || true
    done
}

kill_known_processes TERM


PIDS=()
WATCHDOG_PID=""
FORCED_CLEANUP_PID=""
SONIC_PID=""
UE_PID=""
UE_SUPERVISOR_PID=""
UE_SUPERVISOR_REAPED=0
UE_CONTROL_FD=""
UE_LIFECYCLE_DIR=""
UE_FAILURE_FILE=""
UE_PID_FILE=""
UE_CAMERA_STATE_FILE=""
RUN_SIM_PARENT_PID="${MATRIX_SONIC_LAUNCHER_PID:-$PPID}"
CLEANUP_STARTED=0
CLEANUP_FAILED=0
X_POINTER_ACCELERATION_RESTORE_NEEDED=0
X_POINTER_ACCELERATION=""
X_POINTER_THRESHOLD=""
X_POINTER_DISPLAY=""
X_POINTER_XSET_BIN=""
CENTERED_CAMERA_OVERLAY_ENABLED=false
CENTERED_CAMERA_OVERLAY_INSTALLED=0

record_ue_supervisor_failure() {
    if [[ -z "${UE_FAILURE_FILE:-}" || -e "$UE_FAILURE_FILE" ]]; then
        return
    fi
    local temporary_failure="${UE_FAILURE_FILE}.tmp.$$"
    printf '%s\n' '{"name":"ue","exit_code":255}' > "$temporary_failure"
    mv -f -- "$temporary_failure" "$UE_FAILURE_FILE"
}

remove_managed_pid() {
    local target="$1"
    local -a remaining=()
    local pid
    for pid in "${PIDS[@]:-}"; do
        if [[ -n "$pid" && "$pid" != "$target" ]]; then
            remaining+=("$pid")
        fi
    done
    PIDS=("${remaining[@]}")
}

start_supervised_ue() {
    local ue_log="$1"
    shift
    local -a ue_command=("$@")

    mkdir -p "$PROJECT_ROOT/outputs"
    UE_LIFECYCLE_DIR="$(mktemp -d "$PROJECT_ROOT/outputs/.matrix-ue-lifecycle.XXXXXX")"
    UE_FAILURE_FILE="$UE_LIFECYCLE_DIR/failure.json"
    UE_PID_FILE="$UE_LIFECYCLE_DIR/ue.pid"
    local -a camera_probe_args=()
    if [[ "${MATRIX_GAME_CAMERA_YAW_SOURCE:-fixed}" == "ue-final-pov" ]]; then
        if [[ ! -f "$MATRIX_UE_CAMERA_LAYOUT" ]]; then
            echo "[ERROR] UE final-POV layout is missing: $MATRIX_UE_CAMERA_LAYOUT" >&2
            return 1
        fi
        UE_CAMERA_STATE_FILE="$UE_LIFECYCLE_DIR/camera-state.bin"
        camera_probe_args=(
            --camera-state-file "$UE_CAMERA_STATE_FILE"
            --camera-layout "$MATRIX_UE_CAMERA_LAYOUT"
        )
    fi
    local supervisor_python="${MATRIX_SONIC_PYTHON:-$(command -v python3)}"
    coproc MATRIX_UE_SUPERVISOR {
        exec "$supervisor_python" "$PROJECT_ROOT/scripts/supervise_matrix_ue.py" \
            --pid-file "$UE_PID_FILE" \
            --failure-file "$UE_FAILURE_FILE" \
            --log "$ue_log" \
            --expected-parent-pid "$$" \
            "${camera_probe_args[@]}" \
            -- "${ue_command[@]}"
    }
    UE_SUPERVISOR_PID="$MATRIX_UE_SUPERVISOR_PID"
    UE_CONTROL_FD="${MATRIX_UE_SUPERVISOR[1]}"
    local supervisor_output_fd="${MATRIX_UE_SUPERVISOR[0]}"
    # The helper writes diagnostics to stderr and UE output to ue_log. Close its
    # otherwise-unused coprocess stdout pipe so no descriptor survives cleanup.
    exec {supervisor_output_fd}<&-

    local attempt
    for ((attempt = 0; attempt < 250; attempt++)); do
        if [[ -s "$UE_PID_FILE" ]]; then
            UE_PID="$(<"$UE_PID_FILE")"
            break
        fi
        sleep 0.02
    done
    if [[ ! "$UE_PID" =~ ^[1-9][0-9]*$ ]]; then
        echo "[ERROR] UE supervisor failed to publish the UE PID" >&2
        return 1
    fi
    echo "[INFO] UE PID $UE_PID (supervisor PID $UE_SUPERVISOR_PID)"
}

stop_supervised_ue() {
    if [[ -z "${UE_SUPERVISOR_PID:-}" ]]; then
        return
    fi
    local stop_delivered=0
    local supervisor_exit_code=255
    if [[ "$UE_SUPERVISOR_REAPED" == "1" ]]; then
        if [[ -n "${UE_CONTROL_FD:-}" ]]; then
            exec {UE_CONTROL_FD}>&-
            UE_CONTROL_FD=""
        fi
        record_ue_supervisor_failure
        UE_SUPERVISOR_PID=""
        return
    fi
    if [[ -n "${UE_CONTROL_FD:-}" ]]; then
        if printf '%s\n' stop >&"$UE_CONTROL_FD" 2>/dev/null; then
            stop_delivered=1
        fi
        exec {UE_CONTROL_FD}>&-
        UE_CONTROL_FD=""
    fi
    if wait "$UE_SUPERVISOR_PID"; then
        supervisor_exit_code=0
    else
        supervisor_exit_code=$?
    fi
    if [[ "$stop_delivered" != "1" || "$supervisor_exit_code" == "255" ]]; then
        record_ue_supervisor_failure
    elif [[ "$supervisor_exit_code" != "0" && ! -e "$UE_FAILURE_FILE" ]]; then
        record_ue_supervisor_failure
    fi
    UE_SUPERVISOR_PID=""
}

install_centered_camera_overlay() {
    /usr/bin/python3 -I "$PROJECT_ROOT/scripts/matrix_ue_overlay.py" \
        install \
        --contract "$MATRIX_CENTERED_CAMERA_OVERLAY_CONTRACT" \
        --bundle "$MATRIX_CENTERED_CAMERA_OVERLAY_BUNDLE" \
        --project-root "$PROJECT_ROOT"
    CENTERED_CAMERA_OVERLAY_INSTALLED=1
}

remove_centered_camera_overlay() {
    if [[ "$CENTERED_CAMERA_OVERLAY_INSTALLED" != "1" ]]; then
        return 0
    fi
    if /usr/bin/python3 -I "$PROJECT_ROOT/scripts/matrix_ue_overlay.py" \
        remove \
        --contract "$MATRIX_CENTERED_CAMERA_OVERLAY_CONTRACT" \
        --project-root "$PROJECT_ROOT"; then
        CENTERED_CAMERA_OVERLAY_INSTALLED=0
        return 0
    fi
    echo "[ERROR] Failed to remove the verified centered-camera overlay" >&2
    return 1
}

verify_centered_camera_overlay_mount() {
    local ue_log="$1"
    local start_offset="$2"
    local timeout="${MATRIX_UE_OVERLAY_MOUNT_TIMEOUT_SECONDS:-5}"
    if [[ ! "$timeout" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
        echo "[ERROR] MATRIX_UE_OVERLAY_MOUNT_TIMEOUT_SECONDS must be non-negative" >&2
        return 1
    fi
    local attempts
    attempts="$(/usr/bin/python3 -I - "$timeout" <<'PY'
import math
import sys

timeout = float(sys.argv[1])
if not math.isfinite(timeout):
    raise SystemExit("mount timeout must be finite")
print(max(1, math.ceil(timeout / 0.05) + 1))
PY
)" || return 1
    local attempt
    for ((attempt = 0; attempt < attempts; attempt++)); do
        local mount_status
        mount_status="$(/usr/bin/python3 -I - \
            "$ue_log" "$start_offset" "$CENTERED_CAMERA_OVERLAY_STEM" <<'PY'
from pathlib import Path
import re
import sys

path = Path(sys.argv[1])
offset = int(sys.argv[2])
stem = sys.argv[3]
if not path.is_file():
    print("waiting")
    raise SystemExit(0)
size = path.stat().st_size
if size < offset:
    print("truncated")
    raise SystemExit(0)
with path.open("rb") as stream:
    stream.seek(offset)
    segment = stream.read().decode("utf-8", errors="replace")
stem_lines = [line for line in segment.splitlines() if stem in line]
if any("Failed" in line for line in stem_lines):
    print("failed")
    raise SystemExit(0)
prefix = r"^\s*(?:\[[^\]\r\n]*\]\s*)*LogPakFile:\s*Display:\s*"
found_pattern = re.compile(
    prefix + r"Found Pak file (?P<path>.+?) attempting to mount\.?\s*$"
)
mounted_pattern = re.compile(
    prefix + r"Mounted IoStore container (?P<path>.+?)\s*$"
)

def exact_basename(match, expected):
    if match is None:
        return False
    raw = match.group("path").strip().strip("\"'")
    return raw.replace("\\", "/").rsplit("/", 1)[-1] == expected

found = any(
    exact_basename(found_pattern.fullmatch(line), f"{stem}.pak")
    for line in stem_lines
)
mounted = any(
    exact_basename(mounted_pattern.fullmatch(line), f"{stem}.utoc")
    for line in stem_lines
)
print("mounted" if found and mounted else "waiting")
PY
        )" || return 1
        case "$mount_status" in
            mounted)
                echo "[INFO] Verified Matrix centered-camera IoStore mount:" \
                    "$CENTERED_CAMERA_OVERLAY_STEM"
                return 0
                ;;
            failed)
                echo "[ERROR] UE reported Failed for the current centered-camera" \
                    "overlay log segment: $ue_log" >&2
                return 1
                ;;
            truncated)
                echo "[ERROR] UE log was truncated after the centered-camera" \
                    "startup boundary: $ue_log" >&2
                return 1
                ;;
            waiting) ;;
            *)
                echo "[ERROR] Invalid centered-camera mount verifier status:" \
                    "$mount_status" >&2
                return 1
                ;;
        esac
        if ((attempt + 1 < attempts)); then
            sleep 0.05
        fi
    done
    echo "[ERROR] UE log did not confirm Found and Mounted IoStore events for" \
        "$CENTERED_CAMERA_OVERLAY_STEM: $ue_log" >&2
    return 1
}

verify_material_fix_install() {
    local ue_log="$1"
    local start_offset="$2"
    local status
    status="$(/usr/bin/python3 -I - "$ue_log" "$start_offset" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
offset = int(sys.argv[2])
marker = "matrix-ue-material-fix: installed audited Matrix 0.1.2 material bridge"
if not path.is_file():
    print("missing-log")
    raise SystemExit(0)
size = path.stat().st_size
if size < offset:
    print("truncated")
    raise SystemExit(0)
with path.open("rb") as stream:
    stream.seek(offset)
    lines = stream.read().decode("utf-8", errors="replace").splitlines()
if any(line.strip().startswith("matrix-ue-material-fix FATAL:") for line in lines):
    print("fatal")
elif any(line.strip() == marker for line in lines):
    print("installed")
else:
    print("missing-marker")
PY
    )" || return 1
    case "$status" in
        installed)
            echo "[INFO] Verified Matrix UE material fix installation"
            ;;
        fatal)
            echo "[ERROR] Matrix UE material fix reported a fatal guard failure:" \
                "$ue_log" >&2
            return 1
            ;;
        missing-log|truncated|missing-marker)
            echo "[ERROR] Matrix UE material fix did not emit its current-run" \
                "installation marker ($status): $ue_log" >&2
            return 1
            ;;
        *)
            echo "[ERROR] Invalid Matrix UE material-fix verifier status: $status" >&2
            return 1
            ;;
    esac
}

schedule_forced_cleanup() {
    (
        trap '' HUP
        sleep 1
        kill_known_processes TERM
        sleep 1
        kill_known_processes KILL
    ) </dev/null >/dev/null 2>&1 &
    FORCED_CLEANUP_PID=$!
}

start_parent_watchdog() {
    local run_sim_pid="$$"
    local launcher_pid="$RUN_SIM_PARENT_PID"
    (
        trap 'exit 0' TERM INT
        trap '' HUP
        while kill -0 "${run_sim_pid}" 2>/dev/null \
            && kill -0 "${launcher_pid}" 2>/dev/null; do
            sleep 1
        done

        if kill -0 "${run_sim_pid}" 2>/dev/null; then
            echo "[INFO] Top-level launcher exited unexpectedly; stopping run_sim..."
            kill -TERM "${run_sim_pid}" 2>/dev/null || true
            exit 0
        fi
        echo "[INFO] run_sim exited unexpectedly, cleaning known child processes..."
        schedule_forced_cleanup
        kill_known_processes TERM
    ) &
    WATCHDOG_PID=$!
}

stop_parent_watchdog() {
    if [[ -n "${WATCHDOG_PID:-}" ]] && kill -0 "${WATCHDOG_PID}" 2>/dev/null; then
        kill -TERM "${WATCHDOG_PID}" 2>/dev/null || true
        wait "${WATCHDOG_PID}" 2>/dev/null || true
    fi
}

restore_remote_pointer_acceleration() {
    if [[ "${X_POINTER_ACCELERATION_RESTORE_NEEDED:-0}" != "1" ]]; then
        return 0
    fi

    # Keep the restore armed when the X server is temporarily unavailable so a
    # later cleanup attempt can retry it.  Pointer control belongs to the X
    # display, not to UE, so use the exact display and xset binary recorded at
    # setup time.
    if DISPLAY="$X_POINTER_DISPLAY" "$X_POINTER_XSET_BIN" m \
        "$X_POINTER_ACCELERATION" "$X_POINTER_THRESHOLD" \
        >/dev/null 2>&1; then
        echo "[INFO] Restored X pointer acceleration: " \
            "$X_POINTER_ACCELERATION threshold $X_POINTER_THRESHOLD"
        X_POINTER_ACCELERATION_RESTORE_NEEDED=0
        return 0
    fi

    echo "[WARN] Could not restore X pointer acceleration on" \
        "DISPLAY=$X_POINTER_DISPLAY; restore it manually with:" \
        "xset m $X_POINTER_ACCELERATION $X_POINTER_THRESHOLD" >&2
    return 1
}

configure_remote_pointer_acceleration() {
    # This is deliberately narrower than the SDL raw-relative hints below.
    # It linearizes the X11 pointer stream used by the Remote settings panel
    # and x11-mirror only for interactive SONIC game launches.
    if ! $MATRIX_SONIC_ENABLED \
        || [[ "${MATRIX_SONIC_CONTROL_SOURCE:-planner}" != "game" ]] \
        || [[ "${MATRIX_MOUSE_APPLIED_PROFILE:-local}" != "remote" ]]; then
        return 0
    fi

    if [[ -z "${DISPLAY:-}" ]]; then
        echo "[WARN] Remote mouse profile could not linearize X pointer" \
            "acceleration because DISPLAY is unset; continuing" >&2
        return 0
    fi

    local xset_bin
    xset_bin="$(type -P xset || true)"
    if [[ -z "$xset_bin" ]]; then
        echo "[WARN] Remote mouse profile could not linearize X pointer" \
            "acceleration because xset is unavailable; continuing" >&2
        return 0
    fi

    local pointer_query
    if ! pointer_query="$(DISPLAY="$DISPLAY" LC_ALL=C "$xset_bin" q 2>/dev/null)"; then
        echo "[WARN] Remote mouse profile could not read X pointer" \
            "acceleration on DISPLAY=$DISPLAY; continuing" >&2
        return 0
    fi
    if [[ ! "$pointer_query" =~ acceleration:[[:space:]]*([0-9]+/[0-9]+)[[:space:]]+threshold:[[:space:]]*([0-9]+) ]]; then
        echo "[WARN] Remote mouse profile could not parse X pointer" \
            "acceleration on DISPLAY=$DISPLAY; continuing" >&2
        return 0
    fi

    X_POINTER_ACCELERATION="${BASH_REMATCH[1]}"
    X_POINTER_THRESHOLD="${BASH_REMATCH[2]}"
    X_POINTER_DISPLAY="$DISPLAY"
    X_POINTER_XSET_BIN="$xset_bin"
    # Arm restoration before changing the X server.  Even an unusual xset
    # implementation that changes state and then exits nonzero is covered.
    X_POINTER_ACCELERATION_RESTORE_NEEDED=1
    if DISPLAY="$X_POINTER_DISPLAY" "$X_POINTER_XSET_BIN" m 1/1 0 \
        >/dev/null 2>&1; then
        echo "[INFO] Remote mouse profile temporarily set X pointer" \
            "acceleration to 1/1 threshold 0" \
            "(saved $X_POINTER_ACCELERATION threshold $X_POINTER_THRESHOLD)"
        return 0
    fi

    echo "[WARN] Remote mouse profile could not set X pointer acceleration" \
        "on DISPLAY=$X_POINTER_DISPLAY; continuing" >&2
    restore_remote_pointer_acceleration || true
    return 0
}

cleanup() {
    if [[ "$CLEANUP_STARTED" == "1" ]]; then
        return
    fi
    CLEANUP_STARTED=1
    echo "[INFO] ===== Cleaning up processes ====="

    stop_parent_watchdog
    restore_remote_pointer_acceleration || true

    # 1. 优雅关闭脚本启动的进程
    for pid in "${PIDS[@]:-}"; do
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            echo "[INFO] SIGTERM PID $pid"
            kill -TERM "$pid" 2>/dev/null || true
        fi
    done
    # UE is owned by a dedicated supervisor and is never placed in the generic
    # PID list. Its control-pipe stop plus exact shell wait cannot target a
    # recycled UE or supervisor PID.
    stop_supervised_ue
    # The cooked overlay must remain present for the whole UE lifetime.  Retire
    # its active directory only after the exact supervised UE has stopped.
    if ! remove_centered_camera_overlay; then
        CLEANUP_FAILED=1
    fi

    # 2. 兜底清理（仅限本项目）
    kill_known_processes TERM
    schedule_forced_cleanup

    # Give the SONIC Python runtime time to close its exact native deploy/PICO
    # children. Never pattern-kill those names: TRNA may also host a real robot.
    local attempt
    # NativeProcessGroup can spend about nine seconds on native stop, TERM, and
    # KILL before closing the renderer/simulator. Leave a clear outer margin.
    for ((attempt = 0; attempt < 150; attempt++)); do
        local any_alive=0
        for pid in "${PIDS[@]:-}"; do
            if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
                any_alive=1
                break
            fi
        done
        [[ "$any_alive" == "0" ]] && break
        sleep 0.1
    done

    # 3. 最终兜底
    for pid in "${PIDS[@]:-}"; do
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            kill -KILL "$pid" 2>/dev/null || true
        fi
        [[ -n "$pid" ]] && wait "$pid" 2>/dev/null || true
    done
    kill_known_processes KILL

    if [[ -n "${FORCED_CLEANUP_PID:-}" ]] && kill -0 "${FORCED_CLEANUP_PID}" 2>/dev/null; then
        kill -TERM "${FORCED_CLEANUP_PID}" 2>/dev/null || true
        wait "${FORCED_CLEANUP_PID}" 2>/dev/null || true
    fi

    if [[ -n "${UE_LIFECYCLE_DIR:-}" ]]; then
        rm -rf -- "$UE_LIFECYCLE_DIR"
    fi

    # Retry once after child teardown if the display was transiently
    # unavailable at the beginning of cleanup.
    restore_remote_pointer_acceleration || true
    echo "[INFO] ===== Cleanup finished ====="
    if [[ "$CLEANUP_FAILED" == "1" ]]; then
        echo "[ERROR] Matrix cleanup failed; refusing a successful exit" >&2
        return 1
    fi
}

handle_signal() {
    local exit_code="$1"
    if [[ "$CLEANUP_STARTED" == "1" ]]; then
        return
    fi
    cleanup
    exit "$exit_code"
}

trap cleanup EXIT
trap 'handle_signal 130' SIGINT
trap 'handle_signal 143' SIGTERM
trap 'handle_signal 129' SIGHUP
start_parent_watchdog

#######################################
# Offscreen / PixelStreaming
#######################################
USE_OFFSCREEN=""
[[ "$OFFSCREEN" == "1" ]] && USE_OFFSCREEN="-RenderOffScreen"

USE_PIXELSTREAMER=""
[[ "$PIXELSTREAM" == "1" ]] && USE_PIXELSTREAMER="-PixelStreamingURL=ws://127.0.0.1:8888"

UE_MAX_FPS="${MATRIX_UE_MAX_FPS:-30}"
if [[ ! "$UE_MAX_FPS" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    echo "[ERROR] MATRIX_UE_MAX_FPS must be a non-negative number: $UE_MAX_FPS" >&2
    exit 1
fi
UE_EXEC_CMDS="t.MaxFPS $UE_MAX_FPS,r.MotionBlurQuality 0"

#######################################
# 场景配置
#######################################
SCENE="scene_terrain_wh.xml"
MAPNAME="/Game/Maps/SceneWorld"
WEAPON=""

case "$SCENE_ID" in
    0)  SCENE="scene_terrain_custom.xml"; MAPNAME="/Game/Maps/CustomWorld" ;;
    1)  SCENE="scene_terrain_wh.xml";     MAPNAME="/Game/Maps/SceneWorld" ;;
    2)  SCENE="scene_terrain_t10.xml";    MAPNAME="/Game/Maps/Town10World" ;;
    3)  SCENE="scene_terrain_yard.xml";   MAPNAME="/Game/Maps/YardWorld" ;;
    4)  SCENE="scene_terrain_crowd.xml";  MAPNAME="/Game/Maps/CrowdWorld" ;;
    5)  SCENE="scene_terrain_venice.xml"; MAPNAME="/Game/Maps/VeniceWorld" ;;
    6)  SCENE="scene_terrain_house.xml";  MAPNAME="/Game/Maps/HouseWorld" ;;
    7)  SCENE="scene_terrain_rw.xml";     MAPNAME="/Game/Maps/RunningWorld" ;;
    8)  SCENE="scene_terrain_zombie.xml"; MAPNAME="/Game/Maps/Town10Zombie"; WEAPON="gun" ;;
    9)  SCENE="scene_terrain_flat.xml";   MAPNAME="/Game/Maps/IROSFlatWorld" ;;
    10) SCENE="scene_terrain_sloped.xml"; MAPNAME="/Game/Maps/IROSSlopedWorld" ;;
    11) SCENE="scene_terrain_flat25.xml"; MAPNAME="/Game/Maps/IROSFlatWorld2025" ;;
    12) SCENE="scene_terrain_sloped25.xml"; MAPNAME="/Game/Maps/IROSSloppedWorld2025" ;;
    13) SCENE="scene_terrain_office.xml"; MAPNAME="/Game/Maps/OfficeWorld" ;;
    14) SCENE="3dgs.xml";                 MAPNAME="/Game/Maps/3DGSWorld" ;;
    16) SCENE="3dgs.xml";                 MAPNAME="/Game/Maps/3DGSWorld" ;;
    17) SCENE="3dgs.xml";                 MAPNAME="/Game/Maps/3DGSWorld" ;;
    15)
        SCENE="scene_terrain_moon_dynamic.xml"
        MAPNAME="/Game/Maps/MoonWorld"
        mkdir -p src/robot_mujoco/simulate/build src/UeSim/Linux/zsibot_mujoco_ue/Content/model/dynamicmap
        cp dynamicmaps/moonworld.bin src/robot_mujoco/simulate/build/DynamicMapData.bin
        cp dynamicmaps/moonworld.bin src/UeSim/Linux/zsibot_mujoco_ue/Content/model/dynamicmap/moonworld.bin
        ;;
    20) SCENE="scene_terrain_cali.xml"; MAPNAME="/Game/Maps/CaliWorld" ;;
    21) SCENE="scene_terrain_apart2.xml"; MAPNAME="/Game/Maps/ApartmentWorld" ;;
    22) SCENE="scene_terrain_meet.xml"; MAPNAME="/Game/Maps/MeetRoomWorld" ;;
    *)
        echo "[WARN] Unknown scene id $SCENE_ID, using default"
        ;;
esac

sed -i "s/^robot_scene: .*/robot_scene: \"$SCENE\"/" src/robot_mujoco/simulate/config.yaml

#######################################
# 机器人类型 & 启动策略
#######################################
TARGET_FILE="src/robot_mc/run_mc.sh"
ENABLE_MUJOCO=false
ENABLE_MC=false
ROBOTTYPE="xgb"
RUNTIME_ROBOTTYPE="xgb"

configure_mc_robot_type() {
    local mc_robot_type="$1"
    if $MATRIX_MC_DISABLED; then
        return 0
    fi
    sed -i "s/export ROBOT_TYPE=.*/export ROBOT_TYPE=${mc_robot_type}/" "$TARGET_FILE"
}

set_mc_motor_platform() {
    local config_name="$1"
    local platform_type="$2"
    local config_path="$PROJECT_ROOT/src/robot_mc/build/export/config/$config_name"
    if $MATRIX_MC_DISABLED; then
        return 0
    fi
    if [[ ! -f "$config_path" ]]; then
        echo "[ERROR] Matrix motion-controller config is missing: $config_path" >&2
        exit 1
    fi
    sed -i "s/motor_platform_type: .*/motor_platform_type: ${platform_type}/" "$config_path"
}

# MUJOCORUNNING is 1 config/config.json中"mujoco_running": true，否则为 false
if [[ "$MUJOCORUNNING" == "1" ]]; then
    ENABLE_MUJOCO=true
    echo "[INFO] MuJoCo will be enabled. Please ensure you have the proper license and setup."
else
    ENABLE_MUJOCO=false
    echo "[INFO] MuJoCo will be disabled. The simulation will run without physics-based dynamics."
fi


case "$ROBOT_ARG" in
    4|go2)
        ROBOTTYPE="go2"
        RUNTIME_ROBOTTYPE="go2"
        ENABLE_MC=false
        # sed -i 's/export ROBOT_TYPE=.*/export ROBOT_TYPE=GO2/' "$TARGET_FILE"
        ;;
    5|go2w)
        ROBOTTYPE="go2w"
        RUNTIME_ROBOTTYPE="go2w"
        ENABLE_MC=false
        # sed -i 's/export ROBOT_TYPE=.*/export ROBOT_TYPE=GO2W/' "$TARGET_FILE"
        ;;
    1|xgb)
        ROBOTTYPE="xgb"
        RUNTIME_ROBOTTYPE="xgb"
        ENABLE_MC=true
        configure_mc_robot_type "XG"
        if [[ "$MUJOCORUNNING" == "1" ]]; then
            ENABLE_MUJOCO=true
            set_mc_motor_platform "xg-user-parameters.yaml" "5"
        else
            ENABLE_MUJOCO=false
            set_mc_motor_platform "xg-user-parameters.yaml" "8"
        fi
        ;;
    2|xgw)
        ROBOTTYPE="xgw"
        RUNTIME_ROBOTTYPE="xgw"
        ENABLE_MC=true
        configure_mc_robot_type "XGW"
        if [[ "$MUJOCORUNNING" == "1" ]]; then
            ENABLE_MUJOCO=true
            set_mc_motor_platform "xg_wheel-user-parameters.yaml" "5"
        else
            ENABLE_MUJOCO=false
            set_mc_motor_platform "xg_wheel-user-parameters.yaml" "8"
        fi
        ;;
    3|zgws)
        ROBOTTYPE="zgws"
        RUNTIME_ROBOTTYPE="zgws"
        ENABLE_MC=true
        configure_mc_robot_type "ZGWS"
        if [[ "$MUJOCORUNNING" == "1" ]]; then
            ENABLE_MUJOCO=true
            set_mc_motor_platform "zg_wheels-user-parameters.yaml" "5"
        else
            ENABLE_MUJOCO=false
            set_mc_motor_platform "zg_wheels-user-parameters.yaml" "8"
        fi
        ;;
    6|xxg)
        echo "[ERROR] Robot type '$ROBOT_ARG' is not included in this release" >&2
        exit 1
        ;;
    7|custom)
        ROBOTTYPE="custom"
        RUNTIME_ROBOTTYPE="custom"
        ENABLE_MC=true
        # Read reference_profile from manifest to select the correct MC config
        _CUSTOM_MODEL_DIR="${CUSTOM_NAME:-custom}"
        _MANIFEST="src/robot_mujoco/zsibot_robots/custom/_cache/${_CUSTOM_MODEL_DIR}/manifest.json"
        _REF_PROFILE=""
        if [[ -f "$_MANIFEST" ]]; then
            _REF_PROFILE="$(jq -r '.reference_profile // empty' "$_MANIFEST" 2>/dev/null || true)"
        fi
        echo "[INFO] custom robot reference_profile: '${_REF_PROFILE:-none}'"
        if [[ -n "$_REF_PROFILE" ]]; then
            # Keep custom scene/layout handling, but expose the matched native
            # robot type to downstream runtime config.
            RUNTIME_ROBOTTYPE="$_REF_PROFILE"
        fi
        case "${_REF_PROFILE}" in
            xgw|zgw)
                # 16-DOF wheel-leg (xgw/zgw) → XGW MC config
                configure_mc_robot_type "XGW"
                if [[ "$MUJOCORUNNING" == "1" ]]; then
                    ENABLE_MUJOCO=true
                    set_mc_motor_platform "xg_wheel-user-parameters.yaml" "5"
                else
                    ENABLE_MUJOCO=false
                    set_mc_motor_platform "xg_wheel-user-parameters.yaml" "8"
                fi
                ;;
            xxg)
                # XXG family → XXG MC config
                configure_mc_robot_type "XXG"
                if [[ "$MUJOCORUNNING" == "1" ]]; then
                    ENABLE_MUJOCO=true
                    set_mc_motor_platform "xxg-user-parameters.yaml" "5"
                else
                    ENABLE_MUJOCO=false
                    set_mc_motor_platform "xxg-user-parameters.yaml" "8"
                fi
                ;;
            *)
                # xgb / generic / unknown → XG MC config (default)
                configure_mc_robot_type "XG"
                if [[ "$MUJOCORUNNING" == "1" ]]; then
                    ENABLE_MUJOCO=true
                    set_mc_motor_platform "xg-user-parameters.yaml" "5"
                else
                    ENABLE_MUJOCO=false
                    set_mc_motor_platform "xg-user-parameters.yaml" "8"
                fi
                ;;
        esac
        ;;
    *)
        echo "[ERROR] Unknown robot type: $ROBOT_ARG"
        exit 1
        ;;
esac

if $MATRIX_MC_DISABLED; then
    ENABLE_MC=false
    echo "[INFO] Matrix motion controller disabled by MATRIX_DISABLE_MC=$MATRIX_DISABLE_MC"
fi

case "${MATRIX_SONIC,,}" in
    1|true|yes|on)
        MATRIX_SONIC_ENABLED=true
        ENABLE_MC=false
        if ! $ENABLE_MUJOCO; then
            echo "[ERROR] MATRIX_SONIC requires MuJoCo mode to be enabled" >&2
            exit 1
        fi
        echo "[INFO] Native gear_sonic MuJoCo/DDS driver enabled"
        ;;
    0|false|no|off|"")
        MATRIX_SONIC_ENABLED=false
        ;;
    *)
        echo "[ERROR] MATRIX_SONIC must be a boolean: $MATRIX_SONIC" >&2
        exit 1
        ;;
esac

# The stock cooked package already contains a camera-bearing SpringArm on each
# robot Blueprint.  In interactive SONIC game mode, select the real rendered
# robot as the UE view target and make that native arm direct/collision-aware.
# These are startup console commands, not the Python camera-bridge contract.
# `set Engine.SpringArmComponent` intentionally affects every live spring arm;
# an operator can append a narrower/newer command via MATRIX_UE_EXTRA_EXEC_CMDS.
if $MATRIX_SONIC_ENABLED \
    && [[ "${MATRIX_SONIC_CONTROL_SOURCE:-planner}" == "game" ]] \
    && $GAME_CENTERED_CAMERA_ENABLED; then
    if [[ "$ROBOTTYPE" == "custom" \
        && -n "$MATRIX_CENTERED_CAMERA_OVERLAY_BUNDLE" ]]; then
        if [[ -n "$MATRIX_GAME_CAMERA_VIEW_CLASS" \
            && "$MATRIX_GAME_CAMERA_VIEW_CLASS" != "Spectator_C" ]]; then
            echo "[ERROR] The centered-camera overlay viewclass must be" \
                "Spectator_C or unset; got $MATRIX_GAME_CAMERA_VIEW_CLASS" >&2
            exit 1
        fi
        requested_camera_distance="$MATRIX_GAME_CAMERA_DISTANCE_CM"
        if ! canonical_camera_distance="$(/usr/bin/python3 -I - \
            "$requested_camera_distance" <<'PY'
from decimal import Decimal, InvalidOperation
import re
import sys

raw = sys.argv[1]
if re.fullmatch(r"(?:0|[1-9][0-9]*)(?:\.[0-9]+)?", raw) is None:
    raise SystemExit("camera distance must be a plain non-negative decimal")
try:
    value = Decimal(raw)
except InvalidOperation as exc:
    raise SystemExit("camera distance is invalid") from exc
if value < Decimal("80") or value > Decimal("500"):
    raise SystemExit("camera distance must be within 80..500 cm")
print(format(value.normalize(), "f"))
PY
        )"; then
            echo "[ERROR] MATRIX_GAME_CAMERA_DISTANCE_CM must be a plain" \
                "decimal in [80, 500]: $requested_camera_distance" >&2
            exit 1
        fi
        MATRIX_GAME_CAMERA_DISTANCE_CM="$canonical_camera_distance"
        CENTERED_CAMERA_OVERLAY_ENABLED=true
        GAME_CAMERA_VIEW_CLASS="Spectator_C"
    elif [[ -n "$MATRIX_GAME_CAMERA_VIEW_CLASS" ]]; then
        GAME_CAMERA_VIEW_CLASS="$MATRIX_GAME_CAMERA_VIEW_CLASS"
    else
        case "$ROBOTTYPE" in
            custom) GAME_CAMERA_VIEW_CLASS="MujocoSim_Custom_C" ;;
            go2) GAME_CAMERA_VIEW_CLASS="MujoCoSim_go2_C" ;;
            go2w) GAME_CAMERA_VIEW_CLASS="MujoCoSim_go2w_C" ;;
            xgb) GAME_CAMERA_VIEW_CLASS="MujoCoSim_Xgb_C" ;;
            xgw) GAME_CAMERA_VIEW_CLASS="MujoCoSim_Xgw_C" ;;
            xxg) GAME_CAMERA_VIEW_CLASS="MujoCoSim_Xxg_C" ;;
            zgws) GAME_CAMERA_VIEW_CLASS="MujoCoSim_Zgws_C" ;;
            *)
                echo "[ERROR] No native game-camera view class is mapped for" \
                    "robot type: $ROBOTTYPE" >&2
                exit 1
                ;;
        esac
    fi
    UE_EXEC_CMDS="${UE_EXEC_CMDS},set Engine.SpringArmComponent bEnableCameraLag False"
    UE_EXEC_CMDS="${UE_EXEC_CMDS},set Engine.SpringArmComponent bEnableCameraRotationLag False"
    UE_EXEC_CMDS="${UE_EXEC_CMDS},set Engine.SpringArmComponent bDoCollisionTest True"
    if $CENTERED_CAMERA_OVERLAY_ENABLED; then
        UE_EXEC_CMDS="${UE_EXEC_CMDS},set Engine.SpringArmComponent TargetArmLength ${MATRIX_GAME_CAMERA_DISTANCE_CM}"
    fi
    UE_EXEC_CMDS="${UE_EXEC_CMDS},viewclass ${GAME_CAMERA_VIEW_CLASS}"
    if $CENTERED_CAMERA_OVERLAY_ENABLED; then
        echo "[INFO] Persistent centered-camera overlay enabled:" \
            "robot=MujocoSim_Custom_C viewclass=$GAME_CAMERA_VIEW_CLASS"
    else
        echo "[INFO] Native centered game-camera startup enabled: viewclass=$GAME_CAMERA_VIEW_CLASS"
    fi
elif $MATRIX_SONIC_ENABLED \
    && [[ "${MATRIX_SONIC_CONTROL_SOURCE:-planner}" == "game" ]]; then
    echo "[INFO] Native centered game-camera startup disabled"
fi

# Keep operator commands last by contract.  They can deliberately override a
# default set/viewclass command without editing the launcher.
if [[ -n "${MATRIX_UE_EXTRA_EXEC_CMDS:-}" ]]; then
    UE_EXEC_CMDS="${UE_EXEC_CMDS},${MATRIX_UE_EXTRA_EXEC_CMDS}"
fi

if $MATRIX_SONIC_ENABLED \
    && [[ "${MATRIX_SONIC_QUALIFIED_RUNTIME:-0}" == "1" ]] \
    && [[ "${MATRIX_SONIC_CONTROL_SOURCE:-planner}" == "game" ]]; then
    if [[ "${MATRIX_GAME_CAMERA_YAW_SOURCE:-fixed}" == "fixed" ]]; then
        echo "[ERROR] Qualified game control rejects a fixed camera yaw source" >&2
        exit 1
    fi
    if [[ "${MATRIX_GAME_CAMERA_YAW_SOURCE:-fixed}" == "x11-core-gated" \
        || "${MATRIX_GAME_CAMERA_YAW_SOURCE:-fixed}" == "x11-absolute" \
        || "${MATRIX_GAME_CAMERA_YAW_SOURCE:-fixed}" == "ue-final-pov" ]]; then
        echo "[ERROR] Qualified game control rejects experimental camera yaw sources" >&2
        exit 1
    fi
    if [[ -n "${MATRIX_GAME_INPUT_PYTHON:-}" \
        && "${MATRIX_GAME_INPUT_PYTHON}" != "${MATRIX_SONIC_PYTHON:-}" ]]; then
        echo "[ERROR] Qualified game control requires MATRIX_GAME_INPUT_PYTHON to match the verified runtime Python" >&2
        exit 1
    fi
    GAME_NO_INPUT_PROVIDER_VALUE="${MATRIX_GAME_NO_INPUT_PROVIDER:-0}"
    case "${GAME_NO_INPUT_PROVIDER_VALUE,,}" in
        1|true|yes|on)
            echo "[ERROR] Qualified game control requires the supervised input provider" >&2
            exit 1
            ;;
        0|false|no|off|"") ;;
        *)
            echo "[ERROR] MATRIX_GAME_NO_INPUT_PROVIDER must be a boolean" >&2
            exit 1
            ;;
    esac
fi

sed -i "s/^robot: .*/robot: \"$ROBOTTYPE\"/" src/robot_mujoco/simulate/config.yaml

#######################################
# JSON 同步
#######################################
MUJOCO_RUNNING_JSON=false
if $ENABLE_MUJOCO; then
    MUJOCO_RUNNING_JSON=true
fi

CONFIG_TMP="$(mktemp)"
jq \
    --arg robot_type "$ROBOTTYPE" \
    --arg weapon "$WEAPON" \
    --argjson mujoco_running "$MUJOCO_RUNNING_JSON" \
    '
    .robot = (.robot // {})
    | .robot.robot_type = $robot_type
    | .robot.weapon = $weapon
    | .robot.mujoco_running = $mujoco_running
    | .robot.state_port = (.robot.state_port // 25001)
    | .robot.cmd_port = (.robot.cmd_port // 25002)
    | .robot.EgoView = (.robot.EgoView // true)
    | .robot.position = (.robot.position // {"x": 0, "y": 0, "z": 0})
    ' config/config.json > "$CONFIG_TMP" && mv "$CONFIG_TMP" config/config.json

mkdir -p src/UeSim/Linux/zsibot_mujoco_ue/Content/model/config
mkdir -p src/UeSim/Linux/zsibot_mujoco_ue/Content/model/SceneLoder
cp config/config.json src/UeSim/Linux/zsibot_mujoco_ue/Content/model/config/config.json
cp scene/scene.json  src/UeSim/Linux/zsibot_mujoco_ue/Content/model/SceneLoder/scene.json

#######################################
# UE 场景入口同步
#######################################
# UE 运行时会从固定入口文件读取模型布局：
# - 非 custom 机器人: Content/model/<runtime_robot>/scene_terrain.xml
# - custom 机器人:   Content/model/custom/scene_terrain_custom.xml
# launcher 选中的场景变体需要同步覆盖到该入口，否则 UE 会继续读取默认场景。
compose_custom_runtime_scene() {
    if [[ "$ROBOTTYPE" != "custom" ]]; then
        return
    fi

    local composer="$PROJECT_ROOT/scripts/compose_custom_scene.py"
    local composer_python="${MATRIX_SONIC_PYTHON:-$(command -v python3)}"
    if [[ ! -f "$composer" ]]; then
        echo "[ERROR] Custom scene composer not found: $composer" >&2
        exit 1
    fi

    local mujoco_model_root="$PROJECT_ROOT/src/robot_mujoco/zsibot_robots"
    local ue_model_root="$PROJECT_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Content/model"
    local mujoco_source="$mujoco_model_root/xgb/$SCENE"
    local mujoco_target="$mujoco_model_root/custom/$SCENE"
    local ue_source="$ue_model_root/xgb/$SCENE"
    local ue_target="$ue_model_root/custom/scene_terrain_custom.xml"

    if [[ ! -f "$mujoco_source" ]]; then
        echo "[ERROR] Native MuJoCo scene is unavailable for custom composition: $mujoco_source" >&2
        exit 1
    fi
    if [[ ! -f "$ue_source" ]]; then
        echo "[ERROR] Native UE model scene is unavailable for custom composition: $ue_source" >&2
        exit 1
    fi

    "$composer_python" "$composer" "$mujoco_source" "$mujoco_target"
    "$composer_python" "$composer" "$ue_source" "$ue_target"
    echo "[INFO] Custom robot composed with native scene '$SCENE'"
}

sync_ue_runtime_scene() {
    local ue_model_root="src/UeSim/Linux/zsibot_mujoco_ue/Content/model"

    if [[ "$ROBOTTYPE" == "custom" ]]; then
        local custom_scene_entry="$ue_model_root/custom/scene_terrain_custom.xml"
        if [[ -f "$custom_scene_entry" ]]; then
            echo "[INFO] Custom runtime scene entry ready for '$SCENE': $custom_scene_entry"
        else
            echo "[WARNING] Custom runtime scene entry not found: $custom_scene_entry"
        fi
        return
    fi

    local runtime_dir="$ue_model_root/$RUNTIME_ROBOTTYPE"
    local source_scene="$runtime_dir/$SCENE"
    local target_scene="$runtime_dir/scene_terrain.xml"

    if [[ ! -d "$runtime_dir" ]]; then
        echo "[WARNING] UE runtime model directory not found: $runtime_dir"
        return
    fi
    if [[ ! -f "$source_scene" ]]; then
        echo "[WARNING] UE scene variant not found: $source_scene"
        return
    fi
    if [[ "$source_scene" == "$target_scene" ]]; then
        echo "[INFO] UE runtime scene already points to: $target_scene"
        return
    fi

    cp "$source_scene" "$target_scene"
    echo "[INFO] Synced UE runtime scene: $source_scene -> $target_scene"
}

compose_custom_runtime_scene
sync_ue_runtime_scene

#######################################
# 机器人初始位姿
#######################################
ROBOT_X=$(jq -r '.robot.position.x' config/config.json)
ROBOT_Y=$(jq -r '.robot.position.y' config/config.json)

if [[ "$ROBOTTYPE" == "custom" ]]; then
    CUSTOM_MODEL_DIR="${CUSTOM_NAME:-custom}"
    XML_FILE="src/robot_mujoco/zsibot_robots/custom/_cache/${CUSTOM_MODEL_DIR}/${CUSTOM_MODEL_DIR}.xml"
    if [[ -f "$XML_FILE" ]]; then
        echo "[INFO] Custom robot detected, skipping built-in XML position update for ${XML_FILE}"
    else
        echo "[WARNING] Custom robot XML not found: $XML_FILE"
    fi
else
    XML_FILE="src/robot_mujoco/zsibot_robots/${ROBOTTYPE}/${ROBOTTYPE}.xml"
    sed -i "s/<body name=\"base_link\" pos=\"[^\"]*\"/<body name=\"base_link\" pos=\"${ROBOT_X} ${ROBOT_Y} 0.65\"/" "$XML_FILE"
fi

#######################################
# 启动流程
#######################################
echo "[INFO] Starting processes..."

mkdir -p src/robot_mujoco/simulate/build
cd src/robot_mujoco/simulate/build
if $ENABLE_MUJOCO && ! $MATRIX_SONIC_ENABLED; then
    echo "[INFO] Starting MuJoCo"
    LD_LIBRARY_PATH="$(mujoco_ld_library_path)" ./robot_mujoco > robot_mujoco.log 2>&1 &
    PIDS+=($!)
fi

cd ../../../UeSim/Linux
echo "[INFO] Preparing UE launch"
UE_MOUSE_RELATIVE_SPEED_SCALE="${MATRIX_MOUSE_APPLIED_SPEED_SCALE:-1.0}"
if ! UE_MOUSE_RELATIVE_SPEED_SCALE="$(
    /usr/bin/python3 -I "$PROJECT_ROOT/scripts/matrix_mouse_settings.py" \
        canonical-scale --value "$UE_MOUSE_RELATIVE_SPEED_SCALE"
)"; then
    echo "[ERROR] MATRIX_MOUSE_APPLIED_SPEED_SCALE must use a supported preset" >&2
    exit 1
fi
UE_COMMAND=(
    /usr/bin/env
    "LD_LIBRARY_PATH=$(ue_ld_library_path)"
)
UE_MATERIAL_FIX_PRELOAD="${MATRIX_UE_MATERIAL_FIX_PRELOAD:-}"
UE_MATERIAL_FIX_BINARY=""
UE_G1_SKIN="${MATRIX_G1_SKIN:-}"
UE_G1_MATERIAL_PALETTE="$MATRIX_UE_G1_MATERIAL_PALETTE_CONTRACT"
UE_G1_MATERIAL_SCOPE_ALPHA="$MATRIX_UE_G1_SCOPE_ALPHA_CONTRACT"
UE_G1_PALETTE_PATTERN='^[-0-9eE+.,;]+$'
UE_G1_COMPONENT_PATTERN='^[-0-9eE+.]+$'
if [[ -n "$UE_MATERIAL_FIX_PRELOAD" ]]; then
    if [[ ! "$UE_G1_SKIN" =~ ^[a-z0-9][a-z0-9-]{0,47}$ ]]; then
        echo "[ERROR] MATRIX_G1_SKIN must name a registered skin" >&2
        exit 1
    fi
    if [[ -z "$UE_G1_MATERIAL_PALETTE" \
        || ! "$UE_G1_MATERIAL_PALETTE" =~ $UE_G1_PALETTE_PATTERN ]]; then
        echo "[ERROR] MATRIX_G1_MATERIAL_PALETTE is missing or malformed" >&2
        exit 1
    fi
    if [[ -z "$UE_G1_MATERIAL_SCOPE_ALPHA" \
        || ! "$UE_G1_MATERIAL_SCOPE_ALPHA" =~ $UE_G1_COMPONENT_PATTERN ]]; then
        echo "[ERROR] MATRIX_G1_MATERIAL_SCOPE_ALPHA is missing or malformed" >&2
        exit 1
    fi
    if [[ "$UE_MATERIAL_FIX_PRELOAD" != /* ]]; then
        echo "[ERROR] MATRIX_UE_MATERIAL_FIX_PRELOAD must be absolute" >&2
        exit 1
    fi
    if [[ ! -f "$UE_MATERIAL_FIX_PRELOAD" || -L "$UE_MATERIAL_FIX_PRELOAD" ]]; then
        echo "[ERROR] MATRIX_UE_MATERIAL_FIX_PRELOAD must be a regular non-symlink file:" \
            "$UE_MATERIAL_FIX_PRELOAD" >&2
        exit 1
    fi
    UE_MATERIAL_FIX_PRELOAD="$(realpath -- "$UE_MATERIAL_FIX_PRELOAD")"
    UE_COMMAND+=(
        "LD_PRELOAD=$UE_MATERIAL_FIX_PRELOAD"
        "MATRIX_G1_SKIN=$UE_G1_SKIN"
        "MATRIX_G1_MATERIAL_PALETTE=$UE_G1_MATERIAL_PALETTE"
        "MATRIX_G1_MATERIAL_SCOPE_ALPHA=$UE_G1_MATERIAL_SCOPE_ALPHA"
    )
    for candidate in \
        "$PWD/zsibot_mujoco_ue/Binaries/Linux/zsibot_mujoco_ue-Linux-Shipping" \
        "$PWD/zsibot_mujoco_ue/Binaries/Linux/zsibot_mujoco_ue-Linux-Development" \
        "$PWD/zsibot_mujoco_ue/Binaries/Linux/zsibot_mujoco_ue"
    do
        if [[ -f "$candidate" ]]; then
            UE_MATERIAL_FIX_BINARY="$candidate"
            break
        fi
    done
    if [[ -z "$UE_MATERIAL_FIX_BINARY" ]]; then
        echo "[ERROR] Matrix UE material fix cannot find the packaged executable" >&2
        exit 1
    fi
    echo "[INFO] Matrix UE material fix enabled: $UE_MATERIAL_FIX_PRELOAD"
    echo "[INFO] Matrix UE material skin: $UE_G1_SKIN"
fi
UE_COMMAND+=(
    # Force SDL's raw relative-motion path.  These hints make the behavior
    # explicit across local Xorg and remote nxagent sessions: no warp
    # emulation, viewport scaling, or system pointer acceleration is allowed
    # to reshape a camera drag before UE receives it.
    "SDL_MOUSE_RELATIVE_MODE_WARP=0"
    "SDL_MOUSE_RELATIVE_SCALING=0"
    "SDL_MOUSE_RELATIVE_SPEED_SCALE=$UE_MOUSE_RELATIVE_SPEED_SCALE"
    "SDL_MOUSE_RELATIVE_SYSTEM_SCALE=0"
)
if [[ -n "$UE_MATERIAL_FIX_BINARY" ]]; then
    # LD_PRELOAD must reach only the packaged ELF.  Applying it to the stock
    # shell launcher would load the guarded bridge into bash before exec.
    UE_COMMAND+=("$UE_MATERIAL_FIX_BINARY" zsibot_mujoco_ue)
else
    UE_COMMAND+=(./zsibot_mujoco_ue.sh)
fi
UE_COMMAND+=(
    -game "$MAPNAME"
    # The stock cooked package enables UE's legacy PlayerInput mouse
    # smoothing.  Override it in the Input config hierarchy so a released
    # drag has no interpolated tail; disabling FOV scaling also keeps one
    # physical delta at one stable gain while zoom/FOV changes.
    "-ini:Input:[/Script/Engine.InputSettings]:bEnableMouseSmoothing=False,[/Script/Engine.InputSettings]:bEnableFOVScaling=False"
    "-ExecCmds=$UE_EXEC_CMDS"
)
[[ -n "$USE_OFFSCREEN" ]] && UE_COMMAND+=("$USE_OFFSCREEN")
[[ -n "$USE_PIXELSTREAMER" ]] && UE_COMMAND+=("$USE_PIXELSTREAMER")
if $CENTERED_CAMERA_OVERLAY_ENABLED; then
    install_centered_camera_overlay
fi
configure_remote_pointer_acceleration
UE_LOG="$PWD/zsibot_mujoco_ue.log"
UE_LOG_START_OFFSET=0
if $CENTERED_CAMERA_OVERLAY_ENABLED \
    || [[ -n "$UE_MATERIAL_FIX_PRELOAD" ]]; then
    if [[ -f "$UE_LOG" ]]; then
        UE_LOG_START_OFFSET="$(/usr/bin/stat -c '%s' -- "$UE_LOG")"
    fi
    if [[ ! "$UE_LOG_START_OFFSET" =~ ^[0-9]+$ ]]; then
        echo "[ERROR] Could not record the UE log byte boundary: $UE_LOG" >&2
        exit 1
    fi
fi
echo "[INFO] Starting UE"
start_supervised_ue "$UE_LOG" "${UE_COMMAND[@]}"

UE_STARTUP_SECONDS="${MATRIX_UE_STARTUP_SECONDS:-7}"
if [[ ! "$UE_STARTUP_SECONDS" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    echo "[ERROR] MATRIX_UE_STARTUP_SECONDS must be a non-negative number: $UE_STARTUP_SECONDS" >&2
    exit 1
fi
sleep "$UE_STARTUP_SECONDS"
if [[ -n "$UE_MATERIAL_FIX_PRELOAD" ]]; then
    verify_material_fix_install "$UE_LOG" "$UE_LOG_START_OFFSET"
fi
if $CENTERED_CAMERA_OVERLAY_ENABLED; then
    verify_centered_camera_overlay_mount "$UE_LOG" "$UE_LOG_START_OFFSET"
fi

if $MATRIX_SONIC_ENABLED; then
    MATRIX_SONIC_PYTHON="${MATRIX_SONIC_PYTHON:-python3}"
    MATRIX_SONIC_ROOT="${MATRIX_SONIC_ROOT:-}"
    MATRIX_UNITREE_SDK2_ROOT="${MATRIX_UNITREE_SDK2_ROOT:-}"
    MATRIX_SONIC_CANONICAL_MODEL="${MATRIX_SONIC_CANONICAL_MODEL:-$MATRIX_SONIC_ROOT/gear_sonic/data/robot_model/model_data/g1/g1_29dof_with_hand.xml}"
    MATRIX_SONIC_CANONICAL_MESHES="${MATRIX_SONIC_CANONICAL_MESHES:-$MATRIX_SONIC_ROOT/gear_sonic/data/robot_model/model_data/g1/meshes}"
    GAME_WORLD_PERSISTENCE_ENABLED=0
    case "${MATRIX_GAME_WORLD_PERSISTENCE:-0}" in
        1|true|yes|on) GAME_WORLD_PERSISTENCE_ENABLED=1 ;;
        0|false|no|off|"") ;;
        *)
            echo "[ERROR] MATRIX_GAME_WORLD_PERSISTENCE must be a boolean" >&2
            exit 1
            ;;
    esac
    if [[ "${MATRIX_SONIC_CONTROL_SOURCE:-planner}" == "game" ]]; then
        for required in \
            "$PROJECT_ROOT/scripts/matrix_game_control_input.py" \
            "$PROJECT_ROOT/scripts/matrix_calibration_overlay.py" \
            "$PROJECT_ROOT/scripts/matrix_mc_commands.py" \
            "$PROJECT_ROOT/scripts/matrix_world_state.py" \
            "$PROJECT_ROOT/scripts/prepare_sonic_physics_model.py" \
            "$PROJECT_ROOT/scripts/compose_custom_scene.py"; do
            if [[ ! -f "$required" ]]; then
                echo "[ERROR] Matrix game-control dependency is missing: $required" >&2
                exit 1
            fi
        done
    fi
    for required in \
        "$PROJECT_ROOT/scripts/run_matrix_sonic.py" \
        "$PROJECT_ROOT/scripts/matrix_game_control.py" \
        "$PROJECT_ROOT/scripts/prepare_sonic_physics_model.py" \
        "$MATRIX_SONIC_ROOT/gear_sonic/scripts/run_sim_loop.py" \
        "$MATRIX_SONIC_ROOT/gear_sonic/utils/mujoco_sim/base_sim.py" \
        "$MATRIX_SONIC_ROOT/gear_sonic_deploy/target/release/g1_deploy_onnx_ref" \
        "$MATRIX_SONIC_CANONICAL_MODEL" \
        "$MATRIX_UNITREE_SDK2_ROOT/lib/x86_64/libunitree_sdk2.a"; do
        if [[ ! -f "$required" ]]; then
            echo "[ERROR] Matrix SONIC runtime dependency is missing: $required" >&2
            exit 1
        fi
    done
    if [[ ! -d "$MATRIX_SONIC_CANONICAL_MESHES" ]]; then
        echo "[ERROR] Canonical SONIC G1 meshes are missing: $MATRIX_SONIC_CANONICAL_MESHES" >&2
        exit 1
    fi
    mkdir -p "$PROJECT_ROOT/outputs/logs"
    NATIVE_SONIC_SCENE="$PROJECT_ROOT/src/robot_mujoco/zsibot_robots/xgb/$SCENE"
    SONIC_SPAWN_ARGS=()
    SONIC_WORLD_ARGS=()
    SONIC_SCENE_TRANSFORM_ARGS=()
    SONIC_DYNAMIC_GROUND_ARGS=()
    SONIC_DYNAMIC_GROUND_COLLISION_ARGS=()
    set_sonic_spawn_args() {
        SONIC_SPAWN_ARGS=(
            --spawn-x "$1"
            --spawn-y "$2"
            --spawn-z "$3"
            --spawn-yaw "$4"
        )
    }
    resolve_moon_spawn_args() {
        local source="$1"
        local x="${2:-}"
        local y="${3:-}"
        local z="${4:-}"
        local yaw="${5:-0}"
        local moon_spawn_output
        local -a resolver=(
            "$MATRIX_SONIC_PYTHON" "$PROJECT_ROOT/scripts/matrix_moon_dynamic_ground.py"
            resolve-spawn-pose
            --map "$PROJECT_ROOT/dynamicmaps/moonworld.bin"
            --map-sha256 "62e624b5feca0111033c60d0e820f3a320257acd72b565234ac79c704dbca1df"
            --source "$source"
            --root-clearance "${MATRIX_MOON_DYNAMIC_GROUND_ROOT_CLEARANCE:-0.85}"
            --min-resume-clearance "${MATRIX_MOON_DYNAMIC_GROUND_MIN_RESUME_CLEARANCE:-0.45}"
            --max-resume-clearance "${MATRIX_MOON_DYNAMIC_GROUND_MAX_RESUME_CLEARANCE:-1.30}"
        )
        if [[ -n "$x" && -n "$y" && -n "$z" ]]; then
            resolver+=(--x "$x" --y "$y" --z "$z" --yaw "$yaw")
        fi
        if ! moon_spawn_output="$("${resolver[@]}")"; then
            echo "[ERROR] Could not resolve MoonWorld raw-terrain spawn pose" >&2
            return 1
        fi
        mapfile -t MOON_SPAWN_LINES <<<"$moon_spawn_output"
        if [[ "${MOON_SPAWN_LINES[0]:-}" != "pose" \
            || "${#MOON_SPAWN_LINES[@]}" != "7" ]]; then
            echo "[ERROR] Invalid MoonWorld spawn resolver response" >&2
            return 1
        fi
        set_sonic_spawn_args \
            "${MOON_SPAWN_LINES[1]}" \
            "${MOON_SPAWN_LINES[2]}" \
            "${MOON_SPAWN_LINES[3]}" \
            "${MOON_SPAWN_LINES[4]}"
        echo "[INFO] Matrix MoonWorld spawn pose: ${MOON_SPAWN_LINES[5]} " \
            "x=${MOON_SPAWN_LINES[1]} y=${MOON_SPAWN_LINES[2]} " \
            "z=${MOON_SPAWN_LINES[3]} yaw=${MOON_SPAWN_LINES[4]} " \
            "${MOON_SPAWN_LINES[6]}"
    }
    resolve_default_moon_spawn_args() {
        local -a moon_spawn_override=(
            "${MATRIX_MOON_SPAWN_X:-}"
            "${MATRIX_MOON_SPAWN_Y:-}"
            "${MATRIX_MOON_SPAWN_Z:-}"
            "${MATRIX_MOON_SPAWN_YAW:-}"
        )
        local moon_spawn_override_count=0
        local value
        for value in "${moon_spawn_override[@]}"; do
            [[ -n "$value" ]] && ((moon_spawn_override_count += 1))
        done
        if [[ "$moon_spawn_override_count" == "0" ]]; then
            # Verified MoonWorld plain from the pre-regression mainline.  This
            # point is a locally flat, locked-height patch with real collision;
            # it avoids spawning into the steeper 23,13 test slope or the later
            # 24.43,110.77 route that can destabilize native SONIC.
            resolve_moon_spawn_args \
                "mainline_plain" \
                -94.7 \
                -65.6 \
                -5.251562023162842 \
                0 \
                || return 1
            echo "[INFO] MoonWorld verified mainline plain spawn selected"
            return 0
        fi
        if [[ "$moon_spawn_override_count" == "4" ]]; then
            resolve_moon_spawn_args \
                "explicit" \
                "${moon_spawn_override[0]}" \
                "${moon_spawn_override[1]}" \
                "${moon_spawn_override[2]}" \
                "${moon_spawn_override[3]}" \
                || return 1
            echo "[INFO] MoonWorld explicit spawn aligned by caller"
            return 0
        fi
        echo "[ERROR] MATRIX_MOON_SPAWN_X/Y/Z/YAW are all-or-none" >&2
        return 2
    }
    if [[ "$SCENE" == "scene_terrain_t10.xml" ]]; then
        SONIC_SCENE_TRANSFORM_ARGS+=(
            --scene-transform town10-open-boundary-v1
        )
        echo "[INFO] Town10 perimeter collision walls removed in derived physics scene"
    fi
    if [[ "$SCENE" == "scene_terrain_moon_dynamic.xml" ]]; then
        MOON_DYNAMIC_GROUND_COLLISION_MODE_VALUE="$(
            printf '%s' "${MATRIX_MOON_DYNAMIC_GROUND_COLLISION_MODE:-rolling-mocap-tiles-v1}" \
                | tr '[:upper:]' '[:lower:]' \
                | tr '_' '-'
        )"
        case "$MOON_DYNAMIC_GROUND_COLLISION_MODE_VALUE" in
            ""|stable|default|tiles|tile|mocap-tiles|rolling-tiles|rolling-mocap-tiles|rolling-mocap-tiles-v1|leo|official)
                MOON_DYNAMIC_GROUND_COLLISION_MODE_VALUE="rolling-mocap-tiles-v1"
                ;;
            hfield|heightfield|continuous|continuous-hfield|rolling-hfield|rolling-heightfield|rolling-heightfield-v2)
                MOON_DYNAMIC_GROUND_COLLISION_MODE_VALUE="rolling-heightfield-v2"
                ;;
            *)
                echo "[ERROR] MATRIX_MOON_DYNAMIC_GROUND_COLLISION_MODE must be rolling-heightfield-v2 or rolling-mocap-tiles-v1" >&2
                exit 2
                ;;
        esac
        export MATRIX_MOON_DYNAMIC_GROUND_COLLISION_MODE="$MOON_DYNAMIC_GROUND_COLLISION_MODE_VALUE"
        export MATRIX_MOON_DYNAMIC_GROUND_HEIGHT_FILTER="${MATRIX_MOON_DYNAMIC_GROUND_HEIGHT_FILTER:-raw}"
        SONIC_SCENE_TRANSFORM_ARGS+=(
            --scene-transform moon-dynamic-ground-mocap-v3
        )
        SONIC_DYNAMIC_GROUND_COLLISION_ARGS=(
            --moon-dynamic-ground-collision-mode "$MOON_DYNAMIC_GROUND_COLLISION_MODE_VALUE"
        )
        SONIC_DYNAMIC_GROUND_ARGS=(
            --moon-dynamic-map "$PROJECT_ROOT/dynamicmaps/moonworld.bin"
            --moon-dynamic-map-sha256 "62e624b5feca0111033c60d0e820f3a320257acd72b565234ac79c704dbca1df"
        )
        echo "[INFO] MoonWorld dynamic ground enabled from locked height map: collision=$MOON_DYNAMIC_GROUND_COLLISION_MODE_VALUE height_filter=$MATRIX_MOON_DYNAMIC_GROUND_HEIGHT_FILTER"
    fi
    if [[ "$GAME_WORLD_PERSISTENCE_ENABLED" == "1" ]]; then
        if [[ "${MATRIX_SONIC_CONTROL_SOURCE:-planner}" != "game" ]]; then
            echo "[ERROR] Persistent Matrix world state requires game control" >&2
            exit 1
        fi
        GAME_WORLD_ID="${MATRIX_GAME_WORLD_ID:-${CUSTOM_NAME}:${SCENE%.xml}}"
        GAME_WORLD_REVISION="$(
            "$MATRIX_SONIC_PYTHON" "$PROJECT_ROOT/scripts/matrix_world_state.py" \
                revision \
                --world-id "$GAME_WORLD_ID" \
                --native-scene "$NATIVE_SONIC_SCENE" \
                --canonical-model "$MATRIX_SONIC_CANONICAL_MODEL" \
                --canonical-meshes "$MATRIX_SONIC_CANONICAL_MESHES" \
                "${SONIC_SCENE_TRANSFORM_ARGS[@]}"
        )"
        GAME_WORLD_STATE_FILE="${MATRIX_GAME_WORLD_STATE_FILE:-}"
        if [[ -z "$GAME_WORLD_STATE_FILE" ]]; then
            GAME_WORLD_STATE_FILE="$(
                "$MATRIX_SONIC_PYTHON" "$PROJECT_ROOT/scripts/matrix_world_state.py" \
                    default-path \
                    --profile "${MATRIX_PROFILE:-local}" \
                    --world-id "$GAME_WORLD_ID"
            )"
        fi
        if [[ "$GAME_WORLD_STATE_FILE" != /* ]]; then
            echo "[ERROR] MATRIX_GAME_WORLD_STATE_FILE must be absolute" >&2
            exit 1
        fi
        GAME_WORLD_RESUME_SAFETY_ARGS=()
        if [[ "$SCENE" != "scene_terrain_moon_dynamic.xml" ]]; then
            GAME_WORLD_RESUME_SAFETY_ARGS+=(--min-resume-z 0.55)
        fi
        if ! GAME_WORLD_START_OUTPUT="$(
            "$MATRIX_SONIC_PYTHON" "$PROJECT_ROOT/scripts/matrix_world_state.py" \
                resolve-start \
                --file "$GAME_WORLD_STATE_FILE" \
                --world-id "$GAME_WORLD_ID" \
                --world-revision "$GAME_WORLD_REVISION" \
                "${GAME_WORLD_RESUME_SAFETY_ARGS[@]}"
        )"; then
            echo "[ERROR] Could not resolve the Matrix world resume pose" >&2
            exit 1
        fi
        mapfile -t GAME_WORLD_START_LINES <<<"$GAME_WORLD_START_OUTPUT"
        if [[ "${GAME_WORLD_START_LINES[0]:-}" == "pose" ]]; then
            if [[ "${#GAME_WORLD_START_LINES[@]}" != "7" ]]; then
                echo "[ERROR] Invalid Matrix world-state pose response" >&2
                exit 1
            fi
            if [[ "$SCENE" == "scene_terrain_moon_dynamic.xml" ]]; then
                resolve_moon_spawn_args \
                    "${GAME_WORLD_START_LINES[5]}" \
                    "${GAME_WORLD_START_LINES[1]}" \
                    "${GAME_WORLD_START_LINES[2]}" \
                    "${GAME_WORLD_START_LINES[3]}" \
                    "${GAME_WORLD_START_LINES[4]}" \
                    || exit 1
            else
                set_sonic_spawn_args \
                    "${GAME_WORLD_START_LINES[1]}" \
                    "${GAME_WORLD_START_LINES[2]}" \
                    "${GAME_WORLD_START_LINES[3]}" \
                    "${GAME_WORLD_START_LINES[4]}"
            fi
            echo "[INFO] Matrix resume pose: ${GAME_WORLD_START_LINES[5]} " \
                "world=$GAME_WORLD_ID state=${GAME_WORLD_START_LINES[6]}"
        elif [[ "${GAME_WORLD_START_LINES[0]:-}" == "none" \
            && "${#GAME_WORLD_START_LINES[@]}" == "2" ]]; then
            if [[ "$SCENE" == "scene_terrain_moon_dynamic.xml" ]]; then
                resolve_default_moon_spawn_args || exit 1
            fi
            echo "[INFO] Matrix resume pose: map default " \
                "world=$GAME_WORLD_ID state=${GAME_WORLD_START_LINES[1]}"
        else
            echo "[ERROR] Invalid Matrix world-state helper response" >&2
            exit 1
        fi
        SONIC_WORLD_ARGS=(
            --game-world-id "$GAME_WORLD_ID"
            --game-world-revision "$GAME_WORLD_REVISION"
            --game-world-state-file "$GAME_WORLD_STATE_FILE"
            --game-world-checkpoint-seconds "${MATRIX_GAME_WORLD_CHECKPOINT_SECONDS:-0.75}"
        )
        case "${MATRIX_GAME_AUTO_RESPAWN:-0}" in
            1|true|yes|on) SONIC_WORLD_ARGS+=(--game-auto-respawn) ;;
            0|false|no|off|"") ;;
            *)
                echo "[ERROR] MATRIX_GAME_AUTO_RESPAWN must be a boolean" >&2
                exit 1
                ;;
        esac
    fi
    if [[ "$SCENE" == "scene_terrain_moon_dynamic.xml" \
        && "${#SONIC_SPAWN_ARGS[@]}" == "0" ]]; then
        resolve_default_moon_spawn_args || exit 1
    fi
    SONIC_PHYSICS_DIR="${MATRIX_SONIC_PHYSICS_DIR:-$PROJECT_ROOT/outputs/runtime/matrix_sonic/$CUSTOM_NAME/${SCENE%.xml}}"
    "$MATRIX_SONIC_PYTHON" "$PROJECT_ROOT/scripts/prepare_sonic_physics_model.py" \
        --canonical-model "$MATRIX_SONIC_CANONICAL_MODEL" \
        --canonical-meshes "$MATRIX_SONIC_CANONICAL_MESHES" \
        --native-scene "$NATIVE_SONIC_SCENE" \
        --output-dir "$SONIC_PHYSICS_DIR" \
        "${SONIC_SPAWN_ARGS[@]}" \
        "${SONIC_SCENE_TRANSFORM_ARGS[@]}" \
        "${SONIC_DYNAMIC_GROUND_COLLISION_ARGS[@]}"
    SONIC_STATUS_FILE="${MATRIX_SONIC_STATUS_FILE:-$PROJECT_ROOT/outputs/matrix_sonic_status.json}"
    rm -f -- "$SONIC_STATUS_FILE"
    GAME_INPUT_STATUS_FILE="${MATRIX_GAME_INPUT_STATUS_FILE:-$PROJECT_ROOT/outputs/matrix_game_control_input.json}"
    if [[ "${MATRIX_SONIC_CONTROL_SOURCE:-planner}" == "game" ]]; then
        rm -f -- "$GAME_INPUT_STATUS_FILE"
    fi
    SONIC_STARTUP_ARGS=()
    SONIC_STARTUP_BAND_VALUE="${MATRIX_SONIC_STARTUP_BAND:-1}"
    case "${SONIC_STARTUP_BAND_VALUE,,}" in
        1|true|yes|on) SONIC_STARTUP_ARGS+=(--startup-band) ;;
        0|false|no|off|"") ;;
        *)
            echo "[ERROR] MATRIX_SONIC_STARTUP_BAND must be a boolean" >&2
            exit 1
            ;;
    esac
    SONIC_ACCEPTANCE_ARGS=()
    SONIC_FAIL_ON_FALL_DEFAULT=1
    SONIC_MAX_RESETS_DEFAULT=0
    if [[ "${MATRIX_SONIC_CONTROL_SOURCE:-planner}" == "game" \
        && "${MATRIX_SONIC_MAX_SECONDS:-0}" == "0" ]]; then
        SONIC_FAIL_ON_FALL_DEFAULT=0
        SONIC_MAX_RESETS_DEFAULT=100000
    fi
    case "${MATRIX_SONIC_FAIL_ON_FALL:-$SONIC_FAIL_ON_FALL_DEFAULT}" in
        1|true|yes|on) SONIC_ACCEPTANCE_ARGS+=(--fail-on-fall) ;;
        0|false|no|off|"") ;;
        *)
            echo "[ERROR] MATRIX_SONIC_FAIL_ON_FALL must be a boolean" >&2
            exit 1
            ;;
    esac
    if [[ "${MATRIX_SONIC_MIN_ACTIVE_SECONDS:-0}" != "0" ]]; then
        SONIC_ACCEPTANCE_ARGS+=(--min-active-seconds "${MATRIX_SONIC_MIN_ACTIVE_SECONDS}")
    fi
    if [[ "${MATRIX_SONIC_MIN_DISPLACEMENT_M:-0}" != "0" ]]; then
        SONIC_ACCEPTANCE_ARGS+=(--min-displacement-m "${MATRIX_SONIC_MIN_DISPLACEMENT_M}")
    fi
    SONIC_QUALIFICATION_ARGS=()
    if [[ "${MATRIX_SONIC_QUALIFIED_RUNTIME:-0}" == "1" ]]; then
        SONIC_QUALIFICATION_ARGS+=(
            --qualified-runtime
            --qualification-profile "${MATRIX_SONIC_QUALIFICATION_PROFILE}"
            --runtime-lock-sha256 "${MATRIX_SONIC_RUNTIME_LOCK_SHA256}"
            --matrix-commit "${MATRIX_SONIC_MATRIX_COMMIT}"
            --verification-receipt "${MATRIX_SONIC_VERIFICATION_RECEIPT}"
        )
    fi
    echo "[INFO] Starting native gear_sonic MuJoCo/DDS runtime"
    GAME_INPUT_PROVIDER_PYTHON="${MATRIX_GAME_INPUT_PYTHON:-$MATRIX_SONIC_PYTHON}"
    if [[ "${MATRIX_SONIC_QUALIFIED_RUNTIME:-0}" == "1" \
        && "${MATRIX_SONIC_CONTROL_SOURCE:-planner}" == "game" ]]; then
        GAME_INPUT_PROVIDER_PYTHON="$MATRIX_SONIC_PYTHON"
    fi
    GAME_INPUT_ARGS=(
        --game-input-socket "${MATRIX_GAME_INPUT_SOCKET:-${XDG_RUNTIME_DIR:-/tmp}/matrix-game-control-${UID}-${MATRIX_SONIC_LAUNCHER_PID:-$$}.sock}"
        --game-input-provider "$PROJECT_ROOT/scripts/matrix_game_control_input.py"
        --game-input-provider-python "$GAME_INPUT_PROVIDER_PYTHON"
        --game-input-source "${MATRIX_GAME_INPUT_SOURCE:-auto}"
        --game-camera-yaw-source "${MATRIX_GAME_CAMERA_YAW_SOURCE:-fixed}"
        --game-look-button "${MATRIX_GAME_LOOK_BUTTON:-left}"
        --game-initial-camera-yaw-deg "${MATRIX_GAME_INITIAL_CAMERA_YAW_DEG:-0.0}"
        --game-mouse-sensitivity-deg "${MATRIX_GAME_MOUSE_SENSITIVITY_DEG:-0.12}"
        --game-mouse-settings-file "${MATRIX_MOUSE_SETTINGS_FILE:-${XDG_CONFIG_HOME:-$HOME/.config}/matrix/mouse-control.json}"
        --game-ui-settings-file "${MATRIX_UI_SETTINGS_FILE:-${XDG_CONFIG_HOME:-$HOME/.config}/matrix/hosts/${MATRIX_SETTINGS_PROFILE:-local}/ui-settings.json}"
        --game-motion-settings-file "${MATRIX_MOTION_SETTINGS_FILE:-${XDG_CONFIG_HOME:-$HOME/.config}/matrix/hosts/${MATRIX_SETTINGS_PROFILE:-local}/motion-control.json}"
        --game-video-settings-file "${MATRIX_VIDEO_SETTINGS_FILE:-${XDG_CONFIG_HOME:-$HOME/.config}/matrix/hosts/${MATRIX_SETTINGS_PROFILE:-local}/video-settings.json}"
        --game-function-directory "${MATRIX_GAME_FUNCTION_DIR:-${XDG_CONFIG_HOME:-$HOME/.config}/matrix/hosts/${MATRIX_SETTINGS_PROFILE:-local}/functions}"
        --game-applied-video-settings-json "${MATRIX_GAME_APPLIED_VIDEO_SETTINGS_JSON:-}"
        --game-applied-mouse-profile "${MATRIX_MOUSE_APPLIED_PROFILE:-local}"
        --game-applied-mouse-speed-scale "${MATRIX_MOUSE_APPLIED_SPEED_SCALE:-1.0}"
        --game-keyboard-camera-look-rate-deg-s "${MATRIX_GAME_KEYBOARD_CAMERA_LOOK_RATE_DEG_S:-120.0}"
        --game-camera-yaw-sign "${MATRIX_GAME_CAMERA_YAW_SIGN:--1}"
        --game-camera-yaw-offset-deg "${MATRIX_GAME_CAMERA_YAW_OFFSET_DEG:-0.0}"
        --game-carla-host "${MATRIX_GAME_CARLA_HOST:-127.0.0.1}"
        --game-carla-port "${MATRIX_GAME_CARLA_PORT:-2000}"
        --gamepad-look-yaw-rate-deg-s "${MATRIX_GAMEPAD_LOOK_YAW_RATE_DEG_S:-120.0}"
        --gamepad-look-pitch-rate-deg-s "${MATRIX_GAMEPAD_LOOK_PITCH_RATE_DEG_S:-90.0}"
        --gamepad-look-deadzone "${MATRIX_GAMEPAD_LOOK_DEADZONE:-0.12}"
        --gamepad-look-min-pitch-deg "${MATRIX_GAMEPAD_LOOK_MIN_PITCH_DEG:--80.0}"
        --gamepad-look-max-pitch-deg "${MATRIX_GAMEPAD_LOOK_MAX_PITCH_DEG:-60.0}"
        --game-focus-title "${MATRIX_GAME_FOCUS_TITLE:-(zsibot|matrix|unreal)}"
        --game-input-status-file "$GAME_INPUT_STATUS_FILE"
        --game-max-speed "${MATRIX_GAME_MAX_SPEED:-0.30}"
        --game-max-acceleration "${MATRIX_GAME_MAX_ACCELERATION:-1.20}"
        --game-max-deceleration "${MATRIX_GAME_MAX_DECELERATION:-2.40}"
        --game-max-turn-rate "${MATRIX_GAME_MAX_TURN_RATE:-2.50}"
        --game-stick-deadzone "${MATRIX_GAME_STICK_DEADZONE:-0.15}"
        --game-input-timeout "${MATRIX_GAME_INPUT_TIMEOUT:-0.15}"
        --game-max-snapshot-age "${MATRIX_GAME_MAX_SNAPSHOT_AGE:-0.15}"
        --game-max-future-skew "${MATRIX_GAME_MAX_FUTURE_SKEW:-0.05}"
    )
    if [[ "${MATRIX_GAME_GRAB_UI_KEYS:-1}" == "1" ]]; then
        GAME_INPUT_ARGS+=(--game-grab-ui-keys)
    fi
    if [[ "${MATRIX_GAME_CAMERA_YAW_SOURCE:-fixed}" == "ue-final-pov" ]]; then
        if [[ -z "$UE_CAMERA_STATE_FILE" ]]; then
            echo "[ERROR] UE final-POV state file was not initialized" >&2
            exit 1
        fi
        GAME_INPUT_ARGS+=(
            --game-ue-camera-state-file "$UE_CAMERA_STATE_FILE"
        )
    fi
    if [[ -n "${MATRIX_GAME_RESTART_REQUEST_FILE:-}" \
        && -n "${MATRIX_GAME_RESTART_CAPABILITY_FILE:-}" \
        && -n "${MATRIX_SONIC_LAUNCHER_PID:-}" ]]; then
        GAME_INPUT_ARGS+=(
            --game-restart-request-file "$MATRIX_GAME_RESTART_REQUEST_FILE"
            --game-restart-capability-file "$MATRIX_GAME_RESTART_CAPABILITY_FILE"
            --game-restart-launcher-pid "$MATRIX_SONIC_LAUNCHER_PID"
        )
    fi
    GAME_NO_INPUT_PROVIDER_VALUE="${MATRIX_GAME_NO_INPUT_PROVIDER:-0}"
    case "${GAME_NO_INPUT_PROVIDER_VALUE,,}" in
        1|true|yes|on) GAME_INPUT_ARGS+=(--no-game-input-provider) ;;
        0|false|no|off|"") ;;
        *)
            echo "[ERROR] MATRIX_GAME_NO_INPUT_PROVIDER must be a boolean" >&2
            exit 1
            ;;
    esac
    "$MATRIX_SONIC_PYTHON" "$PROJECT_ROOT/scripts/run_matrix_sonic.py" \
        --model "$SONIC_PHYSICS_DIR/$SCENE" \
        --sonic-root "$MATRIX_SONIC_ROOT" \
        --control-source "${MATRIX_SONIC_CONTROL_SOURCE:-planner}" \
        --planner-bind "${MATRIX_SONIC_PLANNER_BIND:-tcp://127.0.0.1:5556}" \
        --pico-python "${MATRIX_PICO_PYTHON:-$MATRIX_SONIC_PYTHON}" \
        --expected-parent-pid "$$" \
        --external-failure-file "$UE_FAILURE_FILE" \
        --ue-pid "$UE_PID" \
        --physics-hz "${MATRIX_SONIC_PHYSICS_HZ:-200}" \
        --walk-after "${MATRIX_SONIC_WALK_AFTER:--1}" \
        --vx "${MATRIX_SONIC_VX:-0.30}" \
        --vy "${MATRIX_SONIC_VY:-0.0}" \
        --yaw-rate "${MATRIX_SONIC_YAW_RATE:-0.0}" \
        --max-seconds "${MATRIX_SONIC_MAX_SECONDS:-0}" \
        --low-cmd-fresh-timeout-seconds "${MATRIX_SONIC_LOW_CMD_FRESH_TIMEOUT_SECONDS:-0.1}" \
        --min-physics-hz "${MATRIX_SONIC_MIN_PHYSICS_HZ:-195}" \
        --min-rtf "${MATRIX_SONIC_MIN_RTF:-0.95}" \
        --max-resets "${MATRIX_SONIC_MAX_RESETS:-$SONIC_MAX_RESETS_DEFAULT}" \
        "${SONIC_ACCEPTANCE_ARGS[@]}" \
        "${SONIC_QUALIFICATION_ARGS[@]}" \
        "${SONIC_STARTUP_ARGS[@]}" \
        --startup-band-hold "${MATRIX_SONIC_STARTUP_BAND_HOLD:-4}" \
        --startup-band-fade "${MATRIX_SONIC_STARTUP_BAND_FADE:-3}" \
        "${GAME_INPUT_ARGS[@]}" \
        "${SONIC_WORLD_ARGS[@]}" \
        "${SONIC_DYNAMIC_GROUND_ARGS[@]}" \
        --status-file "$SONIC_STATUS_FILE" \
        > "$PROJECT_ROOT/outputs/logs/matrix_sonic_runtime.log" 2>&1 &
    SONIC_PID=$!
    PIDS+=("$SONIC_PID")
fi

cd ../../robot_mc
if $ENABLE_MC; then
    echo "[INFO] Starting MC"
    export SDK_CLIENT_IP="${SDK_CLIENT_IP:-127.0.0.1}"
    ROAMERX_STATE_FILE="${PROJECT_ROOT}/bin/roamerx_link.state"
    if [[ -f "${ROAMERX_STATE_FILE}" ]]; then
        ROAMERX_TARGET_IP="${SDK_CLIENT_IP}"
        SDK_CONFIG_FILE="${PWD}/build/export/config/sdk_config.yaml"
        if [[ -f "${SDK_CONFIG_FILE}" ]]; then
            sed -i "s/^target_ip: .*/target_ip: \"${ROAMERX_TARGET_IP}\"/" "${SDK_CONFIG_FILE}"
        fi
        echo "[INFO] RoamerX link detected, starting MC with UDP target ${ROAMERX_TARGET_IP}:43988 and highlevel port 43997"
        LD_LIBRARY_PATH="$(mc_ld_library_path)" ./run_mc.sh r 25001 25002 43988 43997 25005 > run_mc.log 2>&1 &
    else
        LD_LIBRARY_PATH="$(mc_ld_library_path)" ./run_mc.sh r mc_enable=true > run_mc.log 2>&1 &
    fi
    PIDS+=($!)
fi

# echo "[INFO] Starting ROS2 pub_tf.launch.py"
# ros2 launch pub_tf pub_tf.launch.py tf_type:=mujoco_tf > pub_tf.log 2>&1 &
# PIDS+=($!)

#######################################
# 阻塞等待
#######################################
echo "[INFO] All components started."
if [[ -n "$SONIC_PID" ]]; then
    if ((BASH_VERSINFO[0] < 5)) \
        || ((BASH_VERSINFO[0] == 5 && BASH_VERSINFO[1] < 1)); then
        echo "[ERROR] Matrix SONIC supervision requires Bash 5.1 or newer" >&2
        exit 2
    fi
    set +e
    COMPLETED_PID=""
    wait -n -p COMPLETED_PID "$SONIC_PID" "$UE_SUPERVISOR_PID"
    FIRST_EXIT_CODE=$?
    if [[ "$COMPLETED_PID" == "$UE_SUPERVISOR_PID" ]]; then
        UE_SUPERVISOR_REAPED=1
        record_ue_supervisor_failure
        # Do not signal a numeric PID after wait-n reaped the other child: Bash
        # may already have reaped a near-simultaneous SONIC exit and that PID can
        # be reused. The runner polls this sentinel and exits fail-closed.
        wait "$SONIC_PID"
        SONIC_EXIT_CODE=$?
    else
        SONIC_EXIT_CODE="$FIRST_EXIT_CODE"
    fi
    remove_managed_pid "$SONIC_PID"
    SONIC_PID=""
    set -e
    # The supervisor stays alive after any unexpected UE exit. Asking it to stop
    # and waiting for that exact child is the synchronization barrier between
    # the runner's final poll and the authoritative UE wait status.
    stop_supervised_ue
    if [[ -e "$UE_FAILURE_FILE" ]]; then
        if ! PYTHONPATH="$PROJECT_ROOT/scripts${PYTHONPATH:+:$PYTHONPATH}" \
            "$MATRIX_SONIC_PYTHON" - \
            "$SONIC_STATUS_FILE" "$UE_FAILURE_FILE" <<'PY'
import sys
from pathlib import Path

from run_matrix_sonic import (
    _read_external_failure,
    _record_external_child_failure,
)

failure = _read_external_failure(Path(sys.argv[2]))
if failure is None:
    raise RuntimeError("missing UE failure")
_record_external_child_failure(Path(sys.argv[1]), failure)
PY
        then
            echo "[ERROR] Failed to merge the UE lifecycle failure into status" >&2
        fi
        # Exit 75 is authority only for a clean, status-verified world reload.
        # A UE failure observed at this late boundary must invalidate it just as
        # it invalidates an otherwise-successful zero exit; otherwise the outer
        # launcher can mistake a concurrent UE crash for an authorized teleport
        # or fall respawn.
        if [[ "$SONIC_EXIT_CODE" == "0" || "$SONIC_EXIT_CODE" == "75" ]]; then
            SONIC_EXIT_CODE=2
        fi
    fi
    echo "[INFO] Matrix SONIC runtime exited with code $SONIC_EXIT_CODE"
    exit "$SONIC_EXIT_CODE"
fi
wait
