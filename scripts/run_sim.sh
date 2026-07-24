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
MATRIX_EXTERNAL_REPLAY="${MATRIX_EXTERNAL_REPLAY:-0}"
MATRIX_EXTERNAL_REPLAY_CENTERED_CAMERA="${MATRIX_EXTERNAL_REPLAY_CENTERED_CAMERA:-0}"
MATRIX_GAME_CENTERED_CAMERA="${MATRIX_GAME_CENTERED_CAMERA:-1}"
MATRIX_GAME_CAMERA_VIEW_CLASS="${MATRIX_GAME_CAMERA_VIEW_CLASS:-}"
MATRIX_CENTERED_CAMERA_OVERLAY_CONTRACT="${MATRIX_CENTERED_CAMERA_OVERLAY_CONTRACT:-$PROJECT_ROOT/config/runtime/matrix-centered-camera-overlay-v3.json}"
MATRIX_CENTERED_CAMERA_OVERLAY_BUNDLE="${MATRIX_CENTERED_CAMERA_OVERLAY_BUNDLE:-}"
MATRIX_UE_CAMERA_LAYOUT="${MATRIX_UE_CAMERA_LAYOUT:-$PROJECT_ROOT/config/runtime/matrix-ue-camera-layout-v1.json}"
CENTERED_CAMERA_OVERLAY_STEM="pakchunk99-MatrixCentered-Linux_P"
MATRIX_GAME_CAMERA_DISTANCE_CM="${MATRIX_GAME_CAMERA_DISTANCE_CM:-150}"

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
    case "${MATRIX_EXTERNAL_REPLAY,,}" in
        1|true|yes|on)
            # Trace replay owns only the loopback render-state publisher.  It
            # does not use the legacy ROS/MuJoCo process environment.
            return
            ;;
    esac
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
    case "${MATRIX_EXTERNAL_REPLAY,,}" in
        1|true|yes|on)
            # The accepted trace was executed by TwinBot's offline MuJoCo
            # world. Matrix only launches UE and replays recorded state.
            checked_mujoco=0
            ;;
    esac
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
TRACE_REPLAY_PID=""
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

wait_for_ue_map_ready() {
    local ue_log="$1"
    local start_offset="$2"
    local map_name="$3"
    local timeout="${MATRIX_UE_MAP_READY_TIMEOUT_SECONDS:-120}"
    if [[ ! "$timeout" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
        echo "[ERROR] MATRIX_UE_MAP_READY_TIMEOUT_SECONDS must be positive" >&2
        return 1
    fi
    local attempts
    attempts="$(/usr/bin/python3 -I - "$timeout" <<'PY'
import math
import sys

timeout = float(sys.argv[1])
if not math.isfinite(timeout) or timeout <= 0.0:
    raise SystemExit("map-ready timeout must be positive and finite")
print(max(1, math.ceil(timeout / 0.1) + 1))
PY
)" || return 1
    local attempt
    for ((attempt = 0; attempt < attempts; attempt++)); do
        if [[ -e "$UE_FAILURE_FILE" ]] || ! kill -0 "$UE_PID" 2>/dev/null; then
            echo "[ERROR] UE exited before its current-run map-ready marker" >&2
            return 1
        fi
        local status
        status="$(/usr/bin/python3 -I - \
            "$ue_log" "$start_offset" "$map_name" "$UE_PID" <<'PY'
from pathlib import Path
import re
import sys

path = Path(sys.argv[1])
offset = int(sys.argv[2])
marker = f"LogGlobalStatus: LoadMap Load map complete {sys.argv[3]}"
ue_pid = int(sys.argv[4])
if not path.is_file():
    print("missing-log")
    raise SystemExit(0)
size = path.stat().st_size
if size < offset:
    print("truncated")
    raise SystemExit(0)
with path.open("rb") as stream:
    stream.seek(offset)
    current_run = stream.read().decode("utf-8", errors="replace")
map_ready = marker in current_run
model_ready = (
    "[MuJoCoSimulationRender] "
    "模型加载成功，开始初始化传感器/网格/线程"
) in current_run
model_failed = "[MuJoCoSimulationRender][ERROR]" in current_run

if model_failed:
    print("model-failed")
    raise SystemExit(0)

socket_inodes = set()
for protocol in ("udp", "udp6"):
    try:
        lines = Path(f"/proc/net/{protocol}").read_text(
            encoding="ascii", errors="strict"
        ).splitlines()[1:]
    except OSError:
        continue
    for line in lines:
        fields = line.split()
        if len(fields) < 10:
            continue
        try:
            port = int(fields[1].rsplit(":", 1)[1], 16)
        except (IndexError, ValueError):
            continue
        if port == 9999:
            socket_inodes.add(fields[9])
try:
    descriptors = list(Path(f"/proc/{ue_pid}/fd").iterdir())
except OSError:
    print("udp-unreadable")
    raise SystemExit(0)
owned_socket_inodes = set()
for descriptor in descriptors:
    try:
        target = descriptor.readlink().as_posix()
    except OSError:
        continue
    match = re.fullmatch(r"socket:\[(\d+)\]", target)
    if match is not None:
        owned_socket_inodes.add(match.group(1))
udp_ready = bool(socket_inodes & owned_socket_inodes)
if map_ready and udp_ready and model_ready:
    print("ready")
elif map_ready and udp_ready:
    print("model-wait")
elif map_ready:
    print("map-ready-udp-wait")
elif udp_ready:
    print("udp-ready-map-wait")
else:
    print("waiting")
PY
)" || return 1
        case "$status" in
            ready)
                echo "[INFO] Verified current-run UE map ready: $map_name"
                return 0
                ;;
            missing-log|waiting|model-wait|map-ready-udp-wait|udp-ready-map-wait)
                ;;
            model-failed)
                echo "[ERROR] UE reported a current-run MuJoCo model-load failure:" \
                    "$ue_log" >&2
                return 1
                ;;
            udp-unreadable)
                echo "[ERROR] Could not verify that UE owns UDP receiver 9999" >&2
                return 1
                ;;
            truncated)
                echo "[ERROR] UE log truncated after current-run byte boundary:" \
                    "$ue_log" >&2
                return 1
                ;;
            *)
                echo "[ERROR] Invalid UE map-ready verifier status: $status" >&2
                return 1
                ;;
        esac
        sleep 0.1
    done
    echo "[ERROR] UE did not become map+UDP ready within ${timeout}s:" \
        "$map_name / UDP 9999" >&2
    return 1
}

wait_for_external_camera_ready() {
    local ready_file="$1"
    local camera_mode="$2"
    if [[ -z "$ready_file" ]]; then
        return 0
    fi
    local timeout="${MATRIX_EXTERNAL_REPLAY_CAMERA_READY_TIMEOUT_SECONDS:-120}"
    local settle="${MATRIX_EXTERNAL_REPLAY_CAMERA_SETTLE_SECONDS:-0.5}"
    if [[ ! "$timeout" =~ ^[0-9]+([.][0-9]+)?$ ]] \
        || [[ ! "$settle" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
        echo "[ERROR] Camera-ready timeout/settle must be non-negative numbers" >&2
        return 1
    fi
    local attempts
    attempts="$(/usr/bin/python3 -I - "$timeout" <<'PY'
import math
import sys

timeout = float(sys.argv[1])
if not math.isfinite(timeout) or timeout <= 0.0:
    raise SystemExit("camera-ready timeout must be positive and finite")
print(max(1, math.ceil(timeout / 0.05) + 1))
PY
)" || return 1
    echo "[INFO] Waiting for Matrix scene6 camera confirmation: $ready_file"
    local attempt
    for ((attempt = 0; attempt < attempts; attempt++)); do
        if [[ -e "$UE_FAILURE_FILE" ]] || ! kill -0 "$UE_PID" 2>/dev/null; then
            echo "[ERROR] UE exited before camera confirmation" >&2
            return 1
        fi
        if [[ -L "$ready_file" || -d "$ready_file" ]]; then
            echo "[ERROR] Camera-ready path must be a regular non-symlink file:" \
                "$ready_file" >&2
            return 1
        fi
        if [[ -f "$ready_file" ]]; then
            if ! /usr/bin/python3 -I \
                "$PROJECT_ROOT/scripts/matrix_scene6_camera_receipt.py" \
                inspect-ready --file "$ready_file" --mode "$camera_mode" \
                >/dev/null; then
                echo "[ERROR] Camera-ready confirmation is invalid: $ready_file" >&2
                return 1
            fi
            sleep "$settle"
            echo "[INFO] Matrix scene6 camera confirmation accepted:" \
                "mode=$camera_mode settle=${settle}s"
            return 0
        fi
        sleep 0.05
    done
    echo "[ERROR] Matrix scene6 camera confirmation timed out after ${timeout}s" >&2
    return 1
}

path_is_equal_or_within() {
    local candidate="$1"
    local directory="$2"
    [[ "$candidate" == "$directory" || "$candidate" == "$directory/"* ]]
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
    finalize_exit "$exit_code"
}

finalize_exit() {
    local incoming_exit="$1"
    local cleanup_exit=0
    trap - EXIT
    trap '' SIGINT SIGTERM SIGHUP
    cleanup || cleanup_exit=$?
    if [[ "$incoming_exit" == "0" && "$cleanup_exit" != "0" ]]; then
        incoming_exit="$cleanup_exit"
    fi
    exit "$incoming_exit"
}

trap 'finalize_exit "$?"' EXIT
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
        sed -i 's/export ROBOT_TYPE=.*/export ROBOT_TYPE=XG/' "$TARGET_FILE"
        if [[ "$MUJOCORUNNING" == "1" ]]; then
            ENABLE_MUJOCO=true
            sed -i 's/motor_platform_type: .*/motor_platform_type: 5/' src/robot_mc/build/export/config/xg-user-parameters.yaml
        else
            ENABLE_MUJOCO=false
            sed -i 's/motor_platform_type: .*/motor_platform_type: 8/' src/robot_mc/build/export/config/xg-user-parameters.yaml
        fi
        ;;
    2|xgw)
        ROBOTTYPE="xgw"
        RUNTIME_ROBOTTYPE="xgw"
        ENABLE_MC=true
        sed -i 's/export ROBOT_TYPE=.*/export ROBOT_TYPE=XGW/' "$TARGET_FILE"
        if [[ "$MUJOCORUNNING" == "1" ]]; then
            ENABLE_MUJOCO=true
            sed -i 's/motor_platform_type: .*/motor_platform_type: 5/' src/robot_mc/build/export/config/xg_wheel-user-parameters.yaml
        else
            ENABLE_MUJOCO=false
            sed -i 's/motor_platform_type: .*/motor_platform_type: 8/' src/robot_mc/build/export/config/xg_wheel-user-parameters.yaml
        fi
        ;;
    3|zgws)
        ROBOTTYPE="zgws"
        RUNTIME_ROBOTTYPE="zgws"
        ENABLE_MC=true
        sed -i 's/export ROBOT_TYPE=.*/export ROBOT_TYPE=ZGWS/' "$TARGET_FILE"
        if [[ "$MUJOCORUNNING" == "1" ]]; then
            ENABLE_MUJOCO=true
            sed -i 's/motor_platform_type: .*/motor_platform_type: 5/' src/robot_mc/build/export/config/zg_wheels-user-parameters.yaml
        else
            ENABLE_MUJOCO=false
            sed -i 's/motor_platform_type: .*/motor_platform_type: 8/' src/robot_mc/build/export/config/zg_wheels-user-parameters.yaml
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
            case "${MATRIX_EXTERNAL_REPLAY,,}" in
                1|true|yes|on)
                    _REF_PROFILE="$(/usr/bin/python3 -I - "$_MANIFEST" <<'PY' 2>/dev/null || true
import json
import sys

value = json.load(open(sys.argv[1], encoding="utf-8")).get("reference_profile")
print(value if isinstance(value, str) else "")
PY
)"
                    ;;
                *)
                    _REF_PROFILE="$(jq -r '.reference_profile // empty' "$_MANIFEST" 2>/dev/null || true)"
                    ;;
            esac
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
                sed -i 's/export ROBOT_TYPE=.*/export ROBOT_TYPE=XGW/' "$TARGET_FILE"
                if [[ "$MUJOCORUNNING" == "1" ]]; then
                    ENABLE_MUJOCO=true
                    sed -i 's/motor_platform_type: .*/motor_platform_type: 5/' src/robot_mc/build/export/config/xg_wheel-user-parameters.yaml
                else
                    ENABLE_MUJOCO=false
                    sed -i 's/motor_platform_type: .*/motor_platform_type: 8/' src/robot_mc/build/export/config/xg_wheel-user-parameters.yaml
                fi
                ;;
            xxg)
                # XXG family → XXG MC config
                sed -i 's/export ROBOT_TYPE=.*/export ROBOT_TYPE=XXG/' "$TARGET_FILE"
                if [[ "$MUJOCORUNNING" == "1" ]]; then
                    ENABLE_MUJOCO=true
                    sed -i 's/motor_platform_type: .*/motor_platform_type: 5/' src/robot_mc/build/export/config/xxg-user-parameters.yaml
                else
                    ENABLE_MUJOCO=false
                    sed -i 's/motor_platform_type: .*/motor_platform_type: 8/' src/robot_mc/build/export/config/xxg-user-parameters.yaml
                fi
                ;;
            *)
                # xgb / generic / unknown → XG MC config (default)
                sed -i 's/export ROBOT_TYPE=.*/export ROBOT_TYPE=XG/' "$TARGET_FILE"
                if [[ "$MUJOCORUNNING" == "1" ]]; then
                    ENABLE_MUJOCO=true
                    sed -i 's/motor_platform_type: .*/motor_platform_type: 5/' src/robot_mc/build/export/config/xg-user-parameters.yaml
                else
                    ENABLE_MUJOCO=false
                    sed -i 's/motor_platform_type: .*/motor_platform_type: 8/' src/robot_mc/build/export/config/xg-user-parameters.yaml
                fi
                ;;
        esac
        ;;
    *)
        echo "[ERROR] Unknown robot type: $ROBOT_ARG"
        exit 1
        ;;
esac

case "${MATRIX_DISABLE_MC,,}" in
    1|true|yes|on)
        ENABLE_MC=false
        echo "[INFO] Matrix motion controller disabled by MATRIX_DISABLE_MC=$MATRIX_DISABLE_MC"
        ;;
    0|false|no|off|"")
        ;;
    *)
        echo "[ERROR] MATRIX_DISABLE_MC must be a boolean: $MATRIX_DISABLE_MC" >&2
        exit 1
        ;;
esac

case "${MATRIX_EXTERNAL_REPLAY,,}" in
    1|true|yes|on)
        MATRIX_EXTERNAL_REPLAY_ENABLED=true
        ENABLE_MC=false
        if ! $ENABLE_MUJOCO; then
            echo "[ERROR] MATRIX_EXTERNAL_REPLAY requires MuJoCo render mode" >&2
            exit 1
        fi
        echo "[INFO] External Matrix UE physics-trace replay enabled"
        ;;
    0|false|no|off|"")
        MATRIX_EXTERNAL_REPLAY_ENABLED=false
        ;;
    *)
        echo "[ERROR] MATRIX_EXTERNAL_REPLAY must be a boolean:" \
            "$MATRIX_EXTERNAL_REPLAY" >&2
        exit 1
        ;;
esac

case "${MATRIX_EXTERNAL_REPLAY_CENTERED_CAMERA,,}" in
    1|true|yes|on)
        EXTERNAL_REPLAY_CENTERED_CAMERA_ENABLED=true
        if ! $MATRIX_EXTERNAL_REPLAY_ENABLED; then
            echo "[ERROR] MATRIX_EXTERNAL_REPLAY_CENTERED_CAMERA requires" \
                "MATRIX_EXTERNAL_REPLAY=1" >&2
            exit 1
        fi
        if [[ "$ROBOTTYPE" != "custom" ]]; then
            echo "[ERROR] External replay centered camera supports only the" \
                "custom robot; got $ROBOTTYPE" >&2
            exit 1
        fi
        if [[ -z "$MATRIX_CENTERED_CAMERA_OVERLAY_BUNDLE" ]]; then
            echo "[ERROR] External replay centered camera requires" \
                "MATRIX_CENTERED_CAMERA_OVERLAY_BUNDLE" >&2
            exit 1
        fi
        ;;
    0|false|no|off|"")
        EXTERNAL_REPLAY_CENTERED_CAMERA_ENABLED=false
        ;;
    *)
        echo "[ERROR] MATRIX_EXTERNAL_REPLAY_CENTERED_CAMERA must be a" \
            "boolean: $MATRIX_EXTERNAL_REPLAY_CENTERED_CAMERA" >&2
        exit 1
        ;;
esac

case "${MATRIX_SONIC,,}" in
    1|true|yes|on)
        if $MATRIX_EXTERNAL_REPLAY_ENABLED; then
            echo "[ERROR] MATRIX_EXTERNAL_REPLAY and MATRIX_SONIC are mutually exclusive" >&2
            exit 1
        fi
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
# External trace replay may explicitly opt into the verified Spectator overlay;
# that overlay follows the custom robot while preserving an operator-selected
# orbit, so a replay can use a non-occluded review angle without changing the
# recorded physics trajectory.
# These are startup console commands, not the Python camera-bridge contract.
# `set Engine.SpringArmComponent` intentionally affects every live spring arm;
# an operator can append a narrower/newer command via MATRIX_UE_EXTRA_EXEC_CMDS.
CAMERA_CONFIGURATION_ENABLED=false
CAMERA_CONFIGURATION_CONTEXT=""
if $MATRIX_SONIC_ENABLED \
    && [[ "${MATRIX_SONIC_CONTROL_SOURCE:-planner}" == "game" ]] \
    && $GAME_CENTERED_CAMERA_ENABLED; then
    CAMERA_CONFIGURATION_ENABLED=true
    CAMERA_CONFIGURATION_CONTEXT="SONIC game"
elif $MATRIX_EXTERNAL_REPLAY_ENABLED \
    && $EXTERNAL_REPLAY_CENTERED_CAMERA_ENABLED; then
    CAMERA_CONFIGURATION_ENABLED=true
    CAMERA_CONFIGURATION_CONTEXT="external replay"
fi

if $CAMERA_CONFIGURATION_ENABLED; then
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
            "context=$CAMERA_CONFIGURATION_CONTEXT" \
            "robot=MujocoSim_Custom_C viewclass=$GAME_CAMERA_VIEW_CLASS" \
            "spring_arm_cm=$MATRIX_GAME_CAMERA_DISTANCE_CM"
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
if $MATRIX_EXTERNAL_REPLAY_ENABLED; then
    if /usr/bin/python3 -I - \
        config/config.json "$CONFIG_TMP" "$ROBOTTYPE" "$WEAPON" \
        "$MUJOCO_RUNNING_JSON" <<'PY'
import json
from pathlib import Path
import sys

source, target, robot_type, weapon, mujoco_running = sys.argv[1:]
payload = json.loads(Path(source).read_text(encoding="utf-8"))
if not isinstance(payload, dict):
    raise SystemExit("config root must be an object")
robot = payload.get("robot")
if robot is None:
    robot = {}
    payload["robot"] = robot
if not isinstance(robot, dict):
    raise SystemExit("config robot must be an object")
robot["robot_type"] = robot_type
robot["weapon"] = weapon
robot["mujoco_running"] = mujoco_running == "true"
if robot_type == "custom":
    robot["use_custom_urdf"] = True
    robot["custom_urdf"] = "custom/scene_terrain_custom.xml"
robot.setdefault("state_port", 25001)
robot.setdefault("cmd_port", 25002)
robot.setdefault("EgoView", True)
robot.setdefault("position", {"x": 0, "y": 0, "z": 0})
Path(target).write_text(
    json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
    encoding="utf-8",
)
PY
    then
        mv "$CONFIG_TMP" config/config.json
    else
        rm -f -- "$CONFIG_TMP"
        echo "[ERROR] Failed to update Matrix config for trace replay" >&2
        exit 1
    fi
else
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
        ' config/config.json > "$CONFIG_TMP" \
        && mv "$CONFIG_TMP" config/config.json
fi

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
if $MATRIX_EXTERNAL_REPLAY_ENABLED; then
    ROBOT_POSITION="$(/usr/bin/python3 -I - config/config.json <<'PY'
import json
from pathlib import Path
import sys

position = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))["robot"]["position"]
print(position["x"], position["y"])
PY
)"
    read -r ROBOT_X ROBOT_Y <<< "$ROBOT_POSITION"
else
    ROBOT_X=$(jq -r '.robot.position.x' config/config.json)
    ROBOT_Y=$(jq -r '.robot.position.y' config/config.json)
fi

if [[ "$ROBOTTYPE" == "custom" ]]; then
    if $MATRIX_EXTERNAL_REPLAY_ENABLED; then
        XML_FILE="src/robot_mujoco/zsibot_robots/custom/current.xml"
        if [[ ! -f "$XML_FILE" ]]; then
            echo "[ERROR] Staged Matrix replay robot XML not found: $XML_FILE" >&2
            exit 1
        fi
        echo "[INFO] External replay staged custom robot detected: $XML_FILE"
    else
        CUSTOM_MODEL_DIR="${CUSTOM_NAME:-custom}"
        XML_FILE="src/robot_mujoco/zsibot_robots/custom/_cache/${CUSTOM_MODEL_DIR}/${CUSTOM_MODEL_DIR}.xml"
        if [[ -f "$XML_FILE" ]]; then
            echo "[INFO] Custom robot detected, skipping built-in XML position update for ${XML_FILE}"
        else
            echo "[WARNING] Custom robot XML not found: $XML_FILE"
        fi
    fi
else
    XML_FILE="src/robot_mujoco/zsibot_robots/${ROBOTTYPE}/${ROBOTTYPE}.xml"
    sed -i "s/<body name=\"base_link\" pos=\"[^\"]*\"/<body name=\"base_link\" pos=\"${ROBOT_X} ${ROBOT_Y} 0.65\"/" "$XML_FILE"
fi

#######################################
# 启动流程
#######################################
echo "[INFO] Starting processes..."

cd src/robot_mujoco/simulate/build
if $ENABLE_MUJOCO && ! $MATRIX_SONIC_ENABLED \
    && ! $MATRIX_EXTERNAL_REPLAY_ENABLED; then
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
UE_MATERIAL_FIX_PRELOAD="${MATRIX_UE_MATERIAL_FIX_PRELOAD:-auto}"
UE_MATERIAL_FIX_DEFAULT="$PROJECT_ROOT/outputs/runtime/matrix-ue-material-fix/libmatrix_ue_material_fix.so"
UE_MATERIAL_FIX_BINARY=""
UE_G1_SKIN="${MATRIX_G1_SKIN:-}"
UE_G1_MATERIAL_PALETTE="$MATRIX_UE_G1_MATERIAL_PALETTE_CONTRACT"
UE_G1_MATERIAL_SCOPE_ALPHA="$MATRIX_UE_G1_SCOPE_ALPHA_CONTRACT"
UE_G1_PALETTE_PATTERN='^[-0-9eE+.,;]+$'
UE_G1_COMPONENT_PATTERN='^[-0-9eE+.]+$'
case "${UE_MATERIAL_FIX_PRELOAD,,}" in
    ""|auto)
        if [[ -f "$UE_MATERIAL_FIX_DEFAULT" && ! -L "$UE_MATERIAL_FIX_DEFAULT" ]]; then
            UE_MATERIAL_FIX_PRELOAD="$UE_MATERIAL_FIX_DEFAULT"
        else
            UE_MATERIAL_FIX_PRELOAD=""
            echo "[INFO] Matrix UE material fix default not found; continuing without skin bridge"
        fi
        ;;
    off|none|disabled|0|false|no)
        UE_MATERIAL_FIX_PRELOAD=""
        echo "[INFO] Matrix UE material fix disabled by MATRIX_UE_MATERIAL_FIX_PRELOAD=$MATRIX_UE_MATERIAL_FIX_PRELOAD"
        ;;
    *) ;;
esac
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
    || [[ -n "$UE_MATERIAL_FIX_PRELOAD" ]] \
    || $MATRIX_EXTERNAL_REPLAY_ENABLED; then
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

if $MATRIX_EXTERNAL_REPLAY_ENABLED; then
    wait_for_ue_map_ready "$UE_LOG" "$UE_LOG_START_OFFSET" "$MAPNAME"
    TRACE_REPLAY_PYTHON="${MATRIX_EXTERNAL_REPLAY_PYTHON:-${MATRIX_SONIC_PYTHON:-python3}}"
    TRACE_REPLAY_TRACE="${MATRIX_EXTERNAL_REPLAY_TRACE:-}"
    TRACE_REPLAY_MODEL="${MATRIX_EXTERNAL_REPLAY_MODEL:-}"
    TRACE_REPLAY_STATUS_FILE="${MATRIX_EXTERNAL_REPLAY_STATUS_FILE:-${MATRIX_SONIC_STATUS_FILE:-$PROJECT_ROOT/outputs/matrix_trace_replay_status.json}}"
    TRACE_REPLAY_SUMMARY="${MATRIX_EXTERNAL_REPLAY_SUMMARY:-$PROJECT_ROOT/outputs/matrix_trace_replay_summary.json}"
    TRACE_REPLAY_PRE_ROLL="${MATRIX_EXTERNAL_REPLAY_PRE_ROLL_SECONDS:-2}"
    TRACE_REPLAY_FINAL_HOLD="${MATRIX_EXTERNAL_REPLAY_FINAL_HOLD_SECONDS:-6}"
    TRACE_REPLAY_CAMERA_RECEIPT="${MATRIX_EXTERNAL_REPLAY_CAMERA_RECEIPT:-}"
    TRACE_REPLAY_CAMERA_READY_FILE="${MATRIX_EXTERNAL_REPLAY_CAMERA_READY_FILE:-}"
    if [[ -z "$TRACE_REPLAY_TRACE" ]]; then
        echo "[ERROR] MATRIX_EXTERNAL_REPLAY_TRACE is required" >&2
        exit 1
    fi
    TRACE_REPLAY_TRACE="$(realpath -- "$TRACE_REPLAY_TRACE")"
    TRACE_REPLAY_STATUS_FILE="$(realpath -m -- "$TRACE_REPLAY_STATUS_FILE")"
    TRACE_REPLAY_SUMMARY="$(realpath -m -- "$TRACE_REPLAY_SUMMARY")"
    if [[ -z "$TRACE_REPLAY_CAMERA_RECEIPT" ]]; then
        echo "[ERROR] MATRIX_EXTERNAL_REPLAY_CAMERA_RECEIPT is required" >&2
        exit 1
    fi
    TRACE_REPLAY_CAMERA_RECEIPT="$(realpath -m -- "$TRACE_REPLAY_CAMERA_RECEIPT")"
    if [[ -n "$TRACE_REPLAY_CAMERA_READY_FILE" ]]; then
        TRACE_REPLAY_CAMERA_READY_FILE="$(realpath -m -- "$TRACE_REPLAY_CAMERA_READY_FILE")"
    fi
    if [[ -n "$TRACE_REPLAY_MODEL" ]]; then
        TRACE_REPLAY_MODEL="$(realpath -- "$TRACE_REPLAY_MODEL")"
    fi
    TRACE_REPLAY_PROTECTED_CONTRACT="$(realpath -- "$MATRIX_CENTERED_CAMERA_OVERLAY_CONTRACT")"
    TRACE_REPLAY_PROTECTED_ACTIVE="$(realpath -m -- \
        "$PROJECT_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Saved/Paks/MatrixCenteredCameraActive")"
    TRACE_REPLAY_PROTECTED_BUNDLE=""
    if $EXTERNAL_REPLAY_CENTERED_CAMERA_ENABLED; then
        TRACE_REPLAY_PROTECTED_BUNDLE="$(realpath -- \
            "$MATRIX_CENTERED_CAMERA_OVERLAY_BUNDLE")"
        if path_is_equal_or_within \
            "$TRACE_REPLAY_PROTECTED_BUNDLE" "$PROJECT_ROOT"; then
            echo "[ERROR] External replay camera bundle must be outside Matrix:" \
                "$TRACE_REPLAY_PROTECTED_BUNDLE" >&2
            exit 1
        fi
    fi
    for replay_mutable_path in \
        "$TRACE_REPLAY_STATUS_FILE" "$TRACE_REPLAY_SUMMARY" \
        "$TRACE_REPLAY_CAMERA_RECEIPT" "$TRACE_REPLAY_CAMERA_READY_FILE"; do
        [[ -z "$replay_mutable_path" ]] && continue
        if [[ "$replay_mutable_path" == "$TRACE_REPLAY_PROTECTED_CONTRACT" ]] \
            || path_is_equal_or_within \
                "$replay_mutable_path" "$TRACE_REPLAY_PROTECTED_ACTIVE" \
            || { [[ -n "$TRACE_REPLAY_PROTECTED_BUNDLE" ]] \
                && path_is_equal_or_within \
                    "$replay_mutable_path" "$TRACE_REPLAY_PROTECTED_BUNDLE"; }; then
            echo "[ERROR] Matrix trace replay output aliases protected camera input:" \
                "$replay_mutable_path" >&2
            exit 1
        fi
    done
    if [[ "$TRACE_REPLAY_STATUS_FILE" == "$TRACE_REPLAY_SUMMARY" \
        || "$TRACE_REPLAY_CAMERA_RECEIPT" == "$TRACE_REPLAY_STATUS_FILE" \
        || "$TRACE_REPLAY_CAMERA_RECEIPT" == "$TRACE_REPLAY_SUMMARY" \
        || "$TRACE_REPLAY_STATUS_FILE" == "$TRACE_REPLAY_TRACE" \
        || "$TRACE_REPLAY_SUMMARY" == "$TRACE_REPLAY_TRACE" \
        || "$TRACE_REPLAY_CAMERA_RECEIPT" == "$TRACE_REPLAY_TRACE" \
        || "$TRACE_REPLAY_STATUS_FILE" == "$UE_LOG" \
        || "$TRACE_REPLAY_SUMMARY" == "$UE_LOG" \
        || "$TRACE_REPLAY_CAMERA_RECEIPT" == "$UE_LOG" \
        || ( -n "$TRACE_REPLAY_MODEL" \
            && ( "$TRACE_REPLAY_STATUS_FILE" == "$TRACE_REPLAY_MODEL" \
                || "$TRACE_REPLAY_SUMMARY" == "$TRACE_REPLAY_MODEL" \
                || "$TRACE_REPLAY_CAMERA_RECEIPT" == "$TRACE_REPLAY_MODEL" ) ) ]]; then
        echo "[ERROR] Matrix trace replay source and output paths must be distinct" >&2
        exit 1
    fi
    if [[ -n "$TRACE_REPLAY_CAMERA_READY_FILE" \
        && ( "$TRACE_REPLAY_CAMERA_READY_FILE" == "$TRACE_REPLAY_TRACE" \
            || "$TRACE_REPLAY_CAMERA_READY_FILE" == "$TRACE_REPLAY_STATUS_FILE" \
            || "$TRACE_REPLAY_CAMERA_READY_FILE" == "$TRACE_REPLAY_SUMMARY" \
            || "$TRACE_REPLAY_CAMERA_READY_FILE" == "$TRACE_REPLAY_CAMERA_RECEIPT" \
            || "$TRACE_REPLAY_CAMERA_READY_FILE" == "$UE_LOG" \
            || ( -n "$TRACE_REPLAY_MODEL" \
                && "$TRACE_REPLAY_CAMERA_READY_FILE" == "$TRACE_REPLAY_MODEL" ) ) ]]; then
        echo "[ERROR] Matrix camera-ready path aliases replay input/output" >&2
        exit 1
    fi
    for stale_replay_output in \
        "$TRACE_REPLAY_STATUS_FILE" "$TRACE_REPLAY_SUMMARY" \
        "$TRACE_REPLAY_CAMERA_RECEIPT"; do
        if [[ -L "$stale_replay_output" || -d "$stale_replay_output" ]]; then
            echo "[ERROR] Matrix trace replay output must not be a symlink or directory:" \
                "$stale_replay_output" >&2
            exit 1
        fi
        rm -f -- "$stale_replay_output"
    done
    if [[ ! -f "$PROJECT_ROOT/scripts/replay_matrix_physics_trace.py" ]]; then
        echo "[ERROR] Matrix physics-trace replay script is missing" >&2
        exit 1
    fi
    if [[ ! -f "$PROJECT_ROOT/scripts/matrix_scene6_camera_receipt.py" ]]; then
        echo "[ERROR] Matrix scene6 camera receipt script is missing" >&2
        exit 1
    fi
    if $EXTERNAL_REPLAY_CENTERED_CAMERA_ENABLED; then
        TRACE_REPLAY_CAMERA_MODE="spectator-overlay"
    else
        TRACE_REPLAY_CAMERA_MODE="robot"
    fi
    wait_for_external_camera_ready \
        "$TRACE_REPLAY_CAMERA_READY_FILE" "$TRACE_REPLAY_CAMERA_MODE"
    CAMERA_RECEIPT_COMMAND=(
        /usr/bin/python3 -I
        "$PROJECT_ROOT/scripts/matrix_scene6_camera_receipt.py"
        write
        --output "$TRACE_REPLAY_CAMERA_RECEIPT"
        --mode "$TRACE_REPLAY_CAMERA_MODE"
        --spring-arm-cm "$MATRIX_GAME_CAMERA_DISTANCE_CM"
        --ue-exec-cmds "$UE_EXEC_CMDS"
        --project-root "$PROJECT_ROOT"
    )
    if $EXTERNAL_REPLAY_CENTERED_CAMERA_ENABLED; then
        CAMERA_RECEIPT_COMMAND+=(
            --contract "$MATRIX_CENTERED_CAMERA_OVERLAY_CONTRACT"
            --bundle "$MATRIX_CENTERED_CAMERA_OVERLAY_BUNDLE"
            --ue-log "$UE_LOG"
            --ue-log-start-offset "$UE_LOG_START_OFFSET"
        )
    fi
    if [[ -n "$TRACE_REPLAY_CAMERA_READY_FILE" ]]; then
        CAMERA_RECEIPT_COMMAND+=(--ready-file "$TRACE_REPLAY_CAMERA_READY_FILE")
    fi
    "${CAMERA_RECEIPT_COMMAND[@]}"
    TRACE_REPLAY_CAMERA_RECEIPT_SHA256="$(
        /usr/bin/sha256sum -- "$TRACE_REPLAY_CAMERA_RECEIPT"
    )"
    TRACE_REPLAY_CAMERA_RECEIPT_SHA256="${TRACE_REPLAY_CAMERA_RECEIPT_SHA256%% *}"
    if [[ ! "$TRACE_REPLAY_CAMERA_RECEIPT_SHA256" =~ ^[0-9a-f]{64}$ ]]; then
        echo "[ERROR] Could not bind the current-run camera receipt SHA256" >&2
        exit 1
    fi
    TRACE_REPLAY_COMMAND=(
        "$TRACE_REPLAY_PYTHON"
        "$PROJECT_ROOT/scripts/replay_matrix_physics_trace.py"
        --trace "$TRACE_REPLAY_TRACE"
        --status-file "$TRACE_REPLAY_STATUS_FILE"
        --summary "$TRACE_REPLAY_SUMMARY"
        --camera-receipt "$TRACE_REPLAY_CAMERA_RECEIPT"
        --camera-receipt-sha256 "$TRACE_REPLAY_CAMERA_RECEIPT_SHA256"
        --pre-roll "$TRACE_REPLAY_PRE_ROLL"
        --final-hold "$TRACE_REPLAY_FINAL_HOLD"
        --ue-pid "$UE_PID"
    )
    if [[ -n "$TRACE_REPLAY_MODEL" ]]; then
        TRACE_REPLAY_COMMAND+=(--model "$TRACE_REPLAY_MODEL")
    fi
    mkdir -p "$PROJECT_ROOT/outputs/logs"
    echo "[INFO] Starting Matrix UE physics-trace replay"
    "${TRACE_REPLAY_COMMAND[@]}" \
        > "$PROJECT_ROOT/outputs/logs/matrix_trace_replay.log" 2>&1 &
    TRACE_REPLAY_PID=$!
    PIDS+=("$TRACE_REPLAY_PID")
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
            "$PROJECT_ROOT/scripts/matrix_ui_settings.py" \
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
    SONIC_INVENTORY_ARGS=()
    if [[ -n "${MATRIX_CREATIVE_INVENTORY_CATALOG:-}" ]]; then
        if [[ ! -f "$MATRIX_CREATIVE_INVENTORY_CATALOG" ]]; then
            echo "[ERROR] Creative inventory catalog is missing: $MATRIX_CREATIVE_INVENTORY_CATALOG" >&2
            exit 1
        fi
        if [[ ! -f "$PROJECT_ROOT/scripts/inject_creative_inventory.py" ]]; then
            echo "[ERROR] Creative inventory injector is missing" >&2
            exit 1
        fi
        SONIC_INVENTORY_ARGS+=(
            --creative-inventory-catalog "$MATRIX_CREATIVE_INVENTORY_CATALOG"
        )
    fi
    if [[ "$SCENE" == "scene_terrain_t10.xml" ]]; then
        SONIC_SCENE_TRANSFORM_ARGS+=(
            --scene-transform town10-open-boundary-v1
        )
        echo "[INFO] Town10 perimeter collision walls removed in derived physics scene"
    fi
    if [[ "$SCENE" == "scene_terrain_moon_dynamic.xml" ]]; then
        SONIC_SCENE_TRANSFORM_ARGS+=(
            --scene-transform moon-dynamic-ground-static-v1
        )
        echo "[INFO] MoonWorld dynamic ground blocks staticized in derived SONIC physics scene"
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
        if ! GAME_WORLD_START_OUTPUT="$(
            "$MATRIX_SONIC_PYTHON" "$PROJECT_ROOT/scripts/matrix_world_state.py" \
                resolve-start \
                --file "$GAME_WORLD_STATE_FILE" \
                --world-id "$GAME_WORLD_ID" \
                --world-revision "$GAME_WORLD_REVISION"
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
            SONIC_SPAWN_ARGS=(
                --spawn-x "${GAME_WORLD_START_LINES[1]}"
                --spawn-y "${GAME_WORLD_START_LINES[2]}"
                --spawn-z "${GAME_WORLD_START_LINES[3]}"
                --spawn-yaw "${GAME_WORLD_START_LINES[4]}"
            )
            echo "[INFO] Matrix resume pose: ${GAME_WORLD_START_LINES[5]} " \
                "world=$GAME_WORLD_ID state=${GAME_WORLD_START_LINES[6]}"
        elif [[ "${GAME_WORLD_START_LINES[0]:-}" == "none" \
            && "${#GAME_WORLD_START_LINES[@]}" == "2" ]]; then
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
    SONIC_PHYSICS_DIR="${MATRIX_SONIC_PHYSICS_DIR:-$PROJECT_ROOT/outputs/runtime/matrix_sonic/$CUSTOM_NAME/${SCENE%.xml}}"
    "$MATRIX_SONIC_PYTHON" "$PROJECT_ROOT/scripts/prepare_sonic_physics_model.py" \
        --canonical-model "$MATRIX_SONIC_CANONICAL_MODEL" \
        --canonical-meshes "$MATRIX_SONIC_CANONICAL_MESHES" \
        --native-scene "$NATIVE_SONIC_SCENE" \
        --output-dir "$SONIC_PHYSICS_DIR" \
        "${SONIC_SPAWN_ARGS[@]}" \
        "${SONIC_INVENTORY_ARGS[@]}" \
        "${SONIC_SCENE_TRANSFORM_ARGS[@]}"
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
    PHYSICAL_RECOVERY_ARGS=()
    SONIC_FAIL_ON_FALL_ENABLED=0
    case "${MATRIX_SONIC_FAIL_ON_FALL:-1}" in
        1|true|yes|on)
            SONIC_FAIL_ON_FALL_ENABLED=1
            SONIC_ACCEPTANCE_ARGS+=(--fail-on-fall)
            ;;
        0|false|no|off|"") ;;
        *)
            echo "[ERROR] MATRIX_SONIC_FAIL_ON_FALL must be a boolean" >&2
            exit 1
            ;;
    esac
    case "${MATRIX_GAME_FALL_RECOVERY:-off}" in
        off|"") ;;
        sonic)
            if [[ "$SONIC_FAIL_ON_FALL_ENABLED" == "1" ]]; then
                echo "[ERROR] MATRIX_GAME_FALL_RECOVERY=sonic conflicts with MATRIX_SONIC_FAIL_ON_FALL" >&2
                exit 1
            fi
            if [[ "${MATRIX_SONIC_CONTROL_SOURCE:-planner}" != "game" ]]; then
                echo "[ERROR] MATRIX_GAME_FALL_RECOVERY=sonic requires game control" >&2
                exit 1
            fi
            SONIC_ACCEPTANCE_ARGS+=(
                --game-fall-recovery sonic
                --game-fall-recovery-timeout "${MATRIX_GAME_FALL_RECOVERY_TIMEOUT:-15.0}"
            )
            ;;
        physical)
            if [[ "$SONIC_FAIL_ON_FALL_ENABLED" == "1" ]]; then
                echo "[ERROR] MATRIX_GAME_FALL_RECOVERY=physical conflicts with MATRIX_SONIC_FAIL_ON_FALL" >&2
                exit 1
            fi
            if [[ "${MATRIX_SONIC_CONTROL_SOURCE:-planner}" != "game" ]]; then
                echo "[ERROR] MATRIX_GAME_FALL_RECOVERY=physical requires game control" >&2
                exit 1
            fi
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
            PHYSICAL_RECOVERY_CONTROL_SOCKET="${MATRIX_PHYSICAL_RECOVERY_CONTROL_SOCKET:-}"
            PHYSICAL_RECOVERY_SONIC_CONTROL_SOCKET="${MATRIX_PHYSICAL_RECOVERY_SONIC_CONTROL_SOCKET:-}"
            case "$PHYSICAL_RECOVERY_INITIAL_CONTROLLER" in
                host|amp|kungfu) ;;
                *)
                    echo "[ERROR] Physical recovery initial controller must be host, amp, or kungfu" >&2
                    exit 1
                    ;;
            esac
            case "$PHYSICAL_RECOVERY_HANDOFF" in
                amp|sonic) ;;
                *)
                    echo "[ERROR] Physical recovery handoff must be amp or sonic" >&2
                    exit 1
                    ;;
            esac
            case "$PHYSICAL_RECOVERY_EXECUTION_PROVIDER" in
                cuda|cpu) ;;
                *)
                    echo "[ERROR] Physical recovery execution provider must be cuda or cpu" >&2
                    exit 1
                    ;;
            esac
            if [[ "$PHYSICAL_RECOVERY_RESIDENT_POLICIES" == "1" ]]; then
                if [[ "$PHYSICAL_RECOVERY_INITIAL_CONTROLLER" != "kungfu" \
                    || "$PHYSICAL_RECOVERY_HANDOFF" != "sonic" \
                    || "$PHYSICAL_RECOVERY_EXECUTION_PROVIDER" != "cuda" ]]; then
                    echo "[ERROR] Resident recovery requires kungfu -> sonic with CUDA" >&2
                    exit 1
                fi
            fi
            for required in \
                "$PHYSICAL_RECOVERY_WORKER" \
                "$PHYSICAL_RECOVERY_PYTHON" \
                "$PHYSICAL_RECOVERY_MODEL" \
                "$PHYSICAL_RECOVERY_AMP_CONFIG" \
                "$PHYSICAL_RECOVERY_AMP_MODEL"; do
                if [[ -z "$required" || ! -f "$required" ]]; then
                    echo "[ERROR] Physical recovery dependency is missing: $required" >&2
                    exit 1
                fi
            done
            if [[ ! -x "$PHYSICAL_RECOVERY_PYTHON" ]]; then
                echo "[ERROR] Physical recovery Python is not executable:" \
                    "$PHYSICAL_RECOVERY_PYTHON" >&2
                exit 1
            fi
            if [[ -n "$PHYSICAL_RECOVERY_FALLBACK_MODEL" \
                && ! -f "$PHYSICAL_RECOVERY_FALLBACK_MODEL" ]]; then
                echo "[ERROR] Physical recovery fallback model is missing:" \
                    "$PHYSICAL_RECOVERY_FALLBACK_MODEL" >&2
                exit 1
            fi
            if [[ -n "$PHYSICAL_RECOVERY_MODEL_SHA256" ]]; then
                actual_recovery_sha256="$(/usr/bin/python3 -I - "$PHYSICAL_RECOVERY_MODEL" <<'PY'
import hashlib
from pathlib import Path
import sys
print(hashlib.sha256(Path(sys.argv[1]).read_bytes()).hexdigest())
PY
)"
                if [[ ! "$PHYSICAL_RECOVERY_MODEL_SHA256" =~ ^[0-9a-f]{64}$ \
                    || "$actual_recovery_sha256" != "$PHYSICAL_RECOVERY_MODEL_SHA256" ]]; then
                    echo "[ERROR] Physical recovery model SHA256 mismatch" >&2
                    exit 1
                fi
            fi
            if [[ "$PHYSICAL_RECOVERY_CONTROL_SOCKET" != /* ]]; then
                echo "[ERROR] Physical recovery control socket must be absolute" >&2
                exit 1
            fi
            if [[ "$PHYSICAL_RECOVERY_SONIC_CONTROL_SOCKET" != /* ]]; then
                echo "[ERROR] SONIC writer control socket must be absolute" >&2
                exit 1
            fi
            for digest in \
                "$PHYSICAL_RECOVERY_AMP_CONFIG_SHA256" \
                "$PHYSICAL_RECOVERY_AMP_MODEL_SHA256"; do
                if [[ ! "$digest" =~ ^[0-9a-f]{64}$ ]]; then
                    echo "[ERROR] Physical recovery AMP SHA256 is invalid" >&2
                    exit 1
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
                        echo "[ERROR] KungFu recovery SHA256 is invalid" >&2
                        exit 1
                    fi
                done
                if [[ ! "$PHYSICAL_RECOVERY_KUNGFU_REFERENCE_FRAME" =~ ^[0-9]+$ ]]; then
                    echo "[ERROR] KungFu reference frame must be a non-negative integer" >&2
                    exit 1
                fi
            fi
            if ! PYTHONNOUSERSITE=1 \
                PYTHONPATH="$MATRIX_SONIC_ROOT${PYTHONPATH:+:$PYTHONPATH}" \
                "$PHYSICAL_RECOVERY_PYTHON" -c \
                'import numpy, onnxruntime; from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber; from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_; from unitree_sdk2py.utils.crc import CRC' \
                >/dev/null 2>&1; then
                echo "[ERROR] Physical recovery Python cannot import numpy," \
                    "onnxruntime, and unitree_sdk2py:" \
                    "$PHYSICAL_RECOVERY_PYTHON" >&2
                exit 1
            fi
            if [[ "$PHYSICAL_RECOVERY_EXECUTION_PROVIDER" == "cuda" ]] \
                && ! PYTHONNOUSERSITE=1 \
                    "$PHYSICAL_RECOVERY_PYTHON" -c \
                    'import onnxruntime as ort; assert "CUDAExecutionProvider" in ort.get_available_providers()' \
                    >/dev/null 2>&1; then
                echo "[ERROR] Physical recovery Python has no CUDAExecutionProvider:" \
                    "$PHYSICAL_RECOVERY_PYTHON" >&2
                exit 1
            fi
            SONIC_ACCEPTANCE_ARGS+=(
                --game-fall-recovery physical
                --game-fall-recovery-timeout "${MATRIX_GAME_FALL_RECOVERY_TIMEOUT:-15.0}"
            )
            PHYSICAL_RECOVERY_ARGS+=(
                --physical-recovery-worker "$PHYSICAL_RECOVERY_WORKER"
                --physical-recovery-initial-controller "$PHYSICAL_RECOVERY_INITIAL_CONTROLLER"
                --physical-recovery-handoff "$PHYSICAL_RECOVERY_HANDOFF"
                --physical-recovery-python "$PHYSICAL_RECOVERY_PYTHON"
                --physical-recovery-execution-provider "$PHYSICAL_RECOVERY_EXECUTION_PROVIDER"
                --physical-recovery-model "$PHYSICAL_RECOVERY_MODEL"
                --physical-recovery-amp-config "$PHYSICAL_RECOVERY_AMP_CONFIG"
                --physical-recovery-amp-model "$PHYSICAL_RECOVERY_AMP_MODEL"
                --physical-recovery-amp-config-sha256 "$PHYSICAL_RECOVERY_AMP_CONFIG_SHA256"
                --physical-recovery-amp-model-sha256 "$PHYSICAL_RECOVERY_AMP_MODEL_SHA256"
                --physical-recovery-fallback-after-seconds "${MATRIX_PHYSICAL_RECOVERY_FALLBACK_AFTER_SECONDS:-10.0}"
                --physical-recovery-stable-hold-seconds "${MATRIX_PHYSICAL_RECOVERY_STABLE_HOLD_SECONDS:-1.5}"
                --physical-recovery-policy-exit-hold-seconds "${MATRIX_PHYSICAL_RECOVERY_POLICY_EXIT_HOLD_SECONDS:-0}"
                --physical-recovery-control-socket "$PHYSICAL_RECOVERY_CONTROL_SOCKET"
                --physical-recovery-sonic-control-socket "$PHYSICAL_RECOVERY_SONIC_CONTROL_SOCKET"
                --physical-recovery-sonic-prewarm-timeout-seconds "${MATRIX_PHYSICAL_RECOVERY_SONIC_PREWARM_TIMEOUT_SECONDS:-35.0}"
            )
            if [[ "$PHYSICAL_RECOVERY_RESIDENT_POLICIES" == "1" ]]; then
                PHYSICAL_RECOVERY_ARGS+=(--physical-recovery-resident-policies)
            fi
            if [[ -n "$PHYSICAL_RECOVERY_FALLBACK_MODEL" ]]; then
                PHYSICAL_RECOVERY_ARGS+=(
                    --physical-recovery-fallback-model "$PHYSICAL_RECOVERY_FALLBACK_MODEL"
                )
            fi
            if [[ "$PHYSICAL_RECOVERY_INITIAL_CONTROLLER" == "kungfu" ]]; then
                PHYSICAL_RECOVERY_ARGS+=(
                    --physical-recovery-kungfu-model "$PHYSICAL_RECOVERY_KUNGFU_MODEL"
                    --physical-recovery-kungfu-motion "$PHYSICAL_RECOVERY_KUNGFU_MOTION"
                    --physical-recovery-kungfu-model-sha256 "$PHYSICAL_RECOVERY_KUNGFU_MODEL_SHA256"
                    --physical-recovery-kungfu-model-data-sha256 "$PHYSICAL_RECOVERY_KUNGFU_MODEL_DATA_SHA256"
                    --physical-recovery-kungfu-motion-sha256 "$PHYSICAL_RECOVERY_KUNGFU_MOTION_SHA256"
                    --physical-recovery-kungfu-reference-frame "$PHYSICAL_RECOVERY_KUNGFU_REFERENCE_FRAME"
                    --physical-recovery-kungfu-gain-scale "$PHYSICAL_RECOVERY_KUNGFU_GAIN_SCALE"
                )
            fi
            ;;
        *)
            echo "[ERROR] MATRIX_GAME_FALL_RECOVERY must be off, sonic, or physical" >&2
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
        --game-applied-mouse-profile "${MATRIX_MOUSE_APPLIED_PROFILE:-local}"
        --game-applied-mouse-speed-scale "${MATRIX_MOUSE_APPLIED_SPEED_SCALE:-1.0}"
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
        --max-resets "${MATRIX_SONIC_MAX_RESETS:-0}" \
        "${SONIC_ACCEPTANCE_ARGS[@]}" \
        "${PHYSICAL_RECOVERY_ARGS[@]}" \
        "${SONIC_QUALIFICATION_ARGS[@]}" \
        "${SONIC_STARTUP_ARGS[@]}" \
        --startup-band-hold "${MATRIX_SONIC_STARTUP_BAND_HOLD:-4}" \
        --startup-band-fade "${MATRIX_SONIC_STARTUP_BAND_FADE:-3}" \
        "${GAME_INPUT_ARGS[@]}" \
        "${SONIC_WORLD_ARGS[@]}" \
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
if [[ -n "$TRACE_REPLAY_PID" ]]; then
    if ((BASH_VERSINFO[0] < 5)) \
        || ((BASH_VERSINFO[0] == 5 && BASH_VERSINFO[1] < 1)); then
        echo "[ERROR] Matrix trace-replay supervision requires Bash 5.1 or newer" >&2
        exit 2
    fi
    set +e
    COMPLETED_PID=""
    wait -n -p COMPLETED_PID "$TRACE_REPLAY_PID" "$UE_SUPERVISOR_PID"
    FIRST_EXIT_CODE=$?
    if [[ "$COMPLETED_PID" == "$UE_SUPERVISOR_PID" ]]; then
        UE_SUPERVISOR_REAPED=1
        record_ue_supervisor_failure
        # The replayer pins /proc start-time identity for this exact UE child
        # and exits on its next 40 ms poll; waiting avoids signaling a reused
        # numeric PID after wait-n has reaped a near-simultaneous child.
        wait "$TRACE_REPLAY_PID"
        TRACE_REPLAY_EXIT_CODE=$?
    else
        TRACE_REPLAY_EXIT_CODE="$FIRST_EXIT_CODE"
    fi
    remove_managed_pid "$TRACE_REPLAY_PID"
    TRACE_REPLAY_PID=""
    set -e
    stop_supervised_ue
    if [[ -e "$UE_FAILURE_FILE" && "$TRACE_REPLAY_EXIT_CODE" == "0" ]]; then
        TRACE_REPLAY_EXIT_CODE=2
    fi
    echo "[INFO] Matrix UE physics-trace replay exited with code" \
        "$TRACE_REPLAY_EXIT_CODE"
    exit "$TRACE_REPLAY_EXIT_CODE"
fi
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
