#!/usr/bin/env bash
set -euo pipefail

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
RUN_SIM_PARENT_PID="${MATRIX_SONIC_LAUNCHER_PID:-$PPID}"
CLEANUP_STARTED=0

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
    local supervisor_python="${MATRIX_SONIC_PYTHON:-$(command -v python3)}"
    coproc MATRIX_UE_SUPERVISOR {
        exec "$supervisor_python" "$PROJECT_ROOT/scripts/supervise_matrix_ue.py" \
            --pid-file "$UE_PID_FILE" \
            --failure-file "$UE_FAILURE_FILE" \
            --log "$ue_log" \
            --expected-parent-pid "$$" \
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

cleanup() {
    if [[ "$CLEANUP_STARTED" == "1" ]]; then
        return
    fi
    CLEANUP_STARTED=1
    echo "[INFO] ===== Cleaning up processes ====="

    stop_parent_watchdog

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

    echo "[INFO] ===== Cleanup finished ====="
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
UE_EXEC_CMDS="t.MaxFPS $UE_MAX_FPS"

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
    if [[ -n "$MATRIX_GAME_CAMERA_VIEW_CLASS" ]]; then
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
    UE_EXEC_CMDS="${UE_EXEC_CMDS},viewclass ${GAME_CAMERA_VIEW_CLASS}"
    echo "[INFO] Native centered game-camera startup enabled: viewclass=$GAME_CAMERA_VIEW_CLASS"
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

cd src/robot_mujoco/simulate/build
if $ENABLE_MUJOCO && ! $MATRIX_SONIC_ENABLED; then
    echo "[INFO] Starting MuJoCo"
    LD_LIBRARY_PATH="$(mujoco_ld_library_path)" ./robot_mujoco > robot_mujoco.log 2>&1 &
    PIDS+=($!)
fi

cd ../../../UeSim/Linux
echo "[INFO] Starting UE"
UE_MOUSE_RELATIVE_SPEED_SCALE="${MATRIX_MOUSE_APPLIED_SPEED_SCALE:-1.0}"
if [[ ! "$UE_MOUSE_RELATIVE_SPEED_SCALE" =~ ^(0\.[2-9][0-9]*|1(\.0+)?)$ ]]; then
    echo "[ERROR] MATRIX_MOUSE_APPLIED_SPEED_SCALE must be in [0.2, 1.0]" >&2
    exit 1
fi
UE_COMMAND=(
    /usr/bin/env
    "LD_LIBRARY_PATH=$(ue_ld_library_path)"
    # Force SDL's raw relative-motion path.  These hints make the behavior
    # explicit across local Xorg and remote nxagent sessions: no warp
    # emulation, viewport scaling, or system pointer acceleration is allowed
    # to reshape a camera drag before UE receives it.
    "SDL_MOUSE_RELATIVE_MODE_WARP=0"
    "SDL_MOUSE_RELATIVE_SCALING=0"
    "SDL_MOUSE_RELATIVE_SPEED_SCALE=$UE_MOUSE_RELATIVE_SPEED_SCALE"
    "SDL_MOUSE_RELATIVE_SYSTEM_SCALE=0"
    ./zsibot_mujoco_ue.sh -game "$MAPNAME"
    # The stock cooked package enables UE's legacy PlayerInput mouse
    # smoothing.  Override it in the Input config hierarchy so a released
    # drag has no interpolated tail; disabling FOV scaling also keeps one
    # physical delta at one stable gain while zoom/FOV changes.
    "-ini:Input:[/Script/Engine.InputSettings]:bEnableMouseSmoothing=False,[/Script/Engine.InputSettings]:bEnableFOVScaling=False"
    "-ExecCmds=$UE_EXEC_CMDS"
)
[[ -n "$USE_OFFSCREEN" ]] && UE_COMMAND+=("$USE_OFFSCREEN")
[[ -n "$USE_PIXELSTREAMER" ]] && UE_COMMAND+=("$USE_PIXELSTREAMER")
start_supervised_ue "$PWD/zsibot_mujoco_ue.log" "${UE_COMMAND[@]}"

UE_STARTUP_SECONDS="${MATRIX_UE_STARTUP_SECONDS:-7}"
if [[ ! "$UE_STARTUP_SECONDS" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    echo "[ERROR] MATRIX_UE_STARTUP_SECONDS must be a non-negative number: $UE_STARTUP_SECONDS" >&2
    exit 1
fi
sleep "$UE_STARTUP_SECONDS"

if $MATRIX_SONIC_ENABLED; then
    MATRIX_SONIC_PYTHON="${MATRIX_SONIC_PYTHON:-python3}"
    MATRIX_SONIC_ROOT="${MATRIX_SONIC_ROOT:-}"
    MATRIX_UNITREE_SDK2_ROOT="${MATRIX_UNITREE_SDK2_ROOT:-}"
    MATRIX_SONIC_CANONICAL_MODEL="${MATRIX_SONIC_CANONICAL_MODEL:-$MATRIX_SONIC_ROOT/gear_sonic/data/robot_model/model_data/g1/g1_29dof_with_hand.xml}"
    MATRIX_SONIC_CANONICAL_MESHES="${MATRIX_SONIC_CANONICAL_MESHES:-$MATRIX_SONIC_ROOT/gear_sonic/data/robot_model/model_data/g1/meshes}"
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
    if [[ "${MATRIX_SONIC_CONTROL_SOURCE:-planner}" == "game" \
        && ! -f "$PROJECT_ROOT/scripts/matrix_game_control_input.py" ]]; then
        echo "[ERROR] Matrix game-control input provider is missing: $PROJECT_ROOT/scripts/matrix_game_control_input.py" >&2
        exit 1
    fi
    if [[ ! -d "$MATRIX_SONIC_CANONICAL_MESHES" ]]; then
        echo "[ERROR] Canonical SONIC G1 meshes are missing: $MATRIX_SONIC_CANONICAL_MESHES" >&2
        exit 1
    fi
    mkdir -p "$PROJECT_ROOT/outputs/logs"
    SONIC_PHYSICS_DIR="${MATRIX_SONIC_PHYSICS_DIR:-$PROJECT_ROOT/outputs/runtime/matrix_sonic/$CUSTOM_NAME/${SCENE%.xml}}"
    "$MATRIX_SONIC_PYTHON" "$PROJECT_ROOT/scripts/prepare_sonic_physics_model.py" \
        --canonical-model "$MATRIX_SONIC_CANONICAL_MODEL" \
        --canonical-meshes "$MATRIX_SONIC_CANONICAL_MESHES" \
        --native-scene "$PROJECT_ROOT/src/robot_mujoco/zsibot_robots/xgb/$SCENE" \
        --output-dir "$SONIC_PHYSICS_DIR"
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
    case "${MATRIX_SONIC_FAIL_ON_FALL:-1}" in
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
        "${SONIC_QUALIFICATION_ARGS[@]}" \
        "${SONIC_STARTUP_ARGS[@]}" \
        --startup-band-hold "${MATRIX_SONIC_STARTUP_BAND_HOLD:-4}" \
        --startup-band-fade "${MATRIX_SONIC_STARTUP_BAND_FADE:-3}" \
        "${GAME_INPUT_ARGS[@]}" \
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
        if [[ "$SONIC_EXIT_CODE" == "0" ]]; then
            SONIC_EXIT_CODE=2
        fi
    fi
    echo "[INFO] Matrix SONIC runtime exited with code $SONIC_EXIT_CODE"
    exit "$SONIC_EXIT_CODE"
fi
wait
