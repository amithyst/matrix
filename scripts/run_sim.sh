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

    "$checker" runtime \
        --robot "$ROBOT_ARG" \
        --scene "$SCENE_ID" \
        --mujoco "$MUJOCORUNNING" \
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
    local parent_pid="$$"
    (
        trap 'exit 0' TERM INT
        trap '' HUP
        while kill -0 "${parent_pid}" 2>/dev/null; do
            sleep 1
        done

        echo "[INFO] Parent launcher process exited unexpectedly, cleaning child processes..."
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
    echo "[INFO] ===== Cleaning up processes ====="

    stop_parent_watchdog
    schedule_forced_cleanup

    # 1. 优雅关闭脚本启动的进程
    for pid in "${PIDS[@]:-}"; do
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            echo "[INFO] SIGTERM PID $pid"
            kill -TERM "$pid" 2>/dev/null || true
        fi
    done

    # 2. 兜底清理（仅限本项目）
    kill_known_processes TERM

    # Give the SONIC Python runtime time to close its exact deploy/bridge
    # children. Never pattern-kill those names: TRNA may also host a real robot.
    local attempt
    for ((attempt = 0; attempt < 50; attempt++)); do
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

    echo "[INFO] ===== Cleanup finished ====="
}
trap cleanup EXIT SIGINT SIGTERM SIGHUP
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
if [[ -n "${MATRIX_UE_EXTRA_EXEC_CMDS:-}" ]]; then
    UE_EXEC_CMDS="${UE_EXEC_CMDS},${MATRIX_UE_EXTRA_EXEC_CMDS}"
fi

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
        echo "[INFO] External AndroidTwin/SONIC MuJoCo driver enabled"
        ;;
    0|false|no|off|"")
        MATRIX_SONIC_ENABLED=false
        ;;
    *)
        echo "[ERROR] MATRIX_SONIC must be a boolean: $MATRIX_SONIC" >&2
        exit 1
        ;;
esac

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

    python3 "$composer" "$mujoco_source" "$mujoco_target"
    python3 "$composer" "$ue_source" "$ue_target"
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
LD_LIBRARY_PATH="$(ue_ld_library_path)" ./zsibot_mujoco_ue.sh -game "$MAPNAME" -ExecCmds="$UE_EXEC_CMDS" $USE_OFFSCREEN $USE_PIXELSTREAMER > zsibot_mujoco_ue.log 2>&1 &
PIDS+=($!)

sleep 7

if $MATRIX_SONIC_ENABLED; then
    MATRIX_SONIC_PYTHON="${MATRIX_SONIC_PYTHON:-python3}"
    MATRIX_AUE_ROOT="${MATRIX_AUE_ROOT:-}"
    MATRIX_GEAR_SONIC_ROOT="${MATRIX_GEAR_SONIC_ROOT:-}"
    MATRIX_UNITREE_SDK2_ROOT="${MATRIX_UNITREE_SDK2_ROOT:-}"
    MATRIX_SONIC_CANONICAL_MODEL="${MATRIX_SONIC_CANONICAL_MODEL:-$MATRIX_GEAR_SONIC_ROOT/gear_sonic/data/robot_model/model_data/g1/g1_29dof_with_hand.xml}"
    MATRIX_SONIC_CANONICAL_MESHES="${MATRIX_SONIC_CANONICAL_MESHES:-$MATRIX_GEAR_SONIC_ROOT/gear_sonic/data/robot_model/model_data/g1/meshes}"
    for required in \
        "$PROJECT_ROOT/scripts/run_matrix_sonic.py" \
        "$PROJECT_ROOT/scripts/prepare_sonic_physics_model.py" \
        "$MATRIX_AUE_ROOT/src/androidtwin/control/sonic_sim/fused_sink.py" \
        "$MATRIX_GEAR_SONIC_ROOT/gear_sonic_deploy/target/release/g1_deploy_onnx_ref" \
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
    SONIC_PHYSICS_DIR="${MATRIX_SONIC_PHYSICS_DIR:-$PROJECT_ROOT/outputs/runtime/matrix_sonic/$CUSTOM_NAME/${SCENE%.xml}}"
    "$MATRIX_SONIC_PYTHON" "$PROJECT_ROOT/scripts/prepare_sonic_physics_model.py" \
        --canonical-model "$MATRIX_SONIC_CANONICAL_MODEL" \
        --canonical-meshes "$MATRIX_SONIC_CANONICAL_MESHES" \
        --native-scene "$PROJECT_ROOT/src/robot_mujoco/zsibot_robots/xgb/$SCENE" \
        --output-dir "$SONIC_PHYSICS_DIR"
    SONIC_STATUS_FILE="${MATRIX_SONIC_STATUS_FILE:-$PROJECT_ROOT/outputs/matrix_sonic_status.json}"
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
    echo "[INFO] Starting external AndroidTwin/SONIC MuJoCo runtime"
    "$MATRIX_SONIC_PYTHON" "$PROJECT_ROOT/scripts/run_matrix_sonic.py" \
        --model "$SONIC_PHYSICS_DIR/$SCENE" \
        --aue-root "$MATRIX_AUE_ROOT" \
        --gear-sonic-root "$MATRIX_GEAR_SONIC_ROOT" \
        --unitree-sdk-root "$MATRIX_UNITREE_SDK2_ROOT" \
        --control-source "${MATRIX_SONIC_CONTROL_SOURCE:-planner}" \
        --planner-bind "${MATRIX_SONIC_PLANNER_BIND:-tcp://0.0.0.0:5556}" \
        --physics-hz "${MATRIX_SONIC_PHYSICS_HZ:-200}" \
        --walk-after "${MATRIX_SONIC_WALK_AFTER:--1}" \
        --vx "${MATRIX_SONIC_VX:-0.30}" \
        --vy "${MATRIX_SONIC_VY:-0.0}" \
        --yaw-rate "${MATRIX_SONIC_YAW_RATE:-0.0}" \
        --max-seconds "${MATRIX_SONIC_MAX_SECONDS:-0}" \
        "${SONIC_STARTUP_ARGS[@]}" \
        --startup-band-hold "${MATRIX_SONIC_STARTUP_BAND_HOLD:-4}" \
        --startup-band-fade "${MATRIX_SONIC_STARTUP_BAND_FADE:-3}" \
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
    set +e
    wait "$SONIC_PID"
    SONIC_EXIT_CODE=$?
    set -e
    echo "[INFO] Matrix SONIC runtime exited with code $SONIC_EXIT_CODE"
    exit "$SONIC_EXIT_CODE"
fi
wait
