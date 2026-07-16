#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

SCENE_ID=21
CUSTOM_URDF=""
CUSTOM_NAME="g1_29dof"
CONTROL_SOURCE="planner"
WALK_AFTER="-1"
VX="0.30"
VY="0.0"
YAW_RATE="0.0"
MAX_SECONDS="0"
OFFSCREEN=0
STARTUP_BAND=1
STARTUP_BAND_HOLD="4"
STARTUP_BAND_FADE="3"

usage() {
    printf '%s\n' \
        "Usage: bash scripts/run_matrix_sonic.sh --urdf PATH [options]" \
        "" \
        "Options:" \
        "  --scene ID                 Matrix native scene id (default: 21 ApartmentWorld)" \
        "  --name NAME                Custom robot cache name (default: g1_29dof)" \
        "  --control-source SOURCE    planner, pico, or external (default: planner)" \
        "  --walk-after SECONDS       Start planner walking after delay; -1 stays idle" \
        "  --vx MPS                    Forward command after walk delay (default: 0.30)" \
        "  --vy MPS                    Lateral command after walk delay" \
        "  --yaw-rate RAD_S           Yaw command after walk delay" \
        "  --max-seconds SECONDS      Stop a bounded smoke automatically; 0 is unlimited" \
        "  --no-startup-band          Disable the temporary SONIC INIT root stabilizer" \
        "  --startup-band-hold SEC    Root hold before fade (default: 4, INIT ramp is 3)" \
        "  --startup-band-fade SEC    Root stabilizer fade duration (default: 3)" \
        "  --offscreen                 Start Matrix UE offscreen" \
        "" \
        "Runtime roots can be overridden with MATRIX_AUE_ROOT, MATRIX_GEAR_SONIC_ROOT," \
        "MATRIX_UNITREE_SDK2_ROOT, and MATRIX_SONIC_PYTHON."
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --scene) SCENE_ID="$2"; shift 2 ;;
        --urdf) CUSTOM_URDF="$2"; shift 2 ;;
        --name) CUSTOM_NAME="$2"; shift 2 ;;
        --control-source) CONTROL_SOURCE="$2"; shift 2 ;;
        --walk-after) WALK_AFTER="$2"; shift 2 ;;
        --vx) VX="$2"; shift 2 ;;
        --vy) VY="$2"; shift 2 ;;
        --yaw-rate) YAW_RATE="$2"; shift 2 ;;
        --max-seconds) MAX_SECONDS="$2"; shift 2 ;;
        --startup-band) STARTUP_BAND=1; shift ;;
        --no-startup-band) STARTUP_BAND=0; shift ;;
        --startup-band-hold) STARTUP_BAND_HOLD="$2"; shift 2 ;;
        --startup-band-fade) STARTUP_BAND_FADE="$2"; shift 2 ;;
        --offscreen) OFFSCREEN=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "[ERROR] Unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

if [[ -z "$CUSTOM_URDF" || ! -f "$CUSTOM_URDF" ]]; then
    echo "[ERROR] --urdf must point to the canonical G1 URDF" >&2
    exit 2
fi

find_first_dir() {
    local candidate
    for candidate in "$@"; do
        if [[ -d "$candidate" ]]; then
            realpath "$candidate"
            return 0
        fi
    done
    return 1
}

MATRIX_AUE_ROOT="${MATRIX_AUE_ROOT:-$(find_first_dir \
    "$PROJECT_ROOT/../aue-sim" \
    "$HOME/aue-split-lab/repos/aue-sim" || true)}"
MATRIX_GEAR_SONIC_ROOT="${MATRIX_GEAR_SONIC_ROOT:-$(find_first_dir \
    "$MATRIX_AUE_ROOT/third_party/GR00T-WholeBodyControl" \
    "$HOME/code_bryce/GR00T-WholeBodyControl" \
    "$HOME/metabot-workspace/GR00T-WholeBodyControl" || true)}"
MATRIX_UNITREE_SDK2_ROOT="${MATRIX_UNITREE_SDK2_ROOT:-$MATRIX_GEAR_SONIC_ROOT/gear_sonic_deploy/thirdparty/unitree_sdk2}"
if [[ -x "$PROJECT_ROOT/.venv-audit/bin/python" ]]; then
    DEFAULT_PYTHON="$PROJECT_ROOT/.venv-audit/bin/python"
else
    DEFAULT_PYTHON="$(command -v python3)"
fi
MATRIX_SONIC_PYTHON="${MATRIX_SONIC_PYTHON:-$DEFAULT_PYTHON}"
# The custom-URDF preflight and converter call `python3` internally. Keep those
# subprocesses on the same interpreter environment as the SONIC runtime.
export PATH="$(dirname "$MATRIX_SONIC_PYTHON"):$PATH"

for required in \
    "$MATRIX_AUE_ROOT/src/androidtwin/control/sonic_sim/fused_sink.py" \
    "$MATRIX_GEAR_SONIC_ROOT/gear_sonic_deploy/target/release/g1_deploy_onnx_ref" \
    "$MATRIX_UNITREE_SDK2_ROOT/lib/x86_64/libunitree_sdk2.a" \
    "$MATRIX_SONIC_PYTHON"; do
    if [[ ! -e "$required" ]]; then
        echo "[ERROR] Matrix SONIC runtime dependency is missing: $required" >&2
        exit 1
    fi
done

export MATRIX_SONIC=1
export MATRIX_DISABLE_MC=1
export MATRIX_AUE_ROOT MATRIX_GEAR_SONIC_ROOT MATRIX_UNITREE_SDK2_ROOT MATRIX_SONIC_PYTHON
export MATRIX_SONIC_CONTROL_SOURCE="$CONTROL_SOURCE"
export MATRIX_SONIC_WALK_AFTER="$WALK_AFTER"
export MATRIX_SONIC_VX="$VX"
export MATRIX_SONIC_VY="$VY"
export MATRIX_SONIC_YAW_RATE="$YAW_RATE"
export MATRIX_SONIC_MAX_SECONDS="$MAX_SECONDS"
export MATRIX_SONIC_STARTUP_BAND="$STARTUP_BAND"
export MATRIX_SONIC_STARTUP_BAND_HOLD="$STARTUP_BAND_HOLD"
export MATRIX_SONIC_STARTUP_BAND_FADE="$STARTUP_BAND_FADE"

exec "$PROJECT_ROOT/scripts/run_sim.sh" \
    custom "$SCENE_ID" "$OFFSCREEN" 0 1 "$CUSTOM_URDF" "$CUSTOM_NAME"
