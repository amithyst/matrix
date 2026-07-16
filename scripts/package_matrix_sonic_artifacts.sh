#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

AUE_ROOT=""
GEAR_ROOT=""
INFERENCE_ROOT=""
VISUAL_ROOT=""
BRIDGE=""
ROS_PREFIX=""
NATIVE_DEPS=""
WHEELHOUSE=""
OUTPUT=""

usage() {
    cat <<'EOF'
Usage: bash scripts/package_matrix_sonic_artifacts.sh [options]

Required:
  --aue-root PATH         AUE checkout/overlay containing src/androidtwin and src/aue
  --gear-root PATH        Accepted GR00T-WholeBodyControl runtime
  --inference-root PATH   Root containing TensorRT/ and onnxruntime/
  --visual-root PATH      Canonical G1 visual URDF and meshes
  --bridge PATH           Accepted UDP/DDS bridge binary
  --output PATH           New matrix-sonic-v1 artifact directory

Optional:
  --ros-prefix PATH       Isolated ROS2 ament prefix
  --native-deps PATH      Isolated native dependency root
  --wheelhouse PATH       Offline Python wheelhouse with SHA256SUMS
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --aue-root) AUE_ROOT="$2"; shift 2 ;;
        --gear-root) GEAR_ROOT="$2"; shift 2 ;;
        --inference-root) INFERENCE_ROOT="$2"; shift 2 ;;
        --visual-root) VISUAL_ROOT="$2"; shift 2 ;;
        --bridge) BRIDGE="$2"; shift 2 ;;
        --ros-prefix) ROS_PREFIX="$2"; shift 2 ;;
        --native-deps) NATIVE_DEPS="$2"; shift 2 ;;
        --wheelhouse) WHEELHOUSE="$2"; shift 2 ;;
        --output) OUTPUT="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "[ERROR] Unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

for value_name in AUE_ROOT GEAR_ROOT INFERENCE_ROOT VISUAL_ROOT BRIDGE OUTPUT; do
    if [[ -z "${!value_name}" ]]; then
        echo "[ERROR] --${value_name,,} is required" >&2
        exit 2
    fi
done
for directory in "$AUE_ROOT" "$GEAR_ROOT" "$INFERENCE_ROOT" "$VISUAL_ROOT"; do
    if [[ ! -d "$directory" ]]; then
        echo "[ERROR] Source directory is missing: $directory" >&2
        exit 1
    fi
done
if [[ ! -f "$BRIDGE" ]]; then
    echo "[ERROR] Bridge binary is missing: $BRIDGE" >&2
    exit 1
fi
BRIDGE_SOURCE="$AUE_ROOT/scripts/g1_sonic_sim_udp_dds_bridge.cpp"
if [[ ! -f "$BRIDGE_SOURCE" ]]; then
    echo "[ERROR] Bridge source required by sim_bridge.py is missing: $BRIDGE_SOURCE" >&2
    exit 1
fi
if [[ -e "$OUTPUT" ]]; then
    echo "[ERROR] Output already exists; refusing to replace it: $OUTPUT" >&2
    exit 1
fi
command -v rsync >/dev/null || {
    echo "[ERROR] rsync is required" >&2
    exit 1
}

STAGING="${OUTPUT}.tmp.$$"
cleanup() {
    rm -rf "$STAGING"
}
trap cleanup EXIT
mkdir -p "$STAGING/aue-sim/src" "$STAGING/GR00T-WholeBodyControl/gear_sonic/data/robot_model/model_data"

rsync -aL --exclude='__pycache__/' "$AUE_ROOT/src/androidtwin" "$STAGING/aue-sim/src/"
if [[ -d "$AUE_ROOT/src/aue" ]]; then
    rsync -aL --exclude='__pycache__/' "$AUE_ROOT/src/aue" "$STAGING/aue-sim/src/"
fi
mkdir -p "$STAGING/aue-sim/scripts"
cp -aL "$BRIDGE_SOURCE" "$STAGING/aue-sim/scripts/"

GEAR_TARGET="$STAGING/GR00T-WholeBodyControl"
rsync -aL \
    "$GEAR_ROOT/gear_sonic/data/robot_model/model_data/g1" \
    "$GEAR_TARGET/gear_sonic/data/robot_model/model_data/"
mkdir -p "$GEAR_TARGET/gear_sonic_deploy/target/release"
for directory in policy planner reference thirdparty; do
    rsync -aL "$GEAR_ROOT/gear_sonic_deploy/$directory" \
        "$GEAR_TARGET/gear_sonic_deploy/"
done
cp -aL "$GEAR_ROOT/gear_sonic_deploy/target/release/g1_deploy_onnx_ref" \
    "$GEAR_TARGET/gear_sonic_deploy/target/release/"

rsync -aL "$INFERENCE_ROOT/" "$STAGING/inference/"
rsync -aL "$VISUAL_ROOT/" "$STAGING/g1-visual/"
mkdir -p "$STAGING/bridge"
cp -aL "$BRIDGE" "$STAGING/bridge/g1_sonic_sim_udp_dds_bridge_accepted"
chmod +x "$STAGING/bridge/g1_sonic_sim_udp_dds_bridge_accepted"

if [[ -n "$ROS_PREFIX" ]]; then
    rsync -aL "$ROS_PREFIX/" "$STAGING/ros2-humble-prefix/"
fi
if [[ -n "$NATIVE_DEPS" ]]; then
    rsync -aL "$NATIVE_DEPS/" "$STAGING/matrix-native-deps/"
fi
if [[ -n "$WHEELHOUSE" ]]; then
    rsync -aL "$WHEELHOUSE/" "$STAGING/python-wheelhouse/"
fi

python3 "$SCRIPT_DIR/verify_matrix_sonic_runtime.py" \
    --runtime-root "$STAGING" \
    --matrix-root "$PROJECT_ROOT" \
    --skip-dynamic \
    --skip-installed-assets

python3 - "$STAGING" <<'PY'
import datetime as dt
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
payload = {
    "schema_version": 1,
    "runtime_id": "matrix-sonic-v1",
    "created_at": dt.datetime.now(dt.timezone.utc).isoformat(),
    "note": "Verify contents against config/runtime/matrix-sonic.lock.json",
}
(root / "bundle.json").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
PY

mkdir -p "$(dirname "$OUTPUT")"
mv "$STAGING" "$OUTPUT"
trap - EXIT
echo "[PASS] Runtime artifact bundle created: $OUTPUT"
