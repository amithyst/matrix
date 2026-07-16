#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
export MATRIX_PROJECT_ROOT="$PROJECT_ROOT"

PROFILE="${MATRIX_PROFILE:-}"
ARTIFACT_SOURCE="${MATRIX_ARTIFACT_SOURCE:-}"
RELEASE_CACHE="${MATRIX_RELEASE_CACHE:-}"
SKIP_ASSETS=0
SKIP_PYTHON=0
VERIFY_ONLY=0

usage() {
    cat <<'EOF'
Usage: bash scripts/bootstrap_matrix_sonic.sh --profile heyuan|trna [options]

Options:
  --artifact-source PATH|HOST:PATH  Locked runtime bundle source
  --release-cache PATH              Existing Matrix 0.1.2 archives
  --skip-assets                     Do not install Matrix release/map packages
  --skip-python                     Do not create/update .venv-audit
  --verify-only                     Do not copy/install; run full verification
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --profile) PROFILE="$2"; shift 2 ;;
        --artifact-source) ARTIFACT_SOURCE="$2"; shift 2 ;;
        --release-cache) RELEASE_CACHE="$2"; shift 2 ;;
        --skip-assets) SKIP_ASSETS=1; shift ;;
        --skip-python) SKIP_PYTHON=1; shift ;;
        --verify-only) VERIFY_ONLY=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "[ERROR] Unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

if [[ "$PROFILE" != "heyuan" && "$PROFILE" != "trna" ]]; then
    echo "[ERROR] --profile must be heyuan or trna" >&2
    exit 2
fi

# shellcheck disable=SC1090
source "$PROJECT_ROOT/config/hosts/$PROFILE.env"
if [[ -f "$PROJECT_ROOT/.matrix/local.env" ]]; then
    # shellcheck disable=SC1091
    source "$PROJECT_ROOT/.matrix/local.env"
fi

RUNTIME_ROOT="${MATRIX_RUNTIME_ROOT:-$PROJECT_ROOT/outputs/runtime/matrix-sonic-v1}"
LOCK_FILE="$PROJECT_ROOT/config/runtime/matrix-sonic.lock.json"
mkdir -p "$PROJECT_ROOT/outputs/logs" "$PROJECT_ROOT/releases" "$(dirname "$RUNTIME_ROOT")"

if [[ "$VERIFY_ONLY" != "1" ]]; then
    if [[ -n "$ARTIFACT_SOURCE" ]]; then
        command -v rsync >/dev/null || {
            echo "[ERROR] rsync is required to copy the locked runtime bundle" >&2
            exit 1
        }
        mkdir -p "$RUNTIME_ROOT"
        echo "[INFO] Syncing locked runtime from $ARTIFACT_SOURCE"
        rsync -aL --delete "${ARTIFACT_SOURCE%/}/" "$RUNTIME_ROOT/"
    elif [[ ! -d "$RUNTIME_ROOT" ]]; then
        echo "[ERROR] Runtime is absent and --artifact-source was not provided" >&2
        exit 1
    fi

    if [[ "$SKIP_PYTHON" != "1" ]]; then
        python3 -m venv "$PROJECT_ROOT/.venv-audit"
        "$PROJECT_ROOT/.venv-audit/bin/python" -m pip install \
            -r "$PROJECT_ROOT/research/sonic_integration/requirements-trna.txt"
    fi

    if [[ -n "$RELEASE_CACHE" ]]; then
        python3 "$SCRIPT_DIR/verify_matrix_sonic_runtime.py" \
            --schema-only --lock "$LOCK_FILE"
        while IFS= read -r package; do
            source_path="${RELEASE_CACHE%/}/$package"
            if [[ ! -f "$source_path" ]]; then
                echo "[ERROR] Release cache is missing: $source_path" >&2
                exit 1
            fi
            ln -sfn "$source_path" "$PROJECT_ROOT/releases/$package"
        done < <(python3 - "$LOCK_FILE" <<'PY'
import json
import sys
for item in json.load(open(sys.argv[1], encoding="utf-8"))["matrix_release"]["packages"]:
    print(item["file"])
PY
        )
    fi

    if [[ "$SKIP_ASSETS" != "1" ]]; then
        MATRIX_MAPS=Town10World MATRIX_ASSUME_YES=1 \
            bash "$PROJECT_ROOT/scripts/release_manager/install_chunks.sh" 0.1.2
    fi

    deploy="$RUNTIME_ROOT/GR00T-WholeBodyControl/gear_sonic_deploy/target/release/g1_deploy_onnx_ref"
    bridge="$RUNTIME_ROOT/bridge/g1_sonic_sim_udp_dds_bridge_accepted"
    [[ -f "$deploy" ]] && chmod +x "$deploy"
    [[ -f "$bridge" ]] && chmod +x "$bridge"

    if [[ "$PROFILE" == "heyuan" ]]; then
        rmw_dir="$RUNTIME_ROOT/ros2-humble-prefix/lib"
        ue_rmw="$PROJECT_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux/librmw_fastrtps_cpp.so"
        mkdir -p "$rmw_dir"
        if [[ -f "$ue_rmw" ]]; then
            ln -sfn "$ue_rmw" "$rmw_dir/librmw_fastrtps_cpp.so"
        fi
    fi
fi

VERIFY_ARGS=(
    --lock "$LOCK_FILE"
    --runtime-root "$RUNTIME_ROOT"
    --matrix-root "$PROJECT_ROOT"
    --profile "$PROFILE"
    --json-output "$PROJECT_ROOT/outputs/runtime-verification-$PROFILE.json"
)
if [[ -n "$RELEASE_CACHE" ]]; then
    VERIFY_ARGS+=(--release-cache "$RELEASE_CACHE")
fi
if [[ "$SKIP_ASSETS" == "1" ]]; then
    VERIFY_ARGS+=(--skip-installed-assets)
fi

python3 "$SCRIPT_DIR/verify_matrix_sonic_runtime.py" "${VERIFY_ARGS[@]}"
echo "[PASS] Matrix SONIC bootstrap complete: profile=$PROFILE runtime=$RUNTIME_ROOT"
